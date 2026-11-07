#!/usr/bin/env python3
"""SIM ONLY (it arms): depth-driven corridor follower. Directions, not coordinates.

The route is a list of moves: (turn, end condition). The drone never needs to know where it
is on the map. It flies forward in its own body frame with VELOCITY setpoints, so a jump in
the position estimate cannot make it chase a phantom point. Depth does the rest:
  - front: stop at a known distance from a solid wall ('wall'), or after a short odometry
    distance ('odo', integrated velocity) where the end is the net (depth cannot see the net)
  - sides: centre between two walls, or keep SIDE_M from a single wall
  - safety: never move forward when something is closer than FRONT_SAFE ahead
Rules: never reverse along the path just flown (no-backtrack); turns are +/-90 deg only,
slow, and heading-estimate jumps are absorbed so the airframe does not spin.
"""
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from stage23 import Stage, log

HEIGHT = 1.0
WINDOW_H = 0.76      # window centre height (sill 0.305, top 1.219)
SPEED = 0.25          # forward m/s
YAW_RATE = math.radians(12.0)
YAW_JUMP = math.radians(15.0)
FRONT_SAFE = 0.42     # never fly forward when the front reads closer than this
SIDE_M = 0.45         # keep at least this from a single side wall
K_LAT = 0.8
V_LAT_MAX = 0.15
NAN = float('nan')
LAST = int(os.environ.get('LAST_MOVE', '99'))  # stop after this move number (testing)

# (turn_deg, mode, value, note). turn_deg: relative turn, + = right. mode: 'wall' = stop at front depth value,
# 'odo' = fly value metres by integrated velocity. Course x east / y north, start (-2.51, 0.91) N.
MOVES = [
    (0, 'odo', 1.67, 'N up start corridor to y=2.58 (net ahead: invisible to depth)'),
    (90, 'wall', 0.46, 'E over M1 tip, stop 0.46 before M2 (x=-1.32)'),
    (90, 'wall', 0.52, 'S down M1-M2 lane, stop 0.52 before Barrier 2 (y=1.03)'),
    (-90, 'wall', 0.58, 'E under M2, stop 0.58 before M3 (x=-0.33)'),
    (-90, 'odo', 1.58, 'N up M2-M3 lane to y=2.58'),
    (90, 'odo', 1.17, 'E over M3 to x=0.95'),
    (30, 'odo', 1.57, 'ESE to the tube entrance (2.31, 1.80)'),
    (60, 'odo', 2.55, 'S through the tube to y=-0.75'),
    (41, 'odo', 1.39, 'SW to (1.40, -1.80), clear of C1 tip'),
    (12, 'odo', 1.29, 'SW to the window approach (0.37, -2.58), 0.9 m from C2a'),
    (57, 'window', 1.35, 'WNW through the C2a window (heading -70, perpendicular) at 0.76 m'),
    (29, 'odo', 1.75, 'NW to the landing pad (-2.04, -0.79) at 1.0 m'),
]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Corridor(Stage):
    def __init__(self):
        super().__init__()
        self.fx, self.cx_cam = 421.0, 319.5
        self.f_hist = deque(maxlen=3)
        self.sectors = (None, None, None)
        self.vel = None
        self.win = None
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_info, qos_profile_sensor_data)

    def on_info(self, m):
        if m.k[0] > 100.0:
            self.fx, self.cx_cam = float(m.k[0]), float(m.k[2])

    def on_pos(self, m):
        super().on_pos(m)
        self.vel = (m.vx, m.vy) if (m.v_xy_valid and math.isfinite(m.vx)) else None

    def on_depth(self, m):
        super().on_depth(m)
        a = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.width)
        h, w = a.shape
        band = a[int(h * 0.40):int(h * 0.60), ::16].astype(np.float32) / 1000.0
        band = np.where((band < 0.25) | (band > 4.0), 99.0, band)
        rng = np.percentile(band, 25, axis=0)
        front_f, rays = [], []
        for u, r in zip(np.arange(0, w, 16), rng):
            b = math.atan((float(u) - self.cx_cam) / self.fx)
            rv = None if r >= 50.0 else float(r)
            rays.append((b, rv))
            if rv is not None and abs(b) <= math.radians(11.0):
                front_f.append(rv * math.cos(b))
        front_f.sort()
        f_d = front_f[int(0.25 * len(front_f))] if len(front_f) >= 2 else None
        if f_d is not None:
            self.f_hist.append(f_d)
        f_med = sorted(self.f_hist)[len(self.f_hist) // 2] if self.f_hist else None
        f_ref = f_med if f_med is not None else 99.0
        left_p, right_p = [], []
        for b, rv in rays:
            if rv is None:
                continue
            fwd, lat = rv * math.cos(b), rv * math.sin(b)
            if 0.15 <= fwd <= 1.0 and fwd < f_ref - 0.20:
                if b <= math.radians(-20.0):
                    left_p.append(-lat)
                elif b >= math.radians(20.0):
                    right_p.append(lat)
        left_p.sort()
        right_p.sort()
        l_d = left_p[int(0.25 * len(left_p))] if len(left_p) >= 2 else None
        r_d = right_p[int(0.25 * len(right_p))] if len(right_p) >= 2 else None
        self.sectors = (f_med, l_d, r_d)
        self.win = None
        near = [rv * math.cos(b) for b, rv in rays if rv is not None and rv < 1.6]
        if len(near) >= 4:
            wall = sorted(near)[len(near) // 2]
            far = [rv is None or rv * math.cos(b) > wall + 0.6 for b, rv in rays]
            best, run0 = None, None
            for j, fv in enumerate(far + [False]):
                if fv and run0 is None:
                    run0 = j
                elif not fv and run0 is not None:
                    if run0 > 0 and j < len(rays) and (best is None or j - run0 > best[1] - best[0]):
                        best = (run0, j)
                    run0 = None
            if best is not None and best[1] - best[0] >= 3:
                b_c = 0.5 * (rays[best[0]][0] + rays[best[1] - 1][0])
                self.win = (wall * math.tan(b_c), wall, best[1] - best[0])

    def send_vel(self, vn, ve, z, yaw):
        t = self.stamp()
        o = OffboardControlMode()
        o.timestamp = t
        o.position = True
        o.velocity = True
        self.ocm.publish(o)
        s = TrajectorySetpoint()
        s.timestamp = t
        s.position = [NAN, NAN, float(z)]
        s.velocity = [float(vn), float(ve), NAN]
        s.acceleration = [NAN, NAN, NAN]
        s.yaw = float(yaw)
        self.sp.publish(s)


def fmt(v):
    return '--' if v is None else '%.2f' % v


def main():
    rclpy.init()
    n = Corridor()
    t_start = time.time()

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    while (n.pos is None or n.status is None) and time.time() - t_start < 60:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pos is None or n.status is None:
        log('CR FAIL: no valid PX4 position/status')
        return
    x0, y0, z0, yaw0 = n.pos
    z_sp = z0
    yaw_sp = [yaw0]
    yaw_off = [0.0]      # accumulated heading-estimate jumps
    last_yaw = [yaw0]
    turn_tgt = [None]    # active turn target, shifted with absorbed jumps
    log('CR start NED=(%.2f, %.2f, %.2f) heading=%.2f; %d moves' % (x0, y0, z0, yaw0, len(MOVES)))

    def tick(vf=0.0, vl=0.0, pos_hold=None):
        """vf forward, vl right, in the body frame of the CURRENT heading estimate."""
        p = n.pos
        if p is not None:
            dy = wrap(p[3] - last_yaw[0])
            if abs(dy) > YAW_JUMP:
                yaw_sp[0] = wrap(yaw_sp[0] + dy)
                yaw_off[0] += dy
                if turn_tgt[0] is not None:
                    turn_tgt[0] = wrap(turn_tgt[0] + dy)
                log('CR yaw jump %+.0f deg -> absorbed (total %+.0f)' % (math.degrees(dy), math.degrees(yaw_off[0])))
            last_yaw[0] = p[3]
            hd = p[3]
        else:
            hd = yaw_sp[0]
        if pos_hold is not None:
            n.send(pos_hold, yaw_sp[0])
        else:
            vn = vf * math.cos(hd) - vl * math.sin(hd)
            ve = vf * math.sin(hd) + vl * math.cos(hd)
            n.send_vel(vn, ve, z_sp, yaw_sp[0])
        spin(0.05)

    def lateral(l_d, r_d):
        if l_d is not None and r_d is not None and l_d < 0.8 and r_d < 0.8:
            e = 0.5 * (r_d - l_d)            # >0: more room on the right -> move right
        elif l_d is not None and l_d < SIDE_M:
            e = SIDE_M - l_d
        elif r_d is not None and r_d < SIDE_M:
            e = -(SIDE_M - r_d)
        else:
            return 0.0
        if abs(e) < 0.01:
            return 0.0
        return max(-V_LAT_MAX, min(V_LAT_MAX, K_LAT * e))

    result = 'route done'
    lost_since = None
    try:
        for _ in range(30):
            tick(pos_hold=[x0, y0, z0])
        t0 = time.time()
        while n.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and time.time() - t0 < 10:
            n.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            tick(pos_hold=[x0, y0, z0])
        t0 = time.time()
        while not n.armed() and time.time() - t0 < 20:
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            tick(pos_hold=[x0, y0, z0])
        if not n.armed():
            raise RuntimeError('PX4 did not arm')
        # climb in position mode at the start point
        cz = z0
        t0 = time.time()
        while time.time() - t0 < 20:
            cz = max(cz - 0.5 * 0.05, z0 - HEIGHT)
            tick(pos_hold=[x0, y0, cz])
            if n.pos and abs(n.pos[2] - (z0 - HEIGHT)) < 0.12:
                break
        z_sp = z0 - HEIGHT
        for _ in range(20):
            tick()
        t_fly0 = time.time()
        for k, (turn, mode, val, note) in enumerate(MOVES):
            if k + 1 > LAST:
                log('CR stopping after move %d (LAST_MOVE)' % LAST)
                break
            log('CR move %d: turn=%+d %s %.2f | %s' % (k + 1, turn, mode, val, note))
            # 1) turn in place, slowly, holding still (velocity 0, side walls still respected)
            if turn:
                turn_tgt[0] = wrap(yaw_sp[0] + math.radians(turn))
                t1 = time.time()
                chunk = 0.0
                while abs(wrap(turn_tgt[0] - yaw_sp[0])) > 1e-3 and time.time() - t1 < 25:
                    step_y = max(-YAW_RATE * 0.05, min(YAW_RATE * 0.05, wrap(turn_tgt[0] - yaw_sp[0])))
                    yaw_sp[0] = wrap(yaw_sp[0] + step_y)
                    chunk += abs(step_y)
                    tick()
                    if chunk >= math.radians(30.0):
                        chunk = 0.0
                        for _ in range(20):
                            tick()
                turn_tgt[0] = None
                t1 = time.time()
                while n.pos is not None and abs(wrap(n.pos[3] - yaw_sp[0])) > math.radians(4) and time.time() - t1 < 4:
                    tick()
                for _ in range(20):   # 1 s settle before translating
                    tick()
            z_t = z0 - (WINDOW_H if mode == 'window' else HEIGHT)
            if abs(z_sp - z_t) > 0.02:
                log('CR move %d: height -> %.2f m' % (k + 1, z0 - z_t))
                t1 = time.time()
                while time.time() - t1 < 10:
                    z_sp += max(-0.0125, min(0.0125, z_t - z_sp))
                    tick()
                    if abs(z_sp - z_t) < 1e-3 and n.pos is not None and abs(n.pos[2] - z_t) < 0.06:
                        break
            n.f_hist.clear()
            # 2) fly forward until the end condition
            travelled, last_log, t_m0 = 0.0, 0.0, time.time()
            settle = 0
            while True:
                if not n.armed():
                    raise RuntimeError('disarmed in flight')
                p = n.pos
                if p is None:
                    lost_since = lost_since or time.time()
                    if time.time() - lost_since > 5.0:
                        raise RuntimeError('position lost > 5s')
                    tick()
                    continue
                lost_since = None
                f_d, l_d, r_d = n.sectors
                if n.vel is not None:
                    v_al = n.vel[0] * math.cos(p[3]) + n.vel[1] * math.sin(p[3])
                    travelled += max(-0.6, min(0.6, v_al)) * 0.05
                if mode == 'wall':
                    rem = (f_d - val) if f_d is not None else 1.0
                    if travelled > 6.0:
                        log('CR move %d: wall not found after %.1f m -> next' % (k + 1, travelled))
                        break
                else:
                    rem = val - travelled
                if mode == 'window' and f_d is not None and f_d < FRONT_SAFE and time.time() - t_m0 > 20:
                    raise RuntimeError('window blocked by depth for 20 s')
                vf = max(0.0, min(SPEED, 0.7 * rem + 0.04)) if rem > 0.02 else 0.0
                if f_d is not None and f_d < FRONT_SAFE and not (mode == 'wall' and val < FRONT_SAFE):
                    vf = 0.0
                if mode == 'window':
                    w = n.win
                    vl = max(-V_LAT_MAX, min(V_LAT_MAX, K_LAT * w[0])) if (w is not None and rem > 0.55) else 0.0
                else:
                    vl = lateral(l_d, r_d)
                if rem <= 0.02:
                    settle += 1
                    if settle >= 10:
                        break
                else:
                    settle = 0
                if time.time() - t_m0 > 60:
                    log('CR move %d: timeout' % (k + 1))
                    break
                if time.time() - last_log >= 2.0:
                    last_log = time.time()
                    log('CR t=%5.1f m%d trav=%.2f F=%s L=%s R=%s vf=%.2f vl=%+.2f yaw=%.0f' % (
                        time.time() - t_fly0, k + 1, travelled, fmt(f_d), fmt(l_d), fmt(r_d), vf, vl,
                        math.degrees(p[3])))
                tick(vf, vl)
            log('CR move %d done: trav=%.2f F=%s L=%s R=%s t=%.1f' % (
                k + 1, travelled, fmt(n.sectors[0]), fmt(n.sectors[1]), fmt(n.sectors[2]), time.time() - t_fly0))
            for _ in range(10):
                tick()
        else:
            result = 'reached landing pad'
    except Exception as exc:
        result = 'error: %s' % exc
        log('CR ' + result)
    finally:
        log('CR landing (%s)' % result)
        t0 = time.time()
        while n.armed() and time.time() - t0 < 40:
            n.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            spin(0.5)
        if n.armed():
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        log('CR RESULT: %s | landed=%s | elapsed %.0f s' % (result, not n.armed(), time.time() - t_start))
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
