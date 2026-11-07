#!/usr/bin/env python3
"""SIM ONLY (it arms): depth-aided waypoint localizer through the maze/chicane to the landing pad.

Staggered corner geometry + exact `run_004` flight dynamics:
  1. Staggered lane endpoints matching the physical M1/M2/M3 wall offsets:
     - Leg 2 ends at (-1.32, 2.58, 0.46) -> extra East clearance past M1 north tip (-1.88, 2.12).
     - Leg 3 ends at (-1.38, 0.97, 0.46) -> extra West clearance past M2 south tip (-0.86, 1.44),
       matching the `x <= -1.34` region where `first run_003`, `run_004`, and `run_015` all passed Corner 3.
     - Leg 4 ends at (-0.34, 0.97, 0.58) -> stops 0.58 m from M3 (keeping M3/Barrier 2 edges in the
       74-deg stereo FOV so cuVSLAM never loses tracking on a close blank wall at Corner 4).
     - Leg 5 ends at (-0.22, 2.58, None) -> extra East clearance past M3 north tip (+0.25, 2.12).
  2. Global offset `off` is updated ONLY by orthogonal solid-wall front depth anchors (`dcorr`) and
     VIO jump absorption (never drifted by side-wall `lat_corr`).
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from stage23 import Stage, log

HEIGHT = 1.0
SPEED = 0.28
YAW_RATE = math.radians(18.0)
STOP_DEPTH = 0.45
LAUNCH = (-2.51, 0.91)
PAD = (-2.04, -0.79)

# (cx, cy, solid_wall_target_depth_m or None)
WAYPOINTS = [
    (-2.51, 2.58, None),   # Leg 1: North up start corridor (midpoint between M1=2.12 and Net=3.05)
    (-1.32, 2.58, 0.46),   # Leg 2: East over M1 -> solid wall anchor on M2 (x=-0.86, clears M1 top tip)
    (-1.38, 0.97, 0.46),   # Leg 3: SSW down M1-M2 -> solid wall anchor on Barrier 2 (y=+0.51, clears M2 bottom tip)
    (-0.34, 0.97, 0.58),   # Leg 4: East under M2 -> solid wall anchor on M3 at 0.58 m (keeps M3 edges in FOV)
    (-0.22, 2.58, None),   # Leg 5: NNE up M2-M3 lane (clears M3 top tip)
    ( 0.95, 2.58, None),   # Leg 6: East over M3
    ( 2.31, 1.80, None),   # Leg 7: Align with Tube north entrance (x=2.31)
    ( 2.31,-0.75, None),   # Leg 8: South through Tube (center x=2.31)
    ( 1.70,-1.80, None),   # Leg 9: SW clear of C1 south tip
    ( 0.38,-1.58, 0.54),   # Leg 10: WNW under C1 -> solid wall anchor on C2a/C2b
    ( 0.78,-0.08, 0.50),   # Leg 11: NNE up C1-C2b centerline -> solid wall anchor on Barrier 3 (y=+0.41)
    (-0.36,-0.08, 0.56),   # Leg 12: West through C2b-Barrier2 gap -> solid wall anchor on A1 (x=-0.92)
    (-0.46,-0.78, None),   # Leg 13: SSW down A1-C2b centerline past A1 south tip
    (-0.80,-1.20, None),   # Leg 14: SW around A1 south tip
    (-2.04,-0.79, None),   # Leg 15: WNW onto Landing Pad center (-2.04, -0.79)
]

class LocalizerNode(Stage):
    def __init__(self):
        super().__init__()
        self.fx = 421.0
        self.cx_cam = 319.5
        self.f_hist = deque(maxlen=3)
        self.sectors = (None, None, None)  # (front_d, left_perp, right_perp)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_info, qos_profile_sensor_data)

    def on_info(self, m):
        if m.k[0] > 100.0:
            self.fx = float(m.k[0])
            self.cx_cam = float(m.k[2])

    def on_depth(self, m):
        super().on_depth(m)
        a = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.width)
        h, w = a.shape
        band = a[int(h * 0.40):int(h * 0.60), ::16].astype(np.float32) / 1000.0
        band = np.where((band < 0.35) | (band > 4.0), 99.0, band)
        rng = np.percentile(band, 25, axis=0)
        us = np.arange(0, w, 16)
        front_f = []
        rays = []
        for u, r in zip(us, rng):
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
            fwd = rv * math.cos(b)
            lat = rv * math.sin(b)
            if 0.22 <= fwd <= 0.85 and fwd < f_ref - 0.22:
                if b <= math.radians(-22.0):
                    left_p.append(-lat)
                elif b >= math.radians(22.0):
                    right_p.append(lat)
        left_p.sort()
        right_p.sort()
        l_d = left_p[int(0.25 * len(left_p))] if len(left_p) >= 2 else None
        r_d = right_p[int(0.25 * len(right_p))] if len(right_p) >= 2 else None
        self.sectors = (f_med, l_d, r_d)

def to_ned(cx, cy):
    return (cy - LAUNCH[1], cx - LAUNCH[0])

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def main():
    rclpy.init()
    n = LocalizerNode()
    t_start = time.time()
    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)
    while (n.pos is None or n.status is None) and time.time() - t_start < 60:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pos is None or n.status is None:
        log('WP FAIL: no valid PX4 position/status')
        return
    x0, y0, z0, yaw0 = n.pos
    cur, yaw_sp = [x0, y0, z0], yaw0
    log('WP start NED=(%.2f, %.2f, %.2f) heading=%.2f; %d waypoints (no time limit)' % (
        x0, y0, z0, yaw0, len(WAYPOINTS)))
    off = [0.0, 0.0]
    last = [None]
    turning = [False]
    was_lost = [False]
    def tick():
        n.send(cur, yaw_sp)
        spin(0.05)
        p = n.pos
        if p is None:
            was_lost[0] = True
            return
        if last[0] is not None:
            dn, de = p[0] - last[0][0], p[1] - last[0][1]
            jump_m = 0.035 if (turning[0] or was_lost[0]) else 0.16
            if math.hypot(dn, de) > jump_m:
                cur[0] += dn
                cur[1] += de
                off[0] += dn
                off[1] += de
                log('WP jump %.2f m (%s: n %+.2f e %+.2f) -> absorbed, total off=(%+.2f, %+.2f)' % (
                    math.hypot(dn, de), 'TURN/RECOV' if (turning[0] or was_lost[0]) else 'FLY', dn, de, off[0], off[1]))
        was_lost[0] = False
        last[0] = p

    result = 'route done'
    try:
        for _ in range(30):
            tick()
        t0 = time.time()
        while n.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and time.time() - t0 < 10:
            n.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            tick()
        t0 = time.time()
        while not n.armed() and time.time() - t0 < 20:
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            tick()
        if not n.armed():
            raise RuntimeError('PX4 did not arm')
        t0 = time.time()
        while time.time() - t0 < 20:
            cur[2] = max(cur[2] - 0.5 * 0.05, z0 - HEIGHT)
            tick()
            if n.pos and abs(n.pos[2] - (z0 - HEIGHT)) < 0.15:
                break
        for _ in range(25):
            tick()
        t_fly0 = time.time()
        last_log = 0.0
        lost_since = None
        for i, (cx, cy, anchor_d) in enumerate(WAYPOINTS):
            tn, te = to_ned(cx, cy)
            tn, te = x0 + tn + off[0], y0 + te + off[1]
            sn, se = cur[0], cur[1]
            o0 = list(off)
            L = math.hypot(tn - sn, te - se)
            heading = math.atan2(te - se, tn - sn)
            log('WP leg %d -> course (%.2f, %.2f), %.2f m, heading %.0f deg, anchor=%s' % (
                i + 1, cx, cy, L, math.degrees(heading), 'none' if anchor_d is None else '%.2fm' % anchor_d))
            # 1) Turn in place (18 deg/s) with strict turn-jump rejection + settle
            turning[0] = True
            t_turn0 = time.time()
            while abs(wrap(heading - yaw_sp)) > 1e-3 and time.time() - t_turn0 < 10.0:
                yaw_sp = wrap(yaw_sp + max(-YAW_RATE * 0.05, min(YAW_RATE * 0.05, wrap(heading - yaw_sp))))
                tick()
            t1 = time.time()
            while (n.pos is None or abs(wrap(n.pos[3] - heading)) > math.radians(4.5)) and time.time() - t1 < 4.0:
                tick()
            for _ in range(6):
                tick()
            turning[0] = False
            n.f_hist.clear()
            # Re-anchor leg on post-turn setpoint
            tn, te = tn + off[0] - o0[0], te + off[1] - o0[1]
            o0 = list(off)
            sn, se = cur[0], cur[1]
            L_nom = math.hypot(tn - sn, te - se)
            L = L_nom
            heading = math.atan2(te - se, tn - sn)
            ch, sh = math.cos(heading), math.sin(heading)
            # 2) Fly the line with full deceleration ramp, gated solid-wall depth anchor, and lag-free side centering
            s, blocked_since, done, end_wait_t = 0.0, None, False, None
            along_corr, lat_corr, settle_ticks = 0.0, 0.0, 0
            t_leg0 = time.time()
            while time.time() - t_leg0 < 45.0:
                p = n.pos
                if not n.armed():
                    raise RuntimeError('disarmed in flight')
                if p is None:
                    lost_since = lost_since or time.time()
                    if time.time() - lost_since > 5.0:
                        raise RuntimeError('position lost > 5s')
                    tick()
                    continue
                lost_since = None
                if off != o0:
                    dn_j, de_j = off[0] - o0[0], off[1] - o0[1]
                    sn, se = sn + dn_j, se + de_j
                    tn, te = tn + dn_j, te + de_j
                    o0 = list(off)
                along = (p[0] - sn) * ch + (p[1] - se) * sh
                lat_pos = -(p[0] - sn) * sh + (p[1] - se) * ch
                f_d, l_d, r_d = n.sectors
                hdg_ok = abs(wrap(p[3] - heading)) < math.radians(6.5)
                # (A) Gated solid-wall front depth calibration (exact run_004 logic)
                if anchor_d is not None and hdg_ok and along > max(0.35, L_nom - 0.50) and f_d is not None and 0.36 <= f_d <= 0.92:
                    err_along = (f_d - anchor_d) - (L - along)
                    if 0.012 < abs(err_along) < 0.25:
                        step_c = max(-0.025, min(0.025, 0.30 * err_along))
                        if abs(along_corr + step_c) <= 0.28:
                            along_corr += step_c
                            dn_c, de_c = ch * step_c, sh * step_c
                            tn += dn_c; te += de_c
                            off[0] += dn_c; off[1] += de_c
                            o0[0] += dn_c; o0[1] += de_c
                            L += step_c
                # (B) Lag-free side-wall protection referenced to actual drone position `lat_pos`
                if hdg_ok and 0.05 < along < L - 0.25 and i not in (0, 12, 13):
                    delta_lat = None
                    if l_d is not None and r_d is not None and l_d < 0.75 and r_d < 0.75:
                        delta_lat = 0.5 * (r_d - l_d)
                    elif l_d is not None and l_d < 0.43:
                        delta_lat = 0.43 - l_d
                    elif r_d is not None and r_d < 0.43:
                        delta_lat = -(0.43 - r_d)
                    if delta_lat is not None and abs(delta_lat) > 0.015:
                        des_lat = max(-0.14, min(0.14, lat_pos + delta_lat))
                        step_lat = max(-0.003, min(0.003, 0.12 * (des_lat - lat_corr)))
                        lat_corr += step_lat
                # (C) Protective brake + full deceleration ramp to 0.07 m/s
                stop_lim = 0.38 if anchor_d is not None else STOP_DEPTH
                blocked = (along > L - 0.35) and (f_d is not None and f_d < stop_lim)
                rem = max(0.0, L - s)
                v_des = max(0.07, min(SPEED, 0.55 * rem + 0.05)) if rem < 0.45 else SPEED
                if blocked:
                    s = min(s, max(0.0, along))
                    blocked_since = blocked_since or time.time()
                    if time.time() - blocked_since > 1.2:
                        log('WP leg %d: depth stop (front %.2f m) at %.2f/%.2f m -> next leg' % (i + 1, f_d, along, L))
                        done = True
                        break
                else:
                    blocked_since = None
                    if s < L and s - along < 0.16:
                        s = min(L, s + v_des * 0.05)
                cur[0] = sn + ch * s - sh * lat_corr
                cur[1] = se + sh * s + ch * lat_corr
                dist_wp = math.hypot(p[0] - cur[0], p[1] - cur[1])
                if s >= L:
                    end_wait_t = end_wait_t or time.time()
                anchor_ok = (anchor_d is None) or (f_d is None) or (abs(f_d - anchor_d) < 0.055) or (end_wait_t is not None and time.time() - end_wait_t > 2.0)
                if s >= L and dist_wp < 0.09 and anchor_ok:
                    settle_ticks += 1
                    if settle_ticks >= 5:
                        done = True
                        break
                else:
                    settle_ticks = 0
                if time.time() - last_log >= 2.0:
                    last_log = time.time()
                    est_cx = p[1] - y0 - off[1] + LAUNCH[0]
                    est_cy = p[0] - x0 - off[0] + LAUNCH[1]
                    log('WP t=%4.1f leg %d c=(%.2f,%.2f) yaw=%4.0f s=%.2f/%.2f F=%s L=%s R=%s dcorr=%+.2f lcorr=%+.2f' % (
                        time.time() - t_fly0, i + 1, est_cx, est_cy, math.degrees(p[3]),
                        along, L,
                        '--' if f_d is None else '%.2f' % f_d,
                        '--' if l_d is None else '%.2f' % l_d,
                        '--' if r_d is None else '%.2f' % r_d,
                        along_corr, lat_corr))
                tick()
            if done:
                p = n.pos
                if p is not None:
                    est_cx = p[1] - y0 - off[1] + LAUNCH[0]
                    est_cy = p[0] - x0 - off[0] + LAUNCH[1]
                    log('WP leg %d reached: course=(%.2f, %.2f) F=%s dcorr=%+.2f lcorr=%+.2f t=%.1f' % (
                        i + 1, est_cx, est_cy,
                        '--' if n.sectors[0] is None else '%.2f' % n.sectors[0],
                        along_corr, lat_corr, time.time() - t_fly0))
        else:
            result = 'reached landing pad'
    except Exception as exc:
        result = 'error: %s' % exc
        log('WP ' + result)
    finally:
        log('WP landing (%s)' % result)
        t0 = time.time()
        while n.armed() and time.time() - t0 < 40:
            n.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            spin(0.5)
        if n.armed():
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        p = n.pos
        log('WP RESULT: %s | landed=%s | final course=%s | total off=(%+.2f, %+.2f)' % (
            result, not n.armed(),
            'n/a' if p is None else '(%.2f, %.2f)' % (p[1] - y0 - off[1] + LAUNCH[0], p[0] - x0 - off[0] + LAUNCH[1]),
            off[0], off[1]))
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
