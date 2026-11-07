#!/usr/bin/env python3
"""SIM ONLY (it arms): preplanned waypoint run through the maze lanes + depth obstacle check, 60 s, land.

Waypoints are lane centres from the published layout (course frame x east, y north, launch mark
(-2.51, 0.91)). For each leg: turn to face the leg (camera forward), then fly the straight line.
Depth check: ONLY the middle of the depth image (stage23: centre 30% x 30%, 10th percentile).
If it reads closer than STOP_DEPTH the drone stops on the line; blocked for 2 s -> next leg.
"""
import math
import time

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus

from stage23 import Stage, log

HEIGHT = 1.0
RUN_S = 60.0
SPEED = 0.25
YAW_RATE = math.radians(25.0)
STOP_DEPTH = 0.45                 # centre depth closer than this ahead: stop
LAUNCH = (-2.51, 0.91)
WAYPOINTS = [                     # course (x east, y north), lane centres
    (-2.51, 2.55),                # north up the start corridor (net 0.50 m beyond)
    (-1.37, 2.55),                # east over the end of M1
    (-1.37, 0.98),                # south down the M1-M2 lane
    (-0.25, 0.98),                # east under the end of M2
    (-0.25, 2.55),                # north up the M2-M3 lane
    (0.80, 2.55),                 # east over the end of M3
]

def to_ned(cx, cy):
    return (cy - LAUNCH[1], cx - LAUNCH[0])

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def main():
    rclpy.init()
    n = Stage()
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
    log('WP start NED=(%.2f, %.2f, %.2f) heading=%.2f; %d waypoints, stop depth %.2f m' % (
        x0, y0, z0, yaw0, len(WAYPOINTS), STOP_DEPTH))
    def tick():
        n.send(cur, yaw_sp)
        spin(0.05)
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
        for _ in range(20):
            tick()
        t_fly0 = time.time()
        last_log = 0.0
        lost_since = None
        for i, (cx, cy) in enumerate(WAYPOINTS):
            if time.time() - t_fly0 > RUN_S:
                result = 'timer'
                break
            tn, te = to_ned(cx, cy)
            tn, te = x0 + tn, y0 + te
            sn, se = cur[0], cur[1]                     # leg starts at the current setpoint
            L = math.hypot(tn - sn, te - se)
            heading = math.atan2(te - se, tn - sn)
            log('WP leg %d -> course (%.2f, %.2f), %.2f m, heading %.0f deg' % (i + 1, cx, cy, L, math.degrees(heading)))
            # 1) turn in place to face the leg (camera forward)
            while abs(wrap(heading - yaw_sp)) > 1e-3 and time.time() - t_fly0 < RUN_S:
                yaw_sp = wrap(yaw_sp + max(-YAW_RATE * 0.05, min(YAW_RATE * 0.05, wrap(heading - yaw_sp))))
                tick()
            t1 = time.time()
            while n.pos and abs(wrap(n.pos[3] - heading)) > math.radians(5) and time.time() - t1 < 3:
                tick()
            # 2) fly the line; depth check on the middle of the image only
            s, blocked_since, done = 0.0, None, False
            while time.time() - t_fly0 < RUN_S:
                p = n.pos
                if not n.armed():
                    raise RuntimeError('disarmed in flight')
                if p is None:
                    lost_since = lost_since or time.time()
                    if time.time() - lost_since > 1.0:
                        raise RuntimeError('position lost')
                    tick()
                    continue
                lost_since = None
                along = ((p[0] - sn) * math.cos(heading) + (p[1] - se) * math.sin(heading))
                clear, frac = n.depth
                blocked = clear is not None and clear < STOP_DEPTH
                if blocked:
                    s = min(s, max(0.0, along))           # stop: setpoint back to where the drone is
                    blocked_since = blocked_since or time.time()
                    if time.time() - blocked_since > 2.0:
                        log('WP leg %d: depth blocked (centre %.2f m) at %.2f of %.2f m -> next leg' % (i + 1, clear, along, L))
                        break
                else:
                    blocked_since = None
                    if s < L and s - along < 0.3:
                        s = min(L, s + SPEED * 0.05)
                cur[0], cur[1] = sn + math.cos(heading) * s, se + math.sin(heading) * s
                if s >= L and math.hypot(p[0] - tn, p[1] - te) < 0.10:
                    done = True
                    break
                if time.time() - last_log >= 2.0:
                    last_log = time.time()
                    log('WP t=%4.1f leg %d course=(%.2f, %.2f) yaw=%4.0f along %.2f/%.2f centre depth %s (valid %.0f%%)%s' % (
                        time.time() - t_fly0, i + 1, p[1] - y0 + LAUNCH[0], p[0] - x0 + LAUNCH[1], math.degrees(p[3]),
                        along, L, 'n/a' if clear is None else '%.2f' % clear, 100 * frac, ' BLOCKED' if blocked else ''))
                tick()
            if done:
                p = n.pos
                log('WP leg %d reached: course=(%.2f, %.2f) t=%.1f' % (
                    i + 1, p[1] - y0 + LAUNCH[0], p[0] - x0 + LAUNCH[1], time.time() - t_fly0))
        else:
            result = 'route done'
        if time.time() - t_fly0 > RUN_S:
            result = 'timer'
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
        log('WP RESULT: %s | landed=%s | final course=%s' % (
            result, not n.armed(), 'n/a' if p is None else '(%.2f, %.2f)' % (p[1] - y0 + LAUNCH[0], p[0] - x0 + LAUNCH[1])))
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
