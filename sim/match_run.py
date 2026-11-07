#!/usr/bin/env python3
"""SIM ONLY (it arms): run_003 waypoint flight + depth-vs-map scan matcher + window route.

Base: waypoint_run_best_run003.py (furthest run, Legs 1-10 + tube). Added:
  - scan matcher (scan_match.py): depth columns matched to the course walls give an absolute
    fix of position (only along directions the walls pin down) and heading. It corrects our own
    map->estimate transform (offset `off` + rotation `theta`); the PX4 bridge is NOT changed.
  - south route through the C2a window at 0.76 m (bonus), aligned on the gap seen by depth.
  - depth: band rows only, zero-copy decode, newest frame only; z-depth geometry (fwd = z,
    lateral = z * tan(bearing)).
  - depth brake anywhere on a leg; window leg aborts to a landing instead of hitting C2a.
"""
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from scan_match import course_matcher
from stage23 import Stage, log

HEIGHT = 1.0
SPEED = 0.28
YAW_RATE = math.radians(12.0)
YAW_JUMP = math.radians(3.0)    # > 60 deg/s in one 50 ms tick: impossible for us (we command 12 deg/s) -> estimator noise
POS_JUMP = 0.05                 # > 1 m/s in one tick: impossible (we fly 0.28 m/s) -> estimator noise
YAW_DRIFT = math.radians(15.0)  # estimate held this far from our setpoint for 0.5 s (not turning) -> estimator drift   # heading change in one 50 ms tick beyond what we command = estimator jump
STOP_DEPTH = 0.45
WINDOW_H = 0.76    # window centre height (sill 0.305, top 1.219)
MATCH = os.environ.get('MATCH', '1') == '1'
MATCH_GAIN = 0.4
MATCH_STEP = 0.06      # max position correction per fix (m)
MATCH_HSTEP = math.radians(1.5)
SIDE_M = 0.48     # keep at least this from a single visible side wall (reach 0.28 + 0.20)
LAT_CAP = 0.30    # max sideways shift of the leg line from side-wall protection
LAT_STEP = 0.008  # per 50 ms tick (0.16 m/s)
LAUNCH = (-2.51, 0.91)
PAD = (-2.04, -0.79)

# (cx, cy, solid_wall_target_depth_m or None)
WAYPOINTS = [
    (-2.51, 2.58, None),   # Leg 1: North up start corridor (midpoint between M1=2.12 and Net=3.05)
    (-1.38, 2.58, 0.52),   # Leg 2 (stop 0.52 from M2 = M1-M2 lane centre; 0.62 put Leg 3 at 0.27 m from M1, run_010): East over M1 -> solid wall anchor on M2 (x=-0.86, clears M1 top tip)
    (-1.42, 1.09, 0.58),   # Leg 3 (stop 0.58 from Barrier 2): SSW down M1-M2 -> solid wall anchor on Barrier 2 (y=+0.51, clears M2 bottom tip)
    (-0.37, 1.00, 0.62),   # Leg 4 (stop 0.62 from M3): East under M2 -> solid wall anchor on M3 at 0.58 m (keeps M3 edges in FOV)
    (-0.22, 2.58, None),   # Leg 5: NNE up M2-M3 lane (clears M3 top tip)
    ( 0.95, 2.58, None),   # Leg 6: East over M3
    ( 2.31, 1.80, None),   # Leg 7: Align with Tube north entrance (x=2.31)
    ( 2.31,-0.75, None),   # Leg 8: South through Tube (center x=2.31)
    ( 1.40,-1.80, None),   # Leg 9: SW, clear of C1 south tip (0.93,-1.11)
    ( 0.235,-2.525, None), # Leg 10: window approach on the window axis 0.75 m out (0.42 m from Boulder 3 at 0.76 m height)
    (-0.89,-2.12, 'WINDOW'),  # Leg 11: through the C2a window, heading -70 deg (perpendicular), at 0.76 m
    (-2.04,-0.79, None),   # Leg 12: NW onto Landing Pad center (-2.04, -0.79), back at 1.0 m
]

class LocalizerNode(Stage):
    def __init__(self):
        super().__init__()
        self.fx = 421.0
        self.cx_cam = 319.5
        self.f_hist = deque(maxlen=3)
        self.sectors = (None, None, None)  # (front_d, left_perp, right_perp)
        self.rays = []
        self.win = None
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_info, qos_profile_sensor_data)

    def on_info(self, m):
        if m.k[0] > 100.0:
            self.fx = float(m.k[0])
            self.cx_cam = float(m.k[2])

    def on_depth(self, m):
        # zero-copy view, then only the band rows we use (40-60 %), every 16th column
        a = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        h, w = a.shape
        band = a[int(h * 0.40):int(h * 0.60), ::16].astype(np.float32) / 1000.0
        band = np.where((band < 0.25) | (band > 4.0), 99.0, band)
        zs = np.percentile(band, 25, axis=0)
        front_f, rays = [], []
        for u, z in zip(range(0, w, 16), zs):
            b = math.atan((float(u) - self.cx_cam) / self.fx)
            zv = None if z >= 50.0 else float(z)       # z-depth: forward distance along the optical axis
            rays.append((b, zv))
            if zv is not None and abs(b) <= math.radians(11.0):
                front_f.append(zv)
        self.rays = rays
        front_f.sort()
        f_d = front_f[int(0.25 * len(front_f))] if len(front_f) >= 2 else None
        if f_d is not None:
            self.f_hist.append(f_d)
        f_med = sorted(self.f_hist)[len(self.f_hist) // 2] if self.f_hist else None
        f_ref = f_med if f_med is not None else 99.0
        left_p, right_p = [], []
        for b, zv in rays:
            if zv is None:
                continue
            lat = zv * math.tan(b)
            if 0.22 <= zv <= 0.85 and zv < f_ref - 0.22:
                if b <= math.radians(-22.0):
                    left_p.append(-lat)
                elif b >= math.radians(22.0):
                    right_p.append(lat)
        left_p.sort()
        right_p.sort()
        l_d = left_p[int(0.25 * len(left_p))] if len(left_p) >= 2 else None
        r_d = right_p[int(0.25 * len(right_p))] if len(right_p) >= 2 else None
        self.sectors = (f_med, l_d, r_d)
        # window detector: a run of far/no-return columns flanked by near wall columns
        self.win = None
        near = [zv for b, zv in rays if zv is not None and zv < 1.6]
        if len(near) >= 4:
            wall = sorted(near)[len(near) // 2]
            far = [zv is None or zv > wall + 0.6 for b, zv in rays]
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

class LegFail(Exception):
    def __init__(self, i, why):
        super().__init__(why)
        self.i = i


class Trail:
    """Foundational rule: never fly back into space the drone has already flown through.

    Breadcrumbs in course coordinates every 0.10 m. A direction "backtracks" when points
    0.3-0.7 m ahead along it come within 0.25 m of an old breadcrumb, ignoring breadcrumbs
    within 0.5 m of where the drone is now (the corner it is standing in).
    """
    def __init__(self):
        self.pts = []

    def add(self, cx, cy):
        if not self.pts or math.hypot(cx - self.pts[-1][0], cy - self.pts[-1][1]) >= 0.10:
            self.pts.append((cx, cy))

    def backtracks(self, cx, cy, hx, hy):
        old = [q for q in self.pts if math.hypot(q[0] - cx, q[1] - cy) > 0.5]
        for d in (0.3, 0.5, 0.7):
            ax, ay = cx + hx * d, cy + hy * d
            for qx, qy in old:
                if math.hypot(ax - qx, ay - qy) < 0.25:
                    return True
        return False

def to_ned(cx, cy, th=0.0):
    n_, e_ = cy - LAUNCH[1], cx - LAUNCH[0]
    return (n_ * math.cos(th) - e_ * math.sin(th), n_ * math.sin(th) + e_ * math.cos(th))

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
    cur = [x0, y0, z0]
    yaw_jump = [0.0]
    log('WP start NED=(%.2f, %.2f, %.2f) heading=%.2f; %d waypoints (no time limit)' % (
        x0, y0, z0, yaw0, len(WAYPOINTS)))
    off = [0.0, 0.0]
    theta = [0.0]     # estimate-frame heading minus map heading (from the scan matcher)
    M = course_matcher() if MATCH else None
    log('WP scan matcher %s' % ('ON' if MATCH else 'OFF'))
    last = [None]
    turning = [False]
    was_lost = [False]
    yaw_box = [yaw0]
    rot_events = []   # frame rotations to apply to the active leg line
    drift_ticks = [0]
    jump_run = [0]   # consecutive ignored position jumps (sustained motion is real, not noise)
    def absorb_rotation(dyaw, pn, pe, why):
        # The estimate frame rotated by dyaw about (pn, pe): rotate our yaw setpoint, our
        # map->estimate transform and the current setpoint with it, so the airframe does not react.
        yaw_box[0] = wrap(yaw_box[0] + dyaw)
        yaw_jump[0] += dyaw
        theta[0] = wrap(theta[0] + dyaw)
        cr, sr = math.cos(dyaw), math.sin(dyaw)
        bn, be = x0 + off[0] - pn, y0 + off[1] - pe
        off[0] = pn + bn * cr - be * sr - x0
        off[1] = pe + bn * sr + be * cr - y0
        qn, qe = cur[0] - pn, cur[1] - pe
        cur[0], cur[1] = pn + qn * cr - qe * sr, pe + qn * sr + qe * cr
        rot_events.append((dyaw, pn, pe))
        log('WP heading %s %+.1f deg -> ignored (total %+.0f deg)' % (why, math.degrees(dyaw), math.degrees(yaw_jump[0])))

    def tick():
        n.send(cur, yaw_box[0])
        spin(0.05)
        p = n.pos
        if p is None:
            was_lost[0] = True
            return
        if last[0] is not None:
            dyaw = wrap(p[3] - last[0][3])
            err_before = abs(wrap(last[0][3] - yaw_box[0]))
            err_after = abs(wrap(p[3] - yaw_box[0]))
            # Ignore only changes that are physically impossible AND move the estimate away from
            # our setpoint (a real rotation by PX4 toward the setpoint is fast but genuine).
            if abs(dyaw) > YAW_JUMP and err_after > err_before:
                absorb_rotation(dyaw, last[0][0], last[0][1], 'jump')
            dn, de = p[0] - last[0][0], p[1] - last[0][1]
            jump_m = 0.035 if (turning[0] or was_lost[0]) else POS_JUMP
            e_before = math.hypot(last[0][0] - cur[0], last[0][1] - cur[1])
            e_after = math.hypot(p[0] - cur[0], p[1] - cur[1])
            if math.hypot(dn, de) > jump_m and (e_after > e_before or was_lost[0]) and (jump_run[0] < 5 or was_lost[0]):
                jump_run[0] += 1
                cur[0] += dn
                cur[1] += de
                off[0] += dn
                off[1] += de
                log('WP position jump %.2f m (%s: n %+.2f e %+.2f) -> ignored, total off=(%+.2f, %+.2f)' % (
                    math.hypot(dn, de), 'TURN/RECOV' if (turning[0] or was_lost[0]) else 'FLY', dn, de, off[0], off[1]))
            else:
                jump_run[0] = 0
        # slow heading drift: estimate held far from our (steady) setpoint
        if not turning[0] and abs(wrap(p[3] - yaw_box[0])) > YAW_DRIFT:
            drift_ticks[0] += 1
            if drift_ticks[0] >= 10:
                absorb_rotation(wrap(p[3] - yaw_box[0]), p[0], p[1], 'drift')
                drift_ticks[0] = 0
        else:
            drift_ticks[0] = 0
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
        trail = Trail()
        def est_course(p):
            vn, ve = p[0] - x0 - off[0], p[1] - y0 - off[1]
            c, s_ = math.cos(-theta[0]), math.sin(-theta[0])
            mn, me = vn * c - ve * s_, vn * s_ + ve * c
            return (me + LAUNCH[0], mn + LAUNCH[1])
        last_match = [0.0]
        match_log = [0]
        on_pad = [False]
        next_wait = [10.0]

        def search(i, why):
            # Never land, never reverse: rotate in place in 30 deg steps until depth shows open space
            # (> 1.0 m) in a direction that does not re-enter flown space, move 0.4 m that way,
            # re-fix with the matcher, then the route is retried from leg i.
            log('WP SEARCH (leg %d: %s): rotating in place to find open space; not landing' % (i + 1, why))
            rounds = 0
            while True:
                for _ in range(12):
                    turning[0] = True
                    tgt = wrap(yaw_box[0] + math.radians(30))
                    t1 = time.time()
                    while abs(wrap(tgt - yaw_box[0])) > 1e-3 and time.time() - t1 < 6:
                        yaw_box[0] = wrap(yaw_box[0] + max(-YAW_RATE * 0.05, min(YAW_RATE * 0.05, wrap(tgt - yaw_box[0]))))
                        tick()
                    for _ in range(20):
                        tick()
                    turning[0] = False
                    p = n.pos
                    f = n.sectors[0]
                    if p is None or f is None or f < 1.0:
                        continue
                    ex, ey = est_course(p)
                    hm = wrap(p[3] - theta[0])
                    hx, hy = math.sin(hm), math.cos(hm)
                    if abs(ex + 0.6 * hx) > 2.6 or abs(ey + 0.6 * hy) > 2.6 or trail.backtracks(ex, ey, hx, hy):
                        continue
                    log('WP SEARCH: open, non-backtracking direction %.0f deg (F=%.2f) -> forward 0.4 m' % (math.degrees(hm), f))
                    sn_, se_, h_ = cur[0], cur[1], p[3]
                    s_, t1 = 0.0, time.time()
                    while s_ < 0.4 and time.time() - t1 < 8:
                        fd = n.sectors[0]
                        if fd is not None and fd < STOP_DEPTH:
                            break
                        s_ += 0.15 * 0.05
                        cur[0], cur[1] = sn_ + math.cos(h_) * s_, se_ + math.sin(h_) * s_
                        tick()
                    for _ in range(20):
                        tick()
                    if n.pos is not None:
                        trail.add(*est_course(n.pos))
                        last_match[0] = 0.0
                        do_match(n.pos, 'search')
                    return
                rounds += 1
                log('WP SEARCH: no open non-backtracking direction after %d full turn(s); still looking, not landing' % rounds)
        def do_match(p, label):
            # Absolute fix from depth vs. map; corrects off/theta only along observed directions.
            if M is None or time.time() - last_match[0] < 0.5 or not n.rays:
                return
            last_match[0] = time.time()
            scan = [(b, z / math.cos(b)) for b, z in n.rays if z is not None]
            px, py = est_course(p)
            ph = wrap(p[3] - theta[0])
            if px < 0.9 and py < -1.2:
                return       # C2a window area: the model treats the window as solid wall
            fix = M.match(px, py, ph, scan, ywin=0.0)
            if fix is None or fix['stiff'] == 0:
                return
            dx, dy, dh = fix['x'] - px, fix['y'] - py, wrap(fix['hdg'] - ph)
            if math.hypot(dx, dy) > 0.40 or fix['cost'] > 0.002:
                log('WP match %s REJECT d=(%+.2f,%+.2f) dh=%+.1f cost=%.4f' % (label, dx, dy, math.degrees(dh), fix['cost']))
                return
            k = MATCH_GAIN
            cdx = max(-MATCH_STEP, min(MATCH_STEP, k * dx))
            cdy = max(-MATCH_STEP, min(MATCH_STEP, k * dy))
            cdh = 0.0
            if abs(k * dx) < 0.02 and abs(k * dy) < 0.02:
                return       # deadband: do not integrate noise
            # map shift (cdx east, cdy north) -> estimate frame, then off -= R(theta) * shift
            dn_m, de_m = cdy, cdx
            c, s_ = math.cos(theta[0]), math.sin(theta[0])
            off[0] -= dn_m * c - de_m * s_
            off[1] -= dn_m * s_ + de_m * c
            theta[0] = wrap(theta[0] + cdh * -1.0)
            match_log[0] += 1
            if match_log[0] % 4 == 1:
                log('WP match %s pts=%d stiff=%d d=(%+.2f,%+.2f) dh=%+.1f cost=%.4f -> theta=%+.1f off=(%+.2f,%+.2f)' % (
                    label, fix['n'], fix['stiff'], dx, dy, math.degrees(dh), fix['cost'],
                    math.degrees(theta[0]), off[0], off[1]))
        if n.pos is not None:
            trail.add(*est_course(n.pos))
        start_i = 0
        while True:
          try:
                for i, (cx, cy, anchor_d) in enumerate(WAYPOINTS):
                    if i < start_i:
                        continue
                    is_win = anchor_d == 'WINDOW'
                    if is_win:
                        anchor_d = None
                    p = n.pos
                    if p is not None and i < len(WAYPOINTS) - 1:
                        ex, ey = est_course(p)
                        dx, dy = cx - ex, cy - ey
                        dl = math.hypot(dx, dy)
                        if dl > 0.2 and trail.backtracks(ex, ey, dx / dl, dy / dl):
                            log('WP leg %d SKIPPED: heading to (%.2f, %.2f) from (%.2f, %.2f) re-enters flown space (no-backtrack rule)' % (
                                i + 1, cx, cy, ex, ey))
                            continue
                    tn, te = to_ned(cx, cy, theta[0])
                    tn, te = x0 + tn + off[0], y0 + te + off[1]
                    sn, se = cur[0], cur[1]
                    o0 = list(off)
                    L = math.hypot(tn - sn, te - se)
                    heading = math.atan2(te - se, tn - sn)
                    yjp = yaw_jump[0]
                    log('WP leg %d -> course (%.2f, %.2f), %.2f m, heading %.0f deg, anchor=%s' % (
                        i + 1, cx, cy, L, math.degrees(heading), 'none' if anchor_d is None else '%.2fm' % anchor_d))
                    # 1) Turn in place (18 deg/s) with strict turn-jump rejection + settle
                    turning[0] = True
                    t_turn0 = time.time()
                    while abs(wrap(heading + (yaw_jump[0] - yjp) - yaw_box[0])) > 1e-3 and time.time() - t_turn0 < 15.0:
                        err_y = wrap(heading + (yaw_jump[0] - yjp) - yaw_box[0])
                        yaw_box[0] = wrap(yaw_box[0] + max(-YAW_RATE * 0.05, min(YAW_RATE * 0.05, err_y)))
                        tick()
                    t1 = time.time()
                    while (n.pos is None or abs(wrap(n.pos[3] - heading - (yaw_jump[0] - yjp))) > math.radians(4.5)) and time.time() - t1 < 4.0:
                        tick()
                    for _ in range(6):
                        tick()
                    z_t = z0 - (WINDOW_H if is_win else HEIGHT)
                    if abs(cur[2] - z_t) > 0.02:
                        log('WP leg %d: height -> %.2f m' % (i + 1, z0 - z_t))
                        t1 = time.time()
                        while time.time() - t1 < 10.0:
                            cur[2] += max(-0.0125, min(0.0125, z_t - cur[2]))
                            tick()
                            if abs(cur[2] - z_t) < 1e-3 and n.pos is not None and abs(n.pos[2] - z_t) < 0.06:
                                break
                        for _ in range(10):
                            tick()
                    turning[0] = False
                    if n.pos is not None:
                        last_match[0] = 0.0
                        do_match(n.pos, 'post-turn %d' % (i + 1))
                        # theta/off may have changed: recompute this leg's target from the map
                        tn, te = to_ned(cx, cy, theta[0])
                        tn, te = x0 + tn + off[0], y0 + te + off[1]
                    n.f_hist.clear()
                    # Re-anchor leg on post-turn setpoint
                    o0 = list(off)
                    sn, se = cur[0], cur[1]
                    L_nom = math.hypot(tn - sn, te - se)
                    L = L_nom
                    heading = math.atan2(te - se, tn - sn)
                    ch, sh = math.cos(heading), math.sin(heading)
                    yj0 = yaw_jump[0]
                    rot_events.clear()
                    # 2) Fly the line with full deceleration ramp, gated solid-wall depth anchor, and lag-free side centering
                    s, blocked_since, done, end_wait_t = 0.0, None, False, None
                    along_corr, lat_corr, settle_ticks = 0.0, 0.0, 0
                    best_along, t_prog, sidesteps = -1.0, time.time(), 0
                    t_leg0 = time.time()
                    while time.time() - t_leg0 < 45.0:
                        p = n.pos
                        if not n.armed():
                            raise RuntimeError('disarmed in flight')
                        if p is None:
                            if lost_since is None:
                                lost_since = time.time()
                                next_wait[0] = 10.0
                                log('WP POSITION LOST (estimator invalid): holding setpoint, waiting for recovery')
                            if time.time() - lost_since > next_wait[0]:
                                next_wait[0] += 10.0
                                log('WP still waiting for the estimator (%.0f s), not landing' % (time.time() - lost_since))
                            tick()
                            continue
                        if lost_since is not None:
                            log('WP position recovered after %.1f s' % (time.time() - lost_since))
                            last_match[0] = 0.0
                        lost_since = None
                        while rot_events:
                            # rotate the active leg line with the estimate frame
                            dyaw_r, pn_r, pe_r = rot_events.pop(0)
                            cr, sr = math.cos(dyaw_r), math.sin(dyaw_r)
                            for nm in ('s', 't'):
                                qn, qe = (sn, se) if nm == 's' else (tn, te)
                                qn, qe = qn - pn_r, qe - pe_r
                                qn, qe = pn_r + qn * cr - qe * sr, pe_r + qn * sr + qe * cr
                                if nm == 's':
                                    sn, se = qn, qe
                                else:
                                    tn, te = qn, qe
                            heading = wrap(heading + dyaw_r)
                            ch, sh = math.cos(heading), math.sin(heading)
                            yj0 = yaw_jump[0]
                            log('WP leg line rotated %+.0f deg with the estimate frame' % math.degrees(dyaw_r))
                        if off != o0:
                            dn_j, de_j = off[0] - o0[0], off[1] - o0[1]
                            sn, se = sn + dn_j, se + de_j
                            tn, te = tn + dn_j, te + de_j
                            o0 = list(off)
                        along = (p[0] - sn) * ch + (p[1] - se) * sh
                        lat_pos = -(p[0] - sn) * sh + (p[1] - se) * ch
                        f_d, l_d, r_d = n.sectors
                        trail.add(*est_course(p))
                        # Stuck (no forward progress, not a depth stop): sidestep toward the open side,
                        # never reverse along the leg (no-backtrack rule).
                        if along > best_along + 0.03:
                            best_along, t_prog = along, time.time()
                        elif s < L and s - along > 0.10 and time.time() - t_prog > 2.5 and blocked_since is None:
                            if sidesteps >= 3:
                                log('WP leg %d: stuck after %d sidesteps at %.2f/%.2f m -> next leg' % (i + 1, sidesteps, along, L))
                                break
                            l_open = 9.0 if l_d is None else l_d
                            r_open = 9.0 if r_d is None else r_d
                            step_side = 0.12 if r_open >= l_open else -0.12
                            lat_corr = max(-LAT_CAP, min(LAT_CAP, lat_corr + step_side))
                            sidesteps += 1
                            t_prog = time.time()
                            log('WP leg %d: STUCK at %.2f/%.2f m (L=%s R=%s) -> sidestep %s, lcorr=%+.2f' % (
                                i + 1, along, L, '--' if l_d is None else '%.2f' % l_d,
                                '--' if r_d is None else '%.2f' % r_d, 'right' if step_side > 0 else 'left', lat_corr))
                        hdg_ok = abs(wrap(p[3] - heading - (yaw_jump[0] - yj0))) < math.radians(6.5)
                        if hdg_ok:
                            do_match(p, 'leg %d' % (i + 1))
                        # (A) Gated solid-wall front depth calibration (exact run_004 logic; matcher replaces it)
                        if not MATCH and anchor_d is not None and hdg_ok and along > max(0.35, L_nom - 0.50) and f_d is not None and 0.36 <= f_d <= 0.92:
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
                        # (W) Window: centre on the gap seen by depth until 0.55 m from the end of the leg
                        if is_win and hdg_ok and n.win is not None and along < L - 0.55:
                            w_lat = n.win[0]
                            if abs(w_lat) > 0.01:
                                des_lat = max(-LAT_CAP, min(LAT_CAP, lat_pos + w_lat))
                                lat_corr += max(-LAT_STEP, min(LAT_STEP, 0.25 * (des_lat - lat_corr)))
                        # (B) Lag-free side-wall protection referenced to actual drone position `lat_pos`
                        if hdg_ok and 0.05 < along < L - 0.25 and i != 0 and not is_win:
                            delta_lat = None
                            if l_d is not None and r_d is not None and l_d < 0.75 and r_d < 0.75:
                                delta_lat = 0.5 * (r_d - l_d)
                            elif l_d is not None and l_d < SIDE_M:
                                delta_lat = SIDE_M - l_d
                            elif r_d is not None and r_d < SIDE_M:
                                delta_lat = -(SIDE_M - r_d)
                            if delta_lat is not None and abs(delta_lat) > 0.015:
                                des_lat = max(-LAT_CAP, min(LAT_CAP, lat_pos + delta_lat))
                                step_lat = max(-LAT_STEP, min(LAT_STEP, 0.25 * (des_lat - lat_corr)))
                                lat_corr += step_lat
                        # (C) Protective brake + full deceleration ramp to 0.07 m/s
                        stop_lim = 0.38 if anchor_d is not None else STOP_DEPTH
                        blocked = (f_d is not None and f_d < stop_lim)   # depth safety anywhere on the leg
                        rem = max(0.0, L - s)
                        v_des = max(0.07, min(SPEED, 0.55 * rem + 0.05)) if rem < 0.45 else SPEED
                        if blocked:
                            s = min(s, max(0.0, along))
                            blocked_since = blocked_since or time.time()
                            if is_win and time.time() - blocked_since > 20.0:
                                raise LegFail(i, 'window blocked by depth (front %.2f m) for 20 s' % f_d)
                            if not is_win and time.time() - blocked_since > 1.2:
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
                            est_cx, est_cy = est_course(p)
                            log('WP t=%4.1f leg %d c=(%.2f,%.2f) yaw=%4.0f s=%.2f/%.2f F=%s L=%s R=%s dcorr=%+.2f lcorr=%+.2f' % (
                                time.time() - t_fly0, i + 1, est_cx, est_cy, math.degrees(p[3]),
                                along, L,
                                '--' if f_d is None else '%.2f' % f_d,
                                '--' if l_d is None else '%.2f' % l_d,
                                '--' if r_d is None else '%.2f' % r_d,
                                along_corr, lat_corr))
                        tick()
                    if is_win and not done:
                        raise LegFail(i, 'window not passed')
                    if done:
                        p = n.pos
                        if p is not None:
                            est_cx, est_cy = est_course(p)
                            log('WP leg %d reached: course=(%.2f, %.2f) F=%s dcorr=%+.2f lcorr=%+.2f t=%.1f' % (
                                i + 1, est_cx, est_cy,
                                '--' if n.sectors[0] is None else '%.2f' % n.sectors[0],
                                along_corr, lat_corr, time.time() - t_fly0))
                else:
                    p = n.pos
                    if p is not None and done:
                        ex_, ey_ = est_course(p)
                        d_pad = math.hypot(ex_ - PAD[0], ey_ - PAD[1])
                        if d_pad < 0.45:
                            result = 'reached landing pad (estimate %.2f m from centre)' % d_pad
                            on_pad[0] = True
                            break
                        raise LegFail(len(WAYPOINTS) - 1, 'route ended %.2f m from the pad' % d_pad)
                    raise LegFail(len(WAYPOINTS) - 1, 'last leg not finished')
          except LegFail as lf:
            search(lf.i, str(lf))
            start_i = lf.i
    except Exception as exc:
        result = 'error: %s' % exc
        log('WP ' + result)
    finally:
        if result.startswith('reached landing pad'):
            log('WP landing on the pad (%s)' % result)
            t0 = time.time()
            while n.armed() and time.time() - t0 < 40:
                n.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                spin(0.5)
        else:
            log('WP NOT landing: not on the pad (%s)' % result)
        p = n.pos
        log('WP RESULT: %s | landed=%s | final course=%s | total off=(%+.2f, %+.2f)' % (
            result, not n.armed(),
            'n/a' if p is None else '(%.2f, %.2f)' % (p[1] - y0 - off[1] + LAUNCH[0], p[0] - x0 - off[0] + LAUNCH[1]),
            off[0], off[1]))
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
