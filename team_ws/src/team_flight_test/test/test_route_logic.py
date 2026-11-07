"""Closed-loop tests of route_logic with a toy drone (first-order lag toward the setpoint).
No ROS. Run: python3 -m pytest test/ -q   or   python3 test/test_route_logic.py
This checks the logic, NOT flight behaviour: the toy drone has no dynamics worth the name.
"""
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

from team_flight_test import route_logic as rl  # noqa: E402

DT = 0.05


def fly(cfg, start=(2.0, -1.0, 1.0, 0.7), until=200.0, hook=None, tau=0.4, yaw_tau=0.3):
    """Returns (route, list of (t, pose, setpoint, phase), events)."""
    r = rl.Route(cfg)
    pose = rl.Pose(*start)
    t = 0.0
    r.observe(t, pose)
    out = r.start(t, pose, hover_z=1.0, ground_z=0.0)
    events, trace = list(out.events), []
    sp = out.setpoint
    odom_t = t
    while t < until and r.phase not in (rl.DONE, rl.STOPPED):
        t += DT
        a = DT / tau
        pose = rl.Pose(pose.x + (sp[0] - pose.x) * a, pose.y + (sp[1] - pose.y) * a,
                       pose.z + (sp[2] - pose.z) * a,
                       rl.wrap(pose.yaw + rl.wrap(sp[3] - pose.yaw) * DT / yaw_tau))
        jump = None
        if hook:
            pose, odom_t, jump = hook(t, pose, odom_t)
        else:
            odom_t = t
        j = r.observe(odom_t, pose) if odom_t == t else None
        out = r.step(t, pose, odom_t, jump if jump is not None else j)
        events += out.events
        if out.setpoint:
            sp = out.setpoint
        trace.append((t, pose, sp, r.phase))
    return r, trace, events


def body(r, pose):
    o = r.origin
    dx, dy = pose.x - o.x, pose.y - o.y
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    return dx * c + dy * s, -dx * s + dy * c


def test_every_profile_completes():
    for name in rl.PROFILES:
        r, trace, ev = fly(rl.Config(profile=name, max_run_s=200))
        assert r.done_reason == 'route complete', (name, r.done_reason, ev[-3:])


def test_course_endpoint_in_body_frame():
    r, trace, _ = fly(rl.Config(profile='course', max_run_s=200))
    f, l = body(r, trace[-1][1])
    assert abs(f - 1.64) < 0.2 and abs(l + 3.31) < 0.2, (f, l)   # right of start = negative left


def test_course_faces_each_leg_before_moving():
    r, trace, _ = fly(rl.Config(profile='course', max_run_s=200))
    fly_rows = [(pose, sp) for _, pose, sp, ph in trace if ph == rl.FLY]
    assert fly_rows
    for pose, sp in fly_rows:
        # flying only starts after the turn finished: the drone already faces within 5 deg
        assert abs(rl.wrap(pose.yaw - sp[3])) < math.radians(6), math.degrees(pose.yaw - sp[3])


def test_out_back_keeps_yaw():
    start = (0.0, 0.0, 1.0, 0.3)
    r, trace, _ = fly(rl.Config(profile='out_back'), start=start)
    assert all(abs(rl.wrap(sp[3] - 0.3)) < 1e-6 for _, _, sp, _ in trace)
    assert r.done_reason == 'route complete'


def test_yaw_rate_limited():
    r, trace, _ = fly(rl.Config(profile='yaw', yaw_rate_dps=20))
    prev = None
    for _, _, sp, _ in trace:
        if prev is not None:
            assert abs(rl.wrap(sp[3] - prev)) <= math.radians(20) * DT + 1e-6
        prev = sp[3]


def test_max_legs_and_scale():
    cfg = rl.Config(profile='course', max_legs=2, scale=0.5)
    gotos = [s for s in cfg.steps() if s[0] == 'goto']
    assert len(gotos) == 2 and abs(gotos[0][1] - 0.82) < 1e-9


def test_watchdog_stale_odometry():
    def hook(t, pose, odom_t):
        return pose, (t if t < 3.0 else odom_t), None   # odometry stops at 3 s
    r, _, _ = fly(rl.Config(profile='hover'), hook=hook)
    assert 'stale' in r.done_reason


def test_watchdog_jump():
    def hook(t, pose, odom_t):
        if abs(t - 4.0) < DT / 2:
            pose = rl.Pose(pose.x + 0.8, pose.y, pose.z, pose.yaw)
        return pose, t, None
    r, _, _ = fly(rl.Config(profile='hover'), hook=hook)
    assert 'jump' in r.done_reason, r.done_reason


def test_small_jump_logged_not_landed():
    def hook(t, pose, odom_t):
        if abs(t - 4.0) < DT / 2:
            pose = rl.Pose(pose.x + 0.2, pose.y, pose.z, pose.yaw)
        return pose, t, None
    r, _, _ = fly(rl.Config(profile='hover'), hook=hook)
    assert r.done_reason == 'route complete' and r.jumps >= 1


def test_watchdog_track_error_drift():
    def hook(t, pose, odom_t):   # strong push the controller cannot hold (steady state ~1.2 m off)
        return rl.Pose(pose.x + 0.15, pose.y, pose.z, pose.yaw), t, None
    r, _, _ = fly(rl.Config(profile='hover'), hook=hook)
    assert r.done_reason and ('track' in r.done_reason or 'from start' in r.done_reason), r.done_reason


def test_watchdog_altitude():
    def hook(t, pose, odom_t):
        return rl.Pose(pose.x, pose.y, 0.2 if t > 2 else pose.z, pose.yaw), t, None
    r, _, _ = fly(rl.Config(profile='hover'), hook=hook)
    assert 'altitude' in r.done_reason


def test_run_timer():
    r, _, _ = fly(rl.Config(profile='course', max_run_s=10))
    assert 'timer' in r.done_reason


def test_pilot_takeover_stops_publishing():
    r = rl.Route(rl.Config(profile='hover'))
    p = rl.Pose(0, 0, 1, 0)
    r.start(0.0, p, 1.0, 0.0)
    r.pilot_took_over(rl.Out())
    assert r.step(0.1, p, 0.1).setpoint is None


def test_bad_params_rejected():
    for kw in ({'speed': 2.0}, {'profile': 'nope'}, {'scale': 5}, {'yaw_rate_dps': 200}):
        try:
            rl.Route(rl.Config(**kw))
        except ValueError:
            continue
        raise AssertionError(kw)


def test_next_call_sequence():
    assert rl.next_call(rl.M_WAIT_OFFBOARD, {'mode': 'passthrough', 'offboard': False}) is None
    assert rl.next_call(rl.M_WAIT_OFFBOARD, {'mode': 'passthrough', 'offboard': True}) == 'takeoff'
    assert rl.next_call(rl.M_TAKEOFF, {'mode': 'takeoff'}) is None
    assert rl.next_call(rl.M_TAKEOFF, {'mode': 'hover'}) == 'release'
    assert rl.next_call(rl.M_ROUTE, {'mode': 'passthrough'}) is None


def test_analyzer_on_synthetic_log():
    import analyze_flight
    r, trace, ev = fly(rl.Config(profile='box'))
    d = tempfile.mkdtemp()
    cols = ['t', 'mphase', 'phase', 'step', 'tl_mode', 'offboard', 'armed', 'nav_state', 'sp_x', 'sp_y',
            'sp_z', 'sp_yaw', 'vio_x', 'vio_y', 'vio_z', 'vio_yaw', 'vio_age', 'px4_x', 'px4_y', 'px4_z',
            'px4_yaw', 'px4_xy_valid', 'px4_age', 'track_err', 'jump_m']
    with open(os.path.join(d, 'ticks.csv'), 'w') as fh, open(os.path.join(d, 'odom.csv'), 'w') as fo:
        fh.write(','.join(cols) + '\n')
        fo.write('t,stamp,x,y,z,yaw,step_m\n')
        prev = None
        for t, p, sp, ph in trace:
            te = math.hypot(p.x - sp[0], p.y - sp[1])
            fh.write(','.join(str(v) for v in [t, 'route', ph, 1, 'passthrough', 1, 1, 'OFFBOARD', sp[0], sp[1],
                                               sp[2], sp[3], p.x, p.y, p.z, p.yaw, 0.01, p.x + 0.05, p.y, p.z,
                                               p.yaw, 1, 0.01, te, '']) + '\n')
            st = 0.0 if prev is None else math.hypot(p.x - prev.x, p.y - prev.y)
            fo.write('%f,%f,%f,%f,%f,%f,%f\n' % (t, t, p.x, p.y, p.z, p.yaw, st))
            prev = p
    with open(os.path.join(d, 'events.log'), 'w') as fh:
        fh.write('\n'.join('%s %s' % e for e in ev))
    text, S = analyze_flight.analyze(d)
    assert 'fly' in S['tracking'] and S['turns'], text
    assert abs(S['vio_px4_offset_xy'][0] - (-0.05)) < 1e-6


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS', name)
            except Exception as exc:  # noqa: BLE001
                fails += 1
                print('FAIL', name, repr(exc))
    sys.exit(1 if fails else 0)
