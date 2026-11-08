"""Unit tests for px4_route.route (no ROS needed):  python -m pytest team_src/px4_route/test"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from px4_route.route import (Carrot, Path, RouteError, load_return_route, load_route,  # noqa: E402
                             start_frame_to_enu, validate_route)

ROUTES = os.path.join(os.path.dirname(__file__), '..', 'routes')


def route(**kw):
    raw = {'waypoints': [[1.0, 0.0, 0.8], [1.0, 1.0, 0.8]]}
    raw.update(kw)
    return validate_route(raw)


def path_of(r):
    c = r.cfg
    return Path(r.points, r.speeds, c['corner_speed'], c['corner_angle_deg'], c['corner_radius'])


# ---------------- validation ----------------
def test_shipped_routes_load():
    for f in os.listdir(ROUTES):
        if f.endswith('.yaml'):
            r = load_route(os.path.join(ROUTES, f))
            assert r.points[0] == (0.0, 0.0, 0.0)
            assert r.points[1][:2] == (0.0, 0.0)          # climb straight up first


@pytest.mark.parametrize('bad, why', [
    ({'waypoints': []}, 'non-empty'),
    ({'waypoints': [[1, 0]]}, 'expected'),
    ({'waypoints': [[1, 0, 2.0]]}, 'outside'),            # above max_height 1.5
    ({'waypoints': [[1, 0, 0.1]]}, 'outside'),            # below 0.3
    ({'waypoints': [[1, 0, 0.8, 9.0]]}, 'speed'),
    ({'waypoints': [[1, 0, 0.8]], 'speed': 10}, 'speed'),
    ({'waypoints': [[1, 0, 0.8]], 'max_height': 2.9}, 'max_height'),
    ({'waypoints': [[1, 0, 0.8]], 'spead': 0.3}, 'unknown'),
    ({'waypoints': [[1, 0, float('nan')]]}, 'finite'),
    ({'waypoints': [[1, 0, 0.8]], 'yaw': 'spin'}, 'yaw'),
])
def test_bad_routes_refused(bad, why):
    with pytest.raises(RouteError, match=why):
        validate_route(bad)


def test_above_walls_warns():
    r = validate_route({'waypoints': [[1, 0, 1.8]], 'max_height': 2.0})
    assert any('walls' in w for w in r.warnings)
    assert not route().warnings


def test_vertical_legs_limited_to_climb_speed():
    r = route(speed=1.0, climb_speed=0.3)
    assert r.speeds[1] == pytest.approx(0.3)              # the climb
    r2 = validate_route({'waypoints': [[0.3, 0.0, 1.2]], 'speed': 1.0, 'climb_speed': 0.3})
    leg = math.dist(r2.points[1], r2.points[2])
    assert r2.speeds[2] == pytest.approx(0.3 * leg / 0.4)  # 0.4 m up over that leg


# ---------------- frames ----------------
@pytest.mark.parametrize('heading_deg, fwd_enu, left_enu', [
    (0, (0, 1), (-1, 0)),      # nose North: forward = +N, left = West
    (90, (1, 0), (0, 1)),      # nose East: forward = +E, left = North
    (180, (0, -1), (1, 0)),
])
def test_start_frame_to_enu(heading_deg, fwd_enu, left_enu):
    h = math.radians(heading_deg)
    s = (10.0, 20.0, 0.05)
    f = start_frame_to_enu((1, 0, 0.8), s, h)
    lft = start_frame_to_enu((0, 1, 0.8), s, h)
    assert f[:2] == pytest.approx((10 + fwd_enu[0], 20 + fwd_enu[1]), abs=1e-9)
    assert lft[:2] == pytest.approx((10 + left_enu[0], 20 + left_enu[1]), abs=1e-9)
    assert f[2] == pytest.approx(0.85)                    # height is above the takeoff point


# ---------------- path ----------------
def test_corners_and_end_slow_down():
    p = path_of(route(speed=1.0, corner_speed=0.2, corner_radius=0.3))
    # climb->leg 1 is a 90 deg turn at s = 0.8; leg 1 -> leg 2 another at s = 1.8
    assert p.speed_at(1.3) == pytest.approx(1.0)
    assert p.speed_at(1.75) == pytest.approx(0.2)
    assert p.speed_at(p.length - 0.1) == pytest.approx(0.2)


def test_nearest_respects_window():
    # out along x and back: a point on the way back must not match the way out if windowed
    p = Path([(0, 0, 1), (2, 0, 1), (0, 0.1, 1)], [0, 1, 1], 0.2, 30, 0.3)
    s, d = p.nearest((0.5, 0.1, 1.0), s_lo=2.5)
    assert s > 2.0 and d < 0.05


def test_eta():
    p = path_of(route(speed=0.5, corner_speed=0.5, climb_speed=0.5))
    assert p.eta_s() == pytest.approx(p.length / 0.5, rel=0.05)


# ---------------- carrot ----------------
def test_carrot_waits_for_the_drone():
    p = path_of(route(speed=0.5))
    c = Carrot(p, lead_base=0.3, kp=1.0)
    for _ in range(200):                                  # drone stuck on the ground
        c.step(0.1, (0.0, 0.0, 0.0))
    assert math.dist(p.point_at(c.s), (0, 0, 0)) <= c.max_lead() + 0.05


def sim(route_obj, kp=0.95, vmax=12.0, dt=0.05, lag_s=0.1):
    """Very simple PX4 stand-in: velocity = kp * (setpoint - position), applied after a lag.
    Returns (time to finish, max cross-track)."""
    p = path_of(route_obj)
    c = Carrot(p, 0.3, kp)
    pos = [0.0, 0.0, 0.0]
    hist = []
    t = 0.0
    max_xt = 0.0
    while t < 400:
        sp = c.step(dt, tuple(pos))
        hist.append(sp)
        use = hist[max(0, len(hist) - 1 - int(lag_s / dt))]
        v = [kp * (use[i] - pos[i]) for i in range(3)]
        n = math.sqrt(sum(x * x for x in v))
        if n > vmax:
            v = [x * vmax / n for x in v]
        pos = [pos[i] + v[i] * dt for i in range(3)]
        t += dt
        if c.s > 1.0:
            max_xt = max(max_xt, p.nearest(tuple(pos), c.s - 2.0, c.s + 0.5)[1])
        if c.at_end() and math.dist(pos, p.point_at(p.length)) < 0.2:
            return t, max_xt
    return math.inf, max_xt


def test_sim_box_finishes_and_tracks():
    t, xt = sim(load_route(os.path.join(ROUTES, 'test_box.yaml')))
    assert t < 60 and xt < 0.25


# ---------------- return to base ----------------
def test_return_off_by_default():
    for f in ('test_line.yaml', 'test_box.yaml', 'course_walk_DRAFT.yaml'):
        p = os.path.join(ROUTES, f)
        assert load_return_route(p, load_route(p)) is None, f


def test_return_loads_relative_to_the_route_file():
    p = os.path.join(ROUTES, 'test_hop.yaml')
    ret = load_return_route(p, load_route(p))
    assert ret.name == 'test-hop-return'
    assert ret.path == os.path.join(os.path.dirname(os.path.abspath(p)), 'test_hop_return.yaml')


def test_course_return_route_is_valid_and_high():
    r = load_route(os.path.join(ROUTES, 'return_to_x.yaml'))
    assert r.cfg['takeoff_height'] >= 2.0 and r.cfg['max_height'] <= 2.6
    assert math.hypot(*r.points[-1][:2]) < 0.61 - 0.2     # ends near the X (rule 7.10: within 2 ft), with drift margin


def test_return_route_problems_refuse(tmp_path):
    main = tmp_path / 'main.yaml'
    main.write_text('waypoints: [[1, 0, 0.8]]\nreturn_route: missing.yaml\n')
    with pytest.raises(RouteError, match='cannot read'):
        load_return_route(str(main), load_route(str(main)))
    (tmp_path / 'ret.yaml').write_text('waypoints: [[0, 0, 0.8]]\nreturn_route: main.yaml\n')
    main.write_text('waypoints: [[1, 0, 0.8]]\nreturn_route: ret.yaml\n')
    with pytest.raises(RouteError, match='one return only'):
        load_return_route(str(main), load_route(str(main)))
    with pytest.raises(RouteError, match='return_route'):
        validate_route({'waypoints': [[1, 0, 0.8]], 'return_route': 5})
