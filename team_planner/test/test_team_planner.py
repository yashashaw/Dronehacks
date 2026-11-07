"""Tests that run without ROS or a drone: depth conversion and the planner's run rules."""
import math

import numpy as np

from team_planner.depth_scan import ScanParams, default_intrinsics, depth_to_scan
from team_planner.planner_logic import PlannerConfig, PlannerLogic

W, H = 320, 200
FX, FY, CX, CY = default_intrinsics(W, H)
CAM_H = 0.76            # camera height above the floor in these scenes


def rays():
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    return (u - CX) / FX, (v - CY) / FY         # x/z and y_down/z for every pixel


def corridor_depth(half_width=0.5, length=12.0, ceiling=2.5):
    """Depth (mm, uint16) of a straight corridor seen from its centre line, built from geometry."""
    dx, dy = rays()
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.full(dx.shape, length)
        t = np.minimum(t, np.where(dy > 0, CAM_H / dy, np.inf))                 # floor
        t = np.minimum(t, np.where(dy < 0, (ceiling - CAM_H) / -dy, np.inf))    # ceiling
        t = np.minimum(t, np.where(dx > 0, half_width / dx, np.inf))            # right wall
        t = np.minimum(t, np.where(dx < 0, half_width / -dx, np.inf))           # left wall
    return np.clip(t * 1000, 0, 65535).astype(np.uint16)


# ---- depth_to_scan ------------------------------------------------------------------------
def test_flat_wall_ranges_and_sides():
    depth = np.full((H, W), 2000, dtype=np.uint16)
    out, valid = depth_to_scan(depth, FX, FY, CX, CY)
    assert valid == 1.0 and len(out) == 48
    bearings = [b for b, _ in out]
    assert bearings[0] < 0 < bearings[-1] and bearings == sorted(bearings)
    centre = min(out, key=lambda o: abs(o[0]))
    assert abs(centre[1] - 2.0) < 0.05
    edge = out[0]
    assert abs(edge[1] - 2.0 / math.cos(edge[0])) < 0.1        # range grows toward the edge


def test_left_is_positive_bearing():
    depth = np.zeros((H, W), dtype=np.uint16)
    depth[:, : W // 4] = 1500                                   # obstacle on the image's left side
    out, _ = depth_to_scan(depth, FX, FY, CX, CY)
    assert out and all(b > 0 for b, _ in out)


def test_floor_is_ignored():
    dx, dy = rays()
    with np.errstate(divide="ignore", invalid="ignore"):
        floor = np.where(dy > 0.02, CAM_H / dy, 0.0)
    out, _ = depth_to_scan((floor * 1000).clip(0, 65535).astype(np.uint16), FX, FY, CX, CY)
    assert out == []


def test_low_obstacle_inside_the_body_band_is_kept_but_a_lower_one_is_not():
    z = 2.0
    for top, expect in ((0.53, True), (0.30, False)):
        up = top - CAM_H                                        # height of the obstacle top relative to the camera
        row = int(round(CY - up / z * FY))                      # image row of that height at depth z
        depth = np.zeros((H, W), dtype=np.uint16)
        depth[row - 1: row + 2, :] = int(z * 1000)
        out, _ = depth_to_scan(depth, FX, FY, CX, CY, ScanParams())
        assert bool(out) == expect


def test_isolated_speckle_is_dropped():
    depth = np.zeros((H, W), dtype=np.uint16)
    depth[int(CY), 100] = 1500                                       # one pixel
    assert depth_to_scan(depth, FX, FY, CX, CY)[0] == []


# ---- PlannerLogic: run rules ---------------------------------------------------------------
STATE_UP = {"armed": True, "offboard_active": True, "odom_enu": [0.0, 0.0, 0.0], "local_enu": [0.0, 0.0, 0.0]}


def started_logic(frames=25):
    logic = PlannerLogic(PlannerConfig(depth_decimation=1))
    logic.on_state(STATE_UP, 0.0)
    depth = corridor_depth()
    for i in range(frames):
        t = i / 28.0
        logic.on_odometry(t, 0.0, 0.0, 0.0, 0.0)
        logic.on_depth(t, t, depth)
    return logic, frames / 28.0


def test_no_goal_before_offboard():
    logic = PlannerLogic(PlannerConfig(depth_decimation=1))
    logic.on_odometry(0.0, 0.0, 0.0, 0.0, 0.0)
    logic.on_depth(0.0, 0.0, corridor_depth())
    assert logic.goal(0.0) is None


def test_goal_points_down_the_corridor_at_the_flight_altitude():
    logic, now = started_logic()
    goal = logic.goal(now)
    assert goal is not None, logic.debug()
    x, y, z, yaw = goal
    assert x > 0.1 and abs(y) < 0.4 and abs(yaw) < 0.6
    assert z == logic.cfg.flight_altitude_m
    assert math.hypot(x, y) <= logic.cfg.goal_max_m + 1e-9


def test_stale_depth_stops_the_goal():
    logic, now = started_logic()
    assert logic.goal(now) is not None
    assert logic.goal(now + 5.0) is None


def test_everything_is_forgotten_at_each_arming():
    logic, now = started_logic()
    assert logic.nav.grid.w > 80 or logic.nav.mapper.logodds.any()      # it had learned something
    logic.on_state({"armed": False, "offboard_active": False}, now)
    assert logic.goal(now) is None                                       # session over
    logic.on_state({"armed": True, "offboard_active": False}, now)        # armed again: new run
    assert not logic.nav.mapper.logodds.any() and logic.nav.grid.w == 80
    assert logic.last_cmd is None and len(logic.odom) == 0 and logic.gap0 is None


def test_ekf_drifting_away_from_vio_holds_the_planner():
    logic, now = started_logic()
    assert logic.goal(now) is not None
    logic.on_state(dict(STATE_UP, local_enu=[1.0, 0.0, 0.0]), now)      # EKF2 jumped 1 m, VIO did not
    assert logic.hold_reason == "ekf_gap" and logic.goal(now) is None
    logic.on_state(dict(STATE_UP, local_enu=[0.1, 0.0, 0.0]), now)
    assert logic.hold_reason is None and logic.goal(now) is not None


def test_health_error_stops_and_warn_slows():
    logic, now = started_logic()
    logic.on_health(1)
    assert logic.nav.p.v_max == logic.cfg.v_max * logic.cfg.warn_speed_factor
    logic.on_health(2)
    assert logic.goal(now) is None
    logic.on_health(0)
    assert logic.nav.p.v_max == logic.cfg.v_max and logic.goal(now) is not None


def test_pad_hint_only_counts_during_a_run():
    logic = PlannerLogic()
    logic.on_pad({"detected": True, "position_enu": [2.0, 1.0, 0.0]})
    assert logic.nav.pad is None
    logic.on_state(STATE_UP, 0.0)
    logic.on_pad({"detected": True, "position_enu": [2.0, 1.0, 0.0]})
    assert logic.nav.pad == (2.0, 1.0)
