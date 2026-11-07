"""The planner's logic with no ROS in it, so it runs under pytest.

Inputs: odometry, depth images, /skybeta/state, pad detections, a health level.
Output: the point and heading for /course/goal (Petr's pad_guide steps toward it).

Three modes (config.mode):
  general   Nothing about the course is known. Frontier exploration on the live depth map.
  hybrid    COURSE-SPECIFIC hints: a hardcoded list of waypoints picks the goals; the live depth
            map and the route finder still choose the path and avoid obstacles.
  scripted  COURSE-SPECIFIC: fly the waypoint list from odometry alone, no depth needed. For
            testing the flight, landing and recovery path when perception is the unknown.

Hints are course-frame coordinates in a JSON file (made by tools/make_course_hints.py) plus the
assumed start pose of the drone, so the odometry frame (origin = startup pose, +x = the way the
drone faced) can be related to the course. Anything that reads hints is layout-specific and costs
generality points if flown in a scored run.

Rules this file enforces:
  * Everything the planner knows is dropped at every arming and every Offboard start (rule 6.4:
    nothing learned in one run may reach the next). Nothing is ever written to disk.
  * When localization looks wrong it stops producing goals, and the guide then holds.
"""
import json
import math
from collections import deque
from dataclasses import dataclass

from .depth_scan import ScanParams, default_intrinsics, depth_to_scan
from .planner_core.navigator import NavParams, Navigator
from .planner_core.theta_star import FREE, Grid, theta_star


@dataclass
class PlannerConfig:
    mode: str = "general"               # general | hybrid | scripted
    hints_file: str = ""                # JSON from tools/make_course_hints.py (hybrid / scripted)
    route_planner: str = "astar"        # astar | theta | astar_smooth (astar had the fewest touches in the benchmark)
    v_max: float = 0.5                  # cautious first-run speed, m/s
    flight_altitude_m: float = 0.7      # goal height above the odometry origin, m
    camera_start_height_m: float = 0.10  # camera lens above the floor when the drone sits on the launch mark
    goal_lookahead_s: float = 1.0       # goal is placed speed x this far ahead (sets how fast PX4 chases it)
    goal_min_m: float = 0.15
    goal_max_m: float = 1.0
    depth_decimation: int = 2           # use every n-th depth frame (28 Hz is more than needed)
    max_depth_age_s: float = 0.5        # stop issuing goals if depth stops arriving
    max_ekf_gap_m: float = 0.5          # hold if EKF2 and VIO drift apart by more than this since the start
    warn_speed_factor: float = 0.5      # speed multiplier while localization health reads WARN
    waypoint_radius_m: float = 0.5      # a waypoint counts as reached inside this distance
    publish_pad_hint: bool = False      # COURSE-SPECIFIC: announce the hinted pad position near the pad
    pad_hint_radius_m: float = 1.0      # ... only once the drone is this close to it


def load_hints(path):
    """Hints file -> dict, or None. Course-frame points plus the assumed start pose."""
    if not path:
        return None
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def course_to_odom(point, start):
    """Course frame -> odometry frame, given where the drone starts: odom origin is the startup
    pose and +x is the direction the drone faced then (heading_deg in the course frame)."""
    dx, dy = point[0] - start["x"], point[1] - start["y"]
    h = math.radians(start["heading_deg"])
    return (math.cos(h) * dx + math.sin(h) * dy, -math.sin(h) * dx + math.cos(h) * dy)


class PlannerLogic:
    def __init__(self, config=None, hints=None):
        self.cfg = config or PlannerConfig()
        self.hints = hints if hints is not None else load_hints(
            self.cfg.hints_file if self.cfg.mode != "general" else "")
        self.mode = self.cfg.mode if (self.cfg.mode == "general" or self.hints) else "general"
        self.scan_params = ScanParams()
        self.resets = -1
        self._armed = False
        self._offboard = False
        self.reset()

    # ---- run lifecycle ---------------------------------------------------------------------
    def reset(self):
        """Forget everything. Called at every arming and every Offboard start."""
        self.resets += 1
        self.nav = Navigator(0.05, NavParams(planner=self.cfg.route_planner, v_max=self.cfg.v_max))
        if self.mode != "general":
            start = self.hints["start"]
            self.nav.set_route([course_to_odom((w["x"], w["y"]), start) for w in self.hints["waypoints"]],
                               pad=course_to_odom((self.hints["pad"]["x"], self.hints["pad"]["y"]), start),
                               radius=self.cfg.waypoint_radius_m)
        self.session_active = False
        self.odom = deque(maxlen=200)    # (t, x, y, z, yaw)
        self.last_cmd = None             # (vx, vy, face_yaw, wall_time)
        self.hold_reason = None
        self.health_level = 0
        self.gap0 = None
        self.frames = 0
        self.valid_fraction = None
        self.n_returns = 0

    def on_state(self, state, now):
        armed, off = bool(state.get("armed")), bool(state.get("offboard_active"))
        if (armed and not self._armed) or (off and not self._offboard):
            self.reset()
        self._armed, self._offboard = armed, off
        self.session_active = off
        odom, local = state.get("odom_enu"), state.get("local_enu")
        if off and odom and local:
            gap = (odom[0] - local[0], odom[1] - local[1])
            if self.gap0 is None:
                self.gap0 = gap
            moved = math.hypot(gap[0] - self.gap0[0], gap[1] - self.gap0[1])
            self.hold_reason = "ekf_gap" if moved > self.cfg.max_ekf_gap_m else None

    def on_health(self, level):
        """0 OK, 1 WARN, 2 ERROR (diagnostic_msgs convention)."""
        self.health_level = int(level)
        self.nav.p.v_max = self.cfg.v_max * (self.cfg.warn_speed_factor if self.health_level == 1 else 1.0)

    # ---- sensor inputs ---------------------------------------------------------------------
    def on_odometry(self, t, x, y, z, yaw):
        self.odom.append((t, x, y, z, yaw))
        if self.mode != "general" and self.session_active:
            self.nav._advance_route(x, y)          # waypoint progress needs only the position

    def on_pad(self, detection):
        """A detection with a ground position helps the planner head for the pad; the guide
        takes over once the pad is confirmed."""
        pos = detection.get("position_enu")
        if self.session_active and detection.get("detected") and pos and detection.get("source") != "hint":
            self.nav.set_pad_world(float(pos[0]), float(pos[1]))

    def _pose_at(self, t):
        if not self.odom:
            return None
        return min(self.odom, key=lambda o: abs(o[0] - t))

    def on_depth(self, stamp, now, depth_raw, intrinsics=None):
        if not self.session_active:
            return
        self.frames += 1
        if self.frames % max(1, self.cfg.depth_decimation):
            return
        pose = self._pose_at(stamp)
        if pose is None:
            return
        _, x, y, z, yaw = pose
        fx, fy, cx, cy = intrinsics or default_intrinsics(depth_raw.shape[1], depth_raw.shape[0])
        returns, self.valid_fraction = depth_to_scan(depth_raw, fx, fy, cx, cy, self.scan_params)
        self.n_returns = len(returns)
        (vx, vy), face = self.nav.step((x, y, yaw), returns, None, now)
        self.last_cmd = (vx, vy, face, now)

    # ---- output ----------------------------------------------------------------------------
    def _blocked(self):
        return (not self.session_active or not self.odom or self.hold_reason or self.health_level >= 2)

    def goal(self, now):
        """(x, y, z, yaw) for /course/goal, or None when it must not be published."""
        if self._blocked():
            return None
        _, x, y, _, yaw = self.odom[-1]
        z = self.cfg.flight_altitude_m
        if self.mode == "scripted":
            target = self.nav.hint if self.nav.hint is not None else self.nav.pad
            if target is None:
                return x, y, z, yaw
            dist = math.dist((x, y), target)
            if dist < 0.1:
                return x, y, z, math.atan2(target[1] - y, target[0] - x)
            d = min(dist, self.cfg.goal_max_m, max(self.cfg.goal_min_m, self.nav.p.v_max * self.cfg.goal_lookahead_s))
            return (x + (target[0] - x) / dist * d, y + (target[1] - y) / dist * d, z,
                    math.atan2(target[1] - y, target[0] - x))
        if self.last_cmd is None:
            return None
        vx, vy, face, t_cmd = self.last_cmd
        if now - t_cmd > self.cfg.max_depth_age_s:
            return None
        speed = math.hypot(vx, vy)
        if speed < 0.05:                       # turning in place or waiting: hold position, face the target
            return x, y, z, face
        d = min(self.cfg.goal_max_m, max(self.cfg.goal_min_m, speed * self.cfg.goal_lookahead_s))
        return x + vx / speed * d, y + vy / speed * d, z, face

    def pad_hint_detection(self, now):
        """COURSE-SPECIFIC and off by default. When the drone has flown the whole hinted route and is
        within pad_hint_radius_m of the hinted pad, announce it as a confirmed detection so the
        guide can approach and land. Marked source=hint; the detector's own messages are unaffected."""
        if (self.mode == "general" or not self.cfg.publish_pad_hint or self._blocked()
                or self.nav.route_i < len(self.nav.route) or self.nav.pad is None):
            return None
        _, x, y, _, _ = self.odom[-1]
        if math.dist((x, y), self.nav.pad) > self.cfg.pad_hint_radius_m:
            return None
        return {"stamp": now, "detected": True, "confidence": 0.0, "pixel": None,
                "position_enu": [self.nav.pad[0], self.nav.pad[1], 0.0], "confirmed": True,
                "streak": 0, "tf_exact": False, "source": "hint"}

    def debug(self):
        g = self.nav.grid
        return {
            "mode": self.mode, "status": self.nav.status, "session": self.session_active,
            "hold": self.hold_reason, "health": self.health_level, "resets": self.resets,
            "frames": self.frames, "returns": self.n_returns, "valid_fraction": self.valid_fraction,
            "plan_ms": round(1000 * self.nav.plan_time / max(1, self.nav.plan_calls), 2),
            "stuck_events": self.nav.stuck_events, "map_cells": [g.w, g.h],
            "pad_known": self.nav.pad is not None,
            "route": [self.nav.route_i, len(self.nav.route)],
        }


def warm_up(route_planner="astar"):
    """Compile the numba routines now (it can take seconds on the Jetson), not mid-flight.
    Returns True when the compiled search is available."""
    g = Grid(40, 40, 0.05, (0.0, 0.0))
    g.cells[:] = bytes([FREE]) * len(g.cells)
    for mode in {route_planner, "astar"}:
        theta_star(g, (0.3, 0.3), (1.5, 1.5), 0.1, mode=mode)
    from .planner_core import theta_star as ts
    return ts._search is not None
