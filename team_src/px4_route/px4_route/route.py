"""Route geometry for px4_route. No ROS in here, so every rule is unit-tested on a laptop.

A route file lists waypoints in the START frame:
    x = forward (the way the nose points when Offboard starts), y = left, z = up,
    metres, measured from the takeoff point (z = 0 is the floor under the drone).
The mission turns that into PX4's local frame (ENU) at takeoff and follows it with a
"carrot": a setpoint that slides along the path at the planned speed, never too far ahead
of the drone, slowing for corners. PX4 holds whatever point it is given, so the carrot,
not PX4, decides how fast the drone moves.
"""
import math
from dataclasses import dataclass, field

import yaml

# Course facts (README-course-dimensions): walls are 68 in tall, the net 10 ft.
WALL_TOP_M = 1.73
NET_HEIGHT_M = 3.05
HARD_MAX_HEIGHT_M = 2.6     # never let a route ask for more, whatever it says
HARD_MAX_SPEED = 2.0        # m/s; a typo like speed: 10 is refused, not flown
MIN_HEIGHT_M = 0.3          # flight_interface floors pose setpoints here anyway

DEFAULTS = {
    'name': 'route',
    'takeoff_height': 0.8,    # climb straight up to this before the first waypoint
    'speed': 0.3,             # cruise speed along the path, m/s
    'corner_speed': 0.2,      # speed through turns sharper than corner_angle_deg
    'corner_angle_deg': 30.0,
    'corner_radius': 0.5,     # slow down within this distance of a sharp turn (and of the end)
    'climb_speed': 0.3,       # max vertical speed, m/s
    'max_height': 1.5,        # route refused if any point is above this; flight lands above +0.3
    'max_cross_track': 0.5,   # land if the drone is further than this from the path
    'yaw': 'fixed',           # fixed: keep the takeoff heading. path: face the direction of travel
    'yaw_rate_dps': 30.0,     # yaw: path turns no faster than this
    'yaw_align_deg': 20.0,    # yaw: path -- the target waits until the nose is within this of the
                              # way ahead (turn first, then move; VIO is camera-only and turns hurt it)
    'yaw_lookahead': 0.5,     # yaw: path -- face along the path this far ahead (m), so short
                              # walk legs do not make the nose jitter
    'end': 'land',            # land: land at the last waypoint. hold: hover there (pilot lands)
    'max_time_s': 170.0,      # land after this long, whatever happens (run limit is 3 min)
    # Return to base (rule 7.10), OFF unless set: a second route file flown after this one has
    # landed and PX4 has disarmed: the node re-arms, takes off and flies it. Its waypoints are in
    # THIS route's start frame (metres from this route's takeoff point), so [0, 0, z] is the X.
    # A relative name is looked up next to this file. '' = off.
    'return_route': '',
}
TEXT_KEYS = ('name', 'yaw', 'end', 'return_route')


class RouteError(ValueError):
    """The route file is unusable; nothing will fly."""


@dataclass
class Route:
    name: str
    points: list            # start frame, includes the takeoff ground point and the climb
    speeds: list            # speed on the leg TO points[i] (speeds[0] unused)
    cfg: dict
    warnings: list = field(default_factory=list)


def _num(v, what):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise RouteError(f'{what} must be a finite number, got {v!r}')
    return float(v)


def validate_route(raw):
    """Dict (from YAML) -> Route. Raises RouteError with a readable reason."""
    if not isinstance(raw, dict):
        raise RouteError('route file must be a mapping with a "waypoints" list')
    unknown = set(raw) - set(DEFAULTS) - {'waypoints'}
    if unknown:
        raise RouteError(f'unknown keys {sorted(unknown)} (typo?); known: {sorted(DEFAULTS)} + waypoints')
    cfg = dict(DEFAULTS)
    cfg.update({k: v for k, v in raw.items() if k != 'waypoints'})
    for k in DEFAULTS:
        if k not in TEXT_KEYS:
            cfg[k] = _num(cfg[k], k)
    if cfg['return_route'] is None:
        cfg['return_route'] = ''
    if not isinstance(cfg['return_route'], str):
        raise RouteError('return_route must be a file name (or empty for no return)')
    if cfg['yaw'] not in ('fixed', 'path'):
        raise RouteError('yaw must be "fixed" or "path"')
    if cfg['end'] not in ('land', 'hold'):
        raise RouteError('end must be "land" or "hold"')
    for k in ('speed', 'corner_speed', 'climb_speed'):
        if not 0.05 <= cfg[k] <= HARD_MAX_SPEED:
            raise RouteError(f'{k} {cfg[k]} must be within 0.05..{HARD_MAX_SPEED} m/s')
    if not MIN_HEIGHT_M <= cfg['max_height'] <= HARD_MAX_HEIGHT_M:
        raise RouteError(f'max_height must be within {MIN_HEIGHT_M}..{HARD_MAX_HEIGHT_M} m')
    if not MIN_HEIGHT_M <= cfg['takeoff_height'] <= cfg['max_height']:
        raise RouteError(f'takeoff_height must be within {MIN_HEIGHT_M}..max_height')
    if cfg['max_cross_track'] <= 0.05 or cfg['corner_radius'] < 0 or cfg['max_time_s'] <= 0:
        raise RouteError('max_cross_track, corner_radius and max_time_s must be positive')

    wps = raw.get('waypoints')
    if not isinstance(wps, list) or not wps:
        raise RouteError('"waypoints" must be a non-empty list of [x, y, z] or [x, y, z, speed]')
    points = [(0.0, 0.0, 0.0), (0.0, 0.0, cfg['takeoff_height'])]
    speeds = [0.0, cfg['climb_speed']]
    for i, wp in enumerate(wps):
        if not isinstance(wp, (list, tuple)) or len(wp) not in (3, 4):
            raise RouteError(f'waypoint {i + 1}: expected [x, y, z] or [x, y, z, speed], got {wp!r}')
        x, y, z = (_num(v, f'waypoint {i + 1}') for v in wp[:3])
        if not MIN_HEIGHT_M <= z <= cfg['max_height']:
            raise RouteError(f'waypoint {i + 1}: z {z} outside {MIN_HEIGHT_M}..max_height {cfg["max_height"]}')
        v = _num(wp[3], f'waypoint {i + 1} speed') if len(wp) == 4 else cfg['speed']
        if not 0.05 <= v <= HARD_MAX_SPEED:
            raise RouteError(f'waypoint {i + 1}: speed {v} must be within 0.05..{HARD_MAX_SPEED}')
        points.append((x, y, z))
        speeds.append(v)

    # vertical legs never faster than climb_speed (vertically)
    for i in range(1, len(points)):
        a, b = points[i - 1], points[i]
        length = math.dist(a, b)
        dz = abs(b[2] - a[2])
        if dz > 1e-6 and length > 1e-6:
            speeds[i] = min(speeds[i], cfg['climb_speed'] * length / dz)

    warnings = []
    if cfg['max_height'] > WALL_TOP_M - 0.1:
        warnings.append(f'max_height {cfg["max_height"]} m: the drone may fly with its whole frame above the '
                        f'{WALL_TOP_M} m walls. Allowed only on the return; on the run it costs 85 % of a '
                        'checkpoint. Keep the run below ~1.5 m.')
    tooclose = [i + 1 for i in range(2, len(points) - 1) if math.dist(points[i], points[i + 1]) < 0.05]
    if tooclose:
        warnings.append(f'waypoints {tooclose} are within 5 cm of the next one (harmless, but check the file)')
    return Route(cfg['name'], points, speeds, cfg, warnings)


def load_route(path):
    try:
        with open(path) as f:
            raw = yaml.safe_load(f)
    except OSError as e:
        raise RouteError(f'cannot read route file {path}: {e}') from e
    except yaml.YAMLError as e:
        raise RouteError(f'route file {path} is not valid YAML: {e}') from e
    return validate_route(raw)


def load_return_route(main_path, main_route):
    """The return route named by main_route.cfg['return_route'], or None when the return is off.
    Raises RouteError if it is set but unusable (nothing should fly on a half-configured return)."""
    import os
    name = main_route.cfg.get('return_route') or ''
    if not name:
        return None
    path = name if os.path.isabs(name) else os.path.join(os.path.dirname(os.path.abspath(main_path)), name)
    ret = load_route(path)
    if ret.cfg['return_route']:
        raise RouteError(f'return route {path} has its own return_route: not allowed (one return only)')
    ret.path = path
    return ret


def start_frame_to_enu(p, start_enu, heading_ned):
    """Start-frame point (forward, left, up from takeoff) -> PX4 local ENU (East, North, Up).
    heading_ned: PX4 heading at takeoff, radians from North towards East (the nose)."""
    fx, fy = math.sin(heading_ned), math.cos(heading_ned)   # nose, in ENU
    lx, ly = -fy, fx                                        # left of the nose, in ENU
    x, y, z = p
    return (start_enu[0] + x * fx + y * lx,
            start_enu[1] + x * fy + y * ly,
            start_enu[2] + z)


def _unit(a, b):
    d = [b[i] - a[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    return [c / n for c in d] if n > 1e-9 else [0.0, 0.0, 0.0]


class Path:
    """Polyline with a speed profile: leg speeds, slowed near sharp turns and the end."""

    def __init__(self, points, speeds, corner_speed, corner_angle_deg, corner_radius):
        self.p = [tuple(map(float, q)) for q in points]
        self.leg_speed = list(speeds)
        self.cum = [0.0]
        for i in range(1, len(self.p)):
            self.cum.append(self.cum[-1] + math.dist(self.p[i - 1], self.p[i]))
        self.length = self.cum[-1]
        self.corner_speed = corner_speed
        self.corner_radius = corner_radius
        self.slow_at = []           # arc lengths of sharp turns (and the end)
        for k in range(1, len(self.p) - 1):
            u, w = _unit(self.p[k - 1], self.p[k]), _unit(self.p[k], self.p[k + 1])
            if not any(u) or not any(w):
                continue
            cos = max(-1.0, min(1.0, sum(u[i] * w[i] for i in range(3))))
            if math.degrees(math.acos(cos)) > corner_angle_deg:
                self.slow_at.append(self.cum[k])
        self.slow_at.append(self.length)

    def _leg(self, s):
        s = min(max(s, 0.0), self.length)
        for i in range(1, len(self.p)):
            if s <= self.cum[i] or i == len(self.p) - 1:
                return i, s
        return len(self.p) - 1, s

    def point_at(self, s):
        i, s = self._leg(s)
        a, b = self.p[i - 1], self.p[i]
        seg = self.cum[i] - self.cum[i - 1]
        f = 0.0 if seg < 1e-9 else (s - self.cum[i - 1]) / seg
        return tuple(a[j] + f * (b[j] - a[j]) for j in range(3))

    def direction_at(self, s):
        i, _ = self._leg(s)
        return _unit(self.p[i - 1], self.p[i])

    def heading_ahead(self, s, look=0.5, min_horizontal=0.15):
        """ENU yaw (rad) of the horizontal direction from point_at(s) to the first point at least
        `look` m further along the path that is min_horizontal m away sideways, or None if the
        rest of the path is (nearly) vertical, e.g. the takeoff climb or the final descent."""
        a = self.point_at(s)
        step = 0.05
        t = s + look
        while t <= self.length + step:
            b = self.point_at(min(t, self.length))
            dx, dy = b[0] - a[0], b[1] - a[1]
            if math.hypot(dx, dy) >= min_horizontal:
                return math.atan2(dy, dx)
            if t >= self.length:
                break
            t += step
        return None

    def speed_at(self, s):
        i, s = self._leg(s)
        v = self.leg_speed[i]
        if any(abs(s - c) < self.corner_radius for c in self.slow_at):
            v = min(v, self.corner_speed)
        return v

    def nearest(self, q, s_lo=0.0, s_hi=None):
        """(arc length, distance) of the closest path point to q, searching only s_lo..s_hi so a
        path that loops back near itself is not matched to the wrong part."""
        s_hi = self.length if s_hi is None else min(s_hi, self.length)
        best = (s_lo, math.dist(q, self.point_at(s_lo)))
        for i in range(1, len(self.p)):
            lo, hi = max(self.cum[i - 1], s_lo), min(self.cum[i], s_hi)
            if lo > hi:
                continue
            a, b = self.p[i - 1], self.p[i]
            seg = self.cum[i] - self.cum[i - 1]
            if seg < 1e-9:
                t = 0.0
            else:
                t = sum((q[j] - a[j]) * (b[j] - a[j]) for j in range(3)) / (seg * seg)
            s = min(max(self.cum[i - 1] + t * seg, lo), hi)
            d = math.dist(q, self.point_at(s))
            if d < best[1]:
                best = (s, d)
        return best

    def eta_s(self, step=0.05):
        """Planned time to fly the path at the profile speeds (no waiting for the drone)."""
        t, s = 0.0, 0.0
        while s < self.length:
            v = self.speed_at(s)
            t += step / v
            s += step
        return t


class Carrot:
    """The setpoint. It moves along the path at the profile speed, but only while the drone keeps
    up: PX4 flying to a pure position setpoint lags about speed / MPC_XY_P behind it, so the
    carrot may lead by lead_base + v / kp and then waits."""

    def __init__(self, path, lead_base=0.3, kp=0.95):
        self.path = path
        self.lead_base = lead_base
        self.kp = kp
        self.s = 0.0

    def max_lead(self):
        return self.lead_base + self.path.speed_at(self.s) / self.kp

    def step(self, dt, drone):
        dt = min(max(dt, 0.0), 0.2)
        if math.dist(self.path.point_at(self.s), drone) < self.max_lead():
            self.s = min(self.path.length, self.s + self.path.speed_at(self.s) * dt)
        return self.path.point_at(self.s)

    def at_end(self):
        return self.s >= self.path.length - 1e-6
