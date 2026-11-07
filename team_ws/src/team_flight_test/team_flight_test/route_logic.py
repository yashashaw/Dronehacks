"""Flight-test logic, ported from the sim's best run (sim/waypoint_run_best_run015.py).
No ROS imports, so it runs and is tested on a laptop.

What was kept from run_015
  * leg by leg: turn in place to face the leg, then fly the straight line
  * the setpoint slides along the line at SPEED and never leads the drone by > 0.3 m
  * a leg is done when the drone is within arrive_m of its end point
What changed for the real drone
  * frame: VIO odom ENU (x forward / y left at VIO start), not PX4 NED; the route is
    body-relative to the pose at the moment takeoff_land hands control back (release)
  * no arming, no Offboard request, no direct /fmu setpoints: takeoff, land and
    Offboard go through team_takeoff_land + skybeta flight_interface
  * no depth brake: the real stack publishes no depth topic (loc_stack/facts/drone7)
  * a settle pause after each turn (sim: estimate errors clustered around turns)
  * watchdogs that land: stale odometry, odometry jump, track error, altitude band,
    distance from start, run timer. Small jumps are only logged; that is the data.

Frames: ENU, metres, yaw in radians counter-clockwise from +x (odom forward at VIO start).
"""
import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------- profiles
# Steps: ('goto', forward_m, left_m, face_leg) relative to the release pose,
#        ('yaw', relative_deg)  turn in place relative to the release heading,
#        ('hold', seconds)      hold position.
# forward/left are scaled by Config.scale; yaw and hold are not.
_COURSE_NED = [(1.64, 0.0), (1.64, 1.14), (0.07, 1.14), (0.07, 2.26), (1.64, 2.26), (1.64, 3.31)]
PROFILES = {
    # takeoff (done by takeoff_land), hold, land. Baseline drift with no motion.
    'hover': [('hold', 10.0)],
    # turn in place only: the sim's worst estimator jumps happened during turns
    'yaw': [('hold', 2.0), ('yaw', 90.0), ('hold', 3.0), ('yaw', 0.0), ('hold', 3.0),
            ('yaw', -90.0), ('hold', 3.0), ('yaw', 0.0), ('hold', 3.0)],
    # gentler turns first: +45, back, -45, back (fly before 'yaw')
    'yaw45': [('hold', 2.0), ('yaw', 45.0), ('hold', 3.0), ('yaw', 0.0), ('hold', 3.0),
              ('yaw', -45.0), ('hold', 3.0), ('yaw', 0.0), ('hold', 3.0)],
    # sideways 0.75 m left and back, heading held: facing a wall, the camera keeps it in
    # view the whole time, so depth measures the sideways drift directly
    'strafe': [('hold', 2.0), ('goto', 0.0, 0.75, False), ('hold', 3.0),
               ('goto', 0.0, 0.0, False), ('hold', 3.0)],
    # stage23: forward 1 m and back without turning (camera keeps facing forward)
    'out_back': [('hold', 2.0), ('goto', 1.0, 0.0, False), ('hold', 2.0),
                 ('goto', 0.0, 0.0, False), ('hold', 2.0)],
    # 1 m box turning to face each leg (to the right, like the course)
    'box': [('hold', 2.0), ('goto', 1.0, 0.0, True), ('goto', 1.0, -1.0, True),
            ('goto', 0.0, -1.0, True), ('goto', 0.0, 0.0, True), ('yaw', 0.0), ('hold', 2.0)],
    # run_015 route: start corridor north, then right (east) through the maze lanes.
    # NED offsets (north, east) from the launch mark become (forward, -left).
    'course': [('hold', 1.0)] + [('goto', n, -e, True) for n, e in _COURSE_NED],
}


@dataclass
class Config:
    profile: str = 'hover'
    scale: float = 1.0           # multiplies goto distances (0.5 = half-size box/course)
    max_legs: int = 0            # >0: stop after this many goto legs (course: walk it in)
    speed: float = 0.25          # m/s along a leg (run_015 value)
    lead_m: float = 0.3          # setpoint never more than this ahead of the drone
    yaw_rate_dps: float = 20.0   # sim: 25 deg/s turns caused jumps; slower here
    yaw_tol_deg: float = 5.0
    turn_timeout_s: float = 4.0
    settle_s: float = 1.0        # pause after each turn before translating
    arrive_m: float = 0.15       # sim used 0.10; VIO noise on hardware is larger
    leg_timeout_s: float = 25.0  # a leg that takes this long is skipped -> land
    max_run_s: float = 60.0      # from release; then land
    # watchdogs (any one -> land)
    odom_timeout_s: float = 0.5
    jump_log_m: float = 0.10     # odometry step bigger than this is logged as a jump
    jump_land_m: float = 0.50    # ... bigger than this lands
    max_track_err_m: float = 0.75
    geofence_m: float = 1.0      # beyond the route's own extent from the start
    min_alt_m: float = 0.4       # above the takeoff ground height
    max_alt_m: float = 2.0

    def steps(self):
        if self.profile not in PROFILES:
            raise ValueError('unknown profile %r (have: %s)' % (self.profile, ', '.join(PROFILES)))
        out, legs = [], 0
        for st in PROFILES[self.profile]:
            if st[0] == 'goto':
                if self.max_legs > 0 and legs >= self.max_legs:
                    break
                legs += 1
                out.append(('goto', st[1] * self.scale, st[2] * self.scale, st[3]))
            else:
                out.append(st)
        return out

    def check(self):
        if not 0.05 <= self.speed <= 0.5:
            raise ValueError('speed must be 0.05..0.5 m/s for a test flight')
        if not 0.1 <= self.scale <= 1.5:
            raise ValueError('scale must be 0.1..1.5')
        if not 5.0 <= self.yaw_rate_dps <= 45.0:
            raise ValueError('yaw_rate_dps must be 5..45')
        self.steps()


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class Out:
    setpoint: tuple = None       # (x, y, z, yaw) ENU, or None = publish nothing
    land: str = None             # reason, when the route wants to land
    events: list = field(default_factory=list)   # (level, text)


# phases
WAIT = 'wait'            # before release: publish nothing
HOLD = 'hold'
TURN = 'turn'
SETTLE = 'settle'
FLY = 'fly'
DONE = 'done'            # route over (or a watchdog fired): land requested
STOPPED = 'stopped'      # pilot took over: publish nothing ever again


class Route:
    """Call start() once at release, then step() every tick with the latest odometry."""

    def __init__(self, cfg):
        cfg.check()
        self.cfg = cfg
        self.steps = cfg.steps()
        self.phase = WAIT
        self.i = -1
        self.origin = None       # Pose at release
        self.ground_z = None
        self.sp = None           # [x, y, z, yaw]
        self.t_start = None
        self.t_phase = None
        self.leg = None          # dict for the active goto
        self.last_odom = None    # (t, Pose)
        self.max_step_m = 0.0
        self.jumps = 0
        self.extent = 0.0
        self.done_reason = None

    # ------------------------------------------------------------- helpers
    def world(self, fwd, left):
        o = self.origin
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        return (o.x + c * fwd - s * left, o.y + s * fwd + c * left)

    def _finish(self, reason, out, level='info'):
        if self.phase in (DONE, STOPPED):
            return
        self.phase = DONE
        self.done_reason = reason
        out.land = reason
        out.events.append((level, 'LAND: ' + reason))

    def _next(self, now, pose, out):
        self.i += 1
        self.t_phase = now
        if self.i >= len(self.steps):
            self._finish('route complete', out)
            return
        st = self.steps[self.i]
        if st[0] == 'hold':
            self.phase = HOLD
            out.events.append(('info', 'step %d: hold %.1f s' % (self.i + 1, st[1])))
        elif st[0] == 'yaw':
            self.phase = TURN
            yaw = wrap(self.origin.yaw + math.radians(st[1]))
            self.leg = {'yaw': yaw, 'then': None, 'turn_deg': abs(math.degrees(wrap(yaw - self.sp[3])))}
            out.events.append(('info', 'step %d: turn to %+.0f deg (relative to start)' % (self.i + 1, st[1])))
        else:
            _, f, l, face = st
            tx, ty = self.world(f, l)
            sx, sy = self.sp[0], self.sp[1]
            L = math.hypot(tx - sx, ty - sy)
            heading = math.atan2(ty - sy, tx - sx) if L > 1e-3 else self.sp[3]
            self.leg = {'start': (sx, sy), 'end': (tx, ty), 'L': L, 'heading': heading, 's': 0.0,
                        'yaw': heading if face else self.sp[3], 't0': now}
            out.events.append(('info', 'step %d: goto fwd %.2f left %.2f (%.2f m, heading %.0f deg, %s)' % (
                self.i + 1, f, l, L, math.degrees(heading), 'facing leg' if face else 'yaw held')))
            self.leg['turn_deg'] = abs(math.degrees(wrap(self.leg['yaw'] - self.sp[3])))
            if self.leg['turn_deg'] > 1.0:
                self.phase = TURN
                self.leg['then'] = FLY
            else:
                self.phase = FLY

    # ------------------------------------------------------------- public
    def start(self, now, pose, hover_z, ground_z):
        """Release: the drone is hovering; take the current pose as the route origin."""
        self.origin = Pose(pose.x, pose.y, pose.z, pose.yaw)
        self.ground_z = ground_z if ground_z is not None else hover_z - 1.0
        self.sp = [pose.x, pose.y, hover_z, pose.yaw]
        self.t_start = now
        self._t_tick = now
        pts = [(0.0, 0.0)] + [(s[1], s[2]) for s in self.steps if s[0] == 'goto']
        self.extent = max(math.hypot(f, l) for f, l in pts)
        out = Out()
        out.events.append(('info', 'route %s: %d steps, origin ENU (%.2f, %.2f, %.2f) yaw %.0f deg, '
                                   'extent %.2f m, hover z %.2f, ground z %.2f' % (
                                       self.cfg.profile, len(self.steps), pose.x, pose.y, pose.z,
                                       math.degrees(pose.yaw), self.extent, hover_z, self.ground_z)))
        self._next(now, pose, out)
        out.setpoint = tuple(self.sp)
        return out

    def observe(self, t, pose):
        """Feed every odometry message (not just once per tick) for jump detection."""
        ev = None
        if self.last_odom is not None:
            step = math.hypot(pose.x - self.last_odom[1].x, pose.y - self.last_odom[1].y)
            self.max_step_m = max(self.max_step_m, step)
            if step > self.cfg.jump_log_m:
                self.jumps += 1
                ev = step
        self.last_odom = (t, pose)
        return ev

    def pilot_took_over(self, out):
        if self.phase != STOPPED:
            self.phase = STOPPED
            out.events.append(('warn', 'pilot has the drone: route stopped, publishing nothing'))

    def step(self, now, pose, odom_t, jump_m=None):
        """One control tick. pose: latest odometry (or None); odom_t: its arrival time."""
        out = Out()
        if self.phase in (WAIT, STOPPED):
            return out
        if self.phase == DONE:
            out.setpoint = tuple(self.sp)      # keep holding until takeoff_land takes over
            return out
        c = self.cfg

        # ---- watchdogs
        if pose is None or odom_t is None or now - odom_t > c.odom_timeout_s:
            self._finish('odometry stale (> %.1f s)' % c.odom_timeout_s, out, 'error')
        elif jump_m is not None and jump_m > c.jump_land_m:
            self._finish('odometry jumped %.2f m in one message' % jump_m, out, 'error')
        elif now - self.t_start > c.max_run_s:
            self._finish('run timer %.0f s' % c.max_run_s, out, 'warn')
        else:
            dz = pose.z - self.ground_z
            track = math.hypot(pose.x - self.sp[0], pose.y - self.sp[1])
            dist = math.hypot(pose.x - self.origin.x, pose.y - self.origin.y)
            if dz < c.min_alt_m or dz > c.max_alt_m:
                self._finish('altitude %.2f m outside %.1f..%.1f' % (dz, c.min_alt_m, c.max_alt_m), out, 'error')
            elif track > c.max_track_err_m:
                self._finish('track error %.2f m > %.2f' % (track, c.max_track_err_m), out, 'error')
            elif dist > self.extent + c.geofence_m:
                self._finish('%.2f m from start > route extent %.2f + %.2f' % (
                    dist, self.extent, c.geofence_m), out, 'error')
        if self.phase == DONE:
            out.setpoint = tuple(self.sp)
            return out

        # ---- route
        if self.phase == HOLD:
            if now - self.t_phase >= self.steps[self.i][1]:
                self._next(now, pose, out)
        elif self.phase == TURN:
            target = self.leg['yaw']
            err = wrap(target - self.sp[3])
            dmax = math.radians(c.yaw_rate_dps) * max(0.0, now - getattr(self, '_t_tick', now))
            self.sp[3] = wrap(self.sp[3] + max(-dmax, min(dmax, err)))
            if abs(wrap(target - self.sp[3])) < 1e-3:
                ok = abs(wrap(pose.yaw - target)) < math.radians(c.yaw_tol_deg)
                if ok or now - self.t_phase > c.turn_timeout_s + self.leg['turn_deg'] / c.yaw_rate_dps:
                    if not ok:
                        out.events.append(('warn', 'turn: heading still %.0f deg off after timeout' % math.degrees(
                            wrap(pose.yaw - target))))
                    self.phase = SETTLE
                    self.t_phase = now
        elif self.phase == SETTLE:
            if now - self.t_phase >= c.settle_s:
                if self.leg.get('then') == FLY:
                    self.phase = FLY
                    self.leg['t0'] = now
                    self.t_phase = now
                else:
                    self._next(now, pose, out)
        elif self.phase == FLY:
            lg = self.leg
            (sx, sy), (tx, ty), h = lg['start'], lg['end'], lg['heading']
            along = (pose.x - sx) * math.cos(h) + (pose.y - sy) * math.sin(h)
            dt = max(0.0, now - getattr(self, '_t_tick', now))
            if lg['s'] < lg['L'] and lg['s'] - along < c.lead_m:
                lg['s'] = min(lg['L'], lg['s'] + c.speed * dt)
            self.sp[0] = sx + math.cos(h) * lg['s']
            self.sp[1] = sy + math.sin(h) * lg['s']
            if lg['s'] >= lg['L'] and math.hypot(pose.x - tx, pose.y - ty) < c.arrive_m:
                out.events.append(('info', 'step %d reached in %.1f s, miss %.2f m' % (
                    self.i + 1, now - lg['t0'], math.hypot(pose.x - tx, pose.y - ty))))
                self._next(now, pose, out)
            elif now - lg['t0'] > c.leg_timeout_s:
                self._finish('leg %d not reached in %.0f s (along %.2f of %.2f m)' % (
                    self.i + 1, c.leg_timeout_s, along, lg['L']), out, 'error')
        self._t_tick = now
        out.setpoint = tuple(self.sp)
        return out


# ------------------------------------------------------------ mission sequencing
# What to ask team_takeoff_land for, from its JSON state. Kept here so it is tested.
M_WAIT_OFFBOARD = 'wait_offboard'
M_TAKEOFF = 'takeoff'          # /team/takeoff accepted, waiting for mode hover
M_RELEASE = 'release'          # /team/release accepted, waiting for passthrough
M_ROUTE = 'route'
M_LANDING = 'landing'
M_DONE = 'done'
M_ABORTED = 'aborted'


def next_call(mphase, state):
    """Which service to call now ('takeoff'/'release'/'land'/None) for this phase."""
    if state is None:
        return None
    mode = state.get('mode')
    if mphase == M_WAIT_OFFBOARD:
        return 'takeoff' if (state.get('offboard') and mode == 'passthrough') else None
    if mphase == M_TAKEOFF:
        return 'release' if mode == 'hover' else None
    return None
