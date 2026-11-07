"""The pad guide's stage logic. No ROS: ``pad_guide.py`` feeds it and publishes
what it returns, and ``test/test_guide_machine.py`` runs it without a drone.

Stages
------
HOLD      before Offboard: command the current position (this stream is what
          lets the flight interface enter Offboard at all)
TAKEOFF   Offboard just started: remember home, climb to cruise altitude
FOLLOW    step toward the newest fresh ``/course/goal``; hold if there is none
APPROACH  a pad is confirmed: ignore the goal, step over the pad
LAND      over the pad: lower the pose toward the ground and hold at the
          flight interface's altitude floor (it has no land command)
DONE      on the ground and disarmed after Offboard ran: hold

Everything is ENU ``odom`` (x, y, z) tuples in metres. Times are seconds on one
monotonic-enough clock supplied by the caller.
"""
import enum
from dataclasses import dataclass

from course_pipeline.pad_math import (
    blend, horizontal_distance, is_fresh, step_toward)


class Stage(str, enum.Enum):
    HOLD = 'HOLD'
    TAKEOFF = 'TAKEOFF'
    FOLLOW = 'FOLLOW'
    APPROACH = 'APPROACH'
    LAND = 'LAND'
    DONE = 'DONE'


@dataclass
class GuideConfig:
    cruise_altitude_m: float = 1.5      # climb height above the start pose
    step_m: float = 0.5                 # max distance of a published pose from the vehicle
    arrival_radius_m: float = 0.3       # "reached cruise altitude"
    goal_timeout_s: float = 0.5         # a /course/goal older than this is stale
    detection_timeout_s: float = 0.5    # a confirmed detection older than this is stale
    approach_altitude_m: float = 1.0    # height above the pad while aligning
    align_radius_m: float = 0.25        # horizontal error that counts as over the pad
    align_hold_s: float = 3.0           # how long it must stay that close before landing
    abort_radius_m: float = 0.6         # drifting this far while landing goes back to APPROACH
    descent_rate_mps: float = 0.15      # how fast the landing pose is lowered
    pad_smoothing: float = 0.3          # weight of each new pad sample (0..1)
    home_exclusion_radius_m: float = 1.0  # pads this close to home are the home pad


class GuideMachine:
    def __init__(self, config=None):
        self.cfg = config or GuideConfig()
        self.stage = Stage.HOLD

        self.position = None
        self.offboard_active = False
        self.armed = None
        self.landed = None

        self.home = None
        self.target = None      # what the guide is currently commanding
        self.pad = None         # best estimate of the pad, odom

        self._session_started = False
        self._paused_logged = False
        self._goal = None
        self._goal_at = None
        self._candidate = None
        self._candidate_at = None
        self._candidate_blended_at = None
        self._align_since = None
        self._land_z = None
        self._floor_logged = False
        self._home_pad_logged = False
        self._last_now = None
        self._events = []

    # -- inputs --------------------------------------------------------
    def on_position(self, xyz):
        self.position = tuple(float(v) for v in xyz)

    def on_state(self, offboard_active, armed, landed):
        self.offboard_active = bool(offboard_active)
        self.armed = armed
        self.landed = landed

    def on_goal(self, xyz, now):
        self._goal = tuple(float(v) for v in xyz)
        self._goal_at = now

    def on_detection(self, confirmed, position, now):
        """Only a confirmed detection with a ground position is kept."""
        if confirmed and position is not None:
            self._candidate = tuple(float(v) for v in position)
            self._candidate_at = now

    def pop_events(self):
        events, self._events = self._events, []
        return events

    # -- helpers -------------------------------------------------------
    def _log(self, level, message):
        self._events.append((level, message))

    def _set_stage(self, stage, message):
        self.stage = stage
        self._log('info', f'{stage.value}: {message}')

    def _fresh_goal(self, now):
        if is_fresh(self._goal_at, now, self.cfg.goal_timeout_s):
            return self._goal
        return None

    def _fresh_candidate(self, now):
        if is_fresh(self._candidate_at, now, self.cfg.detection_timeout_s):
            return self._candidate
        return None

    def _update_pad(self, now):
        """Blend a new confirmed sample into the pad estimate, once per sample."""
        candidate = self._fresh_candidate(now)
        if candidate is None or self._candidate_at == self._candidate_blended_at:
            return
        self._candidate_blended_at = self._candidate_at
        self.pad = blend(self.pad, candidate, self.cfg.pad_smoothing)

    # -- the tick ------------------------------------------------------
    def tick(self, now):
        """Return the ENU position to command this tick, or None if no odometry."""
        dt = 0.1 if self._last_now is None else min(max(now - self._last_now, 0.0), 0.5)
        self._last_now = now
        if self.position is None:
            return None
        pos = self.position

        # The pilot landing and disarming ends the session, in any stage.
        if (self._session_started and self.stage is not Stage.DONE
                and self.armed is False and self.landed is True):
            self._set_stage(Stage.DONE, 'on the ground and disarmed; holding. '
                                        'Return to base is not implemented.')

        if self.offboard_active and not self._session_started:
            self._session_started = True
            self.home = pos
            self.target = pos
            self._set_stage(
                Stage.TAKEOFF,
                f'Offboard active; home is ENU ({pos[0]:.2f}, {pos[1]:.2f}, '
                f'{pos[2]:.2f}); climbing {self.cfg.cruise_altitude_m:.2f} m.')

        if self.stage in (Stage.HOLD, Stage.DONE):
            self.target = pos
            return self.target

        # Offboard lost (pilot took over): stop advancing, keep the last target.
        if not self.offboard_active:
            if not self._paused_logged:
                self._paused_logged = True
                self._log('warn', 'Offboard lost; stage paused. It does not return '
                                  'until the vehicle is disarmed and armed again.')
            return self.target if self.target is not None else pos
        self._paused_logged = False

        handler = {
            Stage.TAKEOFF: self._takeoff,
            Stage.FOLLOW: self._follow,
            Stage.APPROACH: self._approach,
            Stage.LAND: self._land,
        }[self.stage]
        handler(now, dt, pos)
        return self.target

    # -- stages --------------------------------------------------------
    def _takeoff(self, now, dt, pos):
        cruise = (self.home[0], self.home[1],
                  self.home[2] + self.cfg.cruise_altitude_m)
        self.target = step_toward(pos, cruise, self.cfg.step_m)
        if pos[2] >= cruise[2] - self.cfg.arrival_radius_m:
            self._set_stage(Stage.FOLLOW, 'at cruise altitude; following /course/goal.')

    def _follow(self, now, dt, pos):
        candidate = self._fresh_candidate(now)
        if candidate is not None:
            if horizontal_distance(candidate, self.home) > self.cfg.home_exclusion_radius_m:
                self.pad = candidate
                self._candidate_blended_at = self._candidate_at
                self._align_since = None
                self._set_stage(
                    Stage.APPROACH,
                    f'pad confirmed at ENU ({candidate[0]:.2f}, {candidate[1]:.2f}); '
                    'ignoring /course/goal.')
                self._approach(now, dt, pos)
                return
            if not self._home_pad_logged:
                self._home_pad_logged = True
                self._log('info', 'confirmed pad is within the home exclusion '
                                  'radius of home; ignoring it.')
        goal = self._fresh_goal(now)
        if goal is not None:
            self.target = step_toward(pos, goal, self.cfg.step_m)
        # No fresh goal: keep the last target (hold).

    def _approach(self, now, dt, pos):
        self._update_pad(now)
        approach_z = self.pad[2] + self.cfg.approach_altitude_m
        self.target = step_toward(
            pos, (self.pad[0], self.pad[1], approach_z), self.cfg.step_m)
        error = horizontal_distance(pos, self.pad)
        if error > self.cfg.align_radius_m:
            self._align_since = None
            return
        if self._align_since is None:
            self._align_since = now
        elif now - self._align_since >= self.cfg.align_hold_s:
            self._land_z = pos[2]
            self._floor_logged = False
            self._set_stage(
                Stage.LAND,
                f'over the pad (error {error:.2f} m); lowering the pose.')

    def _land(self, now, dt, pos):
        self._update_pad(now)
        error = horizontal_distance(pos, self.pad)
        if error > self.cfg.abort_radius_m:
            self._align_since = None
            self._set_stage(
                Stage.APPROACH,
                f'drifted {error:.2f} m off the pad; realigning.')
            self._approach(now, dt, pos)
            return
        self._land_z = max(self.pad[2], self._land_z - self.cfg.descent_rate_mps * dt)
        self.target = (self.pad[0], self.pad[1], self._land_z)
        if self._land_z <= self.pad[2] and not self._floor_logged:
            self._floor_logged = True
            self._log('info', 'LAND: target is at ground level, which the flight '
                              'interface floors at 0.3 m. Holding over the pad '
                              'for the pilot to land.')
