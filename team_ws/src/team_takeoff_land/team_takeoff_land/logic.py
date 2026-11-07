"""Takeoff / land state machine, with no ROS imports so it tests anywhere.

The node in ``takeoff_land_node.py`` feeds this a snapshot of the vehicle
(``Inputs``) every tick and does whatever the returned ``Output`` says.

Modes
-----
PASSTHROUGH  planner setpoints are forwarded to the flight interface.
TAKEOFF      planner is cut off; climb straight up to ``takeoff_altitude_m``
             above the ground under the vehicle.
HOVER        planner still cut off; hold the takeoff point until ``release``.
LANDING      planner cut off for good; PX4 Land mode is requested, PX4
             descends and disarms. A disarm is sent as a fallback.
LANDED       terminal: on the ground and disarmed. Planner stays cut off
             until ``release`` (needed only for the return bonus).
ABORTED      the pilot took the vehicle back (Offboard or Land mode lost).
             Nothing is sent. Resets to PASSTHROUGH once disarmed.

Once LANDING starts, the planner is never forwarded again in that flight.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

PASSTHROUGH = 'passthrough'
TAKEOFF = 'takeoff'
HOVER = 'hover'
LANDING = 'landing'
LANDED = 'landed'
ABORTED = 'aborted'

Vec3 = Tuple[float, float, float]


@dataclass
class Config:
    takeoff_altitude_m: float = 1.0
    arrival_radius_m: float = 0.15
    settle_s: float = 1.0
    takeoff_timeout_s: float = 20.0
    release_after_takeoff: bool = False
    command_retry_s: float = 1.0
    land_retries: int = 5
    disarm_after_landed_s: float = 3.0


@dataclass
class Inputs:
    now: float
    position: Optional[Vec3]   # ENU odom position, None until odometry
    link: bool                 # vehicle_status is arriving
    armed: bool
    landed: Optional[bool]     # None until PX4 reports it
    offboard: bool             # nav_state is Offboard
    land_engaged: bool         # nav_state is AUTO_LAND or DESCEND


@dataclass
class Output:
    forward_planner: bool = False
    setpoint: Optional[Vec3] = None   # ENU pose to publish instead
    send_land: bool = False
    send_disarm: bool = False
    logs: List[Tuple[str, str]] = field(default_factory=list)  # (level, text)


class TakeoffLand:

    def __init__(self, config=None):
        self.cfg = config or Config()
        self.mode = PASSTHROUGH
        self.target: Optional[Vec3] = None
        self.ground_z: Optional[float] = None
        self._takeoff_started = None
        self._settle_started = None
        self._land_sent_at = None
        self._land_attempts = 0
        self._land_confirmed = False
        self._land_failed_logged = False
        self._landed_since = None
        self._disarm_sent_at = None

    # ------------------------------------------------------------------
    # requests (from the services). Each returns (accepted, message).
    # ------------------------------------------------------------------
    def request_takeoff(self, s: Inputs):
        if self.mode != PASSTHROUGH:
            return False, f'refused: mode is {self.mode}, not passthrough'
        if not s.link:
            return False, 'refused: no PX4 link'
        if s.position is None:
            return False, 'refused: no odometry yet'
        if not s.armed:
            return False, 'refused: not armed (the safety pilot arms)'
        if not s.offboard:
            return False, ('refused: not in Offboard. The pilot arms, then '
                           'switches to Position; the flight interface then '
                           'enters Offboard')
        ground = self.ground_z if self.ground_z is not None else s.position[2]
        self.target = (s.position[0], s.position[1],
                       ground + self.cfg.takeoff_altitude_m)
        self.mode = TAKEOFF
        self._takeoff_started = s.now
        self._settle_started = None
        return True, (f'taking off to z={self.target[2]:.2f} m '
                      f'({self.cfg.takeoff_altitude_m:.2f} m above ground); '
                      'planner cut off')

    def request_land(self, s: Inputs):
        if self.mode == LANDED:
            return False, 'refused: already landed'
        if self.mode == LANDING:
            return True, 'already landing'
        if self.mode == ABORTED:
            return False, 'refused: the pilot has the vehicle'
        if not s.link:
            return False, 'refused: no PX4 link'
        if not s.armed:
            if s.landed:
                self.mode = LANDED
                return True, 'already on the ground and disarmed'
            return False, 'refused: not armed'
        self.mode = LANDING
        hold = s.position if s.position is not None else self.target
        self.target = hold
        self._land_sent_at = None
        self._land_attempts = 0
        self._land_confirmed = False
        self._land_failed_logged = False
        self._landed_since = None
        self._disarm_sent_at = None
        return True, 'landing: PX4 Land mode requested; planner cut off'

    def request_release(self, s: Inputs):
        if self.mode == HOVER:
            self.mode = PASSTHROUGH
            return True, 'released: planner setpoints forwarded again'
        if self.mode in (LANDED, ABORTED) and not s.armed:
            self.mode = PASSTHROUGH
            return True, 'released: planner setpoints forwarded again'
        if self.mode == PASSTHROUGH:
            return True, 'already passthrough'
        return False, f'refused: cannot release during {self.mode}'

    # ------------------------------------------------------------------
    # the periodic step
    # ------------------------------------------------------------------
    def step(self, s: Inputs) -> Output:
        out = Output()
        if s.landed and s.position is not None:
            self.ground_z = s.position[2]
        handler = getattr(self, f'_step_{self.mode}')
        handler(s, out)
        return out

    def _abort(self, out, why):
        self.mode = ABORTED
        out.logs.append(('warn', f'aborted: {why}. The pilot has the vehicle; '
                                 'nothing more is sent until disarm.'))

    def _step_passthrough(self, s, out):
        out.forward_planner = True

    def _step_takeoff(self, s, out):
        if not s.offboard:
            self._abort(out, 'Offboard lost during takeoff')
            return
        out.setpoint = self.target
        if s.position is not None and (
                math.dist(s.position, self.target)
                <= self.cfg.arrival_radius_m):
            if self._settle_started is None:
                self._settle_started = s.now
            elif s.now - self._settle_started >= self.cfg.settle_s:
                self._finish_takeoff(out, 'reached takeoff altitude')
                return
        else:
            self._settle_started = None
        if s.now - self._takeoff_started > self.cfg.takeoff_timeout_s:
            self._finish_takeoff(
                out, f'takeoff not settled within '
                     f'{self.cfg.takeoff_timeout_s:.0f} s; holding target')

    def _finish_takeoff(self, out, why):
        if self.cfg.release_after_takeoff:
            self.mode = PASSTHROUGH
            out.logs.append(('info', f'{why}; handing back to the planner'))
        else:
            self.mode = HOVER
            out.logs.append(('info', f'{why}; hovering until release'))

    def _step_hover(self, s, out):
        if not s.offboard:
            self._abort(out, 'Offboard lost while hovering')
            return
        out.setpoint = self.target

    def _step_landing(self, s, out):
        if not s.armed and s.landed:
            self.mode = LANDED
            out.logs.append(('info', 'landed and disarmed'))
            return
        # Keep holding in case PX4 refuses Land and stays in Offboard; PX4
        # ignores these setpoints once it is in Land mode.
        out.setpoint = self.target
        if s.land_engaged:
            if not self._land_confirmed:
                out.logs.append(('info', 'PX4 is in Land mode'))
            self._land_confirmed = True
        elif self._land_confirmed and s.armed and not s.landed:
            # Was landing, now some other mode in the air: the pilot took over.
            self._abort(out, 'Land mode left while airborne')
            return
        if not self._land_confirmed and not s.landed:
            due = (self._land_sent_at is None
                   or s.now - self._land_sent_at >= self.cfg.command_retry_s)
            if due and self._land_attempts < self.cfg.land_retries:
                out.send_land = True
                self._land_attempts += 1
                self._land_sent_at = s.now
            elif (self._land_attempts >= self.cfg.land_retries
                  and not self._land_failed_logged and due):
                self._land_failed_logged = True
                out.logs.append((
                    'error',
                    f'PX4 did not enter Land after {self._land_attempts} '
                    'requests; still holding. Check target_system and '
                    'pre-flight state.'))
        # Disarm fallback: PX4 normally disarms itself (COM_DISARM_LAND).
        if s.landed and s.armed:
            if self._landed_since is None:
                self._landed_since = s.now
            elif (s.now - self._landed_since >= self.cfg.disarm_after_landed_s
                  and (self._disarm_sent_at is None
                       or s.now - self._disarm_sent_at
                       >= self.cfg.command_retry_s)):
                out.send_disarm = True
                self._disarm_sent_at = s.now
        else:
            self._landed_since = None

    def _step_landed(self, s, out):
        pass

    def _step_aborted(self, s, out):
        if not s.armed:
            self.mode = PASSTHROUGH
            out.logs.append(('info', 'disarmed after abort; passthrough'))

    def status(self):
        return {
            'mode': self.mode,
            'target_enu': list(self.target) if self.target else None,
            'ground_z': self.ground_z,
            'land_attempts': self._land_attempts,
            'land_confirmed': self._land_confirmed,
        }
