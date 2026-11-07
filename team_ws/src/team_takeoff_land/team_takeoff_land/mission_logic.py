"""Test mission: take off, then land straight away. No ROS imports.

Reads the JSON that takeoff_land publishes on /team/takeoff_land/state and
says which service to call next. The node in takeoff_land_mission.py does the
calling.

    WAIT_OFFBOARD --takeoff accepted--> WAIT_HOVER --mode hover--> HOVERING
      --hover_s elapsed--> (call land) --land accepted--> WAIT_LANDED --mode landed--> DONE

Safety: if the drone has not started landing ``max_flight_s`` after takeoff
was accepted, land is called anyway. If takeoff_land reports ``aborted`` (the
pilot took over), the mission stops and calls nothing more.
"""
from dataclasses import dataclass

WAIT_OFFBOARD = 'wait_offboard'
WAIT_HOVER = 'wait_hover'
HOVERING = 'hovering'
LANDING = 'landing'
WAIT_LANDED = 'wait_landed'
DONE = 'done'
FAILED = 'failed'

TAKEOFF = 'takeoff'
LAND = 'land'


@dataclass
class MissionConfig:
    hover_s: float = 1.0
    max_flight_s: float = 30.0
    retry_s: float = 1.0
    takeoff_attempts: int = 5


class Mission:

    def __init__(self, config=None):
        self.cfg = config or MissionConfig()
        self.phase = WAIT_OFFBOARD
        self.pending = None        # service call in flight
        self._last_call = None
        self._takeoff_tries = 0
        self._takeoff_at = None
        self._hover_at = None
        self.messages = []

    def _say(self, level, text):
        self.messages.append((level, text))

    def _due(self, now):
        return (self._last_call is None
                or now - self._last_call >= self.cfg.retry_s)

    def step(self, state, now):
        """``state``: latest takeoff_land state dict, or None. Returns the
        service to call now (TAKEOFF / LAND) or None."""
        if self.pending or self.phase in (DONE, FAILED) or state is None:
            return None
        mode = state.get('mode')

        if mode == 'aborted' and self.phase != WAIT_OFFBOARD:
            self.phase = FAILED
            self._say('warn', 'takeoff_land aborted (pilot has the drone); '
                              'mission stopped')
            return None

        if self.phase == WAIT_OFFBOARD:
            if state.get('offboard') and mode == 'passthrough' \
                    and self._due(now):
                if self._takeoff_tries >= self.cfg.takeoff_attempts:
                    self.phase = FAILED
                    self._say('error', 'takeoff refused '
                              f'{self._takeoff_tries} times; mission stopped')
                    return None
                return self._call(TAKEOFF, now)
            return None

        # Airborne phases: the overall flight-time guard comes first.
        if self.phase in (WAIT_HOVER, HOVERING) and \
                now - self._takeoff_at >= self.cfg.max_flight_s:
            self._say('warn', f'not landing after {self.cfg.max_flight_s:.0f}'
                              ' s: landing now')
            self.phase = LANDING
        if self.phase == WAIT_HOVER and mode == 'hover':
            self.phase = HOVERING
            self._hover_at = now
            self._say('info', f'at altitude; landing in {self.cfg.hover_s:.1f} s')
        if self.phase == HOVERING and now - self._hover_at >= self.cfg.hover_s:
            self.phase = LANDING
        if self.phase == LANDING and self._due(now):
            return self._call(LAND, now)
        if self.phase == WAIT_LANDED and mode == 'landed':
            self.phase = DONE
            self._say('info', 'landed and disarmed: test complete')
        return None

    def _call(self, which, now):
        self.pending = which
        self._last_call = now
        if which == TAKEOFF:
            self._takeoff_tries += 1
        return which

    def on_response(self, which, success, message, now):
        self.pending = None
        level = 'info' if success else 'warn'
        self._say(level, f'/team/{which}: {message}')
        if which == TAKEOFF and success:
            self.phase = WAIT_HOVER
            self._takeoff_at = now
        elif which == LAND and success:
            self.phase = WAIT_LANDED
        # On failure the phase is unchanged and step() retries after retry_s.

    def on_no_response(self, which):
        """Service not up, or the call errored: retry without using up an
        attempt (only a real refusal counts toward takeoff_attempts)."""
        self.pending = None
        if which == TAKEOFF:
            self._takeoff_tries -= 1
        self._say('warn', f'/team/{which}: no response; retrying')
