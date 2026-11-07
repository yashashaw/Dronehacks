"""Reactive baseline for benchmarking: steer toward the most open direction, no map, no planner.

Same interface as Navigator.step. It only sees the current range returns; the pad cue is used the
same way as in the mapped navigator. Deliberately simple: it is the "why not skip the map" baseline.
"""
import math
from dataclasses import dataclass


@dataclass
class ReactiveParams:
    v_max: float = 1.0
    a_brake: float = 1.5
    stop_margin: float = 0.45
    window: float = math.radians(14)   # a direction counts only if everything within this angle is clear
    blocked: float = 1.0               # if the best direction is closer than this, turn in place
    momentum: float = 0.35             # preference for continuing straight / the previous direction
    smooth: float = 0.5
    pad_stop: float = 0.25


class ReactiveNavigator:
    def __init__(self, params=None):
        self.p = params or ReactiveParams()
        self.status = "explore"
        self.stuck_events = 0
        self.plan_calls = 0
        self.plan_time = 0.0
        self.grid = None
        self.pad = None
        self.pad_hits = 0
        self.prev = 0.0        # previous chosen direction relative to the camera
        self.spin = 1.0        # which way to turn when blocked

    def _see_pad(self, pose, obs):
        x, y, yaw = pose
        r, b = obs
        px, py = x + r * math.cos(yaw + b), y + r * math.sin(yaw + b)
        k = 1 / min(self.pad_hits + 1, 10)
        self.pad = (px, py) if self.pad is None else (self.pad[0] + k * (px - self.pad[0]), self.pad[1] + k * (py - self.pad[1]))
        self.pad_hits += 1

    def step(self, pose, returns, pad_obs, t=None):
        p = self.p
        x, y, yaw = pose
        if pad_obs:
            self._see_pad(pose, pad_obs)
        if self.pad is not None:
            if math.dist((x, y), self.pad) < p.pad_stop:
                self.status = "landing"
                return (0.0, 0.0), yaw
            self.status = "to_pad"
            heading = math.atan2(self.pad[1] - y, self.pad[0] - x)
            rel = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
            ahead = [r for b, r in returns if abs(b - rel) < math.radians(12)]
            free = min(ahead) if ahead else 2.0
            v = min(p.v_max, math.sqrt(2 * p.a_brake * max(0.0, free - p.stop_margin)))
            v *= max(0.0, math.cos(rel)) if abs(rel) < math.radians(60) else 0.0
            return (v * math.cos(heading), v * math.sin(heading)), heading
        self.status = "explore"
        if not returns:
            return (0.0, 0.0), yaw + 0.5 * self.spin
        rs = sorted(returns)
        lo, hi = -0.68, 0.68                # the camera's horizontal field of view
        best, best_score, best_clear = 0.0, -1e9, 0.0
        n = 25
        for k in range(n):
            th = lo + (hi - lo) * k / (n - 1)
            near = [r for b, r in rs if abs(b - th) <= p.window]
            clear = min(near) if near else 1.0     # a direction with no returns is treated as unknown/short
            score = clear * (1 - p.momentum * abs(th - self.prev) / max(1e-6, hi - lo))
            if score > best_score:
                best, best_score, best_clear = th, score, clear
        if best_clear < p.blocked:
            self.spin = 1.0 if best > 0 else -1.0
            return (0.0, 0.0), yaw + 0.6 * self.spin
        self.prev = p.smooth * self.prev + (1 - p.smooth) * best
        heading = yaw + self.prev
        v = min(p.v_max, math.sqrt(2 * p.a_brake * max(0.0, best_clear - p.stop_margin)))
        return (v * math.cos(heading), v * math.sin(heading)), heading
