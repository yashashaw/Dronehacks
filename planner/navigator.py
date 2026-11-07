"""Frontier exploration + Theta* + pure-pursuit follower. Standard library only.

Consumes estimated pose, range returns and an optional pad cue; produces a velocity command
in the estimate's frame. Nothing here knows the course: goals are frontiers (the edge of
known free space) or the landing pad once it has been seen.
"""
import math
from dataclasses import dataclass

from mapping import OccupancyMapper
from theta_star import FREE, UNKNOWN, theta_star


@dataclass
class NavParams:
    radius: float = 0.25           # drone half-diagonal 0.23 m + buffer
    v_max: float = 1.0
    a_brake: float = 1.5           # speed limit keeps stopping distance inside the seen free range
    stop_margin: float = 0.45
    lookahead: float = 0.5
    plan_every: int = 10           # frames (sensor runs at 15 Hz)
    goal_every: int = 30
    w_visit: float = 6.0           # cost of heading for places the trail already covers
    w_turn: float = 0.5            # cost per radian of turning toward the goal (momentum)
    stickiness: float = 0.8        # cost credit for keeping the current frontier
    max_candidates: int = 6
    pad_stop: float = 0.25
    rep_gain: float = 0.8          # m/s push away from returns closer than rep_dist (0 = off)
    rep_dist: float = 0.5          # centre-to-obstacle distance where the push starts
    rep_min: float = 0.25          # distance at which the push is at full gain
    rep_norm: float = 8.0          # a wall seen by many rays should not push harder


class Navigator:
    def __init__(self, width, height, res, origin, params=None):
        self.p = params or NavParams()
        self.mapper = OccupancyMapper(width, height, res, origin)
        self.grid = self.mapper.grid
        self.visited = bytearray(width * height)
        self.pad = None                  # (x, y) in the estimate frame
        self.pad_hits = 0
        self.path = []
        self.goal = None
        self.frame = 0
        self.status = "explore"

    # ---- perception bookkeeping -------------------------------------------------------
    def _see_pad(self, pose, obs):
        x, y, yaw = pose
        r, b = obs
        px, py = x + r * math.cos(yaw + b), y + r * math.sin(yaw + b)
        if self.pad is None:
            self.pad = (px, py)
        else:  # running mean damps single-frame noise
            k = 1 / min(self.pad_hits + 1, 10)
            self.pad = (self.pad[0] + k * (px - self.pad[0]), self.pad[1] + k * (py - self.pad[1]))
        self.pad_hits += 1

    def _mark_visited(self, x, y, r=0.4):
        g = self.grid
        c = g.to_cell(x, y)
        n = int(r / g.res)
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                if dx * dx + dy * dy <= n * n and g.inside(c[0] + dx, c[1] + dy):
                    self.visited[(c[1] + dy) * g.w + c[0] + dx] = 1

    # ---- goals --------------------------------------------------------------------------
    def _ok(self, c):
        g = self.grid
        return (g.inside(*c) and g.cells[c[1] * g.w + c[0]] == FREE
                and g.clearance()[c[1] * g.w + c[0]] >= self.p.radius + g.res)

    def _nearest_ok(self, point, rmax):
        g = self.grid
        c0 = g.to_cell(*point)
        best = None
        n = int(rmax / g.res)
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                c = (c0[0] + dx, c0[1] + dy)
                d = dx * dx + dy * dy
                if d <= n * n and (best is None or d < best[0]) and self._ok(c):
                    best = (d, c)
        return None if best is None else g.to_world(*best[1])

    def frontiers(self):
        """Clusters of reachable free cells that touch unknown space: (cell, size)."""
        g = self.grid
        cand = set()
        for cy in range(1, g.h - 1):
            row = cy * g.w
            for cx in range(1, g.w - 1):
                if g.cells[row + cx] == FREE and (
                        g.cells[row + cx + 1] == UNKNOWN or g.cells[row + cx - 1] == UNKNOWN
                        or g.cells[row + g.w + cx] == UNKNOWN or g.cells[row - g.w + cx] == UNKNOWN):
                    if self._ok((cx, cy)):
                        cand.add((cx, cy))
        out = []
        while cand:
            seed = cand.pop()
            stack, members = [seed], [seed]
            while stack:
                cx, cy = stack.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in cand:
                            cand.remove(n)
                            stack.append(n)
                            members.append(n)
            if len(members) >= 4:
                mx = sum(m[0] for m in members) / len(members)
                my = sum(m[1] for m in members) / len(members)
                out.append((min(members, key=lambda m: (m[0] - mx) ** 2 + (m[1] - my) ** 2), len(members)))
        return out

    def _trail_fraction(self, c, r=0.6):
        g = self.grid
        n = int(r / g.res)
        tot = hit = 0
        for dy in range(-n, n + 1, 2):
            for dx in range(-n, n + 1, 2):
                if dx * dx + dy * dy <= n * n and g.inside(c[0] + dx, c[1] + dy):
                    tot += 1
                    hit += self.visited[(c[1] + dy) * g.w + c[0] + dx]
        return hit / tot if tot else 0.0

    def _plan(self, pose):
        x, y, yaw = pose
        p = self.p
        start = (x, y) if self._ok(self.grid.to_cell(x, y)) else self._nearest_ok((x, y), 0.5)
        if start is None:
            self.path, self.status = [], "no_start"
            return
        if self.pad is not None:
            goal = self._nearest_ok(self.pad, 1.0)
            if goal is not None:
                res = theta_star(self.grid, start, goal, p.radius)
                if res:
                    self.path, self.goal, self.status = res[0], goal, "to_pad"
                    return
        best = None
        cands = sorted(self.frontiers(), key=lambda f: math.dist(self.grid.to_world(*f[0]), (x, y)))
        for cell, size in cands[:p.max_candidates]:
            pt = self.grid.to_world(*cell)
            res = theta_star(self.grid, start, pt, p.radius)
            if not res:
                continue
            path, cost = res
            ahead = path[1] if len(path) > 1 else pt
            turn = abs((math.atan2(ahead[1] - y, ahead[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi)
            score = cost + p.w_visit * self._trail_fraction(cell) + p.w_turn * turn
            if self.goal is not None and math.dist(pt, self.goal) < 0.5:
                score -= p.stickiness
            if best is None or score < best[0]:
                best = (score, path, pt)
        if best:
            self.path, self.goal, self.status = best[1], best[2], "explore"
        else:
            self.path, self.goal, self.status = [], None, "no_frontier"

    def _reroute(self, pose):
        """Keep the current frontier goal, refresh the route to it as the map changes."""
        x, y, _ = pose
        start = (x, y) if self._ok(self.grid.to_cell(x, y)) else self._nearest_ok((x, y), 0.5)
        res = theta_star(self.grid, start, self.goal, self.p.radius) if start else None
        if res:
            self.path = res[0]
        else:
            self._plan(pose)

    # ---- control --------------------------------------------------------------------------
    def _carrot(self, x, y):
        pts = self.path
        i = min(range(len(pts)), key=lambda k: math.dist(pts[k], (x, y)))
        remaining = self.p.lookahead
        cur = pts[i]
        for nxt in pts[i + 1:]:
            seg = math.dist(cur, nxt)
            if seg >= remaining:
                t = remaining / seg
                return cur[0] + (nxt[0] - cur[0]) * t, cur[1] + (nxt[1] - cur[1]) * t
            remaining -= seg
            cur = nxt
        return pts[-1]

    def _speed(self, heading, yaw, returns, dist_left):
        p = self.p
        off = abs((heading - yaw + math.pi) % (2 * math.pi) - math.pi)
        if off > math.radians(60):
            return 0.0
        rel = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
        seen = [r for b, r in returns if abs(b - rel) < math.radians(15)]
        free = min(seen) if seen else 2.0
        v = math.sqrt(2 * p.a_brake * max(0.0, free - p.stop_margin))
        return min(p.v_max, v, dist_left + 0.1) * math.cos(off)

    def _repel(self, yaw, returns):
        """Push away from nearby returns. Uses only the current frame in the camera frame,
        so pose drift and map errors cannot affect it."""
        p = self.p
        if p.rep_gain <= 0:
            return 0.0, 0.0
        rs = sorted(returns)
        px = py = 0.0
        for i, (b, r) in enumerate(rs):
            if r >= p.rep_dist:
                continue
            # a lone return (noise, outlier) has no neighbour at a similar range: ignore it
            if not any(j != i and abs(rs[j][0] - b) < math.radians(6) and abs(rs[j][1] - r) < 0.25
                       for j in range(max(0, i - 2), min(len(rs), i + 3))):
                continue
            w = min(1.0, (p.rep_dist - r) / (p.rep_dist - p.rep_min))
            px -= w * math.cos(yaw + b)
            py -= w * math.sin(yaw + b)
        k = p.rep_gain / p.rep_norm
        mag = math.hypot(px, py) * k
        if mag > p.rep_gain:
            k *= p.rep_gain / mag
        return px * k, py * k

    def step(self, pose, returns, pad_obs):
        """Returns (vx, vy) in the estimate frame and the yaw to face."""
        x, y, yaw = pose
        self.frame += 1
        self.mapper.update(pose, returns)
        self._mark_visited(x, y)
        if pad_obs:
            self._see_pad(pose, pad_obs)
        reached = self.goal is not None and math.dist((x, y), self.goal) < 0.3
        if self.path and not reached:
            due, refresh = self.frame % self.p.plan_every == 0, self.status == "explore" and self.frame % self.p.goal_every
        else:
            due, refresh = self.frame % 5 == 0 or reached, False
        if due:
            self._reroute(pose) if refresh else self._plan(pose)
        if self.pad is not None and math.dist((x, y), self.pad) < self.p.pad_stop:
            self.status = "landing"
            return (0.0, 0.0), yaw
        if not self.path:  # nothing reachable yet: turn in place to look around
            return (0.0, 0.0), yaw + 1.0
        tx, ty = self._carrot(x, y)
        heading = math.atan2(ty - y, tx - x)
        v = self._speed(heading, yaw, returns, math.dist((x, y), self.path[-1]))
        rx, ry = self._repel(yaw, returns)
        return (v * math.cos(heading) + rx, v * math.sin(heading) + ry), heading
