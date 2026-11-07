"""Occupancy mapping from range returns and an estimated pose. Standard library only.

Inputs are exactly what the drone has: (bearing, range) returns in the camera frame and the
estimated pose. Nothing here knows about the course.
"""
import math

from theta_star import FREE, OCC, UNKNOWN, Grid


class OccupancyMapper:
    def __init__(self, width, height, res, origin, l_hit=0.7, l_miss=-0.35,
                 l_min=-3.0, l_max=3.0, occ_t=1.0, free_t=-0.6, body_radius=0.2):
        self.grid = Grid(width, height, res, origin)
        self.grid.cells[:] = bytes([UNKNOWN]) * len(self.grid.cells)
        self.logodds = [0.0] * (width * height)
        self.l_hit, self.l_miss, self.l_min, self.l_max = l_hit, l_miss, l_min, l_max
        self.occ_t, self.free_t, self.body_radius = occ_t, free_t, body_radius

    def _classify(self, i):
        lo = self.logodds[i]
        self.grid.cells[i] = OCC if lo >= self.occ_t else FREE if lo <= self.free_t else UNKNOWN

    def _bump(self, cx, cy, delta):
        g = self.grid
        if g.inside(cx, cy):
            i = cy * g.w + cx
            self.logodds[i] = max(self.l_min, min(self.l_max, self.logodds[i] + delta))
            self._classify(i)

    def update(self, pose, returns):
        g = self.grid
        x, y, yaw = pose
        c0 = g.to_cell(x, y)
        for bearing, r in returns:
            hx, hy = x + r * math.cos(yaw + bearing), y + r * math.sin(yaw + bearing)
            c1 = g.to_cell(hx, hy)
            # stop carving a cell short of the hit so range noise does not erase the wall
            stop = max(0.0, r - 1.5 * g.res) / max(r, 1e-6)
            end = g.to_cell(x + (hx - x) * stop, y + (hy - y) * stop)
            for cx, cy in _line(c0, end):
                self._bump(cx, cy, self.l_miss)
            self._bump(c1[0], c1[1], self.l_hit)
        # the cells under the airframe are known free unless already seen as occupied
        n = int(self.body_radius / g.res) + 1
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                if math.hypot(dx, dy) * g.res <= self.body_radius:
                    cx, cy = c0[0] + dx, c0[1] + dy
                    if g.inside(cx, cy):
                        i = cy * g.w + cx
                        if g.cells[i] != OCC:
                            self.logodds[i] = min(self.logodds[i], self.free_t)
                            self._classify(i)
        g.invalidate()


def _line(a, b):
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
