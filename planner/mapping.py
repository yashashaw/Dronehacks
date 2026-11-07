"""Occupancy mapping from range returns and an estimated pose. Needs numpy.

Inputs are exactly what the drone has: (bearing, range) returns in the camera frame and the
estimated pose. Nothing here knows about the course.
"""
import math

import numpy as np

from theta_star import FREE, OCC, UNKNOWN, Grid


class OccupancyMapper:
    """The grid starts small, centred on the origin of the estimate frame, and grows as returns
    or the drone reach its edge, so no course size or orientation is assumed. A `visited` layer
    (where the drone has been) grows with it."""

    def __init__(self, res=0.05, size=4.0, l_hit=0.7, l_miss=-0.35,
                 l_min=-3.0, l_max=3.0, occ_t=1.0, free_t=-0.6, body_radius=0.2, cam_forward=0.105,
                 slack=1.0):
        n = int(round(size / res))
        self.grid = Grid(n, n, res, (-n * res / 2, -n * res / 2))
        self.cells = np.frombuffer(self.grid.cells, dtype=np.uint8)  # view: writes land in grid.cells
        self.cells[:] = UNKNOWN
        self.logodds = np.zeros(n * n)
        self.visited = np.zeros(n * n, dtype=np.uint8)
        self.slack = slack
        self.l_hit, self.l_miss, self.l_min, self.l_max = l_hit, l_miss, l_min, l_max
        self.occ_t, self.free_t, self.body_radius = occ_t, free_t, body_radius
        self.cam_forward = cam_forward   # returns are measured from the camera, ahead of the body origin
        n = int(body_radius / res) + 1
        dy, dx = np.mgrid[-n:n + 1, -n:n + 1]
        keep = np.hypot(dx, dy) * res <= body_radius
        self._disc = (dx[keep], dy[keep])

    def ensure(self, xmin, xmax, ymin, ymax):
        """Grow the grid (keeping its cell alignment) so it covers the box plus some slack."""
        g = self.grid
        res = g.res
        x0, y0, x1, y1 = g.ox, g.oy, g.ox + g.w * res, g.oy + g.h * res
        if xmin - self.slack >= x0 and ymin - self.slack >= y0 and xmax + self.slack <= x1 and ymax + self.slack <= y1:
            return
        nx0 = x0 - max(0, math.ceil((x0 - (xmin - 2 * self.slack)) / res)) * res
        ny0 = y0 - max(0, math.ceil((y0 - (ymin - 2 * self.slack)) / res)) * res
        nx1 = x1 + max(0, math.ceil(((xmax + 2 * self.slack) - x1) / res)) * res
        ny1 = y1 + max(0, math.ceil(((ymax + 2 * self.slack) - y1) / res)) * res
        w2, h2 = int(round((nx1 - nx0) / res)), int(round((ny1 - ny0) / res))
        sx, sy = int(round((x0 - nx0) / res)), int(round((y0 - ny0) / res))
        cells = np.full((h2, w2), UNKNOWN, dtype=np.uint8)
        logodds = np.zeros((h2, w2))
        visited = np.zeros((h2, w2), dtype=np.uint8)
        cells[sy:sy + g.h, sx:sx + g.w] = self.cells.reshape(g.h, g.w)
        logodds[sy:sy + g.h, sx:sx + g.w] = self.logodds.reshape(g.h, g.w)
        visited[sy:sy + g.h, sx:sx + g.w] = self.visited.reshape(g.h, g.w)
        g.cells = bytearray(cells.tobytes())
        g.w, g.h, g.ox, g.oy = w2, h2, nx0, ny0
        self.cells = np.frombuffer(g.cells, dtype=np.uint8)
        self.logodds = logodds.ravel()
        self.visited = visited.ravel()
        g.invalidate()

    def _classify(self, idx):
        lo = self.logodds[idx]
        self.cells[idx] = np.where(lo >= self.occ_t, OCC, np.where(lo <= self.free_t, FREE, UNKNOWN))

    def update(self, pose, returns):
        g = self.grid
        res = g.res
        x, y, yaw = pose
        ox, oy = x + self.cam_forward * math.cos(yaw), y + self.cam_forward * math.sin(yaw)
        xs, ys = [x - 0.4, x + 0.4], [y - 0.4, y + 0.4]
        if returns:
            hxs = [ox + r * math.cos(yaw + b) for b, r in returns]
            hys = [oy + r * math.sin(yaw + b) for b, r in returns]
            xs += [min(hxs), max(hxs)]
            ys += [min(hys), max(hys)]
        self.ensure(min(xs), max(xs), min(ys), max(ys))
        w, h = g.w, g.h
        if returns:
            b = np.array([r[0] for r in returns]) + yaw
            r = np.array([r[1] for r in returns])
            cs, sn = np.cos(b), np.sin(b)
            carve = np.maximum(0.0, r - 1.5 * res)   # stop a cell short so range noise does not erase the wall
            steps = int(carve.max() / (res * 0.5)) + 2
            t = np.linspace(0.0, 1.0, steps)
            px = ox + (carve * cs)[:, None] * t[None, :]
            py = oy + (carve * sn)[:, None] * t[None, :]
            cx = np.floor((px - g.ox) / res).astype(np.int64)
            cy = np.floor((py - g.oy) / res).astype(np.int64)
            inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
            ray = np.broadcast_to(np.arange(len(r))[:, None], cx.shape)
            key = np.unique(ray[inside] * (w * h) + (cy * w + cx)[inside])  # each cell once per ray
            miss = key % (w * h)
            hx = np.floor((ox + r * cs - g.ox) / res).astype(np.int64)
            hy = np.floor((oy + r * sn - g.oy) / res).astype(np.int64)
            hin = (hx >= 0) & (hx < w) & (hy >= 0) & (hy < h)
            hit = hy[hin] * w + hx[hin]
            np.add.at(self.logodds, miss, self.l_miss)
            np.add.at(self.logodds, hit, self.l_hit)
            touched = np.unique(np.concatenate((miss, hit)))
            self.logodds[touched] = np.clip(self.logodds[touched], self.l_min, self.l_max)
            self._classify(touched)
        # the cells under the airframe are known free unless already seen as occupied
        c0x, c0y = g.to_cell(x, y)
        bx, by = self._disc[0] + c0x, self._disc[1] + c0y
        ok = (bx >= 0) & (bx < w) & (by >= 0) & (by < h)
        idx = by[ok] * w + bx[ok]
        idx = idx[self.cells[idx] != OCC]
        self.logodds[idx] = np.minimum(self.logodds[idx], self.free_t)
        self._classify(idx)
        g.invalidate()
