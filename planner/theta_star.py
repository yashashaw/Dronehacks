"""Theta* any-angle planner on a 2D occupancy grid. Standard library only.

The planner knows nothing about the course or where the grid came from. In flight the
grid is built from depth plus the estimated pose; in tests it comes from a fixture.
Frame: x/y in metres in whatever frame the grid is defined in (ENU in the sim).
"""
import heapq
import math

FREE, OCC, UNKNOWN = 0, 1, 2
_INF = 1e20
_NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def _dt1d(f):
    """Exact 1D squared distance transform (Felzenszwalb & Huttenlocher)."""
    n = len(f)
    d = [0.0] * n
    v = [0] * n
    z = [0.0] * (n + 1)
    k = 0
    z[0], z[1] = -_INF, _INF
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = _INF
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


class Grid:
    def __init__(self, width, height, res, origin=(0.0, 0.0)):
        self.w, self.h, self.res = width, height, res
        self.ox, self.oy = origin
        self.cells = bytearray(width * height)
        self._clr = None

    def invalidate(self):
        """Call after writing `cells` directly."""
        self._clr = None

    def inside(self, cx, cy):
        return 0 <= cx < self.w and 0 <= cy < self.h

    def to_cell(self, x, y):
        return (int(math.floor((x - self.ox) / self.res)),
                int(math.floor((y - self.oy) / self.res)))

    def to_world(self, cx, cy):
        return (self.ox + (cx + 0.5) * self.res, self.oy + (cy + 0.5) * self.res)

    def set(self, cx, cy, value=OCC):
        if self.inside(cx, cy):
            self.cells[cy * self.w + cx] = value
            self._clr = None

    def clearance(self):
        """Metres from each cell centre to the nearest occupied cell. The grid edge is occupied."""
        if self._clr is not None:
            return self._clr
        w, h = self.w, self.h
        f = [0.0 if c == OCC else _INF for c in self.cells]
        for x in range(w):
            f[x], f[(h - 1) * w + x] = 0.0, 0.0
        for y in range(h):
            f[y * w], f[y * w + w - 1] = 0.0, 0.0
        for x in range(w):
            col = _dt1d([f[y * w + x] for y in range(h)])
            for y in range(h):
                f[y * w + x] = col[y]
        for y in range(h):
            f[y * w:(y + 1) * w] = _dt1d(f[y * w:(y + 1) * w])
        self._clr = [math.sqrt(v) * self.res if v < _INF else _INF for v in f]
        return self._clr


def theta_star(grid, start, goal, radius, margin=0.3, weight=2.0):
    """Return (path, length) from start to goal in world metres, or None.

    radius: required clearance in metres from the drone centre (drone radius + buffer). The
        grid only checks cell centres, so one extra cell is added internally to keep the
        true clearance at or above this value.
    margin, weight: beyond `radius`, edges cost up to (1 + weight) times their length as
        they approach `radius`, which keeps paths off the walls.
    Only FREE cells are traversable, so a path never crosses unknown space. Unknown cells do
    not reduce clearance; only occupied cells do.
    """
    clr = grid.clearance()
    cells = grid.cells
    w, res = grid.w, grid.res
    radius += res
    s = grid.to_cell(*start)
    g = grid.to_cell(*goal)

    def ok(c):
        return (grid.inside(*c) and cells[c[1] * w + c[0]] == FREE
                and clr[c[1] * w + c[0]] >= radius)

    if not ok(s) or not ok(g):
        return None

    def edge(a, b):
        """Cost of the straight segment a->b, or None if it crosses a forbidden cell."""
        x0, y0 = a
        x1, y1 = b
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        pen = 0.0
        n = 0
        while True:
            c = clr[y0 * w + x0]
            if c < radius or cells[y0 * w + x0] != FREE:
                return None
            if c < radius + margin:
                pen += (radius + margin - c) / margin
            n += 1
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return math.hypot(x1 - a[0], y1 - a[1]) * res * (1 + weight * pen / n)

    def h(c):
        return math.hypot(g[0] - c[0], g[1] - c[1]) * res

    gscore = {s: 0.0}
    parent = {s: s}
    closed = set()
    heap = [(h(s), 0, s)]
    tie = 0
    while heap:
        _, _, c = heapq.heappop(heap)
        if c in closed:
            continue
        if c == g:
            cells = [c]
            while cells[-1] != s:
                cells.append(parent[cells[-1]])
            cells.reverse()
            return [grid.to_world(*p) for p in cells], gscore[c]
        closed.add(c)
        for dx, dy in _NEIGHBOURS:
            n = (c[0] + dx, c[1] + dy)
            if n in closed or not ok(n):
                continue
            p = parent[c]
            cand = par = None
            if p != c:
                ep = edge(p, n)
                if ep is not None:
                    cand, par = gscore[p] + ep, p
            if cand is None:
                e = edge(c, n)
                if e is None:
                    continue
                cand, par = gscore[c] + e, c
            if cand < gscore.get(n, _INF):
                gscore[n] = cand
                parent[n] = par
                tie += 1
                heapq.heappush(heap, (cand + h(n), tie, n))
    return None
