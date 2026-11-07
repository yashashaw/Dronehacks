"""Theta* any-angle planner on a 2D occupancy grid. Needs numpy; uses scipy and numba when present.

The planner knows nothing about the course or where the grid came from. In flight the
grid is built from depth plus the estimated pose; in tests it comes from a fixture.
Frame: x/y in metres in whatever frame the grid is defined in (ENU in the sim).
"""
import heapq
import math

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # slower pure-Python fallback below
    distance_transform_edt = None
try:
    from numba import njit
except ImportError:
    njit = None

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
        """Metres from each cell centre to the nearest occupied cell. The grid edge is occupied.
        Returns a flat float64 array."""
        if self._clr is not None:
            return self._clr
        w, h = self.w, self.h
        if distance_transform_edt is not None:
            free = (np.frombuffer(self.cells, dtype=np.uint8) != OCC).reshape(h, w).copy()
            free[0, :] = free[-1, :] = False
            free[:, 0] = free[:, -1] = False
            self._clr = (distance_transform_edt(free) * self.res).ravel()
            return self._clr
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
        self._clr = np.array([math.sqrt(v) * self.res if v < _INF else _INF for v in f])
        return self._clr


def _edge_py(clr, cells, w, x0, y0, x1, y1, radius, margin, weight, res):
    """Bresenham line of sight from (x0,y0) to (x1,y1): cost of the segment, or -1 if blocked."""
    ax, ay = x0, y0
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    pen = 0.0
    n = 0
    while True:
        i = y0 * w + x0
        c = clr[i]
        if c < radius or cells[i] != 0:
            return -1.0
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
    return math.sqrt((x1 - ax) ** 2 + (y1 - ay) ** 2) * res * (1 + weight * pen / n)


_edge = njit(cache=True)(_edge_py) if njit is not None else _edge_py


def _search_py(clr, cells, w, h, sx, sy, gx, gy, radius, margin, weight, res):
    """Theta* on flat arrays. Returns (found, cost, path) with path as flat cell indices."""
    n = w * h
    gs = np.full(n, 1e30)
    parent = np.full(n, -1, np.int64)
    closed = np.zeros(n, np.bool_)
    s = sy * w + sx
    goal = gy * w + gx
    gs[s] = 0.0
    parent[s] = s
    heap = [(math.sqrt(float((gx - sx) ** 2 + (gy - sy) ** 2)) * res, s)]
    dxs = np.array([1, -1, 0, 0, 1, 1, -1, -1])
    dys = np.array([0, 0, 1, -1, 1, -1, 1, -1])
    found = False
    while len(heap) > 0:
        f, c = heapq.heappop(heap)
        if closed[c]:
            continue
        if c == goal:
            found = True
            break
        closed[c] = True
        cx = c % w
        cy = c // w
        p = parent[c]
        px = p % w
        py = p // w
        for k in range(8):
            nx = cx + dxs[k]
            ny = cy + dys[k]
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue
            ni = ny * w + nx
            if closed[ni] or cells[ni] != 0 or clr[ni] < radius:
                continue
            cand = 0.0
            par = -1
            if p != c:
                e = _edge(clr, cells, w, px, py, nx, ny, radius, margin, weight, res)
                if e >= 0:
                    cand = gs[p] + e
                    par = p
            if par < 0:
                e = _edge(clr, cells, w, cx, cy, nx, ny, radius, margin, weight, res)
                if e < 0:
                    continue
                cand = gs[c] + e
                par = c
            if cand < gs[ni]:
                gs[ni] = cand
                parent[ni] = par
                heapq.heappush(heap, (cand + math.sqrt(float((gx - nx) ** 2 + (gy - ny) ** 2)) * res, ni))
    if not found:
        return False, 0.0, np.empty(0, np.int64)
    cnt = 0
    c = goal
    while True:
        cnt += 1
        if c == s:
            break
        c = parent[c]
    path = np.empty(cnt, np.int64)
    c = goal
    for i in range(cnt - 1, -1, -1):
        path[i] = c
        c = parent[c]
    return True, gs[goal], path


_search = njit(cache=True)(_search_py) if njit is not None else None


def theta_star(grid, start, goal, radius, margin=0.3, weight=2.0):
    """Return (path, cost) from start to goal in world metres, or None. See the reference
    implementation below for the meaning of the arguments."""
    if _search is None:
        return _theta_star_python(grid, start, goal, radius, margin, weight)
    w, res = grid.w, grid.res
    radius += res
    clr = grid.clearance()
    cells = np.frombuffer(grid.cells, dtype=np.uint8)
    s, g = grid.to_cell(*start), grid.to_cell(*goal)
    for cx, cy in (s, g):
        if not grid.inside(cx, cy) or cells[cy * w + cx] != FREE or clr[cy * w + cx] < radius:
            return None
    found, cost, path = _search(clr, cells, w, grid.h, s[0], s[1], g[0], g[1], radius, margin, weight, res)
    if not found:
        return None
    return [grid.to_world(int(i % w), int(i // w)) for i in path], float(cost)


def _theta_star_python(grid, start, goal, radius, margin=0.3, weight=2.0):
    """Pure-Python reference version, used when numba is missing. Return (path, length) from start to goal in world metres, or None.

    radius: required clearance in metres from the drone centre (drone radius + buffer). The
        grid only checks cell centres, so one extra cell is added internally to keep the
        true clearance at or above this value.
    margin, weight: beyond `radius`, edges cost up to (1 + weight) times their length as
        they approach `radius`, which keeps paths off the walls.
    Only FREE cells are traversable, so a path never crosses unknown space. Unknown cells do
    not reduce clearance; only occupied cells do.
    """
    clr_np = grid.clearance()
    clr = clr_np.tolist()  # plain lists index faster than numpy in the Python search loop
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

    cells_np = np.frombuffer(grid.cells, dtype=np.uint8)

    def edge(a, b):
        """Cost of the straight segment a->b, or None if it crosses a forbidden cell."""
        r = _edge(clr_np, cells_np, w, a[0], a[1], b[0], b[1], radius, margin, weight, res)
        return None if r < 0 else r

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
