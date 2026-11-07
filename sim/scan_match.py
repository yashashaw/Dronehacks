"""Depth-vs-map localizer (plain Python, no numpy needed).

Matches one depth scan (bearing, range pairs from the depth band) against the solid walls of the
published course model and returns a corrected 2D pose (x east, y north, heading from north,
clockwise = PX4 convention). Nets, floor, pads and low boulders are excluded: depth cannot see the
nets and the depth band at 1 m flight height passes over the boulders.

Usage:
    m = MapMatcher.from_obj('course/dronehacks-course-v3.obj')
    fix = m.match(pred_x, pred_y, pred_hdg, scan)   # scan = [(bearing_rad, range_m), ...]
    fix -> dict(x, y, hdg, cost, n, obs_x, obs_y) or None (not enough wall points)
obs_x / obs_y say whether the fit actually constrains that axis (a straight corridor only
constrains the sideways axis), so the caller corrects only observable axes.
"""
import math

SKIP = ('net', 'floor', 'roof', 'landing', 'pad', 'ring', 'guideline', 'launch', 'boulder', 'cradle')
RES = 0.03          # distance-field cell size (m)
CAP = 0.30          # per-point distance cap (robust to outliers / unmodelled returns)
STIFF = 0.15        # cost curvature above which a direction counts as observed


def _hull(pts):
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def _inside(x, y, P):
    c = False
    for k in range(len(P)):
        (ax, ay), (bx, by) = P[k], P[(k + 1) % len(P)]
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            c = not c
    return c


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


class MapMatcher:
    def __init__(self, segments, polys=(), half=3.05):
        self.segs = segments
        self.polys = polys
        self.half = half
        self.n = int(2 * half / RES) + 1
        # distance field: distance from each cell centre to the nearest wall segment (capped)
        self.df = []
        for j in range(self.n):
            y = -half + j * RES
            row = []
            near = [s for s in segments if min(abs(y - s[1]), abs(y - s[3])) < CAP + 1.0 or (min(s[1], s[3]) <= y <= max(s[1], s[3]))]
            for i in range(self.n):
                x = -half + i * RES
                d = CAP
                for ax, ay, bx, by in near:
                    dd = _seg_dist(x, y, ax, ay, bx, by)
                    if dd < d:
                        d = dd
                if d > 1.5 * RES and any(_inside(x, y, P) for P in self.polys):
                    d = CAP           # deep inside a solid wall: impossible return
                row.append(d)
            self.df.append(row)

    @classmethod
    def from_obj(cls, path):
        verts, groups, cur = [], {}, None
        with open(path, errors='ignore') as f:
            for line in f:
                if line.startswith('v '):
                    verts.append(tuple(float(v) for v in line.split()[1:4]))
                elif line.startswith('g '):
                    cur = line[2:].strip()
                elif line.startswith('f '):
                    for tok in line.split()[1:]:
                        i = int(tok.split('/')[0])
                        groups.setdefault(cur, set()).add(verts[i - 1 if i > 0 else len(verts) + i])
        segs, polys = [], []
        for name, vs in groups.items():
            if any(k in name.lower() for k in SKIP):
                continue
            if max(v[2] for v in vs) < 0.9:      # below the depth band at flight height
                continue
            hull = _hull([(round(v[0], 3), round(v[1], 3)) for v in vs])
            tube = 'tube' in name.lower()
            for k in range(len(hull)):
                a, b = hull[k], hull[(k + 1) % len(hull)]
                if tube and abs(a[1] - b[1]) < 1e-3:
                    continue          # tube is hollow: keep only its side walls, ends are open
                segs.append((a[0], a[1], b[0], b[1]))
            if not tube:
                polys.append(hull)
        return cls(segs, polys)

    def dist(self, x, y):
        fx = (x + self.half) / RES
        fy = (y + self.half) / RES
        i, j = int(math.floor(fx)), int(math.floor(fy))
        if 0 <= i < self.n - 1 and 0 <= j < self.n - 1:
            tx, ty = fx - i, fy - j
            r0, r1 = self.df[j], self.df[j + 1]
            return ((r0[i] * (1 - tx) + r0[i + 1] * tx) * (1 - ty)
                    + (r1[i] * (1 - tx) + r1[i + 1] * tx) * ty)
        return CAP

    def cost(self, x, y, hdg, scan):
        c = 0.0
        for b, r in scan:
            a = hdg + b
            d = self.dist(x + r * math.sin(a), y + r * math.cos(a))
            c += d * d
        return c / max(1, len(scan))

    def match(self, px, py, phdg, scan, win=0.30, ywin=math.radians(12)):
        # keep returns that land inside the net square at the predicted pose: depth sees THROUGH
        # the nets to the room outside, which is not in the map
        keep = []
        for b, r in scan:
            if r is None or not (0.3 < r < 2.5):
                continue
            a = phdg + b
            wx, wy = px + r * math.sin(a), py + r * math.cos(a)
            if abs(wx) < 2.90 and abs(wy) < 2.90:
                keep.append((b, r))
        scan = keep
        if len(scan) < 15:
            return None
        best = (self.cost(px, py, phdg, scan), px, py, phdg)
        # coarse-to-fine grid search around the prediction
        for step, ystep, span, yspan in ((0.05, math.radians(2), win, ywin),
                                         (0.03, math.radians(1), 0.10, math.radians(3)),
                                         (0.01, math.radians(0.3), 0.03, math.radians(1))):
            cx, cy, ch = best[1], best[2], best[3]
            k = int(round(span / step))
            ky = int(round(yspan / ystep))
            for dh in range(-ky, ky + 1):
                h = ch + dh * ystep
                for dx in range(-k, k + 1):
                    for dy in range(-k, k + 1):
                        c = self.cost(cx + dx * step, cy + dy * step, h, scan)
                        if c < best[0]:
                            best = (c, cx + dx * step, cy + dy * step, h)
        c0, x, y, h = best
        # Observability: 2x2 curvature of the cost in x/y. Correct only along stiff directions
        # (a single wall pins the distance to it but not the slide along it).
        e = 0.05
        cxx = (self.cost(x + e, y, h, scan) + self.cost(x - e, y, h, scan) - 2 * c0) / (e * e)
        cyy = (self.cost(x, y + e, h, scan) + self.cost(x, y - e, h, scan) - 2 * c0) / (e * e)
        cxy = (self.cost(x + e, y + e, h, scan) - self.cost(x + e, y - e, h, scan)
               - self.cost(x - e, y + e, h, scan) + self.cost(x - e, y - e, h, scan)) / (4 * e * e)
        tr, det = cxx + cyy, cxx * cyy - cxy * cxy
        disc = math.sqrt(max(0.0, tr * tr / 4 - det))
        dx, dy = x - px, y - py
        cx_, cy_ = 0.0, 0.0
        stiff = 0
        for lam in (tr / 2 + disc, tr / 2 - disc):
            if lam < STIFF:
                continue
            stiff += 1
            if abs(cxy) > 1e-9:
                vx, vy = lam - cyy, cxy
            elif abs(lam - cxx) < abs(lam - cyy):
                vx, vy = 1.0, 0.0
            else:
                vx, vy = 0.0, 1.0
            nv = math.hypot(vx, vy)
            vx, vy = vx / nv, vy / nv
            p = dx * vx + dy * vy
            cx_ += p * vx
            cy_ += p * vy
        eh = math.radians(2.0)
        hstiff = (self.cost(x, y, h + eh, scan) + self.cost(x, y, h - eh, scan) - 2 * c0) / (eh * eh)
        return dict(x=px + cx_, y=py + cy_, hdg=h, cost=c0, n=len(scan), stiff=stiff,
                    hstiff=hstiff, raw=(x, y))

    def raycast(self, x, y, hdg, bearings, max_r=4.0):
        """Synthetic scan for tests: exact range to the nearest wall along each bearing."""
        out = []
        for b in bearings:
            a = hdg + b
            dx, dy = math.sin(a), math.cos(a)
            best = None
            for ax, ay, bx, by in self.segs:
                ex, ey = bx - ax, by - ay
                den = dx * ey - dy * ex
                if abs(den) < 1e-9:
                    continue
                t = ((ax - x) * ey - (ay - y) * ex) / den
                u = ((ax - x) * dy - (ay - y) * dx) / den
                if t > 0.05 and 0 <= u <= 1 and (best is None or t < best):
                    best = t
            out.append((b, best if best is not None and best < max_r else None))
        return out

# Solid walls of dronehacks-course-v3.obj at flight height (generated by from_obj; nets/floor/pads/boulders excluded)
SEGS = [(-0.243, -1.496, -0.147, -1.531), (-0.147, -1.531, 0.205, -0.565), (0.205, -0.565, 0.109, -0.53), (0.109, -0.53, -0.243, -1.496), (0.178, 0.406, 1.575, 0.406), (1.575, 0.406, 1.575, 0.508), (1.575, 0.508, 0.178, 0.508), (0.178, 0.508, 0.178, 0.406), (-1.435, 0.406, 0.178, 0.406), (0.178, 0.406, 0.178, 0.508), (0.178, 0.508, -1.435, 0.508), (-1.435, 0.508, -1.435, 0.406), (0.926, -1.111, 1.021, -1.146), (1.021, -1.146, 1.573, 0.37), (1.573, 0.37, 1.477, 0.404), (1.477, 0.404, 0.926, -1.111), (3.048, -0.305, 3.048, 1.219), (1.575, 1.219, 1.575, -0.305), (-1.981, 0.508, -1.88, 0.508), (-1.88, 0.508, -1.88, 2.121), (-1.88, 2.121, -1.981, 2.121), (-1.981, 2.121, -1.981, 0.508), (-0.794, -3.012, -0.699, -3.047), (-0.699, -3.047, -0.147, -1.531), (-0.147, -1.531, -0.243, -1.497), (-0.243, -1.497, -0.675, -2.684), (-0.675, -2.684, -0.794, -3.012), (-3.048, 0.406, -1.435, 0.406), (-1.435, 0.406, -1.435, 0.508), (-1.435, 0.508, -3.048, 0.508), (-3.048, 0.508, -3.048, 0.406), (-1.026, -0.622, -0.925, -0.622), (-0.925, -0.622, -0.925, 0.406), (-0.925, 0.406, -1.026, 0.406), (-1.026, 0.406, -1.026, -0.622), (-0.864, 1.435, -0.762, 1.435), (-0.762, 1.435, -0.762, 3.048), (-0.762, 3.048, -0.864, 3.048), (-0.864, 3.048, -0.864, 1.435), (0.254, 0.508, 0.356, 0.508), (0.356, 0.508, 0.356, 2.121), (0.356, 2.121, 0.254, 2.121), (0.254, 2.121, 0.254, 0.508)]
POLYS = [[(-0.243, -1.496), (-0.147, -1.531), (0.205, -0.565), (0.109, -0.53)], [(0.178, 0.406), (1.575, 0.406), (1.575, 0.508), (0.178, 0.508)], [(-1.435, 0.406), (0.178, 0.406), (0.178, 0.508), (-1.435, 0.508)], [(0.926, -1.111), (1.021, -1.146), (1.573, 0.37), (1.477, 0.404)], [(-1.981, 0.508), (-1.88, 0.508), (-1.88, 2.121), (-1.981, 2.121)], [(-0.794, -3.012), (-0.699, -3.047), (-0.147, -1.531), (-0.243, -1.497), (-0.675, -2.684)], [(-3.048, 0.406), (-1.435, 0.406), (-1.435, 0.508), (-3.048, 0.508)], [(-1.026, -0.622), (-0.925, -0.622), (-0.925, 0.406), (-1.026, 0.406)], [(-0.864, 1.435), (-0.762, 1.435), (-0.762, 3.048), (-0.864, 3.048)], [(0.254, 0.508), (0.356, 0.508), (0.356, 2.121), (0.254, 2.121)]]


def course_matcher():
    return MapMatcher(SEGS, POLYS)
