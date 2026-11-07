"""TEST ONLY: builds an occupancy grid from the published course OBJ at one flight height.

This stands in for the real world in tests. Never import it from flight code. Walls come
from the mesh; the tunnel is approximated as two side strips and boulders as circles.
Coordinates are the OBJ's (metres, Z up, origin at floor centre).
"""
import math
import struct
import zlib

import numpy as np

from theta_star import FREE, OCC, Grid

NET = 3.048
WALL_WORDS = ("wall", "divider", "Barrier", "approach")


def load_groups(obj_path):
    verts, groups, current = [], {}, None
    with open(obj_path) as fh:
        for line in fh:
            if line.startswith("v "):
                verts.append(tuple(map(float, line.split()[1:4])))
            elif line.startswith("g "):
                current = line[2:].strip()
                groups.setdefault(current, set())
            elif line.startswith("f ") and current:
                for tok in line.split()[1:]:
                    groups[current].add(verts[int(tok.split("/")[0]) - 1])
    return groups


def _hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(reversed(pts))
    return lower[:-1] + upper[:-1]


def _fill_polygon(grid, poly, pad):
    """Mark cells whose centre is within `pad` of the convex polygon (conservative)."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    c0 = grid.to_cell(min(xs) - pad, min(ys) - pad)
    c1 = grid.to_cell(max(xs) + pad, max(ys) + pad)
    for cy in range(c0[1], c1[1] + 1):
        for cx in range(c0[0], c1[0] + 1):
            x, y = grid.to_world(cx, cy)
            inside = True
            for i in range(len(poly)):
                ax, ay = poly[i]
                bx, by = poly[(i + 1) % len(poly)]
                length = math.hypot(bx - ax, by - ay) or 1.0
                if ((bx - ax) * (y - ay) - (by - ay) * (x - ax)) / length < -pad:
                    inside = False
                    break
            if inside:
                grid.set(cx, cy, OCC)


def _wall_footprints(verts, z):
    """Footprint polygons of a wall at height z; a window cut-out splits it in two."""
    ends = sorted({(round(x, 4), round(y, 4)) for x, y, vz in verts if vz < 0.1 or vz > 1.7},
                  key=lambda p: p[1])
    edges = sorted({(round(x, 4), round(y, 4)) for x, y, vz in verts if 0.1 < vz < 1.7},
                   key=lambda p: p[1])
    if not edges:
        return [_hull([(x, y) for x, y, _ in verts])]
    z_lo = min(vz for _, _, vz in verts if 0.1 < vz < 1.7)
    z_hi = max(vz for _, _, vz in verts if 0.1 < vz < 1.7)
    if not z_lo < z < z_hi:
        return [_hull(ends)]
    return [_hull(ends[:2] + edges[:2]), _hull(edges[2:] + ends[2:])]


def build_grid(obj_path, res=0.05, z=0.76, boulders=True, extra_blocks=(), groups=None):
    """extra_blocks: (xmin, ymin, xmax, ymax) rectangles to mark occupied, for what-if tests."""
    groups = groups or load_groups(obj_path)
    n = int(math.ceil(2 * NET / res))
    grid = Grid(n, n, res, (-NET, -NET))
    for name, verts in groups.items():
        zs = [v[2] for v in verts]
        if not min(zs) - 1e-6 <= z <= max(zs) + 1e-6:
            continue
        if any(word in name for word in WALL_WORDS) and not name.startswith("Net"):
            for poly in _wall_footprints(verts, z):
                _fill_polygon(grid, poly, res / 2)
        elif name.startswith("Tube oval"):
            xs, ys = [v[0] for v in verts], [v[1] for v in verts]
            for x0, x1 in ((min(xs), min(xs) + 0.05), (max(xs) - 0.05, max(xs))):
                _fill_polygon(grid, [(x0, min(ys)), (x1, min(ys)), (x1, max(ys)), (x0, max(ys))], res / 2)
        elif boulders and name.startswith("Boulder"):
            xs, ys = [v[0] for v in verts], [v[1] for v in verts]
            cx, cy, r = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (max(xs) - min(xs)) / 2
            for gy in range(grid.h):
                for gx in range(grid.w):
                    x, y = grid.to_world(gx, gy)
                    if math.hypot(x - cx, y - cy) <= r + res / 2:
                        grid.set(gx, gy, OCC)
    for x0, y0, x1, y1 in extra_blocks:
        _fill_polygon(grid, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], res / 2)
    return grid


class World:
    """Course as horizontal layers: (z_lo, z_hi, Grid). Within a layer the footprint is constant."""

    def __init__(self, layers):
        self.layers = layers
        first = layers[0][2]
        self.w, self.h, self.res, self.ox, self.oy = first.w, first.h, first.res, first.ox, first.oy
        self.clr = [g.clearance() for _, _, g in layers]

    def overlapping(self, lo, hi):
        return [i for i, (a, b, _) in enumerate(self.layers) if b > lo and a < hi]

    def clearance_at(self, pos, lo, hi):
        """Distance from pos to the nearest obstacle that intersects the height band [lo, hi]."""
        g = self.layers[0][2]
        cx, cy = g.to_cell(*pos)
        if not g.inside(cx, cy):
            return 0.0
        return min((self.clr[i][cy * g.w + cx] for i in self.overlapping(lo, hi)), default=_BIG)

    def union_grid(self, lo, hi):
        out = Grid(self.w, self.h, self.res, (self.ox, self.oy))
        occ = np.zeros(self.w * self.h, dtype=bool)
        for i in self.overlapping(lo, hi):
            occ |= np.frombuffer(self.layers[i][2].cells, dtype=np.uint8) == OCC
        np.frombuffer(out.cells, dtype=np.uint8)[occ] = OCC
        return out


_BIG = 1e9


def build_world(obj_path, res=0.05, extra_blocks=(), boulders=True):
    """Layers split wherever the geometry changes height (window edges, boulder tops, wall tops)."""
    groups = load_groups(obj_path)
    marks = {0.0}
    for name, verts in groups.items():
        if name.startswith(("Net", "Floor", "Landing", "Guideline", "Launch", "Approach ring", "Tube cradle")):
            continue
        zs = [v[2] for v in verts]
        if name.startswith("Tube oval") or name.startswith("Boulder"):
            marks.update((round(max(min(zs), 0.0), 3), round(max(zs), 3)))
        else:
            marks.update(round(z, 3) for z in zs)
    edges = sorted(marks)
    layers = []
    for lo, hi in zip(edges, edges[1:]):
        if hi - lo < 0.01:
            continue
        grid = build_grid(obj_path, res, z=(lo + hi) / 2, boulders=boulders, extra_blocks=extra_blocks, groups=groups)
        layers.append((lo, hi, grid))
    return World(layers)


def write_png(filename, width, height, rows):
    """rows: one bytes object per pixel row, each starting with a filter byte (0)."""
    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(b"".join(rows))) + chunk(b"IEND", b""))
    with open(filename, "wb") as fh:
        fh.write(png)


def render_png(grid, path, filename, radius, scale=4):
    """Black = obstacle, grey = too close for the drone centre, red = path."""
    clr = grid.clearance()
    on_path = set()
    for i in range(len(path or []) - 1):
        (x0, y0), (x1, y1) = path[i], path[i + 1]
        steps = max(2, int(math.hypot(x1 - x0, y1 - y0) / (grid.res / 2)))
        for k in range(steps + 1):
            t = k / steps
            on_path.add(grid.to_cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    rows = []
    for cy in range(grid.h - 1, -1, -1):
        row = bytearray()
        for cx in range(grid.w):
            c = clr[cy * grid.w + cx]
            if (cx, cy) in on_path:
                px = (220, 30, 30)
            elif grid.cells[cy * grid.w + cx] != FREE:
                px = (0, 0, 0)
            elif c < radius:
                px = (190, 190, 190)
            else:
                px = (255, 255, 255)
            row += bytes(px) * scale
        rows.extend([bytes([0]) + bytes(row)] * scale)
    write_png(filename, grid.w * scale, grid.h * scale, rows)
