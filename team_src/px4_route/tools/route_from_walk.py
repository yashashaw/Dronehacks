#!/usr/bin/env python3
"""Turn a hand-carried walk (VIO path) into a px4_route route file. Runs on the laptop.

    python route_from_walk.py WALK/vio.csv --timeline                  # find the window to use
    python route_from_walk.py WALK/vio.csv --t0 12 --t1 118 --out ../routes/course.yaml --plot course.png

WALK/vio.csv comes from drone_tests/walks/extract_walk.py (columns t, stamp, x, y, z, ...).

Why VIO coordinates work directly as the route's START frame: VIO's origin is where the drone sat
when the stack started, and its +x is the way the nose pointed then. So if the walk started with
the drone powered up ON THE LAUNCH MARK, FACING THE WAY IT WILL TAKE OFF, VIO (x, y) = (forward,
left) from the takeoff point, which is exactly what px4_route expects. Before flying, put the
drone on the same spot with the same heading.

Heights: --height H gives every waypoint the same height. --heights walk takes each waypoint's
height from the walk's RANGEFINDER (lpos.csv dist_bottom next to vio.csv, median over +-1 s),
clamped to --min-height..--max-height. Use it where you deliberately carried the drone low (tunnel).
Never VIO's height: on the 21:55 walk it read 0.3-0.5 m low.
The path is simplified (Ramer-Douglas-Peucker, --tol) so it stays within --tol of the walk.
"""
import argparse
import csv
import math
import sys


def load_vio(path):
    t, x, y = [], [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row['t'])); x.append(float(row['x'])); y.append(float(row['y']))
    if not t:
        sys.exit(f'{path}: no rows')
    t0 = t[0]
    return [ti - t0 for ti in t], x, y


def load_rangefinder(vio_csv):
    """(t, dist_bottom) from lpos.csv beside vio.csv, on the same clock as load_vio, or None."""
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(vio_csv)), 'lpos.csv')
    if not os.path.exists(path):
        return None
    with open(vio_csv) as f:
        tv0 = float(next(csv.DictReader(f))['t'])
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            out.append((float(row['t']) - tv0, float(row['dist_bottom'])))
    return out


def height_at(rng, t, half=1.0):
    vals = sorted(d for ti, d in rng if t - half <= ti <= t + half and d > 0.02)
    return vals[len(vals) // 2] if vals else None


def rdp(pts, tol):
    """Ramer-Douglas-Peucker on the first two coordinates: keep the corners, drop points within
    tol of a line. Extra coordinates (the time) ride along."""
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    best, idx = -1.0, 0
    for i in range(1, len(pts) - 1):
        p = pts[i]
        d = (abs(dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / n) if n > 1e-9 else math.dist(p[:2], a[:2])
        if d > best:
            best, idx = d, i
    if best <= tol:
        return [a, b]
    return rdp(pts[:idx + 1], tol)[:-1] + rdp(pts[idx:], tol)


def window(t, x, y, t0, t1):
    return [(xi, yi, ti) for ti, xi, yi in zip(t, x, y) if t0 <= ti <= t1]


def print_timeline(t, x, y):
    print(' time   x(fwd)  y(left)  walked   note')
    dist, last, left_start = 0.0, (x[0], y[0]), False
    next_print = 0.0
    for ti, xi, yi in zip(t, x, y):
        dist += math.dist(last, (xi, yi)); last = (xi, yi)
        r = math.hypot(xi - x[0], yi - y[0])
        note = ''
        if r > 2.0:
            left_start = True
        if left_start and r < 0.6:
            note, left_start = f'<- back near the start ({r:.2f} m)', False
        if ti >= next_print or note:
            print(f'{ti:6.1f} {xi:7.2f} {yi:7.2f} {dist:7.1f} m  {note}')
            next_print = ti + 5.0


def write_route(path, wps, a, src):
    hsrc = (f'heights from the walk rangefinder, clamped {a.min_height}..{a.max_height} m'
            if a.heights == 'walk' else f'all heights {a.height} m')
    lines = [
        f'# Route from walk {src}, t = {a.t0:.1f}..{a.t1:.1f} s, simplified to within {a.tol} m; {hsrc}.',
        '# Start frame: metres from the takeoff point. x = forward (nose at takeoff), y = left, z = up.',
        '# Put the drone on the SAME spot, facing the SAME way, as when the walk started.',
        f'name: {a.name}',
        f'takeoff_height: {wps[0][2]:.2f}',
        f'speed: {a.speed}',
        f'corner_speed: {a.corner_speed}',
        f'max_height: {a.max_height}',
        f'max_cross_track: {a.max_cross_track}',
        'yaw: fixed',
        f'end: {a.end}',
        'waypoints:',
    ] + [f'  - [{wx:.2f}, {wy:.2f}, {wz:.2f}]' for wx, wy, wz in wps]
    with open(path, 'w', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')


def plot(path, t, x, y, wps, a):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (ax, az) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={'width_ratios': [1, 1]})
    ax.plot(x, y, color='0.8', lw=0.8, label='whole walk (VIO)')
    seg = window(t, x, y, a.t0, a.t1)
    ax.plot([p[0] for p in seg], [p[1] for p in seg], color='tab:blue', lw=1.2, label=f'used: {a.t0:.0f}-{a.t1:.0f} s')
    ax.plot([0] + [w[0] for w in wps], [0] + [w[1] for w in wps], 'o-', color='tab:red', ms=4, lw=1, label='route')
    for i, w in enumerate(wps, 1):
        ax.annotate(f'{i}', (w[0], w[1]), fontsize=7, xytext=(3, 3), textcoords='offset points')
    dist = [0.0]
    for p, q in zip([(0.0, 0.0)] + [w[:2] for w in wps], [w[:2] for w in wps]):
        dist.append(dist[-1] + math.dist(p, q))
    rng = load_rangefinder(a.vio_csv)
    if rng:                                 # what the walk's rangefinder saw, for reference
        wd, wh, last, acc = [], [], (0.0, 0.0), 0.0
        for px, py, pt in seg[::3]:
            acc += math.dist(last, (px, py)); last = (px, py)
            h = height_at(rng, pt, 0.3)
            if h is not None:
                wd.append(acc); wh.append(h)
        az.plot(wd, wh, color='0.6', lw=0.8, label='walk rangefinder (hand-carried)')
    az.plot(dist, [0.0] + [w[2] for w in wps], 'o-', color='tab:red', ms=3, label='route height')
    az.axhline(1.73, color='k', ls='--', lw=0.8); az.text(0.1, 1.76, 'top of walls 1.73 m', fontsize=8)
    az.set_xlabel('distance along the route (m)'); az.set_ylabel('height above the floor (m)')
    az.set_ylim(0, 2.0); az.grid(alpha=0.3); az.legend(fontsize=8)
    ax.plot(0, 0, 'k*', ms=14, label='takeoff (walk start)')
    ax.annotate('', xy=(0.5, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', lw=2))
    ax.text(0.5, 0.08, 'nose', fontsize=8)
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_xlabel('x = forward (m)'); ax.set_ylabel('y = left (m)')
    ax.set_title(f'{a.name}: {len(wps)} waypoints')
    fig.tight_layout(); fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('vio_csv')
    ap.add_argument('--timeline', action='store_true', help='print positions every 5 s and exit')
    ap.add_argument('--t0', type=float, help='start of the part to use (s); default: when the walk starts moving')
    ap.add_argument('--t1', type=float, help='end of the part to use (s); default: end of file')
    ap.add_argument('--out', help='route YAML to write')
    ap.add_argument('--plot', help='PNG of the walk and the route')
    ap.add_argument('--name', default='course-from-walk')
    ap.add_argument('--heights', choices=['fixed', 'walk'], default='fixed',
                    help='fixed: every waypoint at --height. walk: from the walk rangefinder (lpos.csv)')
    ap.add_argument('--height', type=float, default=0.9, help='height for --heights fixed (m)')
    ap.add_argument('--min-height', type=float, default=0.5, help='--heights walk: never lower (m)')
    ap.add_argument('--tol', type=float, default=0.10, help='route stays within this of the walk (m)')
    ap.add_argument('--min-leg', type=float, default=0.25, help='merge waypoints closer than this (m)')
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--corner-speed', type=float, default=0.2)
    ap.add_argument('--max-height', type=float, default=1.5)
    ap.add_argument('--max-cross-track', type=float, default=0.4)
    ap.add_argument('--end', choices=['land', 'hold'], default='land')
    a = ap.parse_args()

    t, x, y = load_vio(a.vio_csv)
    if a.timeline:
        print_timeline(t, x, y)
        return
    if a.t0 is None:
        a.t0 = next((ti for ti, xi, yi in zip(t, x, y) if math.hypot(xi - x[0], yi - y[0]) > 0.15), 0.0)
        a.t0 = max(0.0, a.t0 - 1.0)
    if a.t1 is None:
        a.t1 = t[-1]
    pts = window(t, x, y, a.t0, a.t1)
    if len(pts) < 2:
        sys.exit('no VIO samples in that window')
    r0 = math.hypot(pts[0][0], pts[0][1])
    if r0 > 0.3:
        print(f'WARNING: at t0 the walk is {r0:.2f} m from VIO\'s origin. The route starts at the takeoff '
              'point, so the first leg will fly from the takeoff point to there.', file=sys.stderr)
    simple = rdp(pts, a.tol)
    keep = []
    for p in simple[1:]:
        if math.dist(p[:2], keep[-1][:2] if keep else (0.0, 0.0)) >= a.min_leg:
            keep.append(p)
    if simple and (not keep or keep[-1] is not simple[-1]):
        keep.append(simple[-1])          # always keep the true end point
    rng = load_rangefinder(a.vio_csv) if a.heights == 'walk' else None
    if a.heights == 'walk' and rng is None:
        sys.exit('--heights walk needs lpos.csv next to vio.csv')
    wps = []
    for px, py, pt in keep:
        if rng is None:
            z = a.height
        else:
            h = height_at(rng, pt)
            z = a.height if h is None else min(max(h, a.min_height), a.max_height)
        wps.append((px, py, round(z, 2)))
    if a.heights == 'walk':
        # waypoints only sample the walk at corners: also add the LOWEST rangefinder point of each
        # leg if it is >0.15 m below both ends, so a low passage (tunnel) is not skipped
        out = []
        prev = (0.0, 0.0, wps[0][2], a.t0)
        for (px, py, pz), (_, _, pt) in zip(wps, keep):
            seg = [q for q in pts if prev[3] < q[2] < pt]
            if seg:
                lows = [(height_at(rng, q[2], 0.5) or 9.0, q) for q in seg[::5]]
                h, q = min(lows, key=lambda v: v[0])
                if h < min(prev[2], pz) - 0.15 and h < 9.0:
                    out.append((q[0], q[1], round(min(max(h, a.min_height), a.max_height), 2)))
            out.append((px, py, pz))
            prev = (px, py, pz, pt)
        wps = out
    length = sum(math.dist(p[:2], q[:2]) for p, q in zip([(0.0, 0.0, 0.0)] + wps, wps))
    print(f'{len(pts)} VIO samples, {a.t0:.1f}..{a.t1:.1f} s -> {len(wps)} waypoints, {length:.1f} m '
          f'(walk ends {math.hypot(pts[-1][0], pts[-1][1]):.2f} m from the takeoff point); heights '
          f'{min(w[2] for w in wps):.2f}..{max(w[2] for w in wps):.2f} m')
    if a.out:
        write_route(a.out, wps, a, a.vio_csv)
        print(f'wrote {a.out}')
    if a.plot:
        plot(a.plot, t, x, y, wps, a)
        print(f'wrote {a.plot}')


if __name__ == '__main__':
    main()
