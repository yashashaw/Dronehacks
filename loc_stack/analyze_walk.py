#!/usr/bin/env python3
"""Analyze a walk_logger run on your laptop (standard library only).

  python analyze_walk.py walk_runs/run1                    # drift / loop-closure summary
  python analyze_walk.py walk_runs/run1 --object 0.60,0.40 # known rectangle, L x W metres

How the marks are found: there is no button to press over the flaky SSH link. Instead,
HOLD THE DRONE STILL FOR ~3 s at every mark (start, each corner, end). A "dwell" is a stretch
of at least --min-dwell seconds where the VIO speed stays under --speed-thr. Dwell centroids
are your marks, in the order visited.

Everything below uses distances and angles between marks, never absolute x/y: the odom frame's
heading is whatever the rig pointed at when SLAM started (no compass), so only frame-free
quantities are meaningful.

Object mode: touch the same fixed point on the drone (e.g. the nose) to corner 1, 2, 3, 4 of
the rectangle in order (dwell ~3 s at each), then optionally back to corner 1. Edge lengths,
diagonals and corner angles from VIO are compared with your tape measurements.
"""
import argparse
import csv
import json
import math
import os


def load_odom(run):
    rows = []
    with open(os.path.join(run, 'odom.csv')) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def load_status(run):
    path = os.path.join(run, 'status.csv')
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [(float(r['recv_s']), int(r['vo_state'])) for r in csv.DictReader(f)]


def dist(a, b):
    return math.dist(a, b)


def windowed_speed(rows, win_s=0.5):
    """Speed at each sample = displacement over the last win_s seconds / elapsed time."""
    out, j = [], 0
    for i, r in enumerate(rows):
        while rows[i]['recv_s'] - rows[j]['recv_s'] > win_s:
            j += 1
        dt = r['recv_s'] - rows[j]['recv_s']
        out.append(0.0 if dt < 0.1 else
                   dist((r['x'], r['y'], r['z']), (rows[j]['x'], rows[j]['y'], rows[j]['z'])) / dt)
    return out


def find_dwells(rows, speed_thr=0.04, min_dwell=1.5):
    speed = windowed_speed(rows)
    dwells, start = [], None
    for i, s in enumerate(speed + [math.inf]):  # sentinel closes a trailing dwell
        slow = s < speed_thr
        if slow and start is None:
            start = i
        elif not slow and start is not None:
            seg = rows[start:i]
            if seg and seg[-1]['recv_s'] - seg[0]['recv_s'] >= min_dwell:
                n = len(seg)
                cen = (sum(r['x'] for r in seg) / n, sum(r['y'] for r in seg) / n,
                       sum(r['z'] for r in seg) / n)
                dwells.append({
                    'spread_m': max(dist((r['x'], r['y'], r['z']), cen) for r in seg),
                    't0': seg[0]['recv_s'], 't1': seg[-1]['recv_s'],
                    'pos': (sum(r['x'] for r in seg) / n, sum(r['y'] for r in seg) / n,
                            sum(r['z'] for r in seg) / n),
                    'first': start, 'last': i - 1})
            start = None
    return dwells


def path_length(rows, dwells=(), step_s=0.2):
    """Distance travelled while moving. Summing every 30 Hz step would add up VIO jitter
    (1 mm of noise x 600 samples = 0.6 m of fake path), so: sample every step_s and skip
    anything that falls inside a dwell."""
    still = set()
    for d in dwells:
        still.update(range(d['first'], d['last'] + 1))
    idx, last_t = [], None
    for i, r in enumerate(rows):
        if last_t is None or r['recv_s'] - last_t >= step_s:
            idx.append(i)
            last_t = r['recv_s']
    total = 0.0
    for i, j in zip(idx, idx[1:]):
        if i in still and j in still:
            continue
        a, b = rows[i], rows[j]
        total += dist((a['x'], a['y'], a['z']), (b['x'], b['y'], b['z']))
    return total


def corner_angle(prev, cur, nxt):
    v1 = [a - b for a, b in zip(prev, cur)]
    v2 = [a - b for a, b in zip(nxt, cur)]
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return float('nan')
    c = max(-1.0, min(1.0, sum(a * b for a, b in zip(v1, v2)) / (n1 * n2)))
    return math.degrees(math.acos(c))


def object_report(dwells, length, width, skip=0):
    pts = [d['pos'] for d in dwells][skip:]
    # a fifth dwell back at corner 1 is a closure check, not a fifth corner
    closure = dist(pts[0], pts[4]) if len(pts) >= 5 else None
    if len(pts) < 4:
        return {'error': f'need 4 corner dwells, found {len(pts)} (after skip={skip})'}
    c = pts[:4]
    edges = [dist(c[i], c[(i + 1) % 4]) for i in range(4)]
    diags = [dist(c[0], c[2]), dist(c[1], c[3])]
    best = None
    for exp in ([length, width, length, width], [width, length, width, length]):
        err = sum(abs(m - e) for m, e in zip(edges, exp))
        if best is None or err < best[0]:
            best = (err, exp)
    exp = best[1]
    exp_diag = math.hypot(length, width)
    return {
        'edges_m': [round(e, 4) for e in edges], 'expected_edges_m': exp,
        'edge_error_pct': [round(100 * (m - e) / e, 2) for m, e in zip(edges, exp)],
        'diagonals_m': [round(d, 4) for d in diags], 'expected_diagonal_m': round(exp_diag, 4),
        'diagonal_error_pct': [round(100 * (d - exp_diag) / exp_diag, 2) for d in diags],
        'corner_angles_deg': [round(corner_angle(c[i - 1], c[i], c[(i + 1) % 4]), 1)
                              for i in range(4)],
        'scale_factor_measured_over_true': round(sum(edges) / sum(exp), 4),
        'perimeter_measured_m': round(sum(edges), 4), 'perimeter_true_m': round(sum(exp), 4),
        'height_spread_m': round(max(p[2] for p in c) - min(p[2] for p in c), 4),
        'return_to_corner1_error_m': None if closure is None else round(closure, 4),
    }


def analyze(run, speed_thr, min_dwell, obj, skip):
    rows = load_odom(run)
    if len(rows) < 10:
        return {'error': f'only {len(rows)} odometry rows'}
    dur = rows[-1]['recv_s'] - rows[0]['recv_s']
    gaps = [b['stamp_s'] - a['stamp_s'] for a, b in zip(rows, rows[1:])]
    status = load_status(run)
    bad = [(t, s) for t, s in status if s != 1]
    dwells = find_dwells(rows, speed_thr, min_dwell)
    marks = [d['pos'] for d in dwells]
    plen = path_length(rows, dwells)
    res = {
        'run': run, 'duration_s': round(dur, 1), 'odom_rows': len(rows),
        'odom_rate_hz': round(len(rows) / dur, 1) if dur else None,
        'max_odom_gap_ms': round(max(gaps) * 1000),
        'tracking_state_changes': status,
        'tracking_lost_events': len(bad),
        'path_length_m': round(plen, 3),
        'height_span_m': round(max(r['z'] for r in rows) - min(r['z'] for r in rows), 3),
        'dwells': [{'t0': round(d['t0'], 1), 'dur_s': round(d['t1'] - d['t0'], 1),
                    'jitter_m': round(d['spread_m'], 4),
                    'pos': [round(v, 3) for v in d['pos']]} for d in dwells],
    }
    if len(marks) >= 2:
        legs = [round(dist(a, b), 3) for a, b in zip(marks, marks[1:])]
        res['legs_between_dwells_m'] = legs
        closure = dist(marks[0], marks[-1])
        res['start_to_end_error_m'] = round(closure, 3)
        res['start_to_end_error_pct_of_path'] = round(100 * closure / plen, 2) if plen else None
    if obj:
        res['object'] = object_report(dwells, obj[0], obj[1], skip)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', help='folder with odom.csv from walk_logger')
    ap.add_argument('--object', help='known rectangle "LENGTH,WIDTH" in metres')
    ap.add_argument('--skip', type=int, default=0, help='dwells to ignore before corner 1')
    ap.add_argument('--speed-thr', type=float, default=0.04)
    ap.add_argument('--min-dwell', type=float, default=1.5)
    a = ap.parse_args()
    obj = tuple(float(v) for v in a.object.split(',')) if a.object else None
    res = analyze(a.run, a.speed_thr, a.min_dwell, obj, a.skip)
    text = json.dumps(res, indent=2)
    print(text)
    with open(os.path.join(a.run, 'analysis.json'), 'w') as f:
        f.write(text + '\n')


if __name__ == '__main__':
    main()
