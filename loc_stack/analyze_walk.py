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


# Two-sided 95 % Student-t critical values, df 1..30 (then ~1.96).
_T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228,
        2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086,
        2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042]


def line_report(dwells, length, skip=0, expect_legs=None, claim=0.884, truth_err_m=0.01):
    """Back-and-forth between two floor marks LENGTH apart, holding at each mark.

    Every leg between consecutive holds should be LENGTH. Returns per-leg scale
    (measured / true), the mean with a 95 % t-interval, and whether that interval
    excludes 1.0 (a real bias) and `claim` (the earlier 11.6 %-short reading).
    Fails loudly when the number of holds is not what you walked, because a lost
    or extra hold silently pairs the wrong marks.
    """
    pts = [d['pos'] for d in dwells][skip:]
    legs = len(pts) - 1
    if expect_legs is not None and legs != expect_legs:
        return {'error': f'expected {expect_legs} legs ({expect_legs + 1} holds), found '
                         f'{max(legs, 0)} legs ({len(pts)} holds after skip={skip}). Check '
                         'the dwell list: hold 5 s at each mark and move steadily between '
                         'them; use --skip for a stray hold before the first mark.'}
    if legs < 2:
        return {'error': f'need at least 3 holds, found {len(pts)}'}
    l3 = [dist(a, b) for a, b in zip(pts, pts[1:])]
    lh = [math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(pts, pts[1:])]
    ratios = [v / length for v in l3]
    n = len(ratios)
    mean = sum(ratios) / n
    sd = math.sqrt(sum((r - mean) ** 2 for r in ratios) / (n - 1))
    stat = (_T95[n - 2] if n - 1 <= len(_T95) else 1.96) * sd / math.sqrt(n)
    # The true length is only known to +/- truth_err_m (tape and mark placement), and
    # that error is shared by every leg, so averaging does not shrink it.
    half = math.hypot(stat, truth_err_m / length)
    lo, hi = mean - half, mean + half
    # marks alternate A, B, A, B ... so every even hold is back at the start mark
    returns = [round(dist(pts[0], p), 4) for p in pts[2::2]]
    if lo > 1.0 or hi < 1.0:
        bias = f'real scale bias: {100 * (mean - 1):+.1f} % (95 % interval excludes 1.0)'
    else:
        bias = 'no significant scale bias (95 % interval includes 1.0)'
    claim_txt = (f'{claim:.3f} is {"OUTSIDE" if (claim < lo or claim > hi) else "inside"} '
                 'the interval')
    return {
        'true_length_m': length, 'legs': n,
        'legs_3d_m': [round(v, 4) for v in l3],
        'legs_horizontal_m': [round(v, 4) for v in lh],
        'scale_per_leg': [round(r, 4) for r in ratios],
        'scale_mean': round(mean, 4), 'scale_sd': round(sd, 4),
        'scale_95ci': [round(lo, 4), round(hi, 4)],
        'scale_error_pct': round(100 * (mean - 1), 2),
        'verdict': f'{bias}; {claim_txt}',
        'return_to_start_errors_m': returns,
        'height_spread_m': round(max(p[2] for p in pts) - min(p[2] for p in pts), 4),
    }


def analyze(run, speed_thr, min_dwell, obj, skip, line=None, expect_legs=None):
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
    if line:
        res['line'] = line_report(dwells, line, skip, expect_legs)
    cfg = os.path.join(run, 'config.txt')
    if os.path.exists(cfg):
        with open(cfg) as f:
            res['slam_config'] = f.read().strip()
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', help='folder with odom.csv from walk_logger')
    ap.add_argument('--object', help='known rectangle "LENGTH,WIDTH" in metres')
    ap.add_argument('--line', type=float,
                    help='back-and-forth between two marks this many metres apart')
    ap.add_argument('--legs', type=int, help='legs you walked (--line); fails if it differs')
    ap.add_argument('--skip', type=int, default=0, help='dwells to ignore before corner 1')
    ap.add_argument('--speed-thr', type=float, default=0.04)
    ap.add_argument('--min-dwell', type=float, default=1.5)
    a = ap.parse_args()
    obj = tuple(float(v) for v in a.object.split(',')) if a.object else None
    res = analyze(a.run, a.speed_thr, a.min_dwell, obj, a.skip, a.line, a.legs)
    text = json.dumps(res, indent=2)
    print(text)
    with open(os.path.join(a.run, 'analysis.json'), 'w') as f:
        f.write(text + '\n')


if __name__ == '__main__':
    main()
