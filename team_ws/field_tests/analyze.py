#!/usr/bin/env python3
"""Turn one field-test run (wall_logger.py output) into report.txt, summary.json and plots.

    python3 analyze.py RUN_DIR [--test W2] [--truth 3,2.5,2,1.5,1]
    python3 analyze.py --compare RUN_DIR RUN_DIR [...]

The test id is taken from the run label (e.g. "W3_turn_01" -> W3) unless --test is given.

Truth comes from things that do not move:
  * the start mark      -> closure error (where VIO thinks you ended vs. where you started)
  * the wall            -> "wall drift": the wall's position in the VIO frame should never
                           change; when it does, that is VIO drift along the wall normal
  * tape marks (--truth) -> depth accuracy (W1) and distance scale (W2)
Needs numpy; plots need matplotlib (both are in the skybeta container).
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

STILL_SPEED = 0.06       # m/s
STILL_YAWRATE = 6.0      # deg/s
MIN_DWELL_S = 2.0
SAME_WALL_DEG = 20.0
NAV_NAMES = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 4: 'LOITER', 14: 'OFFBOARD', 17: 'TAKEOFF', 18: 'LAND'}


# ---------------------------------------------------------------------------- loading
def load_csv(path):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return {}
    hdr, body = rows[0], rows[1:]
    out = {}
    for i, k in enumerate(hdr):
        col = []
        for r in body:
            try:
                col.append(float(r[i]) if i < len(r) and r[i] != '' else np.nan)
            except ValueError:
                col.append(np.nan)
        out[k] = np.array(col)
    return out


def test_id(label):
    head = label.split('_')[0].upper()
    return head if head[:1] in 'WCF' else '?'


def nanmed(a):
    a = a[np.isfinite(a)]
    return float(np.median(a)) if a.size else float('nan')


def fmt(v, nd=2, unit=''):
    return '-' if v is None or not np.isfinite(v) else f'{v:.{nd}f}{unit}'


# ---------------------------------------------------------------------------- analysis
def odom_stats(od):
    t = od['t']
    if t.size < 3:
        return {}, None
    dt = np.diff(t)
    p = np.stack([od['x'], od['y'], od['z']], axis=1)
    step = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)
    yaw = np.degrees(np.arctan2(2 * (od['qw'] * od['qz'] + od['qx'] * od['qy']),
                                1 - 2 * (od['qy'] ** 2 + od['qz'] ** 2)))
    s = {'duration_s': float(t[-1] - t[0]), 'rate_hz': float((t.size - 1) / (t[-1] - t[0])),
         'gaps_over_100ms': int((dt > 0.1).sum()), 'max_gap_ms': float(dt.max() * 1000),
         'jumps_over_10cm': int(((step > 0.10) & (dt < 0.15)).sum()), 'max_step_m': float(step.max()),
         'path_length_m': float(step[step < 0.5].sum())}
    return s, (t, p, yaw)


def find_dwells(t, p, yaw):
    """Still segments >= MIN_DWELL_S, from 30 Hz odometry."""
    if t.size < 10:
        return []
    k = 9                                   # ~0.3 s window
    sp = np.full(t.size, np.inf)
    yr = np.full(t.size, np.inf)
    dy = np.degrees(np.unwrap(np.radians(yaw)))
    for i in range(k, t.size):
        dt = t[i] - t[i - k]
        if dt > 0:
            sp[i] = np.linalg.norm(p[i, :2] - p[i - k, :2]) / dt
            yr[i] = abs(dy[i] - dy[i - k]) / dt
    still = (sp < STILL_SPEED) & (yr < STILL_YAWRATE)
    out, i = [], 0
    while i < t.size:
        if still[i]:
            j = i
            while j + 1 < t.size and (still[j + 1] or (t[j + 1] - t[j] < 0.3 and j + 2 < t.size and still[j + 2])):
                j += 1
            if t[j] - t[i] >= MIN_DWELL_S:
                seg = slice(i, j + 1)
                out.append({'t0': float(t[i]), 't1': float(t[j]), 'x': float(p[seg, 0].mean()),
                            'y': float(p[seg, 1].mean()), 'z': float(p[seg, 2].mean()),
                            'yaw': float(np.degrees(math.atan2(np.sin(np.radians(yaw[seg])).mean(),
                                                               np.cos(np.radians(yaw[seg])).mean())))})
            i = j + 1
        else:
            i += 1
    return out


def wall_series(lg):
    """Rows with a good plane, their horizontal world normal angle and dw."""
    ok = (lg.get('plane_ok', np.array([])) == 1) & np.isfinite(lg.get('dw', np.array([])))
    if not ok.any():
        return None
    nx, ny = lg['nwx'], lg['nwy']
    ang = np.degrees(np.arctan2(ny, nx))
    return ok, ang


def same_wall(ang, ref):
    return np.abs((ang - ref + 180) % 360 - 180) < SAME_WALL_DEG


def analyze(run, test=None, truth=None, plots=True):
    lg = load_csv(os.path.join(run, 'log.csv'))
    od = load_csv(os.path.join(run, 'odom.csv'))
    sc = load_csv(os.path.join(run, 'scan.csv'))
    label = os.path.basename(run.rstrip('/')).split('_', 2)[-1] if '_' in os.path.basename(run) else run
    meta = {}
    if os.path.exists(os.path.join(run, 'meta.json')):
        meta = json.load(open(os.path.join(run, 'meta.json')))
        label = meta.get('label', label)
    test = (test or test_id(label)).upper()
    R = {'run': os.path.basename(run.rstrip('/')), 'label': label, 'test': test,
         'clean_stop': os.path.exists(os.path.join(run, 'DONE'))}
    L = [f'# {R["run"]}   test {test}', '']
    if not od or od['t'].size < 3:
        L.append('NO DATA: odom.csv is empty (was the stack running? VIO tracking?)')
        R['error'] = 'no data'
        return R, L
    if not lg or lg['t'].size == 0:
        L.append('NOTE: no depth rows (camera in the old mode?): VIO-only analysis, no wall numbers')
        cols = open(os.path.join(run, 'log.csv')).readline().strip().split(',') if os.path.exists(
            os.path.join(run, 'log.csv')) else ['t']
        lg = {k: np.array([]) for k in cols + ['plane_ok', 'dw', 'nwx', 'nwy', 'armed', 'nav_state', 'valid_frac',
                                               'wall_dist_m', 'wall_yaw_deg', 'vio_x', 'vio_y', 'fc_x', 'fc_y',
                                               'dist_bottom', 'vo_state']}

    st, traj = odom_stats(od)
    R['vio'] = st
    t, p, yaw = traj
    lt = lg['t']
    R['depth_rows'] = int(lt.size)
    R['plane_ok_pct'] = float(100 * np.mean(lg['plane_ok'] == 1)) if lt.size else 0.0
    R['valid_frac_median'] = nanmed(lg['valid_frac']) if lt.size else float('nan')
    L += ['## Data health',
          f'duration {st["duration_s"]:.0f} s | VIO {st["rate_hz"]:.1f} Hz, gaps >100 ms: {st["gaps_over_100ms"]} '
          f'(max {st["max_gap_ms"]:.0f} ms), jumps >10 cm: {st["jumps_over_10cm"]} (max step {st["max_step_m"]*100:.1f} cm)',
          f'depth rows {lt.size}, wall plane found in {R["plane_ok_pct"]:.0f}% of them, '
          f'valid depth pixels (median) {100*R["valid_frac_median"]:.0f}%',
          f'path length (VIO) {st["path_length_m"]:.2f} m' + ('' if R['clean_stop'] else
                                                               '   WARNING: no DONE file, logger did not stop cleanly'),
          '']
    vs = lg.get('vo_state')
    if vs is not None and np.isfinite(vs).any():
        bad = np.isfinite(vs) & (vs >= 0) & (vs != 1)
        R['vo_not_tracking_rows'] = int(bad.sum())
        if bad.any():
            L.append(f'WARNING: VIO not tracking in {bad.sum()} rows')

    # ---- wall reference
    ws = wall_series(lg)
    dws = []
    ref_ang = None
    if ws is not None:
        ok, ang = ws
        ref_ang = float(ang[ok][0])
        same = ok & same_wall(ang, ref_ang)
        dw0 = nanmed(lg['dw'][same][:10])
        drift = np.where(same, lg['dw'] - dw0, np.nan)
        R['wall'] = {'ref_normal_deg': ref_ang, 'rows_same_wall': int(same.sum()),
                     'drift_max_abs_cm': float(np.nanmax(np.abs(drift)) * 100),
                     'drift_final_cm': float(drift[same][-1] * 100) if same.any() else None}
    else:
        drift = np.full(lt.size, np.nan)
        same = np.zeros(lt.size, bool)

    # ---- dwells
    dwells = find_dwells(t, p, yaw)
    for d in dwells:
        m = (lt >= d['t0'] + 0.3) & (lt <= d['t1'] - 0.3)
        d['wall_dist'] = nanmed(np.where(m & (lg['plane_ok'] == 1), lg['wall_dist_m'], np.nan))
        d['wall_yaw'] = nanmed(np.where(m & (lg['plane_ok'] == 1), lg['wall_yaw_deg'], np.nan))
        d['drift_cm'] = nanmed(np.where(m & same, drift, np.nan)) * 100
        d['armed'] = bool(np.nanmax(np.where(m, lg['armed'], 0)) > 0) if m.any() else False
    R['dwells'] = dwells
    L += ['## Still points (held >= 2 s)',
          ' #   time  dur    x      y     yaw | wall dist  wall angle | wall drift',
          ]
    t00 = t[0]
    for i, d in enumerate(dwells, 1):
        L.append(f'{i:2d} {d["t0"]-t00:6.1f} {d["t1"]-d["t0"]:4.1f} {d["x"]:6.2f} {d["y"]:6.2f} {d["yaw"]:6.0f} |'
                 f' {fmt(d["wall_dist"], 3, " m"):>9}  {fmt(d["wall_yaw"], 0, " deg"):>9} | {fmt(d["drift_cm"], 1, " cm"):>8}')
    if not dwells:
        L.append('(none found: hold the drone still for 3 s at each mark)')
    L.append('')

    # ---- closure
    if len(dwells) >= 2:
        a, b = dwells[0], dwells[-1]
        err = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        R['closure'] = {'xy_error_m': err, 'z_error_m': b['z'] - a['z'],
                        'yaw_error_deg': (b['yaw'] - a['yaw'] + 180) % 360 - 180,
                        'pct_of_path': 100 * err / max(st['path_length_m'], 1e-6)}
        c = R['closure']
        L += ['## Closure (first still point vs last still point)',
              f'if you ended on the start mark, this is pure VIO drift: {c["xy_error_m"]*100:.1f} cm '
              f'({c["pct_of_path"]:.1f}% of {st["path_length_m"]:.1f} m), height {c["z_error_m"]*100:+.1f} cm, '
              f'heading {c["yaw_error_deg"]:+.1f} deg', '']

    if 'wall' in R:
        w = R['wall']
        L += ['## Wall drift (the wall cannot move, so any change is VIO error along the wall normal)',
              f'max {w["drift_max_abs_cm"]:.1f} cm, at the end {fmt(w["drift_final_cm"], 1, " cm")} '
              f'({w["rows_same_wall"]} rows saw the first wall)', '']

    # ---- test-specific
    tv = [d for d in dwells if np.isfinite(d['wall_dist'])]
    if test == 'W1':
        L.append('## W1 depth accuracy vs tape')
        if truth:
            res = []
            for i, tr in enumerate(truth):
                if i < len(tv):
                    e = tv[i]['wall_dist'] - tr
                    res.append({'truth_m': tr, 'depth_m': tv[i]['wall_dist'], 'error_cm': e * 100,
                                'wall_angle_deg': tv[i]['wall_yaw']})
                    L.append(f'tape {tr:.2f} m -> depth {tv[i]["wall_dist"]:.3f} m  error {e*100:+.1f} cm '
                             f'(wall angle {fmt(tv[i]["wall_yaw"], 0)} deg)')
                else:
                    L.append(f'tape {tr:.2f} m -> no still point with the wall in view')
            R['W1'] = res
        else:
            L.append('pass --truth 0.5,1,1.5,2,3 (tape distances, in the order you stood) to score it')
        L.append('note: the camera depth origin is about 1 cm behind the front glass; measure tape to the glass')
        L.append('')
    if test == 'W2' and len(tv) >= 2 and ref_ang is not None:
        n = np.array([math.cos(math.radians(ref_ang)), math.sin(math.radians(ref_ang))])
        L.append('## W2 distance scale: VIO movement toward the wall vs depth change')
        rat = []
        for a, b in zip(tv, tv[1:]):
            dv = (b['x'] - a['x']) * n[0] + (b['y'] - a['y']) * n[1]
            dd = a['wall_dist'] - b['wall_dist']
            if abs(dd) > 0.2:
                rat.append(dv / dd)
                L.append(f'depth says {dd*100:+.0f} cm, VIO says {dv*100:+.0f} cm  -> VIO/depth {dv/dd:.3f}')
        if rat:
            R['W2_scale'] = float(np.median(rat))
            L.append(f'VIO distance scale (median): {R["W2_scale"]:.3f}  '
                     f'({(R["W2_scale"]-1)*100:+.1f}%: negative = VIO measures short)')
        if truth and len(truth) == len(tv):
            for tr, d in zip(truth, tv):
                L.append(f'tape {tr:.2f} m vs depth {d["wall_dist"]:.3f} m ({(d["wall_dist"]-tr)*100:+.1f} cm)')
        L.append('')
    if test in ('W3', 'F3', 'F4') and len(tv) >= 2:
        L.append('## Turn test: wall drift at each still point facing the wall (should all be ~0)')
        for i, d in enumerate(tv, 1):
            L.append(f'  facing-wall stop {i}: heading {d["yaw"]:6.0f} deg, drift {fmt(d["drift_cm"], 1, " cm")}')
        jumps = [abs(b['drift_cm'] - a['drift_cm']) for a, b in zip(tv, tv[1:])
                 if np.isfinite(a['drift_cm']) and np.isfinite(b['drift_cm'])]
        if jumps:
            R['turn_jump_max_cm'] = float(max(jumps))
            L.append(f'biggest change between stops: {max(jumps):.1f} cm  (this is the turn-induced position error)')
        L.append('')
    if test in ('W4', 'F6') and same.any():
        L.append('## Sideways / parallel: wall drift while moving')
        moving = same & ~np.any([(lt >= d['t0']) & (lt <= d['t1']) for d in dwells] or [np.zeros(lt.size, bool)], axis=0)
        if moving.any():
            dm = drift[moving] * 100
            R['moving_drift_cm'] = {'p5': float(np.nanpercentile(dm, 5)), 'p95': float(np.nanpercentile(dm, 95))}
            L.append(f'wall drift while moving: {R["moving_drift_cm"]["p5"]:+.1f} .. {R["moving_drift_cm"]["p95"]:+.1f} cm '
                     f'(5th..95th percentile)')
        L.append('')
    if test in ('W5', 'C') or test.startswith('C'):
        L.append('## Walls seen (grouped by direction and position in the VIO frame)')
        if ws is not None:
            ok, ang = ws
            idx = np.nonzero(ok)[0]
            groups = []
            for i in idx:
                for g in groups:
                    if same_wall(np.array([ang[i]]), g['ang'])[0] and abs(lg['dw'][i] - g['dw']) < 0.3:
                        g['rows'].append(i)
                        break
                else:
                    groups.append({'ang': float(ang[i]), 'dw': float(lg['dw'][i]), 'rows': [i]})
            groups = [g for g in groups if len(g['rows']) >= 10]
            R['walls'] = []
            for k, g in enumerate(groups, 1):
                r = np.array(g['rows'])
                v = lg['dw'][r]
                span = float(lt[r[-1]] - lt[r[0]])
                spread = float(np.percentile(v, 95) - np.percentile(v, 5))
                R['walls'].append({'normal_deg': g['ang'], 'offset_m': float(np.median(v)), 'rows': int(r.size),
                                   'seen_over_s': span, 'spread_cm': spread * 100})
                L.append(f'wall {k}: facing {g["ang"]:5.0f} deg, offset {np.median(v):6.2f} m, {r.size} rows over '
                         f'{span:5.0f} s, position spread {spread*100:5.1f} cm  '
                         f'{"<- seen again later: spread = drift between sightings" if span > 20 else ""}')
        L.append('')
    if test.startswith('F') and lt.size:
        arm = lg['armed'] == 1
        R['flight'] = {'armed_s': float(arm.sum() / max(1e-6, (lt.size / max(lt[-1] - lt[0], 1e-6)))),
                       'nav_states': sorted({NAV_NAMES.get(int(v), str(int(v))) for v in lg['nav_state'][np.isfinite(lg['nav_state'])] if v >= 0})}
        L.append('## Flight')
        L.append(f'armed for ~{R["flight"]["armed_s"]:.0f} s, modes seen: {", ".join(R["flight"]["nav_states"]) or "-"}')
        fx, fy = lg['fc_x'], lg['fc_y']
        mm = arm & np.isfinite(fx) & np.isfinite(lg['vio_x'])
        if mm.sum() > 20:
            A = np.stack([fy[mm], fx[mm]], 1)                  # PX4 NED -> ENU-like (east, north)
            B = np.stack([lg['vio_x'][mm], lg['vio_y'][mm]], 1)
            Ac, Bc = A - A.mean(0), B - B.mean(0)
            U, _, Vt = np.linalg.svd(Ac.T @ Bc)
            Rr = (U @ Vt).T
            res = np.linalg.norm(Bc - Ac @ Rr.T, axis=1)
            R['flight']['vio_vs_ekf2_rms_cm'] = float(np.sqrt(np.mean(res ** 2)) * 100)
            R['flight']['vio_vs_ekf2_max_cm'] = float(res.max() * 100)
            L.append(f'VIO vs PX4 EKF2 position (after best rotation): rms {R["flight"]["vio_vs_ekf2_rms_cm"]:.1f} cm, '
                     f'max {R["flight"]["vio_vs_ekf2_max_cm"]:.1f} cm  (big = EKF2 not following VIO)')
        db = lg['dist_bottom'][arm]
        if np.isfinite(db).any():
            L.append(f'rangefinder height while armed: {np.nanmin(db):.2f} .. {np.nanmax(db):.2f} m')
        if 'wall' in R:
            held = [d for d in dwells if d['armed'] and np.isfinite(d['drift_cm'])]
            if held:
                L.append('wall drift at each hold in the air: ' + ', '.join(f'{d["drift_cm"]:+.1f}' for d in held) + ' cm')
        L.append('')

    # ---- verdict lines
    flags = []
    if st['rate_hz'] < 28:
        flags.append(f'VIO rate {st["rate_hz"]:.1f} Hz (< 28)')
    if st['jumps_over_10cm']:
        flags.append(f'{st["jumps_over_10cm"]} VIO jumps > 10 cm')
    if R['plane_ok_pct'] < 30 and test not in ('W0',):
        flags.append(f'wall plane found in only {R["plane_ok_pct"]:.0f}% of frames (camera not on a wall? plain wall?)')
    if 'closure' in R and R['closure']['pct_of_path'] > 5 and st['path_length_m'] > 2:
        flags.append(f'closure {R["closure"]["pct_of_path"]:.1f}% of path (> 5%)')
    if R.get('wall', {}).get('drift_max_abs_cm', 0) > 15:
        flags.append(f'wall drift up to {R["wall"]["drift_max_abs_cm"]:.0f} cm')
    R['flags'] = flags
    L += ['## Flags'] + ([f'- {f}' for f in flags] if flags else ['none']) + ['']

    if plots:
        try:
            make_plots(run, lg, t, p, dwells, sc, drift, same)
            L.append('plots: path.png (top view, walls as grey dots), wall.png (distance + drift over time)')
        except Exception as e:                      # plots are a bonus, never fail the report
            L.append(f'(plots skipped: {e})')
    return R, L


def make_plots(run, lg, t, p, dwells, sc, drift, same):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    if sc:
        k = max(1, sc['xw'].size // 20000)
        ax.scatter(sc['xw'][::k], sc['yw'][::k], s=1, c='0.6', label='depth scan (walls)')
    ax.plot(p[:, 0], p[:, 1], lw=1.2, label='VIO path')
    for i, d in enumerate(dwells, 1):
        ax.plot(d['x'], d['y'], 'o', ms=5, c='C3')
        ax.annotate(str(i), (d['x'], d['y']), textcoords='offset points', xytext=(4, 4), fontsize=8)
    ax.plot(p[0, 0], p[0, 1], 's', c='g', label='start')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('VIO x (m, forward at start)')
    ax.set_ylabel('VIO y (m, left at start)')
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(run, 'path.png'), dpi=110)
    plt.close(fig)

    lt = lg['t'] - t[0]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    ok = lg['plane_ok'] == 1
    a1.plot(lt[ok], lg['wall_dist_m'][ok], '.', ms=2)
    a1.set_ylabel('wall distance (m)')
    a2.plot(lt[same], drift[same] * 100, '.', ms=2, c='C1')
    a2.axhline(0, c='0.5', lw=0.8)
    a2.set_ylabel('wall drift (cm)')
    a2.set_xlabel('time (s)')
    for d in dwells:
        for a in (a1, a2):
            a.axvspan(d['t0'] - t[0], d['t1'] - t[0], color='0.9')
    arm = lg['armed'] == 1
    if arm.any():
        a1.plot(lt[arm], np.full(arm.sum(), np.nanmax(lg['wall_dist_m'][ok]) if ok.any() else 1), 'r|', ms=4,
                label='armed')
        a1.legend(fontsize=8)
    for a in (a1, a2):
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(run, 'wall.png'), dpi=110)
    plt.close(fig)


def compare(runs):
    rows, L = [], ['# Comparison', '',
                   'run                                   test  VIO Hz  jumps  wall%  path m  closure cm  closure %  wall drift max cm']
    for r in runs:
        R, _ = analyze(r, plots=False)
        rows.append(R)
        v, c, w = R.get('vio', {}), R.get('closure', {}), R.get('wall', {})
        L.append(f'{R["run"][:37]:37} {R["test"]:5} {fmt(v.get("rate_hz"), 1):>6} {v.get("jumps_over_10cm", "-"):>6} '
                 f'{fmt(R.get("plane_ok_pct"), 0):>6} {fmt(v.get("path_length_m"), 1):>7} '
                 f'{fmt(c.get("xy_error_m", float("nan")) * 100, 1):>11} {fmt(c.get("pct_of_path"), 1):>10} '
                 f'{fmt(w.get("drift_max_abs_cm"), 1):>18}')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        for i, r in enumerate(runs):
            od, sc = load_csv(os.path.join(r, 'odom.csv')), load_csv(os.path.join(r, 'scan.csv'))
            if sc:
                k = max(1, sc['xw'].size // 8000)
                ax.scatter(sc['xw'][::k], sc['yw'][::k], s=1, c=f'C{i}', alpha=0.3)
            if od:
                ax.plot(od['x'], od['y'], c=f'C{i}', lw=1.2, label=os.path.basename(r.rstrip('/')))
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
        ax.set_title('Same start mark + heading -> walls should land on top of each other')
        fig.tight_layout()
        out = os.path.join(os.path.dirname(runs[0].rstrip('/')), 'compare.png')
        fig.savefig(out, dpi=110)
        L += ['', f'overlay: {out}']
    except Exception as e:
        L.append(f'(overlay skipped: {e})')
    return rows, L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--test')
    ap.add_argument('--truth', help='comma-separated tape distances in metres, in order')
    ap.add_argument('--compare', action='store_true')
    a = ap.parse_args()
    if a.compare:
        _, L = compare(a.runs)
        print('\n'.join(L))
        return
    truth = [float(x) for x in a.truth.split(',')] if a.truth else None
    for run in a.runs:
        R, L = analyze(run, a.test, truth)
        text = '\n'.join(L)
        print(text)
        with open(os.path.join(run, 'report.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(run, 'summary.json'), 'w') as f:
            json.dump(R, f, indent=1, default=float)


if __name__ == '__main__':
    sys.exit(main())
