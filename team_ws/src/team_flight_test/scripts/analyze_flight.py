#!/usr/bin/env python3
"""Turn a flight_test log directory into the numbers we need. Plain Python 3, no numpy.

    python3 analyze_flight.py --latest            # newest run in ~/team_ws/flight_logs
    python3 analyze_flight.py <run directory>

Prints a report and writes summary.json next to the logs. Sections:
  1. what happened (events: LAND reason, warnings, legs reached)
  2. VIO health: rate, gaps, jumps (and which phase they happened in)
  3. tracking: setpoint vs VIO pose, per route phase (hold/turn/settle/fly)
  4. turns: how far the VIO position moved while the setpoint was fixed, heading lag
  5. VIO vs PX4 EKF2: offset at release and how much it drifted (frame/anchoring errors)
  6. FLAGS: what to look at first
"""
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict

DEFAULT_ROOT = os.path.expanduser('~/team_ws/flight_logs')


def pct(vals, q):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return None
    k = (len(v) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def fnum(s):
    try:
        x = float(s)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fmt(x, nd=2, unit=''):
    return 'n/a' if x is None else ('%.*f%s' % (nd, x, unit))


def load(run):
    with open(os.path.join(run, 'ticks.csv')) as fh:
        ticks = list(csv.DictReader(fh))
    for r in ticks:
        for k in list(r):
            if k not in ('mphase', 'phase', 'tl_mode', 'nav_state', 'step'):
                r[k] = fnum(r[k])
    odom = []
    p = os.path.join(run, 'odom.csv')
    if os.path.exists(p):
        with open(p) as fh:
            odom = [{k: fnum(v) for k, v in r.items()} for r in csv.DictReader(fh)]
    events = open(os.path.join(run, 'events.log')).read().splitlines() if os.path.exists(
        os.path.join(run, 'events.log')) else []
    cfg = json.load(open(os.path.join(run, 'config.json'))) if os.path.exists(
        os.path.join(run, 'config.json')) else {}
    return ticks, odom, events, cfg


def phase_at(ticks, t):
    best = None
    for r in ticks:
        if r['t'] is not None and r['t'] <= t:
            best = r
        else:
            break
    return (best['mphase'] + '/' + best['phase']) if best else '?'


def analyze(run):
    ticks, odom, events, cfg = load(run)
    S, flags, out = {}, [], []
    P = out.append
    P('=' * 72)
    P('flight_test analysis: %s' % run)
    P('profile %s  scale %s  speed %s  yaw_rate %s deg/s  steps %d' % (
        cfg.get('profile'), cfg.get('scale'), cfg.get('speed'), cfg.get('yaw_rate_dps'), len(cfg.get('steps', []))))
    P('=' * 72)

    # 1. events
    P('\n## 1. What happened')
    keys = ('LAND', 'WARN', 'ERROR', 'reached', 'mode ->', 'calling', 'landed:', 'route ')
    for line in events:
        if any(k in line for k in keys):
            P('  ' + line)
    land = [l for l in events if 'LAND:' in l or 'landing:' in l]
    S['land_reason'] = land[0].split(':', 1)[-1].strip() if land else None
    reached = [l for l in events if 'reached in' in l]
    S['steps_reached'] = len(reached)
    route_rows = [r for r in ticks if r['mphase'] == 'route']
    if not route_rows:
        flags.append('Route never started: check the mode lines above (Offboard? takeoff accepted? hover?).')
    if S['land_reason'] and any(w in S['land_reason'] for w in ('stale', 'jump', 'track', 'altitude', 'from start')):
        flags.append('Watchdog landed the drone: %s' % S['land_reason'])

    # 2. VIO health
    P('\n## 2. VIO odometry health')
    if len(odom) > 2:
        ts = [r['t'] for r in odom]
        dts = [b - a for a, b in zip(ts, ts[1:])]
        dur = ts[-1] - ts[0]
        S['vio_hz'] = (len(ts) - 1) / dur if dur > 0 else None
        S['vio_gaps_100ms'] = sum(1 for d in dts if d > 0.1)
        S['vio_max_gap_ms'] = 1000 * max(dts)
        steps = [r['step_m'] for r in odom[1:] if r['step_m'] is not None]
        S['vio_step_p99_m'] = pct(steps, 99)
        S['vio_max_step_m'] = max(steps) if steps else None
        big = sorted(((r['step_m'], r['t']) for r in odom if (r['step_m'] or 0) > 0.05), reverse=True)
        S['vio_jumps_5cm'] = len(big)
        S['vio_jumps_10cm'] = sum(1 for s, _ in big if s > 0.10)
        P('  rate %s Hz, gaps >100 ms: %d, max gap %s ms' % (fmt(S['vio_hz'], 1), S['vio_gaps_100ms'],
                                                          fmt(S['vio_max_gap_ms'], 0)))
        P('  per-message step p99 %s m, max %s m; steps >5 cm: %d, >10 cm: %d' % (
            fmt(S['vio_step_p99_m'], 3), fmt(S['vio_max_step_m'], 3), S['vio_jumps_5cm'], S['vio_jumps_10cm']))
        by_phase = defaultdict(int)
        for s, t in big:
            by_phase[phase_at(ticks, t)] += 1
        for s, t in big[:8]:
            P('    jump %.3f m at t=%.1f s during %s' % (s, t, phase_at(ticks, t)))
        if by_phase:
            P('  jumps by phase: ' + ', '.join('%s=%d' % kv for kv in sorted(by_phase.items())))
        S['jumps_by_phase'] = dict(by_phase)
        if S['vio_jumps_10cm']:
            flags.append('%d VIO jumps > 10 cm (phases %s). Sim lost position the same way during turns.' % (
                S['vio_jumps_10cm'], dict(by_phase)))
        if S['vio_hz'] and S['vio_hz'] < 20:
            flags.append('VIO rate %.1f Hz (bench was ~30 Hz): Jetson load or camera drops.' % S['vio_hz'])
        if S['vio_max_gap_ms'] > 300:
            flags.append('VIO gap %.0f ms: PX4 EKF2 will be dead-reckoning on IMU.' % S['vio_max_gap_ms'])
    else:
        P('  no odometry recorded')
        flags.append('No VIO odometry reached flight_test: check /visual_slam/tracking/odometry and QoS.')

    # 3. tracking
    P('\n## 3. Tracking (setpoint vs VIO), route only')
    S['tracking'] = {}
    groups = defaultdict(list)
    zerr = []
    for r in route_rows:
        if r['track_err'] is not None:
            groups[r['phase']].append(r['track_err'])
        if r['sp_z'] is not None and r['vio_z'] is not None:
            zerr.append(abs(r['sp_z'] - r['vio_z']))
    for ph, vals in sorted(groups.items()):
        S['tracking'][ph] = {'p50': pct(vals, 50), 'p95': pct(vals, 95), 'max': max(vals)}
        P('  %-7s xy err p50 %s  p95 %s  max %s m  (n=%d)' % (ph, fmt(pct(vals, 50)), fmt(pct(vals, 95)),
                                                               fmt(max(vals)), len(vals)))
    S['alt_err_p95'] = pct(zerr, 95)
    P('  altitude |sp - vio| p95 %s m' % fmt(S['alt_err_p95']))
    fly = groups.get('fly', [])
    if fly and pct(fly, 95) > 0.35:
        flags.append('Fly-phase tracking p95 %.2f m: setpoint lead 0.3 m + lag; try --speed 0.15 or check PX4 '
                     'position gains (MPC_XY_P).' % pct(fly, 95))
    if S['alt_err_p95'] and S['alt_err_p95'] > 0.2:
        flags.append('Altitude error p95 %.2f m: VIO z vs EKF2 height source (EKF2_HGT_REF) disagree?'
                     % S['alt_err_p95'])

    # 4. turns
    P('\n## 4. Turns (setpoint xy fixed: any VIO motion here is drift or a real slide)')
    segs, cur = [], None
    for r in route_rows:
        if r['phase'] in ('turn', 'settle'):
            if cur is None or cur['step'] != r['step']:
                cur = {'step': r['step'], 'rows': []}
                segs.append(cur)
            cur['rows'].append(r)
        else:
            cur = None
    S['turns'] = []
    for sg in segs:
        rows = [r for r in sg['rows'] if r['vio_x'] is not None]
        if len(rows) < 2:
            continue
        a, b = rows[0], rows[-1]
        # distance from the (fixed) setpoint: growth beyond where the turn started = drift/slide
        dist = [math.hypot(r['vio_x'] - r['sp_x'], r['vio_y'] - r['sp_y']) for r in rows]
        disp = max(0.0, max(dist) - dist[0])
        lag = [abs(wrap(r['sp_yaw'] - r['vio_yaw'])) for r in rows if r['sp_yaw'] is not None
               and r['vio_yaw'] is not None]
        final = abs(math.degrees(wrap(b['sp_yaw'] - b['vio_yaw']))) if b['sp_yaw'] is not None else None
        pxd = None
        if a['px4_x'] is not None and b['px4_x'] is not None:
            pxd = math.hypot(b['px4_x'] - a['px4_x'], b['px4_y'] - a['px4_y'])
        d = {'step': sg['step'], 'vio_shift_m': disp, 'px4_shift_m': pxd,
             'yaw_lag_max_deg': math.degrees(max(lag)) if lag else None, 'final_yaw_err_deg': final,
             'duration_s': b['t'] - a['t']}
        S['turns'].append(d)
        P('  step %s: %.1f s, VIO drifted %.2f m from the fixed setpoint, PX4 moved %s m, yaw lag max %s deg, final err %s deg' % (
            sg['step'], d['duration_s'], disp, fmt(pxd), fmt(d['yaw_lag_max_deg'], 0), fmt(final, 1)))
        if disp > 0.10:
            flags.append('Turn at step %s: VIO drifted %.2f m away from a fixed setpoint (sim saw 0.2-0.65 m). '
                         'Compare with the video: real slide or estimate shift?' % (sg['step'], disp))
    if not segs:
        P('  no turns in this profile')

    # 5. VIO vs PX4
    P('\n## 5. VIO odom vs PX4 EKF2 local position (both in ENU)')
    pairs = [r for r in ticks if r['vio_x'] is not None and r['px4_x'] is not None]
    if pairs:
        ref = next((r for r in pairs if r['mphase'] == 'route'), pairs[0])
        ox, oy = ref['vio_x'] - ref['px4_x'], ref['vio_y'] - ref['px4_y']
        oz = ref['vio_z'] - ref['px4_z']
        dyaw = [wrap(r['vio_yaw'] - r['px4_yaw']) for r in pairs if r['px4_yaw'] is not None]
        drift = [math.hypot((r['vio_x'] - r['px4_x']) - ox, (r['vio_y'] - r['px4_y']) - oy) for r in pairs
                 if r['mphase'] == 'route']
        S['vio_px4_offset_xy'] = [ox, oy]
        S['vio_px4_offset_z'] = oz
        S['vio_px4_drift_p95'] = pct(drift, 95)
        S['vio_px4_drift_max'] = max(drift) if drift else None
        S['vio_px4_yaw_diff_deg_p50'] = math.degrees(pct(dyaw, 50)) if dyaw else None
        P('  offset at release (vio - px4): x %.2f y %.2f z %.2f m; yaw diff p50 %s deg' % (
            ox, oy, oz, fmt(S['vio_px4_yaw_diff_deg_p50'], 1)))
        P('  change of that offset during the route: p95 %s, max %s m' % (
            fmt(S['vio_px4_drift_p95']), fmt(S['vio_px4_drift_max'])))
        if math.hypot(ox, oy) > 0.3:
            flags.append('VIO and EKF2 origins differ by %.2f m. flight_interface sends odom-frame setpoints '
                         'as NED without anchoring (anchor_setpoints=false), so PX4 would fly to the wrong '
                         'place. Check the takeoff/hover xy in the video.' % math.hypot(ox, oy))
        if S['vio_px4_yaw_diff_deg_p50'] is not None and abs(S['vio_px4_yaw_diff_deg_p50']) > 5:
            flags.append('VIO yaw vs EKF2 heading differ by %.0f deg: legs will fly rotated '
                         '(odom_to_ned_yaw_offset_rad).' % S['vio_px4_yaw_diff_deg_p50'])
        if S['vio_px4_drift_max'] and S['vio_px4_drift_max'] > 0.2:
            flags.append('EKF2 and VIO diverged by %.2f m in flight: EKF2 rejecting vision '
                         '(check estimator_innovations / EKF2_EV_DELAY in the bag).' % S['vio_px4_drift_max'])
    else:
        P('  no PX4 local position recorded (topic name / QoS?)')
        flags.append('No PX4 vehicle_local_position reached flight_test.')

    # landing offset
    land_rows = [r for r in ticks if r['vio_x'] is not None and r['mphase'] in ('landing', 'done')]
    if route_rows and land_rows and route_rows[0]['vio_x'] is not None:
        a, b = route_rows[0], land_rows[-1]
        S['landing_offset_vio_m'] = math.hypot(b['vio_x'] - a['vio_x'], b['vio_y'] - a['vio_y'])
        P('\n  landing vs route start (VIO): %.2f m  <- compare with a tape measure!' % S['landing_offset_vio_m'])

    P('\n## 6. FLAGS')
    for f in flags or ['none: measure the real landing offset and compare with the VIO number above']:
        P('  * ' + f)
    S['flags'] = flags
    with open(os.path.join(run, 'summary.json'), 'w') as fh:
        json.dump(S, fh, indent=1)
    P('\nwrote %s' % os.path.join(run, 'summary.json'))
    return '\n'.join(out), S


def main(argv):
    if len(argv) < 2 or argv[1] in ('-h', '--help'):
        print(__doc__)
        return 2
    if argv[1] == '--latest':
        root = argv[2] if len(argv) > 2 else DEFAULT_ROOT
        runs = [d for d in glob.glob(os.path.join(root, '*')) if os.path.isfile(os.path.join(d, 'ticks.csv'))]
        if not runs:
            print('no runs under %s' % root)
            return 1
        run = max(runs, key=os.path.getmtime)
    else:
        run = argv[1]
    text, _ = analyze(run)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
