#!/usr/bin/env python3
"""Does the Pixhawk's EKF2 agree with our camera position? (standard library only)

  python compare_ekf.py runs/ekf1          # needs odom.csv, lpos.csv, ekf_flags.csv from walk_logger
  python compare_ekf.py --selftest

Compares the horizontal track of the raw camera odometry (cuVSLAM, ENU-like) with EKF2's
vehicle_local_position (NED) from the same run. The two frames differ in axis order and
heading, so EKF2's track is rotated (and, if that fits better, axis-swapped) onto the camera
track with the best-fit angle before differencing; only the residual matters.

Reports: fraction of time EKF2 says it is fusing vision, xy_valid, residual error between the
two tracks (RMS / max), jumps in EKF2's output, and EKF2 reset counters.
"""
import argparse
import csv
import math
import os
import random


def load(path):
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def resample(rows, key_x, key_y, t_grid):
    out, j = [], 0
    for t in t_grid:
        while j + 1 < len(rows) and rows[j + 1]['recv_s'] <= t:
            j += 1
        out.append((rows[j][key_x], rows[j][key_y]))
    return out


def best_rotation(a, b):
    """Angle that rotates b onto a (2D, least squares) and the residual RMS."""
    sxx = sum(p[0] * q[0] + p[1] * q[1] for p, q in zip(a, b))
    sxy = sum(q[0] * p[1] - q[1] * p[0] for p, q in zip(a, b))
    th = math.atan2(sxy, sxx)
    c, s = math.cos(th), math.sin(th)
    res = [math.hypot(p[0] - (c * q[0] - s * q[1]), p[1] - (s * q[0] + c * q[1])) for p, q in zip(a, b)]
    return th, res


def compare(odom, lpos, flags=None, valid_only=True):
    t_lo = max(odom[0]['recv_s'], lpos[0]['recv_s'])
    t_hi = min(odom[-1]['recv_s'], lpos[-1]['recv_s'])
    grid = [t_lo + 0.1 * i for i in range(int((t_hi - t_lo) / 0.1))]
    if len(grid) < 20:
        return {'error': 'not enough overlapping time between odometry and EKF2 output'}
    cam = resample(odom, 'x', 'y', grid)
    ekf = resample(lpos, 'x', 'y', grid)
    valid = resample(lpos, 'xy_valid', 'xy_valid', grid)
    idx = [i for i, v in enumerate(valid) if v[0] > 0.5] if valid_only else list(range(len(grid)))
    res = {'overlap_s': round(t_hi - t_lo, 1), 'xy_valid_fraction': round(sum(v[0] > 0.5 for v in valid) / len(valid), 3)}
    if len(idx) < 20:
        res['error'] = 'EKF2 xy_valid was false for almost the whole run: nothing to compare'
    else:
        c0, e0 = cam[idx[0]], ekf[idx[0]]
        cam_r = [(cam[i][0] - c0[0], cam[i][1] - c0[1]) for i in idx]
        best = None
        for name, tr in (('as-is', lambda p: p), ('axes swapped', lambda p: (p[1], p[0]))):
            ek = [tr((ekf[i][0] - e0[0], ekf[i][1] - e0[1])) for i in idx]
            th, r = best_rotation(cam_r, ek)
            rms = math.sqrt(sum(x * x for x in r) / len(r))
            if best is None or rms < best[0]:
                best = (rms, max(r), math.degrees(th), name)
        extent = max(math.hypot(*p) for p in cam_r)
        res.update({'compared_samples': len(idx), 'camera_track_extent_m': round(extent, 3),
                    'residual_rms_m': round(best[0], 4), 'residual_max_m': round(best[1], 4),
                    'frame_rotation_deg': round(best[2], 1), 'frame_mapping': best[3]})
    steps = [math.dist((a['x'], a['y']), (b['x'], b['y'])) for a, b in zip(lpos, lpos[1:])]
    res['ekf_max_step_m'] = round(max(steps), 3)
    res['ekf_steps_over_5cm'] = sum(s > 0.05 for s in steps)
    res['xy_reset_counter_changes'] = int(lpos[-1].get('xy_reset_counter', 0) - lpos[0].get('xy_reset_counter', 0))
    if flags:
        for k in ('cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt', 'cs_ev_vel', 'cs_mag', 'cs_inertial_dead_reckoning'):
            if k in flags[0]:
                res['frac_' + k] = round(sum(r[k] > 0.5 for r in flags) / len(flags), 3)
    return res


def selftest():
    rnd = random.Random(3)
    odom, lpos = [], []
    th = math.radians(37)
    for i in range(900):                      # 30 s at 30 Hz: out 2 m and back
        t = i / 30
        f = 2.0 * (0.5 - 0.5 * math.cos(2 * math.pi * t / 30))
        x, y = f * 0.9 + rnd.gauss(0, .003), f * 0.2 + rnd.gauss(0, .003)
        odom.append({'recv_s': t, 'x': x, 'y': y})
        n, e = x * math.cos(th) - y * math.sin(th), x * math.sin(th) + y * math.cos(th)
        lpos.append({'recv_s': t + 0.01, 'x': e + rnd.gauss(0, .005), 'y': n + rnd.gauss(0, .005),   # NED-like: swapped
                     'xy_valid': 1.0, 'xy_reset_counter': 0.0})
    r = compare(odom, lpos)
    assert r['residual_rms_m'] < 0.02 and r['frame_mapping'] == 'axes swapped', r
    lpos2 = [dict(p, xy_valid=0.0) for p in lpos]
    assert 'error' in compare(odom, lpos2), 'invalid EKF2 must not be compared'
    print('selftest ok:', {k: r[k] for k in ('residual_rms_m', 'frame_rotation_deg', 'frame_mapping')})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    odom = load(os.path.join(a.run, 'odom.csv'))
    lpos = load(os.path.join(a.run, 'lpos.csv'))
    fp = os.path.join(a.run, 'ekf_flags.csv')
    flags = load(fp) if os.path.exists(fp) else None
    import json
    print(json.dumps(compare(odom, lpos, flags), indent=2))


if __name__ == '__main__':
    main()
