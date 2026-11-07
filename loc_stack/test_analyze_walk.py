#!/usr/bin/env python3
"""Self-test for analyze_walk: a synthetic hand-carry around a known rectangle.

Run:  python test_analyze_walk.py     (no ROS, no drone, no third-party packages)
"""
import csv
import math
import os
import random
import tempfile

import analyze_walk as aw

L, W = 0.60, 0.40        # true rectangle, metres
SCALE = 1.03             # VIO over-reads by 3 %
HEADING = math.radians(25)   # odom frame is rotated by an arbitrary heading
HZ, DWELL_S, MOVE_S = 30, 3.0, 3.0


def synth(path, seed=1, noise=0.002):
    rnd = random.Random(seed)
    corners = [(0, 0), (L, 0), (L, W), (0, W), (0, 0)]   # corner 1 again at the end
    rows, t = [], 0.0

    def emit(x, y, z=1.0):
        nonlocal t
        c, s = math.cos(HEADING), math.sin(HEADING)
        X, Y = SCALE * (c * x - s * y), SCALE * (s * x + c * y)
        rows.append([t, 1000 + t, X + rnd.gauss(0, noise), Y + rnd.gauss(0, noise),
                     z + rnd.gauss(0, noise)])
        t += 1.0 / HZ

    for k, (cx, cy) in enumerate(corners):
        for _ in range(int(DWELL_S * HZ)):
            emit(cx, cy)
        if k + 1 < len(corners):
            nx, ny = corners[k + 1]
            n = int(MOVE_S * HZ)
            for i in range(1, n + 1):
                f = 0.5 - 0.5 * math.cos(math.pi * i / n)  # smooth start/stop
                emit(cx + (nx - cx) * f, cy + (ny - cy) * f)
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, 'odom.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['recv_s', 'stamp_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz'])
        for r in rows:
            w.writerow(r + [0, 0, 0, 1, 0, 0, 0])
    with open(os.path.join(path, 'status.csv'), 'w', newline='') as f:
        f.write('recv_s,vo_state\n0.0,1\n')


def test_object_mapping():
    with tempfile.TemporaryDirectory() as d:
        synth(d)
        res = aw.analyze(d, 0.04, 1.5, (L, W), 0)
    assert len(res['dwells']) == 5, res['dwells']   # 4 corners + return to corner 1
    o = res['object']
    for e in o['edge_error_pct']:
        assert 2.0 < e < 4.0, o                       # recovers the injected +3 %
    for a in o['corner_angles_deg']:
        assert abs(a - 90) < 3, o                     # rotation of the odom frame is irrelevant
    assert abs(o['scale_factor_measured_over_true'] - SCALE) < 0.015, o
    assert o['return_to_corner1_error_m'] < 0.01, o
    assert res['tracking_lost_events'] == 0
    print('object mapping ok: edges', o['edges_m'], 'scale', o['scale_factor_measured_over_true'])


def test_swapped_edge_order():
    """Starting on the other side of the rectangle (W first) must still match."""
    with tempfile.TemporaryDirectory() as d:
        synth(d)
        res = aw.analyze(d, 0.04, 1.5, (W, L), 0)     # user typed the dimensions the other way
    assert all(abs(e) < 4.0 for e in res['object']['edge_error_pct']), res['object']
    print('swapped order ok')


def test_stationary_has_one_dwell():
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(d, exist_ok=True)
        rnd = random.Random(2)
        with open(os.path.join(d, 'odom.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['recv_s', 'stamp_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz'])
            for i in range(30 * 20):
                w.writerow([i / 30, 1000 + i / 30, rnd.gauss(0, .001), rnd.gauss(0, .001),
                            rnd.gauss(0, .001), 0, 0, 0, 1, 0, 0, 0])
        res = aw.analyze(d, 0.04, 1.5, None, 0)
    assert len(res['dwells']) == 1 and res['path_length_m'] < 0.5, res
    print('stationary ok: path', res['path_length_m'], 'm')


if __name__ == '__main__':
    test_object_mapping()
    test_swapped_edge_order()
    test_stationary_has_one_dwell()
    print('ALL PASS')
