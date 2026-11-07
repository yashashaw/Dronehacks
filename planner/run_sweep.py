"""Noise sweeps for the closed-loop test. Run: python run_sweep.py [seeds]"""
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

from course_fixture import build_grid
from navigator import NavParams
from run_closed_loop import OBJ, fly
from sensing import EstimatorParams, SensorParams

S, E, N = SensorParams(), EstimatorParams(), NavParams()
GATE_B = (0.0, -0.6, 0.3, 0.4)
CONFIGS = {
    "baseline": {},
    "drift x2": dict(est=replace(E, pos_drift=E.pos_drift * 2, yaw_drift_deg=E.yaw_drift_deg * 2)),
    "drift x4": dict(est=replace(E, pos_drift=E.pos_drift * 4, yaw_drift_deg=E.yaw_drift_deg * 4)),
    "jumps x10": dict(est=replace(E, jump_rate=E.jump_rate * 10)),
    "depth noise x2": dict(sensor=replace(S, disparity_sigma_px=S.disparity_sigma_px * 2)),
    "dropout x3": dict(sensor=replace(S, dropout_base=S.dropout_base * 3, dropout_per_m=S.dropout_per_m * 3)),
    "range 3 m": dict(sensor=replace(S, max_range=3.0)),
    "fov 55 deg": dict(sensor=replace(S, hfov_deg=55.0)),
    "fov 100 deg": dict(sensor=replace(S, hfov_deg=100.0)),
    "speed 0.6": dict(nav_params=replace(N, v_max=0.6)),
    "speed 1.5": dict(nav_params=replace(N, v_max=1.5)),
    "baseline, no safety": dict(nav_params=replace(N, rep_gain=0)),
    "drift x2, no safety": dict(nav_params=replace(N, rep_gain=0), est=replace(E, pos_drift=E.pos_drift * 2, yaw_drift_deg=E.yaw_drift_deg * 2)),
    "drift x4, no safety": dict(nav_params=replace(N, rep_gain=0), est=replace(E, pos_drift=E.pos_drift * 4, yaw_drift_deg=E.yaw_drift_deg * 4)),
    "gate B blocked": dict(blocks=[GATE_B]),
    "gate B blocked, no safety": dict(blocks=[GATE_B], nav_params=replace(N, rep_gain=0)),
    "everything bad": dict(
        est=replace(E, pos_drift=E.pos_drift * 3, yaw_drift_deg=E.yaw_drift_deg * 3, jump_rate=E.jump_rate * 5),
        sensor=replace(S, disparity_sigma_px=S.disparity_sigma_px * 2, dropout_base=S.dropout_base * 2,
                       dropout_per_m=S.dropout_per_m * 2, max_range=4.0)),
}
_worlds = {}


def job(args):
    name, seed = args
    cfg = dict(CONFIGS[name])
    blocks = tuple(cfg.pop("blocks", ()))
    if blocks not in _worlds:
        _worlds[blocks] = build_grid(OBJ, extra_blocks=blocks)
    r = fly(_worlds[blocks], seed, **cfg)
    return name, r["result"], r["time"], r["touches"], r["scored"], r["via_window"]


if __name__ == "__main__":
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        rows = pool.map(job, [(n, s) for n in CONFIGS for s in range(seeds)], chunksize=1)
    print(f"{'config':16} {'landed':>7} {'crash':>6} {'timeout':>8} {'time':>6} {'touches':>8} {'scored':>7} {'worst':>6} {'window':>7}")
    for name in CONFIGS:
        rs = [r for r in rows if r[0] == name]
        ok = [r for r in rs if r[1] == "landed"]
        n = len(rs)
        f = lambda xs: sum(xs) / len(xs) if xs else float("nan")
        print(f"{name:16} {len(ok):>4}/{n:<2} {sum(r[1] == 'crash' for r in rs):>6} "
              f"{sum(r[1] == 'timeout' for r in rs):>8} {f([r[2] for r in ok]):>6.1f} "
              f"{f([r[3] for r in rs]):>8.2f} {f([r[4] for r in ok]):>7.1f} "
              f"{max([r[4] for r in ok], default=float('nan')):>6.1f} {sum(r[5] for r in ok):>4}/{len(ok)}")
