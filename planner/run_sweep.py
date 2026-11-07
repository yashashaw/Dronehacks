"""Sweeps for the closed-loop test. Run with the rover env: python run_sweep.py [seeds]

Each row is flown with the same seeds, so rows differ only in the setting being tested.
"""
import os
import statistics as st
import sys
from dataclasses import replace
from multiprocessing import Pool

from course_fixture import build_world
from navigator import NavParams
from run_closed_loop import OBJ, fly
from sensing import EstimatorParams, LatencyParams, SensorParams

S, E, N = SensorParams(), EstimatorParams(), NavParams()
GATE_B = (0.0, -0.6, 0.3, 0.4)  # blocking it forces the window


def drift(k):
    return replace(E, pos_drift=E.pos_drift * k, yaw_drift_deg=E.yaw_drift_deg * k)


BAD = dict(est=replace(drift(3), jump_rate=E.jump_rate * 5),
           sensor=replace(S, disparity_sigma_px=S.disparity_sigma_px * 2, dropout_base=S.dropout_base * 2,
                          dropout_per_m=S.dropout_per_m * 2, max_range=4.0),
           latency=LatencyParams(fast_s=(0.1, 0.2), slow_s=(0.3, 0.5), p_slow=0.15))
# measured end-to-end delay (image stamp -> position published), drone 7
LAT_DEFAULT = LatencyParams()                                                       # p50 58, p95 281, max 373 ms
LAT_FUSION = LatencyParams(fast_s=(0.045, 0.09), slow_s=(0.11, 0.50), p_slow=0.04)  # p50 61, p95 108, max 502 ms
LAT_MAPPING = LatencyParams(fast_s=(0.12, 0.20), slow_s=(0.30, 0.39), p_slow=0.10)  # p50 158, p95 349, max 388 ms
CONFIGS = {
    "baseline": {},
    "standoff on (0.6 m)": dict(nav_params=replace(N, standoff=0.6)),
    "delay: IMU fusion on (61/108 ms)": dict(latency=LAT_FUSION),
    "delay: mapping on (158/349 ms)": dict(latency=LAT_MAPPING),
    "drift x2 (matches 3% measured)": dict(est=drift(2)),
    "drift x4": dict(est=drift(4)),
    "scale error 0.884 (measured, n=1)": dict(est=replace(E, scale=0.884)),
    "scale 0.884 + drift x2": dict(est=replace(drift(2), scale=0.884)),
    "gate B blocked (window)": dict(blocks=[GATE_B]),
    "altitude 0.55 m": dict(altitude=0.55),
    "altitude 0.95 m": dict(altitude=0.95),
    "min range 0.35 m (extended disparity)": dict(sensor=replace(S, min_range=0.35)),
    "dropout x3": dict(sensor=replace(S, dropout_base=S.dropout_base * 3, dropout_per_m=S.dropout_per_m * 3)),
    "depth noise x2": dict(sensor=replace(S, disparity_sigma_px=S.disparity_sigma_px * 2)),
    "fov 55 deg": dict(sensor=replace(S, hfov_deg=55.0)),
    "speed 0.6": dict(nav_params=replace(N, v_max=0.6)),
    "speed 1.5": dict(nav_params=replace(N, v_max=1.5)),
    "everything bad": dict(BAD),
}
_worlds = {}


def job(args):
    name, seed = args
    cfg = dict(CONFIGS[name])
    blocks = tuple(cfg.pop("blocks", ()))
    if blocks not in _worlds:
        _worlds[blocks] = build_world(OBJ, extra_blocks=blocks)
    r = fly(_worlds[blocks], seed, **cfg)
    return name, r["result"], r["time"], r["touches"], r["scored"], r["via_window"], r["nav"].stuck_events


if __name__ == "__main__":
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        rows = pool.map(job, [(n, s) for n in CONFIGS for s in range(seeds)], chunksize=1)
    print(f"{'config':38} {'landed':>7} {'crash':>5} {'t/o':>4} {'time':>6} {'touch':>6} {'scored':>7} {'window':>7}")
    for name in CONFIGS:
        rs = [r for r in rows if r[0] == name]
        ok = [r for r in rs if r[1] == "landed"]
        med = lambda xs: st.median(xs) if xs else float("nan")
        print(f"{name:38} {len(ok):>4}/{len(rs):<2} {sum(r[1] == 'crash' for r in rs):>5} "
              f"{sum(r[1] == 'timeout' for r in rs):>4} {med([r[2] for r in ok]):>6.1f} "
              f"{med([r[3] for r in rs]):>6.1f} {med([r[4] for r in ok]):>7.1f} {sum(r[5] for r in ok):>4}/{len(ok)}")
    print("(time, touch, scored = medians; scored = time + 5 s per touch, landed flights only)")
