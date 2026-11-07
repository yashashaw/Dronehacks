"""Benchmark of route finders and a no-map reactive baseline. Run with the rover env:

    python run_benchmark.py [seeds_main] [seeds_per_layout]

Part A: this course, normal noise and 2x / 4x position drift.
Part B: eight changed layouts (walls shifted or rotated, gaps narrowed, extra obstacles).
Every contender gets the same seeds, noise, drone model, mapper, goal choice, follower and stuck
recovery; only the route finder (or, for the reactive baseline, the whole strategy) changes.
"""
import math
import os
import pickle
import statistics as st
import sys
import time
from dataclasses import replace
from multiprocessing import Pool

from course_fixture import build_world, make_layouts
from navigator import NavParams
from run_closed_loop import OBJ, fly
from sensing import EstimatorParams, SensorParams

N, E = NavParams(), EstimatorParams()
CONTENDERS = {
    "Theta* (ours)": dict(nav_params=replace(N, planner="theta")),
    "Theta* corner-safe": dict(nav_params=replace(N, planner="theta_safe")),
    "A* (no smoothing)": dict(nav_params=replace(N, planner="astar")),
    "A* + shortcut": dict(nav_params=replace(N, planner="astar_smooth")),
    "RRT + shortcut": dict(nav_params=replace(N, planner="rrt_smooth")),
    "Reactive, no map": dict(strategy="reactive"),
}
CONDITIONS = {
    "normal noise": {},
    "drift x2": dict(est=replace(E, pos_drift=E.pos_drift * 2, yaw_drift_deg=E.yaw_drift_deg * 2)),
    "drift x4": dict(est=replace(E, pos_drift=E.pos_drift * 4, yaw_drift_deg=E.yaw_drift_deg * 4)),
    "depth range 3 m": dict(sensor=replace(SensorParams(), max_range=3.0)),
}
LAYOUTS = []   # (name, perturb, blocks); workers get the spec with each job (Windows starts them fresh)
_worlds = {}


def _world(key, spec):
    if key not in _worlds:
        _worlds[key] = build_world(OBJ) if spec is None else build_world(OBJ, extra_blocks=spec[1], perturb=spec[0])
    return _worlds[key]


def job(a):
    part, contender, key, cond, seed, spec = a
    cfg = dict(CONTENDERS[contender])
    cfg.update(CONDITIONS[cond])
    r = fly(_world(key, spec), seed, **cfg)
    return dict(part=part, contender=contender, key=key, cond=cond, seed=seed, result=r["result"],
                time=r["time"], touches=r["touches"], scored=r["scored"], length=r["length"],
                plan_calls=r["plan_calls"], plan_time=r["plan_time"])


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    m = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - m) / d, (c + m) / d


def summarise(rows):
    ok = [r for r in rows if r["result"] == "landed"]
    lo, hi = wilson(len(ok), len(rows))
    med = lambda xs: st.median(xs) if xs else float("nan")
    calls = sum(r["plan_calls"] for r in rows)
    ms = 1000 * sum(r["plan_time"] for r in rows) / calls if calls else float("nan")
    return (f"{len(ok):>3}/{len(rows):<3} ({100*len(ok)/len(rows):3.0f}%, {100*lo:2.0f}-{100*hi:3.0f}%) "
            f"crash {sum(r['result']=='crash' for r in rows):>2} t/o {sum(r['result']=='timeout' for r in rows):>2} "
            f"time {med([r['time'] for r in ok]):5.1f} touch {med([r['touches'] for r in rows]):4.1f} "
            f"scored {med([r['scored'] for r in ok]):6.1f} path {med([r['length'] for r in ok]):4.1f}m "
            f"plan {ms:5.2f} ms/call")


if __name__ == "__main__":
    n_main = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    n_lay = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    t0 = time.time()
    LAYOUTS.extend(make_layouts(OBJ))
    jobs = [("A", c, "base", cond, s, None) for c in CONTENDERS for cond in CONDITIONS for s in range(n_main)]
    jobs += [("B", c, i, "normal noise", s, (LAYOUTS[i][1], LAYOUTS[i][2]))
             for c in CONTENDERS for i in range(len(LAYOUTS)) for s in range(n_lay)]
    rows = []
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        for k, r in enumerate(pool.imap_unordered(job, jobs, chunksize=1), 1):
            rows.append(r)
            if k % 100 == 0:
                print(f"  {k}/{len(jobs)} flights done, {time.time()-t0:.0f} s", flush=True)
                pickle.dump(rows, open("benchmark_partial.pkl", "wb"))
    out = []
    out.append(f"BENCHMARK  main course: {n_main} flights per cell; changed layouts: {n_lay} per layout x {len(LAYOUTS)} layouts")
    out.append("success = landed in the pad ring; interval is a 95% Wilson interval; time/touch/scored/path are medians")
    out.append("scored = flight time + 5 s per touch (landed flights); plan = mean route-finder time on this laptop (Jetson is ~4x slower)\n")
    for cond in CONDITIONS:
        out.append(f"== Part A, this course, {cond}")
        for c in CONTENDERS:
            out.append(f"  {c:20} " + summarise([r for r in rows if r["part"] == "A" and r["contender"] == c and r["cond"] == cond]))
        out.append("")
    out.append("== Part B, changed layouts, all layouts pooled")
    for c in CONTENDERS:
        out.append(f"  {c:20} " + summarise([r for r in rows if r["part"] == "B" and r["contender"] == c]))
    out.append("\n== Part B, landed per layout (of %d flights)" % n_lay)
    out.append(f"  {'layout':44}" + "".join(f"{c[:14]:>16}" for c in CONTENDERS))
    for i, (name, perturb, blocks) in enumerate(LAYOUTS):
        cells = []
        for c in CONTENDERS:
            rs = [r for r in rows if r["part"] == "B" and r["contender"] == c and r["key"] == i]
            cells.append(f"{sum(r['result']=='landed' for r in rs):>16}")
        out.append(f"  {name:44}" + "".join(cells))
    out.append(f"\nwall time {time.time()-t0:.0f} s")
    text = "\n".join(out)
    print(text)
    open("benchmark_results.txt", "w", encoding="utf-8").write(text + "\n")
