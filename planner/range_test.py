"""How much does the usable depth range matter? Run with the rover env: python range_test.py [seeds]"""
import os, statistics as st, sys
from dataclasses import replace
from multiprocessing import Pool
from course_fixture import build_world
from navigator import NavParams
from run_closed_loop import OBJ, fly
from sensing import SensorParams
S, N = SensorParams(), NavParams()
RANGES = (1.0, 1.5, 2.0, 3.0, 5.0)
PLANNERS = {"Theta*": "theta", "A*": "astar"}
_w = None
def job(a):
    global _w
    if _w is None: _w = build_world(OBJ)
    pl, rng_m, seed = a
    r = fly(_w, seed, nav_params=replace(N, planner=PLANNERS[pl]), sensor=replace(S, max_range=rng_m))
    return pl, rng_m, r["result"], r["time"], r["touches"], r["scored"]
if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    with Pool(15) as p: rows = p.map(job, [(pl, m, s) for pl in PLANNERS for m in RANGES for s in range(n)], chunksize=1)
    print(f"{'planner':8} {'max range':>9} {'landed':>8} {'crash':>5} {'t/o':>4} {'time':>6} {'touch':>6} {'scored':>7}")
    for pl in PLANNERS:
        for m in RANGES:
            rs = [r for r in rows if r[0] == pl and r[1] == m]; ok = [r for r in rs if r[2] == "landed"]
            med = lambda x: st.median(x) if x else float("nan")
            print(f"{pl:8} {m:>7.1f} m {len(ok):>4}/{len(rs)} {sum(r[2]=='crash' for r in rs):>5} {sum(r[2]=='timeout' for r in rs):>4} {med([r[3] for r in ok]):>6.1f} {med([r[4] for r in rs]):>6.1f} {med([r[5] for r in ok]):>7.1f}")
