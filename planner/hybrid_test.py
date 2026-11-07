"""General planner vs the hybrid (hardcoded goals, live map and route finding). Run with the rover env."""
import os, statistics as st, sys
from dataclasses import replace
from multiprocessing import Pool
from course_fixture import build_world, plan_course_route
from navigator import NavParams
from run_closed_loop import OBJ, fly
from sensing import EstimatorParams
E_, N_ = EstimatorParams(), NavParams()
def drift(k): return replace(E_, pos_drift=E_.pos_drift * k, yaw_drift_deg=E_.yaw_drift_deg * k)
CONDS = {"normal noise": {}, "drift x2": dict(est=drift(2)), "drift x4": dict(est=drift(4))}
MODES = ("general", "hybrid")
PLANNERS = ("astar",)
_w = None
def job(a):
    global _w
    mode, planner, cond, seed, route = a
    if _w is None: _w = build_world(OBJ)
    kw = dict(CONDS[cond]); kw["nav_params"] = replace(N_, planner=planner)
    r = fly(_w, seed, route=(route if mode == "hybrid" else None), **kw)
    return mode, planner, cond, r["result"], r["time"], r["touches"], r["scored"]
if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    launch, pad, wps = plan_course_route(OBJ)
    route = (wps, pad)
    with Pool(15) as p:
        rows = p.map(job, [(m, pl, c, s, route) for m in MODES for pl in PLANNERS for c in CONDS for s in range(n)], chunksize=1)
    print(f"{'mode':8} {'planner':7} {'condition':13} {'landed':>7} {'crash':>5} {'t/o':>4} {'time':>6} {'touch':>6} {'scored':>7}")
    for m in MODES:
        for pl in PLANNERS:
            for c in CONDS:
                rs = [r for r in rows if r[:3] == (m, pl, c)]; ok = [r for r in rs if r[3] == "landed"]
                md = lambda x: st.median(x) if x else float("nan")
                print(f"{m:8} {pl:7} {c:13} {len(ok):>4}/{len(rs)} {sum(r[3]=='crash' for r in rs):>5} {sum(r[3]=='timeout' for r in rs):>4} {md([r[4] for r in ok]):>6.1f} {md([r[5] for r in rs]):>6.1f} {md([r[6] for r in ok]):>7.1f}")
