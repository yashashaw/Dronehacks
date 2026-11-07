import sys, statistics as st
from dataclasses import replace
from multiprocessing import Pool
from course_fixture import build_world
from navigator import NavParams
from run_closed_loop import OBJ, fly
from sensing import EstimatorParams
N, E = NavParams(), EstimatorParams()
def drift(k): return replace(E, pos_drift=E.pos_drift*k, yaw_drift_deg=E.yaw_drift_deg*k)
NAV = {
 "safety off":                  replace(N, rep_gain=0.0, standoff=0.0),
 "safety as built (0.5 m)":     replace(N, standoff=0.0),
 "safety reaches 0.95 m":       replace(N, standoff=0.0, rep_dist=0.95, rep_min=0.55),
 "safety 0.95 m + standoff":    replace(N, standoff=0.6, rep_dist=0.95, rep_min=0.55),
 "safety 0.95, gain 1.2":       replace(N, standoff=0.0, rep_dist=0.95, rep_min=0.55, rep_gain=1.2),
}
COND = {"baseline": {}, "drift x2": dict(est=drift(2)), "drift x4": dict(est=drift(4))}
_w=None
def job(a):
    global _w
    if _w is None: _w=build_world(OBJ)
    nav,cond,seed=a
    r=fly(_w,seed,nav_params=NAV[nav],**COND[cond])
    return nav,cond,r['result'],r['time'],r['touches'],r['scored']
if __name__=="__main__":
    n=int(sys.argv[1]) if len(sys.argv)>1 else 40
    with Pool(15) as p: rows=p.map(job,[(a,c,s) for a in NAV for c in COND for s in range(n)],chunksize=1)
    print(f"{'navigator':28} {'condition':10} {'landed':>7} {'crash':>5} {'t/o':>4} {'med time':>8} {'med touch':>9} {'med scored':>10}")
    for a in NAV:
        for c in COND:
            rs=[r for r in rows if r[0]==a and r[1]==c]; ok=[r for r in rs if r[2]=='landed']
            m=lambda xs: st.median(xs) if xs else float('nan')
            print(f"{a:28} {c:10} {len(ok):>4}/{len(rs)} {sum(r[2]=='crash' for r in rs):>5} {sum(r[2]=='timeout' for r in rs):>4} {m([r[3] for r in ok]):>8.1f} {m([r[4] for r in rs]):>9.1f} {m([r[5] for r in ok]):>10.1f}")
