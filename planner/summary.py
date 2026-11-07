"""Compact batch summary: python summary.py [seeds]  (run with the rover env)."""
import sys, time, statistics as st
from multiprocessing import Pool
from course_fixture import build_world
from run_closed_loop import OBJ, fly
_w = None
def job(seed):
    global _w
    if _w is None: _w = build_world(OBJ)
    r = fly(_w, seed)
    return seed, r['result'], r['time'], r['touches'], r['nav'].stuck_events, r['via_window']
if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    t = time.time()
    with Pool(15) as p: rows = p.map(job, range(n), chunksize=1)
    ok = [r for r in rows if r[1] == 'landed']
    print(f"landed {len(ok)}/{n}  crash {sum(r[1]=='crash' for r in rows)}  timeout {sum(r[1]=='timeout' for r in rows)}  "
          f"median time {st.median([r[2] for r in ok]) if ok else float('nan'):.1f}  "
          f"median touches {st.median([r[3] for r in rows]):.1f}  mean stuck events {sum(r[4] for r in rows)/n:.1f}  wall {time.time()-t:.0f}s")
    print("per seed:", [(r[0], r[1][:2], round(r[2]), r[3]) for r in rows])
