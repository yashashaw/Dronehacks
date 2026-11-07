"""How heavy is the planner on THIS machine? No ROS, no drone, no flight. Run it in flyx and on the Jetson:

    python3 -m team_planner.budget_probe            (from team_ws/src/team_planner, or with it installed)

Prints versions, import times, the one-time numba compile (cold) and start-up with a cache (warm), then the
per-frame time of the whole planner on a synthetic corridor stream, plus memory. Compare the per-frame
numbers to the budget line it prints. Also run `top` beside it to see what else is using the CPU.
"""
import math
import os
import statistics as st
import sys
import time

t_start = time.perf_counter()
import numpy as np                                    # noqa: E402
t_numpy = time.perf_counter()
try:
    import numba                                      # noqa: E402
    numba_version = numba.__version__
except ImportError:
    numba_version = None
t_numba = time.perf_counter()
import scipy                                          # noqa: E402
t_scipy = time.perf_counter()


def rss_mb():
    try:
        import psutil
        return psutil.Process().memory_info().rss / 1e6
    except ImportError:
        try:
            import resource
            r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            return r / 1024.0 if sys.platform != "darwin" else r / 1e6
        except ImportError:
            return float("nan")


def corridor_depth(w=320, h=200, hfov=78.4, cam_h=0.76, half_width=0.5, length=12.0):
    fx = (w / 2) / math.tan(math.radians(hfov) / 2)
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    dx, dy = (u - w / 2) / fx, (v - h / 2) / fx
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.full(dx.shape, length)
        t = np.minimum(t, np.where(dy > 0, cam_h / dy, np.inf))
        t = np.minimum(t, np.where(dy < 0, (2.5 - cam_h) / -dy, np.inf))
        t = np.minimum(t, np.where(dx > 0, half_width / dx, np.inf))
        t = np.minimum(t, np.where(dx < 0, half_width / -dx, np.inf))
    return np.clip(t * 1000, 0, 65535).astype(np.uint16)


def main():
    print(f"python {sys.version.split()[0]}  numpy {np.__version__}  scipy {scipy.__version__}  "
          f"numba {numba_version}  cores {os.cpu_count()}")
    print(f"NUMBA_CACHE_DIR = {os.environ.get('NUMBA_CACHE_DIR', '(default: next to the source)')}")
    print(f"import numpy {t_numpy - t_start:.2f} s | numba {t_numba - t_numpy:.2f} s | scipy {t_scipy - t_numba:.2f} s")

    t0 = time.perf_counter()
    from team_planner.planner_logic import PlannerConfig, PlannerLogic, warm_up
    t1 = time.perf_counter()
    compiled = warm_up("astar")
    t2 = time.perf_counter()
    print(f"import planner {t1 - t0:.2f} s | warm-up (compile or cache load) {t2 - t1:.2f} s | compiled search: {compiled}")
    print("  (run it twice: the first run compiles, the second should load the cache and be much faster)")

    logic = PlannerLogic(PlannerConfig(depth_decimation=1))
    logic.on_state({"armed": True, "offboard_active": True, "odom_enu": [0, 0, 0], "local_enu": [0, 0, 0]}, 0.0)
    depth = corridor_depth()
    times, x = [], 0.0
    for i in range(300):
        t = i / 28.0
        x += 0.02                                     # creep forward so the map keeps growing
        logic.on_odometry(t, x, 0.0, 0.0, 0.0)
        c0 = time.perf_counter()
        logic.on_depth(t, t, depth)
        times.append(1000 * (time.perf_counter() - c0))
    steady = times[30:]
    print(f"planner per depth frame over 270 frames: mean {st.mean(steady):.1f} ms | median {st.median(steady):.1f} ms | "
          f"p95 {sorted(steady)[int(0.95 * len(steady))]:.1f} ms | worst {max(steady):.1f} ms")
    print(f"slowest of the first 30 frames (compile or warm-up stalls show here): {max(times[:30]):.0f} ms")
    print(f"memory (RSS): {rss_mb():.0f} MB")
    print("budget: depth runs ~28 Hz and the node uses every 2nd frame, so about 70 ms is available per used frame,"
          " and the planner should stay well under half of that while cuVSLAM runs.")


if __name__ == "__main__":
    main()
