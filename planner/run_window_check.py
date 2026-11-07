"""Does the safety layer still let the drone through the window? Gate B is blocked in the
simulated world so the only way to the pad is the window. Run: python run_window_check.py [seeds]"""
import sys
from dataclasses import replace
from multiprocessing import Pool

from course_fixture import build_grid
from navigator import NavParams
from run_closed_loop import OBJ, fly

GATE_B = (0.0, -0.6, 0.3, 0.4)


def job(args):
    gain, seed = args
    world = build_grid(OBJ, extra_blocks=[GATE_B])
    r = fly(world, seed, nav_params=replace(NavParams(), rep_gain=gain))
    return gain, seed, r["result"], r["time"], r["touches"], r["via_window"]


if __name__ == "__main__":
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    with Pool(12) as pool:
        rows = pool.map(job, [(g, s) for g in (0.0, 0.8) for s in range(seeds)])
    for gain in (0.0, 0.8):
        rs = [r for r in rows if r[0] == gain]
        print(f"safety layer {'off' if gain == 0 else 'on '}: "
              f"landed {sum(r[2] == 'landed' for r in rs)}/{len(rs)}, through window {sum(r[5] for r in rs)}/{len(rs)}, "
              f"mean time {sum(r[3] for r in rs) / len(rs):.1f}s, mean touches {sum(r[4] for r in rs) / len(rs):.2f}")
        for r in rs:
            print("   ", r[1:])
