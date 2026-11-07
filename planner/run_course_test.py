"""Plan launch -> pad on the course fixture. Run: python run_course_test.py"""
import math
import os
import time

from course_fixture import build_grid, render_png
from theta_star import theta_star

HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = os.path.join(HERE, "..", "dronehacks-course-v3.obj")
START, GOAL = (-2.515, 0.914), (-2.042, -0.792)  # launch mark, pad centre
RADIUS = 0.25  # 13 in drone: half-diagonal 0.23 m + buffer
# Window opening in the tilted chicane wall (centre about (-0.47, -2.27)). Blocking it forces Gate B.
WINDOW = (-0.85, -2.80, -0.05, -1.75)


def run(label, grid):
    t = time.time()
    result = theta_star(grid, START, GOAL, RADIUS)
    dt = time.time() - t
    if result is None:
        print(f"{label}: NO PATH ({dt:.1f}s)")
        return None
    path, cost = result
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    clr = grid.clearance()
    worst = min(
        clr[grid.to_cell(*p)[1] * grid.w + grid.to_cell(*p)[0]]
        for a, b in zip(path, path[1:])
        for p in [(a[0] + (b[0] - a[0]) * k / 50, a[1] + (b[1] - a[1]) * k / 50) for k in range(51)]
    )
    print(f"{label}: {len(path)} waypoints, {length:.1f} m, min clearance {worst:.2f} m, {dt:.1f}s")
    assert worst >= RADIUS - 1e-6
    render_png(grid, path, os.path.join(HERE, label + ".png"), RADIUS)
    return path


if __name__ == "__main__":
    default = run("default", build_grid(OBJ))
    no_window = run("window_blocked", build_grid(OBJ, extra_blocks=[WINDOW]))
    assert default and no_window
