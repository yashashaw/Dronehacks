"""Closed-loop test: noisy sensing + drifting pose + frontier/Theta* navigation.

Run: python run_closed_loop.py [seeds]   (default 3). The course is only used to simulate the
world; the navigator sees range returns, an estimated pose and a pad cue, like the real drone.
"""
import math
import os
import random
import sys
import time

from course_fixture import build_grid, write_png
from navigator import NavParams, Navigator
from sensing import (Estimator, EstimatorParams, SensorParams, Vehicle, VehicleParams,
                     detect_pad, scan)

HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = os.path.join(HERE, "..", "dronehacks-course-v3.obj")
LAUNCH, PAD = (-2.515, 0.914), (-2.042, -0.792)
PAD_RADIUS = 0.762      # outer ring, 30 in
DT = 1 / 15             # sensor rate
TIME_LIMIT = 180.0
TOUCH_PENALTY = 5.0


def fly(world, seed, nav_params=None, sensor=None, est=None, veh=None, verbose=False):
    rng = random.Random(seed)
    sensor, est, veh = sensor or SensorParams(), est or EstimatorParams(), veh or VehicleParams()
    estimator = Estimator(LAUNCH, est, rng)
    drone = Vehicle(LAUNCH, math.pi / 2, veh, rng)
    # 10 m square grid in the launch-centred frame; placement is a config choice
    nav = Navigator(200, 200, 0.05, (-4.0, -5.0), nav_params)
    clr = world.clearance()
    trail, hits, touches, touching, t, result = [], [], 0, False, 0.0, "timeout"
    last_progress, best_dist = 0.0, 1e9
    while t < TIME_LIMIT:
        pos, yaw = tuple(drone.pos), drone.yaw
        returns = scan(world, pos, yaw, sensor, rng)
        pad_obs = detect_pad(world, pos, yaw, PAD, sensor, rng)
        pose = estimator.estimate(pos, yaw)
        (vx, vy), face = nav.step(pose, returns, pad_obs)
        vt = estimator.to_true_velocity(vx, vy)
        face_true = face - estimator.theta
        before = tuple(drone.pos)
        drone.step(vt, face_true, DT)
        estimator.advance(math.dist(before, drone.pos), DT)
        t += DT
        trail.append(tuple(drone.pos))
        cx, cy = world.to_cell(*drone.pos)
        c = clr[cy * world.w + cx] if world.inside(cx, cy) else 0.0
        if c < veh.radius and not touching:
            touches += 1
            hits.append((round(drone.pos[0], 2), round(drone.pos[1], 2)))
        touching = c < veh.radius
        if c < veh.radius * 0.5:
            result = "crash"
            break
        if nav.status == "landing":
            inside = math.dist(drone.pos, PAD) + veh.radius <= PAD_RADIUS
            result = "landed" if inside else "landed_outside_ring"
            break
        d = math.dist(drone.pos, PAD)
        if d < best_dist - 0.3:
            best_dist, last_progress = d, t
        if verbose and int(t * DT * 1000) % 5 == 0:
            print(f"t={t:5.1f} pos=({drone.pos[0]:.2f},{drone.pos[1]:.2f}) {nav.status}")
    length = sum(math.dist(a, b) for a, b in zip(trail, trail[1:]))
    return dict(seed=seed, result=result, time=t, scored=t + TOUCH_PENALTY * touches, touches=touches,
                hits=hits, length=length, drift=math.hypot(estimator.tx, estimator.ty),
                yaw_err=math.degrees(estimator.theta), nav=nav, trail=trail)


def render(world, run, filename, scale=4):
    nav, g = run["nav"], run["nav"].grid
    on_trail = {world.to_cell(*p) for p in run["trail"]}
    rows = []
    for cy in range(world.h - 1, -1, -1):
        row = bytearray()
        for cx in range(world.w):
            x, y = world.to_world(cx, cy)
            m = g.to_cell(x - LAUNCH[0], y - LAUNCH[1])  # nav grid is launch-centred
            known = g.inside(*m) and g.cells[m[1] * g.w + m[0]]
            truth_wall = world.cells[cy * world.w + cx] == 1
            if (cx, cy) in on_trail:
                px = (220, 30, 30)
            elif known == 1:
                px = (0, 0, 0)
            elif truth_wall:
                px = (230, 160, 60)      # real wall the drone never mapped
            elif known == 0:
                px = (255, 255, 255)
            else:
                px = (200, 205, 230)     # unknown
            row += bytes(px) * scale
        rows.extend([bytes([0]) + bytes(row)] * scale)
    write_png(filename, world.w * scale, world.h * scale, rows)


if __name__ == "__main__":
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    world = build_grid(OBJ)
    for seed in range(seeds):
        t0 = time.time()
        run = fly(world, seed)
        print(f"seed {seed}: {run['result']:8} {run['time']:5.1f}s +{run['touches']} touches "
              f"= {run['scored']:5.1f}s  path {run['length']:4.1f} m  drift {run['drift']:.2f} m "
              f"yaw {run['yaw_err']:+.1f} deg  (wall {time.time() - t0:.0f}s)", flush=True)
        if run["hits"]:
            print("   touches at", run["hits"])
        render(world, run, os.path.join(HERE, f"closed_loop_{seed}.png"))
