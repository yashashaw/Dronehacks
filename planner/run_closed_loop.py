"""Closed-loop test: noisy sensing + drifting pose + frontier/Theta* navigation.

Run: python run_closed_loop.py [seeds]   (default 3). The course is only used to simulate the
world; the navigator sees range returns, an estimated pose and a pad cue, like the real drone.
"""
import math
import os
import random
import sys
import time

from course_fixture import build_world, write_png
from navigator import NavParams, Navigator
from sensing import (Estimator, EstimatorParams, LatencyParams, SensorParams, Vehicle,
                     VehicleParams, detect_pad, scan)

HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = os.path.join(HERE, "..", "dronehacks-course-v3.obj")
LAUNCH, PAD = (-2.515, 0.914), (-2.042, -0.792)
WINDOW_CENTRE = (-0.47, -2.27)   # metric only: did the flight cross the window?
PAD_RADIUS = 0.762      # outer ring, 30 in
DT = 1 / 20             # depth/estimate rate; the driver delivers 27-30 Hz images, about 20 Hz under load
TIME_LIMIT = 180.0
TOUCH_PENALTY = 5.0


def _rotor_positions(drone, veh):
    c, s = math.cos(drone.yaw), math.sin(drone.yaw)
    return [(drone.pos[0] + c * fx - s * fy, drone.pos[1] + s * fx + c * fy) for fx, fy in veh.rotors]


def fly(world, seed, nav_params=None, sensor=None, est=None, veh=None, latency=None, altitude=0.76,
        verbose=False):
    rng = random.Random(seed)
    sensor, est, veh = sensor or SensorParams(), est or EstimatorParams(), veh or VehicleParams()
    latency = latency or LatencyParams()
    estimator = Estimator(LAUNCH, est, rng)
    drone = Vehicle(LAUNCH, math.pi / 2, veh, rng, altitude)
    nav = Navigator(0.05, nav_params)  # the map starts small at the launch point and grows
    flat = world.union_grid(altitude - sensor.belly - sensor.margin_v, altitude + sensor.top + sensor.margin_v)
    trail, hits, touches, touching, t, result = [], [], 0, False, 0.0, "timeout"
    inbox, last_idx, cmd = [], -1, ((0.0, 0.0), drone.yaw)
    while t < TIME_LIMIT:
        pos, yaw = tuple(drone.pos), drone.yaw
        cam = (pos[0] + sensor.cam_forward * math.cos(yaw), pos[1] + sensor.cam_forward * math.sin(yaw))
        inbox.append((estimator.estimate(pos, yaw),
                      scan(world, cam, drone.z + rng.gauss(0, 0.02), yaw, sensor, rng),
                      detect_pad(flat, pos, yaw, PAD, sensor, rng)))
        # what the navigator receives is stale by the camera-to-command delay
        lag = rng.uniform(*latency.slow_s) if rng.random() < latency.p_slow else rng.uniform(*latency.fast_s)
        idx = min(len(inbox) - 1, max(last_idx, len(inbox) - 1 - round(lag / DT)))
        if idx > last_idx:
            last_idx = idx
            pose, returns, pad_obs = inbox[idx]
            cmd = nav.step(pose, returns, pad_obs, t)
        (vx, vy), face = cmd
        vt = estimator.to_true_velocity(vx, vy)
        face_true = face - estimator.theta
        before = tuple(drone.pos)
        drone.step(vt, face_true, DT)
        estimator.advance(math.dist(before, drone.pos), DT)
        t += DT
        trail.append(tuple(drone.pos))
        lo, hi = drone.z - veh.below, drone.z + veh.above
        in_contact = False
        for _ in range(3):  # contact response: push out, drop the inward velocity; a hard hit is a crash
            worst = None
            for p in _rotor_positions(drone, veh):
                c = world.clearance_at(p, lo, hi)
                if c < veh.prop_radius and (worst is None or c < worst[0]):
                    worst = (c, p)
            if worst is None:
                break
            in_contact = True
            c, p = worst
            e = 0.04
            gx = world.clearance_at((p[0] + e, p[1]), lo, hi) - world.clearance_at((p[0] - e, p[1]), lo, hi)
            gy = world.clearance_at((p[0], p[1] + e), lo, hi) - world.clearance_at((p[0], p[1] - e), lo, hi)
            norm = math.hypot(gx, gy) or 1.0
            nx, ny = gx / norm, gy / norm
            v_in = -(drone.vel[0] * nx + drone.vel[1] * ny)
            if v_in > veh.crash_speed:
                result = "crash"
                break
            if v_in > 0:
                drone.vel[0] += v_in * nx
                drone.vel[1] += v_in * ny
            push = veh.prop_radius - c + 0.005
            drone.pos[0] += nx * push
            drone.pos[1] += ny * push
        if result == "crash":
            break
        if in_contact and not touching:
            touches += 1
            hits.append((round(drone.pos[0], 2), round(drone.pos[1], 2)))
        touching = in_contact
        if nav.status == "landing":
            inside = max(math.dist(p, PAD) for p in _rotor_positions(drone, veh)) + veh.prop_radius <= PAD_RADIUS
            result = "landed" if inside else "landed_outside_ring"
            break
        if verbose and int(t * 1000) % 1000 < DT * 1000:
            print(f"t={t:5.1f} pos=({drone.pos[0]:.2f},{drone.pos[1]:.2f}) {nav.status}")
    length = sum(math.dist(a, b) for a, b in zip(trail, trail[1:]))
    return dict(seed=seed, result=result, time=t, scored=t + TOUCH_PENALTY * touches, touches=touches,
                hits=hits, via_window=any(math.dist(p, WINDOW_CENTRE) < 0.35 for p in trail), length=length, drift=math.hypot(estimator.tx, estimator.ty),
                yaw_err=math.degrees(estimator.theta), nav=nav, trail=trail, flat=flat)


def render(world, run, filename, scale=4):
    nav, g, flat = run["nav"], run["nav"].grid, run["flat"]
    on_trail = {flat.to_cell(*p) for p in run["trail"]}
    rows = []
    for cy in range(flat.h - 1, -1, -1):
        row = bytearray()
        for cx in range(flat.w):
            x, y = flat.to_world(cx, cy)
            m = g.to_cell(x - LAUNCH[0], y - LAUNCH[1])  # nav grid is launch-centred
            known = g.inside(*m) and g.cells[m[1] * g.w + m[0]]
            truth_wall = flat.cells[cy * flat.w + cx] == 1
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
    write_png(filename, flat.w * scale, flat.h * scale, rows)


if __name__ == "__main__":
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    world = build_world(OBJ)
    for seed in range(seeds):
        t0 = time.time()
        run = fly(world, seed)
        print(f"seed {seed}: {run['result']:8} {run['time']:5.1f}s +{run['touches']} touches "
              f"= {run['scored']:5.1f}s  path {run['length']:4.1f} m  drift {run['drift']:.2f} m "
              f"yaw {run['yaw_err']:+.1f} deg  window={run['via_window']}  (wall {time.time() - t0:.0f}s)", flush=True)
        if run["hits"]:
            print("   touches at", run["hits"])
        render(world, run, os.path.join(HERE, f"closed_loop_{seed}.png"))
