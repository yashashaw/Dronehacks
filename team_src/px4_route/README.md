# px4_route: fly a waypoint route on the stack that worked

The **px4_square stack that flew cleanly (PX4 log 26)**, generalised from a fixed square to a
route file:

- **Positions:** every position is PX4's own (`/fmu/out/vehicle_local_position`). VIO reaches PX4 only inside EKF2. Height is PX4's (rangefinder, baro backup), never VIO's.
- **flight_interface:** the ONE `flight_interface`, set exactly like px4_square: `anchor_setpoints: false`, `odometry_topic` = our relay of PX4's position.
- **Takeover:** only on the pilot's **arm on Altitude → flip to Position**, with PX4's position fresh and valid and its height ~0. Nothing here arms or publishes to `/fmu/in`.

What's new:
- **Route from a YAML file**, in the *start frame*: metres from the takeoff point, `x` = forward (nose at takeoff), `y` = left, `z` = up.
- **A "carrot" setpoint** slides along the path at the planned speed and slows for sharp turns and the end. It waits whenever the drone falls behind, so the speed is set by the route, not by how hard PX4 chases a far-away point.
- **Aborts land straight down** (a down-twist, sent until PX4 disarms, which keeps working without a position):
  - more than `max_cross_track` off the path for 0.5 s;
  - higher than `max_height` + 0.3 m;
  - PX4 position stale or invalid;
  - `max_time_s` (170 s) in the air.
- **Height reference only once PX4 has settled:** flight_interface takes its 0.3 m floor and its ceiling from the first PX4 position we pass it. Right after boot, PX4's height reads ~0.5 m off for a while (drone 18, 2026-11-08: reference captured at 0.53 m, so floor 0.83 m and ceiling 2.53 m). So px4_route passes nothing until PX4 reads within 0.25 m of the floor AND has been steady (under 3 cm change) for 2 s, and offers no takeover before then. A steady +0.13 m on the floor is normal on drone 18. Watch for `waiting for PX4 height to be steady near 0`, then `relaying PX4 position`. **Arm only after the `relaying` line.**
- **Logs survive reboots:** every flight writes `~/team_ws/runs/route-<time>.csv` (10 Hz: carrot, PX4 position, cross-track, rangefinder) and `route-<time>.log` (events).

## Files
| | |
|---|---|
| `px4_route/route.py` | route format, validation, frames, path, carrot (no ROS; unit-tested) |
| `px4_route/route_mission.py` | the ROS node |
| `launch/px4_route.launch.py` | flight_interface (px4_square settings) + the node |
| `routes/test_line.yaml` | first flight: up 0.8 m, 1 m forward and back, land (0.2 m/s) |
| `routes/test_box.yaml` | 1 m box, four 90° corners, land (0.3 m/s) |
| `routes/course_walk_DRAFT.yaml/.png` | **draft** course route from the 21:55 course walk (see below) |
| `tools/route_from_walk.py` | laptop: walk recording → route file + plot |
| `test/test_route.py` | `python -m pytest team_src/px4_route/test` (21 tests, includes a crude flight sim) |

## Route file
```yaml
name: test-box
takeoff_height: 0.8      # climb straight up to this first
speed: 0.3               # m/s along the path
corner_speed: 0.2        # through turns sharper than corner_angle_deg (30), and near the end
corner_radius: 0.5       # how far before/after a sharp turn to be slow
climb_speed: 0.3         # max vertical m/s
max_height: 1.2          # refused if any waypoint is higher; lands if the drone gets 0.3 m above it
max_cross_track: 0.4     # lands if this far off the path for 0.5 s
yaw: fixed               # fixed = keep the takeoff heading; path = face the direction of travel (30 deg/s)
end: land                # land | hold
max_time_s: 170
waypoints:
  - [1.0, 0.0, 0.8]          # [x forward, y left, z up]
  - [1.0, 1.0, 0.8, 0.2]     # optional 4th value: speed on the leg TO this waypoint
```
Typos are refused (unknown keys, speeds above 2 m/s, heights above 2.6 m or below 0.3 m).
A refused route means **nothing flies**: the node never offers a setpoint, so there is no takeover.

## The course route from the walk
`tools/route_from_walk.py` uses the walk's **VIO path**. VIO tracked cleanly on the 21:55 walk: about 0.3 m from the start after a 45 m lap. Heights come from the rangefinder or a fixed value, **never VIO's height**, which read 0.3–0.5 m low.

```
python tools/route_from_walk.py ../../drone_tests/walks/walk-20261107-215520/vio.csv --timeline
python tools/route_from_walk.py WALK/vio.csv --t1 59 --height 0.9 --out routes/course.yaml --plot course.png
python tools/route_from_walk.py WALK/vio.csv --t1 59 --heights walk --min-height 0.5 ...   # rangefinder heights
```

`course_walk_DRAFT.yaml` is **lap 1 from leaving the launch mark (13 s) to where the drone was set on the floor at ~60 s** (rangefinder 0.10 m at (−3.6, −0.7)). That is assumed to be the pad. It has 14 waypoints, 15 m, and a fixed 0.9 m height. **Check before flying:**
1. Is ~60 s really the pad? Is there a low passage (the tunnel) that needs `--heights walk` or a hand-lowered waypoint?
2. **Placement:** VIO's start frame = how the drone sat at power-on for the walk. Put it on the **same spot with the same heading**; mark the legs with tape. Every 2° of heading error is 0.17 m sideways at 5 m.
3. The walk was hand-carried, so the path is only as good as where the walkers stepped. Waypoints near walls may need moving away from them by hand.

## Install on the drone (when ready)
From the repo root on the laptop:
```bash
tar -C team_src -cf - px4_route | ssh skybeta-18 'cd ~/team_ws/src && rm -rf px4_route && tar -xmf - && find px4_route -type f -exec sed -i "s/\r$//" {} +'
ssh skybeta-18 'mkdir -p ~/team_ws/routes && cp ~/team_ws/src/px4_route/routes/test_line.yaml ~/team_ws/routes/route.yaml'
```
On the drone: `sudoedit /etc/skybeta/drone.env` → `COMPETITOR_PKG=px4_route`, `COMPETITOR_LAUNCH=px4_route.launch.py`, then
`~/team_ws/drone_tests/bg.sh normal` (rebuilds team_ws and launches it). The competitor window must show
`route "test-line" ... Pilot: arm on Altitude, then switch to Position.`
**Swap routes** by copying a file to `~/team_ws/routes/route.yaml` and running `bg.sh normal` again. No rebuild needed.
**Back to px4_square:** set `COMPETITOR_PKG=px4_square`, `COMPETITOR_LAUNCH=px4_square.launch.py`.

Before every flight:
- power-cycle on the launch spot, untouched for 2 min;
- **no camera-image recording running**: it delayed VIO 170–340 ms and made PX4 jump metres (walk 21:55). Light VIO + PX4 recordings are fine;
- pilot on the kill switch.

## Test plan and speeds
**Speed limits that matter**, from what we measured:

| Limit | Number | Effect |
|---|---|---|
| PX4 lag behind a moving position setpoint | ≈ speed / `MPC_XY_P` (0.95) ≈ **1 m per m/s** | Harmless on straights (the carrot waits). The cause of corner cutting, which is why corners are slowed. |
| VIO | camera-only (`enable_imu_fusion: false`), ~70 ms latency | Motion blur and fast turns are its weak spot. Keep `yaw: fixed` or ≤ 30 °/s, and accelerate gently. |
| VIO drift | < 1 % of distance on the walk | ~0.15 m over this 15 m course. Fine at any speed. |
| Corridor clearance | (corridor width − drone width) / 2, probably 0.2–0.4 m | Max cross-track has to stay below this. **This is the real limit.** |
| Takeoff slide | 0.8 m backwards (log 26) | PX4 thinks level is ~5° nose-down. Fix with a level calibration (ask the organisers) before tight courses. |

Crude sim of `course_walk_DRAFT` (P-controller + 0.1 s lag; real PX4 is worse at corners):

| cruise / corner (m/s) | time | max off-path |
|---|---|---|
| 0.3 / 0.2 | 64 s | 0.08 m |
| 0.5 / 0.25 | 46 s | 0.09 m |
| 0.75 / 0.3 | 37 s | 0.12 m |
| 1.0 / 0.35 | 32 s | 0.15 m |
| 1.5 / 0.5 | 23 s | 0.22 m |
| 2.0 / 0.6 | 20 s | 0.27 m |

**Ladder:** go up one step only when the previous one finishes with
- max cross-track **< 0.2 m** (from `runs/route-*.csv`),
- no PX4 position resets,
- no wall touches,

twice in a row.
1. `test_line` @ 0.2 m/s → `test_box` @ 0.3 m/s (open floor, off-course)
2. course route @ 0.3 / 0.2
3. @ 0.5 / 0.25
4. @ 0.75 / 0.3
5. @ 1.0 / 0.35 (probably the practical ceiling with camera-only VIO in narrow corridors)
6. above 1.0 only if step 5's max cross-track was < 0.15 m

Scoring: finishing beats everything. A 64 s finish at 0.3 m/s still scores 35 + 20 × (fastest/64), far above any crash.

## The return over the walls: open questions
- **Re-arming:** checkpoint 5 needs **land AND disarm** on the pad, and the return needs an **autonomous takeoff** after that. This stack never arms (flight_interface has no arm, and `/fmu/in` is off limits), and the pilot may only kill. **Ask the organisers how the return re-arms** before building it.
- **Height:** over the walls means above 1.73 m. Use a separate route file for the return, with `max_height` ~2.3 and the launch arg `max_altitude_m:=2.5`. Never use those values on the main run, where flying above the walls costs 85 % of a checkpoint.
