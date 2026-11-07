# team_planner: depth planner for the skybeta drone (integration branch)

**Status: written against the repo docs and tested without a drone. Nothing here has flown.**
The logic and the depth conversion run under pytest (19 tests). The ROS node, launch file and
Docker snippet have only been syntax-checked.

**See `../INTEGRATION_STATUS.md` for the algorithm, the three planner modes (`general`, `hybrid`, `scripted`), the problem list and the plan for the first run.**

This branch combines three people's work without changing the shape of any of it:

| Piece | Who | Where |
|---|---|---|
| Localization, depth tests, health signal | Lucas (A) | `loc_stack/`, `skybeta/` |
| Pad detector, guide, flight interface hookup | Petr (C) | `course_pipeline/` |
| Path planner (this package) and its simulator | Yasha (B) | `team_planner/`, `planner/` |

## How it fits together

```
OAK-D depth ──/oak/stereo/image_raw──▶ team_planner ──/course/goal──▶ pad_guide ──/skybeta/setpoint/pose──▶ flight_interface ─▶ PX4
cuVSLAM ──/visual_slam/tracking/odometry──▶ (planner, guide)                        ▲
mono image ──▶ pad_detector ──/pad/detection──▶ (planner as a hint, guide to land)  │ only publisher
```

* `team_planner` turns the depth image into range readings (`depth_scan.py`), keeps a map that is
  rebuilt from nothing at every run, picks where to go, and publishes **one point and a heading**
  to `/course/goal` at 10 Hz. It never publishes a setpoint.
* `pad_guide` (Petr) is still the only node that publishes `/skybeta/setpoint/pose`. It takes off,
  steps toward the goal, and takes over when the pad is confirmed.
* **Do not also launch `team_takeoff_land`** (Lucas). It publishes setpoints too, and it sends
  PX4 a land command, which we are not using (see below).

## Changes made to other people's code (small, on this branch only)

1. `course_pipeline/pad_guide.py`: if `/course/goal` carries a heading, the guide forwards it as
   the setpoint's yaw **while following** (before, it never commanded yaw). The planner needs the
   camera to face the way it steers. An all-zero quaternion still means "no heading", so the old
   behavior is unchanged. `pad_math.py` gained three pure helpers; `CONTRACT.md` and a test were
   updated. All 45 of Petr's tests still pass.
2. Nothing else in `course_pipeline/`, `loc_stack/`, `skybeta/` or `team_ws/` was touched.

## Decisions made here, and why

* **Speed is set by how far ahead the goal is.** The guide moves a setpoint at most `step_m`
  toward the goal, and PX4 chases it roughly at (distance x gain). So the planner places the goal
  `speed x 1 s` ahead (clamped to 0.15-1.0 m), and `step_m` is raised to 1.0. No speed field needed.
* **Route finder: plain A\* by default.** In the benchmark (`planner/benchmark_results.txt`) A\*
  had the fewest wall touches and the best time-with-penalties under drift; Theta\* was close at
  normal noise and worse under drift. Set `route_planner: theta` to compare.
* **One altitude.** `flight_altitude_m` (planner) and `cruise_altitude_m` (guide) must be the same
  number. Default 0.7 m, chosen so the drone fits the window (opening is 0.3-1.2 m above the floor)
  and stays under the 68 in walls. Both are heights above the odometry origin, which is where the
  camera was at startup, so `camera_start_height_m` (default 0.10, **measure it**) converts to
  height above the floor for the depth band.
* **Everything is forgotten at every arming and every Offboard start** (rule 6.4). The node holds
  state only in memory and writes no files.
* **It stops publishing goals** (so the guide holds) when: the depth stops arriving for 0.5 s;
  `/loc/health` reads ERROR; or EKF2's position drifts more than 0.5 m away from the VIO position
  since the run began (Lucas's tests saw EKF2 heading flips that move the drone while VIO does not).
  WARN halves the speed.

## Landing: not a software command

The rules say one start command, then abort only, with no mode changes mid-run, so this branch
sends **no land or disarm command**. PX4 is set to disarm itself 2 s after it detects landing
(`COM_DISARM_LAND = 2`), so the plan is: the guide's LAND stage descends under normal setpoints,
the drone touches down, PX4 disarms. One thing blocks that today: the flight interface floors every
setpoint at `min_altitude_m` (0.3 m above the odometry origin), so the drone cannot reach the ground.
`integrated.launch.py` exposes `min_altitude_m` (default 0.3, unchanged). **Ask the organizers
whether lowering it for landing is allowed** before changing it. The guide's own note says "the
pilot lands"; that does not fit the rules and needs the organizers' answer too.

## Run it

Dependencies (scipy, numba, `numpy<2` pinned) go in the team image, never `pip install` in a
running container. See `docker/Dockerfile.snippet`. **Those versions are unverified on the Jetson.**

```bash
cp -r team_planner course_pipeline ~/team_ws/src/        # then: host/run.sh build  (team image)
# /etc/skybeta/drone.env:  COMPETITOR_PKG=team_planner   COMPETITOR_LAUNCH=integrated.launch.py
docker rm -f skybeta-stack                               # restart on the ground, drone still
```

Check without a drone: `cd team_planner && python -m pytest test -q` and
`cd course_pipeline && python -m pytest test -q`.

If `planner/` changes, run `python tools/sync_planner_core.py` to refresh the copies in
`team_planner/planner_core/` (they are generated; do not edit them).

## Test ladder for the first slow run

1. **Bench, props off, FC disarmed.** `ros2 topic hz /course/goal` stays silent until Offboard
   starts. Confirm `/team_planner/debug` shows `returns` > 0 and a sensible `valid_fraction` with
   the depth pipeline on.
2. **Hand-carry through the net, props off.** Watch the goals point along the corridors. This is
   also where we learn real depth coverage on the course's own panels.
3. **Tethered, low, safety pilot on RC.** Check `clamped` in `/skybeta/state` stays empty.
4. **Slow full runs** at `v_max` 0.5, then 0.4 if corners clip.

Record every run from a second container: `/visual_slam/tracking/odometry`,
`/visual_slam/status`, `/loc/health`, `/oak/stereo/image_raw`, `/oak/stereo/camera_info`,
`/course/goal`, `/team_planner/debug`, `/pad/detection`, `/skybeta/state`,
`/skybeta/setpoint/pose`, `/fmu/out/vehicle_local_position`.

## Known gaps

* Depth and VIO share the camera link; Lucas's notes say VIO degrades with depth on. Not yet
  measured in motion. If it is bad, this whole approach needs another depth source.
* The depth frame is RGB-aligned 320x200. Until `/oak/stereo/camera_info` arrives, the node uses
  the left mono camera's field of view as a stand-in and warns.
* No pad detector weights exist yet; the planner only uses a detection as a hint.
* The planner's per-frame time on the Jetson is unmeasured (the simulator numbers are laptop numbers;
  the Jetson was about 4x slower on a plain-Python loop).
* Checkpoint lines, the floor line and the return leg are not handled.
* The simulator's noise numbers are placeholders until Lucas's measurements replace them.
