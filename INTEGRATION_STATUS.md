# Integration branch: status, algorithm, and known problems

**Read this first. Nothing on this branch has flown.** It has been tested in a simulator and with
unit tests (64 pass). The ROS node, launch file and Docker snippet have only been syntax-checked,
because ROS is not installed on the machine this was built on.

## What is on the branch

| Folder | Owner | What |
|---|---|---|
| `skybeta/`, `loc_stack/`, `team_ws/` | Lucas (A) | Drone stack, localization tests, depth tests, takeoff/land package |
| `course_pipeline/` | Petr (C) | Pad detector (no model yet), guide (only publisher of `/skybeta/setpoint/pose`), contract |
| `team_planner/` | Yasha (B) | Depth planner node, depth conversion, hybrid/scripted modes, tests |
| `planner/` | Yasha (B) | The simulator and the planner source (single source of truth), benchmark results |
| `tools/` | Yasha (B) | `sync_planner_core.py`, `make_course_hints.py` |

One edit was made to Petr's code: `pad_guide.py` forwards a heading carried by `/course/goal` while
following, so the camera can face the way the planner steers. An all-zero quaternion still means "no
heading". His tests still pass. Details in `team_planner/README.md`.

## The algorithm

**Pathfinding: plain A\* (8-neighbour grid search) on an occupancy grid built live from the depth
camera.** Cells near walls cost more, and the drone's size is added to every wall, so routes stay in
the middle of corridors. Theta\* (any-angle) is built in and selectable with `route_planner: theta`.
A\* is the default because it had the fewest wall touches in the benchmark
(`planner/benchmark_results.txt`); Theta\* was close at normal noise and worse when position drifted.

Around it:
1. **Mapping.** Depth image -> range readings in the height band the airframe occupies -> a map that
   starts tiny and grows as the drone explores. Nothing about the course is assumed.
2. **Goals.** The drone heads for the nearest useful edge of known free space ("frontier"), avoiding
   places it has already been and avoiding turning around. Frontiers must be reachable through
   known free cells. When a pad is seen, the pad becomes the goal.
3. **Following.** The route is followed with a short lookahead, and the goal is published as a point
   about `speed x 1 s` ahead of the drone.
4. **Speed regulation.** The drone slows where the route is tight, where it turns sharply, and when
   the free space it can see ahead is short. See below.
5. **Stuck recovery.** If it commands motion for 3 s but barely moves, it backs away, plans with a
   little more clearance, and avoids that goal for 30 s.

## Does it just slow down to finish?

No, but slowing down is part of it. What it does:
* It sets a **maximum speed** (0.5 m/s in the cautious config).
* Below that cap it **regulates speed** from the clearance left on the route, the sharpness of
  upcoming turns, and the stopping distance versus the free space it can see.

What actually got the simulated drone to finish was the **map-based planning, the choice of goals,
and the stuck recovery**, not a lower speed. In simulation, 1.0 m/s and 0.6 m/s both landed 30 of 30
at normal noise, and an earlier version that was simply slow still timed out often. Treat 0.5 m/s as
a safe first-run setting, not as the thing that makes it work.

## Modes (`planner_mode`)

| Mode | What it does | Needs | Generality |
|---|---|---|---|
| `general` | Frontier exploration on the live depth map. No course knowledge. | depth | fully general |
| `hybrid` | A hardcoded waypoint list picks the **goals**; the live map and A\* still choose the path and avoid obstacles | depth, `course_hints.json` | **course-specific** |
| `scripted` | Flies the waypoint list from odometry alone | odometry only | **course-specific** |

`publish_pad_hint:=true` (hybrid/scripted only) announces the hinted pad position as a confirmed
detection once the whole route has been flown and the drone is within 1 m of it, so Petr's guide can
approach and land while there is no real pad detector. It is marked `source: hint`.

**Simulator result, honestly:** hardcoding the goals did **not** help. With the perfect-map route:
normal noise 40/40 either way (47.6 s general, 49.3 s hybrid); drift x2 38/40 vs 40/40; drift x4
35/40 vs 34/40. The general planner already finds this route in simulation. The hybrid's value is
protection against real-world failures the simulator does not model (for example depth gaps on plain
walls stalling exploration), and a way to exercise landing even if perception fails.

## Problems, in order of importance

1. **Nothing has flown.** Every number in this repo about the planner is from simulation, with noise
   levels that are guesses until Lucas's measurements replace them.
2. **Landing is blocked.** The flight interface floors every setpoint 0.3 m above the start point,
   so the drone cannot reach the ground. `min_altitude_m` is exposed in the launch file (default
   0.3). **Ask the organizers** whether lowering it for landing is allowed. PX4 is configured to
   disarm itself 2 s after landing (`COM_DISARM_LAND = 2`); that has not been tested.
3. **No software land command is used**, on purpose: the rules say one start command then abort
   only, with no mode changes mid-run. `team_takeoff_land` (Lucas) sends one, so **do not launch it
   together with this**; it also publishes setpoints and the two would fight.
4. **No pad detector works.** Petr's YOLO slot has no weights, and `ultralytics`/PyTorch are not in
   the image. A classical ring detector on the mono image would not need either.
5. **No return home.** Not implemented anywhere. The rules start the return "after disarming on the
   pad" with an autonomous takeoff, but the flight interface never arms and allows one Offboard attempt
   per arming. How the return is meant to start needs an organizer answer. It would also need a
   climb above the 68 in barrier, which this map treats as a wall.
6. **Depth and VIO share the camera link.** Lucas's notes say VIO degrades while depth is on. Not
   measured in motion. If it is bad, the approach needs another depth source.
7. **Depth details unknown.** The depth stream is RGB-aligned 320x200. Until `/oak/stereo/camera_info`
   is confirmed, the node uses the left mono camera's field of view and warns. Lucas measured only
   32-42% valid pixels in a room; plain course panels are unmeasured.
8. **EKF2 can jump while VIO does not.** Lucas saw heading flips that moved EKF2's position by metres.
   The drone flies on EKF2 but the map is built in the VIO frame. The planner stops issuing goals when
   the two disagree by more than 0.5 m since the start (guide then holds), but a flip during a corner
   could still hurt.
9. **Jetson speed and dependencies.** scipy and numba must be added to the team image (`numpy<2` is
   pinned). The versions in `team_planner/docker/Dockerfile.snippet` are unverified on aarch64. The
   planner's per-frame time on the Jetson has not been measured; without numba it is far too slow.
10. **Yaw patch is unverified on the drone.** And the guide only forwards yaw while following.
11. **Altitude must match.** `flight_altitude_m` (planner) and `cruise_altitude_m` (guide) are the same
    number in the launch file (default 0.7 m). `camera_start_height_m` (0.10) is a guess to measure.
12. **Checkpoints, the floor line, and the window bonus are not used.** The planner picks Gate B or the
    window by whichever its exploration reaches first; hybrid mode takes Gate B.
13. **The reactive and sampling alternatives lost the benchmark;** that was in simulation only.

## Questions for the organizers

* May `min_altitude_m` be lowered at launch for landing? (Problem 2)
* How is the return leg supposed to start without a human arming? (Problem 5)
* What counts as a touch versus a crash, and do props count?
* Where are the checkpoint lines, and does "whole drone" include the props?
* Do images captured in practice flights count as "run data" under rule 6.4 if used to train a pad detector?

## Test status

| Check | Result |
|---|---|
| `team_planner` unit tests (depth conversion, run rules, modes) | 19 pass |
| `course_pipeline` tests (Petr) + the new yaw helper test | 45 pass |
| Node, launch file, setup compile | OK |
| Node on a ROS graph / on the drone | **not run** |
| Simulator: 100 flights, default planner | 100/100 landed, median 48.5 s |

## Plan for the first slow run

Goal: finish the course slowly, and record everything for general use.

1. **Bench, props off, disarmed.** `ros2 topic hz /course/goal` stays silent until Offboard starts.
   `/team_planner/debug` shows returns > 0 and a sensible `valid_fraction` with depth on.
2. **Hand-carry through the net, props off,** `planner_mode:=general`. Record a bag. This is the
   cheapest way to learn real depth coverage on the course panels.
3. **Tethered, low, safety pilot on RC,** `planner_mode:=scripted`. Tests the flight chain and yaw
   without depending on perception.
4. **Slow runs,** `hybrid` with `publish_pad_hint:=true` to exercise approach and landing (blocked
   by problem 2 until the organizers answer), then `general` for a clean data run.
5. **Record every run** with `team_planner/scripts/record_run.sh` from a second container. Use the
   bags to study drift, depth quality and planner decisions. Do not feed anything from one run into
   the next (rule 6.4): the planner resets at every arming and writes no files.

For a scored run, `general` is the one that fits the generality score. If it does not finish by
Sunday, `hybrid` or `scripted` can be used, at a cost to the generality points.

## Run

```bash
cp -r team_planner course_pipeline ~/team_ws/src/             # then: host/run.sh build  (team image with scipy, numba)
# /etc/skybeta/drone.env:  COMPETITOR_PKG=team_planner  COMPETITOR_LAUNCH=integrated.launch.py
docker rm -f skybeta-stack                                    # restart on the ground, drone still
ros2 launch team_planner integrated.launch.py planner_mode:=hybrid publish_pad_hint:=true   # by hand
```

Place the drone **on the launch mark, nose toward the north net side (+y of the course)** for the
hint modes; the hints assume that start pose (`team_planner/config/course_hints.json`, regenerate with
`python tools/make_course_hints.py`).
