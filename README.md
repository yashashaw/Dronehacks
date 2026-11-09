# DroneHacks: drone 18 flies the course by itself

Autonomous indoor flight for the DroneHacks course on a Sky Beta drone (Jetson Orin Nano + Pixhawk 6C/PX4,
OAK-D stereo camera, downward rangefinder). The drone takes off from the launch mark, flies the maze, the dark
tunnel and the window, lands and disarms on the pad, then **re-arms by itself and flies back over the walls to
the launch mark**.

> The competition rules and course description that used to be this README are in
> [`README-competition.md`](README-competition.md). Course measurements: [`README-course-dimensions.md`](README-course-dimensions.md).

---

## Best results

All times are from our own flight logs (takeoff to disarm). Official times are read by the referees from video.

| Run (UTC, 2026-11-08) | What it did | Course time (to last waypoint) | Takeoff → disarm on pad | Return |
|---|---|---|---|---|
| **19:58** | **Full course + return to base** | **58.5 s** | **70 s** | Re-armed after 3 s, flew around the ladder at 2.3 m, came down **0.27 m** from the launch point |
| 14:39 | First full autonomous course | 61.8 s | 74.7 s | (return not enabled) |

- Tracking on the course: median **0.02–0.035 m** off the planned path (PX4's own position).
- Speed that works: **0.3 m/s cruise, 0.2 m/s corners**. 0.5 m/s was tried three times and crashed every time:
  see [`drone_tests/flights/2026-11-08_fast_runs_0.5`](drone_tests/flights/2026-11-08_fast_runs_0.5/README.md).

Flight records with the exact route that flew, the logs and plots:
[`14:39`](drone_tests/flights/2026-11-08_1439_course_success/) ·
[`19:58`](drone_tests/flights/2026-11-08_1958_course_and_return/) ·
[`0.5 m/s attempts`](drone_tests/flights/2026-11-08_fast_runs_0.5/)

---

## How we met every point on the field test

The run is scored on 5 checkpoints in order (35 points), speed, the window (+5) and the return (×1.25).

| # | Checkpoint (rules) | How our run clears it | Evidence |
|---|---|---|---|
| 1 | **First corridor**: reach the end of the first corridor | Waypoints 1–2 follow the floor line out of the launch corridor and make the turn | Route built from a hand-carried walk along the floor line; 14:39 and 19:58 logs |
| 2 | **Maze**: leave the maze through Gate A | The top edge (waypoints 2–5) runs along the maze corridor and exits toward the tunnel | Same; the top edge position was tuned after net and wall contacts (see below) |
| 3 | **Tunnel**: come out of the south end of the dark tunnel | The tunnel is one straight leg at **1.0 m** (into the tunnel at the top-right corner, out at the bottom-right) | 19:58: 0.035 m median off the path through the tunnel |
| 4 | **Chicane**: enter the landing area through the **window** | Rises to **1.3 m** and flies straight through the window (sill ~0.9 m) | Window crossing measured on the walk; flown cleanly at 14:39 and 19:58 (**+5 window bonus**) |
| 5 | **Landing**: land and disarm with the whole drone inside the pad's outer ring | Flies level to the pad at 1.15 m, hovers 1 s, comes straight down; PX4 disarms 2 s after touchdown | 19:58: landed and disarmed 0.14 m from the pad waypoint. The scoring ring is 1.52 m across, so the 0.33 m drone has ~0.6 m of slack |
| Rule | **Stay below the walls (68 in = 1.73 m)** on the run | Every course waypoint is 1.00–1.30 m; routes above 1.5 m are refused by the code | Height profile in every flight record |
| Bonus | **Return (rule 7.10)**: after disarming on the pad, take off by itself, fly back over the barrier, land within 2 ft (0.61 m) of the launch mark, within 3 min | Waits 3 s, sends one Offboard + one ARM command, climbs **straight up to 2.3 m**, flies **around the ladder** (pad → R1 → R2 → over the launch mark), comes **straight down**, lands 15 cm to the right of the mark (away from the net) | 19:58: re-armed and flew the return; came down 0.27 m from the launch point. Whole run ~2 min, inside the 3 min limit |

---

## Our approach

### 1. Fly on PX4's own position, never on our own estimate

Our first autonomous attempts (with the organizers' example) **climbed into the net**: the example took its height
from the camera tracking, while our PX4 uses the **rangefinder** for height (`EKF2_HGT_REF=2`) and the camera only
for horizontal position and yaw (`EKF2_EV_CTRL=9`). The two disagreed, and the example kept commanding "up".
Our route follower takes **every position, including height, from PX4's `vehicle_local_position`**.

### 2. A small waypoint follower: `team_src/px4_route`

- Waypoints are written as `[forward, left, up]` in metres **from the takeoff point**, so the route works wherever
  the drone is placed (we place it ~4 in behind the X, nose toward the pipe wall).
- A **"carrot"** target slides along the path at the planned speed, but only while the drone keeps up. The drone
  follows the line instead of cutting corners. It slows to the corner speed near every turn and at the end.
- **Nose forward** (`yaw: path`): the camera always looks where the drone is going. At corners it turns in place
  first (within 20°) and turns at most 30°/s.
- The only thing that talks to PX4 is the organizers' `flight_interface` (it clamps every target to a height band).
- **Safety:** routes above 1.5 m (2.4 m for the return) or with bad numbers are refused before flight; any internal
  error lands straight down; the node only takes over on a fresh Altitude → arm → Position sequence; every flight
  writes a 10 Hz CSV and an event log.
- 25 unit tests and a **mission simulator** (fake PX4 with arming, landing detection and auto-disarm) that runs
  the real node code through course → pad → disarm → re-arm → return → disarm before anything is flown.

### 3. The route came from walking the course

We carried the running drone along the course's floor line and recorded PX4's position and the rangefinder
(`drone_tests/walks/walk-20261107-215520`). That walk gave the waypoints and the heights: 1.0 m in the tunnel,
1.3 m at the window (the rangefinder saw the sill at ~0.9 m). The CAD model was only used for rough overlays: its
window and chicane do not match the built course.

### 4. Then we fixed it from real flights

| Problem seen | Fix |
|---|---|
| Climbed into the net (organizers' example) | Own follower on PX4 position |
| Top edge at 1.41 m forward hit the net (12:29, 12:36) | Top edge moved to 0.65–0.67 m |
| Rclpy logging bug killed the node in the air | One log severity per call site + crash-proof loop that lands on any error |
| A new flight could start from leftover Offboard | Takeoff only on a fresh Offboard switch |
| Landed off the pad centre | Pad waypoint corrected twice from overhead photos (drone size as the ruler) |
| Ladder stands over the straight return line | Return routed around it at 2.3 m |
| Return landing close to the net | Return lands 15 cm right of the launch mark |
| Rear props touched a wall on the top edge at 0.5 m/s | Extra waypoint 10 in further from that wall; speed back to 0.3 |

### 5. Speed

| Speed / corners | Result |
|---|---|
| **0.3 / 0.2** | **Completed the course (14:39) and the course + return (19:58)** |
| 0.5 / 0.3 | Hit the wall on the top edge (20:05); hit something after the tunnel exit (23:37) |
| 0.5 / 0.2 | Hit something right after takeoff (23:44) |

Camera tracking held 0.03–0.11 m at 0.5 m/s, so localization was not the limit. The course has only a few
centimetres of margin on the top edge and at the tunnel exit, and the drone swings ~13 cm wider at corners at
0.5 m/s. Details: [`drone_tests/flights/2026-11-08_fast_runs_0.5`](drone_tests/flights/2026-11-08_fast_runs_0.5/README.md).

---

## Repository map

| Path | What |
|---|---|
| `team_src/px4_route/` | **The flight code**: route follower node, launch file, route files, tests, mission simulator |
| `team_src/px4_route/routes/course_v2_return.yaml` | The course (10 waypoints, 0.3 / 0.2) with the return switched on |
| `team_src/px4_route/routes/course_v2.yaml` | Same course without the return |
| `team_src/px4_route/routes/return_to_x.yaml` | The return to base |
| `drone_tests/` | Capture and status scripts, the course walk, flight records |
| `planner/` | Theta* planner with frontier exploration (simulation only, never flown) |
| `team_project/` | Team index: the workspace at 14:39, the CV work (YOLOv8n pad detector, stereo hazard bench), branch summaries |

## Flying it

1. Copy the route to the drone as `~/team_ws/routes/route.yaml` (and `return_to_x.yaml` next to it), restart the
   stack with the drone on the ground and disarmed: `docker rm -f skybeta-stack`.
2. Place the drone ~4 in behind the X, nose toward the pipe wall. Power-cycle and leave it still while it boots.
3. Check: `~/team_ws/drone_tests/dt.sh status` and the competitor window must show `route "course-v2-return"`,
   `RETURN ENABLED` and `relaying PX4 position`.
4. Transmitter: **Altitude → arm → Position**. Stay clear after it lands on the pad: it re-arms by itself ~5 s later.
   Flip the mode switch off Position on the pad to cancel the return. Kill switch at any time.
