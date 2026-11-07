# Session notes, 2026-11-07: drone 7 camera, depth, test kit

> **WARNING (2026-11-07): F1 climbed nearly to the ceiling on its first flight.** Read
> `team_ws/F1_CLIMB_INCIDENT.md` before any autonomous flight. Likely cause: VIO-frame height
> targets sent to PX4 without `anchor_setpoints`. `ft.sh fly` now refuses unless given
> `--ack-f1-incident`.

Everything done on **skybeta-07** in this session: what was measured, what changed, how to
undo it, and what is still open. Copies: laptop `Desktop\Sim Example\Dronehacks\team_ws\` and
the drone's `~/team_ws/`.

---

## 1. Getting in

- USB-C from the laptop to the Jetson: `ssh dronehacks@192.168.55.1` (`hostname` must say
  `skybeta-07`). Over Wi-Fi: `ssh dronehacks@skybeta-07.local`.
- An SSH key from this laptop (`~/.ssh/id_ed25519`) is installed on the drone, so no password
  is needed for ssh. `sudo` still asks for the password.
- The drone has no internet, so its clock says 1969/1970. Harmless, but after every stack
  restart the service waits **90 s for a clock sync** before starting. A restart takes ~2 min.

## 2. The camera decision (the main result)

**Problem.** Depth was off on the real drone, but Plan 3/4 need wall distances. The OAK-D's
internal link (XLink, ~120-140 MB/s; not the USB cable, which is USB 3) was full with the two
1024x640 VIO images, which the driver sends as 3-byte colour.

**Tested** (bench, camera only, 40 s each):

| Setup | Link | Images | Depth |
|---|---|---|---|
| today: 1024x640, no depth | 118 MB/s | 30 Hz, 0 lost | none |
| **640x400 + depth** | **61 MB/s** | 30 Hz, 0 lost | 30 Hz |
| 1024x640 + depth | 133 MB/s | 0.3-0.4 % lost | 1.7 % lost (at the limit) |
| rectified mono images | - | not produced by this driver | - |

**Then with the whole stack running (VIO included):**

| | 1024x640, no depth | **640x400 + sharp depth** |
|---|---|---|
| VIO rate | 30 Hz, no gaps | 30 Hz, no gaps |
| VIO delay to the flight controller (p95) | 67 ms | **39 ms** |
| Jetson CPU for camera + VIO | ~62 % | **~44 %** |
| depth | none | **640x400, 30 Hz, 24 ms old, subpixel** |

**Decision: 640x400 + depth.** As good or better on every number, and it adds wall distances.
Two traps found on the way: the earlier "depth" profile used `i_depth_preset: DEFAULT`, which
silently halves depth to 320x200; and the earlier 1024x640+depth profile measured VIO at
27 Hz / 304 ms delay. Both are replaced.

Still unknown: whether 640x400 makes **drift over a long route** worse. The walk tests answer that.

## 3. What changed on the drone (and how to undo it)

| Change | Where | Undo |
|---|---|---|
| "depth" camera profile now = 640x400 + sharp depth | `~/skybeta/container_scripts/run_slam.sh`, message in `~/skybeta/host/run.sh` | backup `run_slam.sh.pre640` |
| **Camera is ON the new mode** (persists across reboots) | `~/team_ws/slam_profile` = `depth` | `~/team_ws/field_tests/ft.sh camera old` |
| Boot mission slot points at our launcher, **autostart OFF** (the example 5 m square no longer starts at boot) | `/etc/skybeta/drone.env` (`COMPETITOR_PKG=team_flight_test`, `COMPETITOR_LAUNCH=autostart.launch.py`), `~/team_ws/autostart.conf` (`enabled=false`) | backup `/etc/skybeta/drone.env.bak.19691231-171428` |
| Mission packages copied to the drone | `~/team_ws/src/team_flight_test`, `~/team_ws/src/team_takeoff_land` | - |
| Field-test kit | `~/team_ws/field_tests/` | delete the folder |
| Sweep tool: new variants `d_640s`, `nopreset` | `~/team_ws/loc_stack/depth_sweep/make_variant.py` | backup `.bak` |
| Bench scripts from the first benchmark | `~/cam_bench/` | delete the folder |

Not changed: PX4 parameters, the flight stack image, the bridge, the uXRCE agent.

## 4. Bugs found and fixed

| Bug | Effect | Fix |
|---|---|---|
| `flight_test_node.py` defined a method `handle` | rclpy uses `Node.handle` internally: the node **crashed on start-up**, every time | renamed to `handle_route_out` |
| `getattr(self.get_logger(), level)(text)` in `flight_test_node.py`, `takeoff_land_node.py`, `takeoff_land_mission.py` | rclpy (Humble) raises "Logger severity cannot be changed between calls" the first time one line logs at a new level: **the node dies**, possibly mid-flight | `log_at()` helper, one line per severity |
| `autostart.sh disable` did not repoint `drone.env` | the example 5 m square would still start at boot | `disable` now repoints too |

All three ROS nodes were then started on the drone and stayed up. Unit tests still pass.

## 5. Start and kill commands

**Field tests** (`~/team_ws/field_tests/ft.sh`, details in `field_tests/README.md`):

| | Command |
|---|---|
| status | `ft.sh status` |
| camera | `ft.sh camera new` / `old` / `status` (restart, disarmed, ~2 min) |
| ground check, props off | `ft.sh watch` (pilot arms/disarms/kill switch; `ft.sh kill` test) |
| walk | `ft.sh walk W2 try1` ... `ft.sh stop` |
| flight | `ft.sh fly F2` ... pilot ARMS, flips to POSITION ... `ft.sh done` |
| **kill** | `ft.sh kill` (lands, stops the mission + logger). **RC kill switch = instant** |
| results | `ft.sh analyze latest`, `ft.sh compare C1 C2 C3`, `ft.sh list` |

**Mission autostart** (`~/team_ws/src/team_flight_test/scripts/`):

| | Command |
|---|---|
| start at boot + now | `autostart.sh enable --profile hover [--alt 1.0 ...]` |
| off | `autostart.sh disable` |
| check | `autostart.sh status` |
| kill | `kill_flight.sh [--disable]` (from the laptop: `ssh dronehacks@192.168.55.1 '~/team_ws/src/team_flight_test/scripts/kill_flight.sh'`) |

Nothing ever arms from the Jetson. A flight starts only when the pilot arms and then flips
to Position. `kill_flight.sh` **lands** (it does not cut motors in the air).

## 6. The test plan

**Truth comes from things that do not move**: the start mark (closure error), the wall (its
position in VIO coordinates must never change, so any change is VIO drift), and tape marks.
That is how one config per walk is enough: each run gets a score against truth.

- **Ground check** (props off): `ft.sh watch`; each motor alone in QGroundControl.
- **Walks** at the practice wall: W0 still, W1 depth vs tape, W2 approach (distance scale),
  W3 turns (the sim's failure), W4 sideways, W5 lose and find the wall.
- **Real course walks** C1-C3: same start mark and heading each time. The camera surveys the
  walls (we don't know their positions otherwise); `ft.sh compare` overlays the three maps.
- **Flights** at the practice wall, in order: F0 rehearsal (props off), F1 take off and land,
  F2 hover, F3 +-45 deg turns (new `yaw45` profile), F4 +-90 deg, F5 1 m toward the wall
  and back, F6 sideways (new `strafe` profile), F7 0.7 m box. In all of them the drone flies
  on VIO only; depth is recorded to check it. Acting on depth comes after F2-F5 look clean.

## 7. Open items

0. **F1 incident: see `F1_CLIMB_INCIDENT.md`.** First autonomous takeoff climbed to near the
   ceiling and was killed; camera found dismounted; team moved to another drone.
1. **Nothing has been tested against a real wall yet.** First thing: `ft.sh walk W1` at the
   practice wall. If the report says "wall plane found in 0%" while facing the wall, the plane
   fit needs looking at.
2. **No battery was connected** all session, so no motor has turned. Ground check first.
3. Per-motor tests are only possible from QGroundControl (this PX4 build rejects them over DDS).
4. A **359 ms VIO gap** was seen once at mission start-up (the mission lands at 500 ms). The
   logger's own data never showed more than 67 ms. Watch the "VIO gap" warnings in F1/F2.
5. The live stack once showed short VIO stalls (one of ~355 ms) at 1024x640; the camera itself
   was clean, so it is Jetson-side. 640x400 lowers the load; check again in the flight reports.
6. Not built yet: the mission code that **uses** depth (stop at a distance from the wall, shift
   the remaining waypoints by the measured error), and the offline "would the correction have
   helped" replay. Both wait for real wall data from the walks.
7. `EKF2_EV_DELAY`: VIO now reaches the FC faster (p95 39 ms vs 67 ms). Re-measure with
   `~/skybeta/tools/latency_probe.py` before flying the new mode seriously.

## 8. Files

| Laptop (`Desktop\Sim Example\Dronehacks\team_ws\`) | Drone (`~/team_ws/`) | What |
|---|---|---|
| `field_tests/ft.sh` | same | every test command |
| `field_tests/wall_logger.py` | same | VIO + wall plane + depth scan + PX4 state, CSV (listens only) |
| `field_tests/analyze.py` | same | report.txt, summary.json, path.png, wall.png; `--compare` |
| `field_tests/motor_check.py` | same | `watch`: arm/disarm/mode log for the ground check |
| `field_tests/README.md` | same | the test checklist |
| `src/team_flight_test/launch/autostart.launch.py` | same | boot launcher, reads `autostart.conf` |
| `src/team_flight_test/scripts/autostart.sh`, `kill_flight.sh` | same | autostart on/off, kill |
| `src/team_flight_test/team_flight_test/route_logic.py` | same | + `yaw45`, `strafe` profiles |
| - | `~/skybeta/container_scripts/run_slam.sh` | the 640x400 + depth profile |
