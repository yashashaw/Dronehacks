# drone_tests: real-drone tests, built up from the ground

Branch `drone-tests`. Kept small on purpose: each test is one script, added only after the
one before it works. Old field tests: branch `drone-field-tests` (drone 7). Sim results:
branch `sim-waypoint-run`.

| File | Runs on | What |
|---|---|---|
| `install.sh` | laptop | copies this folder to `~/team_ws/drone_tests` on the drone (checks hostname first) |
| `dt.sh` | drone (ssh) | `status`, `camera status/depth/stock`, `motors` |
| `depth_motors.py` | stack container (via `dt.sh`) | **test 1**: motor speed follows the depth at the centre of the camera view |
| `skybeta_depth640.patch` | drone `~/skybeta` | adds the depth camera profile (from drone 7, skybeta 0.3.3) |

## 1. Connect (drone 18 over USB-C)

`~/.ssh/config` on the laptop has `Host skybeta-18`: 192.168.55.1 with the host key pinned
to drone 18 (`HostKeyAlias`). Every Jetson is 192.168.55.1 over USB, so plain `ssh
dronehacks@192.168.55.1` trips over whichever drone was plugged in before. Use:

```
ssh skybeta-18            # USB
ssh skybeta-18-wifi       # Wi-Fi (skybeta-18.local)
```
After plug-in the USB network takes up to ~1 min before ssh answers.

## 2. Install

```
drone_tests/install.sh skybeta-18
```

## 3. Camera config (once per drone)

The depth stream (`/oak/stereo/image_raw`, 640x400, 16UC1 mm, 0 = no reading, nothing under
~0.31 m) needs the depth profile in the drone's `~/skybeta`. On the drone:

```
cat ~/skybeta/VERSION                                  # must be 0.3.3+
cd ~/skybeta && git apply --check ~/team_ws/drone_tests/skybeta_depth640.patch \
             && git apply ~/team_ws/drone_tests/skybeta_depth640.patch
~/team_ws/drone_tests/dt.sh camera depth               # disarmed + still; restarts the stack (~2 min)
~/team_ws/drone_tests/dt.sh camera status              # depth ON, 640 wide
```
If `--check` fails, the drone's skybeta is not 0.3.3. Do not force it.

`/etc/skybeta/drone.env` (sudoedit) must also be right for this airframe:
`DRONE_ID=18`, `ROS_DOMAIN_ID=18` (= the FC's `UXRCE_DDS_DOM_ID`), and the camera mount
`CAMERA_X..CAMERA_YAW` **measured on this drone** (base_link -> camera, FLU, m / rad).
Drone 7 had `CAMERA_X=0.1053 CAMERA_Z=0.0304`, rest 0. Don't copy that without measuring.

## 4. Test 1: depth -> motor speed (PROPS OFF)

```
~/team_ws/drone_tests/dt.sh motors                      # dry run: move your hand in front of the camera
~/team_ws/drone_tests/dt.sh motors --spin --props-off   # the real thing
```
Default: hand at 0.4 m -> 15 % throttle, at 2 m or more -> idle, linear in between.
Options: `--farther-faster`, `--near 0.5 --far 1.5`, `--max-throttle 0.10` (max 0.30).

Before `--spin`: props off, drone held down, RC transmitter on with the kill switch checked,
**Pixhawk USB unplugged** (PX4 will not arm with it in). The team approves before it arms.

**This test is the one exception to "never publish to /fmu/in/\*".** This PX4 build rejects
motor-test commands from the Jetson, so the only way to set motor speed from code is Offboard
direct-actuator, published straight to PX4. It therefore cannot run alongside flight_interface:
`dt.sh motors --spin` stops the competitor window, and restarts it when the test ends. The script
also refuses if a flight_interface node is still up. It arms itself, so it is bench only.

Stops by itself when: Ctrl-C, no depth for 0.5 s (motors stop) or 2 s (disarm), PX4 leaves
Offboard, or PX4 disarms. PX4 auto-disarms ~10 s after arming because the drone never takes
off (`COM_DISARM_PRFLT`). That is expected; run it again.

## Next tests (in order, each only after the last one is clean)

The sim run on `sim-waypoint-run` (match_run v4: full route, window, pad) depends on depth
wall stops, a depth-vs-walls scan matcher, and VIO that holds through turns. Check those on
hardware in this order:

1. depth -> motors (above): depth is live, centred, in metres, with the right sign
2. depth accuracy at tape marks 0.5 / 1 / 2 / 3 m, props off
3. VIO walks by hand: still, straight, 90° turns (the sim's main failure was turn jumps)
4. first flight: hover through `/skybeta/setpoint/pose`. The pilot arms in Altitude and
   switches to Position; flight_interface (`auto_offboard_on_arm: true`, exactly one copy)
   takes over. Flight code **never** arms and never publishes to `/fmu/in/*`. The sim's
   `match_run.py` does both, so it does not run on the drone as it is.

## 5. Boot mission: px4_square (no depth, no VIO height)

**Drone 18 runs WITHOUT depth mode.** Steps 3-4 above (depth profile, depth -> motors) and
`hover.py` / `drone_mission` (which need depth) are retired for this drone.

`team_src/px4_square` is the organisers' newest example square (climb 1.0 m above the switch
point, hover 5 s, 0.5 m square counter-clockwise, then HOLD; the pilot lands) with no height
from VIO anywhere:

* the mission's start, targets and "arrived" checks use PX4's `/fmu/out/vehicle_local_position`
  (height = rangefinder with baro backup, `EKF2_HGT_REF = 2`);
* `flight_interface` runs with `anchor_setpoints: false` (targets are already in PX4's frame) and
  `odometry_topic: /px4_square/odom`, a relay of PX4's position, so its 0.3 m floor / 2.0 m
  ceiling are measured from PX4, not from VIO (the default VIO origin can rewrite a 1 m target to
  "VIO origin + 0.3 m": the drone 7 climb);
* no takeover unless PX4's position is fresh and valid and its height on the ground is within
  0.3 m of zero (the node publishes nothing, so the pilot keeps Position).
* change from the example: leg 1 points along the nose **at the switch** (PX4 heading), not VIO +x.

VIO still reaches PX4 through the organisers' bridge; EKF2 must use it for horizontal + yaw only.

```
drone_tests/install.sh skybeta-18 --boot       # laptop: ~/team_ws/src/px4_square, removes drone_mission
# on the drone (disarmed, on the ground, still; team approves):
cat ~/skybeta/VERSION
rm -f ~/team_ws/slam_profile                    # depth mode file: gone
sudoedit /etc/skybeta/drone.env                 # COMPETITOR_PKG=px4_square
                                                # COMPETITOR_LAUNCH=px4_square.launch.py
                                                # delete any SKYBETA_SLAM_PROFILE line
git -C ~/skybeta status --short                 # if skybeta_depth640.patch was applied:
#   git -C ~/skybeta apply -R --check ~/team_ws/drone_tests/skybeta_depth640.patch
#   git -C ~/skybeta apply -R ~/team_ws/drone_tests/skybeta_depth640.patch
docker exec skybeta-stack bash -c 'source skybeta_env.sh >/dev/null 2>&1; team_build.sh --packages-select px4_square'
docker exec skybeta-stack bash -c 'source skybeta_env.sh >/dev/null 2>&1; ros2 launch px4_square px4_square.launch.py --show-args'
docker rm -f skybeta-stack                      # restart (~2 min), re-zeroes VIO
~/team_ws/drone_tests/dt.sh camera status       # depth OFF, 1024 wide
docker exec -it skybeta-stack tmux attach -t skybeta   # competitor window: "px4_square ready ..."
```

Before the first flight, on the flight controller (`loc_stack/preflight.sh` reads these):
`EKF2_EV_CTRL = 9` (bit 1 = vision height must be OFF), `EKF2_HGT_REF = 2`, `EKF2_BARO_CTRL = 1`,
`EKF2_RNG_CTRL = 1`, and EKF2 fusing vision (`cs_ev_pos`, `xy_valid`). Also re-check
`flight_interface.py` on the drone (newer than the repo's 0.3.2) still uses `odometry_topic` only
for the altitude band and honours `anchor_setpoints: false`.

The flight: pilot arms on Altitude, flips to Position within a few seconds; the square runs and
then holds; the pilot takes over and lands. Back to the stock example: `COMPETITOR_PKG=competitor_example`.
`drone_tests/COLCON_IGNORE` keeps `drone_tests/` out of the team build.
