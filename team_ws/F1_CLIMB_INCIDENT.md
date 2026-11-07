# F1 climbed far too high (drone 7, 2026-11-07)

**DO NOT fly F1-F7 (or anything using team_takeoff_land / team_flight_test) autonomously
again until this is confirmed from the logs and fixed.**

## What happened

`ft.sh fly F1` (run `20261107-220956_F1_takeoff_land`): pilot armed, switched to Position, the
drone took off and **kept climbing, nearly hitting the ceiling**, until it was killed. F1 is
supposed to climb to 1.0 m, hold 3 s, and land. The OAK-D camera was found dismounted
afterwards (timing relative to the climb: not yet known). The team moved to a new drone.

## Most likely cause (from the code; to be confirmed with the logs)

**The takeoff height is computed in VIO coordinates but flown by PX4 in its own (EKF2)
coordinates, and nothing converts between the two.**

1. `team_takeoff_land/logic.py:99-101` sets the takeoff target to
   `VIO ground z + takeoff_altitude_m` (1.0 m), from `/visual_slam/tracking/odometry`.
2. It publishes that on `/skybeta/setpoint/pose` to `flight_interface`.
3. Our launch files start `flight_interface` with **`anchor_setpoints` left at its default,
   false** (`takeoff_land_test.launch.py`, `flight_test.launch.py`). With it false,
   `skybeta_flight_interface/frames.py` only flips ENU to NED: the VIO height goes to PX4 **as if
   VIO's zero and EKF2's zero were the same point**. With it true, the VIO origin is subtracted
   and EKF2's origin (captured at the same instant) added (`frames.py:164-167`).
4. They are not the same point. VIO's zero is where the drone was when the stack started.
   EKF2's height zero is set at flight-controller boot and, **on the ground, below the
   rangefinder's minimum range, EKF2 height runs on the barometer** (we saw
   `dist_bottom_valid=False` while sitting on the floor). Baro drifts by a metre or more over
   the hours the FC had been powered. Any offset between the two zeros is added straight onto
   "1.0 m".
5. Nothing caught it:
   - the `flight_interface` ceiling (`max_altitude_m` = 3.0 in `takeoff_land_test.launch.py`,
     2.0 in `flight_test.launch.py`) is measured in **VIO** coordinates, and clamping the
     setpoint does not remove a frame offset (the clamped value gets the same offset);
   - `takeoff_land` judges "arrived" with VIO only, and has **no over-height watchdog**;
   - PX4 has **no vertical geofence** (`GF_MAX_VER_DIST = 0`) and may climb at
     **3 m/s** (`MPC_Z_VEL_MAX_UP = 3`), so a few seconds is enough to reach a ceiling.

`team_flight_test/README.md` ("Known risks") and `analyze_flight.py` already flagged the
VIO-vs-EKF2 frame risk; it was never fixed before flying.

**The dismounted camera alone does not explain a climb.** PX4 does not use vision for height
(`EKF2_EV_CTRL = 9`: horizontal position + yaw only; height reference = rangefinder,
`EKF2_HGT_REF = 2`, conditional `EKF2_RNG_CTRL = 1`, baro backup). A loose camera would corrupt
VIO (and so `takeoff_land`'s idea of where it is), and horizontal control, but PX4's height loop
does not see it.

Other causes still to rule out with the data: rangefinder misreading on this airframe; VIO
origin set while the drone was held off the floor; the camera coming loose before the climb.

## How to confirm (old drone, props off, connect USB)

The field logger recorded the three heights side by side, 10x per second:
`~/team_ws/field_tests/runs/20261107-220956_F1_takeoff_land/log.csv`

| column | meaning |
|---|---|
| `vio_z` | VIO height (up positive), what our code thinks |
| `fc_z` | EKF2 height, **NED: negative = up** |
| `dist_bottom` | rangefinder distance to the floor: closest thing to the truth |
| `armed`, `nav_state` | 14 = Offboard, 18 = Land |

- **Frame offset** (expected cause): on the ground before arming, `vio_z` ~ 0 but `-fc_z` is
  clearly not 0. In the climb, `-fc_z` heads to about `-(vio target)` while `dist_bottom` and
  `vio_z` show the drone far above 1 m.
- **VIO wrong** instead: `dist_bottom` ~ `-fc_z` ~ the real height, `vio_z` disagrees.
- **Rangefinder / EKF2 wrong**: `dist_bottom` and `vio_z` agree on a big height, `-fc_z` says ~1 m.

Also get the PX4 log from the Pixhawk (QGroundControl -> Analyze Tools -> Log Download):
`vehicle_local_position.z / z_valid / dist_bottom`, `trajectory_setpoint.position[2]`,
`estimator_status_flags` (height source).

Copy everything: `scp -r dronehacks@192.168.55.1:~/team_ws/field_tests/runs ./field_runs`

## Fixes to make before flying again (none applied yet, none flight-tested)

1. **`anchor_setpoints: True`** for `flight_interface` in `takeoff_land_test.launch.py` and
   `flight_test.launch.py`, so setpoints are converted from VIO's origin to EKF2's.
2. **Pre-flight gate in `ft.sh fly`**: refuse if `|vio_z - (-fc_z)|` > 0.2 m on the ground.
3. **Height watchdog independent of VIO** in `takeoff_land`: land if `dist_bottom` (rangefinder)
   exceeds target + 0.5 m, or 2.0 m absolute.
4. **Ask the organizers** for PX4-side limits during testing: `GF_MAX_VER_DIST` (e.g. 2.5 m,
   with a geofence action) and a lower `MPC_Z_VEL_MAX_UP` (e.g. 1.0 m/s).
5. Retest F1 at `--alt 0.7` first, with the pilot ready to take over at the first sign of climb.
