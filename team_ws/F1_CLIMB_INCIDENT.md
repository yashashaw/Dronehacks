# F1 climbed far too high (drone 7, 2026-11-07): cause CONFIRMED from the logs

**DO NOT fly F1-F7 (or anything using team_takeoff_land / team_flight_test) autonomously
again until the fixes below are in and tested.** `ft.sh fly` refuses without
`--ack-f1-incident` for this reason.

## What happened

`ft.sh fly F1` was loaded twice (22:08:06, then 22:09:56). Pilot armed, switched to Position;
the drone took off and **kept climbing, nearly hitting the ceiling**, until it was killed. F1
should climb to 1.0 m, hold 3 s, land. The OAK-D camera was found dismounted. The team moved to
another drone.

## What the logs show

Data: `team_ws/incidents/F1_2026-11-07/` (copied from drone 7's `~/team_ws/field_tests/runs/`).
The flight run itself (`220956`) is **empty**: the battery came out before Linux wrote it to disk.
The run loaded two minutes earlier (`220806`, 107 s, drone on the floor, disarmed, Altitude)
survived and shows the state just before the flight:

| | height | sideways |
|---|---|---|
| **VIO** (`/visual_slam/tracking/odometry`) | **26.55 m** | x 10.3 m, **y 52.9 m** |
| PX4 EKF2 (`-vehicle_local_position.z`) | 0.04 m | |
| rangefinder (`dist_bottom`) | 0.10 m (on the floor) | |

**VIO was tens of metres wrong while the drone sat still**, and still reported `vo_state = 1`
(tracking). Every run that day: 21:47 VIO z 35.6 m / y 64.9 m; 21:59 (just after a stack
restart) z -3.2 m; 22:08 z 26.6 m / y 52.9 m. Its error moved by tens of metres between runs.

## The chain that made it climb

1. `team_takeoff_land/logic.py:99-101`: takeoff target = **VIO ground z + 1.0** = 26.6 + 1.0 =
   **~27.6 m**.
2. Published on `/skybeta/setpoint/pose` to `flight_interface`, which our launch files run with
   **`anchor_setpoints` = false** (default; `takeoff_land_test.launch.py`, `flight_test.launch.py`).
   So `frames.py` only flips ENU->NED and PX4 receives **z = -27.6 m in EKF2's frame**, whose
   zero is the floor: "climb to 27.6 m".
3. The `flight_interface` ceiling (`max_altitude_m` = 3.0 for F1) is measured from **VIO's own
   origin** (captured at ~26.6 m), so 27.6 m was inside it. Nothing clamped.
4. `takeoff_land` has no over-height watchdog; PX4 has no vertical geofence
   (`GF_MAX_VER_DIST = 0`) and may climb at 3 m/s (`MPC_Z_VEL_MAX_UP = 3`).

PX4's own height was right the whole time: it does not use vision for height
(`EKF2_EV_CTRL = 9` = horizontal + yaw; height from rangefinder `EKF2_HGT_REF = 2`, baro backup).
It climbed because it was **told** to go to ~27 m.

**Two faults, either one alone would have prevented it:** VIO producing garbage (most likely the
loose camera), and our code passing VIO's **absolute** height to PX4 with no anchoring and no
sanity check. With anchoring, only VIO's *change* in height matters: target ~1 m.

Also: EKF2 fuses VIO horizontally (`EKF2_EV_CTRL` bit 0), so a VIO this wrong also poisons
PX4's horizontal position, and **Position mode itself is unsafe** in that state.

## Separate bug found in the same data

The wall logger found a wall plane in **0 of 845 frames**, although the depth centre read 2.0 m
(the wall the drone faced). `wall_logger.py`'s plane fit is broken (fit returned ~0.06 m); the
wall numbers in the field-test reports cannot be trusted until it is fixed.

## Fixes before flying again (none applied yet, none flight-tested)

1. **`anchor_setpoints: True`** for `flight_interface` in `takeoff_land_test.launch.py` and
   `flight_test.launch.py` (it subtracts VIO's origin and adds EKF2's, `frames.py:164-167`).
2. **Pre-flight gate in `ft.sh fly` / `takeoff_land`**: refuse when VIO and EKF2 disagree on the
   ground (`|vio_z - (-fc_z)|` > 0.3 m), or VIO is far from where the stack started.
3. **Height watchdog independent of VIO** in `takeoff_land`: land if the rangefinder
   (`dist_bottom`) exceeds target + 0.5 m or 2.0 m absolute.
4. **Find why VIO went bad**: check the camera mount is rigid; `ft.sh walk W0` (still, 60 s)
   must show VIO drift of millimetres, not metres; restart the stack (re-zeroes VIO) right
   before each flight session.
5. **Ask the organizers** for PX4-side limits while testing: `GF_MAX_VER_DIST` (e.g. 2.5 m) with
   a geofence action, and a lower `MPC_Z_VEL_MAX_UP` (e.g. 1.0 m/s).
6. Fix the wall-plane bug in `wall_logger.py`.
7. Retest F1 at `--alt 0.7` with the pilot ready to take over at the first sign of a climb.

## Data lost, and how to keep it next time

The flight's own run is empty because the drone lost power before the filesystem flushed.
After any incident: **wait ~30 s (or run `sync` on the drone) before pulling the battery**,
and download the PX4 log from the Pixhawk SD card (QGroundControl -> Analyze Tools -> Log
Download), which is the best record of what PX4 was told and did.
