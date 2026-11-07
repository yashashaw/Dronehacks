# loc_stack: localization health, hand-carry drift and known-object tests

Additive tools for drone 7's existing `skybeta` stack. Read `SPEC_DEVIATIONS.md` first: it
explains what was deliberately *not* taken from the spec, and what can't be tested while the
flight controller is unlinked.

| File | Runs on | What it does |
|---|---|---|
| `health_node.py` | drone (container) | Publishes `/loc/health` at 30 Hz. Subscribe-only; never writes to `/fmu/*`. |
| `walk_logger.py` | drone (container) | Logs odometry (30 Hz) + tracking state to CSV for a fixed time, detached. |
| `analyze_walk.py` | laptop | Finds your marks, reports drift / closure / tracking gaps / object dimensions. |
| `run_walk.sh` | laptop | `start`, `pull`, `health` wrappers with SSH retries. |
| `test_analyze_walk.py` | laptop | Self-test of the analysis on a synthetic rectangle (`python test_analyze_walk.py`). |
| `depth_wall.py` | drone (container) | Depth rate (by stamp), fill %, plane-fit distance and noise against a wall. |
| `depth_test.sh` | laptop | `on` / `measure` / `off` for the depth pipeline; restores the previous slam pipeline. |

## Safety
Props off, FC disarmed. Nothing here arms, commands, or publishes to the autopilot.
Do not run a second SLAM stack or a second `/fmu/in/vehicle_visual_odometry` publisher.

## Test A: stationary noise floor (1 minute)
```
./run_walk.sh start table 60        # drone sitting on a table
./run_walk.sh pull table
```
`dwells[0].jitter_m` is your VIO noise floor (about 1 cm over 20 s in the first run).

## Test B: known object (map its length and width)
1. Measure the object with a tape: LENGTH x WIDTH in metres. Mark its four corners.
2. Pick one fixed point on the drone (the nose). Hold the drone level at about the same
   height for the whole run.
3. `./run_walk.sh start object1 90`
4. Hold still **3 s** at the start (corner 1). Then move the point to corner 2, hold 3 s;
   corner 3, hold 3 s; corner 4, hold 3 s; and finally back to corner 1, hold 3 s.
   Move smoothly (about 0.3 m/s); keep the cameras uncovered and facing textured scenery.
5. `./run_walk.sh pull object1 --object LENGTH,WIDTH`

Report: the four edge lengths and two diagonals vs. truth (% error), corner angles (should
be 90 deg), `scale_factor_measured_over_true` (the VIO scale error), the height spread, and
`return_to_corner1_error_m`. Odometry x/y have an arbitrary heading, so only distances and
angles are used.

If a stray dwell appears before corner 1 (for example while you pick the drone up), pass
`--skip 1`.

## Test C: hand-carry "flight"
Carry the drone around the planned course at flight height, level and at walking speed,
holding still 3 s at each waypoint, and end at the start mark:
```
./run_walk.sh start carry1 120
./run_walk.sh pull carry1
```
`start_to_end_error_m` and `start_to_end_error_pct_of_path` are the drift. Also read
`max_odom_gap_ms` and `tracking_lost_events`.

The logger keeps running if you unplug USB-C, but only if the drone stays powered. Check how
it is powered before unplugging.

## Test D: is the 11.6 % scale error real? (repeated measured line)
The 11.6 %-short reading came from one hand-carried out-and-back with an *inferred*
turnaround; the IMU-fusion run (`imu1`) read 3.7 % *long*. This test settles it.

Kit: tape measure, floor tape, a cart or rolling chair (rigid carrier at about flight
height), a block or book to stop against. **Depth must be off** (`depth_test.sh off`).
1. Tape two floor marks **A** and **B** exactly 3.00 m apart (or what fits; measure it).
   Put the stop block at each mark so the cart stops in the same place every time.
2. Fix the drone on the cart, cameras facing along the line at textured scenery, level.
3. `./run_walk.sh start lineA_<config> 120`
4. Hold **5 s** at A. Push to B (about 0.3 m/s), stop against the block, hold 5 s. Back to
   A, hold 5 s. Repeat until you have walked **6 legs** (A B A B A B A), ending at A.
5. `./run_walk.sh pull lineA_<config> --line 3.00 --legs 6`

Read `line`: `scale_per_leg`, `scale_mean`, `scale_95ci` and `verdict`. If the interval
excludes 1.0 the bias is real; if it excludes 0.884, the 11.6 % figure is not. The
interval includes ±1 cm of tape uncertainty. `return_to_start_errors_m` is the drift
after each round trip. A hold-count error means a hold was lost or a stray one added;
re-run, or use `--skip`.

Do it once per pipeline configuration and never mix them. `config.txt` in the run folder
records which launch the slam pane was running. At minimum: the current one (IMU variant,
mapping off), then IMU fusion off. Rotate the cart 90 degrees for one run, so the line is
not always along the same camera axis.

## Test E: on-device depth on a plain wall
Kit: a blank wall, a textured poster or newspaper to tape on the same wall, tape measure.
1. `./depth_test.sh on` saves the current slam pipeline and switches it to depth. It ends
   with a 5 s check: `depth` should be about 28 Hz.
2. Place the drone with the cameras square to the wall, `T` metres from the camera front
   glass. For each T in 0.5, 1.0, 2.0, 3.0 (and 4.0 if there is room):
   - `./depth_test.sh measure blank_<T> <T>` on the blank wall
   - `./depth_test.sh measure tex_<T> <T>` with the poster centred in view
3. `./depth_test.sh off`. It restores the saved pipeline and prints the left/right and VIO
   odometry rates by header stamp; they should be about 30 Hz.

Read in each `runs/depth/<label>/result.json`: `rates_by_header_stamp` (depth, left,
right while depth is on), `valid_pct_roi_median` (fill on the blank wall is the
plain-wall answer; the S2 has no IR projector), and on the textured target
`plane_distance_m`, `error_pct`, `plane_rms_mm` (flatness noise) and
`plane_distance_frame_sd_mm` (frame-to-frame noise). `plane_tilt_deg` over ~5 means the
drone was not square to the wall.

Cross-check with Test D: on-device depth uses the camera's factory calibration, while
cuVSLAM uses the published `camera_info` (D = 0, no stereo rotation; see
`mapping/FINDINGS.md`). If depth is accurate on the textured target but VIO scale is off,
suspect `camera_info`, not the hardware.

## Health signal
`./run_walk.sh health`, then `ros2 topic echo /loc/health` in the container. OK under 100 ms
of odometry age with tracking; WARN at 100-300 ms or when `vo_state != 1`; ERROR over 300 ms
or if EKF2 reports `xy_valid` false. With the FC unlinked it reports `fc_linked: False` and
judges from VIO alone. `quality: 0` is the bridge's "unknown", not a fault.
