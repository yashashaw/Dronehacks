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

## Health signal
`./run_walk.sh health`, then `ros2 topic echo /loc/health` in the container. OK under 100 ms
of odometry age with tracking; WARN at 100-300 ms or when `vo_state != 1`; ERROR over 300 ms
or if EKF2 reports `xy_valid` false. With the FC unlinked it reports `fc_linked: False` and
judges from VIO alone. `quality: 0` is the bridge's "unknown", not a fault.
