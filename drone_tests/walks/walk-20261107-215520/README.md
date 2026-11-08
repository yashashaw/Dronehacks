# Walk 2026-11-07 21:55: drone 18 carried around the course (~3 laps)

Recorded with `bg.sh capture 360` (both cameras, IMU, VIO, bridge → PX4, all `/fmu/out`).
The drone was disarmed and carried by hand at roughly flight height. Lap 1 ended back near the launch mark.

## Data
- Source: `~/team_ws/bags/capture-20261107-215520` on drone 18 (30 GB MCAP, stays on the drone).
- The drone lost power before the recording was closed. The file ends in zeros and had no index.
  `ros2 bag reindex ... -s mcap` recovered **256 s (21:55:22 → 21:59:38)**. Anything after
  21:59:38, including the final return to the launch mark, is lost.
- These CSVs were extracted on the drone with `../extract_walk.py` (images skipped):

| File | Topic | Columns |
|---|---|---|
| `vio.csv` | `/visual_slam/tracking/odometry` | t = arrival (Jetson clock, s), stamp = camera stamp, ENU position + quaternion |
| `vio_status.csv` | `/visual_slam/status` | vo_state (1 = tracking) |
| `lpos.csv` | `/fmu/out/vehicle_local_position` | NED x/y/z, velocities, heading, valid flags, dist_bottom (rangefinder), reset counters, dead_reckoning |
| `att.csv` | `/fmu/out/vehicle_attitude` (every 4th) | quaternion |
| `status.csv` | `/fmu/out/vehicle_status_v1` | arming_state (1 = disarmed the whole walk), nav_state |
| `ev_in.csv` | `/fmu/in/vehicle_visual_odometry` | what the bridge sent PX4 |
| `cam.csv` | `/oak/left/camera_info` | camera frame times |

All `t` columns are the same Jetson clock, so the files line up directly.

## Findings (`walk_215520.png`)
1. **VIO tracked well.**
   - vo_state = 1 the whole time, and no step larger than 0.15 m between frames.
   - After lap 1 (~45 m walked, t ≈ 120 s) VIO was **≈ 0.3 m from the start**, i.e. drift under 1 %, assuming the walkers stood on the start spot.
   - The laps overlay within ~0.3–0.7 m.
2. **PX4's position did not hold up.**
   - From ~60–145 s and after ~240 s, VIO's output reached PX4 **late**: median 174 ms, p95 343 ms, vs 69 ms normally and `EKF2_EV_DELAY` = 70 ms.
   - VIO computed every frame on time (camera-stamp gaps ≤ 0.2 s); only the delivery lagged.
   - Result: ~100 PX4 horizontal resets with **5–9 m jumps**, repeated dead reckoning, and PX4 heading **flipping ~180°** vs VIO at t ≈ 140 s and 240 s.
3. **Likely cause: the recording itself.** Writing both raw camera streams (~6 GB/min, ~100 MB/s) loads the Jetson. The clean autonomous flight (PX4 log 26) had no recording running.
4. **Height can't be judged from this walk.** PX4 height vs rangefinder: median −0.07 m, but p5/p95 −0.95/+0.79 m during the disturbed periods.

## Follow-ups
- Don't record camera images during flights (`bg.sh hover/square` currently does).
- Confirm item 3: repeat the walk recording only VIO + PX4 topics (no images). The VIO delivery delay should stay ~70 ms and PX4 should not reset.
- Depth from VIO's images on the Jetson (option A in `DEPTH_OPTIONS`) must be kept small and slow, with this delay watched.
