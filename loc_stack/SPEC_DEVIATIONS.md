# Spec vs. what drone 7 actually runs

The "Localization, SLAM & Ground Height Spec" was written for the simulator image. Drone 7's
stack (`skybeta` 0.3.2, in `skybeta/`) already implements most of it, and several spec values
contradict measurements recorded in this repo. Per "discover, don't assume names", this
branch **adds** to the drone's stack and does **not** replace it. Nothing here restarts the
stack, publishes to `/fmu/*`, or changes a PX4 parameter.

| Spec item | Spec value | Drone 7 actually runs | Decision |
|---|---|---|---|
| Driver | depthai-ros 2.x `camera.launch.py`, `/oak/*/image_rect` | depthai **v3** driver, `/oak/{left,right}/image_raw`, no rect topics | Keep. |
| Resolution / fps | 400p @ 60 | 1024x640 @ 30 (USB bandwidth ceiling, measured in `oakd_vslam.yaml`) | Keep. 60 fps would saturate USB. |
| cuVSLAM `rectified_images` | true | false (driver publishes no rect images) | Keep. |
| cuVSLAM `enable_imu_fusion` | true | **false** (raw uncalibrated BNO086; pure stereo VO) | Keep. Spec itself says to turn it off if tracking gets worse. |
| cuVSLAM SLAM / loop closure | `enable_localization_n_mapping: true` | **false** (loop closure teleports the pose; EKF2 integrates the jump as motion) | Keep. |
| ROS distro | "Humble" | Humble, Python 3.10 (confirmed) | Matches. |
| PX4 bridge | new `vio_px4_bridge.py` | `px4_odom_bridge` already running, tested, with `reset_counter` from `vo_state` | **Not duplicated.** A second publisher on `/fmu/in/vehicle_visual_odometry` would fight it. |
| Bridge world conversion | FLU->FRD: (x,-y,-z) | ENU->NED permutation (y,x,-z) with `pose_frame=FRD` | Keep. Both are a constant yaw offset of the arbitrary-heading vision frame, which EKF2 estimates under `POSE_FRAME_FRD`. |
| `/loc/health` | new | **added**: `health_node.py`, subscribe-only | New. |
| Camera mount | at the IMU, `EKF2_EV_POS_*` = 0 | 0.1053, 0, 0.0304 m via the static TF; `EKF2_EV_POS_*` = 0 | Matches the spec's rule (offset in TF only). |
| `EKF2_BARO_CTRL` | **0** | **1** | **Rejected.** `px4/params/03a_height_reference.params` calls BARO_CTRL=1 "the essential fix" for a runaway climb when the rangefinder drops out. |
| `EKF2_HGT_REF` | 3 (vision) | 2 (range), baro as backup | Keep, with the same warning. Changing it is the organizers' call and needs a param file, not the DDS link. |
| `EKF2_EV_DELAY` | 40 ms to start | 70 ms in the param file; measured **58 ms p50** (`tests/hardware_facts/vision_latency.txt`) | Re-measure with `latency_probe.py` before changing. |
| `EKF2_MAG_TYPE` 5, `EKF2_RNG_CTRL` 2 | | not touched | Not applied. PX4 params cannot be set over DDS; a human must load a reviewed file. |
| `/fmu/out/vehicle_local_position` | `_v1` suffix possible | **no suffix** here (`vehicle_status_v1` has one) | Health node takes the name as a parameter. |

## What cannot be verified right now

At the time of writing the flight controller is **not linked to the Jetson**:
`/fmu/out/vehicle_local_position` has zero publishers, and the only subscriber on
`/fmu/in/vehicle_visual_odometry` is the container's own status node. So these spec tests are
untestable until the Pixhawk is powered and the uXRCE-DDS agent is connected:

V4 axis test, V5 vision delay (needs a PX4 log), V6 hover hold, V7 square (flight),
V8 boulder / `dist_bottom`, V9 blackout against `xy_valid`, V10 reset recovery.

`/loc/health` reports `fc_linked: False` and `xy_valid: unknown` in that state and judges
health from VIO alone.

Verified on the drone: V1 camera/IMU rates, odometry at 30 Hz, tracking state from
`/visual_slam/status`, `/loc/health` at 30.0 Hz, the logger, and the analysis pipeline.
