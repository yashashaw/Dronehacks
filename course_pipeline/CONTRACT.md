# course_pipeline contract

The integration sheet for this package. Everything another teammate has to match is here.

## Files and who replaces them

| File | Role | Replaced by |
|---|---|---|
| `course_pipeline/pad_detector.py` | ROS node: camera image in, `/pad/detection` out | stays |
| `course_pipeline/yolo_pad.py` | **Model slot.** `PadModel.detect(gray)` returns boxes | the trained YOLO weights (body of this file only) |
| `course_pipeline/pad_math.py` | Ray-to-ground, confirmation counter, short steps, freshness. No ROS | stays |
| `course_pipeline/path_placeholder.py` | **Temporary planner.** Publishes `/course/goal` | the real pathfinder (a different node, same topic) |
| `course_pipeline/pad_guide.py` | ROS node. **The only publisher of `/skybeta/setpoint/pose`** | stays |
| `course_pipeline/guide_machine.py` | The guide's stage logic. No ROS, runs under pytest | stays |
| `launch/course.launch.py` | Starts `flight_interface` and the three nodes | edit one entry, see below |

## Topics

| Topic | Type | From | To | Notes |
|---|---|---|---|---|
| `/oak/left/image_mono` | `sensor_msgs/Image` mono8, 1024x640, ~30 Hz | camera stack | `pad_detector` | grayscale; the color camera is not running |
| `/oak/left/camera_info` | `sensor_msgs/CameraInfo` | camera stack | `pad_detector` | K used for unprojection, distortion assumed zero |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | localization | `pad_guide`, `path_placeholder` | ENU in `odom`, origin = startup pose |
| `/skybeta/state` | `std_msgs/String` JSON | `flight_interface` | `pad_guide`, `path_placeholder` | `offboard_active`, `armed`, `landed` |
| `/course/goal` | `geometry_msgs/PoseStamped` | **planner** | `pad_guide` | see below |
| `/pad/detection` | `std_msgs/String` JSON | `pad_detector` | `pad_guide` | see below |
| `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | `pad_guide` only | `flight_interface` | ENU in `odom`, all-zero quaternion = no yaw |

No node here publishes `/fmu/in/*` or `/skybeta/setpoint/twist`.

### `/course/goal` (what the real pathfinder must publish)

- `PoseStamped`, position in `odom` (ENU, metres). Orientation is ignored.
- Publish it **continuously while the drone flies**, whenever the chosen point changes and at least every 0.5 s even when it does not (the placeholder uses 5 Hz). The guide treats a goal it has not received for 0.5 s as stale and holds.
- Staleness is measured from when the guide **received** the message, not from `header.stamp`.
- The goal can be far away. The guide never forwards it: each tick it publishes a pose at most `step_m` (default 0.5 m) from the vehicle toward the newest goal.
- Goal height is the planner's. The flight interface still clamps altitude to 0.3-3.0 m above the odometry origin.
- Once a pad is confirmed the guide ignores the topic. The planner can keep publishing.

### `/pad/detection` (JSON, one message per camera frame)

```json
{"stamp": 1234.5, "detected": true, "confidence": 0.81,
 "pixel": [512.0, 400.0], "position_enu": [3.1, 0.4, 0.0],
 "confirmed": true, "streak": 5, "tf_exact": true}
```

- `position_enu` is the box center intersected with the ground plane (`ground_z`, default 0.0 in `odom`), or `null` when there is no detection or no valid ground intersection.
- `confirmed` is true after `confirm_frames` consecutive detections above `min_confidence` whose ground points agree within `confirm_radius_m`. One frame is never a match.
- `tf_exact` is false when the TF at the image stamp was missing and the latest transform was used instead.

## Replacing the placeholder planner

1. Run the real planner so it publishes `/course/goal` as above.
2. Launch with `planner:=false`, or delete the `path_placeholder` entry in `launch/course.launch.py` (marked `# PLACEHOLDER`).
3. Nothing in `pad_guide.py` or `guide_machine.py` changes.

## Behavior to know before integrating

- **Stages:** HOLD (before Offboard), TAKEOFF (climb to `cruise_altitude_m` above the start pose), FOLLOW (newest fresh goal), APPROACH (over the pad), LAND, DONE.
- **Pad detections are ignored during TAKEOFF**, and in FOLLOW anything within `home_exclusion_radius_m` (1.0 m) of the saved home position is ignored, so the home pad is not mistaken for the landing pad.
- **There is no land or disarm command.** LAND lowers the pose to the flight interface's 0.3 m floor over the pad and holds. The pilot lands. The guide goes to DONE when `/skybeta/state` shows `landed` true and `armed` false after Offboard has run.
- **Return to base is not implemented.** The saved home pose is kept on the machine for it. A second arm after DONE hovers at the floor.
- **Losing Offboard** (pilot takeover) stops the stage and keeps publishing the last target. Offboard does not return until the pilot disarms and arms again.
- Setpoints are published from startup, including while disarmed, because the flight interface will not enter Offboard without a setpoint in the last second.
