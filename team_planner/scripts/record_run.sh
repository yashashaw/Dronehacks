#!/bin/bash
# Record one run for offline study (allowed for development; nothing recorded here may be fed back
# into a later run, rule 6.4). Run it in a SECOND container so it does not steal CPU from cuVSLAM:
#   host/run.sh shell      (attaches to the running stack), then:  bash record_run.sh <name>
# UNTESTED. Adjust the output path to a mounted folder (the docs use /bags).
name=${1:-run-$(date +%Y%m%d-%H%M%S)}
exec ros2 bag record -s mcap -o "/bags/$name" \
  /oak/stereo/image_raw /oak/stereo/camera_info /oak/left/image_raw /oak/imu/data \
  /visual_slam/tracking/odometry /visual_slam/status /loc/health \
  /course/goal /team_planner/debug /pad/detection \
  /skybeta/state /skybeta/setpoint/pose /fmu/in/trajectory_setpoint /fmu/out/vehicle_local_position
