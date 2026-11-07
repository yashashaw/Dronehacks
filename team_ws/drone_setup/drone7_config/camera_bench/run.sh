#!/bin/bash
# Camera-only bench (no cuVSLAM): start the depthai driver with one config in a throwaway
# container and measure it with cam_bench.py. STOP THE STACK FIRST (it holds the camera):
#   sudo systemctl stop skybeta-stack.service      ... and start it again afterwards.
# usage (on the drone, from ~/cam_bench):  ./run.sh CONFIG.yaml LABEL [SECONDS]
# Isolated: its own ROS_DOMAIN_ID (77) and localhost only, so it cannot reach the FC or other drones.
CFG=$1; LABEL=$2; SECS=${3:-40}
docker run --rm --privileged --network host -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb \
  -e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1 -v $HOME/cam_bench:/bench --name cam_bench \
  --entrypoint bash skybeta:$(cat ~/skybeta/VERSION) -c "
  source /opt/ros/humble/setup.bash
  ros2 launch depthai_ros_driver_v3 driver.launch.py params_file:=/bench/$CFG > /bench/$LABEL.driver.log 2>&1 &
  sleep 14
  ${EXTRA:-true}
  python3 /bench/cam_bench.py $SECS $LABEL
  kill -INT %1; sleep 4"
