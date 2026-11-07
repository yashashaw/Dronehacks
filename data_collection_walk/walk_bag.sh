#!/usr/bin/env bash
# Inside the skybeta-stack container. record_walk.sh copies this in and runs it;
# it is not meant to be run by hand.
#
#   walk_bag.sh topics                     print the live topic list
#   walk_bag.sh info <bag_dir>             ros2 bag info
#   walk_bag.sh record <bag_dir> [mono]    record until SIGINT
#
# The topic set is skybeta's record_bag.sh set (both eyes + calibration, IMU,
# cuVSLAM odometry and status, what the bridge sends PX4, every /fmu/out topic
# present at start), plus /skybeta/state and TF. "mono" records the mono8
# copies of the eyes instead of bgr8: the sensors are monochrome, so it is the
# same picture at a third of the size.
set -uo pipefail

# ROS + skybeta underlay + team overlay. skybeta_env.sh handles nounset itself.
# shellcheck disable=SC1091
source skybeta_env.sh

case "${1:-}" in
  topics)
    exec ros2 topic list
    ;;
  info)
    exec ros2 bag info "$2"
    ;;
  record)
    BAG="$2"
    EYES="${3:-raw}"
    if [[ "${EYES}" == "mono" ]]; then
      IMAGES=(/oak/left/image_mono /oak/right/image_mono)
    else
      IMAGES=(/oak/left/image_raw /oak/right/image_raw)
    fi
    TOPICS=(
      "${IMAGES[@]}"
      /oak/left/camera_info
      /oak/right/camera_info
      /oak/imu/data
      /visual_slam/tracking/odometry
      /visual_slam/status
      /fmu/in/vehicle_visual_odometry
      /skybeta/state
      /tf
      /tf_static
    )
    while IFS= read -r t; do
      [[ -n "${t}" ]] && TOPICS+=("${t}")
    done < <(ros2 topic list 2>/dev/null | grep '^/fmu/out/' || true)

    mkdir -p "$(dirname "${BAG}")"
    echo "[walk] recording ${#TOPICS[@]} topics into ${BAG}:"
    printf '        %s\n' "${TOPICS[@]}"
    # SIGINT is rosbag2's clean shutdown; record_walk.sh sends it to this process.
    exec ros2 bag record -s mcap -o "${BAG}" "${TOPICS[@]}"
    ;;
  *)
    echo "walk_bag.sh: expected topics | info <bag> | record <bag> [mono]" >&2
    exit 2
    ;;
esac
