#!/bin/bash
# Save the stereo camera (left + right grayscale, what VIO sees) to ~/team_ws/videos on the drone.
#
#   ~/team_ws/drone_tests/stereo_video.sh          record until Ctrl-C
#   ~/team_ws/drone_tests/stereo_video.sh 30       record 30 s
#
# Read-only: runs stereo_video.py inside the running stack container; never arms or commands.
set -euo pipefail
SECS="${1:-0}"
docker ps --format '{{.Names}}' | grep -qx skybeta-stack || { echo "stack not running (docker ps)"; exit 1; }
mkdir -p ~/team_ws/videos
docker exec -it skybeta-stack bash -c "source skybeta_env.sh >/dev/null 2>&1; \
  python3 /root/team_ws/drone_tests/stereo_video.py $SECS /root/team_ws/videos; \
  chown -R $(id -u):$(id -g) /root/team_ws/videos"
