#!/bin/bash
# Gather the no-hand-test facts about the drone. Run ON the Jetson host (not in the
# container); it is read-only and sends no commands. Output: $OUT (default ~/gather_out).
#
#   nohup bash gather_hardware_facts.sh > /tmp/gather.log 2>&1 &
#
# Covers: camera mount + TF, CameraInfo/FOV/variant, PX4 motion limits (from the param
# files in the repo, not the live FC), vision-pipeline latency, Jetson compute load.
SECS=${SECS:-30}
OUT=${OUT:-$HOME/gather_out}
C=skybeta-stack
REPO=${REPO:-$HOME/skybeta}
mkdir -p "$OUT"
cdo() { docker exec "$C" bash -lc "source /opt/ros/*/setup.bash; [ -f skybeta_env.sh ] && source skybeta_env.sh; $*"; }

echo "start $(date -Is)" > "$OUT/meta.txt"
uname -a >> "$OUT/meta.txt"; cat /etc/nv_tegra_release >> "$OUT/meta.txt" 2>/dev/null
docker ps --format '{{.Names}} {{.Image}} {{.Status}}' >> "$OUT/meta.txt"
cat /etc/skybeta/drone.env > "$OUT/drone_env.txt" 2>/dev/null || cp "$REPO/host/drone.env.example" "$OUT/drone_env.txt"

# 2. camera mount: the declared static TF, the env it came from, and the USB device
{ echo "## TF base_link -> oak_parent_frame (declared mount)";
  cdo "timeout 8 ros2 run tf2_ros tf2_echo base_link oak_parent_frame" 2>&1 | head -12
  echo "## base_link -> oak_left_camera_optical_frame";
  cdo "timeout 8 ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame" 2>&1 | head -12
  echo "## base_link -> oak_imu_frame";
  cdo "timeout 8 ros2 run tf2_ros tf2_echo base_link oak_imu_frame" 2>&1 | head -12
  echo "## CAMERA_* in the running stack";
  docker inspect "$C" --format '{{range .Config.Env}}{{println .}}{{end}}' | grep -E 'CAMERA_|DRONE_ID'
  echo "## usb"; lsusb | grep -iE '03e7|luxonis|movidius'
} > "$OUT/camera_mount.txt" 2>&1

# 3. CameraInfo (-> FOV), driver config, sensor
{ for s in left right; do echo "## /oak/$s/camera_info"; cdo "timeout 10 ros2 topic echo --once /oak/$s/camera_info" 2>&1; done
  echo "## driver params"; cdo "timeout 10 ros2 param dump /oak" 2>&1 | head -150
} > "$OUT/camera_info.txt" 2>&1

# 4. PX4 limits as configured in the repo's param files
{ echo "# from $REPO/px4/params/*.params (the files loaded onto the FC, not read live)";
  grep -hE '^(MPC_(ACC_[A-Z_]+|DEC_[A-Z_]+|JERK_[A-Z]+|XY_VEL_MAX|XY_CRUISE|Z_VEL_MAX_[A-Z]+|TILTMAX_[A-Z]+|YAWRAUTO_[A-Z]+|MAN_Y_MAX|VEL_MANUAL)|MC_(YAWRATE_MAX|ROLLRATE_MAX|PITCHRATE_MAX)|EKF2_(EV_[A-Z_]+|HGT_REF)|COM_[A-Z_]*LOSS[A-Z_]*)[ ,=[:space:]]' "$REPO"/px4/params/*.params | sort -u
} > "$OUT/px4_limits.txt" 2>&1

# 5 + 6 run together for SECS seconds: vision latency probe, tegrastats, docker stats, top
docker cp "$REPO/tools/latency_probe.py" "$C:/tmp/latency_probe.py" 2>/dev/null
( cdo "python3 /tmp/latency_probe.py --seconds $SECS" > "$OUT/vision_latency.txt" 2>&1 ) &
( timeout "$SECS" tegrastats --interval 500 > "$OUT/tegrastats.txt" 2>&1 ) &
( for i in $(seq 1 $((SECS/3))); do date -Is; docker stats --no-stream --format '{{.Name}} cpu={{.CPUPerc}} mem={{.MemUsage}}' ; sleep 2; done > "$OUT/docker_stats.txt" 2>&1 ) &
( top -b -d 3 -n $((SECS/3)) -w 200 | grep -E '^top|Cpu|PID|visual|component|oak|python|ros|isaac' > "$OUT/top_processes.txt" 2>&1 ) &
# topic delivery rates over the same window (the pipeline's throughput)
( for t in /oak/left/image_raw /oak/right/image_raw /oak/imu/data /visual_slam/tracking/odometry; do
    echo "## $t"; cdo "timeout $((SECS/3)) ros2 topic hz $t" 2>&1 | tail -4; done > "$OUT/topic_rates.txt" 2>&1 ) &
wait
echo "done $(date -Is)" >> "$OUT/meta.txt"
echo DONE > "$OUT/DONE"
