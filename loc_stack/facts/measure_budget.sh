#!/bin/bash
# Run ON the Jetson host. Read-only. Output: ~/facts_out (default). ~75 s.
OUT=${OUT:-$HOME/facts_out}; C=skybeta-stack; SECS=${SECS:-30}
rm -rf "$OUT"; mkdir -p "$OUT"
cdo() { docker exec "$C" bash -lc "source /opt/ros/*/setup.bash; source /usr/local/bin/skybeta_env.sh; $*"; }

# --- static facts
{ echo "## cpus"; nproc; echo "## nvpmodel"; (nvpmodel -q 2>&1 || sudo -n nvpmodel -q 2>&1) | head -4
  echo "## cpu freq MHz"; for c in 0 1 2 3 4 5; do echo -n "$(( $(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_cur_freq 2>/dev/null || echo 0) / 1000 )) "; done; echo
  echo "## usb speed"; lsusb -t 2>/dev/null | grep -iE "03e7|Myriad|Class=Vendor|5000M|10000M" | head -5
} > "$OUT/static.txt" 2>&1
{ echo "## camera sockets seen by the driver (slam pane log)";
  docker exec "$C" tmux capture-pane -p -t skybeta:slam -S -3000 | grep -iE "cam_[a-d]|imx|ov9|sensor|socket|imu|BNO|BMI|stereo|rgb|color" | head -25
  echo "## rgb/color topics"; cdo "ros2 topic list | grep -iE 'rgb|color|depth|disparity|points'" ; echo "(end of list)"
  echo "## driver pipeline param"; cdo "timeout 8 ros2 param dump /oak 2>/dev/null | grep -iE 'pipeline_type|i_enable_rgb|rgb' | head"
} > "$OUT/camera_sockets.txt" 2>&1

# --- load test: everything running, 30 s
docker cp "$HOME/skybeta/tools/latency_probe.py" "$C:/tmp/latency_probe.py" 2>/dev/null
( cdo "python3 /tmp/latency_probe.py --seconds $SECS" > "$OUT/latency.txt" 2>&1 ) &
( timeout "$SECS" tegrastats --interval 500 > "$OUT/tegrastats.txt" 2>&1 ) &
( for i in $(seq 1 $((SECS/3))); do date -Is; docker stats --no-stream --format '{{.Name}} cpu={{.CPUPerc}} mem={{.MemUsage}}'; sleep 1; done > "$OUT/docker_stats.txt" 2>&1 ) &
( top -b -d 3 -n $((SECS/3)) -w 200 | grep -E "visual_slam|component|oak|depthai|python|ros|isaac|MicroXRCE|Cpu" > "$OUT/top.txt" 2>&1 ) &
( for t in /oak/left/image_raw /oak/right/image_raw /oak/imu/data /visual_slam/tracking/odometry /visual_slam/vis/slam_odometry /visual_slam/vis/landmarks_cloud; do
    echo "## $t"; cdo "timeout $((SECS/3)) ros2 topic hz $t 2>&1 | grep -E 'average|min' | tail -2"; done > "$OUT/rates.txt" 2>&1 ) &
wait

# --- cuVSLAM warnings since the current launch (tracking loss etc.)
docker exec "$C" tmux capture-pane -p -t skybeta:slam -S -20000 > "$OUT/slam_pane.txt" 2>&1
{ echo "lines: $(wc -l < "$OUT/slam_pane.txt")"; echo "warn/lost/jitter counts:";
  for k in "lost" "jitter" "WARN" "ERROR" "reinit" "failed"; do echo "  $k: $(grep -ci "$k" "$OUT/slam_pane.txt")"; done
  echo "last 6 WARN/ERROR lines:"; grep -iE "WARN|ERROR|lost" "$OUT/slam_pane.txt" | tail -6 | cut -c1-200; } > "$OUT/slam_warnings.txt"

# --- compute benchmarks (sequential, nothing else running from us)
cat > /tmp/bench.py <<'PY'
import time, cv2, numpy as np, sys
def t(f, n=1):
    s = time.perf_counter(); [f() for _ in range(n)]; return (time.perf_counter() - s) / n * 1000
def loop():
    a = 0
    for i in range(1500000): a += i * i % 7
print('pure python loop (1.5M iters): %.0f ms   python %s' % (t(loop), sys.version.split()[0]))
import os
L = cv2.imread('/tmp/rect_L.png', 0); R = cv2.imread('/tmp/rect_R.png', 0)
if L is None: print('no stereo pair in container'); sys.exit()
def sg(nd, bs, mode):
    return cv2.StereoSGBM_create(0, nd, bs, 8*bs*bs, 32*bs*bs, uniquenessRatio=10, mode=mode)
for name, a, b in (('1024x640', L, R), ('512x320', cv2.resize(L, None, fx=.5, fy=.5), cv2.resize(R, None, fx=.5, fy=.5))):
    nd = 128 if name[0] == '1' else 64
    print('%s SGBM  (%d disp, 3way): %.0f ms/frame' % (name, nd, t(lambda: sg(nd, 7, cv2.STEREO_SGBM_MODE_SGBM_3WAY).compute(a, b), 5)))
    print('%s SGBM  (%d disp, fast): %.0f ms/frame' % (name, nd, t(lambda: sg(nd, 7, cv2.STEREO_SGBM_MODE_HH4).compute(a, b), 5)))
    print('%s BM    (%d disp):       %.0f ms/frame' % (name, nd, t(lambda: cv2.StereoBM_create(nd, 15).compute(a, b), 5)))
PY
docker cp /tmp/bench.py "$C:/tmp/bench.py"; cdo "python3 /tmp/bench.py" > "$OUT/bench.txt" 2>&1
echo DONE > "$OUT/DONE"
