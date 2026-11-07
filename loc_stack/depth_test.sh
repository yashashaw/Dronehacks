#!/bin/bash
# Laptop-side helper for the on-device depth (plain wall) test. Props off, FC disarmed.
#
#   ./depth_test.sh prepare                    copy + validate the depth files. Changes NOTHING running
#   ./depth_test.sh wall [T1 T2 ...]           guided session: on, blank + textured at each distance, off
#                                              (default distances 0.5 1.0 2.0 3.0 m)
#   ./depth_test.sh on                         save the running slam pipeline, switch it to depth
#   ./depth_test.sh measure LABEL TRUTH_M [S]  record S s (default 15) of depth at a wall
#   ./depth_test.sh off                        restore the saved pipeline, check rates by stamp
#   ./depth_test.sh video [SECONDS] [NAME]     record an mp4 of depth beside the left image (depth on)
#   ./depth_test.sh loctest [SECONDS] [NAME]   2:00 default: depth video with live VIO map, PLUS the
#                                              full localization log (walk_logger), started together
#
# Only the stack's `slam` tmux pane is swapped; the agent keeps running, so the Pixhawk
# link survives. If the depth launch ever exits, the pane falls back to the saved pipeline
# by itself: a dead slam pane would make skybeta-stack.service restart the whole stack,
# which on stock PX4 v1.16.2 needs an FC power cycle.
#
# While depth is on, the camera link carries depth + both images, so VIO degrades. Do the
# drift tests with depth OFF.
DRONE=${DRONE:-dronehacks@192.168.55.1}
HERE=$(cd "$(dirname "$0")" && pwd)
D=/root/team_ws/loc_stack/depth            # = ~/team_ws/loc_stack/depth on the host
HD='~/team_ws/loc_stack/depth'
ENVSRC='source /opt/ros/*/setup.bash; source /usr/local/bin/skybeta_env.sh'

retry() {  # the USB link drops connections; retry only connection failures (rc 255)
  for i in 1 2 3 4 5 6; do
    "$@" && return 0
    rc=$?; [ $rc -ne 255 ] && return $rc; sleep 2
  done; return 255
}
r() { retry ssh -o BatchMode=yes -o ConnectTimeout=8 "$DRONE" "$@"; }
# Feed a script on stdin. Not retried blindly: a retry would get an empty stdin.
rs() { ssh -o BatchMode=yes -o ConnectTimeout=15 "$DRONE" "bash -s"; }

prepare() {
  # Only new files: depth_wall.py and the depth/ folder. The rest of ~/team_ws/loc_stack
  # (which teammates may have edited on the drone) is left alone.
  r "mkdir -p $HD" || return 1
  retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$HERE/depth_wall.py" \
    "$DRONE:team_ws/loc_stack/depth_wall.py" || return 1
  rs <<EOF || { echo "prepare FAILED"; return 1; }
set -e
docker exec -i skybeta-stack bash -l <<'IN'
set -e
source /opt/ros/*/setup.bash; source /usr/local/bin/skybeta_env.sh
python3 -c "import numpy, yaml, rclpy, sensor_msgs.msg, nav_msgs.msg; print('depth_wall imports ok')"
python3 -m py_compile /root/team_ws/loc_stack/depth_wall.py
share=\$(ros2 pkg prefix --share oakd_vslam_bringup)
sed 's/i_pipeline_type: stereo/i_pipeline_type: depth/' \$share/config/oakd_vslam.yaml > $D/depth.yaml
python3 - <<'PY'
import yaml
p = '$D/depth.yaml'
s = open(p).read()
block = ("    stereo:\n      i_publish_topic: true\n      i_depth_preset: DEFAULT\n"
         "      i_subpixel: false\n      i_extended_disparity: false\n      i_lr_check: true\n")
i = s.index('    pipeline_gen:')
s = s[:i] + block + s[i:]
open(p, 'w').write(s)
cfg = yaml.safe_load(s)
node = next(iter(cfg.values()))['ros__parameters']
assert node['pipeline_gen']['i_pipeline_type'] == 'depth', node['pipeline_gen']
assert node['stereo']['i_publish_topic'] is True
print('depth.yaml valid')
PY
sed "s#os.path.join(bringup_share, 'config', 'oakd_vslam.yaml')#'$D/depth.yaml'#" \$share/launch/oakd_vslam.launch.py > $D/depth.launch.py
grep -q "'$D/depth.yaml'" $D/depth.launch.py || { echo 'could not point the launch at depth.yaml'; exit 1; }
python3 -m py_compile $D/depth.launch.py
cat > $D/start_depth.sh <<'SH'
#!/bin/bash
export SKYBETA_UNDERLAY_ONLY=1
source skybeta_env.sh
ros2 launch /root/team_ws/loc_stack/depth/depth.launch.py camera_x:=\${CAMERA_X:-0.0} camera_y:=\${CAMERA_Y:-0.0} camera_z:=\${CAMERA_Z:-0.0}
echo "[depth_test] depth launch exited: restoring the saved slam pipeline"
exec bash /root/team_ws/loc_stack/depth/restore_slam.sh
SH
bash -n $D/start_depth.sh
echo 'depth launch files written and validated'
IN
echo "slam pane currently runs: \$(docker exec skybeta-stack tmux display -p -t skybeta:slam '#{pane_start_command}')"
EOF
  echo "prepared. Nothing running was changed."
}

on() {
  r "test -f $HD/start_depth.sh" || prepare || return 1
  rs <<EOF || { echo "on FAILED"; return 1; }
set -e
cur=\$(docker exec skybeta-stack tmux display -p -t skybeta:slam '#{pane_start_command}')
if grep -q 'start_depth.sh' <<<"\$cur"; then echo "depth is already on"; exit 0; fi
# Save what is running NOW, so 'off' puts back exactly that. If the pane is running a
# previous restore (an earlier 'off'), reuse that saved pipeline: saving
# "bash restore_slam.sh" would make the restore file exec itself forever.
if grep -q 'restore_slam' <<<"\$cur"; then
  [ -f $HD/restore_slam.used ] || { echo "pane runs a restore but no restore_slam.used to reuse"; exit 1; }
  cp $HD/restore_slam.used $HD/restore_slam.sh
else
  cmd=\$(sed -e 's/^"//' -e 's/"\$//' <<<"\$cur")
  [ -n "\$cmd" ] || cmd='run_slam.sh'
  printf '#!/bin/bash\n# saved by depth_test.sh on %s\nexec %s\n' "\$(date '+%F %T')" "\$cmd" > $HD/restore_slam.sh
fi
bash -n $HD/restore_slam.sh
grep -q 'restore_slam' $HD/restore_slam.sh && { echo "refusing: restore file points at itself"; exit 1; }
echo "saved pipeline: \$(tail -1 $HD/restore_slam.sh)"
docker exec skybeta-stack tmux respawn-pane -k -t skybeta:slam "bash $D/start_depth.sh"
echo 'slam pane now runs the depth pipeline'
EOF
  echo "waiting 25 s for the camera to start, then a 5 s check..."
  sleep 25
  measure _depth_on_check - 5
}

off() {
  r "test -f $HD/restore_slam.sh" || { echo "no saved pipeline: depth was not switched on by this script"; return 1; }
  r "docker exec skybeta-stack tmux respawn-pane -k -t skybeta:slam 'bash $D/restore_slam.sh' && mv $HD/restore_slam.sh $HD/restore_slam.used" || return 1
  echo "restored. Waiting 25 s, then checking the image rates by header stamp..."
  sleep 25
  measure _restore_check - 10
  echo "expect left/right ~30 Hz, vio_odom ~30 Hz, depth n=0 (depth is off)"
}

measure() {  # LABEL TRUTH SECONDS
  local label=$1 truth=$2 secs=${3:-15} tflag=""
  [ -n "$truth" ] && [ "$truth" != "-" ] && tflag="--truth $truth"
  local out=/root/team_ws/walk_runs/depth/$label
  # The folder is root-owned (made by the container): delete old results from INSIDE it,
  # and refuse to go on if they survive, or a stale DONE would be read as this run.
  r "docker exec skybeta-stack rm -rf $out && ! test -e ~/team_ws/walk_runs/depth/$label" \
    || { echo "could not clear old results for '$label'"; return 1; }
  rm -rf "$HERE/runs/depth/$label"
  r "docker exec -d skybeta-stack bash -lc '$ENVSRC; python3 /root/team_ws/loc_stack/depth_wall.py --label $label $tflag --seconds $secs --out $out'" || return 1
  echo "recording '$label' for $secs s: keep the drone still"
  for i in $(seq 1 $((secs + 40))); do
    sleep 1
    r "test -f ~/team_ws/walk_runs/depth/$label/DONE" 2>/dev/null && break
  done
  mkdir -p "$HERE/runs/depth/$label"
  for f in result.json sample_depth.npy; do
    retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$DRONE:team_ws/walk_runs/depth/$label/$f" "$HERE/runs/depth/$label/" 2>/dev/null
  done
  [ -f "$HERE/runs/depth/$label/result.json" ] || { echo "no result came back"; return 1; }
  python - "$HERE/runs/depth/$label/result.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
rt = d.get('rates_by_header_stamp', {})
print('  rates (Hz, by stamp): ' + '  '.join(f"{k} {v.get('rate_hz')}" for k, v in rt.items()))
keys = ['valid_pct_full_median', 'valid_pct_roi_median', 'plane_distance_m', 'error_pct',
        'plane_rms_mm', 'plane_distance_frame_sd_mm', 'plane_tilt_deg', 'plane_note', 'error']
print('  ' + '  '.join(f'{k}={d[k]}' for k in keys if k in d))
if d.get('plane_tilt_deg') and d['plane_tilt_deg'] > 5:
    print('  WARNING: tilt over 5 deg, square the drone to the wall and repeat')
PY
}

video() {  # SECONDS NAME
  local secs=${1:-30} name=${2:-video_$(date +%H%M%S)}
  local out=/root/team_ws/walk_runs/depth_video/$name
  retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$HERE/depth_video.py" \
    "$DRONE:team_ws/loc_stack/depth_video.py" || return 1
  # walk_runs is root-owned (the container made it): clear and create the folder from inside.
  r "docker exec skybeta-stack rm -rf $out && ! test -e ~/team_ws/walk_runs/depth_video/$name"     || { echo "could not clear old results for '$name'"; return 1; }
  rm -rf "$HERE/runs/depth_video/$name"
  r "docker exec -d skybeta-stack bash -lc 'mkdir -p $out; $ENVSRC; python3 /root/team_ws/loc_stack/depth_video.py --seconds $secs --out $out > $out/run.log 2>&1'" || return 1
  echo "recording depth video '$name' for $secs s..."
  for i in $(seq 1 $((secs + 60))); do
    sleep 1
    r "test -f ~/team_ws/walk_runs/depth_video/$name/DONE" 2>/dev/null && break
  done
  mkdir -p "$HERE/runs/depth_video/$name"
  r "tail -5 ~/team_ws/walk_runs/depth_video/$name/run.log 2>/dev/null"
  for f in depth.mp4 depth.avi frames.csv first_frame.png run.log DONE; do
    retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$DRONE:team_ws/walk_runs/depth_video/$name/$f" "$HERE/runs/depth_video/$name/" 2>/dev/null
  done
  ls -la "$HERE/runs/depth_video/$name"
}

loctest() {  # SECONDS NAME: depth video + localization log, started together
  local secs=${1:-120} name=${2:-loctest_$(date +%H%M%S)}
  local out=/root/team_ws/walk_runs/depth_video/$name
  local here_out="$HERE/runs/depth_video/$name"
  # Depth must be on (the video needs /oak/stereo/image_raw). Exit 255 = SSH failed:
  # that says nothing about depth, so stop instead of guessing.
  r "docker exec skybeta-stack tmux display -p -t skybeta:slam '#{pane_start_command}' | grep -q start_depth.sh"
  case $? in
    0) ;;
    255) echo "drone unreachable over SSH (USB-C plugged in? ping 192.168.55.1)"; return 1 ;;
    *) echo "depth is off: switching it on first"; on || return 1 ;;
  esac
  retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$HERE/depth_video.py" \
    "$DRONE:team_ws/loc_stack/depth_video.py" || return 1
  r "test -f ~/team_ws/loc_stack/walk_logger.py" || retry scp -q -o BatchMode=yes \
    -o ConnectTimeout=8 "$HERE/walk_logger.py" "$DRONE:team_ws/loc_stack/walk_logger.py" || return 1
  r "docker exec skybeta-stack rm -rf $out && ! test -e ~/team_ws/walk_runs/depth_video/$name"     || { echo "could not clear old results for '$name'"; return 1; }
  rm -rf "$here_out"
  # Both start in ONE command, so they cover the same window.
  r "docker exec skybeta-stack tmux display -p -t skybeta:slam '#{pane_start_command}' > /tmp/slam_cfg_$name.txt; \
     docker exec -d skybeta-stack bash -lc 'mkdir -p $out/loc; cp /dev/null $out/loc/config.txt; $ENVSRC; \
       python3 /root/team_ws/loc_stack/walk_logger.py --seconds $secs --out $out/loc > $out/loc/run.log 2>&1 & \
       python3 /root/team_ws/loc_stack/depth_video.py --seconds $secs --out $out > $out/run.log 2>&1; wait'" || return 1
  echo "RECORDING '$name' for $secs s: depth video + localization. Started $(date +%H:%M:%S)."
  echo "Hold still ~3 s at the start and at the end (and at any mark you want measured)."
  for i in $(seq 1 $((secs + 90))); do
    sleep 1
    (( i % 30 == 0 && i <= secs )) && echo "  ${i}s / ${secs}s"
    r "test -f ~/team_ws/walk_runs/depth_video/$name/DONE && test -f ~/team_ws/walk_runs/depth_video/$name/loc/DONE" 2>/dev/null && break
  done
  mkdir -p "$here_out/loc"
  for f in depth.mp4 depth.avi frames.csv first_frame.png last_frame.png run.log DONE; do
    retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$DRONE:team_ws/walk_runs/depth_video/$name/$f" "$here_out/" 2>/dev/null
  done
  for f in odom.csv status.csv lpos.csv ekf_flags.csv run.log DONE; do
    retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$DRONE:team_ws/walk_runs/depth_video/$name/loc/$f" "$here_out/loc/" 2>/dev/null
  done
  retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$DRONE:/tmp/slam_cfg_$name.txt" "$here_out/loc/config.txt" 2>/dev/null
  echo "== video";        cat "$here_out/DONE" 2>/dev/null || echo "video: no DONE (see $here_out/run.log)"
  echo "== localization"; cat "$here_out/loc/DONE" 2>/dev/null || echo "logger: no DONE"
  if [ -f "$here_out/loc/odom.csv" ]; then
    python "$HERE/analyze_walk.py" "$here_out/loc" > /dev/null && \
      python - "$here_out/loc/analysis.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
for k in ('duration_s', 'odom_rate_hz', 'max_odom_gap_ms', 'tracking_lost_events', 'path_length_m',
          'height_span_m', 'start_to_end_error_m', 'start_to_end_error_pct_of_path'):
    print(f'  {k}: {d.get(k)}')
print(f"  holds found: {len(d.get('dwells', []))}")
PY
    python "$HERE/plot_walk.py" "$here_out/loc" > /dev/null 2>&1 && echo "  3D track: $here_out/loc/map.html"
  fi
  echo "files: $here_out"
}

wall() {  # guided session
  local dists=("$@"); [ ${#dists[@]} -gt 0 ] || dists=(0.5 1.0 2.0 3.0)
  trap 'echo; echo "interrupted: restoring the slam pipeline"; off; exit 1' INT
  on || { trap - INT; return 1; }
  for t in "${dists[@]}"; do
    read -r -p ">> BLANK wall, camera glass ${t} m from it, square. Enter to record (s = skip): " a
    [ "$a" = s ] || measure "blank_${t}" "$t" 15
    read -r -p ">> TEXTURED target centred, same ${t} m. Enter to record (s = skip): " a
    [ "$a" = s ] || measure "tex_${t}" "$t" 15
  done
  trap - INT
  off
  echo "results: $HERE/runs/depth/"
}

case "$1" in
  prepare) prepare ;;
  wall)    shift; wall "$@" ;;
  on)      on ;;
  measure) measure "${2:?label}" "${3:?truth in metres, or - for none}" "${4:-15}" ;;
  video)   video "${2:-30}" "$3" ;;
  loctest) loctest "${2:-120}" "$3" ;;
  off)     off ;;
  *) sed -n 2,13p "$0" ;;
esac
