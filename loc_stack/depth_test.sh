#!/bin/bash
# Laptop-side helper for the on-device depth (plain wall) test. Props off, FC disarmed.
#
#   ./depth_test.sh prepare                    copy + validate the depth files. Changes NOTHING running
#   ./depth_test.sh wall [T1 T2 ...]           guided session: on, blank + textured at each distance, off
#                                              (default distances 0.5 1.0 2.0 3.0 m)
#   ./depth_test.sh on                         save the running slam pipeline, switch it to depth
#   ./depth_test.sh measure LABEL TRUTH_M [S]  record S s (default 15) of depth at a wall
#   ./depth_test.sh off                        restore the saved pipeline, check rates by stamp
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
# Save what is running NOW, so 'off' puts back exactly that.
cmd=\$(sed -e 's/^"//' -e 's/"\$//' <<<"\$cur")
[ -n "\$cmd" ] || cmd='run_slam.sh'
printf '#!/bin/bash\n# saved by depth_test.sh on %s\nexec %s\n' "\$(date '+%F %T')" "\$cmd" > $HD/restore_slam.sh
bash -n $HD/restore_slam.sh
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
  r "rm -rf ~/team_ws/walk_runs/depth/$label; docker exec -d skybeta-stack bash -lc '$ENVSRC; python3 /root/team_ws/loc_stack/depth_wall.py --label $label $tflag --seconds $secs --out $out'" || return 1
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
  off)     off ;;
  *) sed -n 2,10p "$0" ;;
esac
