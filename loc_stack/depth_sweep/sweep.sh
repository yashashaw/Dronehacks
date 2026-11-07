#!/bin/bash
# Depth/latency sweep. Run ON THE JETSON HOST, detached:
#   nohup ~/team_ws/loc_stack/depth_sweep/sweep.sh stock d_base d_q2 ... > ~/team_ws/loc_stack/depth_sweep/sweep.log 2>&1 &
# Swaps ONLY the stack's slam pane per variant (bench only; disarmed). Afterwards do a FULL
# stack restart (docker rm -f skybeta-stack) before any flight: slam-only restarts preceded
# start-up jumps. Results: depth_sweep/results/<variant>.{json,top,tegra,params}; DONE at the end.
set -u
H=$HOME/team_ws/loc_stack/depth_sweep
C=/root/team_ws/loc_stack/depth_sweep
WAIT=${WAIT:-45}; MEAS=${MEAS:-20}
ENVSRC='source /opt/ros/humble/setup.bash; source /usr/local/bin/skybeta_env.sh'
mkdir -p "$H/results"; rm -f "$H/DONE"
dx() { docker exec skybeta-stack bash -lc "$ENVSRC; $*"; }

# launch args of the stock slam launch (camera pose), captured once
if [ ! -s "$H/launch_args" ]; then
  docker exec skybeta-stack ps -eo args | grep -m1 'oakd_vslam.launch.py' \
    | sed 's/.*oakd_vslam\.launch\.py//' > "$H/launch_args"
fi
ARGS=$(cat "$H/launch_args"); echo "launch args:$ARGS"

for v in "$@"; do
  safe=${v//+/__}
  echo "=== $v  $(date +%T)"
  lp=$(dx "python3 $C/make_variant.py '$v' $C/gen") || { echo "make_variant failed"; continue; }
  # sleep infinity keeps the pane alive if the launch dies (a dead pane restarts the whole stack)
  docker exec skybeta-stack tmux respawn-pane -k -t skybeta:slam \
    "bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh; ros2 launch $lp $ARGS; echo LAUNCH EXITED; sleep infinity'"
  sleep "$WAIT"
  dx "timeout 20 ros2 param dump /oak" > "$H/results/$safe.params" 2>&1
  dx "timeout $((MEAS + 30)) python3 $C/measure.py $MEAS $C/results/$safe.json" > "$H/results/$safe.out" 2>&1 &
  mp=$!
  sleep 6
  top -b -n 2 -d 5 -w 220 | awk '/^top -/{i++} i==2' | head -22 > "$H/results/$safe.top"
  timeout 6 tegrastats --interval 1000 > "$H/results/$safe.tegra" 2>&1
  wait $mp
  docker exec skybeta-stack tmux capture-pane -p -t skybeta:slam -S -60 > "$H/results/$safe.pane" 2>&1
  tail -1 "$H/results/$safe.out"
done
date +%T > "$H/DONE"
