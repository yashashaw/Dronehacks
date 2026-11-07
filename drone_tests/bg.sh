#!/bin/bash
# Run a PROPS-OFF spin test in the background on the drone, so a dropped ssh does not cut it off.
#
#   ~/team_ws/drone_tests/bg.sh motors        depth -> motor speed (closer = faster), ~10 s
#   ~/team_ws/drone_tests/bg.sh pulse         15 % for 4 s, then off
#   ~/team_ws/drone_tests/bg.sh log           show the output (run again until it says DONE)
#
#   ~/team_ws/drone_tests/bg.sh hover         PROPS ON flight: load hover.py (0.7 m, 5 s, land).
#                                             Nothing flies until the pilot arms + flips to Position.
#   ~/team_ws/drone_tests/bg.sh hoverlog      what the hover node is saying
#   ~/team_ws/drone_tests/bg.sh restore       put the normal competitor launch back
#
# Extra options pass through to depth_motors.py, e.g.  bg.sh motors --farther-faster
# It checks the drone is disarmed, stops the competitor window (flight_interface), runs the test,
# then restarts the competitor window. Props off, drone held down, RC kill switch ready.
LOG=/tmp/drone_test.log
C=skybeta-stack
E="source skybeta_env.sh >/dev/null 2>&1"

case "${1:-}" in
  hover)   # load the hover flight into the competitor window; the PILOT starts it (arm, then Position)
    TS=$(date +%Y%m%d-%H%M%S)
    mkdir -p ~/team_ws/bags ~/team_ws/drone_tests/runs
    # bag: cameras + IMU + VIO + bridge + every /fmu/out topic, max 10 min (~25 GB), kept in team_ws
    docker exec -d $C bash -lc "source skybeta_env.sh; record_bag.sh --duration 600 -o /root/team_ws/bags/hover-$TS > /root/team_ws/drone_tests/runs/hover-$TS.bag.log 2>&1"
    docker exec $C tmux respawn-window -k -t skybeta:competitor \
      "bash -lc 'source skybeta_env.sh; ros2 launch /root/team_ws/drone_tests/hover.launch.py 2>&1 | tee /root/team_ws/drone_tests/runs/hover-$TS.log; exec bash'"
    sleep 8
    docker exec $C tmux capture-pane -p -J -t skybeta:competitor | grep -v '^$' | tail -4
    echo "hover loaded; recording ~/team_ws/bags/hover-$TS and runs/hover-$TS.log"
    echo "Pilot: arm on Altitude, then Position. Watch: $0 hoverlog   After landing: $0 restore"
    exit 0 ;;
  hoverlog) docker exec $C tmux capture-pane -p -J -t skybeta:competitor -S -200 | grep -v '^$' | grep -E 'hover_test|python3|LAND|ERROR|WARN' | tail -25; exit 0 ;;
  restore)   # stop the bag (flushes it), put the normal launch back, sync to disk
    docker exec $C pkill -INT -f "ros2 bag record" && sleep 5 && echo "bag stopped"
    docker exec $C tmux respawn-window -k -t skybeta:competitor "bash -lc 'source skybeta_env.sh; run_competitor.sh; exec bash'"
    docker exec $C chown -R "$(id -u):$(id -g)" /root/team_ws/bags /root/team_ws/drone_tests/runs 2>/dev/null
    sync
    echo "competitor window back to COMPETITOR_PKG from drone.env; files synced to disk"; exit 0 ;;
  log) tr '\r' '\n' < "$LOG" | grep -v '^$' | awk '!/centre/ || NR%5==0'; exit 0 ;;
  motors) shift; ARGS="--spin --props-off --seconds 15 $*" ;;
  pulse)  shift; ARGS="--pulse 0.15 --pulse-seconds 4 --spin --props-off $*" ;;
  run)    shift; ARGS="$*" ;;   # internal: the detached part
  *) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac

if [[ "${BG_CHILD:-}" != 1 ]]; then
  BG_CHILD=1 setsid nohup "$0" run $ARGS > "$LOG" 2>&1 < /dev/null &
  echo "started in the background. Watch it with:  $0 log"
  exit 0
fi

a=$(docker exec $C bash -c "$E; timeout 6 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_status_v1" | awk '/^arming_state:/{print $2}')
echo "arming_state=$a"
[ "$a" = 1 ] || { echo "NOT DISARMED (or no FC link) - not starting. DONE"; exit 1; }
docker exec $C tmux respawn-window -k -t skybeta:competitor "bash -c 'echo [dt] competitor stopped for a bench test; exec bash'"
echo "competitor stopped $(date +%T)"; sleep 2
docker exec $C bash -c "$E; python3 /root/team_ws/drone_tests/depth_motors.py $ARGS"
echo "python exit $?"
docker exec $C bash -c "$E; timeout 6 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_status_v1" | grep -E "^arming_state:"
docker exec $C tmux respawn-window -k -t skybeta:competitor "bash -lc 'source skybeta_env.sh; run_competitor.sh; exec bash'"
docker exec $C chown -R "$(id -u):$(id -g)" /root/team_ws/drone_tests/runs
echo "competitor restarted $(date +%T) DONE"
