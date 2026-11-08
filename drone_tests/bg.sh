#!/bin/bash
# Run a PROPS-OFF spin test in the background on the drone, so a dropped ssh does not cut it off.
#
#   ~/team_ws/drone_tests/bg.sh motors        depth -> motor speed (closer = faster), ~10 s
#   ~/team_ws/drone_tests/bg.sh pulse         15 % for 4 s, then off
#   ~/team_ws/drone_tests/bg.sh log           show the output (run again until it says DONE)
#
#   ~/team_ws/drone_tests/bg.sh capture [S]   props off: record S s (default 60) of grayscale stereo
#                                             video + IMU + VIO + PX4 to ~/team_ws/bags. Never arms.
#   ~/team_ws/drone_tests/bg.sh stopcapture   stop a capture early
#
#   ~/team_ws/drone_tests/bg.sh hover        PROPS ON flight: load hover.py (0.7 m, 5 s, land).
#                                             Nothing flies until the pilot arms + flips to Position.
#   ~/team_ws/drone_tests/bg.sh square [S]    same, then a S x S m square (default 0.5, max 1.0), land
#   ~/team_ws/drone_tests/bg.sh hoverlog      what the hover/square node is saying
#   ~/team_ws/drone_tests/bg.sh restore       after landing: stop recording, unload (pilot only), sync
#   ~/team_ws/drone_tests/bg.sh normal        put the boot-default competitor launch back
#
# Extra options pass through to depth_motors.py, e.g.  bg.sh motors --farther-faster
# It checks the drone is disarmed, stops the competitor window (flight_interface), runs the test,
# then restarts the competitor window. Props off, drone held down, RC kill switch ready.
LOG=/tmp/drone_test.log
C=skybeta-stack
E="source skybeta_env.sh >/dev/null 2>&1"

case "${1:-}" in
  capture)   # props-off data capture: grayscale stereo video + IMU + VIO + PX4. Never arms.
    SECS="${2:-60}"; TS=$(date +%Y%m%d-%H%M%S); OUT=/root/team_ws/bags/capture-$TS
    mkdir -p ~/team_ws/bags
    docker exec -d $C bash -lc "source skybeta_env.sh; record_bag.sh --duration $SECS -o $OUT > $OUT.log 2>&1; chown -R $(id -u):$(id -g) $OUT $OUT.log; sync"
    echo "recording $SECS s -> ~/team_ws/bags/capture-$TS  (~2.4 GB/min). Stop early: $0 stopcapture"
    exit 0 ;;
  stopcapture)
    docker exec $C pkill -INT -f "ros2 bag record" && sleep 5; sync
    ls -dt ~/team_ws/bags/capture-* 2>/dev/null | head -1 | xargs -r du -sh
    exit 0 ;;
  hover|square)   # load the flight into the competitor window; the PILOT starts it (arm, then Position)
    if [[ "$1" == square ]]; then SQ="${2:-0.5}"; NAME=square; else SQ=0.0; NAME=hover; fi
    TS=$(date +%Y%m%d-%H%M%S)
    mkdir -p ~/team_ws/bags ~/team_ws/drone_tests/runs
    docker exec $C pkill -INT -f "ros2 bag record" 2>/dev/null && sleep 3   # one bag at a time
    # bag: cameras + IMU + VIO + bridge + every /fmu/out topic, max 10 min (~25 GB), kept in team_ws
    docker exec -d $C bash -lc "source skybeta_env.sh; record_bag.sh --duration 600 -o /root/team_ws/bags/$NAME-$TS > /root/team_ws/drone_tests/runs/$NAME-$TS.bag.log 2>&1"
    docker exec $C tmux respawn-window -k -t skybeta:competitor \
      "bash -lc 'source skybeta_env.sh; ros2 launch /root/team_ws/drone_tests/hover.launch.py square:=$SQ 2>&1 | tee /root/team_ws/drone_tests/runs/$NAME-$TS.log; exec bash'"
    sleep 8
    docker exec $C tmux capture-pane -p -J -t skybeta:competitor | grep -v '^$' | tail -4
    echo "$NAME loaded (square side $SQ m); recording ~/team_ws/bags/$NAME-$TS and runs/$NAME-$TS.log"
    echo "Pilot: arm on Altitude, then Position. Watch: $0 hoverlog   After landing: $0 restore"
    exit 0 ;;
  hoverlog) docker exec $C tmux capture-pane -p -J -t skybeta:competitor -S -200 | grep -v '^$' | grep -E 'hover_test|python3|LAND|ERROR|WARN' | tail -25; exit 0 ;;
  restore)   # stop the bag (flushes it), leave NO mission loaded (arming + Position = pilot only), sync
    docker exec $C pkill -INT -f "ros2 bag record" && sleep 5 && echo "bag stopped"
    docker exec $C tmux respawn-window -k -t skybeta:competitor "bash -c 'echo [dt] no mission loaded: Position stays with the pilot; exec bash'"
    docker exec $C chown -R "$(id -u):$(id -g)" /root/team_ws/bags /root/team_ws/drone_tests/runs 2>/dev/null
    sync
    echo "no mission loaded (pilot only); files synced to disk. Default launch back: $0 normal"; exit 0 ;;
  normal)    # the boot default: COMPETITOR_PKG from drone.env (now: competitor_example, NOT anchored)
    docker exec $C tmux respawn-window -k -t skybeta:competitor "bash -lc 'source skybeta_env.sh; run_competitor.sh; exec bash'"
    echo "competitor window back to COMPETITOR_PKG from drone.env"; exit 0 ;;
  log) tr '\r' '\n' < "$LOG" | grep -v '^$' | awk '!/centre/ || NR%5==0'; exit 0 ;;
  motors) shift; ARGS="--spin --props-off --seconds 15 $*" ;;
  pulse)  shift; ARGS="--pulse 0.15 --pulse-seconds 4 --spin --props-off $*" ;;
  run)    shift; ARGS="$*" ;;   # internal: the detached part
  *) sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
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
