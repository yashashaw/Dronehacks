#!/bin/bash
# Laptop-side helper for the hand-carry and known-object tests.
#
#   ./run_walk.sh start NAME SECONDS        deploy, then start the logger DETACHED on the drone
#   ./run_walk.sh pull  NAME [--object L,W] fetch the run and analyze it
#   ./run_walk.sh health                    start the /loc/health node detached
#
# The logger keeps running if the SSH link resets or you unplug the USB-C cable (as long as
# the drone stays powered), so: start -> carry -> reconnect -> pull.
# Set DRONE to the drone's address if it is not the USB link default.
DRONE=${DRONE:-dronehacks@192.168.55.1}
HERE=$(cd "$(dirname "$0")" && pwd)
ENVSRC='source /opt/ros/*/setup.bash; source /usr/local/bin/skybeta_env.sh'

retry() {  # the USB link drops connections; retry only connection failures (rc 255)
  for i in 1 2 3 4 5 6; do
    "$@" && return 0
    rc=$?; [ $rc -ne 255 ] && return $rc; sleep 2
  done; return 255
}
r() { retry ssh -o BatchMode=yes -o ConnectTimeout=8 "$DRONE" "$@"; }

case "$1" in
  start)
    NAME=${2:?run name}; SECS=${3:-60}
    tar czf - -C "$HERE/.." loc_stack 2>/dev/null | r 'mkdir -p ~/team_ws && tar xzf - -C ~/team_ws' || exit 1
    r "docker exec -d skybeta-stack bash -lc '$ENVSRC; python3 /root/team_ws/loc_stack/walk_logger.py --seconds $SECS --out /root/team_ws/walk_runs/$NAME'" || exit 1
    echo "logging '$NAME' for $SECS s. Hold still ~3 s at every mark. Pull with: ./run_walk.sh pull $NAME" ;;
  health)
    r "docker exec -d skybeta-stack bash -lc '$ENVSRC; python3 /root/team_ws/loc_stack/health_node.py'" && echo "health node started (/loc/health)" ;;
  pull)
    NAME=${2:?run name}; shift 2
    r "test -f ~/team_ws/walk_runs/$NAME/DONE" || { echo "run not finished yet"; exit 1; }
    mkdir -p "$HERE/runs/$NAME"
    for f in odom.csv status.csv lpos.csv DONE; do
      retry scp -q -o BatchMode=yes -o ConnectTimeout=8 "$DRONE:team_ws/walk_runs/$NAME/$f" "$HERE/runs/$NAME/" 2>/dev/null
    done
    python "$HERE/analyze_walk.py" "$HERE/runs/$NAME" "$@"
    python "$HERE/plot_walk.py" "$HERE/runs/$NAME" "$@" && echo "open: $HERE/runs/$NAME/map.html" ;;
  *) sed -n 2,9p "$0" ;;
esac
