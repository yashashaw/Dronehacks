#!/usr/bin/env bash
# One command per test flight. Run ON THE JETSON host (not in the container):
#
#   ~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --profile hover
#   ~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --profile yaw
#   ~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --profile out_back
#   ~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --profile box --scale 0.7
#   ~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --profile course --max-legs 2
#
# Options:  --profile P  (hover|yaw|yaw45|strafe|out_back|box|course, default hover)
#           --alt M      takeoff altitude (default 1.0)      --speed V   (default 0.25 m/s)
#           --scale S    distance multiplier (default 1.0)   --max-legs N  (0 = all)
#           --yaw-rate D deg/s (default 20)                  --run-s S   (default 60)
#           --sysid N    FC MAV_SYS_ID (default DRONE_ID from /etc/skybeta/drone.env)
#           --no-bag     do not record a rosbag alongside
#           --stop       put competitor_example back, stop the bag window
#
# It never arms. The pilot ARMS, then flips to POSITION: that starts the run.
# ABORT: kill switch, or take over on RC (mode switch / sticks).
set -euo pipefail

CONTAINER=skybeta-stack
SESSION=skybeta
STATUS_CONTAINER=skybeta-ros-status
PKG=team_flight_test
LAUNCH=flight_test.launch.py
PROFILE=hover ALT=1.0 SPEED=0.25 SCALE=1.0 LEGS=0 YAWRATE=20 RUNS=60 SYSID="" BAG=1 STOP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)  PROFILE="$2"; shift 2 ;;
    --alt)      ALT="$2"; shift 2 ;;
    --speed)    SPEED="$2"; shift 2 ;;
    --scale)    SCALE="$2"; shift 2 ;;
    --max-legs) LEGS="$2"; shift 2 ;;
    --yaw-rate) YAWRATE="$2"; shift 2 ;;
    --run-s)    RUNS="$2"; shift 2 ;;
    --sysid)    SYSID="$2"; shift 2 ;;
    --no-bag)   BAG=0; shift ;;
    --stop)     STOP=1; shift ;;
    -h|--help)  sed -n '2,21p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$PROFILE" in hover|yaw|yaw45|strafe|out_back|box|course) ;; *) echo "bad --profile $PROFILE" >&2; exit 2 ;; esac

say() { printf '[flight_test] %s\n' "$*"; }
die() { printf '[flight_test] ERROR: %s\n' "$*" >&2; exit 1; }

# -- 1. stack -----------------------------------------------------------------
[[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == "true" ]] \
  || die "$CONTAINER is not running. Check: systemctl status skybeta-stack"
windows="$(docker exec "$CONTAINER" tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null)" \
  || die "tmux session '$SESSION' not found in $CONTAINER"
for w in agent slam bridge competitor; do
  grep -qx "$w" <<<"$windows" || die "stack window '$w' is missing"
done

if [[ $STOP -eq 1 ]]; then
  if grep -qx bag <<<"$windows"; then
    docker exec "$CONTAINER" tmux send-keys -t "$SESSION:bag" C-c
    say "bag recording stopped (Ctrl-C sent; it finalises in a few seconds)"
  fi
  docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" 'exec run_competitor.sh'
  say "competitor window back to the default"
  exit 0
fi

# -- 2. FC system id ----------------------------------------------------------
if [[ -z "$SYSID" ]]; then
  SYSID="$(sed -n 's/^[[:space:]]*DRONE_ID=\([0-9]\+\).*/\1/p' /etc/skybeta/drone.env 2>/dev/null | tail -1)"
  [[ -n "$SYSID" ]] || die "no DRONE_ID in /etc/skybeta/drone.env; pass --sysid N"
fi
[[ "$SYSID" =~ ^[0-9]+$ ]] || die "--sysid must be a number"

# -- 3. health (status sidecar) -------------------------------------------------
summary="$(docker exec "$STATUS_CONTAINER" cat /out/ros_summary.json 2>/dev/null)" \
  || die "cannot read the status sidecar ($STATUS_CONTAINER)"
health="$(python3 - "$summary" <<'PY'
import json, sys
d = json.loads(sys.argv[1]); fc = d.get('fc') or {}; vio = d.get('vio') or {}
lp = fc.get('local_position') or {}
print('link' if fc.get('link') else 'nolink')
print(f"armed={fc.get('armed')} mode={fc.get('nav_state_name')} vio={vio.get('vo_state_name')} "
      f"xy_valid={lp.get('xy_valid')} battery={(fc.get('battery') or {}).get('connected')}")
for bad, text in [(fc.get('armed'), 'already ARMED: disarm first'),
                  (vio.get('vo_state_name') != 'tracking', 'VIO is not tracking'),
                  (not lp.get('xy_valid'), 'EKF2 xy not valid: Offboard will be refused'),
                  (not (fc.get('battery') or {}).get('connected'), 'no battery seen by the FC')]:
    if bad: print('WARN ' + text)
PY
)" || die "could not parse the status sidecar"
[[ "$(head -1 <<<"$health")" == "link" ]] \
  || die "no Pixhawk link (power it; on stock firmware power-cycle the FC after any stack restart)"
say "$(sed -n 2p <<<"$health")"
grep '^WARN ' <<<"$health" | sed 's/^WARN /[flight_test] WARNING: /' || true

# -- 4. bag (separate tmux window, stops with --stop) ----------------------------
STAMP="$(date +%Y%m%d-%H%M%S)"
if [[ $BAG -eq 1 ]]; then
  grep -qx bag <<<"$windows" && docker exec "$CONTAINER" tmux kill-window -t "$SESSION:bag" || true
  docker exec "$CONTAINER" tmux new-window -d -t "$SESSION" -n bag \
    "bash -c 'mkdir -p /root/team_ws/flight_logs/bags; record_bag.sh -o /root/team_ws/flight_logs/bags/${STAMP}_${PROFILE}; exec bash'"
  say "rosbag -> ~/team_ws/flight_logs/bags/${STAMP}_${PROFILE}  (stop with --stop after landing)"
fi

# -- 5. launch in the competitor window ------------------------------------------
ARGS="fc_system_id:=$SYSID takeoff_altitude_m:=$ALT profile:=$PROFILE speed:=$SPEED scale:=$SCALE max_legs:=$LEGS yaw_rate_dps:=$YAWRATE max_run_s:=$RUNS"
cmd="source skybeta_env.sh; \
team_build.sh --packages-select team_takeoff_land team_flight_test || { echo '[flight_test] build failed'; exec bash; }; \
source skybeta_env.sh; \
ros2 launch $PKG $LAUNCH $ARGS; exec bash"
docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" "bash -c \"$cmd\""

say "launched: profile=$PROFILE alt=$ALT speed=$SPEED scale=$SCALE max_legs=$LEGS yaw_rate=$YAWRATE run_s=$RUNS sysid=$SYSID"
say "watch:  docker exec -it $CONTAINER tmux attach -t $SESSION    (Ctrl-b w to pick a window, Ctrl-b d to leave)"
say "TO FLY: pilot ARMS, then flips to POSITION.   ABORT: kill switch / take over on RC."
say "after landing: $0 --stop ; then python3 ~/team_ws/src/team_flight_test/scripts/analyze_flight.py --latest"
