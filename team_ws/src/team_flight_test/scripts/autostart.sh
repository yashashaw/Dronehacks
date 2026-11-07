#!/usr/bin/env bash
# Make the drone start the mission by itself (at boot and right now), or stop doing so.
# Run ON THE JETSON host (ssh in), not in the container.
#
#   autostart.sh enable [--profile hover] [--alt 1.0] [--speed 0.25] [--scale 1.0]
#                       [--max-legs 0] [--yaw-rate 20] [--run-s 60] [--bag]
#   autostart.sh disable        nothing starts; the drone stays in Position under the pilot
#   autostart.sh status
#
#   --next-boot   only write the settings; leave the running competitor window alone
#
# How it works: drone.env (once, needs sudo) points the stack's competitor window at
# team_flight_test/autostart.launch.py; that launch file reads ~/team_ws/autostart.conf.
# "Now" means respawning ONLY the competitor window: the agent, VIO and the Pixhawk
# link are not restarted. Refuses to touch the running window while the drone is armed.
#
# Even when enabled, nothing flies until the pilot ARMS and flips to POSITION.
# Abort: RC kill switch, or ~/team_ws/src/team_flight_test/scripts/kill_flight.sh
set -euo pipefail

CONTAINER=skybeta-stack
SESSION=skybeta
STATUS_CONTAINER=skybeta-ros-status
TEAM_WS="${TEAM_WS:-$HOME/team_ws}"
CONF="$TEAM_WS/autostart.conf"
ENV_FILE=/etc/skybeta/drone.env
PKG=team_flight_test
LAUNCH=autostart.launch.py

say() { printf '[autostart] %s\n' "$*"; }
die() { printf '[autostart] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; }

CMD="${1:-status}"; [[ $# -gt 0 ]] && shift
PROFILE=hover ALT=1.0 SPEED=0.25 SCALE=1.0 LEGS=0 YAWRATE=20 RUNS=60 BAG=false NOW=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)  PROFILE="$2"; shift 2 ;;
    --alt)      ALT="$2"; shift 2 ;;
    --speed)    SPEED="$2"; shift 2 ;;
    --scale)    SCALE="$2"; shift 2 ;;
    --max-legs) LEGS="$2"; shift 2 ;;
    --yaw-rate) YAWRATE="$2"; shift 2 ;;
    --run-s)    RUNS="$2"; shift 2 ;;
    --bag)      BAG=true; shift ;;
    --next-boot) NOW=0; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) die "unknown option: $1 (see --help)" ;;
  esac
done
case "$PROFILE" in hover|yaw|yaw45|strafe|out_back|box|course) ;; *) die "bad --profile $PROFILE" ;; esac

stack_up() { [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == "true" ]]; }

armed() {   # prints true/false/unknown
  docker exec "$STATUS_CONTAINER" cat /out/ros_summary.json 2>/dev/null | python3 -c '
import json, sys
try: a = (json.load(sys.stdin).get("fc") or {}).get("armed")
except Exception: a = None
print({True: "true", False: "false"}.get(a, "unknown"))' 2>/dev/null || echo unknown
}

point_stack_here() {   # drone.env: COMPETITOR_PKG / COMPETITOR_LAUNCH (persist across boots)
  if grep -qx "COMPETITOR_PKG=$PKG" "$ENV_FILE" && grep -qx "COMPETITOR_LAUNCH=$LAUNCH" "$ENV_FILE"; then
    return 0
  fi
  say "pointing the boot stack at $PKG/$LAUNCH in $ENV_FILE (sudo)"
  sudo cp "$ENV_FILE" "$ENV_FILE.bak.$(date +%Y%m%d-%H%M%S)"
  sudo sed -i -e "s|^COMPETITOR_PKG=.*|COMPETITOR_PKG=$PKG|" \
              -e "s|^COMPETITOR_LAUNCH=.*|COMPETITOR_LAUNCH=$LAUNCH|" "$ENV_FILE"
  grep -q '^COMPETITOR_PKG=' "$ENV_FILE"    || echo "COMPETITOR_PKG=$PKG"       | sudo tee -a "$ENV_FILE" >/dev/null
  grep -q '^COMPETITOR_LAUNCH=' "$ENV_FILE" || echo "COMPETITOR_LAUNCH=$LAUNCH" | sudo tee -a "$ENV_FILE" >/dev/null
}

apply_now() {
  [[ $NOW -eq 1 ]] || { say "--next-boot: running stack left alone"; return 0; }
  stack_up || { say "stack not running; the setting takes effect when it starts"; return 0; }
  local a; a="$(armed)"
  [[ "$a" == "false" ]] || die "drone armed state is '$a' -- not touching the running mission. Disarm first, or use --next-boot."
  # The container's COMPETITOR_* env is from when it started, so pass ours explicitly.
  docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" \
    "COMPETITOR_PKG=$PKG COMPETITOR_LAUNCH=$LAUNCH exec run_competitor.sh"
  say "competitor window restarted (it builds team_ws first: allow ~1 min). Watch:"
  say "  docker exec -it $CONTAINER tmux attach -t $SESSION    (Ctrl-b w -> competitor, Ctrl-b d to leave)"
}

case "$CMD" in
  enable)
    [[ -d "$TEAM_WS/src/$PKG" ]] || die "$TEAM_WS/src/$PKG not found -- copy the package to the drone first"
    [[ -d "$TEAM_WS/src/team_takeoff_land" ]] || die "$TEAM_WS/src/team_takeoff_land not found -- it is needed next to $PKG"
    cat > "$CONF" <<EOF
# written by autostart.sh $(date '+%F %T') -- read by team_flight_test/launch/autostart.launch.py
enabled=true
profile=$PROFILE
alt=$ALT
speed=$SPEED
scale=$SCALE
max_legs=$LEGS
yaw_rate=$YAWRATE
run_s=$RUNS
bag=$BAG
EOF
    point_stack_here
    say "ENABLED: profile=$PROFILE alt=$ALT speed=$SPEED scale=$SCALE max_legs=$LEGS yaw_rate=$YAWRATE run_s=$RUNS bag=$BAG"
    apply_now
    say "TO FLY: pilot ARMS, then flips to POSITION.  ABORT: RC kill switch, or scripts/kill_flight.sh"
    ;;
  disable)
    if [[ -f "$CONF" ]]; then sed -i 's/^enabled=.*/enabled=false/' "$CONF"; else echo "enabled=false" > "$CONF"; fi
    # Point the boot stack at our launcher too: with enabled=false it starts nothing.
    # Otherwise drone.env keeps competitor_example and the 5 m square starts at every boot.
    point_stack_here
    say "DISABLED: the mission will not start (now or at boot). The drone stays in Position under the pilot."
    apply_now
    ;;
  status)
    echo "--- $CONF"; cat "$CONF" 2>/dev/null || echo "(missing = disabled)"
    echo "--- $ENV_FILE"; grep -E '^COMPETITOR_(PKG|LAUNCH)=' "$ENV_FILE" || true
    if stack_up; then
      echo "--- competitor window (last lines), armed=$(armed)"
      docker exec "$CONTAINER" tmux capture-pane -p -t "$SESSION:competitor" | sed '/^[[:space:]]*$/d' | tail -6
    else
      echo "--- stack not running"
    fi
    ;;
  *) usage; exit 2 ;;
esac
