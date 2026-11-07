#!/usr/bin/env bash
# One command for the takeoff-then-land test. Run ON THE JETSON (host, not in
# the container), or from the laptop:
#
#   ssh dronehacks@192.168.55.1 '~/team_ws/src/team_takeoff_land/scripts/start_test.sh'
#
# Options:
#   --sysid N        PX4 system id of this drone's FC (default: DRONE_ID from
#                    /etc/skybeta/drone.env; the fleet flasher sets MAV_SYS_ID to it)
#   --alt M          takeoff altitude in metres (default 1.0)
#   --hover S        seconds at altitude before landing (default 1.0)
#   --stop           put competitor_example back in the competitor window
#
# What it does:
#   1. Checks the boot-time stack is running (agent, slam, bridge windows).
#   2. Checks the Pixhawk link, VIO tracking and EKF2 position from the status
#      sidecar. Refuses on a dead link; warns on anything else.
#   3. Restarts ONLY the stack's "competitor" tmux window with this package's
#      test launch (it builds ~/team_ws first). The agent keeps running, so the
#      Pixhawk link survives (restarting the agent on stock v1.16.2 firmware
#      would need an FC power cycle, skybeta bug #15).
#
# It never arms. The pilot arms, then flips to Position: that starts the run.
set -euo pipefail

CONTAINER=skybeta-stack
SESSION=skybeta
STATUS_CONTAINER=skybeta-ros-status
PKG=team_takeoff_land
LAUNCH=takeoff_land_test.launch.py
ALT=1.0
HOVER=1.0
SYSID=""
STOP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sysid) SYSID="$2"; shift 2 ;;
    --alt)   ALT="$2"; shift 2 ;;
    --hover) HOVER="$2"; shift 2 ;;
    --stop)  STOP=1; shift ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

say()  { printf '[start_test] %s\n' "$*"; }
die()  { printf '[start_test] ERROR: %s\n' "$*" >&2; exit 1; }

# -- 1. stack ---------------------------------------------------------------
[[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == "true" ]] \
  || die "$CONTAINER is not running. Check: systemctl status skybeta-stack"
windows="$(docker exec "$CONTAINER" tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null)" \
  || die "tmux session '$SESSION' not found in $CONTAINER"
for w in agent slam bridge competitor; do
  grep -qx "$w" <<<"$windows" || die "stack window '$w' is missing"
done
say "stack running: $(tr '\n' ' ' <<<"$windows")"

if [[ $STOP -eq 1 ]]; then
  docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" \
    'exec run_competitor.sh'
  say "competitor window back to the default (COMPETITOR_PKG from drone.env)"
  exit 0
fi

# -- 2. FC system id ----------------------------------------------------------
if [[ -z "$SYSID" ]]; then
  SYSID="$(sed -n 's/^[[:space:]]*DRONE_ID=\([0-9]\+\).*/\1/p' /etc/skybeta/drone.env 2>/dev/null | tail -1)"
  [[ -n "$SYSID" ]] || die "no DRONE_ID in /etc/skybeta/drone.env; pass --sysid N"
  say "FC system id $SYSID (DRONE_ID from drone.env; override with --sysid)"
fi
[[ "$SYSID" =~ ^[0-9]+$ ]] || die "--sysid must be a number"

# -- 3. health from the status sidecar -------------------------------------
summary="$(docker exec "$STATUS_CONTAINER" cat /out/ros_summary.json 2>/dev/null)" \
  || die "cannot read the status sidecar ($STATUS_CONTAINER)"
health="$(python3 - "$summary" <<'EOF'
import json, sys
d = json.loads(sys.argv[1])
fc = d.get('fc') or {}
vio = d.get('vio') or {}
lp = fc.get('local_position') or {}
print('link' if fc.get('link') else 'nolink')
print(f"armed={fc.get('armed')} mode={fc.get('nav_state_name')} "
      f"vio={vio.get('vo_state_name')} xy_valid={lp.get('xy_valid')} "
      f"battery_connected={(fc.get('battery') or {}).get('connected')}")
for cond, text in [
    (fc.get('armed'), 'already ARMED: disarm first (Offboard only follows a fresh arm)'),
    (vio.get('vo_state_name') != 'tracking', 'VIO is not tracking'),
    (not lp.get('xy_valid'), 'EKF2 xy position not valid: the flight interface will refuse Offboard'),
    (not (fc.get('battery') or {}).get('connected'), 'no battery seen by the FC (on USB power? PX4 will not arm)'),
]:
    if cond:
        print('WARN ' + text)
EOF
)" || die "could not parse the status sidecar"
[[ "$(head -1 <<<"$health")" == "link" ]] \
  || die "no Pixhawk link (power it on; on stock firmware power-cycle it after any stack restart)"
say "$(sed -n 2p <<<"$health")"
grep '^WARN ' <<<"$health" | sed 's/^WARN /[start_test] WARNING: /' || true

# -- 4. start the test in the competitor window -----------------------------
cmd="export COMPETITOR_PKG=$PKG COMPETITOR_LAUNCH=$LAUNCH; \
source skybeta_env.sh; \
team_build.sh || { echo '[start_test] team_ws build failed'; exec bash; }; \
source skybeta_env.sh; \
exec ros2 launch $PKG $LAUNCH fc_system_id:=$SYSID takeoff_altitude_m:=$ALT hover_s:=$HOVER"
docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" "bash -c \"$cmd\""

say "test launch started in the competitor window (alt ${ALT} m, hover ${HOVER} s, FC sysid ${SYSID})"
say "watch it:   docker exec -it $CONTAINER tmux attach -t $SESSION   (Ctrl-b 3, Ctrl-b d to leave)"
say "state:      docker exec $CONTAINER bash -lc 'source skybeta_env.sh; ros2 topic echo /team/takeoff_land/state'"
say "TO FLY:     pilot ARMS, then flips to POSITION. It takes off, then lands."
say "ABORT:      kill switch, or take over with the mode switch / sticks"
say "undo:       $0 --stop"
