#!/usr/bin/env bash
# Stop the autonomous mission NOW: land if flying, then stop the mission code.
# Run ON THE JETSON, or from the laptop in one line:
#
#   ssh dronehacks@<drone-ip> '~/team_ws/src/team_flight_test/scripts/kill_flight.sh'
#
#   --disable   also turn autostart off, so the mission does not come back at next boot
#
# This LANDS the drone; it does not cut the motors (cutting motors in the air drops it).
# The RC kill switch is the motor cut, and the pilot's sticks/mode switch always win.
#
# Order matters: land first, stop the code after. Stopping the code first would leave
# PX4 in Offboard with no setpoints, and what it does then depends on COM_OBL_RACT.
#   1. armed?  -> /team/land (takeoff_land: NAV_LAND, disarm on touchdown)
#                 if that fails -> NAV_LAND straight to /fmu/in/vehicle_command
#                 then wait up to 5 s to see PX4 in AUTO_LAND (or disarmed)
#   2. stop the mission: the competitor window is replaced by an idle shell
#      (VIO, the agent and the Pixhawk link keep running). Offboard cannot come back
#      until the pilot disarms and arms again.
set -uo pipefail

CONTAINER=skybeta-stack
SESSION=skybeta
STATUS_CONTAINER=skybeta-ros-status
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISABLE=0
for a in "$@"; do
  case "$a" in
    --disable) DISABLE=1 ;;
    -h|--help) sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $a" >&2; exit 2 ;;
  esac
done

say() { printf '[kill] %s\n' "$*"; }

fc_state() {   # prints "<armed> <nav_state_name>", e.g. "true OFFBOARD"; "unknown unknown" if no data
  docker exec "$STATUS_CONTAINER" cat /out/ros_summary.json 2>/dev/null | python3 -c '
import json, sys
try: fc = json.load(sys.stdin).get("fc") or {}
except Exception: fc = {}
a = {True: "true", False: "false"}.get(fc.get("armed"), "unknown")
print(a, fc.get("nav_state_name") or "unknown")' 2>/dev/null || echo "unknown unknown"
}

in_stack() { docker exec "$CONTAINER" bash -c "source skybeta_env.sh >/dev/null 2>&1; $1"; }

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]]; then
  say "stack container is not running: no mission code is running. (If the drone is flying, use the RC.)"
  exit 1
fi

read -r ARMED MODE <<<"$(fc_state)"
say "drone: armed=$ARMED mode=$MODE"

if [[ "$ARMED" != "false" ]]; then
  say "LANDING via /team/land"
  if in_stack "timeout 6 ros2 service call /team/land std_srvs/srv/Trigger" 2>&1 | grep -q "success=True"; then
    say "takeoff_land accepted the land request"
  else
    say "/team/land unavailable or refused -- sending NAV_LAND to PX4 directly"
    SYSID="$(sed -n 's/^[[:space:]]*DRONE_ID=\([0-9]\+\).*/\1/p' /etc/skybeta/drone.env 2>/dev/null | tail -1)"
    in_stack "timeout 8 ros2 topic pub -t 5 -r 5 /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
      '{command: 21, param4: .nan, param5: .nan, param6: .nan, param7: .nan, \
        target_system: ${SYSID:-1}, target_component: 1, source_system: 1, source_component: 1, from_external: true}'" \
      >/dev/null 2>&1 || true
  fi
  for _ in $(seq 1 10); do
    read -r ARMED MODE <<<"$(fc_state)"
    [[ "$MODE" == AUTO_LAND || "$ARMED" == false ]] && break
    sleep 0.5
  done
  if [[ "$MODE" == AUTO_LAND || "$ARMED" == false ]]; then
    say "PX4: armed=$ARMED mode=$MODE -- landing/landed"
  else
    say "WARNING: PX4 still reports mode=$MODE armed=$ARMED -- PILOT, TAKE OVER ON RC"
  fi
fi

docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" \
  "bash -c 'echo \"[kill] mission stopped by kill_flight.sh at \$(date +%T). Restart: autostart.sh enable\"; exec bash'"
say "mission code stopped (competitor window is now an idle shell; VIO and the Pixhawk link are untouched)"

if [[ $DISABLE -eq 1 ]]; then
  "$HERE/autostart.sh" disable --next-boot
fi
