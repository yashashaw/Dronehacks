#!/usr/bin/env bash
# Drone tests, built up one at a time. Run ON THE JETSON (ssh skybeta-18).
#
#   dt.sh status                       stack, PX4 link/armed/mode, camera mode, competitor window
#   dt.sh camera status                what the camera is doing right now
#   dt.sh camera depth                 640x400 VIO + on-device depth (restarts the stack, ~2 min)
#   dt.sh camera stock                 back to the stock 1024x640, no depth (restarts the stack)
#   dt.sh motors [options]             PROPS OFF bench test: motor speed follows the centre depth
#                                      dry run unless you add:  --spin --props-off
#                                      stops the competitor window (flight_interface) first, and
#                                      brings it back when done
#   dt.sh pulse [THR] [options]        PROPS OFF: motors on at THR (default 0.10) for 2 s, then off
#                                      dry run unless you add:  --spin --props-off
#   Both log the VIO + PX4 position estimate (should stay ~0: the drone stands still) to
#   ~/team_ws/drone_tests/runs/<time>_{motors,pulse}.csv and print how far it moved.
#
# The RC kill switch is always the real emergency stop.
set -uo pipefail

CONTAINER=skybeta-stack
SESSION=skybeta
TEAM_WS="${TEAM_WS:-$HOME/team_ws}"
DT_C=/root/team_ws/drone_tests

say()  { printf '[dt] %s\n' "$*"; }
warn() { printf '[dt] WARNING: %s\n' "$*" >&2; }
die()  { printf '[dt] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; }

stack_up() { [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == "true" ]]; }
in_stack() { docker exec "$CONTAINER" bash -c "source skybeta_env.sh >/dev/null 2>&1; $1"; }
camera_mode_file() { awk 'NF {print $1; exit}' "$TEAM_WS/slam_profile" 2>/dev/null || echo stock; }

has_depth() {
  local topics; topics="$(in_stack "timeout 10 ros2 topic list" 2>/dev/null)"
  grep -qx /oak/stereo/image_raw <<<"$topics"
}

px4_state() {   # prints: armed nav_state   (or "? ?")
  in_stack "timeout 8 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_status_v1 2>/dev/null" \
    | awk '/^arming_state:/ {a=$2} /^nav_state:/ {n=$2} END {print (a==""?"?":(a==2?"yes":"no")), (n==""?"?":n)}'
}

require_disarmed() {
  local armed nav; read -r armed nav <<<"$(px4_state)"
  [[ "$armed" == "no" ]] || die "PX4 armed state is '$armed' -- this needs a DISARMED drone with a live FC link"
}

competitor_window_alive() {
  docker exec "$CONTAINER" tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx competitor
}

competitor_stop() {   # replace the competitor window (and its flight_interface) with an idle shell
  competitor_window_alive || return 0
  docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" \
    "bash -c 'echo \"[dt] competitor stopped at \$(date +%T) for a bench test (dt.sh motors)\"; exec bash'"
  say "competitor window stopped (no flight_interface now)"
  sleep 2
}

competitor_restart() {   # back to what the stack normally runs there
  competitor_window_alive || return 0
  docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" \
    "bash -lc 'source skybeta_env.sh; run_competitor.sh; exec bash'"
  say "competitor window restarted (builds team_ws, then launches COMPETITOR_PKG)"
}

cmd_status() {
  stack_up && say "stack: running" || { say "stack: NOT running (systemctl status skybeta-stack)"; return; }
  local armed nav; read -r armed nav <<<"$(px4_state)"
  say "PX4: armed=$armed nav_state=$nav   (14 = Offboard; '?' = no FC link)"
  local rc usb
  rc=$(in_stack "timeout 6 ros2 topic echo --once --qos-reliability best_effort /fmu/out/failsafe_flags 2>/dev/null" \
       | awk '/^manual_control_signal_lost:/ {print ($2=="false" ? "OK" : "LOST - transmitter off/unbound: do not fly")}')
  usb=$(in_stack "timeout 6 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_status_v1 2>/dev/null" \
       | awk '/^usb_connected:/ {print ($2=="false" ? "unplugged (OK)" : "PLUGGED IN - unplug before flight")}')
  say "RC: ${rc:-?}   Pixhawk USB: ${usb:-?}"
  say "camera mode file: $(camera_mode_file)   depth topic: $(has_depth && echo yes || echo no)"
  say "competitor window (last lines):"
  docker exec "$CONTAINER" tmux capture-pane -p -t "$SESSION:competitor" 2>/dev/null \
    | sed '/^[[:space:]]*$/d' | tail -4 | sed 's/^/      /'
}

cmd_camera() {
  case "${1:-status}" in
    status)
      say "mode file: $(camera_mode_file) ($TEAM_WS/slam_profile)"
      grep -q SKYBETA_SLAM_PROFILE "$HOME/skybeta/host/run.sh" 2>/dev/null \
        || warn "~/skybeta has no depth profile support yet: apply skybeta_depth640.patch (README step 3)"
      if stack_up; then
        has_depth && say "running: depth ON (/oak/stereo/image_raw)" || say "running: depth OFF"
        in_stack "timeout 6 ros2 topic echo --once /oak/left/camera_info 2>/dev/null | grep -E '^(width|height):' | tr '\n' ' '"; echo
      fi ;;
    depth|stock)
      grep -q SKYBETA_SLAM_PROFILE "$HOME/skybeta/host/run.sh" 2>/dev/null \
        || die "~/skybeta has no depth profile support: apply skybeta_depth640.patch first (README step 3)"
      require_disarmed
      echo "$1" > "$TEAM_WS/slam_profile"
      say "camera -> $1; restarting the stack (sudo), ~2 min. Keep the drone STILL (VIO origin resets)."
      local since; since="$(date '+%F %T')"
      sudo systemctl restart skybeta-stack.service || die "restart failed"
      local up=0
      for _ in $(seq 1 80); do
        sleep 3
        journalctl -u skybeta-stack --since "$since" --no-pager 2>/dev/null | grep -q "stack up" && { up=1; break; }
      done
      [[ $up -eq 1 ]] || die "stack did not come up in 4 min: journalctl -u skybeta-stack -n 50"
      sleep 15
      cmd_camera status ;;
    *) die "camera status|depth|stock" ;;
  esac
}

cmd_motors() {
  stack_up || die "stack is not running"
  local spin=0 pulse=0
  for x in "$@"; do [[ "$x" == --spin ]] && spin=1; [[ "$x" == --pulse ]] && pulse=1; done
  [[ $pulse -eq 1 ]] || has_depth || die "no depth topic: run 'dt.sh camera depth' first"
  if [[ $spin -eq 1 ]]; then
    require_disarmed
    cat <<'EOM'
[dt] ===== PROPS OFF BENCH TEST =====
[dt]  * propellers REMOVED, drone held/strapped down, hands clear of the motors
[dt]  * RC transmitter ON, kill switch checked (it is the emergency stop)
[dt]  * Pixhawk USB cable UNPLUGGED (PX4 will not arm with it in)
EOM
    read -r -p "[dt] type 'props off' to continue: " ok
    [[ "$ok" == "props off" ]] || die "not confirmed"
    competitor_stop
    trap 'competitor_restart' EXIT
  fi
  docker exec -it "$CONTAINER" bash -c "source skybeta_env.sh >/dev/null 2>&1; python3 $DT_C/depth_motors.py $*"
  docker exec "$CONTAINER" chown -R "$(id -u):$(id -g)" "$DT_C/runs" 2>/dev/null || true   # container writes as root
}

cmd="${1:-}"; shift || true
case "$cmd" in
  status) cmd_status ;;
  camera) cmd_camera "$@" ;;
  motors) cmd_motors "$@" ;;
  pulse)  thr="${1:-0.10}"; shift || true; cmd_motors --pulse "$thr" "$@" ;;
  ""|-h|--help|help) usage ;;
  *) usage; exit 2 ;;
esac
