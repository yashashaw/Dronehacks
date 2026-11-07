#!/usr/bin/env bash
# Field tests: one command for every walk, flight, stop and kill. Run ON THE JETSON (ssh in).
#
#   ft.sh status                      stack, drone, camera mode, logger, mission window
#   ft.sh camera new|old|status       new = 640x400 + depth (use this), old = 1024x640 no depth
#                                     (switching restarts the stack: disarmed only, ~1 min)
#
# WALKS (props off, carried by hand):
#   ft.sh walk W0|W1|W2|W3|W4|W5|C [name]   start logging        (see README.md for each test)
#   ft.sh stop [--truth 3,2.5,2]            stop logging + analyse
#
# FLIGHTS (props on, safety pilot on the RC):
#   ft.sh fly F0..F7 [--bag] [extra options]  start logger + load the test; pilot ARMS then POSITION
#   ft.sh kill                                LAND NOW, stop the mission, stop logging
#   ft.sh done                                after landing: stop the mission, stop logging, analyse
#
# GROUND CHECKS (props OFF, battery in):
#   ft.sh motors                              explains per-motor tests (QGroundControl), then runs watch
#   ft.sh watch [--seconds 120]               report arm/disarm/mode while the pilot does RC checks
#
#   ft.sh analyze latest|RUN [--truth ...]    re-run the analysis
#   ft.sh compare RUN RUN ...                 side by side + overlay map (same start mark!)
#   ft.sh list                                runs so far
#
# The RC kill switch is always the real emergency stop. `ft.sh kill` lands (takes seconds).
set -uo pipefail

CONTAINER=skybeta-stack
SESSION=skybeta
STATUS_CONTAINER=skybeta-ros-status
TEAM_WS="${TEAM_WS:-$HOME/team_ws}"
FT="$TEAM_WS/field_tests"
FT_C=/root/team_ws/field_tests
RUNS="$FT/runs"
MISSION="$TEAM_WS/src/team_flight_test/scripts"
TAKEOFF="$TEAM_WS/src/team_takeoff_land/scripts"
mkdir -p "$RUNS"

say()  { printf '[ft] %s\n' "$*"; }
warn() { printf '[ft] WARNING: %s\n' "$*" >&2; }
die()  { printf '[ft] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,28p' "$0" | sed 's/^# \{0,1\}//'; }

stack_up() { [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == "true" ]]; }
in_stack() { docker exec "$CONTAINER" bash -c "source skybeta_env.sh >/dev/null 2>&1; $1"; }

fc() {   # prints: link armed nav vio xy_valid
  docker exec "$STATUS_CONTAINER" cat /out/ros_summary.json 2>/dev/null | python3 -c '
import json, sys
try: d = json.load(sys.stdin)
except Exception: d = {}
fc = d.get("fc") or {}; v = d.get("vio") or {}; lp = fc.get("local_position") or {}
b = lambda x: {True: "yes", False: "no"}.get(x, "?")
print(b(fc.get("link")), b(fc.get("armed")), fc.get("nav_state_name") or "?", v.get("vo_state_name") or "?", b(lp.get("xy_valid")))' 2>/dev/null || echo "? ? ? ? ?"
}

require_disarmed() {
  local link armed nav vio xy
  read -r link armed nav vio xy <<<"$(fc)"
  [[ "$armed" == "no" ]] || die "drone armed state is '$armed' -- this needs a DISARMED drone"
}

has_depth() {   # capture first: grep -q exiting early + pipefail would read as "no"
  local topics; topics="$(in_stack "timeout 10 ros2 topic list" 2>/dev/null)"
  grep -qx /oak/stereo/image_raw <<<"$topics"
}

camera_mode_file() { awk 'NF {print $1; exit}' "$TEAM_WS/slam_profile" 2>/dev/null || echo stock; }

# ------------------------------------------------------------------------------ logger
fix_owner() { stack_up && docker exec "$CONTAINER" chown -R "$(id -u):$(id -g)" "$FT_C/runs" 2>/dev/null; true; }   # container writes as root

logger_running() { stack_up && docker exec "$CONTAINER" pgrep -f wall_logger.py >/dev/null 2>&1; }

logger_start() {   # label
  stack_up || die "stack is not running (systemctl status skybeta-stack)"
  logger_running && { warn "a logger was still running -- stopping it first"; logger_stop >/dev/null; }
  docker exec -d "$CONTAINER" bash -c "source skybeta_env.sh >/dev/null 2>&1; \
    exec python3 $FT_C/wall_logger.py --label '$1' > $FT_C/runs/.logger.out 2>&1"
  sleep 3
  logger_running || { cat "$RUNS/.logger.out" 2>/dev/null; die "logger did not start"; }
  say "logging: $(ls -td "$RUNS"/*_"$1" 2>/dev/null | head -1)"
}

logger_stop() {    # prints the run dir
  if logger_running; then
    docker exec "$CONTAINER" pkill -INT -f wall_logger.py
    for _ in $(seq 1 20); do logger_running || break; sleep 0.5; done
    logger_running && docker exec "$CONTAINER" pkill -KILL -f wall_logger.py
  fi
  fix_owner
  ls -td "$RUNS"/*/ 2>/dev/null | head -1 | sed 's:/$::'
}

analyze() {        # run_dir [args]
  local run="$1"; shift
  [[ -d "$run" ]] || die "no run '$run'"
  stack_up || die "analysis runs in the stack container (numpy/matplotlib); start the stack"
  in_stack "python3 $FT_C/analyze.py $FT_C/runs/$(basename "$run") $*"
  fix_owner
  say "files: $run/{report.txt,path.png,wall.png}   laptop: scp -r dronehacks@<drone>:$run ."
}

resolve_run() {
  case "${1:-latest}" in
    latest) ls -td "$RUNS"/*/ 2>/dev/null | head -1 | sed 's:/$::' ;;
    /*) echo "$1" ;;
    *) [[ -d "$RUNS/$1" ]] && echo "$RUNS/$1" || ls -td "$RUNS"/*"$1"*/ 2>/dev/null | head -1 | sed 's:/$::' ;;
  esac
}

mission_idle() {
  docker exec "$CONTAINER" tmux respawn-window -k -t "$SESSION:competitor" \
    "bash -c 'echo \"[ft] mission idle since \$(date +%T). Nothing will fly until a new ft.sh fly / autostart.\"; exec bash'"
  if docker exec "$CONTAINER" tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx bag; then
    docker exec "$CONTAINER" tmux send-keys -t "$SESSION:bag" C-c
    say "rosbag stopped"
  fi
}

# ------------------------------------------------------------------------------ tests
walk_info() {
  case "$1" in
    W0) echo "noise floor: drone still on a table/floor for 60 s, camera facing the wall" ;;
    W1) echo "depth accuracy: stand at each tape mark (e.g. 0.5 1 1.5 2 3 m from the wall), square to the wall, 3 s still at each; repeat turned ~30 deg. stop with --truth 0.5,1,1.5,2,3" ;;
    W2) echo "approach: start at the 3 m mark facing the wall, walk straight in, 3 s still at 2.5 / 2 / 1.5 / 1 m, then walk back out the same way" ;;
    W3) echo "turn: 1.5 m from the wall, still 3 s; turn 90 left slowly, 3 s; back to the wall, 3 s; 90 right, 3 s; back, 3 s; 180, 3 s; back, 3 s" ;;
    W4) echo "parallel: 1 m from the wall, camera facing the wall, side-step 3-4 m along it, 3 s still, side-step back, 3 s" ;;
    W5) echo "lose and find: start mark facing the wall 3 s; walk a loop where the wall leaves view (turn away, ~5 m), come back to the start mark facing the wall, 3 s" ;;
    C*) echo "course: start mark, facing 'north', still 60 s after power-on; carry at ~1 m height, level, facing travel, slow; 3 s still at every corner; end on the start mark, 3 s" ;;
    *) return 1 ;;
  esac
}

# id|label|kind|arg|where the drone goes|what to watch
FLIGHTS='
F0|rehearsal|mission|hover|PROPS OFF. Practice the whole sequence: ft.sh fly F0, pilot arms + Position, then ft.sh kill|motors spin up, Offboard engages; kill lands/disarms
F1|takeoff_land|takeoff|--alt 1.0 --hover 3|2 m from the wall, facing it|clean takeoff to 1 m, 3 s, lands by itself
F2|hover|mission|hover|2 m from the wall, facing it|holds 10 s; wall drift should stay within a few cm
F3|yaw45|mission|yaw45|2 m from the wall, facing it|turns +-45 deg and back; wall drift after each turn back
F4|yaw90|mission|yaw|2 m from the wall, facing it|turns +-90 deg (wall leaves view) and back; turn jump
F5|out_back|mission|out_back|2.5 m from the wall, facing it|1 m toward the wall and back (closest 1.5 m); distance scale
F6|strafe|mission|strafe|2 m from the wall, facing it, 1 m clear on its LEFT|0.75 m left and back; sideways drift
F7|box|mission|box --scale 0.7|2.5 m from the wall, facing it, 1 m clear on its RIGHT|0.7 m square turning at corners
'

flight_row() { grep "^$1|" <<<"$FLIGHTS"; }

cmd_fly() {
  local id="${1:-}"; shift || true
  local row; row="$(flight_row "$id")" || die "unknown flight '$id' (F0..F7, see ft.sh list-flights)"
  IFS='|' read -r _ name kind arg where watch <<<"$row"
  local bag=0 extra=() allow_old=0 ack=0
  while [[ $# -gt 0 ]]; do
    case "$1" in --bag) bag=1; shift ;; --old-camera) allow_old=1; shift ;; --ack-f1-incident) ack=1; shift ;;
                 *) extra+=("$1"); shift ;; esac
  done
  # 2026-11-07: F1 climbed nearly to the ceiling (VIO-frame height sent to PX4 un-anchored).
  [[ $ack -eq 1 ]] || die "flights are blocked: read ~/team_ws/F1_CLIMB_INCIDENT.md (repo: team_ws/F1_CLIMB_INCIDENT.md), fix it, then add --ack-f1-incident"
  stack_up || die "stack is not running"
  local link armed nav vio xy
  read -r link armed nav vio xy <<<"$(fc)"
  [[ "$link" == "yes" ]] || die "no Pixhawk link"
  [[ "$armed" == "no" ]] || die "drone is ARMED -- land and disarm first"
  [[ "$vio" == "tracking" ]] || warn "VIO is '$vio', not tracking"
  if ! has_depth; then
    [[ $allow_old -eq 1 ]] || die "camera is in the old mode (no depth): run 'ft.sh camera new' first, or pass --old-camera"
    warn "no depth: the wall columns will be empty"
  fi
  logger_start "${id}_${name}"
  case "$kind" in
    takeoff)
      # shellcheck disable=SC2086
      "$TAKEOFF/start_test.sh" $arg "${extra[@]}" || { logger_stop >/dev/null; die "start_test.sh failed"; } ;;
    mission)
      local b=(--no-bag); [[ $bag -eq 1 ]] && b=()
      # shellcheck disable=SC2086
      "$MISSION/start_flight_test.sh" --profile $arg "${b[@]}" "${extra[@]}" || { logger_stop >/dev/null; die "start_flight_test.sh failed"; } ;;
  esac
  cat <<EOM

[ft] ===================== $id $name =====================
[ft]  PLACE:   $where
[ft]  WATCH:   $watch
[ft]  FLY:     leave it still a few s, then pilot ARMS (Altitude) -> flips to POSITION
[ft]  ABORT:   RC kill switch (instant)  |  ft.sh kill  (lands, ~5-15 s)
[ft]  AFTER:   ft.sh done   (stops the mission + logger, prints the report)
[ft] =====================================================
EOM
}

cmd_camera() {
  case "${1:-status}" in
    status)
      say "mode file: $(camera_mode_file) ($TEAM_WS/slam_profile)"
      if stack_up; then
        has_depth && say "running: depth ON (/oak/stereo/image_raw)" || say "running: depth OFF"
        in_stack "timeout 6 ros2 topic echo --once /oak/left/camera_info 2>/dev/null | grep -E '^(width|height):' | tr '\n' ' '"; echo
      fi ;;
    new|old)
      require_disarmed
      if [[ "$1" == new ]]; then echo depth > "$TEAM_WS/slam_profile"; else echo stock > "$TEAM_WS/slam_profile"; fi
      logger_running && logger_stop >/dev/null
      say "camera mode -> $1; restarting the stack (sudo), ~1 min. Keep the drone STILL."
      local since; since="$(date '+%F %T')"
      sudo systemctl restart skybeta-stack.service || die "restart failed"
      say "the service waits up to 90 s for a clock sync (no internet) before starting -- ~2 min"
      local up=0
      for _ in $(seq 1 80); do
        sleep 3
        journalctl -u skybeta-stack --since "$since" --no-pager 2>/dev/null | grep -q "stack up" && { up=1; break; }
      done
      [[ $up -eq 1 ]] || die "stack did not come up in 4 min: journalctl -u skybeta-stack -n 50"
      sleep 20      # VIO initialises; the status sidecar refreshes
      read -r link armed nav vio xy <<<"$(fc)"
      say "stack back: FC link=$link VIO=$vio"
      cmd_camera status ;;
    *) die "camera new|old|status" ;;
  esac
}

cmd_status() {
  local link armed nav vio xy
  if stack_up; then say "stack: running"; else say "stack: NOT running"; fi
  read -r link armed nav vio xy <<<"$(fc)"
  say "drone: FC link=$link armed=$armed mode=$nav VIO=$vio EKF2 xy_valid=$xy"
  say "camera mode file: $(camera_mode_file)   depth topic: $(has_depth && echo yes || echo no)"
  logger_running && say "logger: RUNNING" || say "logger: stopped"
  stack_up && { say "mission window (last lines):"; docker exec "$CONTAINER" tmux capture-pane -p -t "$SESSION:competitor" \
    | sed '/^[[:space:]]*$/d' | tail -4 | sed 's/^/      /'; }
}

CMD="${1:-help}"; [[ $# -gt 0 ]] && shift
case "$CMD" in
  status) cmd_status ;;
  camera) cmd_camera "$@" ;;
  walk)
    id="${1:-}"; [[ -n "$id" ]] || die "walk W0..W5 or C [name]"
    info="$(walk_info "$id")" || die "unknown walk '$id' (W0..W5, C1/C2/...)"
    has_depth || warn "camera in old mode (no depth): wall numbers will be empty. 'ft.sh camera new' first?"
    logger_start "${id}_${2:-walk}"
    say "DO: $info"
    say "then: ft.sh stop$([[ "$id" == W1 ]] && echo ' --truth 0.5,1,1.5,2,3')" ;;
  stop)
    run="$(logger_stop)"; say "stopped: $run"; analyze "$run" "$@" ;;
  fly) cmd_fly "$@" ;;
  list-flights) awk -F'|' 'NF>2 {printf "%-3s %-13s %s\n      watch: %s\n", $1, $2, $5, $6}' <<<"$FLIGHTS" ;;
  kill)
    "$MISSION/kill_flight.sh" || warn "kill_flight.sh reported a problem -- PILOT, TAKE OVER"
    run="$(logger_stop)"; say "logger stopped: $run"
    say "after it has landed: ft.sh analyze latest" ;;
  done)
    read -r link armed nav vio xy <<<"$(fc)"
    if [[ "$armed" == yes && "${1:-}" != --force ]]; then die "still ARMED: wait for landing + disarm (or ft.sh kill)"; fi
    mission_idle; say "mission stopped (idle)"
    run="$(logger_stop)"; analyze "$run" ;;
  analyze) run="$(resolve_run "${1:-latest}")"; shift || true; analyze "$run" "$@" ;;
  compare)
    [[ $# -ge 2 ]] || die "compare RUN RUN [...]"
    args=(); for r in "$@"; do args+=("$FT_C/runs/$(basename "$(resolve_run "$r")")"); done
    in_stack "python3 $FT_C/analyze.py --compare ${args[*]}"; fix_owner ;;
  list) ls -t "$RUNS" 2>/dev/null | grep -v '^\.' | head -30 ;;
  motors)
    say "Per-motor spin tests are NOT available from the Jetson on this PX4 build (tested 2026-11-07:"
    say "DO_MOTOR_TEST -> UNSUPPORTED, ACTUATOR_TEST -> ignored). For each motor alone use"
    say "QGroundControl -> Vehicle Setup -> Motors (USB to the Pixhawk, PROPS OFF)."
    say "Motors on/off + kill switch + software kill, guided by the RC pilot: running 'ft.sh watch' now."
    exec "$0" watch "$@" ;;
  watch)
    stack_up || die "stack is not running"
    tt=(); [[ -t 0 ]] && tt=(-it)
    docker exec "${tt[@]}" "$CONTAINER" bash -c "source skybeta_env.sh >/dev/null 2>&1; exec python3 $FT_C/motor_check.py watch $*" ;;
  help|-h|--help) usage ;;
  *) usage; exit 2 ;;
esac
