#!/usr/bin/env bash
# Hand-carried walk-through recorder. Run on the Jetson host (over SSH), with
# the skybeta stack already up. Nothing here arms, commands or publishes: it
# only records what the camera, cuVSLAM and the Pixhawk already publish.
#
#   bash record_walk.sh            label, Enter to start, Enter (or Ctrl-C) to stop
#   bash record_walk.sh --mono     same, eyes recorded as mono8 (1/3 the disk)
#   bash record_walk.sh --status   is a walk recording running? how big is it?
#   bash record_walk.sh --stop     stop a recording left running (e.g. SSH dropped)
#
# Each pass lands in <host team_ws>/walk_bags/walk-<timestamp>/:
#   bag/         the mcap bag (ros2 bag play / ros2 bag info)
#   record.log   recorder output, including the topic list it subscribed to
#   WALK.txt     label, start/stop times, ros2 bag info
#
# The recorder runs detached inside skybeta-stack, so an SSH drop does not end
# the pass; reconnect and run --stop.
set -uo pipefail

CONTAINER="${CONTAINER:-skybeta-stack}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPER_SRC="${HERE}/walk_bag.sh"
HELPER="/tmp/walk_bag.sh"
C_ROOT="/root/team_ws/walk_bags"
REC_PATTERN="bag record -s mcap -o ${C_ROOT}/"

say()  { printf '[walk] %s\n' "$*"; }
warn() { printf '[walk] WARNING: %s\n' "$*" >&2; }
die()  { printf '[walk] ERROR: %s\n' "$*" >&2; exit 1; }

usage() { sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

in_stack() { docker exec "${CONTAINER}" "$@"; }

require_stack() {
  command -v docker >/dev/null || die "docker not found: run this on the Jetson host, not inside a container."
  [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null)" == "true" ]] \
    || die "${CONTAINER} is not running. Start it first: ~/skybeta/host/run.sh stack -d"
  HOST_TEAM_WS="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/root/team_ws"}}{{.Source}}{{end}}{{end}}' "${CONTAINER}")"
  [[ -n "${HOST_TEAM_WS}" ]] || die "${CONTAINER} has no /root/team_ws mount; cannot place bags on the host."
  HOST_ROOT="${HOST_TEAM_WS}/walk_bags"
  [[ -f "${HELPER_SRC}" ]] || die "missing ${HELPER_SRC}"
  docker cp "${HELPER_SRC}" "${CONTAINER}:${HELPER}" >/dev/null || die "could not copy walk_bag.sh into ${CONTAINER}"
}

# Container-side bag dir of the recording in progress, or nothing.
running_bag() {
  in_stack pgrep -af -- "${REC_PATTERN}" 2>/dev/null \
    | sed -n 's|.* -o \(/root/team_ws/walk_bags/[^ ]*\).*|\1|p' | head -n1
}

is_recording() { in_stack pgrep -f -- "bag record -s mcap -o $1" >/dev/null 2>&1; }

host_path() { printf '%s%s' "${HOST_TEAM_WS}" "${1#/root/team_ws}"; }

stop_recorder() {
  local cbag="$1" i
  say "stopping recorder (SIGINT, clean mcap close) ..."
  in_stack pkill -INT -f -- "bag record -s mcap -o ${cbag}" >/dev/null 2>&1 || true
  for i in $(seq 1 60); do
    is_recording "${cbag}" || { say "recorder stopped."; return 0; }
    sleep 1
  done
  warn "recorder still running after 60 s; leaving it alone. Check $(host_path "$(dirname "${cbag}")")/record.log"
  return 1
}

finalize() {
  local cbag="$1" csession hsession walk start_epoch stop_epoch
  csession="$(dirname "${cbag}")"
  hsession="$(host_path "${csession}")"
  walk="${hsession}/WALK.txt"
  in_stack chown -R "$(id -u):$(id -g)" "${csession}" >/dev/null 2>&1 || true

  stop_epoch="$(date +%s)"
  start_epoch="$(sed -n 's/^start_epoch: //p' "${walk}" 2>/dev/null | head -n1)"
  {
    echo "stop:        $(date -Is)"
    [[ -n "${start_epoch}" ]] && echo "duration_s:  $((stop_epoch - start_epoch))"
    echo
    echo "== ros2 bag info =="
    in_stack bash "${HELPER}" info "${cbag}" 2>&1 || echo "(ros2 bag info failed; see record.log)"
  } >> "${walk}"

  echo
  sed -n '/== ros2 bag info ==/,$p' "${walk}"
  echo
  say "saved ${hsession} ($(du -sh "${hsession}" 2>/dev/null | cut -f1))"
  say "check above that /oak/left/image_raw (or image_mono) and /fmu/out/vehicle_local_position_v1 have Count > 0."
}

cmd_status() {
  require_stack
  local cbag; cbag="$(running_bag)"
  if [[ -z "${cbag}" ]]; then say "no walk recording running."; return 0; fi
  say "recording into $(host_path "${cbag}") ($(du -sh "$(host_path "${cbag}")" 2>/dev/null | cut -f1))"
}

cmd_stop() {
  require_stack
  local cbag; cbag="$(running_bag)"
  [[ -n "${cbag}" ]] || { say "no walk recording running."; return 0; }
  stop_recorder "${cbag}" && finalize "${cbag}"
}

preflight() {
  local topics fmu_count free_gb rate
  say "checking live topics in ${CONTAINER} ..."
  topics="$(in_stack bash "${HELPER}" topics 2>/dev/null || true)"
  [[ -n "${topics}" ]] || die "ros2 topic list returned nothing; is the stack healthy? (~/skybeta/host/run.sh doctor)"
  for t in /oak/left/image_raw /oak/right/image_raw /oak/imu/data /visual_slam/tracking/odometry /skybeta/state; do
    grep -qx -- "${t}" <<<"${topics}" && say "  ok       ${t}" || warn "  missing  ${t}"
  done
  fmu_count="$(grep -c '^/fmu/out/' <<<"${topics}" || true)"
  if [[ "${fmu_count}" -gt 0 ]]; then
    say "  ok       ${fmu_count} /fmu/out/* topics (Pixhawk link up)"
  else
    warn "  no /fmu/out/* topics: this pass would have NO Pixhawk data. Check the flight controller is powered and the agent is up."
  fi

  if [[ "${EYES}" == "mono" ]]; then rate="~40 MB/s (~2.4 GB/min)"; else rate="~120 MB/s (~7 GB/min)"; fi
  mkdir -p "${HOST_ROOT}"
  free_gb="$(df -BG --output=avail "${HOST_ROOT}" | tail -n1 | tr -dc '0-9')"
  say "disk: ${free_gb} GB free under ${HOST_ROOT}; eyes=${EYES} records ${rate}."
  [[ "${free_gb:-0}" -ge 20 ]] || warn "less than 20 GB free; keep passes short or use --mono."
}

cmd_record() {
  require_stack
  local existing; existing="$(running_bag)"
  [[ -z "${existing}" ]] || die "a walk recording is already running into $(host_path "${existing}"). Stop it first: bash $0 --stop"

  preflight
  echo
  local label ts csession cbag hsession walk
  read -r -p "[walk] label for this pass (e.g. 'lap1 slow, pad on left'): " label
  read -r -p "[walk] press Enter to START recording ... " _

  ts="$(date +%Y%m%d-%H%M%S)"
  csession="${C_ROOT}/walk-${ts}"
  cbag="${csession}/bag"
  hsession="${HOST_ROOT}/walk-${ts}"
  walk="${hsession}/WALK.txt"
  mkdir -p "${hsession}"
  {
    echo "label:       ${label}"
    echo "host:        $(hostname)"
    echo "eyes:        ${EYES}"
    echo "start:       $(date -Is)"
    echo "start_epoch: $(date +%s)"
  } > "${walk}"

  docker exec -d "${CONTAINER}" bash -c "exec bash '${HELPER}' record '${cbag}' '${EYES}' > '${csession}/record.log' 2>&1" \
    || die "could not start the recorder in ${CONTAINER}"
  sleep 4
  if ! is_recording "${cbag}"; then
    warn "recorder exited right away; record.log says:"
    tail -n 20 "${hsession}/record.log" >&2 || true
    exit 1
  fi
  say "RECORDING into ${hsession}"
  say "hold the drone steady for a few seconds, then walk the course."
  say "press Enter (or Ctrl-C) to STOP."

  local stop_requested=0 rc started now
  trap 'stop_requested=1' INT
  started="$(date +%s)"
  while :; do
    read -r -t 15 _
    rc=$?
    [[ ${rc} -eq 0 || ${stop_requested} -eq 1 ]] && break
    if [[ ${rc} -le 128 ]]; then warn "stdin closed; stopping."; break; fi
    if ! is_recording "${cbag}"; then warn "recorder stopped on its own; see record.log"; break; fi
    now="$(date +%s)"
    say "  $((now - started)) s, $(du -sh "${hsession}/bag" 2>/dev/null | cut -f1)  (Enter to stop)"
  done
  trap - INT

  stop_recorder "${cbag}" || true
  finalize "${cbag}"
}

EYES="raw"
MODE="record"
for arg in "$@"; do
  case "${arg}" in
    --mono)    EYES="mono" ;;
    --status)  MODE="status" ;;
    --stop)    MODE="stop" ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: ${arg}" ;;
  esac
done

case "${MODE}" in
  status) cmd_status ;;
  stop)   cmd_stop ;;
  *)      cmd_record ;;
esac
