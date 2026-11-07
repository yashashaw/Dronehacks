#!/bin/bash
# Pre-flight GO / NO-GO for drone 7. Run from the laptop (Git Bash), drone on the launch mark,
# Pixhawk on battery, props on, DISARMED. See loc_stack/PREFLIGHT.md.
#
#   ./preflight.sh                 check only (read-only), ~20 s window with the drone still
#   ./preflight.sh --fix           first make it ready: stop stray recorders and, if the slam
#                                  pipeline was restarted on its own or depth is on, do the
#                                  documented FULL stack restart, wait for every stage, then check
#   ./preflight.sh --lift          rangefinder check: lift the drone ~0.5 m and back down
#   ./preflight.sh --record 240    after a GO, start the flight recorder (walk_logger) for 240 s
#   ./preflight.sh --wait          cold-boot test: re-run until not WAIT, shows time since boot
#   options: --seconds N (check window, default 20)  --profile stock|depth (default stock)
#   DRONE=dronehacks@<wifi-ip> ./preflight.sh   to use WiFi instead of USB-C
#
# Exit code: 0 GO, 2 WAIT (not ready yet, re-run), 1 NO-GO. Never arms, never commands PX4.
DRONE=${DRONE:-dronehacks@192.168.55.1}
HERE=$(cd "$(dirname "$0")" && pwd)
PF='~/team_ws/loc_stack/preflight'                 # host side
CPF=/root/team_ws/loc_stack/preflight              # same folder in the container
ENVSRC='source /opt/ros/*/setup.bash; source /usr/local/bin/skybeta_env.sh'
SECS=20; PROFILE=stock; FIX=0; LIFT=0; RECORD=0; WAITMODE=0; PASSARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --wait) WAITMODE=1; shift; continue ;;
  esac
  PASSARGS+=("$1")
  case "$1" in
    --record|--seconds|--profile) PASSARGS+=("$2") ;;
  esac
  case "$1" in
    --fix) FIX=1; shift ;;
    --lift) LIFT=1; shift ;;
    --record) RECORD=${2:?seconds}; shift 2 ;;
    --seconds) SECS=${2:?}; shift 2 ;;
    --profile) PROFILE=${2:?}; shift 2 ;;
    -h|--help) sed -n 2,17p "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done

# --wait: for cold-boot tests. Re-run while the verdict is WAIT (every 15 s, up to 10 min);
# each report shows the Jetson's uptime, so the time from power-on to GO can be read off.
if [ $WAITMODE -eq 1 ]; then
  W0=$SECONDS
  until ping -n 1 -w 1000 192.168.55.1 >/dev/null 2>&1 || ping -c 1 -W 1 192.168.55.1 >/dev/null 2>&1 \
        || [ "${DRONE#*@}" != 192.168.55.1 ]; do
    (( SECONDS - W0 > 600 )) && { echo "drone not reachable after 10 min"; exit 1; }
    printf '[%s] waiting for the drone to answer...\n' "$(date +%H:%M:%S)"; sleep 5
  done
  while :; do
    bash "$HERE/$(basename "$0")" "${PASSARGS[@]}"; code=$?
    [ $code -ne 2 ] && exit $code
    (( SECONDS - W0 > 600 )) && { echo "still WAIT after 10 min"; exit 2; }
    printf '[%s] WAIT: checking again in 15 s (%d s since --wait started)\n' "$(date +%H:%M:%S)" $((SECONDS - W0))
    sleep 15
  done
fi

retry() {  # the USB link drops connections; retry only connection failures (rc 255)
  for i in 1 2 3 4 5 6; do
    "$@" && return 0
    rc=$?; [ $rc -ne 255 ] && return $rc; sleep 2
  done; return 255
}
r() { retry ssh -o BatchMode=yes -o ConnectTimeout=10 "$DRONE" "$@"; }
say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
T0=$SECONDS
el() { echo "$((SECONDS - T0)) s"; }

sidecar() {  # one field from the status sidecar's summary, e.g. sidecar fc.link
  r "docker exec skybeta-ros-status cat /out/ros_summary.json 2>/dev/null | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
    for k in \"$1\".split(\".\"): d = d.get(k) if isinstance(d, dict) else None
    print(d)
except Exception: print(None)'" 2>/dev/null
}

read -r -d '' HOSTPY <<'PY'
import json, os, re, shutil, subprocess as sp, time
def sh(c):
    try: return sp.run(c, shell=True, capture_output=True, text=True, timeout=15).stdout.strip()
    except Exception: return ''
h = {}
h['ntp_synced'] = sh('timedatectl show -p NTPSynchronized --value') == 'yes'
h['has_network'] = bool(sh('ip route show default'))
h['nproc'] = os.cpu_count()
h['uptime_s'] = float(open('/proc/uptime').read().split()[0])
m = re.search(r'Power Mode:\s*(\S+)', sh('nvpmodel -q 2>/dev/null'))
h['power_mode'] = m.group(1) if m else ''
h['stack_running'] = sh("docker inspect -f '{{.State.Running}}' skybeta-stack") == 'true'
started = sh("docker inspect -f '{{.State.StartedAt}}' skybeta-stack")
if started:
    t = sh(f"date -d '{started}' +%s")
    h['stack_up_s'] = time.time() - int(t) if t.isdigit() else None
panes = {}
for line in sh("docker exec skybeta-stack tmux list-panes -a -F '#W #{pane_dead}'").splitlines():
    w, _, dead = line.partition(' ')
    panes[w] = dead.strip() == '0'
h['panes'] = panes
h['slam_cmd'] = sh("docker exec skybeta-stack tmux display -p -t skybeta:slam '#{pane_start_command}'")
procs = sh("docker exec skybeta-stack ps -eo args")
h['stray_procs'] = sorted({n for n in ('depth_video.py', 'depth_wall.py', 'walk_logger.py',
                                       'stereo_probe.py') if n in procs})
# Profile the RUNNING slam pipeline uses (run_slam.sh launches /tmp/skybeta_depth/depth.launch.py
# in depth mode). The profile file only says what the NEXT start will use.
h['running_profile'] = 'depth' if 'skybeta_depth/depth.launch.py' in procs else 'stock'
du = shutil.disk_usage(os.path.expanduser('~'))
h['disk_free_gb'] = round(du.free / 1e9, 1)
did = re.search(r'^\s*DRONE_ID=(\d+)', open('/etc/skybeta/drone.env').read(), re.M) \
    if os.path.exists('/etc/skybeta/drone.env') else None
h['drone_id'] = int(did.group(1)) if did else None
prof = None
pf = os.path.expanduser('~/team_ws/slam_profile')
if os.path.exists(pf):
    words = open(pf).read().split()
    prof = words[0] if words else None
if prof is None and os.path.exists('/etc/skybeta/drone.env'):
    m2 = re.search(r'^\s*SKYBETA_SLAM_PROFILE=(\w+)', open('/etc/skybeta/drone.env').read(), re.M)
    prof = m2.group(1) if m2 else None
h['slam_profile'] = prof or 'stock'
print(json.dumps(h))
PY

host_facts() {  # writes ~/team_ws/loc_stack/preflight/host.json on the Jetson and prints it
  # The script goes over stdin, so each attempt re-sends it (a blind retry would send nothing).
  for i in 1 2 3 4 5; do
    ssh -o BatchMode=yes -o ConnectTimeout=10 "$DRONE"       "mkdir -p $PF && python3 - > $PF/host.json && cat $PF/host.json" <<<"$HOSTPY" && return 0
    sleep 2
  done
  return 1
}

# ------------------------------------------------------------------ reach + deploy
r true || { echo "drone unreachable over SSH ($DRONE). USB-C plugged in? ping 192.168.55.1"; exit 1; }
for f in preflight_rules.py preflight_check.py; do
  retry scp -q -o BatchMode=yes -o ConnectTimeout=10 "$HERE/$f" "$DRONE:team_ws/loc_stack/$f" \
    || { echo "could not copy $f"; exit 1; }
done
say "collecting Jetson facts"
HOST=$(host_facts) || { echo "could not read Jetson state"; exit 1; }
field() { python -c "import json,sys; d=json.loads(sys.argv[1]); print(d.get('$1'))" "$HOST"; }
DRONE_ID=$(field drone_id); [ "$DRONE_ID" = None ] && DRONE_ID=7

# ------------------------------------------------------------------ --fix
if [ $FIX -eq 1 ]; then
  armed=$(sidecar fc.armed)
  [ "$armed" = True ] && { echo "REFUSING --fix: the drone is ARMED. Disarm first."; exit 1; }
  stray=$(field stray_procs)
  if [ "$stray" != "[]" ]; then
    say "stopping stray recorders: $stray"
    r "docker exec skybeta-stack pkill -INT -f 'loc_stack/(depth_video|depth_wall|walk_logger)\.py'" || true
  fi
  slam_cmd=$(field slam_cmd); panes=$(field panes); running=$(field running_profile)
  if [ -n "$slam_cmd" ] || grep -q False <<<"$panes" || [ "$running" != "$PROFILE" ]; then
    echo
    echo ">>> FULL STACK RESTART needed (slam pane: '${slam_cmd:-ok}', panes: $panes, running profile: $running, wanted: $PROFILE)."
    echo ">>> Put the drone ON THE LAUNCH MARK now and do not touch it until this finishes:"
    echo ">>> the camera origin and the flight interface's altitude zero are set during start-up."
    read -r -p ">>> Enter to restart (Ctrl-C to cancel): " _
    T0=$SECONDS
    say "docker rm -f skybeta-stack (systemd restarts the whole stack)"
    r "docker rm -f skybeta-stack >/dev/null" || { echo "restart failed"; exit 1; }
    ok=0
    for i in $(seq 1 80); do
      sleep 3
      n=$(r "docker exec skybeta-stack tmux list-panes -a -F '#W #{pane_dead}' 2>/dev/null | grep -c ' 0$'" 2>/dev/null)
      [ "${n:-0}" -ge 4 ] && { ok=1; break; }
    done
    [ $ok -eq 1 ] || { echo "stack did not come back in 240 s: journalctl -u skybeta-stack -n 50"; exit 1; }
    say "stack windows up after $(el)"
    ok=0
    for i in $(seq 1 30); do sleep 3; [ "$(sidecar fc.link)" = True ] && { ok=1; break; }; done
    if [ $ok -eq 0 ]; then
      echo ">>> NO PIXHAWK LINK 90 s after the restart. On stock PX4 v1.16.2 a stack restart wedges"
      echo ">>> the Pixhawk's TELEM2 until it is power-cycled. Power-cycle the WHOLE drone (battery"
      echo ">>> out and back in) with it on the launch mark, then run ./preflight.sh again (no --fix)."
      exit 1
    fi
    say "Pixhawk link after $(el)"
    ok=0
    for i in $(seq 1 100); do
      sleep 3
      hz=$(sidecar vio.odom_rate_hz)
      python -c "import sys; sys.exit(0 if float(sys.argv[1]) >= 25 else 1)" "$hz" 2>/dev/null && { ok=1; break; }
    done
    [ $ok -eq 1 ] || { echo "VIO odometry not flowing 5 min after restart: tmux attach and look at the slam window"; exit 1; }
    say "VIO odometry at ${hz} Hz after $(el)"
    say "holding still 75 s so VIO and EKF2 settle (ground3 procedure)"
    for i in 75 60 45 30 15; do printf '  %s s\n' "$i"; sleep 15; done
    say "ready after $(el) in total; re-reading Jetson facts"
    HOST=$(host_facts) || exit 1
  fi
fi

# ------------------------------------------------------------------ --lift
if [ $LIFT -eq 1 ]; then
  read -r -p ">>> Lift check: when you press Enter, slowly LIFT the drone straight up ~0.5 m, hold 2 s, put it back (15 s). Enter: " _
  r "docker exec skybeta-stack rm -rf $CPF/lift" >/dev/null
  r "docker exec -d skybeta-stack bash -lc '$ENVSRC; python3 /root/team_ws/loc_stack/preflight_check.py --mode lift --seconds 15 --out $CPF/lift'" || exit 1
  say "lift now"
  for i in $(seq 1 45); do sleep 1; r "test -f $PF/lift/DONE" 2>/dev/null && break; done
  r "cat $PF/lift/report.txt" || echo "no lift result"
  exit 0
fi

# ------------------------------------------------------------------ the check
say "measuring for ${SECS} s: keep the drone STILL on the launch mark"
r "docker exec skybeta-stack rm -rf $CPF/out" >/dev/null
r "docker exec -d skybeta-stack bash -lc '$ENVSRC; DRONE_ID=$DRONE_ID python3 /root/team_ws/loc_stack/preflight_check.py --seconds $SECS --profile $PROFILE --host-json $CPF/host.json --out $CPF/out'" \
  || { echo "could not start the check"; exit 1; }
for i in $(seq 1 $((SECS + 60))); do sleep 1; r "test -f $PF/out/DONE" 2>/dev/null && break; done
STAMP=$(date +%Y%m%d_%H%M%S)
LOCAL="$HERE/runs/preflight/$STAMP"; mkdir -p "$LOCAL"
for f in report.txt result.json DONE; do
  retry scp -q -o BatchMode=yes -o ConnectTimeout=10 "$DRONE:team_ws/loc_stack/preflight/out/$f" "$LOCAL/" 2>/dev/null
done
[ -f "$LOCAL/report.txt" ] || { echo "no report came back (see $PF/out on the drone)"; exit 1; }
echo; cat "$LOCAL/report.txt"; echo "(saved: $LOCAL)"
CODE=$(tr -d '[:space:]' < "$LOCAL/DONE" 2>/dev/null); CODE=${CODE:-1}

if [ "$CODE" = 0 ] && [ "$RECORD" -gt 0 ]; then
  NAME=flight_$STAMP
  r "docker exec skybeta-stack tmux display -p -t skybeta:slam '#{pane_start_command}' > /tmp/$NAME.cfg; \
     docker exec -d skybeta-stack bash -lc '$ENVSRC; python3 /root/team_ws/loc_stack/walk_logger.py --seconds $RECORD --out /root/team_ws/walk_runs/$NAME'" \
    && say "flight recorder running for $RECORD s: walk_runs/$NAME (pull later: ./run_walk.sh pull $NAME)"
fi
exit "$CODE"
