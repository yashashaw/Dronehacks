#!/usr/bin/env bash
# THE ONE COMMAND FOR A TEST DAY. Run on the drone:   ~/team_ws/field_tests/day.sh
#
# It walks you through everything in order and waits for you at every step:
#   status check -> ground check (props off) -> F1 rehearsal (props off) -> F1 ->
#   F0 rehearsal (props off) -> F2 F3 F4 F5 F6 F7
# Before each flight it tells you where to put the drone, starts the recording, and waits.
# After each flight it stops the recording and shows the report.
#
# At every prompt:  Enter = go on    s = skip this step    q = quit
# While a flight is running:  k + Enter = LAND NOW (the RC kill switch is always faster)
#
#   day.sh --from F3      start at a later step (e.g. after a battery swap)
set -uo pipefail
FT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ft.sh"
FROM="${2:-}"; [[ "${1:-}" == "--from" ]] || FROM=""

B=$'\e[1m'; R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
title() { printf '\n%s======================================================\n %s\n======================================================%s\n' "$B" "$*" "$N"; }
ask() {   # ask "question" -> returns 0 go, 1 skip ; q quits
  local a
  read -r -p "${Y}$* [Enter=go, s=skip, q=quit] ${N}" a
  case "$a" in q|Q) echo "bye -- run '$0 --from <step>' to continue later"; exit 0 ;; s|S) return 1 ;; *) return 0 ;; esac
}

started=0
[[ -z "$FROM" ]] && started=1
reached() { [[ $started -eq 1 ]] && return 0; [[ "$1" == "$FROM" ]] && started=1; [[ $started -eq 1 ]]; }

# --------------------------------------------------------------------------- flight
fly() {   # id  props(on|off)  where  what
  local id="$1" props="$2" where="$3" what="$4" extra=("${@:5}")
  title "$id  ${what}   (props ${props^^})"
  echo " PLACE:  $where"
  [[ "$props" == off ]] && echo " ${R}PROPS OFF.${N} The drone will not lift; after ~5 s type k to test the kill." \
                        || echo " ${R}PROPS ON. Pilot on the RC, kill switch ready, people clear. Start the phone video.${N}"
  ask "Ready to start the recording for $id?" || { echo "skipped $id"; return; }
  "$FT" fly "$id" "${extra[@]}" || { echo "${R}could not start $id (see above). Fix it, then: $0 --from $id${N}"; exit 1; }
  "$FT" status | grep -q "logger: RUNNING" && echo "${G}recording: OK${N}" || echo "${R}WARNING: recording is NOT running${N}"
  echo
  echo " ${B}PILOT: ARM (Altitude), then flip to POSITION.${N}"
  echo " When it has landed and disarmed: press Enter.     To land it NOW: type k + Enter."
  local a
  while :; do
    read -r -p "${Y}[Enter = landed and disarmed, k = LAND NOW] ${N}" a
    if [[ "$a" == k || "$a" == K ]]; then
      "$FT" kill
      echo "wait until it is on the ground and disarmed, then press Enter"
      read -r _
      "$FT" analyze latest
      return
    fi
    "$FT" done && break
    echo "${R}still armed? wait for touchdown + disarm, then Enter again (or k)${N}"
  done
  if [[ "$id" != F1 ]]; then
    python3 "$HOME/team_ws/src/team_flight_test/scripts/analyze_flight.py" --latest 2>/dev/null | tail -25 || true
  fi
  echo
  echo "${B} Read the Flags section above. Anything you can't explain: stop here and look into it.${N}"
}

# --------------------------------------------------------------------------- steps
if reached STATUS; then
  title "STATUS"
  "$FT" status
  echo
  echo " Need: FC link=yes, armed=no, VIO=tracking, depth topic: yes"
  echo " If depth topic: no ->  $FT camera new   (takes ~2 min, drone still)"
  ask "Does it look right?" || true
fi

if reached WATCH; then
  title "GROUND CHECK   (props OFF, battery in, people clear)"
  echo " Pilot, in order: ARM -> wait 5 s -> DISARM -> ARM -> KILL SWITCH -> kill switch back -> ARM"
  echo " At the last ARM, in a SECOND terminal run:   $FT kill     (it should land + disarm by itself)"
  echo " Each motor alone: QGroundControl -> Vehicle Setup -> Motors (USB to the Pixhawk)."
  if ask "Start watching? (Ctrl-C ends it)"; then
    mkdir -p "$HOME/team_ws/field_tests/runs"
    "$FT" watch --seconds 300 | tee "$HOME/team_ws/field_tests/runs/watch_$(date +%m%d-%H%M).log"
  fi
fi

reached F1R && fly F1 off "anywhere"                                            "F1 REHEARSAL: take off + land"
reached F1  && fly F1 on  "camera ${B}2.0 m${N} from the wall, square to it"    "take off to 1 m, 3 s, land"
reached F0  && fly F0 off "anywhere"                                            "F0 REHEARSAL: the mission F2-F7 use"
reached F2  && fly F2 on  "camera ${B}EXACTLY 2.0 m${N} from the wall (tape it, write it down)" "hover 10 s (+ depth check)" --bag
reached F3  && fly F3 on  "camera 2.0 m from the wall, facing it"               "turns +-45 deg"
reached F4  && fly F4 on  "camera 2.0 m from the wall, facing it"               "turns +-90 deg"
reached F5  && fly F5 on  "camera ${B}2.5 m${N} from the wall, facing it"       "1 m toward the wall and back"
reached F6  && fly F6 on  "camera 2.0 m from the wall, ${B}1.5 m clear on its LEFT${N}"  "0.75 m sideways and back"
reached F7  && fly F7 on  "camera ${B}2.5 m${N} from the wall, ${B}1.5 m clear on its RIGHT${N}" "0.7 m box"

title "DONE"
"$FT" list
echo
echo " Copy everything to the laptop (run this ON THE LAPTOP, not here):"
echo "   scp -r dronehacks@<drone-ip>:~/team_ws/field_tests/runs ./field_runs"
echo "   scp -r dronehacks@<drone-ip>:~/team_ws/flight_logs ./flight_logs"
