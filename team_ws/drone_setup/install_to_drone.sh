#!/usr/bin/env bash
# Install the team packages + field-test kit on a drone. Run ON THE LAPTOP (Git Bash / Linux /
# macOS), from anywhere inside this repo:
#
#   team_ws/drone_setup/install_to_drone.sh skybeta-07 192.168.55.1      # USB
#   team_ws/drone_setup/install_to_drone.sh skybeta-12 192.168.68.92     # Wi-Fi
#
# The first argument is the hostname you EXPECT. It is checked before anything is copied:
# logging in to another team's drone gets you disqualified, and so does writing to one.
#
# What it does (nothing here arms, flies, restarts the stack or touches PX4):
#   1. checks `hostname` on the target == the name you gave
#   2. copies team_ws/src/team_flight_test, team_ws/src/team_takeoff_land and
#      team_ws/field_tests to ~/team_ws/ on the drone
#   3. strips Windows line endings (CRLF) there and makes the scripts executable
#   4. syntax-checks every script on the drone
# What it does NOT do (each is a separate, deliberate step: see drone_setup/README.md):
#   * point drone.env at our launcher / turn autostart on or off   (needs sudo)
#   * switch the camera to 640x400 + depth                          (needs the run_slam patch)
#
# READ team_ws/F1_CLIMB_INCIDENT.md BEFORE FLYING ANYTHING.
set -euo pipefail

EXPECT="${1:-}"; HOST="${2:-}"
[[ -n "$EXPECT" && -n "$HOST" ]] || { sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ "$EXPECT" =~ ^skybeta-[0-9]{2}$ ]] || { echo "expected hostname must look like skybeta-07" >&2; exit 2; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEAM_WS="$(cd "$HERE/.." && pwd)"
T="dronehacks@$HOST"
SSH=(ssh -o ConnectTimeout=8)

echo "[install] checking $HOST is $EXPECT ..."
got="$("${SSH[@]}" "$T" hostname)"
[[ "$got" == "$EXPECT" ]] || { echo "[install] REFUSING: $HOST says it is '$got', not '$EXPECT'." >&2; exit 1; }
echo "[install] ok: $got"

echo "[install] copying packages + field_tests ..."
( cd "$TEAM_WS" && tar --exclude='__pycache__' --exclude='field_tests/runs' -cf - \
    src/team_flight_test src/team_takeoff_land field_tests ) \
  | "${SSH[@]}" "$T" 'mkdir -p ~/team_ws && cd ~/team_ws && tar -xmf - && mkdir -p field_tests/runs'

echo "[install] fixing line endings + permissions, syntax check ..."
"${SSH[@]}" "$T" 'set -e; cd ~/team_ws
  find src/team_flight_test src/team_takeoff_land field_tests -type f \
       \( -name "*.sh" -o -name "*.py" -o -name "*.cfg" -o -name "*.xml" -o -name "*.md" \) \
       -not -path "*/runs/*" -exec sed -i "s/\r$//" {} +
  chmod +x src/team_flight_test/scripts/*.sh src/team_takeoff_land/scripts/*.sh field_tests/*.sh
  bad=0
  for f in src/team_flight_test/scripts/*.sh src/team_takeoff_land/scripts/*.sh field_tests/*.sh; do
    bash -n "$f" || { echo "SYNTAX ERROR: $f"; bad=1; }
  done
  for f in field_tests/*.py src/team_flight_test/team_flight_test/*.py src/team_takeoff_land/team_takeoff_land/*.py; do
    python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read())" "$f" || { echo "PY ERROR: $f"; bad=1; }
  done
  left=$(grep -rlI $'"'"'\r'"'"' src/team_flight_test src/team_takeoff_land field_tests --include=*.sh --include=*.py | wc -l)
  echo "files still with CRLF: $left"
  [ $bad -eq 0 ] && [ "$left" -eq 0 ] && echo "[install] scripts OK"'

cat <<EOM

[install] done on $EXPECT. Nothing was started. Next steps are in team_ws/drone_setup/README.md:
  - autostart off + boot slot pointing at our launcher:
      ssh $T '~/team_ws/src/team_flight_test/scripts/autostart.sh disable'   (asks for sudo)
  - camera 640x400 + depth: see README (run_slam patch), then  ~/team_ws/field_tests/ft.sh camera new
  - status:  ssh $T '~/team_ws/field_tests/ft.sh status'
  - READ team_ws/F1_CLIMB_INCIDENT.md before any autonomous flight.
EOM
