#!/usr/bin/env bash
# Copy drone_tests/ to ~/team_ws/drone_tests on a drone. Run ON THE LAPTOP (Git Bash), from the repo:
#
#   drone_tests/install.sh skybeta-18            # uses the `skybeta-18` alias in ~/.ssh/config (USB)
#   drone_tests/install.sh skybeta-18 skybeta-18-wifi
#   drone_tests/install.sh skybeta-18 --boot     # ALSO the boot package -> ~/team_ws/src/px4_square
#
# The first argument is the hostname you EXPECT; it is checked before anything is copied
# (logging in to / writing to another team's drone gets you disqualified).
# It copies files only: nothing arms, restarts or touches PX4.
# ~/team_ws/drone_tests is outside ~/team_ws/src on purpose: the boot build of team_ws ignores it,
# so nothing here can break the competitor launch (drone_tests/COLCON_IGNORE makes sure).
# --boot also copies team_src/px4_square into ~/team_ws/src (and removes drone_mission);
# it only runs at boot once drone.env says COMPETITOR_PKG=px4_square.
set -euo pipefail

BOOT=0; ARGS=()
for a in "$@"; do [[ "$a" == --boot ]] && BOOT=1 || ARGS+=("$a"); done
EXPECT="${ARGS[0]:-}"; HOST="${ARGS[1]:-${ARGS[0]:-}}"
[[ "$EXPECT" =~ ^skybeta-[0-9]{2}$ ]] || { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH=(ssh -o ConnectTimeout=8)

got="$("${SSH[@]}" "$HOST" hostname)"
[[ "$got" == "$EXPECT" ]] || { echo "[install] REFUSING: $HOST says it is '$got', not '$EXPECT'." >&2; exit 1; }
echo "[install] $HOST is $got"

( cd "$HERE/.." && tar --exclude='__pycache__' -cf - drone_tests ) \
  | "${SSH[@]}" "$HOST" 'mkdir -p ~/team_ws && cd ~/team_ws && tar -xmf -'

"${SSH[@]}" "$HOST" 'set -e; cd ~/team_ws/drone_tests
  sed -i "s/\r$//" *.sh *.py *.md *.patch
  chmod +x *.sh *.py
  for f in *.sh; do bash -n "$f"; done
  for f in *.py; do python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read())" "$f"; done
  echo "[install] ok: $(ls | tr "\n" " ")"'
if [[ "$BOOT" == 1 ]]; then
  # px4_square: the example square on PX4 position only (no VIO height, no depth).
  # drone_mission (hover.py, needs depth) is REMOVED from ~/team_ws/src: no depth on this drone.
  ( cd "$HERE/../team_src" && tar -cf - px4_square ) \
    |"${SSH[@]}" "$HOST" 'set -e; mkdir -p ~/team_ws/src && cd ~/team_ws/src && rm -rf px4_square drone_mission && tar -xmf -
      find px4_square -type f -exec sed -i "s/\r$//" {} +
      python3 -c "import ast,sys; [ast.parse(open(f).read()) for f in sys.argv[1:]]" px4_square/setup.py px4_square/launch/*.py px4_square/px4_square/*.py
      echo "[install] boot package: ~/team_ws/src/px4_square (drone_mission removed)"; grep -E "^(COMPETITOR_|SKYBETA_SLAM)" /etc/skybeta/drone.env || true'
  echo "[install] for boot: drone.env needs COMPETITOR_PKG=px4_square COMPETITOR_LAUNCH=px4_square.launch.py and NO SKYBETA_SLAM_PROFILE (README step 5)"
fi
echo "[install] next:  ssh $HOST  then  ~/team_ws/drone_tests/dt.sh status"
