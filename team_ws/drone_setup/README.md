# Setting up a drone with the team packages and field tests

Branch `drone-field-tests`. Everything from the 2026-11-07 session on drone 7:
`team_ws/SESSION_NOTES_2026-11-07.md` (what was done and measured) and
`team_ws/field_tests/README.md` (the tests).

> **Read `team_ws/F1_CLIMB_INCIDENT.md` first.** F1 climbed nearly to the ceiling on its first
> flight. Do not fly autonomously until that is understood and fixed.

Every drone-side setting from drone 7 (drone.env, autostart switch, camera mode, the running
camera config, camera benchmark configs, measured results): **`drone7_config/`**.

## 0. Get the code

```
git clone https://github.com/yashashaw/Dronehacks.git
cd Dronehacks
git checkout drone-field-tests
```
`.gitattributes` forces Linux line endings on everything that runs on the drone. If you cloned
before this branch existed, re-checkout the files (`git rm --cached -r team_ws && git reset --hard`)
or the scripts fail with `bash\r: No such file or directory`.

## 1. Connect to YOUR drone only

USB-C laptop -> Jetson: `ssh dronehacks@192.168.55.1`. Wi-Fi ("DroneHacks Wifi"): the IP, e.g.
`ssh dronehacks@192.168.68.92` (`skybeta-NN.local` did not resolve on our Windows laptop).
Always check `hostname` first. Find the Wi-Fi IP over USB with `ip -4 -br addr show wlP1p1s0`;
if the drone is not on the Wi-Fi: `sudo nmcli con up skybeta-wifi`.

## 2. Install (from the laptop)

```
team_ws/drone_setup/install_to_drone.sh skybeta-NN 192.168.55.1
```
Copies `team_flight_test`, `team_takeoff_land`, `field_tests`, fixes line endings, syntax-checks.
Starts nothing.

## 3. Boot slot + autostart OFF (on the drone)

```
~/team_ws/src/team_flight_test/scripts/autostart.sh disable      # asks for the sudo password
~/team_ws/src/team_flight_test/scripts/autostart.sh status
```
Points `/etc/skybeta/drone.env` at our launcher (`team_flight_test` / `autostart.launch.py`) with
`enabled=false`, so the example 5 m square never starts at boot. Backup of drone.env is kept.

## 4. Camera 640x400 + depth (optional, measured better on every number)

Needs the depth profile in the drone's skybeta checkout. `skybeta_depth640.patch` (taken from
drone 7, against skybeta 0.3.3 @ 3c42933) adds it to `container_scripts/run_slam.sh`,
`host/run.sh`, `host/drone_env.sh`. From the laptop, then on the drone:
```
scp team_ws/drone_setup/skybeta_depth640.patch dronehacks@<drone>:/tmp/
ssh dronehacks@<drone>
cd ~/skybeta && git apply --check /tmp/skybeta_depth640.patch && git apply /tmp/skybeta_depth640.patch
```
If `--check` fails, the drone's skybeta differs from 0.3.3: do not force it, ask. Then:
```
~/team_ws/field_tests/ft.sh camera new      # restarts the stack, disarmed, ~2 min
~/team_ws/field_tests/ft.sh camera status   # depth ON, 640 wide
```

## 5. Use

```
~/team_ws/field_tests/ft.sh status
~/team_ws/field_tests/day.sh                 # guided test day (flights: see the incident note!)
```
Commands: `ft.sh fly F1..F7`, `ft.sh done`, `ft.sh kill` (lands), `ft.sh walk W1 name`,
`ft.sh stop`, `ft.sh watch` (ground check), `ft.sh analyze latest`, `ft.sh compare ...`.
RC kill switch is always the real emergency stop.
