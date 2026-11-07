# field_tests: walks and wall flights, scored against things that do not move

> **WARNING (2026-11-07): F1 climbed nearly to the ceiling on its first flight.** Read
> `team_ws/F1_CLIMB_INCIDENT.md` before any autonomous flight. Likely cause: VIO-frame height
> targets sent to PX4 without `anchor_setpoints`. `ft.sh fly` now refuses unless given
> `--ack-f1-incident`.

## TEST DAY: the short version

```
ssh dronehacks@192.168.68.92            # Wi-Fi (laptop on "DroneHacks Wifi"); password: the organizers' drone login (not stored in git)
#   or, USB cable to the Jetson:  ssh dronehacks@192.168.55.1
~/team_ws/field_tests/day.sh             # walks you through every step, in order
```
Kill at any time: RC kill switch (instant), or type `k` in day.sh, or from a second terminal
`~/team_ws/field_tests/ft.sh kill`. After a battery swap: `day.sh --from F3` (any step name:
STATUS WATCH F1R F1 F0 F2 F3 F4 F5 F6 F7).

If the Wi-Fi address does not answer: plug in USB, `ssh dronehacks@192.168.55.1`, then
`ip -4 -br addr show wlP1p1s0` shows the address (`sudo nmcli con up skybeta-wifi` if it is
not on the Wi-Fi). After the drone first gets internet its clock jumps and the stack restarts
itself (~2 min): wait for `ft.sh status` to show VIO=tracking before flying.

Everything here is **separate** from the flight stack. `wall_logger.py` only listens (it
publishes nothing); `ft.sh` starts/stops it and calls the mission's own start/kill scripts.
Lives at `~/team_ws/field_tests/` on the drone (outside `src/`, so the boot build ignores it).

```
ssh dronehacks@192.168.55.1          # USB, or the drone's Wi-Fi IP
cd ~/team_ws/field_tests
./ft.sh status
```

## How the scoring works (why one config per walk is enough)

You never compare two walks to each other directly. Each walk is scored against **truth**,
and then the scores are compared:

| Truth | Gives you |
|---|---|
| **Start mark** (tape on the floor, same heading every time) | closure error: where VIO thinks you ended vs. where you really ended |
| **The wall** (it never moves) | *wall drift*: the wall's position in VIO coordinates should stay fixed. Any change = VIO error |
| **Tape marks** (only W1/W2) | depth accuracy, and whether VIO measures distances short or long |

Hold the drone **still for 3 s** at every mark. The analyser finds those pauses by itself.

## 0. Before anything: camera mode

```
./ft.sh camera new        # 640x400 + depth (measured better on every number). ~1 min, disarmed
./ft.sh camera status
./ft.sh camera old        # back to 1024x640, no depth
```

## Ground check: motors on / off, props OFF (do it before the first flight of the day)

Battery in, props off, people clear.

```
./ft.sh watch              # records every arm / disarm / mode change while the pilot does:
```
1. **Arm** on the RC (Altitude): all four motors spin at idle.
2. Wait 5 s, **disarm**: motors stop.
3. Arm, flip the **kill switch**: motors stop at once; PX4 disarms a few seconds later.
4. Kill switch back, arm, then `./ft.sh kill` on the Jetson: PX4 goes to **Land** and disarms by itself.

**Each motor on its own** (order and spin direction): QGroundControl -> Vehicle Setup -> Motors,
USB cable from the laptop to the Pixhawk. This PX4 build does not accept motor tests from the
Jetson (tested: `DO_MOTOR_TEST` -> UNSUPPORTED, `ACTUATOR_TEST` -> ignored), so `./ft.sh motors`
just says this and runs `watch`.

## A. Walks, props off (practice wall now, real course when you get it)

```
./ft.sh walk W2 try1      # starts logging and prints what to do
...walk it...
./ft.sh stop              # stops + prints the report (report.txt, path.png, wall.png)
```

| Test | Do this | You learn |
|---|---|---|
| **W0** still | drone still 60 s facing the wall | noise floor |
| **W1** depth accuracy | stand at tape marks 0.5 / 1 / 1.5 / 2 / 3 m, square to the wall, 3 s each. Again turned ~30 deg. Again on each wall colour. `ft.sh stop --truth 0.5,1,1.5,2,3` | can depth be trusted, at what range, on which colours |
| **W2** approach | from 3 m walk straight at the wall, 3 s at 2.5 / 2 / 1.5 / 1 m, walk back out | does VIO measure distance short/long (bench said ~12 % short) |
| **W3** turn | 1.5 m from the wall: still, 90 left, back, 90 right, back, 180, back. 3 s at each | **the sim's failure**: does position jump when turning? |
| **W4** parallel | 1 m from the wall facing it, side-step 3-4 m along it and back | sideways drift |
| **W5** lose and find | facing the wall on the start mark; walk a loop with the wall out of view; return | what a wall correction would fix after losing sight of it |
| **C1, C2, C3** real course | start mark, power on and wait 60 s still; carry at ~1 m, level, facing travel, slow; 3 s at every corner; end on the start mark. **Same start mark and heading every time** | drift over the real route, **a map of where the walls are**, how consistent it is |

Do 2-3 repeats of each. Then compare:
```
./ft.sh compare C1 C2 C3          # table + compare.png (walls from all runs overlaid)
```
On the course walk, the camera is surveying the walls: `path.png` shows the walls as grey
dots in VIO coordinates (forward/left from the start mark). If the three walks put each wall
in the same place, that map is good enough to plan flights against.

## B. Flights at the practice wall, props on

Order matters: **do not move on until the previous report has no unexplained flags.**
Pilot on the RC with the kill switch the whole time. Never closer than 1 m to the wall.

```
./ft.sh list-flights               # where to put the drone for each
./ft.sh fly F2                     # loads the test + starts logging; prints PLACE / WATCH
#   pilot: ARM (Altitude) -> flip to POSITION  => it takes off and flies the test
./ft.sh done                       # after it lands and disarms: report
./ft.sh kill                       # ANY time: lands now + stops the mission (RC kill switch is the instant one)
```

| # | Test | Drone placed | What the report shows |
|---|---|---|---|
| F0 | rehearsal, **props off** | anywhere | the start + kill sequence works (do this first, every day) |
| F1 | take off, 3 s, land | 2 m from the wall, facing it | the whole chain: Offboard, takeoff, landing |
| F2 | hover 10 s | 2 m, facing the wall | wall drift while hovering; VIO vs PX4 agreement |
| F3 | turns +-45 deg | 2 m, facing the wall | wall drift after each turn back |
| F4 | turns +-90 deg | 2 m, facing the wall | turn jump (the sim's main failure) |
| F5 | 1 m toward the wall and back | 2.5 m, facing the wall | distance scale in flight |
| F6 | 0.75 m sideways and back | 2 m, facing the wall, 1 m clear on the left | sideways drift |
| F7 | 0.7 m box, turning at corners | 2.5 m, facing the wall, 1 m clear on the right | moving + turning together |

Options pass through: `./ft.sh fly F2 --alt 1.2`, `./ft.sh fly F5 --speed 0.2`, `--bag` to also
record the camera bag (large). In F1-F7 the drone flies on **VIO only**; depth is only recorded
and used to check VIO. Acting on depth (stop at 1.0 m from the wall) is the next step, once
F2-F5 look clean.

## Get the results onto the laptop

```
scp -r dronehacks@192.168.55.1:~/team_ws/field_tests/runs ./field_runs
```

## Files

| File | Runs | What |
|---|---|---|
| `ft.sh` | drone host | every command above |
| `wall_logger.py` | stack container | VIO pose, wall plane (distance, angle, position in VIO frame), 48-ray depth scan, PX4 state |
| `analyze.py` | stack container | report.txt, summary.json, path.png, wall.png; `--compare` |
| `runs/<time>_<test>_<name>/` | | log.csv, odom.csv, scan.csv, meta.json, DONE, report |
