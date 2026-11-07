# team_flight_test: test flights from the sim's best run, with logging

The goal for today is data, not distance. Each profile tests one thing. Every
flight writes logs that `analyze_flight.py` turns into numbers.

**Status:** the logic and the analyzer pass 16 offline tests (`python3 test/test_route_logic.py`).
The ROS node, launch file and start script have **not** been run on ROS, in sim
or on the drone yet. The first flight is `hover` for that reason.

## What it is

For what comes after these test flights (how to fix drift with depth), see
[`PLAN_3_VS_4.md`](PLAN_3_VS_4.md).


Based on `sim/waypoint_run_best_run015.py`: turn in place to face the leg, then
slide the setpoint along the line at 0.25 m/s, never more than 0.3 m ahead of
the drone. Changes for the real drone:

| Sim (run_015) | Here |
|---|---|
| arms and requests Offboard itself, publishes `/fmu/in/*` | **never arms**. The pilot arms and switches to Position. `flight_interface` owns Offboard, `takeoff_land` does takeoff and landing |
| PX4 NED frame, waypoints from the course map | VIO odom ENU, route **relative to the hover pose at release** |
| depth brake | **none**: the real stack has no depth topic |
| turn at 25 deg/s, translate right away | 20 deg/s, then a **1 s settle** (in sim the estimate jumped during turns) |
| lands on timer or error | also lands on: VIO stale > 0.5 s, VIO jump > 0.5 m, track error > 0.75 m, altitude outside 0.4–2.0 m, > 1 m outside the route, 60 s |

```
pilot: ARM -> POSITION  ==> flight_interface: Offboard
flight_test --/team/takeoff--> takeoff_land climbs to --alt, hovers
flight_test --/team/release--> flight_test --/team/planner/pose--> takeoff_land --> flight_interface --> PX4
flight_test --/team/land--> takeoff_land: NAV_LAND, disarm
```

## Profiles (run them in this order)

| # | Profile | What it does | What it tells you |
|---|---|---|---|
| 1 | `hover` | hold 10 s | whether the whole chain works (Offboard, takeoff, release, land). Also baseline VIO drift and the VIO vs EKF2 offset |
| 2 | `yaw` | turn to +90, back to 0, to −90, back to 0, with 3 s holds | **the main sim failure**: does the estimate jump or slide during turns? |
| 3 | `out_back` | 1 m forward and back, no turn (stage23) | along-track scale error (the bench showed VIO ~12 % short). Tape-measure the real distance |
| 4 | `box --scale 0.7` | 0.7 m square, turning to face each leg | turns and translation together, without walls |
| 5 | `course --max-legs 1`, then 2, 3… | the run_015 route (start corridor north, then right) | only on the real course, facing "north" at the launch mark |

Do not move to the next profile until the previous one's analysis has no
unexplained flags.

## Copying it to the Jetson

From the laptop (the package needs `team_takeoff_land` next to it):

```bash
scp -r team_ws/src/team_flight_test team_ws/src/team_takeoff_land dronehacks@192.168.55.1:~/team_ws/src/
ssh dronehacks@192.168.55.1 'chmod +x ~/team_ws/src/team_flight_test/scripts/*'
```

## A flight

On the Jetson (ssh in):

```bash
~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --profile hover
#   builds team_ws in the container, checks link/VIO/EKF2, starts a rosbag,
#   restarts ONLY the competitor tmux window (the agent and its Pixhawk link stay up)
docker exec -it skybeta-stack tmux attach -t skybeta   # watch it (Ctrl-b w, Ctrl-b d)
```

1. Leave the drone still for a few seconds so VIO starts cleanly.
2. Pilot **arms**, then flips to **Position**. It takes off to 1 m and flies the profile.
3. Abort at any time: kill switch, or take over with the mode switch / sticks. The node stops publishing.
4. After landing:

```bash
~/team_ws/src/team_flight_test/scripts/start_flight_test.sh --stop     # stop the bag, restore competitor_example
python3 ~/team_ws/src/team_flight_test/scripts/analyze_flight.py --latest
```

Also write down by hand: the real landing offset (tape measure), what the
drone visibly did, and the video timestamp. The analyzer only sees what the
drone believed; that comparison is what finds estimator errors.

Get the logs onto the laptop:
`scp -r dronehacks@192.168.55.1:~/team_ws/flight_logs ./flight_logs`

## Logs

`~/team_ws/flight_logs/<stamp>_<profile>/`: `ticks.csv` (20 Hz: setpoint, VIO
pose, PX4 pose, modes, track error, jumps), `odom.csv` (every VIO message),
`events.log` (every decision and watchdog), `config.json`, `summary.json`
(written by the analyzer). The rosbag goes to `flight_logs/bags/` and has the
camera images, the IMU and all `/fmu/out` topics.

## Options

`--alt 1.0 --speed 0.25 --scale 1.0 --max-legs 0 --yaw-rate 20 --run-s 60 --sysid N --no-bag`.
Speed is limited to 0.5 m/s and yaw rate to 45 deg/s in code.

## Known risks (check before the first flight)

- **VIO frame vs EKF2 frame.** `flight_interface` runs with `anchor_setpoints=false`, so an
  odom-frame setpoint is sent to PX4 as if both frames had the same origin. If they don't,
  the drone moves toward the offset as soon as Offboard starts. Section 5 of the analyzer
  measures this. `hover` shows it first.
- **FC system id.** The launch passes `fc_system_id` (DRONE_ID) to both `flight_interface`
  and `takeoff_land`. If Offboard never engages, check `MAV_SYS_ID` and pass `--sysid`.
- **Stock v1.16.2 firmware:** power-cycle the Pixhawk after any full stack restart (skybeta bug #15).
  The start script restarts only the competitor window, so it shouldn't need one.
- No obstacle sensing. Fly `box` and `course` only with the area in front of each leg checked by eye.
