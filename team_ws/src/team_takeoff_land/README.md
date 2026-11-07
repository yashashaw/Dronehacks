# team_takeoff_land: how a run starts, end to end

This explains what starts what, from power-on to touchdown, and where
`team_takeoff_land` fits. Sources are the skybeta scripts on the drone
(`host/stack_service.sh`, `container_scripts/run_full_stack.sh`,
`container_scripts/run_competitor.sh`) and the flight interface
(`skybeta_flight_interface/auto_offboard.py`).

> **Status:** `logic.py` and `mission_logic.py` are unit-tested (25 tests).
> The ROS nodes, launch file and `start_test.sh` have not been built or run
> on the drone or in sim yet. Test in sim before anything arms.

## Quick start: the takeoff-then-land test (one command)

With the package in `~/team_ws/src` on the drone and the Pixhawk powered:

```bash
ssh dronehacks@192.168.55.1 '~/team_ws/src/team_takeoff_land/scripts/start_test.sh'
#   options: --sysid N  --alt 1.0  --hover 1.0   |   --stop  restores competitor_example
```

It checks the stack, the Pixhawk link, VIO and EKF2, then restarts only the
stack's `competitor` window with `launch/takeoff_land_test.launch.py`
(flight_interface + takeoff_land + takeoff_land_mission; no planner). The
agent is not restarted, so the Pixhawk link survives.

Then the **pilot arms and flips to Position**. That is the start: the drone
takes off to `--alt`, holds `--hover` seconds, lands and disarms. Nothing
arms by software. Abort with the kill switch or by taking over on RC. If it
has not started landing 30 s after takeoff, it lands anyway.

## 1. Power-on: the stack starts by itself

```
Jetson boots
 └─ systemd: skybeta-stack.service
     └─ host/stack_service.sh        waits (≤90 s) for NTP clock sync, then
         └─ docker: skybeta-stack    starts the container
             └─ run_full_stack.sh    tmux session "skybeta", 4 windows:
                 ├─ agent       run_agent.sh       MicroXRCEAgent on /dev/ttyTHS1 @ 921600 (talks to the Pixhawk)
                 ├─ slam        run_slam.sh        OAK-D driver + cuVSLAM -> /visual_slam/tracking/odometry
                 ├─ bridge      run_bridge.sh      odometry -> /fmu/in/vehicle_visual_odometry (PX4 EKF2)
                 └─ competitor  run_competitor.sh  YOUR code (see below)
```

`run_competitor.sh` does two things:

1. If `~/team_ws/src` contains any package, it builds the team workspace
   (`team_build.sh`). A failed build leaves a shell with the error.
2. It runs `ros2 launch $COMPETITOR_PKG $COMPETITOR_LAUNCH`. Both come from
   `/etc/skybeta/drone.env` and default to `competitor_example competitor.launch.py`.

**This means `team_takeoff_land` does not run on its own.** The default
launch starts only `flight_interface` and `competitor_example`. You need a
team launch file (section 4) and must point `drone.env` at it.

Watch the windows with `docker exec -it skybeta-stack tmux attach -t skybeta`
(Ctrl-b then a window number; Ctrl-b d to detach).

## 2. The nodes in your launch, and the topics between them

```
your planner ──/team/planner/pose|twist──▶ takeoff_land ──/skybeta/setpoint/*──▶ flight_interface ──/fmu/in/*──▶ PX4
your mission ──calls /team/takeoff, /team/land, /team/release──▶ takeoff_land
takeoff_land ──VEHICLE_CMD_NAV_LAND / disarm──▶ /fmu/in/vehicle_command ──▶ PX4
```

- **flight_interface** (skybeta, do not modify): the only node that streams
  Offboard setpoints to PX4. It converts ENU to NED and clamps every pose to
  0.3–3.0 m altitude and 1.5 m/s. It never arms.
- **takeoff_land** (this package): forwards the planner's setpoints, or
  replaces them while taking off, hovering or landing.
- **your planner**: must publish to `/team/planner/pose` or `/team/planner/twist`,
  **not** to `/skybeta/setpoint/*`. Anything publishing there directly
  bypasses the cut-off; `/team/takeoff` refuses while such a publisher exists.
- **your mission node**: decides *when* to take off and land, and calls the
  services. Nothing in this package calls them for you.

## 3. A run, step by step

| # | Who | What happens |
|---|---|---|
| 1 | Stack | Everything in section 1 is up. Leave the drone still so VIO starts cleanly. |
| 2 | takeoff_land | In passthrough. While the drone is not in Offboard and the planner is silent, it publishes "hold where I am" at 10 Hz. The flight interface needs a setpoint less than 1 s old before it will request Offboard. |
| 3 | Pilot | Arms on RC (rule 7.4: in front of the referee). |
| 4 | Pilot | Flips the mode switch to **Position**. This is the start signal. |
| 5 | flight_interface | Requests Offboard once, only if: armed *before* the switch, on the ground, local position valid (VIO fused), a fresh team setpoint, hold service not in force. Any failure is logged as `staying out of Offboard: <reason>`, and it will not try again until disarm. |
| 6 | PX4 | Enters Offboard. The hold-here pose gets clamped to 0.3 m minimum, so **the drone lifts to about 0.3 m straight away**. |
| 7 | Mission | Sees Offboard (`offboard: true` in `/team/takeoff_land/state`) and calls `/team/takeoff`. |
| 8 | takeoff_land | Cuts the planner off. Climbs to `takeoff_altitude_m` above the ground height it recorded while landed. Within 0.15 m for 1 s → **hover**. |
| 9 | Mission | Calls `/team/release`. Planner setpoints flow again. (Or launch with `release_after_takeoff:=true` to skip this.) |
| 10 | Planner | Flies the course. |
| 11 | Mission | Over the pad, calls `/team/land`. |
| 12 | takeoff_land | Cuts the planner off for good. Sends `NAV_LAND` (retries every 1 s, up to 5 times) until PX4 reports Land or Descend. |
| 13 | PX4 | Descends, detects touchdown and disarms itself. If still armed 3 s after touchdown, takeoff_land sends a disarm. Mode → **landed**. |

**Abort:** if the pilot takes the drone back (mode switch or sticks) during
takeoff, hover or landing, takeoff_land goes to **aborted** and sends nothing
more until the drone is disarmed.

## 4. Making it start: a team launch file

Create your own package in `~/team_ws/src` (e.g. `team_mission`) with a
launch file that starts all four nodes. Keep `flight_interface` exactly as
`competitor_example/launch/competitor.launch.py` starts it (including
`auto_offboard_on_arm: true`); only swap out `competitor_example`.

```python
# team_mission/launch/mission.launch.py  (sketch)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    share = FindPackageShare
    return LaunchDescription([
        # flight_interface only (competitor:=false drops competitor_example)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share('competitor_example'), 'launch', 'competitor.launch.py'])),
            launch_arguments={'competitor': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share('team_takeoff_land'), 'launch', 'takeoff_land.launch.py'])),
            launch_arguments={'takeoff_altitude_m': '1.0'}.items()),
        Node(package='team_mission', executable='planner'),   # publishes /team/planner/pose
        Node(package='team_mission', executable='mission'),   # calls /team/takeoff, /team/land
    ])
```

Then in `/etc/skybeta/drone.env` set:

```
COMPETITOR_PKG=team_mission
COMPETITOR_LAUNCH=mission.launch.py
```

and restart the stack (`docker rm -f skybeta-stack`; systemd brings it back in
about 10 s). **On stock PX4 v1.16.2 firmware, power-cycle the Pixhawk after
every stack restart**, or the link stays dead (skybeta bug #15).

## 5. Calling the services by hand (bench and sim only)

```bash
ros2 service call /team/takeoff std_srvs/srv/Trigger
ros2 service call /team/release std_srvs/srv/Trigger
ros2 service call /team/land    std_srvs/srv/Trigger
ros2 topic echo /team/takeoff_land/state
```

In a scored run, only your on-drone mission code may call takeoff (rules 5.1,
5.2). A human calling `/team/land` counts as an abort, which is allowed.

## 6. Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `takeoff_altitude_m` | 1.0 | climb height above the recorded ground |
| `arrival_radius_m` | 0.15 | how close counts as "at altitude" |
| `settle_s` | 1.0 | time inside the radius before hover |
| `takeoff_timeout_s` | 20.0 | after this, hover at the target anyway |
| `release_after_takeoff` | false | hand back to the planner automatically |
| `command_retry_s` | 1.0 | resend interval for land / disarm |
| `land_retries` | 5 | NAV_LAND attempts before logging an error |
| `disarm_after_landed_s` | 3.0 | fallback disarm delay after touchdown |
| `target_system` | -1 | PX4 system id; -1 reads it from `vehicle_status` |
| `planner_pose_topic` / `planner_twist_topic` | `/team/planner/pose` / `twist` | planner inputs |

## 7. Known risks (check on the bench)

- **Offboard may never engage on drone 7.** The flight interface sends its
  Offboard request to PX4 system id 1 by default. Provisioned FCs use the drone
  number as their id (drone 16 was 16). If drone 7's FC is id 7, check
  whether Offboard engages; if not, start `flight_interface` with your own
  `Node(...)` and the parameter `target_system: 7` (`competitor.launch.py`
  does not expose it). Not yet verified.
- **Land may drift sideways.** If EKF2 is not fusing VIO (`xy_valid` false),
  PX4 uses Descend instead of Land, which does not hold horizontal position.
- **Takeoff height follows VIO z.** The ground height comes from cuVSLAM
  odometry. If VIO has drifted (drone 7 once read z = 23 m on the bench),
  restart the stack with the drone still before flying.
