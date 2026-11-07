# Drone 7 pre-flight: settings, wait times, and the one command

`./preflight.sh` (in this folder) is the last thing you run before the pilot arms. It reads
everything over DDS, never arms and never commands PX4, and ends with **GO**, **WAIT** or
**NO-GO** plus the fix for each failure. Reports are saved in `runs/preflight/<time>/`.

```
./preflight.sh --fix         # make ready (full restart if needed), then check
./preflight.sh               # check only
./preflight.sh --lift        # rangefinder: does dist_bottom follow a 0.5 m lift?
./preflight.sh --record 240  # check; if GO, start the flight recorder for 240 s
```

> Status: the GO/NO-GO rules are unit-tested offline (`python test_preflight_rules.py`).
> The script and the on-drone collector have **not yet run on the drone** (it was
> disconnected when they were written). Do a bench run, props off, before relying on them.

## 1. Run-day order

| When | What | Why |
|---|---|---|
| Once per session, Pixhawk USB to laptop | Parameter check (section 3), read the firmware hash, then **unplug the Pixhawk USB** | DDS cannot read parameters. PX4 refuses to arm while its USB is connected. |
| Power-on | Drone **on the launch mark**, battery in, do not touch it | The camera's origin and heading, and the flight interface's altitude zero, are set during start-up. |
| ~3-5 min later | `./preflight.sh --fix` | Waits for each stage, prints how long each took, then checks. |
| Optional, once | `./preflight.sh --lift` | The rangefinder is the height reference and has never been seen off the ground. |
| Right before arming | `./preflight.sh` (add `--record 240` to log the run) | Must say GO. |
| Then | RC on, kill switch tested, pilot arms, flips to Position | The start signal (rule 7.4). |
| Between runs | Carry back, place on the mark, `./preflight.sh --fix` | After a run the origin is stale; a restart re-zeroes it on the mark. |

## 2. Wait times (what is known, and where it comes from)

| Stage | Typical | Source |
|---|---|---|
| Jetson boot to stack started | ~20 s with a network; **up to 90 s** without (it waits for NTP) | drone 16, `skybeta/CLAUDE.md`; `host/stack_service.sh` |
| Full stack restart (`docker rm -f`) to container back | ~104 s | drone 7, ground3 |
| ...to VIO odometry flowing | ~2.5 min | drone 7, ground3 |
| Pixhawk power-on to link | ~15-20 s | drone 16 |
| Hold still after odometry starts | **75 s** | ground3 procedure (the clean result) |
| Total, power-on to GO | roughly 3-5 min | estimate; `--fix` prints drone 7's real times, so write them down |

**When a reboot or power cycle is needed**

- **Pixhawk firmware decides it.** On stock PX4 v1.16.2, any stack restart or Jetson reboot
  wedges the Pixhawk's TELEM2 until the Pixhawk is power-cycled (skybeta bug #15). The patched
  build (git hash `7a39c59`) reconnects by itself. Drone 7's hash is **unknown**: read it in
  section 3. Until then, if `--fix` reports no Pixhawk link 90 s after a restart, power-cycle
  the whole drone (battery out and in, on the launch mark).
- A cold power-on of everything together is fine on either firmware: the Pixhawk boots first
  and has never lost a session.
- **Never restart only the slam pane before a flight** (that includes `depth_test.sh off`).
  ground1/2 (slam-only restarts, IMU fusion) had a 14.6 m start-up jump and EKF2 heading flips
  of 55-142 deg; ground3 (full restart, default config) had none. Pre-flight fails if the slam
  pane was restarted on its own, and `--fix` does the full restart.
- **Clock:** if the Jetson has a network but no NTP sync yet, a later sync steps the clock and
  the stack service restarts the stack mid-run. Pre-flight fails in that state. Without any
  network the clock never steps (only a warning).

## 3. Parameters (once per session, over the Pixhawk's USB)

In QGroundControl (Parameters), or `python skybeta/tools/param_diff.py --master COM<n>`
(needs pymavlink) for the full fleet diff. Firmware hash: QGC > Analyze Tools > MAVLink
Console > `ver all`.

| Parameter | Expected | What breaks if wrong |
|---|---|---|
| `MAV_SYS_ID` | **7** | `flight_interface` must target the same id or Offboard never engages (pre-flight checks this live) |
| `UXRCE_DDS_DOM_ID` | **7** (= `ROS_DOMAIN_ID`) | no `/fmu/*` topics at all |
| `UXRCE_DDS_PTCFG` | 2 | no `/fmu/*` topics in local scope |
| `UXRCE_DDS_CFG` / `MAV_1_CONFIG` | 102 / 0 | TELEM2 owned by MAVLink, no DDS |
| `SER_TEL2_BAUD` | 921600 | silence on the link |
| `EKF2_EV_CTRL` | 9 (horizontal position + yaw) | no vision fusion, no Position/Offboard |
| `EKF2_EV_DELAY` | 60-105 ms (team file: 105) | vision fused at the wrong time; least certain value |
| `EKF2_HGT_REF` / `EKF2_BARO_CTRL` / `EKF2_RNG_CTRL` | 2 / 1 / 1 | height runaway if the rangefinder drops out with baro off |
| `SENS_EN_VL53L1X` | 1 | no rangefinder |
| `EKF2_GPS_CTRL` / `EKF2_OF_CTRL` | 0 / 0 | rule 6.1 (no GNSS); no flow sensor exists |
| `EKF2_MAG_TYPE` | 0 (baseline) | **open risk**: see below |
| `COM_RC_IN_MODE`, `RC_MAP_KILL_SW`, `RC_MAP_ARM_SW` | 0, 5, 7 | RC ignored or switches swapped |
| `COM_RC_OVERRIDE` | 3 | the pilot cannot take over from Offboard with the sticks |
| `SENS_BOARD_ROT` | 8 (roll 180) | attitude upside down |
| `COM_DISARM_LAND` | 2 s | no automatic disarm after landing (checkpoint 5) |

Do not change failsafe, kill-switch, geofence or battery parameters (rule 8.5). EKF2 changes
need the organizers' OK (`px4/ekf2_vision_drone7.params` header).

**Biggest open localization risk: the compass.** With `EKF2_MAG_TYPE=0`, EKF2 sometimes falls
back to compass heading for under a second (motion1: heading flipped 77 deg, position rotated
about the origin, eph 14 m). Indoors the compass disagrees with the camera's heading. A 20 s
still window cannot rule it out; pre-flight fails if it is seen. Ask the organizers whether
`EKF2_MAG_TYPE=5` (no magnetometer) is allowed.

## 4. What `./preflight.sh` checks

**Jetson:** NTP sync (vs network), 6 CPU cores, MAXN power mode, stack container running and
its age, all four tmux windows alive, the slam pane not restarted on its own, no leftover
recorders, free disk.

**Camera and localization (measured over the window, by header stamp):** left/right camera
>= 28 Hz, camera IMU >= 150 Hz, VIO odometry >= 28 Hz, bridge to PX4 >= 25 Hz, depth off
(stock profile), VIO tracking the whole window, no frame step over 5 cm, still within 5 cm,
distance from where SLAM started (warns over 2 m: a start-up jump if the drone never moved).

**Flight interface:** running, `auto_offboard_on_arm` true, `target_system` equal to PX4's
system id, **the drone within 10 cm of the flight interface's altitude zero** (its 0.3-3.0 m band
is measured from the first odometry it received, so the competitor window must start with the
drone on the floor), exactly one publisher on each `/fmu/in` topic, at most one on
`/skybeta/setpoint/*`, and a warning if no team node publishes setpoints.

**Pixhawk:** link, system id, disarmed, landed, PX4's own arming checks (lists the failsafe
flags that are set), battery connected with charge, EKF2 xy valid, eph under 10 cm, no xy
resets or jumps in the window, vision position and yaw fused >= 98 %, compass not used,
height from the rangefinder, rangefinder reading 2-30 cm on the ground.

Exit code 0 GO, 2 WAIT, 1 NO-GO, so a run script can refuse to start without a GO.

## 5. Flying with depth (obstacle avoidance): the `depth` camera profile

Opt-in; the stock profile is unchanged. With it set, the stack starts the camera with
on-device depth **from boot** (so no slam-only restart is ever needed):

| Item | Value (measured on drone 7, 2026-11-06) |
|---|---|
| Topic / format | `/oak/stereo/image_raw`, 16UC1 millimetres, 320x200 |
| Rate | ~25-28 Hz |
| Frame | `oak_rgb_camera_optical_frame` (check the TF from `base_link` exists) |
| Nearest reading | ~0.31 m; anything closer is 0 |
| 0 pixels | **unknown, never free space** |
| Cost | VIO ~25-26 Hz instead of 30; left/right ~27.5 Hz |

**Switch on (no sudo):** `echo depth > ~/team_ws/slam_profile` on the drone, then a full
restart (`./preflight.sh --fix`, or cold boot). **Switch off:** `rm ~/team_ws/slam_profile`,
full restart. (`SKYBETA_SLAM_PROFILE=depth` in `/etc/skybeta/drone.env` does the same with sudo.)
Check with `./preflight.sh --profile depth`: it fails if no depth is streaming (the slam
window falls back to the stock pipeline when the depth launch cannot start, so a silent
fallback is caught here) and if the stack's profile and `--profile` disagree.

**Before relying on it:** run the blank-wall test (`depth_test.sh wall`): the S2 has no IR
projector, and plain walls may return little depth. Then repeat the drift (Test D) and
vision-delay (`tools/latency_probe.py`) measurements in this profile; `EKF2_EV_DELAY` was
measured without depth.
