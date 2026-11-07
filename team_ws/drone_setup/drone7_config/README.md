# Drone 7 configuration, 2026-11-07

Every setting that lived **on drone 7** (outside this repo) at the end of the session, so the
same setup can be rebuilt on another drone. Drone 7 is no longer available; these files were
reproduced from the session (commands run, output printed), not copied off the drone. Each row
says how.

## Where each file goes on a drone

| File here | On the drone | What it does | Source |
|---|---|---|---|
| `drone.env` | `/etc/skybeta/drone.env` (root, `sudoedit`) | identity, DDS domain, **camera mount**, which team launch file the stack starts at boot | drone 7's file as printed in the session, with the 2 lines `autostart.sh disable` changed (comment above them added here). **`DRONE_ID`, `ROS_DOMAIN_ID` and `CAMERA_*` are drone 7's: change them for another drone** |
| `team_ws_autostart.conf` | `~/team_ws/autostart.conf` | autostart switch read by `team_flight_test/launch/autostart.launch.py` | written by `autostart.sh disable` (exact) |
| `team_ws_slam_profile` | `~/team_ws/slam_profile` | camera mode: `depth` = 640x400 + depth, `stock` (or no file) = 1024x640 no depth | exact |
| `oakd_vslam_depth640.yaml` | generated at every stack start into the container's `/tmp/skybeta_depth/depth.yaml` | the camera config actually running in `depth` mode | regenerated here with the same transformation as `../skybeta_depth640.patch`; matches what drone 7 printed |
| `../skybeta_depth640.patch` | applied to `~/skybeta` (0.3.3 @ 3c42933) | adds the `depth` profile to `run_slam.sh`, `host/run.sh`, `host/drone_env.sh` | taken from drone 7 with `git diff` (exact) |
| `camera_bench/` | `~/cam_bench/` | camera-only benchmark: configs A-E, `cam_bench.py`, `run.sh`, `all.sh` | exact (configs written by the same generator; `run.sh` now takes the image tag from `~/skybeta/VERSION` instead of a fixed 0.3.2) |
| `loc_stack/depth_sweep/make_variant.py` (repo root) | `~/team_ws/loc_stack/depth_sweep/` | full-stack sweep variants, incl. the deployed `d_640s+nopreset` | updated in the repo to drone 7's state |

Not stored anywhere: `sudo`/Wi-Fi passwords (ask the organizers). PX4 parameters were not changed.

## Order to set up a drone the same way

1. `team_ws/drone_setup/install_to_drone.sh skybeta-NN <ip>` (packages + field tests).
2. `~/team_ws/src/team_flight_test/scripts/autostart.sh disable` on the drone (writes
   `autostart.conf` and the two `COMPETITOR_*` lines in `drone.env`; asks for sudo).
3. Apply `skybeta_depth640.patch` (see `../README.md` step 4), then `ft.sh camera new`
   (writes `slam_profile` = `depth`, restarts the stack, ~2 min, disarmed).
4. Check `drone.env`'s `CAMERA_*` match the real camera mount on that airframe.
5. **Read `team_ws/F1_CLIMB_INCIDENT.md` before flying.**

## Measured results behind the camera choice (drone 7)

Camera only (driver alone, 40 s each, `camera_bench/`):

| config | link MB/s | left/right | depth |
|---|---|---|---|
| A stereo 1024x640, no depth (old default) | 118 | 30 Hz, 0 lost | none |
| **B1 depth mode, raw 640x400 + depth** | **61** | 30 Hz, 0 lost, L/R sync 100 % < 1 ms | 30 Hz, 2/1200 lost |
| E depth mode, raw 1024x640 + depth | 133 | 0.3-0.4 % lost | 1.7 % lost (at the XLink ceiling) |
| B2 / C / D rectified pair | - | rectified images never published by this driver | 30 Hz |

Full stack with cuVSLAM (`loc_stack/depth_sweep`, 20 s each):

| variant | VIO | VIO age p95 | depth | camera+VIO CPU |
|---|---|---|---|---|
| stock 1024x640 | 30 Hz, 0 gaps | 67 ms | none | ~62 % |
| d_base (old "depth" profile, 1024x640 + 320x200 depth) | 27 Hz, 53 gaps | 304 ms | 27 Hz | |
| d_640 (640x400, preset DEFAULT) | 30 Hz, 0 gaps | 38 ms | 320x200 | |
| **d_640s+nopreset (deployed)** | **30 Hz, 0 gaps** | **39 ms** | **640x400 @ 30 Hz, 24 ms, subpixel** | **~44 %** |

Gotchas found: `i_depth_preset: DEFAULT` silently decimates depth to 320x200; combining two
sweep variants re-applies `DEPTH_BASE` (so `d_640s+d_noalign` lost subpixel); stack restarts wait
up to 90 s for an NTP clock sync; a clock jump (drone first gets internet) restarts the stack.
