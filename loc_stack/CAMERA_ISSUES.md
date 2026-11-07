# Camera problems on drone 7 (2026-11-07)

Status of the OAK-D (S2) camera pipeline: what is confirmed, what is only suspected, and
what is still being measured. Companion to `HANDOFF.md` and `PREFLIGHT.md`.

## 1. Depth profile makes vision too old for EKF2 (CONFIRMED)

| Profile | image -> bridge latency | EKF2 vision fusion | Result |
|---|---|---|---|
| depth (`slam_profile=depth`) | p50 289 ms, p95 518 ms; ~371 ms old at the FC | pos 0-48 %, yaw 0-32 %, 24 xy resets / 20 s, jumps up to 1.5 m | NO-GO |
| stock (no depth) | ~58 ms | pos/yaw fused, no compass, 0 resets, eph < 4.3 cm | all EKF/VIO checks PASS, also from a cold boot |

- EKF2_EV_DELAY is 70-105 ms (max 300), so depth-mode vision arrives far outside the
  window and EKF2 keeps falling back to the compass.
- The delay is upstream of the bridge: the VIO odometry itself is ~290 ms behind the image
  header stamp in depth mode.
- **Current state: depth is OFF on the drone** (`~/team_ws/slam_profile` renamed to
  `slam_profile.depth_off`). Do not turn it back on for flight until the latency is fixed.

## 2. Camera link is near its throughput ceiling (CONFIRMED, see skybeta/CLAUDE.md #3)

- The MyriadX XLink tops out at ~120-140 MB/s regardless of USB 3 link speed.
- Stock stream: 2 x 1024x640 @ 30 fps ~ 118 MB/s. Depth (320x200 16-bit, ~28 Hz) adds only
  ~4 MB/s, but it pushes the link to the ceiling and adds the on-device StereoDepth work.
- **Working hypothesis (NOT yet confirmed):** 289 ms ~ 8-9 frames at 30 fps, i.e. frames
  queue up (device/XLink/host output queues) rather than being slow to compute. If true,
  small queues (`i_max_q_size`), less bandwidth (960x600 or 20 fps) or less device work
  (`lr_check` off, no alignment) should cut the latency.

## 3. Depth/latency sweep (IN PROGRESS)

`depth_sweep/` swaps only the slam pane through these variants, 45 s settle + 20 s measure each,
recording rate and age by header stamp for left/right/depth, VIO age, bridge lag, `top`,
`tegrastats`:

`stock`, `d_base` (as deployed), `d_q2` (queues 2), `d_nolr` (lr_check off), `d_noalign`,
`d_960` (960x600), `d_20fps`, `d_fastd` (FAST_DENSITY preset).

- The first attempt was contaminated (two sweeps ran at once, 50 s apart) and was moved to
  `results_contaminated_*` on the drone. A single clean run started at ~01:50.
- Results: `~/team_ws/loc_stack/depth_sweep/results/` on the drone. To be added here.
- After the sweep, do a FULL stack restart (`docker rm -f skybeta-stack`) before flying.

## 4. USB network to the Jetson keeps dropping (OPEN)

- Laptop <-> Jetson USB-C networking (RNDIS "Ethernet 3" + NCM "Ethernet 4", both on
  192.168.55.x) drops every 10-60 s; the laptop falls back to 169.254.x and SSH times out.
  `ipconfig /renew "Ethernet 4"` brings it back.
- During the sweep the link went down at 01:50:28, 01:50:49, 01:51:05 (and stayed down),
  i.e. while the camera pipeline was being restarted. Suspected (NOT confirmed): restarting
  the OAK-D (it re-enumerates on the Jetson's USB) disturbs the Jetson's USB device-mode link.
  Needs `journalctl -k` on the Jetson around those times.
- The USB-C cable also came out once today. Use WiFi (`skybeta-07.local` on DroneHacks Wifi)
  for long jobs, and always run them detached on the drone with a DONE file.

## 5. VIO start-up jump after a stack restart (SEEN ONCE)

- After a full restart at 00:40 (stock profile) VIO reported 172 m from its origin (z +87 m)
  and EKF2 jumped 275 m. A second full restart at 00:49 was clean, and the cold boot at
  01:19 was clean (0.09 m from origin, 1.6 cm max frame step).
- Cause unknown (drone possibly moved during start-up, or a cuVSLAM init glitch). Always run
  `./preflight.sh` after any restart; "VIO vs its origin" / "altitude origin" catch it.

## 6. Calibration published to cuVSLAM is not a real stereo rectification (KNOWN, see mapping/FINDINGS.md)

- `camera_info` has D = 0 and no stereo rotation (fx 627/637, cy 326/304 left/right).
  Host-side depth from the published images is unusable; on-device depth uses the factory
  calibration. Possibly related to the unresolved 11.6 % VIO scale question (README Test D).

## Tooling gotchas found today

- `preflight.sh --fix` used to skip the restart when the profile file was removed but depth
  was still running (fixed in 54c9b6e: compares the running profile).
- `pkill -f <pattern>` / `pgrep -f` over SSH match the SSH shell's own command line. Use
  `pkill -x sweep.sh` or a `[s]weep` style pattern.
- Do not wrap a "start detached job" SSH command in a blind retry loop: a dropped connection
  after the job started makes the retry start a second copy.
