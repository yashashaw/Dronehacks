# Run imu1: IMU fusion ON, mapping OFF (hand-carried, 100 s, props off)

Sections are inferred from the speed / yaw timeline (no marks were pressed).

| # | Section | Time (s) | Result |
|---|---|---|---|
| 1 | Start hold | 0 - 13.9 | wander 3.1 cm |
| 2 | Outbound (8 ft = 2.438 m) | 13.9 - 30.3 | reached 2.53 m horizontal (2.61 m 3D): 3.7 % LONG |
| 3 | Return | 30.3 - ~37 | at t=37 s the drone was 0.75 - 0.85 m from the start point, nearly still |
| 4 | Spin in place | ~37.9 - 42.3 | yaw -21 -> -340 deg (about 318 deg in 4.4 s, peak 157 deg/s); offset swung 0.75 -> 0.40 -> 1.00 m |
| 5 | Final hold | 43.7 - 99.8 | 0.980 m horizontal from start (0.982 m 3D); wander 1.3 cm; height +5.8 cm |

- Net yaw at the end: -336.3 deg (23.7 deg short of a full 360, 6.6 %), if a full turn was intended.
- Pose jump at t = 39.5 s (during the spin, yaw ~ -113 deg): 145 ms receive gap and a 0.717 m step.
  `/visual_slam/status` vo_state stayed 1 the whole run (no tracking-lost report).
- Steps > 10 cm in one frame: 8 (t = 14.0, 14.6, 14.6, 39.5, 41.4, 41.5, 43.4, 43.5 s), around pick-up and the spin.
- Rate 29.3 Hz received, max gap by header stamp 67 ms.
- Mapping was off, so no landmark cloud was produced; `slam_odom` (map frame) is identical to the odometry.
- Not a clean IMU-fusion comparison: this run also had a fast pick-up and a 160 deg/s spin that run 1 did not.
