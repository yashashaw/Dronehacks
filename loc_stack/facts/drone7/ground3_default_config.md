# ground3: documented full-stack restart, DEFAULT config (IMU fusion off, no depth, mapping off), Pixhawk linked, disarmed, still

Procedure: `docker rm -f skybeta-stack` (service restarted everything, container back ~104 s, odometry flowing after ~2.5 min),
confirmed imu_fusion=False / mapping=False / no depth topic, waited 75 s still, 8 s settle check, 150 s recording, then a 40 s delay probe alone.

| | ground1 / ground2 (IMU fusion ON, SLAM-only restarts) | ground3 (default, full restart) |
|---|---|---|
| Camera pose after the wait | jumped 14.65 m (z -11 m) at 17 s / sat 8.3 m from origin | 0.73 m from origin, no jump |
| Largest single-frame camera step | 14.65 m | 0.10 m |
| EKF2 xy resets in the run | 36 / 15 | **0** |
| EKF2 jumps > 5 cm | 25 / 10 (up to 14.9 m) | **0** (largest 1 mm) |
| Vision dropouts / compass-on events | 7 / 10 and 4 / 5 | **0 / 0** |
| ev_pos, ev_yaw fused | ~94-98 % | **100 %**, compass on 0 % |
| EKF2 heading | flipped by 55 and 142 deg | 89.6 - 92.7 deg |
| EKF2 eph | median 4 cm, spikes to 14 m | 3.8 - 4.0 cm |

Camera while sitting still: net drift 0.167 m / 150 s, wander 0.26 m, height span 0.13 m. EKF2: 0.111 m drift (4.6 cm/min), wander 0.21 m.
Delay (probe alone, ~4 min after restart): p50 58.7 ms, p95 250 ms, max 371 ms (matches the original 58 ms).
Conclusion: the startup jump, the 14 m EKF2 flips and the compass flicker were caused by our changes (IMU fusion on and/or SLAM-only restarts),
not by the Pixhawk's compass setting. The stack is currently in the default configuration.
