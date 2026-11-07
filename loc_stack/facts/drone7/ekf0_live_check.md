# Live EKF2 check, drone 7 (20 s, drone sitting still, position-only setup, Pixhawk linked and DISARMED)

Not loaded by us: someone enabled EKF2 vision fusion on the Pixhawk between the 13:xx and 22:2x checks
(earlier flags: cs_ev_* all false; in this run cs_ev_pos / cs_ev_yaw true 95 % of the time, compass fusion off).

- Stable agreement: camera odometry (x 9.74, y 31.6) vs EKF2 NED position (31.63, 9.74): axes swapped as expected
  from the ENU->NED bridge, equal to within ~4 cm. EKF2 eph 0.05 m, epv 0.07 m, heading -11.7 deg.
- Wander while still: EKF2 4.7 cm drift over the window, camera 6.5 cm.
- GLITCH at t = 16.77 - 17.5 s (0.7 s): eph rose to 14.1 m, vision fusion dropped for ~0.8 s (dead-reckoning),
  xy_reset_counter 79 -> 82, EKF2 position jumped from (31.63, 9.74) to (22.11, -5.11) (17.6 m), heading -11 -> +9 deg,
  while the camera position moved only ~4 cm. During the excursion xy_valid stayed 1 and eph read 0.15 m.
  A 323 ms gap in camera odometry sat at the same moment (similar 0.32-0.34 ms gaps at t = 3.7, 11.8, 14.1 s did not cause it).
  The compass fusion flag briefly turned on at t = 17.5 s: possible yaw conflict between compass and vision (unconfirmed).
- Rates: camera odometry 28.0 Hz in this window, EKF2 local position ~100 Hz.
