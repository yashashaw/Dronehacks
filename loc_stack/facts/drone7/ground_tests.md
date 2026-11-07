# Ground tests, drone 7 (disarmed, props off, sitting still). 2026-11-06 / 07

The guarded Pixhawk reboot (one VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, disarmed check passed) had NO effect: xy_reset_counter kept
counting, no gap in /fmu/out. Same EKF2 session throughout. Cause of the non-reboot not investigated (no command ack recorded).

## ground1 (recording started 16 s after a SLAM restart)
- Camera pose frozen at (0.34, -0.44, -1.07) for 17 s, then ONE frame jumped 14.65 m (z -11 m) while the drone sat still. Steady after.
- EKF2 followed the jump (xy_reset_counter 104 -> 105) and later flipped repeatedly between (7.41, -12.57) heading +63 deg and
  (-6.3, -13.5) heading +8 deg. Both positions are ~14.6-14.9 m from the origin and 55 deg apart = the heading difference.
- 36 EKF2 xy resets in 180 s, 7 vision-fusion dropouts, compass-fusion flag turned on 10 times, xy_valid never false, eph 0.04 m typical, 14 m spikes.

## ground2 (recording started 70 s after a SLAM restart)
- Settled pose was (-6.88, -0.03, -4.62) m: NOT near the origin although the drone was still (startup jump again).
- Camera drift 0.35 m in 150 s (12 cm/min), height span 0.27 m, one 0.17 m single-frame step at t = 63 s.
- EKF2: 15 xy resets, 4 dropouts, 5 compass-on events, 3 heading flips +9 <-> -133 deg each moving the position ~13.3-13.7 m
  (radius 6.9 m from the origin, 142 deg flip: 2*6.9*sin(71 deg) = 13.1 m).
- Hypothesis (supported, not proven): EKF2 alternates between compass-aligned and vision-aligned heading; the position error equals
  (distance from origin) x (heading flip).
- The bridge raises reset_counter when tracking resumes, so EKF2 accepts the big camera jump as a legitimate reset instead of gating it.
