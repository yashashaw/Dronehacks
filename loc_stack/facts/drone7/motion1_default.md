# motion1: default config (IMU fusion off), Pixhawk linked and disarmed, 90 s: short move out and back, then a turn in place

Camera: tracking steady (vo_state 1, 29.8 Hz, largest frame step 3.2 cm, longest gap 100 ms). Farthest point 0.566 m horizontal (0.621 m 3D)
at t = 20.9 s, peak speed 0.67 m/s. Ended 0.057 m horizontal from start (0.122 m 3D, 10.7 cm higher). Heading change 345.9 deg for the spin (360 intended, ~4 % short).
Hold t = 33 - 90 s: no measurable drift (1 mm).
EKF2: xy_valid always, eph 4 cm, camera heading change matched (-346.1 deg vs 345.9). Return error 5.8 cm.
GLITCH at t = 85.9 - 86.75 s while the drone sat still: EKF2 heading flipped +102.7 -> +25.9 deg (cs_ev_yaw 0, cs_mag 1, cs_yaw_align 1),
position rotated about the origin by the same angle (|p| = 1.24 m -> 1.53 m jump), eph spiked to 14.14 m, xy_reset_counter 115 -> 118, then it
returned to the vision heading and the original position. The camera was steady (a 0.33 s receive gap just before, at t = 85.2 s; the same
size gap at 40.9, 53.9, 60.2 s caused no event). Mechanism: EKF2 briefly falls back to compass yaw alignment, which is 77 deg away from the
vision frame.
