# Drone 7 hard numbers (2026-11-06)

Method notes: hand-carry only, props off, flight controller NOT linked to the Jetson (no EKF2,
no rangefinder, no command response). "Measured" = recorded on drone 7. "Arithmetic" = computed
from measured calibration. "Not measurable" = needs the flight controller or a flight.

## Position drift (stereo VIO without IMU fusion, hand-carried, one 8 ft out-and-back)
| | Value |
|---|---|
| Held still 30 s: position wander / yaw drift | 0.9 cm / 0.1 deg |
| Start to turnaround (true 2.438 m) | 2.16 m (3D), 2.11 m horizontal: 11.6 % / 13.4 % short |
| Landing vs takeoff, odometry | 0.160 m (0.155 along the line, 0.038 sideways) |
| Landing vs takeoff, SLAM loop closure | 0.107 m |
| Net yaw change over the run (hand-held) | +5.0 deg |
| Largest odometry step (33 ms) | 4.7 cm (no jumps over 5 cm) |
| SLAM pose steps over 5 cm | 2, largest 11.7 cm (loop-closure corrections) |
| Tracking lost | 0 events in ~212 s of logged hand-carry; 155 frame-gap warnings (>40 ms) in ~95 min of runtime |
| Not yet measured | heading error after turns, several loop lengths |

## End-to-end delay, image header stamp to bridge publish (30 s probe each)
| Config | p50 | p95 | max |
|---|---|---|---|
| default (no IMU fusion, no mapping) | 58 ms | 281 ms | 373 ms |
| IMU fusion ON, mapping OFF | 61 ms | 108 ms | 502 ms |
| IMU fusion ON, mapping + landmark viz ON | 158 ms | 349 ms | 388 ms |

Driver leg is 36 ms of each. Mapping/visualization is what adds ~100 ms, not IMU fusion.
Excludes the serial hop to the FC (not measurable).

## Jetson budget (IMU fusion + mapping + viz config, with probes running)
CPU avg 32 % / max 66 % of 6 cores; stack container ~130 % of 600 % typical; GPU avg 5 % / max 95 %;
RAM max 3.25 / 7.6 GB; 57 C; 9.2 W avg / 10.3 W max; power mode MAXN_SUPER, 1728 MHz; USB 5 Gbps.
Speed vs laptop: same pure-Python loop, 263 ms (Jetson, py3.10) vs 65 ms (laptop, py3.13): ~4x slower.
Host stereo matching on the Jetson (images not rectified well enough to be usable): SGBM 94 ms / BM 29 ms
at 1024x640; SGBM 16 ms / BM 6 ms at 512x320.

## Depth
No depth, disparity or point-cloud topic is published (pipeline `stereo`, mono pair only). Min range
arithmetic from real calibration: fx x baseline = 627 x 0.0754 = 47.3 px*m, so 0.50 m at 95 disparity
levels, 0.25 m with extended disparity. Not hardware-confirmed. Plain-wall patchiness not measured.

## Rules and layout (from the repo, not measured)
Touch of a wall/tunnel/boulder: +5 s each. Crash ends the run (also: ground contact elsewhere, leaving
the course, 3-minute limit). The repo does not define touch vs crash or frame vs props: ask organizers.
Course dimensions: `../../README-course-dimensions.md` (derived from the OBJ model, preliminary).
