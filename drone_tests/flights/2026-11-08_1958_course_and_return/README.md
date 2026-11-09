# 2026-11-08 19:58 UTC (11:58 PST): full course + return to base, drone 18

The best complete run: the whole course at **0.3 m/s (corners 0.2)**, a landing on the pad, an automatic
re-arm, and the return over the walls to the launch mark.

| | |
|---|---|
| Route | `route_flown.yaml`: course v2, 9 waypoints, 15.1 m, `yaw: path`, speed 0.3 / corners 0.2 |
| Return | `return_flown.yaml`: up to 2.3 m, around the ladder (pad → [-3.86, -2.00] → [0.00, -2.00] → [0.00, -0.15]), down, land |
| Course time (takeoff → last waypoint) | **58.5 s** |
| Takeoff → **landed and disarmed on the pad** (the run clock) | **70 s** (19:58:56 → 20:00:06 in the log) |
| Tracking on the course | median **0.035 m** off the path, 95 % within 0.08 m, max 0.14 m |
| Landed on the pad at | [-3.78, -0.49] (route frame), ~0.14 m from the pad waypoint |
| Pad → re-arm | 3 s wait, then one Offboard + one ARM command; armed in Offboard 1 s later |
| Return | climbed to 2.31 m, flew around the ladder, came down over the launch mark |
| Return landing | last logged position **0.27 m** from the takeoff point (limit 0.61 m), in the final descent |

## Events (`route-20261108-195856.log`)

```
19:58:56 Offboard active: route "course-v2-return", 15.1 m, start (PX4 ENU) (-0.07, -0.05, 0.04), heading 90 deg
19:59:55 last waypoint reached in 58.5 s, max cross-track 0.22 m
19:59:56 LAND: route complete
20:00:06 landed and disarmed: done
20:00:06 RETURN: on the pad, re-arming in 3 s (mode switch off Position cancels)
20:00:09 RETURN: requesting Offboard + ARM
20:00:10 RETURN: armed in Offboard, flying "return-to-X" (11.3 m) back to the takeoff point
```

## Notes

- The CSV restarts its time column at 0 when the return starts (row 732).
- The log ends during the return's final descent (0.29 m above the floor, 0.27 m from the takeoff point):
  the battery was disconnected before the last rows were written, so the final disarm is not in the log.
- On the last high leg of the return the drone flew two loops (cross-track up to 0.41 m at 2.3 m height)
  before settling over the launch mark. Camera tracking looks weaker at 2.3 m than at 1 m.
- The pad touchdown took ~10 s from arriving over the pad to disarm, including a small bounce.
