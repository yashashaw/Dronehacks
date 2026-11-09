# Why 0.5 m/s did not work (runs 20:05, 23:37, 23:44 UTC on 2026-11-08)

After the 0.3 m/s course + return worked (19:58, see `../2026-11-08_1958_course_and_return`), we tried
cruising at **0.5 m/s** with corners at 0.3 and then at 0.2. None of the three runs finished. The route
on `main` is back at **0.3 / 0.2**.

| Run (UTC) | Speed / corners | Route | What happened |
|---|---|---|---|
| **20:05** | 0.5 / 0.3 | course v2, 9 waypoints | **Hit a wall** on the top edge between waypoints 2 and 3, 11.4 s in, then fell |
| **23:37** | 0.5 / 0.3 | course v2, 10 waypoints (extra waypoint at the 20:05 hit spot) | Clean through the top edge, the tunnel and out (0.03–0.11 m off the path), then **hit something** on the leg from the tunnel exit toward the window, after 30.6 s |
| **23:44** | 0.5 / 0.2 | same 10 waypoints | Log ends 5.7 s in, still climbing at 0.72 m over the launch mark; the team reported a hit |

## What the logs say

**Localization was not the problem.** Camera tracking (cuVSLAM, about 30 frames per second) held the drone within
**0.03–0.11 m** of the path at 0.5 m/s, including inside the dark tunnel, with no position jumps. At 0.5 m/s the
drone moves ~1.7 cm between camera frames.

**What changes at 0.5 m/s is how wide the drone swings and how little time there is to correct:**

- **20:05, top edge.** Tracking was 0.06 m off the line until 11.3 s. At 11.4 s the heading was kicked more than 20°
  off the path (the program stopped advancing its target, which it only does when the actual heading is more than
  20° off), the drone was pushed 9 cm back, bumped up, and then dropped ~1 m in under half a second.
  That is a prop strike, not an estimation error. The drone was 0.52 m forward of the takeoff point; at 0.3 m/s
  the successful runs passed the same spot at 0.55–0.59 m. The margin there is only a few centimetres.
  The 14:36 run (0.3 m/s) also lost the Jetson at almost the same spot (0.59, 2.00 right).
- **23:37, tunnel exit.** At the waypoint-7 corner the drone swung out to 5.26 m right where the route is at 5.13 m
  (13 cm wide). The last logged position is at the start of the climb toward the window, on the path. The hit
  happened in the next few seconds, which are missing from the log (see below).
- **23:44, takeoff.** During the first metre of climb the drone slid ~0.18 m back and toward the net side (the usual
  takeoff slide), then recovered toward the line. The log is cut at 5.7 s.

**Why the end of each log is missing.** When the battery is pulled or the Jetson loses power, the last 1–4 s of the
CSV (still in memory) are never written. The files end in zero bytes (`NUL`). So the collision itself is never in
the log; the rows just before it are.

## Conclusion

- 0.5 m/s cuts the time to the last waypoint from ~58 s to ~38–42 s, and tracking stays good, but **this course
  has only a few centimetres of margin on the top edge and at the tunnel exit**. The extra corner swing and the
  faster arrival at those spots were enough to clip the course three times.
- **0.3 / 0.2 is the speed that has completed the course (14:39) and the course plus return (19:58).**
- To go faster later: keep 0.3 on the tight legs and use the per-waypoint speed (a 4th number on a waypoint) for
  the open legs, and add the PX4 `.ulg` log of any crash (it records motor outputs and battery voltage after the
  Jetson stops).

## Files

- `route-20261108-200517.*`, `route-20261108-233715.*`, `route-20261108-234445.*`: the logs (CSV at 10 Hz + events).
- `top_edge_history.png`: every flight on the top edge and every version of waypoints 3–4.
- `DEPLOYED_with_errors.png`: the route at the time and every error marked (E1–E8).
