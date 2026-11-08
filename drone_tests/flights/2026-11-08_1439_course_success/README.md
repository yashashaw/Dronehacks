# First successful autonomous course run: drone 18, 2026-11-08 14:39 (PX4 clock)

Flown by `team_src/px4_route` (this commit) with `route_flown.yaml` (= `routes/course_walk_DRAFT.yaml`).

| | |
|---|---|
| Start | **slightly behind the black X (less than 4 in / ~10 cm, per the team)**, nose along the floor line toward the pipe wall. The route assumes centred on the X, so the whole flight was ~0.1 m further back (away from the pipe wall) than planned. That margin probably helped near the forward net at waypoints 3�4. |
| Route | 14 waypoints from the 21:55 course walk, 15.0 m, walk heights 0.95–1.30 m, 1.3 m at the window |
| Mode | `yaw: path` (nose faces the way ahead, turns first), 0.3 m/s, corners 0.2 m/s, off-path landing off |
| Result | **all 14 waypoints in 61.8 s, landed and disarmed by itself 0.12 m from waypoint 14 (the pad)** |
| Tracking (PX4's own position) | median 0.02 m off the planned line, max 0.09 m after takeoff, 0.23 m during the takeoff slide |
| PX4 position | stable throughout; largest step 4 cm per 0.1 s; nose-forward turns caused no jumps |

`route-20261108-143920.csv` has 10 Hz rows: phase, carrot, PX4 position, cross-track, rangefinder. `.log` has the events. `success_run_1439.png` shows flown vs planned.

## Notes for next runs
- **Close calls seen by the team are not in this data.** The drone flew where the route said, so tight spots come from the route itself (the walk passed close to walls), takeoff heading/placement, or VIO drift (~1 % of distance). Nudge the specific waypoints 0.15–0.25 m away from the obstacle.
- **The rangefinder never saw the window sill** in this flight; the walk saw a ~0.9 m sill there. Confirm whether it went through the window (+5) or Gate B.
- **The earlier attempt (route-20261108-143603) stopped logging mid-flight at 15.8 s while on track, with the file ending in zeros: the Jetson lost power or rebooted in the air.** Check the battery connector and the Jetson power lead.
