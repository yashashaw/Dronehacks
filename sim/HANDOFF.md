# DroneHacks sim project: handoff (read this first)

Written 2026-11-07 mid-session so a fresh context can continue. Everything here is verified from this session unless marked **UNVERIFIED**.

## 1. What the user wants (goals, in order)

Staged goal set by the user (`/goal`):
1. Revert to the original mission that goes up then down. **DONE** (run_002 PASS).
2. A simple localizer that commands the Pixhawk: 1 m forward, 1 m back, land. **PARTLY**: `stage23.py` does out-and-back and PASSED (run_008) but with a **0.8 m** leg, not 1.0 m. Re-run at exactly 1.0 m is still owed.
3. Add depth estimation so it understands its state. **DONE in stage23** (reads clearance ahead before moving, sizes the leg).
4. A full A*-based run on that depth estimation, **now 60 seconds** (user lowered it from 2 min "until I say otherwise"), then land. **NOT DONE**: runs 009-011 failed the grade (see section 5).

Standing user instructions:
- Build and test everything **locally** on the laptop, put **only what is necessary** on the simulator. Keep the simulator project as small as possible.
- **No numba.** Plain Python (numpy is fine). The planner must be plain Python.
- Remove the repo clone and other extras from the simulator (done).
- **Turn so the depth camera stays at the front** (turn first, then translate; goals within the camera field of view). Implemented (patch v3).
- **Clearance 0.1 m.** I interpreted it as a 0.10 m margin beyond the 0.28 m airframe reach, so planner radius is 0.38 m. If the user meant something else it is one constant (`CLEARANCE` in `depth_planner.py`). Ask if unsure.
- Run time is **60 s** until told otherwise (`RUN_S` in `explore_run.py`, `RUN_LIMIT_S` in `stage23.py`).
- User wants real analysis of readings, not blind re-runs. They said so twice ("analyze why it crashed instead of just trying the same thing over again").
- Confirm anything that arms/flies is within what the user asked; they have explicitly authorized sim flights, world resets, and Keyboard RC use. The docs require team approval before arming; user acts as the team.

## 2. The website and how it behaves

- URL: `https://team-04.dronehacks-lab-136-112-11-33.sslip.io/` (FlyX Lab, Stanford DroneHacks 2026, "Sky Beta" drone, simulation). Chrome is driven through the Claude in Chrome tools; the browser must be selected with `select_browser` (two Chrome devices appeared once).
- **Workbench** tab: editor + terminal on the left, Live Observation (follow-rear, main RGB, stereo depth) top right, Program Output bottom right, status bar at the bottom (READY / Armed / RTF, Reset world, Land, Stop program).
- **Run saved file**: runs the file chosen in the dropdown next to it, as an independently recorded + graded run. Any `.py` in the workspace root works (not only `main.py`). After a run the site says "Reset the world before running the next program" and disables Run until **Reset world** finishes (about 2-3 minutes, status goes RESETTING then READY/Disarmed).
- **Recordings** tab: per-run Program status, Flight grade (PASS/FAIL), reason text (for example "truth clearance 0.06 m < airframe reach 0.28 m (contact)" or "estimator horizontal error p95 2.23 m > 1.00 m"), "View output".
- **Full-size editor/terminal**: open `https://team-04.dronehacks-lab-136-112-11-33.sslip.io/workspace` in a **second browser tab**. The workbench terminal is only ~27 columns wide; the /workspace tab gives a wide terminal. Use it for everything terminal-related.
- Keyboard RC panel exists (Manual/Altitude/Position/Offboard, Enable keyboard RC, Arm). Not used in the end: scripts arm through PX4 commands.
- Banners like "The server did not respond" or "Connection interrupted" appear during resets and clear on their own; do not retry actions while they show. "Action could not be completed: runtime refused the request" had one real cause (section 4, lesson 1).

## 3. Project structure

### Local (source of truth), `C:\Users\lucas\dronehacks-local\`
- `course/` : `dronehacks-course-v3.obj/.mtl/README.txt` (the official course layout, metres, Z up, origin floor centre, 20x20 ft net 10 ft tall).
- `tools/astar_course.py`, `tools/obj_bboxes.py` : offline A* over the OBJ and bbox listing (analysis only).
- `sim/` : everything that goes to the simulator, plus local-only tests:
  - `stage23.py` : PX4 interface class `Stage` (Offboard setpoints, arm, depth clearance) + the out-and-back mission. Other scripts import `Stage` and `log` from it.
  - `depth_planner.py` : plain-Python A* on a drone-centred grid built only from depth. `build`, `astar` (bounded 1500 nodes), `choose_goal` (60 ms budget, goals 0.8-1.6 m ahead within +/-45 deg), `carrot`. RADIUS = 0.28 + 0.10.
  - `explore_run.py` : depth-only exploration flight (class `Flyer`): depth columns to hits, 2 s memory, A*, turn-first logic, HOLD when depth validity < 30%, SCAN when no goal, POSITION LOST handling, lands at `RUN_S`.
  - `depth_probe.py` : hover at 1.0 m and print the real depth profile + planner decision (written, pushed, **not yet run**).
  - `planner_lib.py`, `astar_run.py` : old OBJ-map version, kept locally only (archived, removed from sim).
  - `patch_v2.py`, `patch_v3.py`, `patch_v4.py` : the patches already applied (keep local copies in sync with the sim).
  - `test_worst_case.py` : local timing test (`python test_worst_case.py`; planner worst case about 14-35 ms).
- `HANDOFF.md` : this file.

### On the simulator, `/var/lib/flyx-lab/workspace/` (about 3.7 MB; keep it small)
Original files untouched: `main.py` (reference up/down mission: 1.2 m, hover 5 s, land), `refnav/`, `route.json` (restored to `{"route": [], "params": {}}`), `flight.yaml`, `tests/`, `learn/`, `onboard/`, docs.
Added by me: `stage23.py`, `depth_planner.py`, `explore_run.py`, `depth_probe.py`. Nothing else. `/home/flyx` is empty. The GitHub clone, numba, ROS build and venv were all deleted.

### GitHub repo (reference only): `https://github.com/yashashaw/Dronehacks`
Branches: main, integration, baseproject, slam-one-shot, pad-pipeline, CV_experiments. `integration` already contains `slam-one-shot`. A local clone with a `sim-integration` branch (3 commits: timed land, test script) existed on the sim and was **deleted**; those commits were never pushed and are gone. The repo's ROS chain (team_planner, course_pipeline, skybeta flight interface) targets the real drone (ROS Humble, `/oak/*` topics) and was NOT used in the final approach.

## 4. Lessons learned (important)

1. **Workspace size limit.** "Run saved file" snapshots the workspace. At 258 MB (numba/llvmlite 206 MB, repo clone 50 MB) every run was refused with the generic "runtime refused the request". Moving extras out fixed it instantly. Keep the workspace to a few MB; never put clones/venvs/builds there. (Saved to memory as `dronehacks-lab-workspace-size`.)
2. **Terminal width.** The workbench terminal is ~27 columns; output is unreadable. Use the **/workspace tab** (wide). Always click inside the terminal before typing (a typed command was silently lost when focus was on a panel button).
3. **Big heredocs.** Pasting a very large multi-heredoc command (two files at once, ~16 KB) once hung the shell and applied nothing. Send **one file per command** (up to ~9 KB worked), and verify with `grep -c` / `wc -c` afterwards.
4. **Reading program output.** The page blocks JavaScript access to the Program Output panel (`[BLOCKED: Cookie/query string data]`) and `get_page_text` only returns the run list. Working method: click the panel's **Copy** button (aria "Copy visible output"), then in the /workspace terminal run `cat > /tmp/run.log`, press ctrl+v, ctrl+d, then grep/cut the file.
5. **Dropdown selection.** The file dropdown is a native select; set it with `find` + `form_input` (value = file name), then refresh with the circular arrow beside it. Clicking Run with `computer left_click ref_16` (the button's ref) was more reliable than coordinates.
6. **Reset is mandatory** between runs and takes 2-3 minutes. Coordinates of Reset world move with the layout (about (1016,621) in the 1070x639 layout); check a screenshot.
7. **Zoom/screenshots.** Layout shifts when panels change; re-screenshot before clicking. Keep tab 1 = workbench, tab 2 = /workspace terminal; close extra tabs.
8. PowerShell here is 5.1: no `&&`; use `;` and `if ($?)`. `python -I -` cannot import local modules; run test scripts as files.
9. Reset can show "Connection interrupted"; wait it out, do not press anything.
10. **Workflow rule from the user: press Reset world immediately after diagnosing a run, then write and test the next iteration locally while the reset runs (2-3 min).** Waiting to reset until the new code is ready just wastes that time. The sequence is: run finishes, read the grade and log, click Reset world, then analyze, patch locally, push files, and run as soon as the status shows READY.

## 5. Run history and findings

| run | file | result | key reason |
|---|---|---|---|
| 001 | main.py | PASS | original up/hover/land |
| 002 | main.py | PASS | after moving extras out of the workspace |
| 003-006 | main.py + route.json | FAIL | route legs blocked/contact: the spawn is a tight pocket (walls ~0.53 m left/right, ~0.4 m behind) |
| 007 | stage23.py | NO_FLIGHT | PX4 did not arm (arm was sent before Offboard) |
| 008 | stage23.py | **PASS** | Offboard then arm; climb 1.2 m, 0.8 m forward, back, land; offset 0.08 m, 38 s |
| 009 | astar_run (OBJ map) | FAIL | contact 0.05 m + **estimator error p95 2.23 m** (> 1.00 m); position estimate drifted, drone reported at e=5.86 m (outside the net) |
| 010 | explore_run v1 | FAIL | contact 0.06 m; log stops after HOLD begins at t=14 s |
| 011 | explore_run v3 | FAIL | contact 0.06 m; log stops after HOLD at t=22 s; final position None |

| 012 | depth_probe.py (hover + slow yaw to -60/+60) | PASS | measured depth (below) |
| 013 | forward_run.py v1 (forward-first, map choice) | FAIL | truth clearance 0.19 m: touched the north net at the first stop; then turned east (+90), got blocked after 0.2 m, turned +30/+70 and flew back down the start corridor, STUCK 27 s |
| 014 | forward_run.py v2 | FAIL | position lost. 5th-percentile depth columns picked up stereo noise as phantom walls: "room ahead 0.07 m" in the open corridor, so it never flew forward; fallback turned it to face M1, estimate jumped, landed. Lesson: keep the 20th percentile / centre-window depth; the local 2D sim has no stereo noise |
| 015 | waypoint_run.py (preplanned lane-centre waypoints + centre-window depth stop 0.45 m), 60 s | FAIL (near miss) | truth clearance 0.26 m < 0.28. Rounded 3 corners: north, east over M1, south M1-M2 lane, east under M2, was on leg 5 (M2-M3 lane) at the 60 s timer; landed fine |
| 016 | waypoint_run.py, RUN_S = 180 (user: no other changes) | FAIL (position lost) | legs 1-2 fine (rounded the M1 corner again). While turning in place to face south at (-1.43, 2.63) the **estimate jumped 0.65 m east** (to -0.78, 2.28); the controller chased it, pushed the real drone west into M1/net (yaw snapped to -99), position lost, landed |

### BEST RUN = run_015 -> `sim/waypoint_run_best_run015.py` (60 s, md5 b86bb01c, exactly as flown)
User asked to mark it as the one to go back to. Closest so far: rounded 3 corners in 60 s, landed, only failed on 0.26 m vs 0.28 m clearance.

### Next (written, NOT yet flown): `sim/waypoint_run.py` with an estimator-jump guard
If the estimate moves > 0.15 m in one 50 ms tick it is a jump: the setpoint and remaining route are shifted by the same amount so the controller does not chase it; legs re-anchor after each turn. RUN_S = 180. The GPU lease ended (session CLOSING) before it could be pushed/flown; push it (gzip+base64 heredoc), and also copy `waypoint_run_best_run015.py` to the sim if it is not there.
Other ideas for the 0.26 m near miss: move the lane-centre waypoints 5 cm further from whichever wall was closest (needs the grader's location), or slow the corner turns.

### Session 2 findings (2026-11-07, run_012 probe and run_013)
- Depth camera FOV is **74 deg** (fx 421, cx 319, 640x400), not 87.
- **Depth sees through the net**: facing north it read 2.9-3.2 m with the net at 2.14 m. At -60 deg (net ~1.08 m, face-on) it read 1.00-1.14 m, so the net is seen **face-on at ~1 m**, not at grazing angles or far away. Solid walls (M1) are seen fine.
- Slow yaw (20 deg/s) gave 0 position-invalid ticks, but the estimate shifted ~0.2 m during turns.
- run_013: stopped at estimated y=2.54 (0.51 m from the net by estimate) but real clearance was 0.19 m, so **the estimate under-reports forward motion by ~0.3 m over 1.7 m** and the net's depth reads ~0.14 m too far (20th percentile mixes mesh + background).
- User rules now: go forward whenever possible; otherwise turn right away (only straight or 90 deg, no 30/60 deg turns) to the most open direction in the map built from depth; never outside the border; forward buffer 10 cm (STOP_M 0.38).
- New files: `sim/forward_brain.py` (pure logic), `sim/forward_run.py` (ROS wrapper), `sim/test_forward.py` (local 2D closed-loop test; my own simple model, NOT the FlyX sim: course geometry + camera from measurements, drift models are guesses).
- `stage23.py` default LEG_M changed to 1.0 locally (not yet pushed to the sim).

Evidence-based conclusions:
- **Geometry**: launch mark course (-2.51, 0.91); spawn heading is north (PX4 heading 0.00 rad); west net 0.53 m left, maze divider M1 0.54 m right, barrier 0.41 m behind, about 2.1 m open ahead (depth read 1.99-2.12 m ahead, 91-97% valid, matching the OBJ). Corridor is 1.07 m wide, drone reach 0.28 m, so only about +/-0.25 m of lateral play.
- **Depth does not reliably see the net**: the stereo-depth view near the net is mostly black (no depth). RGB sees the mesh; depth does not. It sees the open corridor and walls at range fine.
- **The estimator drifts**: p95 horizontal error 2.23 m in run_009. An absolute OBJ map pinned to the estimate is unusable; that is why the planner became depth-only/egocentric.
- **Why the logs go silent**: in `explore_run.py` v1-v3, when PX4's position became invalid (`n.pos is None`) the loop did `tick(); continue` and skipped logging; final position `None` in runs 010/011 confirms the estimate was lost. With position invalid PX4 leaves position control and the drone drifted into the wall/net. Patch v4 now logs `POSITION LOST` and lands after 1 s.
- **Goal angle is -45 deg from the first log line** even in the open start pocket (user observed the unwanted turn). Cause **UNVERIFIED**; leading suspect is the "unknown depth = obstacle at 1.0 m" rule and/or sparse hits creating a phantom block straight ahead. `depth_probe.py` was written to measure this.
- Repeated grader text "AUTO_LAND never observed ... PX4 did not report landed" is a consequence (crash/disarm), not the root cause.

## 6. Where we are and what to do next

State right now: the simulator has all four scripts (stage23, depth_planner, explore_run with patches v2-v4, depth_probe) and is compiled; local copies match. The world needs a reset before the next run (last run was a crash).

Next steps, in order:
1. Reset world, select `depth_probe.py`, run it, copy its output (lesson 4) and read: camera fx/cx and FOV, per-column ranges (bearing:range, `--` = no depth), centre-column rows, left/centre/right validity, and the planner decision with and without the unknown-as-obstacle rule. Use it to explain the -45 deg turn, then fix the real cause.
2. Fix, test offline (`python test_worst_case.py` + synthetic cases), push one file at a time, re-run `explore_run.py` (60 s).
3. Re-run `stage23.py` with `LEG_M = 1.0` to finish goal 2 exactly.
4. Consider: slower yaw, using the distance sensor for height in all modes, bounding lateral movement in the 1.07 m corridor, and detecting estimator loss earlier (valid depth falling plus the estimator health topic `/estimator/healthy`).
5. Keep run length at 60 s until the user changes it.

## 7. Useful facts

- PX4 topics: `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`, `/fmu/out/distance_sensor` (downward, field `current_distance`), `/fmu/in/{offboard_control_mode,trajectory_setpoint,vehicle_command}`. Depth: `/camera/depth/image_raw` (16UC1 mm, 640x400, about 12 Hz), `/camera/depth/camera_info`, stereo `/stereo/{left,right}/image`, cuVSLAM odometry `/visual_slam/tracking/odometry`, estimator health `/estimator/healthy`.
- ROS 2 Jazzy, `use_sim_time` false; PX4 NED local frame, heading 0 = north; EKF fed by cuVSLAM.
- Airframe reach used by the grader: 0.28 m; contact below that clearance. Net walls 3.05 m from centre; barrier spans the whole width except the tunnel on the east side.
- `refnav` (the original mission) already does depth-based obstacle stopping (`stop_m` 0.9, `min_obstacle_px` 1200, `min_valid_frac` 0.3) and route legs from `route.json` as (forward, left) metres from the start.
- Rules (README in the repo): GPS banned, preloaded map from the published layout allowed, no run data reused between runs; the real-drone chain must not rely on the simulator oracle.

## 8. Decision notes for the hybrid approach (written 2026-11-07, end of session)

### What ran in run_015 (best run, `sim/waypoint_run_best_run015.py`, 60 s)
- Start: waits for PX4 position/status, records start estimate, ASSUMES the start is the launch mark (-2.51, 0.91) facing north. Offboard -> arm -> climb to 1.0 m at 0.5 m/s.
- Route: 6 lane-centre waypoints from the OBJ (course x east, y north), converted to NED offsets from the start estimate (n = y - 0.91, e = x + 2.51):
  1 (-2.51,2.55) N 1.64 m | 2 (-1.37,2.55) E 1.14 m | 3 (-1.37,0.98) S 1.57 m | 4 (-0.25,0.98) E 1.12 m | 5 (-0.25,2.55) N 1.57 m | 6 (0.80,2.55) E 1.05 m.
  Planned clearance 0.40-0.50 m -> only 0.12-0.19 m budget for position error over the 0.28 m reach.
- Each leg: yaw in place at 25 deg/s to face the leg (wait <5 deg, max 3 s); setpoint slides along the line at 0.25 m/s, never >0.3 m ahead of the drone; leg done within 0.10 m of the waypoint.
- Depth: brake only. Centre window of the depth image (rows/cols 35-65%), 10th percentile of valid 0.35-8 m pixels; < 0.45 m -> hold; blocked 2 s -> skip leg. Never triggered (closest centre reading 0.66 m). Depth never corrected position.
- So it was pure waypoint following on the PX4 EKF (cuVSLAM-fed). Result: legs 1-4 done, 3 corners, mid leg 5 at 60 s, landed; FAIL only on truth clearance 0.26 m < 0.28 m (location unknown; check the run video).

### What was written next (`sim/waypoint_run.py`, 180 s, NOT flown)
- Same plus an estimator-jump guard: estimate moving > 0.15 m in one 50 ms tick = jump -> shift setpoint and remaining route by the same amount; re-anchor each leg after the turn. Catches sudden jumps only, not slow drift.

### Measured facts
- Depth: 640x400, ~12 Hz, 74 deg HFOV. Solid walls accurate at all angles. Net: invisible at grazing angles and at range (reads through, 2.9-3.2 m for 2.14 m), seen face-on at ~1 m, reads ~0.1-0.15 m long. Low percentiles = stereo-noise phantom walls (run_014).
- Estimator: 0.08 m (run_008, no turns); ~0.2 m shift during slow yaw (012); ~0.3 m along-track short (013, maybe a start offset); 0.65 m jump during an in-place turn (016); p95 2.23 m (009). Big errors cluster around in-place turns.
- Grader: truth clearance >= 0.28 m, estimator horizontal error p95 < 1.0 m, landing observed.
- Lane widths: start corridor 1.07, gap above M1 0.93, M1-M2 1.02, under M2 0.93, M2-M3 1.01 m (+/-0.18-0.25 m of play after the airframe).

### Where depth can anchor position on this route (only solid walls give a reliable fix)
| Leg | Wall ahead at the end | Walls beside |
|---|---|---|
| 1 N | net (unreliable) | net left, M1 right edge |
| 2 E | M2 (solid) | net left, M1 end right |
| 3 S | barrier (solid) | M1 / M2 |
| 4 E | M3 (solid) | barrier / M2 end |
| 5 N | net (unreliable) | M2 / M3 |
| 6 E | east side (tube) | net / M3 end |

### Options (cheapest first)
- A. Waypoints + centre-depth brake (+ jump guard): what ran. Relies on the estimate staying within ~0.15 m.
- B. Depth-terminated corners on solid walls: legs 2, 3, 4 end facing M2, the barrier, M3. Stop when centre depth = planned distance (~0.5 m); expected minus measured = along-track error, subtract it from the rest of the route. Legs ending at the net (1, 5, 6) keep an estimate stop with a bigger margin.
- C. Sideways centring from edge-of-view depth in lanes with solid walls on both sides (M1-M2, M2-M3), with the same offset correction. Not next to a net.
- D. Full map matching: ray-cast the 40 depth columns against the OBJ at the estimated pose (solid walls only), solve for the pose offset, apply it to the route. Most robust, most code and tuning.
- E. Gentler turns: slower yaw plus a ~1 s settle before translating (targets the turn jumps directly).
- Recommendation: next build A + B + E, then add C; D only if errors still exceed ~0.15 m.

### Still pending
- Push `waypoint_run.py` + `waypoint_run_best_run015.py` (the sim copy may not have been saved before the session closed).
- Stage 2 at exactly 1.0 m: `sim/stage23.py` has LEG_M default 1.0 locally, not yet pushed.
