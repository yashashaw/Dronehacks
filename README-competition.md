# Stanford DroneHacks 2026 — Challenge Reference

> **Team project index: see [`team_project/`](team_project/README.md) for the code that flew, the CV runs and a map of every branch.**

> Program the issued drone to fly an indoor obstacle course **autonomously, as fast as possible, with no GPS and no pilot input.**
> Nov 6–8, 2026 · Drone net in Tresidder Oak · Questions: hello@stanfordhorizons.com

| | |
|---|---|
| **Course** | Indoor, GPS-denied, 20 × 20 × 10 ft net |
| **Score** | 100 = 60 run + 25 presentation + 15 generality (+25% return bonus on run points) |
| **Prizes** | 5 awards, $15,000 total |
| **Scored runs close** | Sun 4:00 PM PST |

---

## Table of contents
1. [The drone](#the-drone)
2. [Software stack](#software-stack)
3. [The course](#the-course)
4. [Checkpoints](#checkpoints)
5. [Scoring](#scoring)
6. [Awards](#awards)
7. [Timeline](#timeline)
8. [Rules (condensed)](#rules-condensed)
9. [Open questions](#open-questions)

---

## The drone

Every team flies the same drone ("Sky Beta"), pre-built, tuned and flight-tested.

**Data flow:** Camera sees → Jetson plans → Pixhawk stabilizes → ESCs drive → Motors make thrust.
The Jetson sends position/velocity targets to the Pixhawk; the Pixhawk does the fast stabilizing corrections.

| Part | Role | Key specs |
|---|---|---|
| **NVIDIA Jetson Orin Nano 8GB** | Companion computer — all our code runs here | 1024-core Ampere GPU, 32 tensor cores · 6-core Cortex-A78AE · 8 GB LPDDR5 (68 GB/s) · 40 TOPS INT8 (67 in Super mode) |
| **Holybro Pixhawk 6C** (PX4) | Flight controller: attitude, rates, safety envelope | STM32H743 @ 480 MHz · 2 MB flash / 1 MB RAM · dual IMU (ICM-42688-P + BMI055) · BMP388 baro |
| **Luxonis OAK-D S2** | Perception: stereo depth + on-device inference | 12 MP IMX378 colour (4056×3040) · 2× 1 MP global-shutter mono stereo · Myriad X VPU (4 TOPS) · metric depth from ~0.35 m |
| **ST VL53L1X** (Holybro) | Downward ToF rangefinder → height above floor | Up to 4 m (less in bright light) · up to 50 Hz · 27° FoV (narrower with ROI) · 2.4 g |

CAD (STEP of the assembled drone + OAK-D mount STL, mm, +Z up) is available from the challenge site for designing brackets/mounts.

---

## Software stack

Pre-installed on every drone:

| Component | Where | Notes |
|---|---|---|
| PX4 Autopilot | Pixhawk 6C | Stock firmware, pre-flashed and tuned |
| ROS 2 Humble | Jetson | PX4 messages + bridged uXRCE-DDS agent |
| DepthAI + depthai-ros | Jetson | ROS 2 topics for RGB, depth, detections |
| Starter repo | Laptop + Jetson | Offboard control node, depth-to-obstacle example, launch files, Gazebo sim |

**Workflow**
1. Write ROS 2 nodes for the Jetson; test in Gazebo on a laptop first.
2. Read colour/depth/detections from the OAK-D topics (depth is computed on the camera).
3. Fly via PX4 **offboard mode**: send position or velocity setpoints, read back the estimated state.
4. Build on the starter project's flying baseline.

---

## The course

- Fills a **20 × 20 ft drone net, 10 ft tall**. Walls are **68 in tall, 4 in thick**. Boulders are **18 in across**.
- Layout files: OBJ+MTL (zip), STEP, GLB — in metres; OBJ/STEP are Z-up, GLB is Y-up.
- **Layout is preliminary.** Changes are posted on the site and in Slack before scored runs open. The drone must still localize and react to what it sees.

**Stages, in order, from the launch mark:**
1. **Maze** — navigate the corridors; a line on the floor marks the route.
2. **Tunnel** — fly through the dark tunnel, the only way through the barrier.
3. **Chicane** — enter the landing area via **Gate B** (north end) or the **window** in the lower wall (window = +5 on a finished run).
4. **Landing** — land and disarm with the whole drone inside the pad's outer ring.
5. **Bonus return** — take off again, fly back over the barrier, land within 2 ft of the launch mark.

**Constraints**
- Touching a wall, the tunnel or a boulder: **+5 s** per touch. A crash ends the run.
- **Stay below the walls:** some part of the frame must always be below 68 in (except on the return). A checkpoint reached by going over a wall or the barrier earns only **15%** of its points.

**Using the layout:** preloading a map built from the published layout is allowed. Hard-coding the course is allowed but costs generality points. Storing data from one run and feeding it into a later run is **not** allowed (rule 6.4).

---

## Checkpoints

The whole drone must cross a checkpoint's line. Checkpoints count only in order.

| # | Checkpoint | Cleared when | Points |
|---|---|---|---|
| 1 | First corridor | Reaches the end of the first corridor, where the floor line turns east | 4 |
| 2 | Maze | Leaves the maze where the line ends, through Gate A | 7 |
| 3 | Tunnel | Comes out of the south end of the dark tunnel | 10 |
| 4 | Chicane | Enters the landing area through Gate B or the window, without landing | 7 |
| 5 | Landing | Lands and disarms with the whole drone inside the pad's outer ring | 7 |
| | | **Total** | **35** |

---

## Scoring

| Part | Points | What counts |
|---|---|---|
| The run | 60 | How far the drone gets, then how fast |
| Technical presentation | 25 | Autonomy design, execution, originality, evidence of testing/iteration, use of auxiliary sensors |
| Generality | 15 | How well the software would work on a different course (judged from Q&A + AI code review) |
| Return bonus | +25% | Of run points, for the return to the launch mark |

### Run points

**Unfinished run**
```
checkpoint points reached + 7 × (fastest time to your last checkpoint / your time to it)
```
Reaching no checkpoint = 0. Clearing one more checkpoint always beats clearing fewer faster.

**Finished run**
```
checkpoint points (35 if all below walls) + 20 × (fastest finish / your finish) + 5 if via window
```
Every below-the-walls finish beats every unfinished run.

**Return bonus:** finished run + completed return → run points × 1.25.

**Over the walls:** affected checkpoints earn 15%; that run can never set the fastest reference time and can't win Fastest Run.

**Timing:** from leaving the launch mark, including touch penalties; compared against the fastest time to the same checkpoint by any team, by either route. Ties go to the faster run time.

### Worked examples (fastest finish = 60 s)

| Scenario | Run points |
|---|---|
| Finish in 90 s via Gate B | 35 + 20 × 60/90 = **48.3** |
| Finish in 90 s via window | **53.3** |
| Window + return | 53.3 × 1.25 = **66.7** |
| Reach landing area in 50 s (fastest 45 s), crash on landing | 28 + 7 × 45/50 = **34.3** |
| Only exit the maze, 40 s (fastest 30 s) | 11 + 7 × 30/40 = **16.3** |

---

## Awards

| Award | Prize | For |
|---|---|---|
| Rothberg Catalyzer Award | $7,000 | Highest overall score |
| Runner-Up | $3,000 | Second-highest score |
| Third Place | $2,000 | Third-highest score |
| Most Technologically Impressive | $2,000 | Presentation + generality scores only |
| Fastest Run | $1,000 | Fastest finished run below the walls, penalties included |

Prizes are per team.

---

## Timeline (PST)

| When | What |
|---|---|
| Fri 4:45 PM | Hacking begins (end of opening ceremony) |
| Fri 7:50 PM | Oak opens: flight-net safety briefing + kit pickup |
| Saturday | Build and practice in the nets |
| Sat 7:30 PM | Scored-run blocks and presentation slots posted in Slack |
| Sat 10 PM – Sun 2 AM | Scored runs (first half) |
| Sun 8 AM – 4 PM | Scored runs (second half), then forfeited time |
| **Sun 12:30 PM** | **Repo link due** |
| Sun 12:30 – 3:10 PM | Technical presentations (two panels in parallel) |
| **Sun 4:00 PM** | **Scored runs close; all flown code must be pushed** |
| Sun 4:30 – 5:30 PM | Kit return |
| Sun 6:00 PM | Closing ceremony + awards, Simonyi |

---

## Rules (condensed)

Full rulebook is on the challenge site. Organizers may amend rules during the event (announced in Slack). Head judge's decisions are final. When in doubt, ask before doing it.

### Kit & hardware (rules 2–3)
- One kit per team, signed out by the captain, returned complete Sun 4:30–5:30 PM.
- Only fly the issued drone, propellers and batteries.
- Core kit (frame, motors, ESCs, props, power, Jetson, Pixhawk, OAK-D, VL53L1X) must stay on and in use — no removing, replacing or bypassing.
- **Re-mounting is allowed** (move/re-angle camera or rangefinder, print new brackets) — requires re-inspection.
- No changes to motors, ESCs, props, battery or power wiring. **No additional computers.** Nothing else that draws power except auxiliary sensors.

### Auxiliary sensors (rule 4)
- **Allowed**, each **< US$100 market value** and **< 50 g** (sensor as sold, incl. breakout/MCU needed to read it; cables/mounts excluded).
- Market value = normal retail price new, not what we paid (a free $129 sensor is over the limit).
- Declare at inspection: part number, price listing link, mass. Undeclared sensors can't fly.
- Powered from the drone battery only.
- They earn nothing by themselves; judges consider **how well we use them** in the presentation.
- **Banned at any price:** GNSS, anything relying on off-drone equipment (UWB anchors, beacons, mocap), lasers above Class 1.

### Autonomy (rule 5)
- From the start command to the end: one start command, then **abort only** (kill or land).
- No RC inputs, waypoints, mode changes or parameter changes mid-run. RC transmitter = kill switch only.
- All perception, localization, planning and control on the drone. No laptop/server/cloud in the loop; laptop may monitor read-only.

### Localization & mapping (rule 6)
- No GNSS, even as fallback; PX4 GNSS input stays disabled.
- No external positioning (mocap, UWB, Wi-Fi/BT ranging, beacons, markers) unless provided by organizers.
- Don't place, move, mark or alter anything on the course.
- **Preloaded maps OK** (including from the published layout). **Run data not OK:** no splats/point clouds/maps captured on one run and fed to a later one, no flight replays. Tuning code from what we learned is fine.

### The run (rule 7)
- Equal net time per team, in two blocks (one per half), 1.5 min changeover. Be at the net, charged, 10 min early — blocks start on time.
- Fly as many runs as fit; **best run counts**. A run still airborne at block end is scored on checkpoints cleared.
- Can forfeit the rest of a block; remaining time (minus 1.5 min) is added after Sunday's last block if it fits before 4 PM.
- Clock: leaves launch mark → disarms on pad (or run ends). Filmed from above; checkpoint times read from video.
- Each non-ending touch: +5 s to finish and every later checkpoint.
- **Run ends on:** disarm on pad (no return attempt), crash, ground contact elsewhere, leaving the course, **3-minute limit**, team abort, referee stop, or any human input other than abort.
- **Return (7.10):** after disarming on the pad, take off autonomously, fly over the barrier, land and disarm within 2 ft of the launch mark, all within 3 min. Failing the return loses only the bonus.

### Safety (rule 8)
- Everyone attends the safety briefing before flying.
- Fly only in the closed net; never arm elsewhere; props off at the table.
- A designated teammate watches every flight, ready on the kill switch. Kill immediately when an organizer says so.
- Inspection before first flight and after any hardware change.
- Don't touch the organizer's PX4 safety settings (failsafes, kill switch, geofence, battery limits).
- Issued batteries only, charged only at the charging station, never unattended.

### Software (rule 9)
- Built during the event. Open-source libs, pretrained models, public datasets and the starter project are fine — **credit them in the README and presentation**.
- AI coding tools are allowed; we must be able to explain how our code works.
- Repo link due **Sun 12:30 PM**; everything flown pushed by **Sun 4:00 PM**.
- Repo needs a README explaining the stack and how to run it.
- An AI code review flags layout-specific hard-coding (fixed waypoints, routes, timings) for the judges, who weigh it in generality.

### Fair play (rule 10)
- Sharing ideas is fine; copying another team's code is not.
- No interfering with other teams' drones, laptops, networks or radios.
- No tampering with the course, timing or organizer equipment.
- Misstating sensor price/mass, flying undeclared hardware, or flying code not in the repo by 4 PM = cheating.

---

## Open questions

- **Extra sensors:** rule 4 *does* allow them (< $100, < 50 g each, declared). They don't score points by being on the drone, but judges reward using them well.
- **RL in the provided Gazebo sim:** is it fast/realistic enough for training, or is a classical stack (VIO + map-based planning) the better bet?
- **If not RL, what's the alternative?**
