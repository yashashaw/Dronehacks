# Planner: Theta\* with frontier exploration (simulated)

Status: a working planner **tested only in simulation**. Depth is simulated (the drone does not
publish depth yet) and the noise numbers are placeholders until measured on the drone. Nothing here
has flown.

## What it does

The drone starts with no map. Each frame it turns range returns (distance at each bearing, from the
depth camera) plus its estimated pose into an occupancy grid that grows as it explores, picks a goal
at the edge of known free space (or the landing pad once seen), plans a route with Theta\*, and
steers along it, slowing where clearance is small or the path turns sharply. If it stops making
progress it backs off and plans with a little more clearance. No waypoints, coordinates or stage
timings are in the flight-side code.

## Files

Flight side (portable; consume only range returns, estimated pose, a pad cue):

| File | Role |
|---|---|
| `theta_star.py` | Any-angle grid search with clearance cost. Exact distance map (scipy); search compiled with numba when available, pure-Python fallback otherwise. |
| `mapping.py` | Log-odds occupancy mapper, grows as needed. |
| `navigator.py` | Frontier goals, Theta\* planning, path following, stuck recovery. Output: velocity command in the estimate frame. |

Test only (never import from flight code):

| File | Role |
|---|---|
| `course_fixture.py` | Builds the simulated world from `dronehacks-course-v3.obj` as height layers. |
| `sensing.py` | Simulated depth scan, drifting pose estimate, delay, vehicle with prop footprint. |
| `run_closed_loop.py` | One simulated flight. |
| `summary.py`, `run_sweep.py`, `ablate.py` | Batches of flights; sweeps over noise, delay, drift, altitude, speed. |
| `run_course_test.py` | Planner alone on the perfect course map. |

## Running

Needs numpy and scipy; numba is optional but makes planning about 80x faster. In the `rover` conda env:

    cd planner
    python summary.py 50        # 50 flights, one-line summary
    python run_sweep.py 20      # full table of conditions
    python run_course_test.py   # planner on the perfect map

## Inputs it is built around (drone 7)

Camera calibration (78 x 54 degree FOV, 7.5 cm baseline, 1024x640), camera mount, prop footprint
(about 0.50 x 0.41 m with 7 in props), PX4 limits, and measured position delay. Numbers with no
measurement behind them (depth noise and dropouts, drift growth, altitude hold, how a prop strike is
judged) are guesses and are marked in `sensing.py`.

## Known limits

- Drift is the weak point: at about 4x the assumed drift, roughly 1 flight in 6 fails.
- Flight altitude is a fixed 0.76 m, chosen to fit the window; it is not chosen from perception.
- Checkpoint lines and the floor line are not used or scored.
- Pad detection is idealised in the simulator.

## Credits

Theta\*: Daniel, Nash, Koenig, Felner, "Theta\*: Any-Angle Path Planning on Grids" (2010).
Libraries: numpy, scipy, numba. Written with AI assistance (Claude); the team is responsible for
understanding and explaining it.
