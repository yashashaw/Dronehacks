# course_pipeline

Landing-pad detection and guidance for the skybeta drone, plus a placeholder planner. Read `CONTRACT.md` first: it is the integration sheet.

Status: written against the repo's docs and source, not yet run on a drone. Nothing here has flown.

## Put it on the drone

Team code lives in `~/team_ws/src` on the drone. Copy the package there:

```bash
cp -r course_pipeline ~/team_ws/src/course_pipeline
```

Point the stack's `competitor` window at it by changing only these two lines in `/etc/skybeta/drone.env`:

```
COMPETITOR_PKG=course_pipeline
COMPETITOR_LAUNCH=course.launch.py
```

Then restart the stack on the ground, with the drone still: `docker rm -f skybeta-stack`. The launch file starts `flight_interface` with `auto_offboard_on_arm:=true` and the three nodes here. Do not run another `flight_interface` or any other publisher of `/fmu/in/*`.

## YOLO weights

The detector runs without weights and reports no pad. To use a model, set `weights_path` (a path inside the container, for example under `/root/team_ws`):

```bash
ros2 launch course_pipeline course.launch.py weights_path:=/root/team_ws/pad.pt
```

`ultralytics` is not in the image. Add it through `~/team_ws/Dockerfile` (`host/run.sh build --init`, then `host/run.sh build`), never with `pip install` in a running container. It needs a PyTorch build that works on the Jetson's JetPack; that has not been checked here.

## Checks without a drone

The pure modules import no ROS:

```bash
cd course_pipeline
python -m pytest test -q
```

## First run

Props off. Check on the bench that `/pad/detection` publishes, that `/skybeta/setpoint/pose` appears from startup, and that `clamped` in `/skybeta/state` stays empty. Then low and tethered, with the safety pilot on RC, as the skybeta rules require.
