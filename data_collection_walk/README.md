# Walk-through data collection

Carry the drone through the course by hand and record what it sees: both camera
eyes, the IMU, cuVSLAM's odometry, and everything the Pixhawk streams to the
Jetson. One Enter starts a pass, the next Enter stops it.

Nothing here flies. The scripts do not arm, do not publish setpoints and do not
touch `/fmu/in/*`. Props stay on, nobody arms, and the safety pilot's
transmitter stays in hand but is not used.

| File | Runs on | Purpose |
|---|---|---|
| `record_walk.sh` | Jetson host, in your SSH session | the button: label, start, stop, summary |
| `walk_bag.sh` | inside `skybeta-stack` | the `ros2 bag record` call; copied in by `record_walk.sh` |

## What gets recorded

The topic set of skybeta's `container_scripts/record_bag.sh`, plus a few extras:

- Both eyes, `/oak/left/image_raw` and `/oak/right/image_raw` (1024x640 bgr8,
  grayscale content), and both `camera_info` topics
- `/oak/imu/data`
- `/visual_slam/tracking/odometry` and `/visual_slam/status`
- `/fmu/in/vehicle_visual_odometry` (what the bridge forwards to PX4)
- Every `/fmu/out/*` topic present when the pass starts: `vehicle_local_position_v1`,
  `vehicle_attitude`, `vehicle_status_v1`, `vehicle_land_detected`, ...
- `/skybeta/state`, `/tf`, `/tf_static`

This is the live stream the Jetson receives from the Pixhawk, not the
flight controller's own SD-card ULog. Download the ULog from QGroundControl if
you want that too.

`/fmu/out/*` topics are picked up only if they exist when you press Enter. The
script checks this before you start and warns if there are none.

## Disk

Raw bgr8 eyes are about 120 MB/s, roughly 7 GB per minute. The sensors are
monochrome, so `bash record_walk.sh --mono` records the `image_mono` copies
instead: the same picture at about 40 MB/s, roughly 2.4 GB per minute. The
script prints free space before each pass and warns below 20 GB.

Bags are written on the Jetson, under `~/team_ws/walk_bags/` (the team
workspace the stack container mounts). They are not streamed to the laptop over
Wi-Fi.

## Getting it onto the drone

From the laptop, in this repo:

```bash
scp -r data_collection_walk dronehacks@<drone-ip>:~/team_ws/src/
```

`COLCON_IGNORE` keeps the team build from treating it as a package.

## Once you are in the course

1. Power the drone and put it down where the walk will start. Leave it still
   for the first minute so cuVSLAM locks its origin and heading to that spot.
2. From the laptop, SSH in (`ssh dronehacks@<wifi-ip>`, or
   `ssh dronehacks@192.168.55.1` over the USB-C cable). Run `hostname` and
   confirm it is your drone.
3. Confirm the stack is up: `docker exec -it skybeta-stack tmux attach -t skybeta`,
   glance at the `slam` window, then detach with Ctrl-b d.
4. Start the recorder: `bash ~/team_ws/src/data_collection_walk/record_walk.sh`
   (add `--mono` to save disk). Read the topic check it prints, then type a
   label for this pass (which lap, lighting, where the pad is). It waits. Do
   not press Enter yet.
5. Pick the drone up only when you are ready to walk. Press Enter. Recording
   starts. Hold it steady for a few seconds before moving.
6. Walk the course at a slow walking pace, cameras uncovered, holding the drone
   the way you want it to see the course (level, at flight height, facing the
   direction of travel). Pass over the landing pad the way a landing would.
   Set the drone down if you need to pause; the bag keeps recording. The
   script prints elapsed time and bag size every 15 s.
7. Press Enter again (or Ctrl-C) when the walk is finished. The script closes
   the bag cleanly and prints its path, size and `ros2 bag info`.
8. Repeat from step 4 for another pass (different lighting, a second lap).
   Each start/stop pair is its own bag.

If the SSH session drops mid-walk, the recording keeps going: it runs detached
inside the stack container. Reconnect and run
`bash ~/team_ws/src/data_collection_walk/record_walk.sh --stop`. Use `--status`
to see whether a pass is still recording and how big it is.

## After the walk

Each pass is one folder:

```
~/team_ws/walk_bags/walk-<timestamp>/
  bag/          mcap bag
  record.log    recorder output, including the topics it subscribed to
  WALK.txt      label, start/stop time, duration, hostname, ros2 bag info
```

Check the pass before you leave the course. In the `ros2 bag info` the script
prints (also at the end of `WALK.txt`), `/oak/left/image_raw` (or
`/oak/left/image_mono`) and `/fmu/out/vehicle_local_position_v1` must both show
`Count` above zero. If the camera has messages and the Pixhawk topic has none,
the flight controller link was down: the camera data is still usable, but
there is no Pixhawk track for that pass.

Copy one pass to the laptop:

```bash
scp -r dronehacks@<drone-ip>:~/team_ws/walk_bags/walk-<timestamp> .
```

At 7 GB/min this is slow over Wi-Fi; the USB-C link (`192.168.55.1`) is
faster. Delete passes from the drone once they are safely copied
(`rm -rf ~/team_ws/walk_bags/walk-<timestamp>`).

To look at a pass: `ros2 bag play bag/` in any ROS 2 Humble environment with
`px4_msgs` available, or open `bag/*.mcap` in Foxglove.
