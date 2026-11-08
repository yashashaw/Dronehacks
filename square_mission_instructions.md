# Flying the square mission

The square mission is the example node already on the drone, at
`~/skybeta/src/competitor_example/competitor_example/square_mission.py`. To fly
it, copy the ROS package `~/skybeta/src/competitor_example` (not the whole
`~/skybeta` tree) into `~/team_ws/src`, install it with `team_build.sh` so ROS
can launch it, and tell the stack to start it at power-up. Do not replace
`square_mission.py`.

This needs skybeta **0.3.3 or later** (`cat ~/skybeta/VERSION`). Drone 18 is
already on 0.3.3. On 0.3.2 and earlier, PX4 ignores the Offboard request and
the drone stays in Position.

The rules behind these steps are in `autonomy_pipeline_md.md`. The ordered
checklist is [Prepare the drone](#prepare-the-drone). Do that before the
flight.

## What the square mission does

`square_mission.py` is a state machine that publishes position setpoints. It
never arms the drone and never talks to PX4 directly.

Once the pilot switches to Position, it does this:

1. **Hold** the current position until Offboard is active.
2. **Climb** to 1.0 m above the start point and hover there for 5 seconds.
3. **Fly a 0.5 m square** at that height, counter-clockwise, starting from where
   Offboard began: 0.5 m forward, 0.5 m left, 0.5 m back, then back to the start
   corner. A corner counts as reached within 0.3 m. If a corner is not reached
   in 60 seconds, it gives up on that corner.
4. **Hold** the last corner. It does not command yaw, so the nose stays
   wherever PX4 leaves it.

Positions are in metres in the vision frame. The origin is where the drone was
when the stack started, +x is the direction the nose pointed then, +y is left,
and +z is up. The flight interface floors height at 0.3 m and caps it at 3.0 m,
so the drone climbs as soon as Offboard starts.

The square is 0.5 m on a side. A corner counts as reached at 0.3 m, so the
drone turns before it is exactly on the corner.

## What has to be true when the pilot switches

Arming does not start the code. The code must already be running and
publishing. At the moment of the switch:

1. Exactly one `flight_interface` is running with `auto_offboard_on_arm: true`.
   The example launch file already sets that.
2. The square node has published a setpoint in the last second.
3. The drone is on the ground, vision tracking has a valid position, and the
   pilot armed on Altitude before switching to Position.

If any of those is false, the drone stays in Position under the pilot.

```
square_mission ──/skybeta/setpoint/pose──▶ flight_interface ──/fmu/in/*──▶ PX4
    ▲                                          │  (clamps, ENU→NED, 10 Hz heartbeat)
    └──────────────── /skybeta/state ◀─────────┘  (offboard_active, armed, landed ...)

pilot: arm (switch on Altitude) ──▶ switch to Position ──▶ flight_interface asks PX4 for Offboard
                                                        ──▶ PX4 follows the square setpoints
```

## Prepare the drone

Do this on the ground before anyone is watching. When the last step checks
out, every power-up builds and starts the square mission by itself. A node
started by hand in a shell does not survive a restart, so the demo has to
come from this setup.

The commands use the package name `my_team_nav`. Any name of lowercase
letters and underscores is fine; use the same name in every step.

### 1. Log in

```bash
ssh dronehacks@skybeta-18.local
cat ~/skybeta/VERSION
```

The version must be **0.3.3 or later**. Drone 18 is already on 0.3.3.

### 2. Copy the package into `~/team_ws/src`

`~/team_ws` is the only code directory that survives restarts and updates.
Copy `~/skybeta/src/competitor_example` only, not the rest of `~/skybeta`.

```bash
mkdir -p ~/team_ws/src && cd ~/team_ws/src
cp -r ~/skybeta/src/competitor_example my_team_nav
cd my_team_nav
mv competitor_example my_team_nav
mv resource/competitor_example resource/my_team_nav
sed -i 's/competitor_example/my_team_nav/g' package.xml setup.py setup.cfg launch/competitor.launch.py
```

Leave `square_mission.py` and `launch/competitor.launch.py` as they are. The
copy already hovers at 1.0 m and flies a 0.5 m square. The launch file starts
`flight_interface` with `auto_offboard_on_arm: true` and starts
`square_mission`.

### 3. Leave `~/team_ws/src` able to build

On every stack start the competitor window builds **all** of `~/team_ws`. If
any package fails, it prints `team_ws build failed` and launches nothing, so
there is no takeover. Move unfinished packages out of `src/` before the build
below.

### 4. Build it and check the launch file

From the drone, not from inside a container yet:

```bash
cd ~/skybeta && ./host/run.sh shell
```

That shell is inside the stack container. Then:

```bash
team_build.sh --packages-select my_team_nav
ros2 launch my_team_nav competitor.launch.py --show-args
```

Stop here if the build fails or `--show-args` does not list the launch
arguments (`hover_altitude_m` default `1.0`, `square_side_m` default `0.5`).
Exit the container shell when both succeed.

### 5. Start this launch at power-up

```bash
sudoedit /etc/skybeta/drone.env
```

Set only these two lines:

```
COMPETITOR_PKG=my_team_nav
COMPETITOR_LAUNCH=competitor.launch.py
```

Do not change `DRONE_ID`, `ROS_DOMAIN_ID`, or `TEAM` in that file.

### 6. Restart the stack

The drone must be on the ground, disarmed, and still. A restart re-zeroes the
position origin. Unplug the Pixhawk USB first; PX4 will not arm while it is
plugged in.

```bash
docker rm -f skybeta-stack
```

The service starts the stack again in about 10 seconds. After this restart,
power-cycle the Pixhawk. On the stock firmware the telemetry link stays dead
until the flight controller is power-cycled, and reseating the cable does not
fix it.

### 7. Confirm it is ready to show

```bash
docker exec -it skybeta-stack tmux attach -t skybeta
```

Open the `competitor` window. It should show the square mission waiting for
the pilot, not `team_ws build failed`. Leave tmux with `Ctrl-b` then `d`.

You are ready for the flight section when all of these are true:

- `~/team_ws/src/my_team_nav` is the package that was built.
- `/etc/skybeta/drone.env` points at `my_team_nav` and `competitor.launch.py`.
- The stack came back after the restart, and the competitor window is waiting.
- The Pixhawk USB is unplugged, and the flight controller has been power-cycled.

## The flight

1. Power on in the spot you will fly from, and keep the drone still for the
   first minute so vision tracking can settle.
2. The safety pilot turns the transmitter on, checks the kill switch, and
   stands clear.
3. The pilot arms with the mode switch on **Altitude** and the throttle low.
4. Within a few seconds, the pilot switches to **Position**. PX4 disarms on
   its own if the drone has not taken off within 10 seconds of arming. The
   square mission then climbs, hovers, and flies the square.
5. To take control back, the pilot raises the throttle to centre or moves a
   stick. The drone returns to Position under the pilot, and the pilot lands
   it. The mission gets one run per arming. It does not resume until the drone
   is disarmed and armed again.
