# Making your code fly when the pilot switches to Position

This is the step-by-step path from an empty workspace to: **the pilot arms, switches to
Position, and your code flies the drone.** The rules behind it are in the
[competitor guide](competitor_guide.md) §4 (the procedure) and §5 (running code); this page is the
checklist.

Needs skybeta **0.3.3 or later** on the drone (`cat ~/skybeta/VERSION`). On 0.3.2 and earlier, PX4
ignores the Offboard request and the drone just stays in Position. Ask the organisers to update.

## How it works, in one picture

```
your node ──/skybeta/setpoint/pose or /twist──▶ flight_interface ──/fmu/in/*──▶ PX4
    ▲                                              │  (clamps, ENU→NED, 10 Hz heartbeat)
    └──────────────── /skybeta/state ◀─────────────┘  (offboard_active, armed, landed ...)

pilot: arm (switch on Altitude) ──▶ switch to Position ──▶ flight_interface asks PX4 for Offboard
                                                            ──▶ PX4 follows your setpoints
```

Arming does **not** start your code. Your code must **already be running and publishing** when the
pilot flips to Position. The switch only hands PX4 over to whatever `flight_interface` is
forwarding at that moment.

So three things must be true at the moment of the switch:

1. **`flight_interface` is running with `auto_offboard_on_arm: true`**, exactly one copy of it.
2. **Your node published a setpoint in the last second.**
3. The drone is on the ground, VIO is tracking (valid position), and the pilot armed **before**
   switching to Position.

If any is false, nothing bad happens: the drone stays in Position under the pilot, and
`flight_interface` logs why (see [Troubleshooting](#troubleshooting)).

## Step 1 — make your package in `~/team_ws/src`

Your code lives in `~/team_ws/src` on the drone. Nothing else survives restarts and updates. Copy
the example and rename it (replace `my_team_nav` with your own name, lowercase and underscores):

```bash
# on the drone, in a normal SSH shell
mkdir -p ~/team_ws/src && cd ~/team_ws/src
cp -r ~/skybeta/src/competitor_example my_team_nav
cd my_team_nav
mv competitor_example my_team_nav
mv resource/competitor_example resource/my_team_nav
sed -i 's/competitor_example/my_team_nav/g' package.xml setup.py setup.cfg launch/competitor.launch.py
```

You now have a working package that flies the example square. Replace `square_mission.py` with
your own node (step 2), or keep it and edit it.

## Step 2 — write your node to this contract

| Before Offboard (`offboard_active` false) | After Offboard starts | When Offboard ends |
|---|---|---|
| Publish a setpoint **at least once a second** (10 Hz is best): your current position is the safe choice | Fly your mission: publish poses (waypoints) or twists (velocities) | The pilot has taken over. Stop the mission. It does not come back until the drone is disarmed and armed again |

What you publish:

| Topic | Type | Meaning |
|---|---|---|
| `/skybeta/setpoint/pose` | `geometry_msgs/PoseStamped` | Go to (x, y, z) and hold. Stays in force until replaced. All-zero orientation = don't command yaw |
| `/skybeta/setpoint/twist` | `geometry_msgs/TwistStamped` | Move at (vx, vy, vz) m/s in the **world** axes (not the body), `angular.z` = yaw rate. **Expires after 1 s** unless refreshed, then the drone holds. Capped at 1.5 m/s |

What you read:

| Topic | Type | Meaning |
|---|---|---|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | Where the drone is |
| `/skybeta/state` | `std_msgs/String` (JSON) | `offboard_active`, `armed`, `landed`, `clamped`, ... |

**Frame:** ENU metres in the vision frame. The origin is where the drone was when the stack
started, +x is the direction its nose pointed then, +y its left, +z up. Not compass north. Height
is clamped to 0.3–3.0 m above the start point.

**It lifts off the moment Offboard starts.** Every pose is floored at 0.3 m, so even "hold where you
are" climbs to 0.3 m. And if the drone has not taken off 10 s after **arming**, PX4 disarms it
(`COM_DISARM_PRFLT`), so the pilot should switch to Position within a few seconds of arming.

If your node stops publishing or crashes in the air, the drone does **not** fall out of Offboard:
`flight_interface` keeps streaming to PX4 and holds the last pose (or holds position once a twist
expires). It hovers until the pilot takes over.

Never publish to `/fmu/in/*` yourself, and never run a second `flight_interface`.

A minimal node that does all of the above: holds where it is until Offboard, then flies three
waypoints relative to where Offboard started, then holds. Save it as
`~/team_ws/src/my_team_nav/my_team_nav/my_mission.py`:

```python
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ENU metres, relative to where the drone was when Offboard started.
WAYPOINTS = [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0)]
ARRIVED_M = 0.3


class MyMission(Node):

    def __init__(self):
        super().__init__('my_mission')
        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.position = None     # current ENU position
        self.offboard = False
        self.start = None        # ENU position when Offboard started
        self.index = 0
        self.create_timer(0.1, self.tick)   # 10 Hz

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def on_state(self, msg):
        self.offboard = bool(json.loads(msg.data).get('offboard_active', False))

    def send(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # orientation left all-zero: do not command yaw
        self.pub.publish(msg)

    def tick(self):
        if self.position is None:
            return                      # no VIO yet: publish nothing
        if not self.offboard:
            if self.start is not None:
                self.get_logger().warn('Offboard ended: the pilot has it. Mission reset.')
                self.start, self.index = None, 0
            self.send(*self.position)   # keeps a fresh setpoint: required for the takeover
            return
        if self.start is None:
            self.start = self.position
            self.get_logger().info('Offboard active: mission start')
        dx, dy, dz = WAYPOINTS[self.index]
        target = (self.start[0] + dx, self.start[1] + dy, self.start[2] + dz)
        self.send(*target)
        if math.dist(self.position, target) < ARRIVED_M and self.index < len(WAYPOINTS) - 1:
            self.index += 1
            self.get_logger().info(f'waypoint {self.index}: {WAYPOINTS[self.index]}')


def main():
    rclpy.init()
    node = MyMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                            # Ctrl-C or the stack stopping
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Register it in `setup.py`, in `console_scripts`:

```python
'my_mission = my_team_nav.my_mission:main',
```

## Step 3 — a launch file that starts `flight_interface` with takeover on

The stack launches **one launch file** of yours, and that file must start `flight_interface` too.
The copied `launch/competitor.launch.py` already does; to use your own node instead of the square,
change its `competitor` node's `executable='square_mission'` to `executable='my_mission'` (and drop
the square's parameters). Or save this minimal one as
`~/team_ws/src/my_team_nav/launch/my_mission.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{'auto_offboard_on_arm': True}]),   # without this: no takeover
        Node(package='my_team_nav', executable='my_mission',
             name='my_mission', output='screen', emulate_tty=True),
    ])
```

`auto_offboard_on_arm` defaults to **false** in `flight_interface` itself, so a launch file that
starts it without that parameter never takes over.

## Step 4 — build it

Inside the stack container (`host/run.sh shell` attaches to it):

```bash
team_build.sh --packages-select my_team_nav
ros2 launch my_team_nav my_mission.launch.py --show-args   # launch file found and parses
```

**The build must succeed.** At every stack start the `competitor` window builds the whole of
`~/team_ws`; if any package in it fails, it prints `team_ws build failed` and launches **nothing**,
so there is no `flight_interface` and no takeover. A half-finished package anywhere in
`~/team_ws/src` breaks it. Move unfinished work out of `src/`.

## Step 5 — make the stack launch it at power-up

```bash
sudoedit /etc/skybeta/drone.env
#   COMPETITOR_PKG=my_team_nav
#   COMPETITOR_LAUNCH=my_mission.launch.py
docker rm -f skybeta-stack      # restarts the stack; back in about 10 s
```

Restart only with the drone **on the ground, disarmed and still**: a restart re-zeroes the
position origin. From now on every power-up builds and launches your code by itself, so it is
running before the pilot arms. Code you start by hand in a shell does **not** survive a restart or
power cycle; only this does.



Then disarm. PX4 refuses to arm while the Pixhawk's USB is plugged in, so unplug it for this test.

## The flight

1. Power on in the spot you will fly from, and keep the drone still for the first minute.
2. Safety pilot: transmitter on, kill switch checked, standing clear.
3. Pilot **arms** with the mode switch on **Altitude**, throttle low.
4. Within a few seconds, pilot switches to **Position**. Your code takes off and flies.
5. To take back control: **raise the throttle to centre** (or move a stick). The drone goes to
   Position under the pilot. The pilot lands it. Your code gets one go per arming.
## Working by hand during development

To iterate without restarting the stack: attach to the `competitor` window and Ctrl-C it (this stops
its `flight_interface` too). Then in `host/run.sh shell`:

```bash
team_build.sh --packages-select my_team_nav
ros2 launch my_team_nav my_mission.launch.py
```

This is fine for bench work, but it dies with the stack. Before handing the drone to the pilot for a
real run, do step 5 so the code starts by itself