"""The pad guide: the ONLY publisher of /skybeta/setpoint/pose in this package.

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)
    subscribe  /skybeta/state                   offboard_active, armed, landed
    subscribe  /course/goal                     where the planner wants to go
    subscribe  /pad/detection                   from pad_detector
    publish    /skybeta/setpoint/pose           ENU position, no yaw

The stage logic is in ``guide_machine.py``; this node only moves messages in
and out. It never talks to /fmu/*, never arms, never changes mode.

It publishes a setpoint from startup, including while disarmed: the flight
interface enters Offboard only if one arrived in the last second. A pose is
floored at 0.3 m by the interface and is not speed limited, so the machine only
ever asks for a pose a short step from the vehicle.
"""
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from course_pipeline.guide_machine import GuideConfig, GuideMachine


class PadGuide(Node):
    def __init__(self, **kwargs):
        super().__init__('pad_guide', **kwargs)

        defaults = GuideConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, float(value))
        self.declare_parameter('publish_rate_hz', 10.0)

        config = GuideConfig(**{
            name: float(self.get_parameter(name).value)
            for name in vars(defaults)})
        self.machine = GuideMachine(config)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.odometry_cb, 10)
        self.create_subscription(
            String, '/skybeta/state', self.state_cb, 10)
        self.create_subscription(
            PoseStamped, '/course/goal', self.goal_cb, 10)
        self.create_subscription(
            String, '/pad/detection', self.detection_cb, 10)

        self.log_once = set()
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            'pad_guide ready. Waiting for the safety pilot to arm, then '
            'switch to Position.')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info_once(self, key, message):
        if key not in self.log_once:
            self.log_once.add(key)
            self.get_logger().info(message)

    # -- inputs ----------------------------------------------------------
    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        self.machine.on_position((p.x, p.y, p.z))

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.machine.on_state(
            state.get('offboard_active', False),
            state.get('armed'), state.get('landed'))
        if state.get('clamped'):
            self.info_once(
                f'clamped:{",".join(state["clamped"])}',
                f'the flight interface is CLAMPING our setpoints '
                f'({", ".join(state["clamped"])}); envelope '
                f'{state.get("envelope")}')

    def goal_cb(self, msg):
        p = msg.pose.position
        self.machine.on_goal((p.x, p.y, p.z), self.now_s())

    def detection_cb(self, msg):
        try:
            detection = json.loads(msg.data)
        except ValueError:
            return
        self.machine.on_detection(
            detection.get('confirmed', False),
            detection.get('position_enu'), self.now_s())

    # -- output ----------------------------------------------------------
    def send(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 0.0   # all-zero quaternion: do not command yaw
        self.setpoint_pub.publish(msg)

    def tick(self):
        target = self.machine.tick(self.now_s())
        for level, message in self.machine.pop_events():
            if level == 'warn':
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)
        if target is None:
            self.info_once(
                'no_odom',
                'no /visual_slam/tracking/odometry yet -- is cuVSLAM tracking? '
                'Nothing will be commanded until it is.')
            return
        self.send(*target)


def main(args=None):
    rclpy.init(args=args)
    node = PadGuide()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
