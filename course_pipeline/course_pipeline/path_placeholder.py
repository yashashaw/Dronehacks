"""PLACEHOLDER PLANNER. The real pathfinder replaces this node, not the guide.

It publishes /course/goal the way the real planner must: continuously, as a
PoseStamped in ``odom``. This stub just publishes one fixed point a few metres
ahead (+x in odom, the vehicle's forward at startup) of the pose where Offboard
began, at the cruise height. That is enough to exercise the guide's loop.

    subscribe  /visual_slam/tracking/odometry
    subscribe  /skybeta/state                  to learn when Offboard began
    publish    /course/goal                    PoseStamped, odom, ENU

It never publishes a setpoint. See CONTRACT.md for what a real planner owes the
guide (rate, staleness, frames).
"""
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class PathPlaceholder(Node):
    def __init__(self, **kwargs):
        super().__init__('path_placeholder', **kwargs)
        self.declare_parameter('forward_m', 3.0)
        self.declare_parameter('altitude_m', 1.5)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.forward = float(self.get_parameter('forward_m').value)
        self.altitude = float(self.get_parameter('altitude_m').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.position = None
        self.reference = None     # pose when Offboard began
        self.goal_pub = self.create_publisher(PoseStamped, '/course/goal', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.odometry_cb, 10)
        self.create_subscription(
            String, '/skybeta/state', self.state_cb, 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            'path_placeholder: publishing a fixed forward goal once Offboard '
            'starts. Replace this node with the real planner.')

    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def state_cb(self, msg):
        try:
            active = bool(json.loads(msg.data).get('offboard_active', False))
        except ValueError:
            return
        if active and self.reference is None and self.position is not None:
            self.reference = self.position

    def tick(self):
        if self.reference is None:
            return
        x0, y0, z0 = self.reference
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = x0 + self.forward
        goal.pose.position.y = y0
        goal.pose.position.z = z0 + self.altitude
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlaceholder()
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
