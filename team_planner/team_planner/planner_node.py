"""ROS 2 wrapper around PlannerLogic. All decisions live in planner_logic.py.

    subscribe  /visual_slam/tracking/odometry   VIO pose (ENU, odom)
    subscribe  /oak/stereo/image_raw            depth, 16UC1 millimetres
    subscribe  /oak/stereo/camera_info          depth intrinsics (a fallback is used until it arrives)
    subscribe  /skybeta/state                   armed / offboard_active / odom_enu / local_enu
    subscribe  /pad/detection                   pad_detector JSON (position only, as a hint)
    subscribe  /loc/health                      optional, Lucas's localization health (DiagnosticStatus)
    publish    /course/goal                     PoseStamped for pad_guide: point + heading, ENU odom
    publish    /team_planner/debug              JSON at 2 Hz for the bag

It never publishes a setpoint, never talks to /fmu/*, never arms, never changes mode, and writes
no files. Not yet run on a drone.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from team_planner.planner_logic import PlannerConfig, PlannerLogic, warm_up

try:
    from diagnostic_msgs.msg import DiagnosticStatus
except ImportError:  # keeps the module importable off the drone
    DiagnosticStatus = None


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PlannerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("team_planner", **kwargs)
        defaults = PlannerConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("depth_topic", "/oak/stereo/image_raw")
        self.declare_parameter("depth_info_topic", "/oak/stereo/camera_info")
        config = PlannerConfig(**{n: type(v)(self.get_parameter(n).value) for n, v in vars(defaults).items()})
        self.logic = PlannerLogic(config)
        self.intrinsics = None
        self.warned_fallback = False

        if warm_up(config.route_planner):
            self.get_logger().info("route finder compiled (numba).")
        else:
            self.get_logger().warn("numba is missing: the planner runs in pure Python and may be "
                                   "too slow on the Jetson. Add numba to the team image.")

        self.goal_pub = self.create_publisher(PoseStamped, "/course/goal", 10)
        self.debug_pub = self.create_publisher(String, "/team_planner/debug", 10)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self.odometry_cb, 10)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter("depth_info_topic").value, self.info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/skybeta/state", self.state_cb, 10)
        self.create_subscription(String, "/pad/detection", self.pad_cb, 10)
        if DiagnosticStatus is not None:
            self.create_subscription(DiagnosticStatus, "/loc/health", self.health_cb, 10)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self.publish_goal)
        self.create_timer(0.5, self.publish_debug)
        self.get_logger().info("team_planner ready; it starts working when Offboard starts.")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs -------------------------------------------------------------------------------
    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.logic.on_odometry(t, p.x, p.y, p.z, _yaw(msg.pose.pose.orientation))

    def info_cb(self, msg):
        k = msg.k
        if k[0] > 1.0:
            self.intrinsics = (k[0], k[4], k[2], k[5])

    def depth_cb(self, msg):
        if msg.encoding not in ("16UC1", "mono16"):
            self.get_logger().warn(f"depth encoding {msg.encoding} not handled", throttle_duration_sec=5.0)
            return
        dtype = ">u2" if msg.is_bigendian else "<u2"
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
        if self.intrinsics is None and not self.warned_fallback:
            self.warned_fallback = True
            self.get_logger().warn("no depth CameraInfo yet: using the left mono field of view as a stand-in.")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.logic.on_depth(stamp, self.now_s(), img, self.intrinsics)

    def state_cb(self, msg):
        try:
            self.logic.on_state(json.loads(msg.data), self.now_s())
        except ValueError:
            return

    def pad_cb(self, msg):
        try:
            self.logic.on_pad(json.loads(msg.data))
        except ValueError:
            return

    def health_cb(self, msg):
        level = msg.level[0] if isinstance(msg.level, (bytes, bytearray)) else msg.level
        self.logic.on_health(int(level))

    # ---- outputs ------------------------------------------------------------------------------
    def publish_goal(self):
        goal = self.logic.goal(self.now_s())
        if goal is None:
            return
        x, y, z, yaw = goal
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

    def publish_debug(self):
        self.debug_pub.publish(String(data=json.dumps(self.logic.debug())))


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
