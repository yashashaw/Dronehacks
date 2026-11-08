"""PX4 local position -> nav_msgs/Odometry (ENU) on /px4_square/odom, for flight_interface.

flight_interface measures its altitude band (0.3 m floor, max_altitude_m ceiling) from the
first sample on its `odometry_topic`, which defaults to VIO. If VIO's height is off when that
sample arrives, the floor clamp can REWRITE a 1.0 m target to "VIO origin + 0.3 m" (drone 7:
~26 m). px4_square.launch.py points odometry_topic here instead, so the band is measured in
PX4's own frame (rangefinder/baro height), the same frame the mission's targets are in.

Publishes only when PX4 reports xy_valid and z_valid. Position only (no orientation/velocity):
flight_interface reads pose.pose.position and nothing else (skybeta 0.3.2).
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition

from px4_square.geometry import ned_to_enu

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
TOPIC = '/px4_square/odom'


class Px4OdomRelay(Node):

    def __init__(self):
        super().__init__('px4_odom_relay')
        self.pub = self.create_publisher(Odometry, TOPIC, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_lpos, PX4_QOS)
        self.first = True

    def on_lpos(self, lp):
        if not (lp.xy_valid and lp.z_valid):
            return
        x, y, z = ned_to_enu(lp.x, lp.y, lp.z)
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'px4_local_enu'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = z
        m.pose.pose.orientation.w = 1.0
        self.pub.publish(m)
        if self.first:
            self.first = False
            self.get_logger().info(f'relaying PX4 local position to {TOPIC}; first ENU '
                                   f'({x:.2f}, {y:.2f}, {z:.2f}) = flight_interface envelope origin')


def main(args=None):
    rclpy.init(args=args)
    node = Px4OdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
