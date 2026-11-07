#!/usr/bin/env python3
"""Localization health signal: /loc/health (diagnostic_msgs/DiagnosticStatus, 30 Hz).

SUBSCRIBE-ONLY with respect to PX4: this node never publishes on /fmu/*. The drone already
runs px4_odom_bridge (the one publisher on /fmu/in/vehicle_visual_odometry); a second
publisher would fight it. We read what the stack already produces:

  /visual_slam/tracking/odometry   odometry age            (cuVSLAM, odom -> base_link)
  /visual_slam/status              vo_state 0/1/2          (1 = tracking)
  /fmu/in/vehicle_visual_odometry  reset_counter, quality  (what px4_odom_bridge sent;
                                   it sends quality=0, which PX4 defines as 'unknown', not 'bad')
  /fmu/out/vehicle_local_position  xy_valid, z_valid, dist_bottom   (EKF2, if the FC is linked)

Level logic (spec: OK / WARN "degrading" / ERROR "lost"):
  ERROR  odometry older than error_age_s, or EKF2 reports xy_valid false
  WARN   odometry older than warn_age_s, or vo_state is not 1 (tracking lost)
  OK     otherwise
If the flight controller is not linked (no recent vehicle_local_position) the EKF2 fields are
reported as 'unknown' and the level comes from VIO alone; fc_linked says so explicitly.

Run it detached (the SSH link to the drone resets often):
  docker exec -d skybeta-stack bash -lc "source /opt/ros/*/setup.bash; \
      source /usr/local/bin/skybeta_env.sh; python3 /root/team_ws/loc_stack/health_node.py"
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

try:
    from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
except ImportError:  # keeps `--selftest` importable off the drone
    VehicleLocalPosition = VehicleOdometry = None
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

# BEST_EFFORT + VOLATILE subscribers are compatible with any publisher QoS.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
VO_TRACKING = 1  # VisualSlamStatus.vo_state: 0 unknown, 1 success, 2 failed


def classify(age_s, tracking, xy_valid, warn_age_s, error_age_s):
    """Pure health rule. tracking/xy_valid are True/False/None (None = unknown)."""
    if age_s > error_age_s or xy_valid is False:
        return DiagnosticStatus.ERROR
    if age_s > warn_age_s or tracking is False:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.OK


class HealthNode(Node):
    def __init__(self):
        super().__init__('loc_health')
        p = self.declare_parameter
        self.odom_topic = p('odom_topic', '/visual_slam/tracking/odometry').value
        self.status_topic = p('status_topic', '/visual_slam/status').value
        self.vio_topic = p('px4_vio_topic', '/fmu/in/vehicle_visual_odometry').value
        self.lpos_topic = p('px4_lpos_topic', '/fmu/out/vehicle_local_position').value
        self.warn_age = p('warn_age_s', 0.10).value
        self.error_age = p('error_age_s', 0.30).value
        self.fc_timeout = p('fc_timeout_s', 1.0).value

        self.last_odom_ns = None
        self.tracking = None
        self.vio = None        # last VehicleOdometry the bridge sent
        self.lpos = None
        self.last_lpos_ns = None

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, self.status_topic, self.on_status,
                                     qos_profile_sensor_data)
        else:
            self.get_logger().warn('VisualSlamStatus not importable: tracking state unknown')
        if VehicleOdometry is not None:
            self.create_subscription(VehicleOdometry, self.vio_topic, self.on_vio, PX4_QOS)
            self.create_subscription(VehicleLocalPosition, self.lpos_topic, self.on_lpos, PX4_QOS)
        self.pub = self.create_publisher(DiagnosticStatus, '/loc/health', 10)
        self.create_timer(1.0 / 30.0, self.tick)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def on_odom(self, _msg):
        self.last_odom_ns = self.now_ns()

    def on_status(self, msg):
        self.tracking = int(msg.vo_state) == VO_TRACKING

    def on_vio(self, msg):
        self.vio = msg

    def on_lpos(self, msg):
        self.lpos = msg
        self.last_lpos_ns = self.now_ns()

    def tick(self):
        now = self.now_ns()
        age = (now - self.last_odom_ns) / 1e9 if self.last_odom_ns else math.inf
        fc_linked = (self.last_lpos_ns is not None
                     and (now - self.last_lpos_ns) / 1e9 < self.fc_timeout)
        xy_valid = bool(self.lpos.xy_valid) if fc_linked else None
        level = classify(age, self.tracking, xy_valid, self.warn_age, self.error_age)

        def tf(v):
            return 'unknown' if v is None else str(v)
        kv = {
            'odom_age_ms': f'{age * 1000:.0f}' if math.isfinite(age) else 'inf',
            'tracking': tf(self.tracking),
            'fc_linked': str(fc_linked),
            'xy_valid': tf(xy_valid),
            'z_valid': tf(bool(self.lpos.z_valid) if fc_linked else None),
            'dist_bottom': f'{self.lpos.dist_bottom:.3f}' if fc_linked else 'unknown',
            'dist_bottom_valid': tf(bool(self.lpos.dist_bottom_valid) if fc_linked else None),
            'reset_counter': str(int(self.vio.reset_counter)) if self.vio else 'unknown',
            'quality': str(int(self.vio.quality)) if self.vio else 'unknown',
        }
        s = DiagnosticStatus()
        s.level = level  # the OK/WARN/ERROR constants are already the byte values rclpy wants
        s.name = 'localization'
        s.message = {DiagnosticStatus.OK: 'ok', DiagnosticStatus.WARN: 'degrading',
                     DiagnosticStatus.ERROR: 'lost'}[level]
        s.hardware_id = 'oak-d-s2+cuvslam'
        s.values = [KeyValue(key=k, value=v) for k, v in kv.items()]
        self.pub.publish(s)


def selftest():
    W, E = 0.10, 0.30
    cases = [
        ((0.02, True, True), DiagnosticStatus.OK),
        ((0.02, None, None), DiagnosticStatus.OK),        # no FC, no status: VIO age only
        ((0.15, True, True), DiagnosticStatus.WARN),
        ((0.02, False, True), DiagnosticStatus.WARN),     # tracking lost
        ((0.50, True, True), DiagnosticStatus.ERROR),
        ((0.02, True, False), DiagnosticStatus.ERROR),    # EKF2 dropped xy
        ((math.inf, None, None), DiagnosticStatus.ERROR),  # never saw odometry
    ]
    for args, want in cases:
        got = classify(*args, W, E)
        assert got == want, (args, got, want)
    print('selftest ok:', len(cases), 'cases')


def main():
    import sys
    if '--selftest' in sys.argv:
        return selftest()
    rclpy.init()
    node = HealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
