#!/usr/bin/env python3
"""Log cuVSLAM odometry (+ tracking state, + EKF2 height if the FC is linked) to CSV.

For the hand-carry drift test and the known-object test. Odometry and status only: no
images, so it keeps up with the 30 Hz stream. Fixed duration, so it can run detached while
you unplug the USB-C cable and carry the drone around.

  docker exec -d skybeta-stack bash -lc "source /opt/ros/*/setup.bash; \
     source /usr/local/bin/skybeta_env.sh; python3 /root/team_ws/loc_stack/walk_logger.py \
     --seconds 90 --out /root/team_ws/walk_runs/run1"

Writes <out>/odom.csv, <out>/status.csv (state changes only) and <out>/lpos.csv (only if
vehicle_local_position arrives). `recv_s` is this node's monotonic clock; `stamp_s` is the
header stamp (image capture time). Nothing is published; nothing is sent to the FC.
"""
import argparse
import csv
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None
try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:
    VehicleLocalPosition = None

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
ODOM_COLS = ['recv_s', 'stamp_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz']


class Logger(Node):
    def __init__(self, out, odom_topic, status_topic, lpos_topic):
        super().__init__('walk_logger')
        os.makedirs(out, exist_ok=True)
        self.t0 = time.monotonic()
        self.f_odom = open(os.path.join(out, 'odom.csv'), 'w', newline='')
        self.w_odom = csv.writer(self.f_odom)
        self.w_odom.writerow(ODOM_COLS)
        self.f_status = open(os.path.join(out, 'status.csv'), 'w', newline='')
        self.w_status = csv.writer(self.f_status)
        self.w_status.writerow(['recv_s', 'vo_state'])
        self.f_lpos = None
        self.out = out
        self.last_state = None
        self.n_odom = self.n_lpos = 0
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, status_topic, self.on_status,
                                     qos_profile_sensor_data)
        if VehicleLocalPosition is not None:
            self.create_subscription(VehicleLocalPosition, lpos_topic, self.on_lpos, PX4_QOS)

    def rel(self):
        return time.monotonic() - self.t0

    def on_odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        s = m.header.stamp
        self.w_odom.writerow([f'{self.rel():.4f}', f'{s.sec + s.nanosec * 1e-9:.6f}',
                              p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z])
        self.n_odom += 1

    def on_status(self, m):
        if m.vo_state != self.last_state:  # state changes only
            self.last_state = m.vo_state
            self.w_status.writerow([f'{self.rel():.4f}', int(m.vo_state)])

    def on_lpos(self, m):
        if self.f_lpos is None:
            self.f_lpos = open(os.path.join(self.out, 'lpos.csv'), 'w', newline='')
            self.w_lpos = csv.writer(self.f_lpos)
            self.w_lpos.writerow(['recv_s', 'x', 'y', 'z', 'xy_valid', 'z_valid',
                                  'dist_bottom', 'dist_bottom_valid'])
        self.w_lpos.writerow([f'{self.rel():.4f}', m.x, m.y, m.z, int(m.xy_valid),
                              int(m.z_valid), m.dist_bottom, int(m.dist_bottom_valid)])
        self.n_lpos += 1

    def close(self):
        for f in (self.f_odom, self.f_status, self.f_lpos):
            if f:
                f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--out', default='/root/team_ws/walk_runs/run')
    ap.add_argument('--odom', default='/visual_slam/tracking/odometry')
    ap.add_argument('--status', default='/visual_slam/status')
    ap.add_argument('--lpos', default='/fmu/out/vehicle_local_position')
    a = ap.parse_args()

    rclpy.init()
    node = Logger(a.out, a.odom, a.status, a.lpos)
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close()
        summary = (f'{a.seconds:.0f} s: {node.n_odom} odom ({node.n_odom / a.seconds:.1f} Hz), '
                   f'{node.n_lpos} lpos\n')
        with open(os.path.join(a.out, 'DONE'), 'w') as f:
            f.write(summary)
        print(summary, end='')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
