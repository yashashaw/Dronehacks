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
from sensor_msgs.msg import PointCloud2
try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    point_cloud2 = None
try:
    from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition
except ImportError:
    EstimatorStatusFlags = VehicleLocalPosition = None

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
ODOM_COLS = ['recv_s', 'stamp_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz']


class Logger(Node):
    def __init__(self, out, odom_topic, status_topic, lpos_topic, slam_topic=None, lm_topic=None):
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
        self.n_odom = self.n_lpos = self.n_slam = 0
        self.f_slam = self.w_slam = None
        self.lm_cloud = None      # latest landmark cloud, written once at the end
        self.lm_frame = ''
        self.n_lm = 0
        if slam_topic:
            self.f_slam = open(os.path.join(out, 'slam_odom.csv'), 'w', newline='')
            self.w_slam = csv.writer(self.f_slam)
            self.w_slam.writerow(['recv_s', 'stamp_s', 'x', 'y', 'z', 'frame'])
            self.create_subscription(Odometry, slam_topic, self.on_slam, qos_profile_sensor_data)
        if lm_topic and point_cloud2 is not None:
            self.create_subscription(PointCloud2, lm_topic, self.on_lm, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, status_topic, self.on_status,
                                     qos_profile_sensor_data)
        self.f_flags = self.w_flags = None
        self.last_flags_t = -1.0
        if VehicleLocalPosition is not None:
            self.create_subscription(VehicleLocalPosition, lpos_topic, self.on_lpos, PX4_QOS)
            self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
                                     self.on_flags, PX4_QOS)

    def rel(self):
        return time.monotonic() - self.t0

    def on_odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        s = m.header.stamp
        self.w_odom.writerow([f'{self.rel():.4f}', f'{s.sec + s.nanosec * 1e-9:.6f}',
                              p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z])
        self.n_odom += 1

    def on_slam(self, m):
        p, s = m.pose.pose.position, m.header.stamp
        self.w_slam.writerow([f'{self.rel():.4f}', f'{s.sec + s.nanosec * 1e-9:.6f}',
                              p.x, p.y, p.z, m.header.frame_id])
        self.n_slam += 1

    def on_lm(self, m):
        self.n_lm += 1
        pts = [(float(x), float(y), float(z)) for x, y, z in
               point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)]
        if pts:                      # keep the biggest-so-far snapshot, it is the fullest map
            if self.lm_cloud is None or len(pts) >= len(self.lm_cloud):
                self.lm_cloud, self.lm_frame = pts, m.header.frame_id

    def on_status(self, m):
        if m.vo_state != self.last_state:  # state changes only
            self.last_state = m.vo_state
            self.w_status.writerow([f'{self.rel():.4f}', int(m.vo_state)])

    def on_lpos(self, m):
        if self.f_lpos is None:
            self.f_lpos = open(os.path.join(self.out, 'lpos.csv'), 'w', newline='')
            self.w_lpos = csv.writer(self.f_lpos)
            self.w_lpos.writerow(['recv_s', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'xy_valid',
                                  'z_valid', 'v_xy_valid', 'eph', 'epv', 'xy_reset_counter',
                                  'dist_bottom', 'dist_bottom_valid'])
        self.w_lpos.writerow([f'{self.rel():.4f}', m.x, m.y, m.z, m.vx, m.vy, m.vz, m.heading,
                              int(m.xy_valid), int(m.z_valid), int(m.v_xy_valid), m.eph, m.epv,
                              int(m.xy_reset_counter), m.dist_bottom, int(m.dist_bottom_valid)])
        self.n_lpos += 1

    FLAG_COLS = ['cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt', 'cs_ev_vel', 'cs_mag', 'cs_baro_hgt',
                 'cs_rng_hgt', 'cs_tilt_align', 'cs_yaw_align', 'cs_inertial_dead_reckoning',
                 'reject_hor_pos', 'reject_yaw']

    def on_flags(self, m):
        t = self.rel()
        if t - self.last_flags_t < 0.5:        # 2 Hz is plenty for status bits
            return
        self.last_flags_t = t
        if self.f_flags is None:
            self.f_flags = open(os.path.join(self.out, 'ekf_flags.csv'), 'w', newline='')
            self.w_flags = csv.writer(self.f_flags)
            self.w_flags.writerow(['recv_s'] + self.FLAG_COLS)
        self.w_flags.writerow([f'{t:.3f}'] + [int(getattr(m, c, 0)) for c in self.FLAG_COLS])

    def close(self):
        if self.lm_cloud:
            with open(os.path.join(self.out, 'landmarks.csv'), 'w', newline='') as f:
                f.write('# frame_id=' + self.lm_frame + chr(10) + 'x,y,z' + chr(10))
                csv.writer(f).writerows(self.lm_cloud)
        for f in (self.f_odom, self.f_status, self.f_lpos, self.f_slam, self.f_flags):
            if f:
                f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--out', default='/root/team_ws/walk_runs/run')
    ap.add_argument('--odom', default='/visual_slam/tracking/odometry')
    ap.add_argument('--status', default='/visual_slam/status')
    ap.add_argument('--lpos', default='/fmu/out/vehicle_local_position')
    ap.add_argument('--slam-odom', default=None, help='e.g. /visual_slam/vis/slam_odometry')
    ap.add_argument('--landmarks', default=None, help='e.g. /visual_slam/vis/landmarks_cloud')
    a = ap.parse_args()

    rclpy.init()
    node = Logger(a.out, a.odom, a.status, a.lpos, a.slam_odom, a.landmarks)
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close()
        summary = (f'{a.seconds:.0f} s: {node.n_odom} odom ({node.n_odom / a.seconds:.1f} Hz), '
                   f'{node.n_lpos} lpos, {node.n_slam} slam odom, {node.n_lm} landmark clouds '
                   f'({len(node.lm_cloud or [])} pts in the fullest)\n')
        with open(os.path.join(a.out, 'DONE'), 'w') as f:
            f.write(summary)
        print(summary, end='')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
