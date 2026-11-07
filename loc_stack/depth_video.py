#!/usr/bin/env python3
"""Record an mp4 of the OAK-D on-device depth with live localization (skybeta container).

Needs the depth pipeline running (depth_test.sh on). Subscribes only; publishes nothing.

  python3 depth_video.py --seconds 120 --out /root/team_ws/walk_runs/depth_video/run1

One video frame per depth frame (about 25 Hz), three panels side by side:
  1. left camera image (greyscale, scaled to the depth size)
  2. depth colour map on a FIXED 0-5 m scale (red near, blue far, black = no depth);
     the white box is the centre region whose median distance is printed
  3. top-down localization map: the cuVSLAM (VIO) track relative to where the
     recording started, 1 m grid, +/-4 m, with a heading arrow. The text shows VIO
     x/y/z, tracking state, and PX4 EKF2's xy_valid when the Pixhawk is linked.
Writes depth.mp4 (or depth.avi), first_frame.png, frames.csv (one row per video frame,
with depth stats and the pose at that instant), DONE.
"""
import argparse
import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None
try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:
    VehicleLocalPosition = None

MAX_M = 5.0          # depth colour scale
MAP_PX = 200         # map panel is MAP_PX x depth height
MAP_HALF_M = 4.0     # map shows +/- this many metres around the start
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
VO_NAMES = {0: 'unknown', 1: 'tracking', 2: 'LOST'}


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):

    def __init__(self, out, fps):
        super().__init__('depth_video')
        self.out, self.fps = out, fps
        self.left = None
        self.odom = None          # latest VIO Odometry
        self.origin = None        # VIO position when the recording started
        self.trail = []           # (x, y) relative to origin
        self.vo_state = None
        self.lpos = None          # latest PX4 VehicleLocalPosition
        self.writer = None
        self.path = None
        self.t0 = None
        self.n = 0
        self.f_csv = open(os.path.join(out, 'frames.csv'), 'w', newline='')
        self.w_csv = csv.writer(self.f_csv)
        self.w_csv.writerow(['stamp_s', 't_s', 'valid_pct', 'centre_median_m', 'left_age_ms',
                             'vio_x', 'vio_y', 'vio_z', 'vio_yaw_deg', 'vio_age_ms', 'vo_state',
                             'ekf_x', 'ekf_y', 'ekf_z', 'ekf_xy_valid'])
        q = qos_profile_sensor_data
        # Keep only the latest left message; convert it only when a depth frame needs it.
        self.create_subscription(Image, '/oak/left/image_raw',
                                 lambda m: setattr(self, 'left', m), q)
        self.create_subscription(Image, '/oak/stereo/image_raw', self.on_depth, q)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                                 lambda m: setattr(self, 'odom', m), q)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                     lambda m: setattr(self, 'vo_state', int(m.vo_state)), q)
        if VehicleLocalPosition is not None:
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                     lambda m: setattr(self, 'lpos', m), PX4_QOS)

    def open_writer(self, w, h):
        for name, fourcc in (('depth.mp4', 'mp4v'), ('depth.avi', 'MJPG')):
            path = os.path.join(self.out, name)
            wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), self.fps, (w, h))
            if wr.isOpened():
                self.writer, self.path = wr, path
                return
        raise RuntimeError('no usable video codec (mp4v, MJPG)')

    def pose(self, ts):
        """(rel x, y, z, yaw deg, age ms) of the latest VIO pose, or None."""
        o = self.odom
        if o is None:
            return None
        p = o.pose.pose.position
        if self.origin is None:
            self.origin = (p.x, p.y, p.z)
        rel = (p.x - self.origin[0], p.y - self.origin[1], p.z - self.origin[2])
        return rel + (math.degrees(yaw_of(o.pose.pose.orientation)),
                      round(1000 * (ts - stamp(o)), 1))

    def draw_map(self, h, pose):
        m = np.full((h, MAP_PX, 3), 30, np.uint8)
        s = (min(h, MAP_PX) / 2 - 4) / MAP_HALF_M          # px per metre
        cx, cy = MAP_PX // 2, h // 2
        for k in range(-int(MAP_HALF_M), int(MAP_HALF_M) + 1):
            g = (70, 70, 70) if k else (110, 110, 110)
            cv2.line(m, (int(cx + k * s), 0), (int(cx + k * s), h), g, 1)
            cv2.line(m, (0, int(cy - k * s)), (MAP_PX, int(cy - k * s)), g, 1)

        def px(x, y):
            return int(cx + x * s), int(cy - y * s)

        for a, b in zip(self.trail, self.trail[1:]):
            cv2.line(m, px(*a), px(*b), (0, 220, 255), 1)
        cv2.circle(m, px(0, 0), 3, (0, 255, 0), -1)               # start
        if pose is not None:
            x, y, yaw = pose[0], pose[1], math.radians(pose[3])
            tip = px(x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
            cv2.arrowedLine(m, px(x, y), tip, (255, 255, 255), 1, tipLength=0.4)
        cv2.putText(m, 'VIO track, 1 m grid', (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (200, 200, 200), 1)
        return m

    def on_depth(self, m):
        if m.encoding != '16UC1':
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, -1)[:, :m.width]
        h, w = d.shape
        ts = stamp(m)
        if self.t0 is None:
            self.t0 = ts
        valid = d > 0
        cy, cx, ry, rx = h // 2, w // 2, int(h * 0.15), int(w * 0.15)
        c = d[cy - ry:cy + ry, cx - rx:cx + rx]
        cmed = float(np.median(c[c > 0])) / 1000.0 if (c > 0).any() else None

        scaled = np.clip(d.astype(np.float32) / (MAX_M * 1000.0), 0, 1)
        col = cv2.applyColorMap((255 * (1 - scaled)).astype(np.uint8), cv2.COLORMAP_JET)
        col[~valid] = 0
        cv2.rectangle(col, (cx - rx, cy - ry), (cx + rx, cy + ry), (255, 255, 255), 1)

        left_age = None
        if self.left is not None:
            lm = self.left
            img = np.frombuffer(lm.data, np.uint8).reshape(lm.height, lm.step)
            img = img[:, :lm.width * 3].reshape(lm.height, lm.width, 3)
            grey = cv2.cvtColor(cv2.resize(img, (w, h)), cv2.COLOR_BGR2GRAY)
            left = cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)
            left_age = round(1000 * (ts - stamp(lm)), 1)
        else:
            left = np.zeros_like(col)

        pose = self.pose(ts)
        if pose is not None and (not self.trail or
                                 math.dist(self.trail[-1], pose[:2]) > 0.01):
            self.trail.append(pose[:2])
        mp = self.draw_map(h, pose)
        vo = VO_NAMES.get(self.vo_state, '?')
        lp = self.lpos
        ekf = '' if lp is None else f'EKF xy_valid {int(lp.xy_valid)}'
        if pose is not None:
            ptxt = f'x {pose[0]:+.2f} y {pose[1]:+.2f} z {pose[2]:+.2f}'
            cv2.putText(mp, ptxt, (4, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.33,
                        (255, 255, 255), 1)
        vcol = (0, 255, 0) if vo == 'tracking' else (0, 0, 255)
        cv2.putText(mp, f'VIO {vo}  {ekf}', (4, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.33,
                    vcol, 1)

        frame = np.hstack([left, col, mp])
        txt = (f't={ts - self.t0:5.1f}s  valid {100 * valid.mean():4.1f}%  '
               f'centre {cmed:.2f} m' if cmed else
               f't={ts - self.t0:5.1f}s  valid {100 * valid.mean():4.1f}%  centre: no depth')
        cv2.putText(frame, txt, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame, f'0-{MAX_M:.0f} m: red near, blue far, black none',
                    (w + 6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        if self.writer is None:
            self.open_writer(frame.shape[1], frame.shape[0])
            cv2.imwrite(os.path.join(self.out, 'first_frame.png'), frame)
        self.writer.write(frame)
        if self.n % 125 == 0:   # refresh a still every ~5 s, so a mid-run frame can be checked
            cv2.imwrite(os.path.join(self.out, 'last_frame.png'), frame)

        pz = pose if pose is not None else (None,) * 5
        self.w_csv.writerow([
            f'{ts:.6f}', f'{ts - self.t0:.3f}', round(100 * valid.mean(), 2),
            None if cmed is None else round(cmed, 4), left_age,
            *(None if v is None else round(v, 4) for v in pz[:4]), pz[4], self.vo_state,
            *((None,) * 4 if lp is None else
              (round(lp.x, 4), round(lp.y, 4), round(lp.z, 4), int(lp.xy_valid)))])
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--fps', type=float, default=25.0, help='playback rate written into the file')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Recorder(a.out, a.fps)
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node.writer is not None:
            node.writer.release()
        node.f_csv.close()
        summary = (f'{node.n} depth frames in {a.seconds:.0f} s '
                   f'({node.n / a.seconds:.1f}/s); video {node.path}\n')
        with open(os.path.join(a.out, 'DONE'), 'w') as f:
            f.write(summary)
        print(summary, end='')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
