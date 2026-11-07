#!/usr/bin/env python3
"""Measure the OAK-D's on-device depth against a wall (runs in the skybeta container).

Needs the depth pipeline running (depth_test.sh on). Subscribes only; publishes nothing.

  python3 depth_wall.py --label blank_1m --truth 1.00 --seconds 15 \
      --out /root/team_ws/walk_runs/depth/blank_1m

Point the camera square at a flat wall, `truth` metres from the front of the camera.
Reports, over every depth frame received:
  * rates by HEADER STAMP for depth, left and right (camera_info, one per image).
    `ros2 topic hz` undercounts these streams, so it is not used.
  * valid-pixel % over the whole frame and in a centre region (ROI).
  * a plane fitted to the ROI's 3D points: its perpendicular distance from the
    camera (robust to a few degrees of yaw), RMS flatness noise, tilt, and the
    frame-to-frame spread of that distance.
Writes result.json, sample_depth.npy (one frame, uint16 mm) and DONE.
"""
import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image

MIN_ROI_PTS = 200


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rate_stats(stamps):
    s = sorted(set(stamps))
    if len(s) < 2:
        return {'n': len(s), 'rate_hz': None, 'max_gap_ms': None}
    gaps = np.diff(s)
    return {'n': len(s), 'rate_hz': round((len(s) - 1) / (s[-1] - s[0]), 2),
            'max_gap_ms': round(1000 * float(gaps.max()), 1),
            'gaps_over_50ms': int((gaps > 0.05).sum())}


def plane_fit(pts):
    """Least-squares plane through Nx3 points: (perp. distance from origin, rms, tilt deg)."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    dist = float(abs(n @ c))
    rms = float(np.sqrt(np.mean(((pts - c) @ n) ** 2)))
    tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
    return dist, rms, tilt


class DepthWall(Node):

    def __init__(self, roi_frac):
        super().__init__('depth_wall')
        self.roi_frac = roi_frac
        self.K = None
        self.info = None
        self.stamps = {'depth': [], 'left': [], 'right': [], 'vio_odom': []}
        self.frames = []
        self.sample = None
        q = qos_profile_sensor_data
        self.create_subscription(Image, '/oak/stereo/image_raw', self.on_depth, q)
        self.create_subscription(CameraInfo, '/oak/stereo/camera_info', self.on_info, q)
        self.create_subscription(CameraInfo, '/oak/left/camera_info',
                                 lambda m: self.stamps['left'].append(stamp(m)), q)
        self.create_subscription(CameraInfo, '/oak/right/camera_info',
                                 lambda m: self.stamps['right'].append(stamp(m)), q)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                                 lambda m: self.stamps['vio_odom'].append(stamp(m)), q)

    def on_info(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
            self.info = {'width': m.width, 'height': m.height, 'fx': round(m.k[0], 2),
                         'frame_id': m.header.frame_id}

    def on_depth(self, m):
        self.stamps['depth'].append(stamp(m))
        if m.encoding != '16UC1':
            self.get_logger().warn(f'unexpected depth encoding {m.encoding}')
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, -1)[:, :m.width]
        h, w = d.shape
        rh, rw = int(h * self.roi_frac / 2), int(w * self.roi_frac / 2)
        r0, c0 = h // 2 - rh, w // 2 - rw
        roi = d[r0:h // 2 + rh, c0:w // 2 + rw].astype(np.float64) / 1000.0
        valid = roi > 0
        f = {'valid_full': float((d > 0).mean()), 'valid_roi': float(valid.mean()),
             'roi_median': float(np.median(roi[valid])) if valid.any() else None,
             'plane': None}
        if self.K is not None and valid.sum() >= MIN_ROI_PTS:
            fx, fy, cx, cy = self.K
            vv, uu = np.nonzero(valid)
            z = roi[vv, uu]
            x = (uu + c0 - cx) * z / fx
            y = (vv + r0 - cy) * z / fy
            f['plane'] = plane_fit(np.column_stack([x, y, z]))
        self.frames.append(f)
        if self.sample is None and len(self.frames) == 10:
            self.sample = d.copy()


def summarize(node, label, truth, seconds):
    fr = node.frames
    res = {'label': label, 'truth_m': truth, 'seconds': seconds, 'roi_frac': node.roi_frac,
           'depth_camera_info': node.info,
           'rates_by_header_stamp': {k: rate_stats(v) for k, v in node.stamps.items()},
           'depth_frames': len(fr)}
    if not fr:
        res['error'] = 'no depth frames on /oak/stereo/image_raw (is depth_test.sh on running?)'
        return res
    res['valid_pct_full_median'] = round(100 * float(np.median([f['valid_full'] for f in fr])), 1)
    res['valid_pct_roi_median'] = round(100 * float(np.median([f['valid_roi'] for f in fr])), 1)
    meds = [f['roi_median'] for f in fr if f['roi_median'] is not None]
    res['roi_median_depth_m'] = round(float(np.median(meds)), 4) if meds else None
    planes = [f['plane'] for f in fr if f['plane'] is not None]
    res['frames_with_plane_fit'] = len(planes)
    if planes:
        dists = np.array([p[0] for p in planes])
        res['plane_distance_m'] = round(float(np.median(dists)), 4)
        res['plane_distance_frame_sd_mm'] = round(1000 * float(dists.std()), 1)
        res['plane_rms_mm'] = round(1000 * float(np.median([p[1] for p in planes])), 1)
        res['plane_tilt_deg'] = round(float(np.median([p[2] for p in planes])), 1)
        if truth:
            err = res['plane_distance_m'] - truth
            res['error_m'] = round(err, 4)
            res['error_pct'] = round(100 * err / truth, 2)
    else:
        res['plane_note'] = (f'fewer than {MIN_ROI_PTS} valid ROI pixels in every frame: '
                             'no distance accuracy from this target')
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--truth', type=float, default=None, help='tape distance to the wall, m')
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--roi', type=float, default=0.3, help='centre ROI size as a fraction')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = DepthWall(a.roi)
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        res = summarize(node, a.label, a.truth, a.seconds)
        with open(os.path.join(a.out, 'result.json'), 'w') as f:
            json.dump(res, f, indent=2)
        if node.sample is not None:
            np.save(os.path.join(a.out, 'sample_depth.npy'), node.sample)
        with open(os.path.join(a.out, 'DONE'), 'w') as f:
            f.write('ok\n')
        print(json.dumps(res, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
