#!/usr/bin/env python3
"""Field-test logger: VIO pose + wall plane + depth scan + PX4 state, one CSV row per depth frame.

Runs INSIDE the skybeta-stack container (ft.sh starts and stops it). Subscribe-only: it never
publishes anything, so it cannot affect the flight.

    python3 wall_logger.py --label W3_turn_01 [--rate 10] [--seconds 0]

Writes /root/team_ws/field_tests/runs/<stamp>_<label>/
    log.csv    rate-limited rows: VIO pose, wall plane fit (camera + world frame), FC state
    odom.csv   every VIO odometry message (30 Hz) for jump / gap analysis
    scan.csv   48-ray horizontal depth scan per row, as world-frame points (wall map)
    meta.json  settings, counts, camera geometry
    DONE       written on a clean stop (SIGINT / SIGTERM / --seconds)

Wall plane: least-squares plane (one outlier-rejection pass) through the centre third of the
depth image. Its normal points from the camera to the wall; `wall_dist_m` is the perpendicular
distance. Transformed to the VIO frame with the odometry pose at the depth frame's stamp, the
plane is n_w . x = d_w. The wall does not move, so a change in d_w for the same wall is VIO
drift along the wall normal. That is the number most tests here are built on.
"""
import argparse
import bisect
import csv
import json
import math
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image

try:
    from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
except ImportError:
    VehicleStatus = VehicleLocalPosition = None
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

N_RAYS = 48
MIN_PTS = 150
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
# VOLATILE: PX4's /fmu/out publishers are best-effort + volatile; asking for TRANSIENT_LOCAL
# makes the subscription incompatible and it silently receives nothing.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

LOG_COLS = ['t', 'vio_x', 'vio_y', 'vio_z', 'vio_yaw_deg', 'vio_dt_ms', 'vo_state',
            'armed', 'nav_state', 'fc_x', 'fc_y', 'fc_z', 'fc_heading_deg', 'dist_bottom',
            'valid_frac', 'roi_n', 'plane_ok', 'wall_dist_m', 'wall_rms_m', 'wall_inlier_frac',
            'wall_yaw_deg', 'wall_pitch_deg', 'nwx', 'nwy', 'nwz', 'dw', 'center_med_m']


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


# optical (x right, y down, z forward) -> body FLU (x forward, y left, z up)
R_OPT_TO_FLU = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


def fit_plane(P):
    """P: (N,3) optical-frame points. Returns (n, dist, rms, inlier_frac) or None."""
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    n = vt[2]
    r = (P - c) @ n
    mad = np.median(np.abs(r - np.median(r))) * 1.4826
    keep = np.abs(r) < max(0.02, 3.0 * mad)
    if keep.sum() < MIN_PTS:
        return None
    Q = P[keep]
    c = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
    n = vt[2]
    if n @ c < 0:
        n = -n
    rms = float(np.sqrt(np.mean(((Q - c) @ n) ** 2)))
    return n, float(n @ c), rms, float(keep.mean())


class Logger(Node):
    def __init__(self, a):
        super().__init__('field_test_logger')
        self.a = a
        stamp_s = time.strftime('%Y%m%d-%H%M%S')
        self.dir = os.path.join(a.out_root, f'{stamp_s}_{a.label}')
        os.makedirs(self.dir, exist_ok=True)
        self.f_log = open(os.path.join(self.dir, 'log.csv'), 'w', newline='')
        self.w_log = csv.writer(self.f_log)
        self.w_log.writerow(LOG_COLS)
        self.f_odom = open(os.path.join(self.dir, 'odom.csv'), 'w', newline='')
        self.w_odom = csv.writer(self.f_odom)
        self.w_odom.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.f_scan = open(os.path.join(self.dir, 'scan.csv'), 'w', newline='')
        self.w_scan = csv.writer(self.f_scan)
        self.w_scan.writerow(['t', 'ray', 'bearing_deg', 'range_m', 'xw', 'yw'])

        env = os.environ
        self.cam_t = np.array([float(env.get('CAMERA_X', 0.0)), float(env.get('CAMERA_Y', 0.0)),
                               float(env.get('CAMERA_Z', 0.0))])
        self.cam_R = rpy_to_R(float(env.get('CAMERA_ROLL', 0.0)), float(env.get('CAMERA_PITCH', 0.0)),
                              float(env.get('CAMERA_YAW', 0.0))) @ R_OPT_TO_FLU

        self.odom_t, self.odom_p = [], []      # ring buffer of (stamp, (pos, quat))
        self.vo_state = -1
        self.armed = None
        self.nav_state = -1
        self.lpos = None
        self.K = None
        self.ci_size = None
        self.depth = None
        self.last_depth_t = None
        self.last_row_t = 0.0
        self.counts = {'odom': 0, 'depth': 0, 'rows': 0, 'plane_ok': 0, 'px4_status': 0, 'px4_lpos': 0}
        self.started = time.time()

        self.create_subscription(Odometry, a.odom_topic, self.on_odom, SENSOR_QOS)
        self.create_subscription(CameraInfo, '/oak/stereo/camera_info', self.on_ci, SENSOR_QOS)
        self.create_subscription(Image, '/oak/stereo/image_raw', self.on_depth, SENSOR_QOS)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                     lambda m: setattr(self, 'vo_state', int(m.vo_state)), 10)
        if VehicleStatus is not None:
            self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, PX4_QOS)
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                     self.on_lpos, PX4_QOS)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'logging to {self.dir}')

    # ------------------------------------------------------------------ callbacks
    def on_odom(self, m):
        t = stamp(m)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.w_odom.writerow([f'{t:.4f}', f'{p.x:.4f}', f'{p.y:.4f}', f'{p.z:.4f}',
                              f'{q.x:.5f}', f'{q.y:.5f}', f'{q.z:.5f}', f'{q.w:.5f}'])
        self.odom_t.append(t)
        self.odom_p.append((np.array([p.x, p.y, p.z]), (q.x, q.y, q.z, q.w)))
        if len(self.odom_t) > 90:
            del self.odom_t[:30], self.odom_p[:30]
        self.counts['odom'] += 1

    def on_lpos(self, m):
        self.lpos = m
        self.counts['px4_lpos'] += 1

    def on_status(self, m):
        self.counts['px4_status'] += 1
        self.armed = int(m.arming_state) == 2
        self.nav_state = int(m.nav_state)

    def on_ci(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
        self.ci_size = (m.width, m.height)

    def on_depth(self, m):
        self.depth = m
        self.counts['depth'] += 1

    # ------------------------------------------------------------------ processing
    def pose_at(self, t):
        if not self.odom_t:
            return None, None
        i = bisect.bisect_left(self.odom_t, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.odom_t)]
        j = min(cands, key=lambda k: abs(self.odom_t[k] - t))
        return self.odom_p[j], (self.odom_t[j] - t) * 1000.0

    def tick(self):
        if self.a.seconds > 0 and time.time() - self.started > self.a.seconds:
            raise KeyboardInterrupt
        m = self.depth
        if m is None or self.K is None:
            return
        t = stamp(m)
        if t == self.last_depth_t or t - self.last_row_t < 1.0 / self.a.rate:
            return
        self.last_depth_t = t
        self.last_row_t = t
        if m.encoding not in ('16UC1', 'mono16'):
            return
        d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        h, w = d.shape
        fx, fy, cx, cy = self.K
        if self.ci_size and self.ci_size[0] != w:          # camera_info for a different size
            s = w / float(self.ci_size[0])
            fx, fy, cx, cy = fx * s, fy * s, cx * s, cy * s
        valid = (d > 0.2) & (d < 8.0)
        pose, dt_ms = self.pose_at(t)
        if pose is not None:
            pos, q = pose
            Rw = quat_to_R(*q)
            yaw = math.degrees(math.atan2(Rw[1, 0], Rw[0, 0]))
        row = {k: '' for k in LOG_COLS}
        row.update(t=f'{t:.4f}', valid_frac=f'{valid.mean():.3f}', vo_state=self.vo_state,
                   armed='' if self.armed is None else int(self.armed), nav_state=self.nav_state)
        if pose is not None:
            row.update(vio_x=f'{pos[0]:.4f}', vio_y=f'{pos[1]:.4f}', vio_z=f'{pos[2]:.4f}',
                       vio_yaw_deg=f'{yaw:.2f}', vio_dt_ms=f'{dt_ms:.1f}')
        lp = self.lpos
        if lp is not None:
            row.update(fc_x=f'{lp.x:.3f}', fc_y=f'{lp.y:.3f}', fc_z=f'{lp.z:.3f}',
                       fc_heading_deg=f'{math.degrees(lp.heading):.2f}', dist_bottom=f'{lp.dist_bottom:.3f}')

        # ---- wall plane in the centre third
        r0, r1, c0, c1 = h // 3, 2 * h // 3, w // 3, 2 * w // 3
        roi = d[r0:r1:3, c0:c1:3]
        rv = valid[r0:r1:3, c0:c1:3]
        row['roi_n'] = int(rv.sum())
        if rv.any():
            row['center_med_m'] = f'{float(np.median(roi[rv])):.3f}'
        if rv.sum() >= MIN_PTS:
            vv, uu = np.nonzero(rv)
            z = roi[rv]
            u = (uu * 3 + c0).astype(np.float32)
            v = (vv * 3 + r0).astype(np.float32)
            P = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
            fit = fit_plane(P)
            if fit is not None:
                n, dist, rms, inl = fit
                ok = rms < 0.04 and inl > 0.7 and abs(n[2]) > 0.5   # must face the camera (< 60 deg)
                row.update(plane_ok=int(ok), wall_dist_m=f'{dist:.4f}', wall_rms_m=f'{rms:.4f}',
                           wall_inlier_frac=f'{inl:.3f}',
                           wall_yaw_deg=f'{math.degrees(math.atan2(n[0], n[2])):.2f}',
                           wall_pitch_deg=f'{math.degrees(math.atan2(n[1], n[2])):.2f}')
                if pose is not None:
                    n_w = Rw @ (self.cam_R @ n)
                    p_cam_w = Rw @ self.cam_t + pos                  # camera origin in VIO frame
                    d_w = float(n_w @ p_cam_w + dist)                # plane: n_w . x = d_w
                    row.update(nwx=f'{n_w[0]:.4f}', nwy=f'{n_w[1]:.4f}', nwz=f'{n_w[2]:.4f}', dw=f'{d_w:.4f}')
                if ok:
                    self.counts['plane_ok'] += 1
            else:
                row['plane_ok'] = 0
        else:
            row['plane_ok'] = 0

        # ---- horizontal scan (rows around the optical centre) -> world points
        if pose is not None:
            band = slice(max(0, int(cy) - h // 12), min(h, int(cy) + h // 12), 2)
            edges = np.linspace(0, w, N_RAYS + 1).astype(int)
            p_cam_w = Rw @ self.cam_t + pos
            for i in range(N_RAYS):
                blk = d[band, edges[i]:edges[i + 1]:2]
                bv = blk[(blk > 0.2) & (blk < 8.0)]
                if bv.size < 0.3 * blk.size:
                    continue
                zz = float(np.percentile(bv, 30))
                uc = 0.5 * (edges[i] + edges[i + 1])
                xo = (uc - cx) * zz / fx
                pw = Rw @ (self.cam_R @ np.array([xo, 0.0, zz])) + p_cam_w
                self.w_scan.writerow([f'{t:.4f}', i, f'{math.degrees(math.atan2(-xo, zz)):.1f}',
                                      f'{math.hypot(xo, zz):.3f}', f'{pw[0]:.3f}', f'{pw[1]:.3f}'])

        self.w_log.writerow([row[k] for k in LOG_COLS])
        self.counts['rows'] += 1
        if self.counts['rows'] % 50 == 0:
            for f in (self.f_log, self.f_odom, self.f_scan):
                f.flush()

    def close(self):
        for f in (self.f_log, self.f_odom, self.f_scan):
            f.close()
        meta = {'label': self.a.label, 'started': self.started, 'stopped': time.time(),
                'counts': self.counts, 'rate': self.a.rate, 'odom_topic': self.a.odom_topic,
                'camera_mount_xyz': self.cam_t.tolist(), 'depth_K': self.K, 'depth_size': self.ci_size,
                'px4_msgs': VehicleStatus is not None, 'vslam_status_msgs': VisualSlamStatus is not None}
        with open(os.path.join(self.dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        open(os.path.join(self.dir, 'DONE'), 'w').write(time.strftime('%F %T') + '\n')
        print(f'[logger] stopped: {self.counts} -> {self.dir}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--seconds', type=float, default=0.0)
    ap.add_argument('--odom-topic', default='/visual_slam/tracking/odometry')
    ap.add_argument('--out-root', default='/root/team_ws/field_tests/runs')
    a = ap.parse_args()
    rclpy.init()
    node = Logger(a)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            print(f'[logger] error: {e!r}', flush=True)
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
