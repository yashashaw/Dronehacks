#!/usr/bin/env python3
"""Quick camera + IMU test: record /oak camera frames with live IMU values drawn on them.

Runs inside the skybeta-stack container (the stack owns the OAK-D, so a standalone depthai
script cannot open it). Subscribes to the camera and IMU topics for DURATION seconds,
overlays the IMU reading closest in time to each frame, and writes:

  <out>/overlay.mp4        frames with accel / gyro / orientation drawn on top
  <out>/overlay_sample.png the middle frame, for a quick look
  <out>/imu.csv            every IMU sample received
  <out>/report.txt         per-topic message counts and rates (publisher health check)

Usage (in the container, ROS sourced):
  python3 sensor_overlay_test.py [--seconds 10] [--image /oak/left/image_raw]
                                 [--imu /oak/imu/data] [--out /tmp/sensor_test]
"""
import argparse
import bisect
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR, without cv_bridge."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported encoding {msg.encoding!r}')


def quat_to_rpy_deg(q):
    sinr = 2 * (q.w * q.x + q.y * q.z)
    cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return (math.degrees(math.atan2(sinr, cosr)),
            math.degrees(math.asin(sinp)),
            math.degrees(math.atan2(siny, cosy)))


class Recorder(Node):
    def __init__(self, image_topic, imu_topic):
        super().__init__('sensor_overlay_test')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.frames = []        # (stamp, bgr ndarray)
        self.imu = []           # (stamp, msg)
        self.create_subscription(Image, image_topic, self.on_image, qos)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos)

    def on_image(self, msg):
        try:
            self.frames.append((stamp_s(msg), image_to_bgr(msg)))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)

    def on_imu(self, msg):
        self.imu.append((stamp_s(msg), msg))


def nearest(times, t):
    i = bisect.bisect_left(times, t)
    if i == 0:
        return 0
    if i == len(times):
        return len(times) - 1
    return i if times[i] - t < t - times[i - 1] else i - 1


def draw_overlay(img, imu_msg, dt_ms, t_rel):
    out = img.copy()
    a, g = imu_msg.linear_acceleration, imu_msg.angular_velocity
    r, p, y = quat_to_rpy_deg(imu_msg.orientation)
    lines = [
        f't = {t_rel:6.2f} s   imu-frame dt = {dt_ms:+5.1f} ms',
        f'accel  x {a.x:+7.3f}  y {a.y:+7.3f}  z {a.z:+7.3f}  m/s^2',
        f'gyro   x {g.x:+7.3f}  y {g.y:+7.3f}  z {g.z:+7.3f}  rad/s',
        f'orient r {r:+7.1f}  p {p:+7.1f}  y {y:+7.1f}  deg',
    ]
    scale = max(0.5, out.shape[1] / 1280 * 0.8)
    h = int(28 * scale * 1.4) * len(lines) + 12
    box = out.copy()
    cv2.rectangle(box, (0, 0), (int(out.shape[1] * 0.62), h), (0, 0, 0), -1)
    out = cv2.addWeighted(box, 0.55, out, 0.45, 0)
    for i, line in enumerate(lines):
        cv2.putText(out, line, (8, int(28 * scale * 1.4 * (i + 1))),
                    cv2.FONT_HERSHEY_SIMPLEX, scale * 0.7, (0, 255, 0), 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--image', default='/oak/left/image_raw')
    ap.add_argument('--imu', default='/oak/imu/data')
    ap.add_argument('--out', default='/tmp/sensor_test')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Recorder(args.image, args.imu)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    frames, imu = node.frames, node.imu
    lines = [f'duration requested : {args.seconds:.1f} s',
             f'{args.image:<24}: {len(frames)} frames  ({len(frames) / args.seconds:.1f} Hz)',
             f'{args.imu:<24}: {len(imu)} samples ({len(imu) / args.seconds:.1f} Hz)']
    if not frames or not imu:
        lines.append('FAIL: a topic published nothing; no video written')
        print('\n'.join(lines))
        open(os.path.join(args.out, 'report.txt'), 'w').write('\n'.join(lines) + '\n')
        raise SystemExit(1)

    with open(os.path.join(args.out, 'imu.csv'), 'w') as f:
        f.write('stamp,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n')
        for t, m in imu:
            a, g, q = m.linear_acceleration, m.angular_velocity, m.orientation
            f.write(f'{t:.6f},{a.x},{a.y},{a.z},{g.x},{g.y},{g.z},{q.x},{q.y},{q.z},{q.w}\n')

    imu_t = [t for t, _ in imu]
    t0 = frames[0][0]
    fps = max(1.0, (len(frames) - 1) / max(frames[-1][0] - t0, 1e-6))
    h, w = frames[0][1].shape[:2]
    writer = cv2.VideoWriter(os.path.join(args.out, 'overlay.mp4'),
                             cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    dts, mid = [], None
    for k, (t, img) in enumerate(frames):
        j = nearest(imu_t, t)
        dt_ms = (imu_t[j] - t) * 1000
        dts.append(abs(dt_ms))
        shown = draw_overlay(img, imu[j][1], dt_ms, t - t0)
        writer.write(shown)
        if k == len(frames) // 2:
            mid = shown
    writer.release()
    cv2.imwrite(os.path.join(args.out, 'overlay_sample.png'), mid)

    lines += [f'video               : {w}x{h} @ {fps:.1f} fps, {len(frames)} frames',
              f'frame<->imu offset  : mean {np.mean(dts):.1f} ms, max {np.max(dts):.1f} ms',
              'PASS']
    print('\n'.join(lines))
    open(os.path.join(args.out, 'report.txt'), 'w').write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
