#!/usr/bin/env python3
"""Save the stereo camera to video files on the Jetson. Read-only: subscribes, never commands.

Run on the drone (the stack must be up; it does not touch the camera itself):

    ~/team_ws/drone_tests/stereo_video.sh            # until Ctrl-C
    ~/team_ws/drone_tests/stereo_video.sh 30         # 30 s

Writes ~/team_ws/videos/stereo-<time>/left.avi and right.avi: the grayscale left/right images
VIO uses (/oak/{left,right}/image_mono, 1024x640, 30 fps), MJPG, about 5-10 MB/s together,
plus stamps.csv (frame number, ROS header stamp in s for each eye) so frames can be matched
to logs. Frames are written as they arrive; a dropped frame is simply missing (see stamps.csv).
"""
import csv
import os
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

FPS = 30.0


def to_gray(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('mono8', '8UC1'):
        return a.reshape(msg.height, msg.step)[:, :msg.width]
    if msg.encoding in ('bgr8', 'rgb8'):
        return a.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)[:, :, 0]
    raise ValueError(f'unsupported encoding {msg.encoding}')


class StereoVideo(Node):

    def __init__(self, out_dir):
        super().__init__('stereo_video')
        self.out_dir = out_dir
        self.writers = {}
        self.counts = {'left': 0, 'right': 0}
        self.stamps = open(os.path.join(out_dir, 'stamps.csv'), 'w', newline='')
        self.csv = csv.writer(self.stamps)
        self.csv.writerow(['eye', 'frame', 'stamp_s'])
        for eye in ('left', 'right'):
            self.create_subscription(Image, f'/oak/{eye}/image_mono',
                                     lambda m, e=eye: self.on_image(e, m), qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.create_timer(5.0, self.report)

    def on_image(self, eye, msg):
        img = to_gray(msg)
        w = self.writers.get(eye)
        if w is None:
            path = os.path.join(self.out_dir, f'{eye}.avi')
            w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), FPS,
                                (img.shape[1], img.shape[0]), False)
            if not w.isOpened():
                raise RuntimeError(f'cannot open {path} for writing')
            self.writers[eye] = w
            self.get_logger().info(f'{eye}: {img.shape[1]}x{img.shape[0]} {msg.encoding} -> {path}')
        w.write(np.ascontiguousarray(img))
        self.csv.writerow([eye, self.counts[eye], f'{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.6f}'])
        self.counts[eye] += 1

    def report(self):
        dt = time.monotonic() - self.t0
        self.get_logger().info(f'{dt:5.0f} s  left {self.counts["left"]} ({self.counts["left"] / dt:.1f} fps)'
                               f'  right {self.counts["right"]} ({self.counts["right"] / dt:.1f} fps)')

    def close(self):
        for w in self.writers.values():
            w.release()
        self.stamps.close()


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    root = sys.argv[2] if len(sys.argv) > 2 else '/root/team_ws/videos'
    out_dir = os.path.join(root, time.strftime('stereo-%Y%m%d-%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = StereoVideo(out_dir)
    stop = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    print(f'recording to {out_dir}' + (f' for {seconds:.0f} s' if seconds else ' until Ctrl-C'), flush=True)
    try:
        while rclpy.ok() and not stop['now'] and (not seconds or time.monotonic() - node.t0 < seconds):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f'saved {out_dir}: left {node.counts["left"]} frames, right {node.counts["right"]} frames',
              flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
