"""Landing-pad detector: camera frames in, /pad/detection out.

    subscribe  /oak/left/image_mono       grayscale 1024x640, ~30 Hz
    subscribe  /oak/left/camera_info      intrinsics (distortion assumed zero)
    publish    /pad/detection             JSON, one message per frame

This node only reports. It never publishes a setpoint and never talks to
/fmu/*. The model lives in ``yolo_pad.py``; with no weights it reports no pad.

A detection becomes a position in ``odom``: the box center is unprojected with
CameraInfo.k, rotated into ``odom`` with TF, and intersected with the ground
plane ``z = ground_z``. ``confirmed`` needs several consecutive detections
whose ground points agree, so one stray frame is never a match.

The image subscription keeps only the newest frame (depth 1, best effort), so
slow inference drops frames instead of building a backlog next to cuVSLAM.
"""
import json

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from course_pipeline.pad_math import Confirmer, pixel_to_ground, quat_to_matrix
from course_pipeline.yolo_pad import PadModel


def image_to_gray(msg):
    """sensor_msgs/Image -> H x W uint8 numpy array, or None if unsupported."""
    import numpy as np
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('mono8', '8UC1'):
        gray = buf.reshape(msg.height, msg.step)[:, :msg.width]
    elif msg.encoding in ('bgr8', 'rgb8'):
        # The camera is monochrome: the three channels are identical.
        rows = buf.reshape(msg.height, msg.step)[:, :msg.width * 3]
        gray = rows.reshape(msg.height, msg.width, 3)[:, :, 0]
    else:
        return None
    return np.ascontiguousarray(gray)


class PadDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('pad_detector', **kwargs)

        self.declare_parameter('weights_path', '')
        self.declare_parameter('image_topic', '/oak/left/image_mono')
        self.declare_parameter('camera_info_topic', '/oak/left/camera_info')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('confirm_radius_m', 0.5)
        # Height of the floor the pad lies on, in odom. odom z = 0 is the
        # startup pose, which is ground level if the drone started on the ground.
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', '')

        def param(name):
            return self.get_parameter(name).value

        self.odom_frame = param('odom_frame')
        self.min_confidence = float(param('min_confidence'))
        self.ground_z = float(param('ground_z'))
        self.max_range_m = float(param('max_range_m'))

        self.model = PadModel(
            weights_path=param('weights_path'),
            confidence=self.min_confidence,
            image_size=int(param('image_size')),
            device=param('device') or None)
        self.confirmer = Confirmer(
            frames=int(param('confirm_frames')),
            radius_m=float(param('confirm_radius_m')))

        self.camera_info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.warned = set()

        self.detection_pub = self.create_publisher(
            String, '/pad/detection', 10)
        self.create_subscription(
            CameraInfo, param('camera_info_topic'), self.camera_info_cb, 10)
        self.create_subscription(
            Image, param('image_topic'), self.image_cb,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))

        if self.model.available:
            self.get_logger().info(
                f'pad model loaded from {self.model.weights_path}')
        else:
            self.get_logger().warn(
                f'pad model unavailable ({self.model.error}); '
                'publishing detected=false until weights are provided.')

    # ------------------------------------------------------------------
    def warn_once(self, key, message):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(message)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def camera_info_cb(self, msg):
        if self.camera_info is None:
            self.camera_info = msg

    # ------------------------------------------------------------------
    def ground_point(self, detection, image_msg):
        """Return (position in odom or None, tf_exact)."""
        frame = image_msg.header.frame_id or self.camera_info.header.frame_id
        transform, exact = None, True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, frame, Time.from_msg(image_msg.header.stamp),
                timeout=Duration(seconds=0.0))
        except TransformException:
            # The newest odometry TF often lags the newest frame slightly.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame, frame, Time(),
                    timeout=Duration(seconds=0.0))
                exact = False
            except TransformException as exc:
                self.warn_once(
                    'tf', f'no TF {self.odom_frame} <- {frame} yet: {exc}')
                return None, False
        t = transform.transform.translation
        q = transform.transform.rotation
        try:
            rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
        except ValueError:
            return None, exact
        position = pixel_to_ground(
            detection.cx, detection.cy, self.camera_info.k, rotation,
            (t.x, t.y, t.z), self.ground_z, self.max_range_m)
        return position, exact

    def image_cb(self, msg):
        if self.camera_info is None:
            self.warn_once('info', 'no CameraInfo yet; not detecting.')
            return
        gray = image_to_gray(msg)
        if gray is None:
            self.warn_once(
                'encoding', f'unsupported image encoding {msg.encoding!r}')
            return

        candidates = [d for d in self.model.detect(gray)
                      if d.confidence >= self.min_confidence]
        best = max(candidates, key=lambda d: d.confidence, default=None)

        position, tf_exact = None, None
        if best is not None:
            position, tf_exact = self.ground_point(best, msg)
        confirmed = self.confirmer.update(position)

        report = {
            'stamp': round(self.now_s(), 3),
            'detected': best is not None,
            'confidence': round(best.confidence, 3) if best else 0.0,
            'pixel': [round(best.cx, 1), round(best.cy, 1)] if best else None,
            'position_enu': ([round(v, 3) for v in position]
                             if position else None),
            'confirmed': confirmed,
            'streak': self.confirmer.count,
            'tf_exact': tf_exact,
        }
        out = String()
        out.data = json.dumps(report)
        self.detection_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PadDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
