"""Measure camera / depth / VIO / bridge rates and latency for SECONDS (inside skybeta-stack).

    python3 measure.py SECONDS OUT.json

Uses camera_info (same header stamp as the image, tiny) so the probe itself adds no load.
Rates come from header stamps; age = Jetson now - header stamp at arrival.
Read-only: subscribes only.
"""
import json, statistics as st, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
try:
    from px4_msgs.msg import VehicleOdometry
except ImportError:
    VehicleOdometry = None

secs, out = float(sys.argv[1]), sys.argv[2]
rclpy.init()
n = Node('depth_sweep_measure')
D = {}


def stamp_cb(name):
    D[name] = {'stamp': [], 'age': [], 'size': None}

    def cb(m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        D[name]['stamp'].append(s)
        D[name]['age'].append((n.get_clock().now().nanoseconds * 1e-9 - s) * 1000.0)
        if D[name]['size'] is None and hasattr(m, 'width'):
            D[name]['size'] = f'{m.width}x{m.height}'
    return cb


for name, topic, typ in [('left', '/oak/left/camera_info', CameraInfo),
                         ('right', '/oak/right/camera_info', CameraInfo),
                         ('depth', '/oak/stereo/camera_info', CameraInfo),
                         ('vio', '/visual_slam/tracking/odometry', Odometry)]:
    n.create_subscription(typ, topic, stamp_cb(name), qos_profile_sensor_data)

bridge = []
if VehicleOdometry is not None:
    PQ = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST, depth=10)
    n.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
                          lambda m: bridge.append((m.timestamp - m.timestamp_sample) / 1000.0), PQ)

end = time.time() + secs
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.02)


def pct(x, p):
    x = sorted(x)
    return round(x[min(len(x) - 1, int(p * (len(x) - 1)))], 1)


def summ(x):
    return {'n': len(x), 'p50': pct(x, .5), 'p95': pct(x, .95), 'max': round(max(x), 1)} if x else {'n': 0}


res = {'seconds': secs}
for k, v in D.items():
    s = sorted(set(v['stamp']))
    r = {'n': len(s), 'size': v['size']}
    if len(s) > 2:
        gaps = [b - a for a, b in zip(s, s[1:])]
        med = st.median(gaps)
        r['rate_hz'] = round((len(s) - 1) / (s[-1] - s[0]), 2)
        r['gaps_over_1p5'] = sum(g > 1.5 * med for g in gaps)
        r['age_ms'] = summ(v['age'])
    res[k] = r
res['bridge_lag_ms'] = summ(bridge)
json.dump(res, open(out, 'w'), indent=1)
print(json.dumps(res))
