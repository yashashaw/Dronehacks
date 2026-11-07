#!/usr/bin/env python3
"""Pre-flight measurements over DDS (runs in the skybeta container). READ-ONLY:
subscribes and queries parameters; publishes nothing, commands nothing.

  python3 preflight_check.py --seconds 20 --host-json /root/team_ws/loc_stack/preflight/host.json \
      --out /root/team_ws/loc_stack/preflight/out
  python3 preflight_check.py --mode lift --seconds 15 --out ...

check: measures for SECONDS with the drone still, then applies preflight_rules and writes
       report.txt, result.json and DONE (exit code 0 GO, 2 WAIT, 1 NO-GO).
lift:  records rangefinder distance and VIO height while someone lifts the drone ~0.5 m.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import preflight_rules as pr  # noqa: E402

from px4_msgs.msg import (BatteryStatus, EstimatorStatusFlags, VehicleLandDetected,  # noqa: E402
                          VehicleLocalPosition, VehicleOdometry, VehicleStatus)
try:
    from px4_msgs.msg import FailsafeFlags
except ImportError:
    FailsafeFlags = None
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None
try:
    from skybeta_flight_interface.topics import fmu_topic
except ImportError:
    def fmu_topic(direction, base, msg=None, msg_dir=None):
        return f'/fmu/{direction}/{base}'

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
SINGLE_PUBLISHER = ['/fmu/in/vehicle_visual_odometry', '/fmu/in/offboard_control_mode',
                    '/fmu/in/trajectory_setpoint', '/skybeta/setpoint/pose',
                    '/skybeta/setpoint/twist']


def hdr(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def rate(stamps):
    s = sorted(set(stamps))
    return round((len(s) - 1) / (s[-1] - s[0]), 2) if len(s) > 2 and s[-1] > s[0] else None


class Collector(Node):

    def __init__(self):
        super().__init__('preflight_check')
        q = qos_profile_sensor_data
        self.t = {k: [] for k in ('left', 'right', 'imu', 'vio', 'bridge', 'depth')}
        self.vio_pos, self.vo_states = [], set()
        self.status = self.landed = self.battery = self.failsafe = None
        self.lpos, self.flags, self.sky = [], [], None
        sub = self.create_subscription
        sub(CameraInfo, '/oak/left/camera_info', lambda m: self.t['left'].append(hdr(m)), q)
        sub(CameraInfo, '/oak/right/camera_info', lambda m: self.t['right'].append(hdr(m)), q)
        sub(Imu, '/oak/imu/data', lambda m: self.t['imu'].append(hdr(m)), q)
        sub(Image, '/oak/stereo/image_raw', lambda m: self.t['depth'].append(hdr(m)), q)
        sub(Odometry, '/visual_slam/tracking/odometry', self.on_vio, q)
        sub(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
            lambda m: self.t['bridge'].append(time.monotonic()), PX4_QOS)
        if VisualSlamStatus is not None:
            sub(VisualSlamStatus, '/visual_slam/status',
                lambda m: self.vo_states.add(int(m.vo_state)), q)
        sub(VehicleStatus, fmu_topic('out', 'vehicle_status', 'VehicleStatus'),
            lambda m: setattr(self, 'status', m), PX4_QOS)
        sub(VehicleLocalPosition, fmu_topic('out', 'vehicle_local_position', 'VehicleLocalPosition'),
            lambda m: self.lpos.append(m), PX4_QOS)
        sub(EstimatorStatusFlags, fmu_topic('out', 'estimator_status_flags', 'EstimatorStatusFlags'),
            lambda m: self.flags.append(m), PX4_QOS)
        sub(BatteryStatus, fmu_topic('out', 'battery_status', 'BatteryStatus'),
            lambda m: setattr(self, 'battery', m), PX4_QOS)
        sub(VehicleLandDetected, fmu_topic('out', 'vehicle_land_detected', 'VehicleLandDetected'),
            lambda m: setattr(self, 'landed', bool(m.landed)), PX4_QOS)
        if FailsafeFlags is not None:
            sub(FailsafeFlags, fmu_topic('out', 'failsafe_flags', 'FailsafeFlags'),
                lambda m: setattr(self, 'failsafe', m), PX4_QOS)
        sub(String, '/skybeta/state', self.on_sky, 10)

    def on_vio(self, m):
        self.t['vio'].append(hdr(m))
        p = m.pose.pose.position
        self.vio_pos.append((p.x, p.y, p.z))

    def on_sky(self, m):
        try:
            self.sky = json.loads(m.data)
        except ValueError:
            pass

    # -------------------------------------------------------------- graph
    def flight_interface(self):
        names = self.get_node_names_and_namespaces()
        hit = [(n, ns) for n, ns in names if n == 'flight_interface']
        if not hit:
            return {'present': False}
        n, ns = hit[0]
        full = (ns.rstrip('/') + '/' + n) if ns != '/' else '/' + n
        cli = self.create_client(GetParameters, f'{full}/get_parameters')
        out = {'present': True, 'node': full}
        if cli.wait_for_service(timeout_sec=3.0):
            req = GetParameters.Request(names=['target_system', 'auto_offboard_on_arm',
                                               'min_altitude_m', 'max_altitude_m'])
            fut = cli.call_async(req)
            end = time.monotonic() + 3.0
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done() and fut.result():
                v = fut.result().values
                out['target_system'] = v[0].integer_value
                out['auto_offboard_on_arm'] = v[1].bool_value
                out['altitude_band_m'] = [v[2].double_value, v[3].double_value]
        return out

    # -------------------------------------------------------------- summary
    def summary(self):
        live = {'rates': {k: rate(v) for k, v in self.t.items()}}
        b = self.t['bridge']
        live['rates']['bridge'] = (round((len(b) - 1) / (b[-1] - b[0]), 2)
                                   if len(b) > 2 and b[-1] > b[0] else None)
        p = self.vio_pos
        if p:
            steps = [math.dist(a, c) for a, c in zip(p, p[1:])]
            live['vio'] = {'n': len(p), 'states': sorted(self.vo_states),
                           'max_step_m': round(max(steps), 4) if steps else 0.0,
                           'wander_m': round(max(math.dist(p[0], q) for q in p), 4),
                           'pos_last': [round(v, 3) for v in p[-1]]}
        st = self.status
        live['fc'] = {'link': st is not None}
        if st is not None:
            live['fc'].update({
                'system_id': int(getattr(st, 'system_id', -1)),
                'armed': st.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                'nav_state': int(st.nav_state), 'failsafe': bool(st.failsafe),
                'preflight_ok': bool(getattr(st, 'pre_flight_checks_pass', False))})
        live['landed'] = self.landed
        if self.battery is not None:
            bt = self.battery
            live['battery'] = {'connected': bool(bt.connected), 'voltage': float(bt.voltage_v),
                               'remaining': float(bt.remaining),
                               'cells': int(getattr(bt, 'cell_count', 0) or 4)}
        if self.failsafe is not None:
            f = self.failsafe
            live['failsafe_true'] = [k for k, t in f.get_fields_and_field_types().items()
                                     if t == 'boolean' and getattr(f, k)]
        if self.lpos:
            L = self.lpos
            xy = [(m.x, m.y) for m in L]
            db = [m.dist_bottom for m in L]
            live['lpos'] = {
                'xy_valid_all': all(m.xy_valid for m in L),
                'eph_max': round(max(m.eph for m in L), 4),
                'reset_change': int(L[-1].xy_reset_counter) - int(L[0].xy_reset_counter),
                'xy_jump_max': round(max((math.dist(a, c) for a, c in zip(xy, xy[1:])),
                                         default=0.0), 4),
                'heading_deg': round(math.degrees(L[-1].heading), 1),
                'dist_bottom_min': round(min(db), 3), 'dist_bottom_max': round(max(db), 3),
                'dist_bottom_valid_all': all(m.dist_bottom_valid for m in L)}
        if self.flags:
            F = self.flags
            live['flags'] = {k: round(sum(bool(getattr(m, k, False)) for m in F) / len(F), 3)
                             for k in ('cs_ev_pos', 'cs_ev_yaw', 'cs_mag', 'cs_rng_hgt',
                                       'cs_baro_hgt')}
        if self.sky:
            live['skybeta_state'] = {k: self.sky.get(k) for k in
                                     ('px4_link', 'origin_enu', 'odom_enu', 'armed',
                                      'offboard_active', 'nav_state_name')}
        live['flight_interface'] = self.flight_interface()
        live['publishers'] = {t: self.count_publishers(t) for t in SINGLE_PUBLISHER}
        live['nodes'] = sorted(n for n, _ in self.get_node_names_and_namespaces())
        return live


def report_text(checks, verdict_word, live):
    lines = [f'{"STATUS":5}  {"CHECK":28} DETAIL']
    for status, name, detail, fix in checks:
        lines.append(f'{status:5}  {name:28} {detail}')
        if fix and status != pr.PASS:
            lines.append(f'{"":5}  {"":28} -> {fix}')
    lines.append('')
    lines.append(f'VERDICT: {verdict_word}')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['check', 'lift'], default='check')
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--host-json', default=None)
    ap.add_argument('--profile', default='stock')
    ap.add_argument('--drone-id', type=int, default=int(os.environ.get('DRONE_ID', '7')))
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Collector()
    samples = []
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if a.mode == 'lift' and node.lpos:
                m = node.lpos[-1]
                z = node.vio_pos[-1][2] if node.vio_pos else None
                if not samples or samples[-1][3] is not m:
                    samples.append((float(m.dist_bottom), bool(m.dist_bottom_valid), z, m))
        if a.mode == 'lift':
            status, msg = pr.lift_report([s[:3] for s in samples])
            text = f'{status}  rangefinder lift check: {msg}'
            res = {'mode': 'lift', 'status': status, 'detail': msg, 'samples': len(samples)}
            code = 0 if status != pr.FAIL else 1
        else:
            host = {}
            if a.host_json and os.path.exists(a.host_json):
                with open(a.host_json) as f:
                    host = json.load(f)
            live = node.summary()
            cfg = pr.Config(drone_id=a.drone_id, profile=a.profile)
            checks = pr.evaluate(host, live, cfg)
            worst, word = pr.verdict(checks)
            text = report_text(checks, word, live)
            res = {'mode': 'check', 'verdict': word, 'worst': worst, 'checks': checks,
                   'host': host, 'live': live, 'window_s': a.seconds}
            code = {pr.FAIL: 1, pr.WAIT: 2}.get(worst, 0)
        with open(os.path.join(a.out, 'result.json'), 'w') as f:
            json.dump(res, f, indent=1, default=str)
        with open(os.path.join(a.out, 'report.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(a.out, 'DONE'), 'w') as f:
            f.write(f'{code}\n')
        print(text)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
