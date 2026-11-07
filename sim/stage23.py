#!/usr/bin/env python3
"""SIM ONLY (it arms): simple localizer + PX4 commands, with depth awareness.

Localizer: PX4 local position (fed by cuVSLAM). Commands: Offboard position setpoints.
Flow: Offboard, arm, climb, read depth clearance ahead, fly LEG_M forward (shortened to
fit the clearance), fly back to the start, land. Stops flying after RUN_LIMIT_S and lands.
"""
import math
import os
import time

import numpy as np
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image

HEIGHT = float(os.environ.get('HEIGHT_M', '1.2'))
LEG = float(os.environ.get('LEG_M', '1.0'))
RUN_LIMIT_S = 60.0
STOP_MARGIN = 0.55
NAN = float('nan')

def log(msg):
    print('[stage23] ' + msg, flush=True)

class Stage(Node):
    def __init__(self):
        super().__init__('stage23')
        pub = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        sub = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.ocm = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub)
        self.sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub)
        self.cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_pos, sub)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, sub)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.pos = None
        self.status = None
        self.depth = (None, 0.0)

    def on_pos(self, m):
        ok = bool(m.xy_valid and m.z_valid and math.isfinite(m.heading))
        self.pos = (m.x, m.y, m.z, m.heading) if ok else None

    def on_status(self, m):
        self.status = m

    def on_depth(self, m):
        a = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.width)
        h, w = a.shape
        band = a[int(h * 0.35):int(h * 0.65), int(w * 0.35):int(w * 0.65)]
        valid = band[(band > 350) & (band < 8000)]
        clear = float(np.percentile(valid, 10)) / 1000.0 if valid.size > 50 else None
        self.depth = (clear, valid.size / band.size)

    def stamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def send(self, p, yaw):
        t = self.stamp()
        o = OffboardControlMode()
        o.timestamp = t
        o.position = True
        self.ocm.publish(o)
        s = TrajectorySetpoint()
        s.timestamp = t
        s.position = [float(p[0]), float(p[1]), float(p[2])]
        s.velocity = [NAN, NAN, NAN]
        s.acceleration = [NAN, NAN, NAN]
        s.yaw = float(yaw)
        self.sp.publish(s)

    def command(self, c, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.stamp()
        m.command = c
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd.publish(m)

    def armed(self):
        return self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

def main():
    rclpy.init()
    n = Stage()
    t_start = time.time()
    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)
    log('waiting for PX4 position, status and depth')
    while (n.pos is None or n.status is None) and time.time() - t_start < 60:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pos is None or n.status is None:
        log('FAIL: no valid PX4 position/status')
        return
    x0, y0, z0, yaw0 = n.pos
    cur = [x0, y0, z0]
    log('start NED=(%.2f, %.2f, %.2f) heading=%.2f rad (0 = north)' % (x0, y0, z0, yaw0))

    def tick():
        n.send(cur, yaw0)
        spin(0.05)

    def fly(target, speed, label, max_s=40.0, tol=0.12):
        log('%s -> NED(%.2f, %.2f, %.2f)' % (label, target[0], target[1], target[2]))
        t0, hold0, last = time.time(), None, 0.0
        while time.time() - t0 < max_s and time.time() - t_start < RUN_LIMIT_S + 30:
            d = [target[i] - cur[i] for i in range(3)]
            dist = math.sqrt(sum(v * v for v in d))
            step = min(dist, speed * 0.05)
            if dist > 1e-6:
                for i in range(3):
                    cur[i] += d[i] / dist * step
            tick()
            p = n.pos
            if time.time() - last >= 2.0 and p:
                last = time.time()
                log('  t=%5.1f pos=(%.2f, %.2f, %.2f) moved=%.2f m depth clear=%s valid=%.0f%%' % (
                    time.time() - t_start, p[0], p[1], -(p[2] - z0), math.hypot(p[0] - x0, p[1] - y0),
                    ('%.2f' % n.depth[0]) if n.depth[0] else 'n/a', 100 * n.depth[1]))
            if p and math.dist(p[:3], target) < tol:
                hold0 = hold0 or time.time()
                if time.time() - hold0 > 1.0:
                    return True
            else:
                hold0 = None
        return False

    result = 'ok'
    try:
        for _ in range(30):
            tick()
        def why():
            st = n.status
            return 'arming_state=%s nav_state=%s preflight_ok=%s failsafe=%s' % (
                st.arming_state, st.nav_state, st.pre_flight_checks_pass, st.failsafe)
        log('requesting Offboard (setpoint stream running): ' + why())
        t0 = time.time()
        while n.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and time.time() - t0 < 10:
            n.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            tick()
        log('after Offboard request: ' + why())
        log('arming')
        t0 = time.time()
        while not n.armed() and time.time() - t0 < 20:
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            tick()
        if not n.armed():
            raise RuntimeError('PX4 did not arm: ' + why())
        log('armed: ' + why())
        if not fly([x0, y0, z0 - HEIGHT], 0.5, 'climb to %.1f m' % HEIGHT):
            raise RuntimeError('climb did not finish')
        for _ in range(40):
            tick()
        clear, frac = n.depth
        leg = LEG if clear is None else min(LEG, max(0.0, clear - STOP_MARGIN))
        log('DEPTH STATE: clearance ahead=%s valid=%.0f%% -> leg %.2f m (asked %.2f)' % (
            ('%.2f m' % clear) if clear else 'n/a', 100 * frac, leg, LEG))
        fwd = [x0 + leg * math.cos(yaw0), y0 + leg * math.sin(yaw0), z0 - HEIGHT]
        if time.time() - t_start < RUN_LIMIT_S and leg > 0.05:
            if not fly(fwd, 0.3, 'forward %.2f m' % leg):
                log('forward leg not reached')
        if time.time() - t_start < RUN_LIMIT_S:
            if not fly([x0, y0, z0 - HEIGHT], 0.3, 'back to start'):
                log('return leg not reached')
    except Exception as exc:
        result = 'error: %s' % exc
        log(result)
    finally:
        log('landing')
        t0 = time.time()
        while n.armed() and time.time() - t0 < 40:
            n.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            spin(0.5)
        if n.armed():
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        p = n.pos
        log('RESULT: %s | landed=%s | final offset from start %.2f m | elapsed %.0f s' % (
            result, not n.armed(), math.hypot(p[0] - x0, p[1] - y0) if p else -1, time.time() - t_start))
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
