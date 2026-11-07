#!/usr/bin/env python3
"""Simplest real flight: take off to 0.7 m above where Offboard starts, hover 5 s, land.

Started by hover.launch.py together with the ONE flight_interface (auto_offboard_on_arm,
anchor_setpoints). Nothing here arms. The flight is:

  1. pilot arms on Altitude, then flips to Position within a few seconds
  2. flight_interface asks PX4 for Offboard; this node climbs to start + ALT, holds HOVER_S
  3. this node sends PX4 LAND (the only /fmu/in message it sends: setpoints are floored at
     0.3 m, so setpoints alone can never land); PX4 descends and disarms on touchdown
  Pilot takes over at any time with the sticks / mode switch / kill switch. Once Offboard ends
  this node stops and never commands again.

Safety, from the drone 7 climb incident (F1_CLIMB_INCIDENT.md on drone-field-tests): VIO can be
metres wrong while reporting "tracking". So:
  * no setpoint (= no takeover) unless VIO height agrees with PX4's height (rangefinder/baro)
    within GATE_M on the ground
  * targets are relative to where Offboard started, and flight_interface runs anchor_setpoints
  * LAND at once if the rangefinder reads above ALT + 0.5 m or 2.0 m, if VIO and the
    rangefinder disagree by more than 0.4 m in the air, or 30 s after takeoff whatever happens
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

ALT = 0.7            # m above the Offboard start point
HOVER_S = 5.0        # s of hover once at height
ARRIVED_M = 0.15     # height error that counts as "at height"
GATE_M = 0.3         # max |VIO height - PX4 height| on the ground before we allow a takeover
RANGE_MAX = min(ALT + 0.5, 2.0)   # rangefinder ceiling -> LAND
DISAGREE_M = 0.4     # max |VIO climb - rangefinder climb| in the air -> LAND
MAX_FLIGHT_S = 30.0  # LAND after this long in Offboard, whatever happens

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class Hover(Node):
    def __init__(self):
        super().__init__('hover_test')
        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'lpos', m), PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 lambda m: setattr(self, 'status', m), PX4_QOS)
        self.vio = None          # (x, y, z) ENU
        self.lpos = None
        self.status = None
        self.state = {}
        self.phase = 'WAIT'      # WAIT -> CLIMB -> HOVER -> LAND -> DONE
        self.start = None        # VIO position when Offboard started
        self.range0 = 0.0        # rangefinder reading at that moment
        self.t_offboard = None
        self.t_hover = None
        self.land_sent = 0
        self.last_log = 0.0
        self.create_timer(0.1, self.tick)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.vio = (p.x, p.y, p.z)

    def on_state(self, m):
        try:
            self.state = json.loads(m.data)
        except ValueError:
            pass

    def say(self, msg, every=0.0):
        if time.monotonic() - self.last_log >= every:
            self.get_logger().info(msg)
            self.last_log = time.monotonic()

    def send(self, xyz):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
        self.pub.publish(m)      # orientation all-zero: do not command yaw

    def land(self, why):
        if self.phase not in ('LAND', 'DONE'):
            self.get_logger().warn(f'LAND: {why}')
            self.phase = 'LAND'
        if self.land_sent < 10:
            c = VehicleCommand()
            c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            c.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
            sysid = int(self.status.system_id) if self.status is not None else 1
            c.target_system, c.target_component = sysid, 1
            c.source_system, c.source_component = 1, 1
            c.from_external = True
            self.cmd.publish(c)
            self.land_sent += 1

    def range_m(self):
        lp = self.lpos
        return float(lp.dist_bottom) if lp is not None and lp.dist_bottom_valid else None

    def tick(self):
        if self.vio is None or self.lpos is None:
            self.say('waiting for VIO and PX4 local position ...', every=2.0)
            return
        offboard = bool(self.state.get('offboard_active', False))
        vio_h = self.vio[2]
        px4_h = -float(self.lpos.z)

        if self.phase == 'WAIT':
            if offboard:
                self.phase, self.start, self.t_offboard = 'CLIMB', self.vio, time.monotonic()
                self.range0 = self.range_m() or 0.0
                self.get_logger().info(f'Offboard: climbing to {ALT:.1f} m above {tuple(round(v, 2) for v in self.start)}')
            else:
                # Gate: only offer a setpoint (= allow the takeover) while VIO height is sane.
                if abs(vio_h - px4_h) > GATE_M:
                    self.say(f'NOT READY: VIO height {vio_h:+.2f} m vs PX4 {px4_h:+.2f} m (> {GATE_M} m apart). '
                             'No takeover. Restart the stack with the drone still.', every=2.0)
                    return
                self.send(self.vio)
                self.say(f'ready: VIO/PX4 height {vio_h:+.2f}/{px4_h:+.2f} m. Pilot: arm on Altitude, then Position.',
                         every=5.0)
                return

        if self.phase in ('CLIMB', 'HOVER') and not offboard:
            self.get_logger().warn('Offboard ended: the pilot has it. Not commanding anything more.')
            self.phase = 'DONE'
            return

        if self.phase in ('CLIMB', 'HOVER'):
            rng = self.range_m()
            climb_vio = vio_h - self.start[2]
            if time.monotonic() - self.t_offboard > MAX_FLIGHT_S:
                self.land(f'{MAX_FLIGHT_S:.0f} s in Offboard')
            elif rng is not None and rng > RANGE_MAX:
                self.land(f'rangefinder {rng:.2f} m > {RANGE_MAX:.2f} m')
            elif rng is not None and rng > 0.3 and abs(climb_vio - (rng - self.range0)) > DISAGREE_M:
                self.land(f'VIO climb {climb_vio:.2f} m vs rangefinder climb {rng - self.range0:.2f} m')

        if self.phase == 'CLIMB':
            target = (self.start[0], self.start[1], self.start[2] + ALT)
            self.send(target)
            if abs(vio_h - target[2]) < ARRIVED_M:
                self.phase, self.t_hover = 'HOVER', time.monotonic()
                self.get_logger().info(f'at height: hovering {HOVER_S:.0f} s')
        elif self.phase == 'HOVER':
            self.send((self.start[0], self.start[1], self.start[2] + ALT))
            if time.monotonic() - self.t_hover >= HOVER_S:
                self.land('hover done')
        elif self.phase == 'LAND':
            self.land('')        # resend a few times; PX4 leaves Offboard into Land
            if self.state.get('landed') and not self.state.get('armed'):
                self.get_logger().info('landed and disarmed. Done.')
                self.phase = 'DONE'

        rng = self.range_m()
        self.say(f'{self.phase}: VIO h {vio_h:+.2f} m, PX4 h {px4_h:+.2f} m, '
                 f'range {"--" if rng is None else f"{rng:.2f}"} m', every=1.0)


def main():
    rclpy.init()
    n = Hover()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
