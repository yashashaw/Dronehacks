"""The organisers' example square (competitor_example, newest version), with two changes:
1. every position this node uses comes from PX4 (/fmu/out/vehicle_local_position), not VIO;
2. leg 1 of the square points along the nose at the moment Offboard starts (PX4 heading);
   the example used VIO's +x, i.e. the nose direction when the stack started.

Why: the example takes VIO's ABSOLUTE height when Offboard starts and adds 1.0 m. With
flight_interface's default anchor_setpoints=false that number goes to PX4 unchanged, and PX4
measures height from the rangefinder/baro (floor = ~0). Any VIO height error on the ground
(drone 7: 26 m) becomes a climb. Here the start point, the targets and "have I arrived" are
all PX4's own estimate, the same one PX4's controller holds. VIO still reaches PX4, but only
inside EKF2 and only for horizontal position + yaw (EKF2_EV_CTRL = 9); PX4 height is the
rangefinder with the baro as backup (EKF2_HGT_REF = 2). No depth camera is used.

Mission (same as the example): wait for Offboard, climb to 1.0 m above where Offboard started,
hover 5 s, fly a 0.5 m square counter-clockwise (leg 1 along the nose at the switch), hold.
It does not land: the safety pilot takes over and lands.

Contract: publishes /skybeta/setpoint/pose only. Never publishes /fmu/in/*, never arms, never
changes mode. Run with exactly one flight_interface, anchor_setpoints FALSE (see the launch
file): the ENU numbers sent here are PX4's local frame converted to ENU, so anchoring would
shift them by the VIO origin and break that.

No takeover unless (checked every tick before Offboard; failing = publish nothing, so the
pilot keeps Position): PX4 local position fresh and valid (xy and z), and PX4 height on the
ground within GROUND_Z_MAX_M of its zero.
"""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

from px4_square.geometry import ned_to_enu, square_corners_enu

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LPOS_STALE_S = 0.5      # PX4 local position older than this = no position
GROUND_Z_MAX_M = 0.3    # |PX4 height| on the ground before a takeover is allowed


class Px4Square(Node):

    def __init__(self):
        super().__init__('px4_square')
        self.declare_parameter('hover_altitude_m', 1.0)
        self.declare_parameter('hover_seconds', 5.0)
        self.declare_parameter('square_side_m', 0.5)
        self.declare_parameter('arrival_radius_m', 0.3)
        self.declare_parameter('corner_timeout_s', 60.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.hover_altitude = float(self.get_parameter('hover_altitude_m').value)
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)
        self.side = float(self.get_parameter('square_side_m').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius_m').value)
        self.corner_timeout = float(self.get_parameter('corner_timeout_s').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_lpos, PX4_QOS)

        self.lpos = None
        self.lpos_t = 0.0
        self.offboard = False
        self.was_offboard = False
        self.state = 'WAIT_FOR_OFFBOARD'
        self.start = None
        self.corners = []
        self.corner_index = 0
        self.hover_started_at = None
        self.corner_started_at = None
        self.last_warn = {}

        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'px4_square ready (PX4 position only, no VIO height, no depth): hover '
            f'{self.hover_altitude:.2f} m for {self.hover_seconds:.0f} s, then a '
            f'{self.side:.2f} m square. Pilot: arm on Altitude, then switch to Position.')

    # ---------------- inputs ----------------
    def on_lpos(self, msg):
        self.lpos = msg
        self.lpos_t = time.monotonic()

    def on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(s.get('offboard_active', False))
        clamped = [r for r in (s.get('clamped') or []) if r != 'min_altitude']
        if clamped and self.offboard:
            self.warn_every('clamp', f'flight_interface is CLAMPING our setpoint: {clamped}')

    def warn_every(self, key, text, every=2.0):
        now = time.monotonic()
        if now - self.last_warn.get(key, -1e9) >= every:
            self.last_warn[key] = now
            self.get_logger().warn(text)

    # ---------------- PX4 position ----------------
    def position_enu(self):
        """PX4's local position as ENU, or None if missing/stale/invalid."""
        lp = self.lpos
        if lp is None or time.monotonic() - self.lpos_t > LPOS_STALE_S:
            return None
        if not (lp.xy_valid and lp.z_valid):
            return None
        return ned_to_enu(lp.x, lp.y, lp.z)

    def send(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # orientation all-zero: do not command yaw (same as the example)
        self.pub.publish(msg)

    def arrived_at(self, pos, target):
        return math.dist(pos, target) <= self.arrival_radius

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- 10 Hz state machine ----------------
    def tick(self):
        pos = self.position_enu()
        if pos is None:
            self.warn_every('nopos', 'no fresh, valid PX4 local position (xy and z): '
                                     'publishing nothing, so no takeover')
            return

        if self.was_offboard and not self.offboard:
            self.get_logger().warn('Offboard lost: the pilot has it. Mission stopped until '
                                   'the drone is disarmed and armed again.')
            self.was_offboard = False
            self.state = 'WAIT_FOR_OFFBOARD'
            self.start, self.corners = None, []
        if self.offboard:
            self.was_offboard = True

        getattr(self, f'_state_{self.state.lower()}')(pos)

    def _state_wait_for_offboard(self, pos):
        if not self.offboard:
            if abs(pos[2]) > GROUND_Z_MAX_M:
                self.warn_every('ground', f'PX4 height {pos[2]:.2f} m is not ~0 on the ground: '
                                          'publishing nothing, so no takeover')
                return
            self.send(*pos)     # hold where we are; keeps the setpoint fresh for the takeover
            return
        self.start = pos
        heading = float(self.lpos.heading)
        z = self.start[2] + self.hover_altitude
        self.corners = square_corners_enu(self.start, heading, self.side, z)
        self.hover_started_at = self.now_s()
        self.state = 'HOVER'
        self.get_logger().info(
            f'Offboard active. Start (PX4, ENU) ({self.start[0]:.2f}, {self.start[1]:.2f}, '
            f'{self.start[2]:.2f}), heading {math.degrees(heading):.0f} deg; climbing to '
            f'{z:.2f} m')

    def _state_hover(self, pos):
        target = (self.start[0], self.start[1], self.start[2] + self.hover_altitude)
        self.send(*target)
        if not self.arrived_at(pos, target):
            self.hover_started_at = self.now_s()
            return
        if self.now_s() - self.hover_started_at < self.hover_seconds:
            return
        self.corner_index = 0
        self.corner_started_at = self.now_s()
        self.state = 'SQUARE'
        self.get_logger().info(f'hover done: square, corner 1/{len(self.corners)}')

    def _state_square(self, pos):
        target = self.corners[self.corner_index]
        self.send(*target)
        if self.arrived_at(pos, target):
            self.corner_index += 1
            self.corner_started_at = self.now_s()
            if self.corner_index >= len(self.corners):
                self.state = 'HOLD'
                self.get_logger().info('square complete: holding. Pilot takes over and lands.')
            else:
                self.get_logger().info(f'corner reached: next {self.corner_index + 1}/'
                                       f'{len(self.corners)}')
            return
        if self.now_s() - self.corner_started_at > self.corner_timeout:
            self.get_logger().error(f'corner {self.corner_index + 1} not reached in '
                                    f'{self.corner_timeout:.0f} s: holding')
            self.state = 'HOLD'

    def _state_hold(self, pos):
        self.send(*self.corners[-1])


def main(args=None):
    rclpy.init(args=args)
    node = Px4Square()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
