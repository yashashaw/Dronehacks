#!/usr/bin/env python3
"""Real flight: take off to 0.5 m above where Offboard starts, then straight into a built-in square
flown nose-first (no hover; hover-only mode holds 1 s), land. Lands at once if the depth camera sees something closer than DEPTH_LAND_M.

Started by hover.launch.py together with the ONE flight_interface (auto_offboard_on_arm,
anchor_setpoints). Nothing here arms. The flight is:

  1. pilot arms on Altitude, then flips to Position within a few seconds
  2. flight_interface asks PX4 for Offboard; this node climbs to start + ALT (hover-only: holds HOVER_S)
  3. with --square S (bg.sh square): the square is built in here, relative to where the drone
     is and which way its nose points when Offboard starts: leg 1 straight ahead, then turn
     left 90 deg in place and fly the next leg, four legs, back to the start. The drone only
     ever flies forwards (the camera looks where it goes): it turns first, then moves.
  4. LAND: a straight-down twist on /skybeta/setpoint/twist (poses are floored at 0.3 m,
     twists are not), every tick until PX4 disarms. This node never publishes to /fmu/in/*
     (organizers' autonomy_pipeline.md); it only READS /fmu/out/vehicle_local_position.
  Pilot takes over at any time with the sticks / mode switch / kill switch. Once Offboard ends
  this node stops commanding; after the drone is disarmed it is ready again for the next arming.

Safety, from the drone 7 climb incident (F1_CLIMB_INCIDENT.md on drone-field-tests): VIO can be
metres wrong while reporting "tracking". So:
  * no setpoint (= no takeover) unless VIO height agrees with PX4's height (rangefinder/baro)
    within GATE_M on the ground, and (with --square) the depth stream is live
  * targets are relative to where Offboard started, and flight_interface runs anchor_setpoints
  * LAND at once if the rangefinder reads above ALT + 0.5 m or 2.0 m, if VIO and the
    rangefinder disagree by more than 0.4 m in the air, or 30 s after takeoff whatever happens
  * LAND at once if the depth camera sees an obstacle closer than DEPTH_LAND_M ahead (checked
    from the hover on, not during the climb, when the floor is in view), or if depth stops
    arriving for DEPTH_STALE_S in the air (with --square)

Depth: /oak/stereo/image_raw (16UC1 mm, 0 = no reading) needs the depth camera profile
(README step 3). The OAK-D stereo cannot measure closer than ~0.31 m: anything nearer reads 0
(no reading), NOT near. So DEPTH_LAND_M is 0.35 m, the closest distance that can actually be
seen; a 0.2 m threshold would never fire.
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

ALT = 0.5            # m above the Offboard start point
HOVER_S = 1.0        # s of hover once at height
ARRIVED_M = 0.15     # height error that counts as "at height"
GATE_M = 0.3         # max |VIO height - PX4 height| on the ground before we allow a takeover
RANGE_MAX = min(ALT + 0.5, 2.0)   # rangefinder ceiling -> LAND
DISAGREE_M = 0.4     # max |VIO climb - rangefinder climb| in the air -> LAND
MAX_FLIGHT_S = 30.0  # LAND after this long in Offboard, whatever happens (60 s with --square)
CORNER_M = 0.15      # horizontal distance that counts as "at the corner"
CORNER_TIMEOUT_S = 12.0
TURN_LEFT = True     # square direction: True = turn left (CCW) at each corner
YAW_OK_DEG = 10.0    # heading error that counts as "facing the next leg"
TURN_TIMEOUT_S = 10.0
LAND_MPS = 0.3       # descent speed while landing (straight-down twist)

DEPTH_TOPIC = '/oak/stereo/image_raw'
DEPTH_LAND_M = 0.35  # obstacle closer than this ahead -> LAND (camera min range ~0.31 m)
DEPTH_FRAMES = 3     # consecutive close frames before landing (one noisy frame is not a wall)
DEPTH_STALE_S = 0.5  # no depth this long in the air (with --square) -> LAND

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)


def near_distance(msg):
    """Close-obstacle distance (m) ahead: 1st percentile of the valid depth pixels in the middle
    of the image (middle half wide, middle third tall: keeps the floor out at 0.5 m). None if
    too few valid pixels to say anything."""
    if msg.encoding not in ('16UC1', 'mono16'):
        return None
    img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
    h, w = msg.height, msg.width
    box = img[h // 3: 2 * h // 3, w // 4: 3 * w // 4]
    valid = box[box > 0]
    if valid.size < 0.01 * box.size:
        return None
    return float(np.percentile(valid, 1)) / 1000.0


def yaw_of(q):
    """ENU yaw (rad) of a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Hover(Node):
    def __init__(self, square_m=0.0):
        super().__init__('hover_test')
        self.side = square_m
        self.corners = []        # built when Offboard starts: [(x, y, heading), ...] ENU
        self.corner = 0
        self.t_corner = None
        self.max_flight_s = 60.0 if self.side > 0 else MAX_FLIGHT_S
        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'lpos', m), PX4_QOS)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, SENSOR_QOS)
        self.vio = None          # (x, y, z) ENU
        self.yaw = None          # ENU yaw from VIO
        self.lpos = None
        self.state = {}
        self.depth = None        # latest near distance (m) or None
        self.t_depth = None      # time of the latest depth frame
        self.close_count = 0
        self.phase = 'WAIT'      # WAIT -> CLIMB -> HOVER -> [TURN -> LEG] x4 -> LAND -> DONE
        self.start = None        # VIO position when Offboard started
        self.yaw0 = None         # VIO heading when Offboard started
        self.range0 = 0.0        # rangefinder reading at that moment
        self.t_offboard = None
        self.t_hover = None
        self.t_land = None
        self.last_log = 0.0
        self.create_timer(0.1, self.tick)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.vio = (p.x, p.y, p.z)
        self.yaw = yaw_of(m.pose.pose.orientation)

    def on_state(self, m):
        try:
            self.state = json.loads(m.data)
        except ValueError:
            pass

    def on_depth(self, m):
        self.t_depth = time.monotonic()
        self.depth = near_distance(m)
        if self.depth is not None and self.depth < DEPTH_LAND_M:
            self.close_count += 1
        else:
            self.close_count = 0

    def say(self, msg, every=0.0):
        if time.monotonic() - self.last_log >= every:
            self.get_logger().info(msg)
            self.last_log = time.monotonic()

    def send(self, xyz, yaw=None):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
        if yaw is not None:      # all-zero orientation = do not command yaw
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.pub.publish(m)

    def land(self, why):
        """Land through flight_interface: a straight-down twist (twists are not floored at 0.3 m,
        poses are). Sent every tick until PX4 disarms: if it stopped, the twist would expire
        after 1 s and flight_interface would go back to the last pose, 0.5 m up."""
        if self.phase not in ('LAND', 'DONE'):
            self.get_logger().warn(f'LAND: {why}')
            self.phase, self.t_land = 'LAND', time.monotonic()
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.twist.linear.z = -LAND_MPS
        self.twist.publish(m)

    def range_m(self):
        lp = self.lpos
        return float(lp.dist_bottom) if lp is not None and lp.dist_bottom_valid else None

    def depth_fresh(self):
        return self.t_depth is not None and time.monotonic() - self.t_depth < DEPTH_STALE_S

    def build_square(self):
        """Corners relative to the takeoff pose: leg 1 along the nose, then 90 deg turns."""
        turn = math.pi / 2 if TURN_LEFT else -math.pi / 2
        x, y = self.start[0], self.start[1]
        out = []
        for i in range(4):
            h = wrap(self.yaw0 + i * turn)
            x, y = x + self.side * math.cos(h), y + self.side * math.sin(h)
            out.append((x, y, h))
        return out

    def tick(self):
        if self.vio is None or self.lpos is None:
            self.say('waiting for VIO and PX4 local position ...', every=2.0)
            return
        offboard = bool(self.state.get('offboard_active', False))
        vio_h = self.vio[2]
        px4_h = -float(self.lpos.z)

        if self.phase == 'DONE' and self.state.get('armed') is False:
            # One go per arming. Disarmed again: ready for the next one.
            self.get_logger().info('disarmed: mission reset, ready for the next arming')
            self.phase, self.start, self.corners, self.corner, self.t_land = 'WAIT', None, [], 0, None
            return

        if self.phase == 'WAIT':
            if offboard:
                self.phase, self.start, self.t_offboard = 'CLIMB', self.vio, time.monotonic()
                self.yaw0 = self.yaw
                self.range0 = self.range_m() or 0.0
                if self.side > 0:
                    self.corners = self.build_square()
                self.get_logger().info(f'Offboard: climbing to {ALT:.1f} m above {tuple(round(v, 2) for v in self.start)}, '
                                       f'heading {math.degrees(self.yaw0):.0f} deg')
            else:
                # Gate: only offer a setpoint (= allow the takeover) while VIO height is sane.
                if abs(vio_h - px4_h) > GATE_M:
                    self.say(f'NOT READY: VIO height {vio_h:+.2f} m vs PX4 {px4_h:+.2f} m (> {GATE_M} m apart). '
                             'No takeover. Restart the stack with the drone still.', every=2.0)
                    return
                if self.side > 0 and not self.depth_fresh():
                    self.say(f'NOT READY: no depth on {DEPTH_TOPIC} (depth camera profile on?). No takeover.',
                             every=2.0)
                    return
                self.send(self.vio)
                self.say(f'ready: VIO/PX4 height {vio_h:+.2f}/{px4_h:+.2f} m. Pilot: arm on Altitude, then Position.',
                         every=5.0)
                return

        z = self.start[2] + ALT if self.start is not None else None   # after WAIT: start may be new
        flying = ('CLIMB', 'HOVER', 'TURN', 'LEG')
        if self.phase in flying and not offboard:
            self.get_logger().warn('Offboard ended: the pilot has it. Not commanding anything more.')
            self.phase = 'DONE'
            return

        if self.phase in flying:
            rng = self.range_m()
            climb_vio = vio_h - self.start[2]
            if time.monotonic() - self.t_offboard > self.max_flight_s:
                self.land(f'{self.max_flight_s:.0f} s in Offboard')
            elif rng is not None and rng > RANGE_MAX:
                self.land(f'rangefinder {rng:.2f} m > {RANGE_MAX:.2f} m')
            elif rng is not None and rng > 0.3 and abs(climb_vio - (rng - self.range0)) > DISAGREE_M:
                self.land(f'VIO climb {climb_vio:.2f} m vs rangefinder climb {rng - self.range0:.2f} m')
            elif self.phase != 'CLIMB' and self.close_count >= DEPTH_FRAMES:
                self.land(f'obstacle {self.depth:.2f} m ahead (< {DEPTH_LAND_M:.2f} m)')
            elif self.side > 0 and self.phase != 'CLIMB' and not self.depth_fresh():
                self.land(f'no depth for {DEPTH_STALE_S} s')

        if self.phase == 'CLIMB':
            target = (self.start[0], self.start[1], z)
            self.send(target, self.yaw0)
            if abs(vio_h - z) < ARRIVED_M:
                if self.corners:     # square: no hover, straight into leg 1
                    self.phase, self.t_corner = 'TURN', time.monotonic()
                    self.get_logger().info('at height: square, leg 1 (straight ahead), no hover')
                else:
                    self.phase, self.t_hover = 'HOVER', time.monotonic()
                    self.get_logger().info(f'at height: hovering {HOVER_S:.0f} s')
        elif self.phase == 'HOVER':
            self.send((self.start[0], self.start[1], z), self.yaw0)
            if time.monotonic() - self.t_hover >= HOVER_S:
                if self.corners:
                    self.phase, self.t_corner = 'TURN', time.monotonic()
                    self.get_logger().info('hover done: square, leg 1 (straight ahead)')
                else:
                    self.land('hover done')
        elif self.phase == 'TURN':
            # Turn in place at the current corner until the nose points along the next leg.
            here = self.corners[self.corner - 1][:2] if self.corner > 0 else self.start[:2]
            heading = self.corners[self.corner][2]
            self.send((here[0], here[1], z), heading)
            if abs(wrap(self.yaw - heading)) < math.radians(YAW_OK_DEG):
                self.phase, self.t_corner = 'LEG', time.monotonic()
                self.get_logger().info(f'facing leg {self.corner + 1}: flying forward {self.side:.2f} m')
            elif time.monotonic() - self.t_corner > TURN_TIMEOUT_S:
                self.land(f'turn {self.corner + 1} not done in {TURN_TIMEOUT_S:.0f} s')
        elif self.phase == 'LEG':
            x, y, heading = self.corners[self.corner]
            self.send((x, y, z), heading)
            if math.dist(self.vio[:2], (x, y)) < CORNER_M:
                self.corner += 1
                if self.corner == len(self.corners):
                    self.land('square done')
                else:
                    self.phase, self.t_corner = 'TURN', time.monotonic()
                    self.get_logger().info(f'corner {self.corner} reached: turning for leg {self.corner + 1}')
            elif time.monotonic() - self.t_corner > CORNER_TIMEOUT_S:
                self.land(f'corner {self.corner + 1} not reached in {CORNER_TIMEOUT_S:.0f} s')
        elif self.phase == 'LAND':
            if not offboard:
                self.get_logger().warn('Offboard ended while landing: the pilot has it.')
                self.phase = 'DONE'
            else:
                self.land('')    # keep pushing down until PX4 disarms (COM_DISARM_LAND)
                if self.state.get('landed') and not self.state.get('armed'):
                    self.get_logger().info('landed and disarmed. Done.')
                    self.phase = 'DONE'
                elif self.state.get('landed'):
                    self.say('on the ground, waiting for PX4 to disarm (pilot: disarm if it does not)', every=2.0)

        rng = self.range_m()
        d = '--' if self.depth is None or not self.depth_fresh() else f'{self.depth:.2f}'
        self.say(f'{self.phase}: VIO h {vio_h:+.2f} m, PX4 h {px4_h:+.2f} m, '
                 f'range {"--" if rng is None else f"{rng:.2f}"} m, depth ahead {d} m', every=1.0)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--square', type=float, default=0.0, help='side in m (0 = hover only, max 1.0)')
    a, _ = ap.parse_known_args()
    if not 0.0 <= a.square <= 1.0:
        raise SystemExit('--square must be 0..1.0 m')
    rclpy.init()
    n = Hover(a.square)
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
