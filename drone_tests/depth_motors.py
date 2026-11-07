#!/usr/bin/env python3
"""Bench test, PROPS OFF: spin the motors according to the depth at the centre of the OAK-D view.

Runs inside the skybeta-stack container (use `dt.sh motors`, which does the docker exec for you).

    depth_motors.py                                 dry run: prints distance -> throttle, touches nothing
    depth_motors.py --spin --props-off              arms and spins (all four motors, same throttle)
    depth_motors.py --spin --props-off --farther-faster --max-throttle 0.10

Mapping (default "closer = faster"): an object at --near metres or closer -> --max-throttle,
at --far metres or farther -> 0 (motor idle). Linear in between. Reversed with --farther-faster.
The distance is the median of the valid pixels in a small box at the image centre
(/oak/stereo/image_raw, 16UC1 millimetres; 0 = no reading, ignored).

How the motors are driven (and why this is BENCH ONLY):
  This PX4 build does not run motor tests sent from the Jetson (DO_MOTOR_TEST -> UNSUPPORTED,
  ACTUATOR_TEST -> ignored; drone 7, 2026-11-07). The only way to set motor speed from code is
  Offboard "direct actuator": OffboardControlMode(direct_actuator) + ActuatorMotors, published
  straight to /fmu/in/*. Flight code must never do that (it goes through /skybeta/setpoint/* and
  flight_interface), so this script refuses to run while a flight_interface node is up: two
  Offboard heartbeats would fight. Stop the competitor window first (see README).

Safety:
  * dry run unless BOTH --spin and --props-off are given
  * --max-throttle defaults to 0.15 and cannot exceed 0.30
  * no depth for 0.5 s -> motors stop; no depth for 2 s -> disarm and exit
  * Ctrl-C, any error, or leaving Offboard -> motors stop + disarm
  * PX4 itself disarms after COM_DISARM_PRFLT (~10 s) because the drone never takes off;
    the script reports that and exits. Run it again for another go.
  * the RC kill switch is always the real emergency stop.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import (ActuatorMotors, OffboardControlMode, VehicleCommand,
                          VehicleCommandAck, VehicleStatus)

DEPTH_TOPIC = '/oak/stereo/image_raw'
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
NUM_MOTORS = 4
HARD_MAX_THROTTLE = 0.30
STALE_STOP_S = 0.5
STALE_EXIT_S = 2.0
NAV_OFFBOARD = 14
ARMED = 2


def throttle_for(dist_m, near, far, max_thr, farther_faster):
    """Distance (m, or None) -> throttle in [0, max_thr]. None -> None (stop motors)."""
    if dist_m is None:
        return None
    t = (dist_m - near) / (far - near)
    t = min(1.0, max(0.0, t))
    if not farther_faster:
        t = 1.0 - t
    return t * max_thr


def centre_distance(msg, box_px):
    """Median depth (m) of the valid pixels in a box_px x box_px box at the image centre."""
    if msg.encoding not in ('16UC1', 'mono16'):
        raise RuntimeError(f'expected 16UC1 depth in mm, got {msg.encoding}')
    img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
    cy, cx, h = msg.height // 2, msg.width // 2, box_px // 2
    roi = img[cy - h:cy + h + 1, cx - h:cx + h + 1]
    valid = roi[roi > 0]
    if valid.size < max(5, roi.size // 10):   # fewer than 10 % valid pixels: no reading
        return None
    return float(np.median(valid)) / 1000.0


class DepthMotors(Node):
    def __init__(self, a):
        super().__init__('depth_motors_bench')
        self.a = a
        self.dist = None
        self.depth_t = None
        self.status = None
        self.acks = []
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, SENSOR_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 lambda m: setattr(self, 'status', m), PX4_QOS)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 lambda m: self.acks.append(m), PX4_QOS)
        if a.spin:
            self.ocm_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
            self.mot_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', PX4_QOS)
            self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    # ---------------------------------------------------------------- inputs
    def on_depth(self, msg):
        try:
            self.dist = centre_distance(msg, self.a.box)
        except Exception as e:  # bad encoding: stop, do not guess
            self.get_logger().error(str(e))
            self.dist = None
        self.depth_t = time.monotonic()

    def depth_age(self):
        return math.inf if self.depth_t is None else time.monotonic() - self.depth_t

    def armed(self):
        return self.status is not None and int(self.status.arming_state) == ARMED

    def offboard(self):
        return self.status is not None and int(self.status.nav_state) == NAV_OFFBOARD

    # ---------------------------------------------------------------- outputs
    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def heartbeat(self):
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.thrust_and_torque = False
        m.direct_actuator = True
        self.ocm_pub.publish(m)

    def motors(self, thr):
        """thr None -> NaN = motors stopped (disarmed output value)."""
        m = ActuatorMotors()
        m.timestamp = self.now_us()
        m.reversible_flags = 0
        ctl = [math.nan] * len(m.control)
        for i in range(NUM_MOTORS):
            ctl[i] = math.nan if thr is None else float(thr)
        m.control = ctl
        self.mot_pub.publish(m)

    def command(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = cmd
        m.param1, m.param2 = float(p1), float(p2)
        sysid = int(self.status.system_id) if self.status is not None else 1
        m.target_system, m.target_component = sysid, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def ack_for(self, cmd):
        for m in self.acks:
            if int(m.command) == cmd:
                return int(m.result)
        return None

    def disarm(self):
        for _ in range(5):
            self.motors(None)
            self.heartbeat()
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def bar(thr, max_thr, width=30):
    if thr is None:
        return '[' + ' ' * width + ']'
    n = int(round(width * thr / max_thr)) if max_thr > 0 else 0
    return '[' + '#' * n + ' ' * (width - n) + ']'


def wait_for(node, cond, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if node.a.spin:
            node.heartbeat()
            node.motors(None)
        rclpy.spin_once(node, timeout_sec=0.05)
        if cond():
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--spin', action='store_true', help='actually arm and spin (otherwise dry run)')
    ap.add_argument('--props-off', action='store_true', help='you confirm the propellers are removed')
    ap.add_argument('--near', type=float, default=0.4, help='m, distance for full (closer=faster) throttle')
    ap.add_argument('--far', type=float, default=2.0, help='m, distance for zero (closer=faster) throttle')
    ap.add_argument('--max-throttle', type=float, default=0.15, help=f'0..{HARD_MAX_THROTTLE}')
    ap.add_argument('--farther-faster', action='store_true', help='reverse the mapping')
    ap.add_argument('--box', type=int, default=21, help='centre box size in pixels')
    ap.add_argument('--seconds', type=float, default=60.0, help='stop after this long')
    a = ap.parse_args()

    if not (0.0 < a.max_throttle <= HARD_MAX_THROTTLE):
        sys.exit(f'--max-throttle must be in (0, {HARD_MAX_THROTTLE}]')
    if not (0.3 <= a.near < a.far):
        sys.exit('need 0.3 <= --near < --far (the OAK-D sees nothing closer than ~0.31 m)')
    if a.spin and not a.props_off:
        sys.exit('--spin needs --props-off: take the propellers off first. This is a bench test.')

    rclpy.init()
    n = DepthMotors(a)
    mode = 'SPIN' if a.spin else 'DRY RUN (nothing is sent to PX4)'
    print(f'[motors] {mode}: {"farther" if a.farther_faster else "closer"} = faster, '
          f'{a.near:.2f} m .. {a.far:.2f} m -> 0 .. {a.max_throttle:.2f} throttle')

    try:
        if not wait_for(n, lambda: n.depth_t is not None, 5.0):
            print(f'[motors] ERROR: no {DEPTH_TOPIC} in 5 s. Is the camera in depth mode? (dt.sh camera status)')
            return 1
        if a.spin:
            if not wait_for(n, lambda: n.status is not None, 3.0):
                print('[motors] ERROR: no PX4 vehicle_status -- is the Pixhawk linked?')
                return 1
            others = [nm for nm, _ in n.get_node_names_and_namespaces() if 'flight_interface' in nm]
            if others:
                print(f'[motors] REFUSING: {others} is running. It streams its own Offboard heartbeat.\n'
                      '         Stop the competitor window first (README: "Stop flight_interface").')
                return 1
            if n.armed():
                print('[motors] REFUSING: the drone is already armed. Disarm on the RC first.')
                return 1
            print('[motors] streaming Offboard heartbeat, switching to Offboard, arming ...')
            wait_for(n, lambda: False, 1.0)                      # >= 1 s of heartbeat before the switch
            n.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)   # custom mode, PX4 OFFBOARD
            if not wait_for(n, n.offboard, 3.0):
                print(f'[motors] ERROR: PX4 did not enter Offboard (ack '
                      f'{n.ack_for(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)}).')
                return 1
            n.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if not wait_for(n, n.armed, 3.0):
                print(f'[motors] ERROR: PX4 refused to arm (ack '
                      f'{n.ack_for(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)}). Pixhawk USB '
                      'plugged in? RC transmitter off? Check QGC / the PX4 console for the reason.')
                return 1
            print('[motors] ARMED. Ctrl-C to stop. RC kill switch = emergency stop.')

        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.02)
            age = n.depth_age()
            if age > STALE_EXIT_S:
                print(f'\n[motors] depth stopped ({age:.1f} s): stopping.')
                break
            dist = n.dist if age <= STALE_STOP_S else None
            thr = throttle_for(dist, a.near, a.far, a.max_throttle, a.farther_faster)
            if a.spin:
                if not n.armed():
                    print('\n[motors] PX4 disarmed (kill switch, RC, or COM_DISARM_PRFLT auto-disarm). Done.')
                    break
                if not n.offboard():
                    print('\n[motors] PX4 left Offboard (pilot took over): stopping.')
                    break
                n.heartbeat()
                n.motors(thr)
            now = time.monotonic()
            if now - last_print > 0.1:
                d = f'{dist:5.2f} m' if dist is not None else '  --   '
                t = f'{thr:4.2f}' if thr is not None else 'stop'
                print(f'\r[motors] centre {d}  throttle {t} {bar(thr, a.max_throttle)}', end='', flush=True)
                last_print = now
        print()
    except KeyboardInterrupt:
        print('\n[motors] Ctrl-C')
    finally:
        if a.spin:
            n.disarm()
            print('[motors] motors stopped, disarm sent. armed now:', n.armed())
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
