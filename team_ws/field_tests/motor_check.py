#!/usr/bin/env python3
"""Ground check with PROPS OFF: watch PX4 while the pilot runs the motors on / off from the RC.

Runs inside the skybeta-stack container; `ft.sh watch` (or `ft.sh motors`) calls it.
It only LISTENS: it never commands, arms or spins anything.

    motor_check.py watch [--seconds 120]

Why there is no "spin each motor" command here: tested on drone 7 (2026-11-07), this PX4 build
does not take motor tests over the Jetson's DDS link -- VEHICLE_CMD_DO_MOTOR_TEST (209) is
answered UNSUPPORTED and VEHICLE_CMD_ACTUATOR_TEST (310) is ignored. Per-motor tests go through
MAVLink: QGroundControl -> Vehicle Setup -> Motors (USB cable laptop <-> Pixhawk, props OFF).
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, BatteryStatus

SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
NAV = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL',
       14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND'}

STEPS = """[watch] PROPS OFF, battery in, people clear. Pilot, in this order:
   1. ARM on the RC (mode Altitude)               -> motors spin at idle, all four
   2. wait 5 s, DISARM on the RC                  -> motors stop
   3. ARM again, flip the KILL switch              -> motors stop at once (PX4 disarms a few s later)
   4. kill switch back, ARM again; on the Jetson:  ft.sh kill
                                                  -> PX4 switches to LAND and disarms by itself
   (each motor alone: QGroundControl -> Vehicle Setup -> Motors)"""


class Watch(Node):
    def __init__(self):
        super().__init__('ground_check_watch')
        self.status = None
        self.batt = None
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', lambda m: setattr(self, 'status', m), SUB_QOS)
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', lambda m: setattr(self, 'batt', m), SUB_QOS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['watch'])
    ap.add_argument('--seconds', type=float, default=120.0)
    a = ap.parse_args()
    rclpy.init()
    n = Watch()
    t0 = time.time()
    while time.time() - t0 < 3 and n.status is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.status is None:
        print('[watch] ERROR: no PX4 status -- is the Pixhawk linked?')
        return
    v = float(n.batt.voltage_v) if n.batt is not None else 0.0
    print(STEPS)
    if v < 5.0:
        print(f'[watch] WARNING: the FC sees no battery ({v:.1f} V): the ESCs have no power, motors cannot spin.')
    print(f'[watch] watching for {a.seconds:.0f} s (Ctrl-C to stop early) ...', flush=True)
    last, arms, disarms, vmin, armed_at, armed_total = None, 0, 0, None, None, 0.0
    try:
        while time.time() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.1)
            armed = int(n.status.arming_state) == 2
            nav = int(n.status.nav_state)
            v = float(n.batt.voltage_v) if n.batt is not None else 0.0
            if v > 5:
                vmin = v if vmin is None else min(vmin, v)
            cur = (armed, nav)
            if cur != last:
                ts = time.time() - t0
                if last is not None and armed != last[0]:
                    if armed:
                        arms += 1
                        armed_at = ts
                    else:
                        disarms += 1
                        if armed_at is not None:
                            armed_total += ts - armed_at
                            print(f'[watch]           (was armed {ts - armed_at:.1f} s)')
                print(f'[watch] {ts:6.1f} s  {"ARMED   " if armed else "disarmed"}  mode {NAV.get(nav, nav):12}  '
                      f'battery {v:.2f} V', flush=True)
                last = cur
    except KeyboardInterrupt:
        pass
    print(f'[watch] done: {arms} arm(s), {disarms} disarm(s), armed {armed_total:.0f} s in total' +
          (f', lowest battery {vmin:.2f} V' if vmin else ', battery never seen'))
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
