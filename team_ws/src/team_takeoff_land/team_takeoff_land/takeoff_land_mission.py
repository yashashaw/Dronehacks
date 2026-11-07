"""Test mission node: wait for Offboard, take off, land straight away.

Started by launch/takeoff_land_test.launch.py, which scripts/start_test.sh
runs in the stack's competitor window. Logic is in mission_logic.py.
It only calls /team/takeoff and /team/land; it never arms.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from team_takeoff_land.mission_logic import (LAND, TAKEOFF, Mission,
                                             MissionConfig)

def log_at(logger, level, text):
    """Log at a level chosen at run time. rclpy (Humble) ties one severity to each call
    site and raises 'Logger severity cannot be changed between calls' -- which kills the
    node -- if the same line logs at two levels. One line per severity avoids that."""
    if level == 'error':
        logger.error(text)
    elif level in ('warn', 'warning'):
        logger.warning(text)
    elif level == 'debug':
        logger.debug(text)
    else:
        logger.info(text)


class TakeoffLandMission(Node):

    def __init__(self, **kwargs):
        super().__init__('takeoff_land_mission', **kwargs)
        self.declare_parameter('hover_s', 1.0)
        self.declare_parameter('max_flight_s', 30.0)
        # The FC system id the flight interface sends its Offboard request
        # to. Compared with what PX4 reports; -1 skips the check.
        self.declare_parameter('expected_fc_system_id', -1)

        self.mission = Mission(MissionConfig(
            hover_s=float(self.get_parameter('hover_s').value),
            max_flight_s=float(self.get_parameter('max_flight_s').value),
        ))
        self.expected_sysid = int(
            self.get_parameter('expected_fc_system_id').value)
        self.sysid_checked = False
        self.state = None

        self.svc_clients = {
            TAKEOFF: self.create_client(Trigger, '/team/takeoff'),
            LAND: self.create_client(Trigger, '/team/land'),
        }
        self.create_subscription(String, '/team/takeoff_land/state',
                                 self.state_cb, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'takeoff_land_mission ready. To start: the pilot ARMS, then '
            'switches to POSITION. The drone takes off, then lands.')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state_cb(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            return
        sysid = self.state.get('target_system')
        if (not self.sysid_checked and self.expected_sysid >= 0
                and self.state.get('px4_link') and sysid is not None):
            self.sysid_checked = True
            if int(sysid) != self.expected_sysid:
                self.get_logger().error(
                    f'PX4 reports system id {sysid}, but the flight interface '
                    f'requests Offboard from system {self.expected_sysid}. '
                    f'Offboard will likely never engage. Restart with '
                    f'--sysid {sysid}.')
            else:
                self.get_logger().info(f'FC system id {sysid} matches.')

    def tick(self):
        which = self.mission.step(self.state, self.now_s())
        if which is not None:
            client = self.svc_clients[which]
            if not client.service_is_ready():
                self.mission.on_no_response(which)
            else:
                future = client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda f, w=which: self.on_done(w, f))
        for level, text in self.mission.messages:
            log_at(self.get_logger(), level, text)
        self.mission.messages.clear()

    def on_done(self, which, future):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - report and retry
            self.get_logger().warn(f'/team/{which} failed: {exc}')
            self.mission.on_no_response(which)
            return
        self.mission.on_response(which, result.success, result.message,
                                 self.now_s())


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
