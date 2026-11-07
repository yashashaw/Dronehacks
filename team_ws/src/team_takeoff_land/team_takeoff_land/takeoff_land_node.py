"""Takeoff / land node: sits between your planner and the flight interface.

    planner  --/team/planner/pose,twist-->  takeoff_land  --/skybeta/setpoint/*-->  flight_interface --> PX4

Your planner publishes to ``/team/planner/pose`` (or ``/twist``) instead of
``/skybeta/setpoint/*``. In passthrough this node forwards them unchanged.
Call a service and the planner is cut off:

    ros2 service call /team/takeoff std_srvs/srv/Trigger
    ros2 service call /team/land    std_srvs/srv/Trigger
    ros2 service call /team/release std_srvs/srv/Trigger   # hover -> planner

Takeoff climbs through the flight interface (Offboard pose setpoints).
Land sends PX4 ``VEHICLE_CMD_NAV_LAND``: PX4 descends and disarms on touchdown.
That is needed because the flight interface clamps every pose to its
``min_altitude_m`` (0.3 m), so setpoints alone can never land.

This node never arms and never requests Offboard. State is published as JSON
on ``/team/takeoff_land/state``. Logic lives in ``logic.py``.

Wiring: nothing else may publish to ``/skybeta/setpoint/*`` (stop
``competitor_example``), or it bypasses the cut-off; takeoff refuses then.

Rules: in a scored run ``/team/takeoff`` and ``/team/land`` must be called
by your own mission code on the drone. A human calling ``/team/land`` counts
as an abort (allowed); a human calling takeoff after the start is not.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleCommand, VehicleLandDetected, VehicleStatus

from skybeta_flight_interface.topics import resolve_endpoints

from team_takeoff_land.logic import (Config, Inputs, PASSTHROUGH,
                                     TakeoffLand)

STATUS_TIMEOUT_S = 1.0
NAN = float('nan')


class TakeoffLandNode(Node):

    def __init__(self, **kwargs):
        super().__init__('takeoff_land', **kwargs)
        p = self.declare_parameter
        p('takeoff_altitude_m', 1.0)
        p('arrival_radius_m', 0.15)
        p('settle_s', 1.0)
        p('takeoff_timeout_s', 20.0)
        p('release_after_takeoff', False)
        p('command_retry_s', 1.0)
        p('land_retries', 5)
        p('disarm_after_landed_s', 3.0)
        p('rate_hz', 10.0)
        p('planner_timeout_s', 0.5)
        # -1: use vehicle_status.system_id (the FC's MAV_SYS_ID). PX4 ignores
        # commands addressed to another system id.
        p('target_system', -1)
        p('planner_pose_topic', '/team/planner/pose')
        p('planner_twist_topic', '/team/planner/twist')
        p('pose_setpoint_topic', '/skybeta/setpoint/pose')
        p('twist_setpoint_topic', '/skybeta/setpoint/twist')
        p('odometry_topic', '/visual_slam/tracking/odometry')
        p('state_topic', '/team/takeoff_land/state')

        def v(name):
            return self.get_parameter(name).value

        self.logic = TakeoffLand(Config(
            takeoff_altitude_m=float(v('takeoff_altitude_m')),
            arrival_radius_m=float(v('arrival_radius_m')),
            settle_s=float(v('settle_s')),
            takeoff_timeout_s=float(v('takeoff_timeout_s')),
            release_after_takeoff=bool(v('release_after_takeoff')),
            command_retry_s=float(v('command_retry_s')),
            land_retries=int(v('land_retries')),
            disarm_after_landed_s=float(v('disarm_after_landed_s')),
        ))
        self.planner_timeout_s = float(v('planner_timeout_s'))
        self.target_system_param = int(v('target_system'))

        self.position = None
        self.status = None
        self.status_time = None
        self.landed = None
        self.last_planner_time = None
        self.planner_dropped = 0
        self.commands_sent = {'land': 0, 'disarm': 0}

        px4_pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        px4_sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        topics = resolve_endpoints()

        self.command_pub = self.create_publisher(
            VehicleCommand, topics['vehicle_command'], px4_pub_qos)
        self.create_subscription(VehicleStatus, topics['vehicle_status'],
                                 self.status_cb, px4_sub_qos)
        self.create_subscription(
            VehicleLandDetected, topics['vehicle_land_detected'],
            self.land_cb, px4_sub_qos)

        self.pose_pub = self.create_publisher(
            PoseStamped, v('pose_setpoint_topic'), 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, v('twist_setpoint_topic'), 10)
        self.state_pub = self.create_publisher(String, v('state_topic'), 10)
        self.create_subscription(PoseStamped, v('planner_pose_topic'),
                                 self.planner_pose_cb, 10)
        self.create_subscription(TwistStamped, v('planner_twist_topic'),
                                 self.planner_twist_cb, 10)
        self.create_subscription(Odometry, v('odometry_topic'),
                                 self.odom_cb, 10)

        self.create_service(Trigger, '/team/takeoff', self.takeoff_cb)
        self.create_service(Trigger, '/team/land', self.land_srv_cb)
        self.create_service(Trigger, '/team/release', self.release_cb)

        self.create_timer(1.0 / float(v('rate_hz')), self.tick)
        self.create_timer(0.2, self.publish_state)
        self.get_logger().info(
            f'takeoff_land up: planner {v("planner_pose_topic")} -> '
            f'{v("pose_setpoint_topic")}; commands on '
            f'{topics["vehicle_command"]}. Never arms.')

    # -- helpers -------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def link(self):
        return (self.status_time is not None
                and self.now_s() - self.status_time <= STATUS_TIMEOUT_S)

    def inputs(self):
        st = self.status
        nav = st.nav_state if st is not None else None
        return Inputs(
            now=self.now_s(),
            position=self.position,
            link=self.link(),
            armed=bool(st is not None and st.arming_state
                       == VehicleStatus.ARMING_STATE_ARMED),
            landed=self.landed,
            offboard=nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            land_engaged=nav in (VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                                 VehicleStatus.NAVIGATION_STATE_DESCEND),
        )

    def target_ids(self):
        if self.target_system_param >= 0:
            return self.target_system_param, 1
        st = self.status
        sysid = int(getattr(st, 'system_id', 0) or 0) if st else 0
        compid = int(getattr(st, 'component_id', 0) or 0) if st else 0
        return (sysid or 1), (compid or 1)

    def send_command(self, command, param1=0.0, param2=0.0, param4=0.0,
                     param5=0.0, param6=0.0, param7=0.0):
        sysid, compid = self.target_ids()
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = sysid
        msg.target_component = compid
        msg.source_system = sysid
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_pose(self, xyz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2]))
        msg.pose.orientation.w = 0.0  # all-zero quaternion: do not command yaw
        self.pose_pub.publish(msg)

    # -- callbacks -----------------------------------------------------
    def status_cb(self, msg):
        self.status = msg
        self.status_time = self.now_s()

    def land_cb(self, msg):
        self.landed = bool(msg.landed)

    def odom_cb(self, msg):
        pt = msg.pose.pose.position
        xyz = (float(pt.x), float(pt.y), float(pt.z))
        if all(math.isfinite(c) for c in xyz):
            self.position = xyz

    def planner_pose_cb(self, msg):
        self.last_planner_time = self.now_s()
        if self.logic.mode == PASSTHROUGH:
            self.pose_pub.publish(msg)
        else:
            self.planner_dropped += 1

    def planner_twist_cb(self, msg):
        self.last_planner_time = self.now_s()
        if self.logic.mode == PASSTHROUGH:
            self.twist_pub.publish(msg)
        else:
            self.planner_dropped += 1

    def _service(self, request_fn, response):
        ok, text = request_fn(self.inputs())
        response.success, response.message = ok, text
        log = self.get_logger().info if ok else self.get_logger().warn
        log(text)
        return response

    def bypassing_publishers(self):
        """Other publishers on the setpoint topics. Each one bypasses the
        cut-off (e.g. competitor_example still running), so takeoff refuses."""
        extra = {}
        for pub in (self.pose_pub, self.twist_pub):
            others = self.count_publishers(pub.topic_name) - 1
            if others > 0:
                extra[pub.topic_name] = others
        return extra

    def takeoff_cb(self, _request, response):
        extra = self.bypassing_publishers()
        if extra:
            response.success = False
            response.message = (
                f'refused: other nodes publish setpoints directly {extra}; '
                'they would bypass the cut-off. Point the planner at '
                '/team/planner/* and stop competitor_example.')
            self.get_logger().warn(response.message)
            return response
        return self._service(self.logic.request_takeoff, response)

    def land_srv_cb(self, _request, response):
        extra = self.bypassing_publishers()
        if extra:
            # Land must always work: PX4 Land mode ignores Offboard setpoints,
            # so other publishers cannot fight it. Warn only.
            self.get_logger().warn(
                f'other setpoint publishers {extra}; landing anyway')
        response = self._service(self.logic.request_land, response)
        if response.success:
            self.tick()  # send the first land command now, not next tick
        return response

    def release_cb(self, _request, response):
        return self._service(self.logic.request_release, response)

    # -- the periodic step ---------------------------------------------
    def tick(self):
        s = self.inputs()
        out = self.logic.step(s)
        for level, text in out.logs:
            getattr(self.get_logger(), level)(text)
        if out.setpoint is not None:
            self.publish_pose(out.setpoint)
        elif out.forward_planner and not s.offboard and s.position is not None:
            # Before Offboard, keep a hold-here stream going if the planner is
            # silent: the flight interface needs a live setpoint stream to
            # enter Offboard (same as competitor_example's wait state).
            silent = (self.last_planner_time is None
                      or s.now - self.last_planner_time
                      > self.planner_timeout_s)
            if silent:
                self.publish_pose(s.position)
        if out.send_land:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                              param4=NAN, param5=NAN, param6=NAN, param7=NAN)
            self.commands_sent['land'] += 1
            self.get_logger().info(
                f'sent NAV_LAND to system {self.target_ids()[0]}')
        if out.send_disarm:
            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.commands_sent['disarm'] += 1
            self.get_logger().warn('landed but still armed: sent disarm')

    def publish_state(self):
        s = self.inputs()
        state = self.logic.status()
        state.update({
            'armed': s.armed, 'landed': s.landed, 'offboard': s.offboard,
            'land_engaged': s.land_engaged, 'px4_link': s.link,
            'target_system': self.target_ids()[0],
            'planner_dropped': self.planner_dropped,
            'bypassing_publishers': self.bypassing_publishers(),
            'commands_sent': self.commands_sent,
        })
        self.state_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
