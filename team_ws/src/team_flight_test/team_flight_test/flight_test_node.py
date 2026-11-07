"""Flight-test node: run_015's waypoint logic on the real stack, with heavy logging.

    flight_test --/team/planner/pose--> takeoff_land --/skybeta/setpoint/pose--> flight_interface --> PX4
    flight_test --calls /team/takeoff, /team/release, /team/land--> takeoff_land

It never arms and never requests Offboard. Start: the pilot ARMS, then flips to
POSITION; flight_interface enters Offboard; this node then calls takeoff, waits for
hover, calls release, flies the chosen profile, calls land.

Abort: kill switch, or take over with the mode switch / sticks. takeoff_land reports
'aborted' and this node stops publishing for good.

Logs (bind-mounted, survive the container): <log_dir>/<stamp>_<profile>/
    ticks.csv    one row per 20 Hz tick: setpoint, VIO pose, PX4 pose, modes, flags
    odom.csv     every VIO odometry message (for jump/rate analysis)
    events.log   every decision, service call and watchdog
    config.json  the parameters used
Analyse afterwards: python3 analyze_flight.py <that directory>
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleLocalPosition

from skybeta_flight_interface.topics import resolve_endpoints

from team_flight_test import route_logic as rl

TICK_HZ = 20.0
SERVICE_RETRY_S = 1.0
TAKEOFF_TIMEOUT_S = 30.0

TICK_COLS = ['t', 'mphase', 'phase', 'step', 'tl_mode', 'offboard', 'armed', 'nav_state',
             'sp_x', 'sp_y', 'sp_z', 'sp_yaw',
             'vio_x', 'vio_y', 'vio_z', 'vio_yaw', 'vio_age',
             'px4_x', 'px4_y', 'px4_z', 'px4_yaw', 'px4_xy_valid', 'px4_age',
             'track_err', 'jump_m']


def _f(v, nd=4):
    if v is None:
        return ''
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return '' if not math.isfinite(v) else ('%.*f' % (nd, v))
    return str(v)

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


class FlightTest(Node):

    def __init__(self):
        super().__init__('flight_test')
        d = rl.Config()
        p = self.declare_parameter
        for name in d.__dataclass_fields__:
            p(name, getattr(d, name))
        p('odometry_topic', '/visual_slam/tracking/odometry')
        p('log_dir', '/root/team_ws/flight_logs')
        v = lambda n: self.get_parameter(n).value  # noqa: E731

        kw = {}
        for name, fld in d.__dataclass_fields__.items():
            kw[name] = type(getattr(d, name))(v(name))
        self.cfg = rl.Config(**kw)
        self.route = rl.Route(self.cfg)       # raises on a bad profile/parameter

        # -- logs
        stamp = time.strftime('%Y%m%d-%H%M%S')
        self.dir = os.path.join(v('log_dir'), '%s_%s' % (stamp, self.cfg.profile))
        os.makedirs(self.dir, exist_ok=True)
        with open(os.path.join(self.dir, 'config.json'), 'w') as fh:
            json.dump(dict(kw, steps=self.route.steps, odometry_topic=v('odometry_topic')), fh, indent=1)
        self.f_tick = open(os.path.join(self.dir, 'ticks.csv'), 'w', buffering=1)
        self.f_tick.write(','.join(TICK_COLS) + '\n')
        self.f_odom = open(os.path.join(self.dir, 'odom.csv'), 'w', buffering=1)
        self.f_odom.write('t,stamp,x,y,z,yaw,step_m\n')
        self.f_ev = open(os.path.join(self.dir, 'events.log'), 'w', buffering=1)

        # -- state
        self.t0 = self.now()
        self.pose = None
        self.odom_t = None
        self.jump_since_tick = None
        self.px4 = None
        self.px4_t = None
        self.tl = None            # takeoff_land state
        self.fi = None            # flight_interface state
        self.mphase = rl.M_WAIT_OFFBOARD
        self.pending = None
        self.last_call = {}
        self.takeoff_at = None
        self.land_reason = None

        # -- ROS
        px4_sub = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        topics = resolve_endpoints()
        self.create_subscription(Odometry, v('odometry_topic'), self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, topics['vehicle_local_position'],
                                 self.px4_cb, px4_sub)
        self.create_subscription(String, '/team/takeoff_land/state', self.tl_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.fi_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/team/planner/pose', 10)
        self.clients_ = {n: self.create_client(Trigger, '/team/' + n) for n in ('takeoff', 'release', 'land')}
        self.create_timer(1.0 / TICK_HZ, self.tick)

        self.event('info', 'flight_test up: profile %s, %d steps %s' % (
            self.cfg.profile, len(self.route.steps), self.route.steps))
        self.event('info', 'PX4 local position on %s; logging to %s' % (topics['vehicle_local_position'], self.dir))
        self.event('info', 'TO FLY: pilot ARMS, then flips to POSITION. ABORT: kill switch / take over on RC.')

    # ---------------------------------------------------------------- utils
    def now(self):
        return time.monotonic()

    def event(self, level, text):
        log_at(self.get_logger(), level, text)
        self.f_ev.write('%8.3f %-5s [%s/%s] %s\n' % (self.now() - self.t0, level.upper(), self.mphase,
                                                    self.route.phase, text))

    # ------------------------------------------------------------ callbacks
    def odom_cb(self, m):
        q, pt = m.pose.pose.orientation, m.pose.pose.position
        vals = (pt.x, pt.y, pt.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(c) for c in vals):
            self.event('warn', 'non-finite odometry ignored')
            return
        pose = rl.Pose(pt.x, pt.y, pt.z, rl.yaw_from_quat(q.x, q.y, q.z, q.w))
        now = self.now()
        jump = self.route.observe(now, pose)
        step = 0.0 if self.pose is None else math.hypot(pose.x - self.pose.x, pose.y - self.pose.y)
        if jump is not None:
            self.jump_since_tick = max(self.jump_since_tick or 0.0, jump)
            self.event('warn', 'VIO jump %.3f m in one message (phase %s)' % (jump, self.route.phase))
        if self.odom_t is not None and now - self.odom_t > 0.2:
            self.event('warn', 'VIO gap %.0f ms' % (1000 * (now - self.odom_t)))
        self.pose, self.odom_t = pose, now
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.f_odom.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (
            now - self.t0, st, pose.x, pose.y, pose.z, pose.yaw, step))

    def px4_cb(self, m):
        self.px4, self.px4_t = m, self.now()

    def tl_cb(self, m):
        try:
            new = json.loads(m.data)
        except ValueError:
            return
        if self.tl is None or self.tl.get('mode') != new.get('mode'):
            self.event('info', 'takeoff_land mode -> %s (offboard=%s armed=%s)' % (
                new.get('mode'), new.get('offboard'), new.get('armed')))
        self.tl = new

    def fi_cb(self, m):
        try:
            self.fi = json.loads(m.data)
        except ValueError:
            pass

    # -------------------------------------------------------------- services
    def call(self, name):
        now = self.now()
        if self.pending or now - self.last_call.get(name, -1e9) < SERVICE_RETRY_S:
            return
        cl = self.clients_[name]
        if not cl.service_is_ready():
            self.event('warn', '/team/%s not available yet' % name)
            self.last_call[name] = now
            return
        self.pending = name
        self.last_call[name] = now
        self.event('info', 'calling /team/%s' % name)
        cl.call_async(Trigger.Request()).add_done_callback(lambda f, n=name: self.on_done(n, f))

    def on_done(self, name, fut):
        self.pending = None
        try:
            r = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.event('warn', '/team/%s failed: %s' % (name, exc))
            return
        self.event('info' if r.success else 'warn', '/team/%s -> %s: %s' % (name, r.success, r.message))
        if not r.success:
            return
        if name == 'takeoff':
            self.mphase, self.takeoff_at = rl.M_TAKEOFF, self.now()
        elif name == 'release':
            self.mphase = rl.M_RELEASE
        elif name == 'land':
            self.mphase = rl.M_LANDING

    def request_land(self, reason):
        if self.mphase in (rl.M_LANDING, rl.M_DONE, rl.M_ABORTED):
            return
        if self.land_reason is None:
            self.land_reason = reason or 'requested'
            self.event('warn', 'landing: ' + self.land_reason)
        self.call('land')

    # ------------------------------------------------------------------ tick
    def tick(self):
        now = self.now()
        tl = self.tl or {}
        mode = tl.get('mode')

        # pilot took over / someone else landed
        if mode == 'aborted' and self.mphase not in (rl.M_WAIT_OFFBOARD, rl.M_DONE, rl.M_ABORTED):
            self.mphase = rl.M_ABORTED
            out = rl.Out()
            self.route.pilot_took_over(out)
            for lv, tx in out.events:
                self.event(lv, tx)
        if self.mphase in (rl.M_RELEASE, rl.M_ROUTE) and mode in ('landing', 'landed'):
            self.event('warn', 'takeoff_land is landing (not requested by this node?)')
            self.mphase = rl.M_LANDING

        want = rl.next_call(self.mphase, self.tl)
        if want:
            self.call(want)
        if self.mphase == rl.M_TAKEOFF and now - self.takeoff_at > TAKEOFF_TIMEOUT_S:
            self.request_land('no hover %.0f s after takeoff' % TAKEOFF_TIMEOUT_S)

        if self.mphase == rl.M_RELEASE and mode == 'passthrough':
            fresh = self.odom_t is not None and now - self.odom_t < self.cfg.odom_timeout_s
            if not fresh:
                self.request_land('released but no fresh VIO odometry')
            else:
                tgt = tl.get('target_enu')
                hover_z = float(tgt[2]) if tgt else self.pose.z
                out = self.route.start(now, self.pose, hover_z, tl.get('ground_z'))
                self.mphase = rl.M_ROUTE
                self.handle_route_out(out)

        elif self.mphase == rl.M_ROUTE:
            if mode != 'passthrough':
                self.event('warn', 'takeoff_land left passthrough (%s): route stopped' % mode)
                self.mphase = rl.M_LANDING
            else:
                out = self.route.step(now, self.pose, self.odom_t, self.jump_since_tick)
                self.handle_route_out(out)
                if self.route.phase == rl.DONE:
                    self.request_land(self.route.done_reason)   # retried until accepted

        elif self.mphase == rl.M_LANDING:
            if mode == 'landed':
                self.mphase = rl.M_DONE
                self.event('info', 'landed: test complete (%s). Logs: %s' % (
                    self.land_reason or 'route complete', self.dir))

        self.log_row(now)
        self.jump_since_tick = None

    def handle_route_out(self, out):
        for lv, tx in out.events:
            self.event(lv, tx)
        if out.setpoint is not None and self.mphase == rl.M_ROUTE:
            self.publish(out.setpoint)
        if out.land:
            self.request_land(out.land)

    def publish(self, sp):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(sp[0]), float(sp[1]), float(sp[2]))
        qx, qy, qz, qw = rl.quat_from_yaw(sp[3])
        m.pose.orientation.x, m.pose.orientation.y = qx, qy
        m.pose.orientation.z, m.pose.orientation.w = qz, qw
        self.pose_pub.publish(m)

    def log_row(self, now):
        tl, fi = self.tl or {}, self.fi or {}
        sp, p, x = self.route.sp, self.pose, self.px4
        px = py = pz = pyaw = pvalid = None
        if x is not None:
            # PX4 NED -> ENU
            px, py, pz = float(x.y), float(x.x), -float(x.z)
            pyaw = rl.wrap(math.pi / 2 - float(x.heading)) if math.isfinite(x.heading) else None
            pvalid = bool(x.xy_valid)
        track = (math.hypot(p.x - sp[0], p.y - sp[1]) if (p and sp and self.mphase == rl.M_ROUTE) else None)
        step = self.route.steps[self.route.i] if 0 <= self.route.i < len(self.route.steps) else None
        row = [now - self.t0, self.mphase, self.route.phase, (self.route.i + 1) if step else '',
               tl.get('mode'), tl.get('offboard'), tl.get('armed'), fi.get('nav_state_name'),
               sp[0] if sp else None, sp[1] if sp else None, sp[2] if sp else None, sp[3] if sp else None,
               p.x if p else None, p.y if p else None, p.z if p else None, p.yaw if p else None,
               (now - self.odom_t) if self.odom_t else None,
               px, py, pz, pyaw, pvalid, (now - self.px4_t) if self.px4_t else None,
               track, self.jump_since_tick]
        self.f_tick.write(','.join(_f(c) for c in row) + '\n')

    def close(self):
        for fh in (self.f_tick, self.f_odom, self.f_ev):
            try:
                fh.close()
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = FlightTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.event('info', 'node stopping (mission phase %s)' % node.mphase)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
