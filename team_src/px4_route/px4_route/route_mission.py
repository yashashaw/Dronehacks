"""Follow a waypoint route on PX4's own position: the px4_square stack (PX4 log 26), generalised.

Same pipeline as px4_square, which flew cleanly on drone 18:
  * every position used here is PX4's /fmu/out/vehicle_local_position (VIO reaches PX4 only
    inside EKF2, for horizontal position + yaw); height is PX4's (rangefinder, baro backup);
  * ONE flight_interface, anchor_setpoints FALSE, odometry_topic = this node's relay of PX4's
    position (so its altitude band is measured from PX4's floor, not VIO's);
  * takeover only on the pilot's arm-on-Altitude -> switch-to-Position, with PX4 position fresh
    and valid and PX4 height ~0 on the ground; this node never arms and never touches /fmu/in.

What is new:
  * the route comes from a YAML file (route.py has the format); it is in the START frame
    (forward / left / up from where the drone is when Offboard starts), turned into PX4's frame
    at takeoff with PX4's heading;
  * a carrot setpoint slides along the path at the planned speed and waits for the drone, so the
    speed is ours, not PX4's position-jump response;
  * aborts LAND straight down (twist, not floored at 0.3 m), sent every tick until PX4 disarms,
    even if PX4's position goes invalid: too far off the path, too high, no PX4 position,
    time limit;
  * every flight writes runs/route-<time>.csv (10 Hz: phase, carrot, PX4 position, cross-track)
    and route-<time>.log to ~/team_ws/runs, which survive reboots.
Never record camera images while this flies: that delayed VIO 170-340 ms on the 21:55 walk and
made PX4's position jump metres. Light recordings (VIO + PX4 only) are fine.
"""
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

from px4_route.route import Carrot, Path, RouteError, load_route, start_frame_to_enu

RELAY_TOPIC = '/px4_route/odom'
RELAY_Z_MAX_M = 0.25     # relay (= flight_interface's altitude origin) starts only within this of 0
RELAY_STEADY_S = 2.0     # ... and only once PX4's height has been steady for this long
RELAY_STEADY_M = 0.03    # (max - min over RELAY_STEADY_S). On drone 18 PX4 rests at a steady
                         # +0.13 m on the floor (fine), but drifts 0.5-1.4 m -> 0 for ~30 s after
                         # boot or after being carried (not fine): that drift is what this catches.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def ned_to_enu(n, e, d):
    """PX4 NED -> ENU. flight_interface (anchor off) sends ENU (x, y, z) to PX4 as NED (y, x, -z)."""
    return (float(e), float(n), float(-d))


class RouteMission(Node):

    def __init__(self):
        super().__init__('px4_route')
        self.declare_parameter('route', '')
        self.declare_parameter('log_dir', '/root/team_ws/runs')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lead_base_m', 0.3)       # carrot lead at zero speed
        self.declare_parameter('px4_xy_p', 0.95)         # PX4 MPC_XY_P: lag ~ speed / this
        self.declare_parameter('land_mps', 0.3)
        self.declare_parameter('lpos_stale_s', 0.5)
        self.declare_parameter('ground_z_max_m', 0.3)
        self.declare_parameter('cross_track_hold_s', 0.5)  # this long beyond max_cross_track -> land
        self.declare_parameter('arrive_m', 0.2)           # at the last waypoint within this -> end
        self.declare_parameter('end_hover_s', 1.0)
        g = lambda n: self.get_parameter(n).value
        self.log_dir = g('log_dir')
        self.lead_base, self.kp = float(g('lead_base_m')), float(g('px4_xy_p'))
        self.land_mps, self.lpos_stale = float(g('land_mps')), float(g('lpos_stale_s'))
        self.ground_z_max, self.xt_hold = float(g('ground_z_max_m')), float(g('cross_track_hold_s'))
        self.arrive_m, self.end_hover = float(g('arrive_m')), float(g('end_hover_s'))

        self.route = None
        try:
            self.route = load_route(g('route'))
        except RouteError as e:
            self.get_logger().error(f'ROUTE REFUSED, nothing will fly: {e}')
        if self.route:
            p = Path(self.route.points, self.route.speeds, self.route.cfg['corner_speed'],
                     self.route.cfg['corner_angle_deg'], self.route.cfg['corner_radius'])
            self.get_logger().info(
                f'route "{self.route.name}" from {g("route")}: {len(self.route.points) - 2} waypoints, '
                f'{p.length:.1f} m, planned {p.eta_s():.0f} s at {self.route.cfg["speed"]:.2f} m/s '
                f'(corners {self.route.cfg["corner_speed"]:.2f}), yaw {self.route.cfg["yaw"]}, '
                f'end {self.route.cfg["end"]}. Pilot: arm on Altitude, then switch to Position.')
            for w in self.route.warnings:
                self.get_logger().warn(w)

        self.pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, RELAY_TOPIC, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_lpos, PX4_QOS)

        self.lpos, self.lpos_t = None, 0.0
        self.relay_ok = False        # see on_lpos: no relay until PX4 reads ~0 height on the ground
        self.relay_hist = []         # (time, PX4 z) while waiting for it to be steady
        self.crash_count = 0
        self.offboard, self.armed = False, None
        self.phase = 'WAIT'          # WAIT -> FLY -> (END_HOVER) -> LAND -> DONE -> WAIT on disarm
        self.path = self.carrot = None
        self.start = None
        self.t_fly = self.t_last = self.t_end = None
        self.xt_since = None
        self.yaw_cmd = None
        self.max_xt = 0.0
        self.csv = self.events = None
        self.last_flush = 0.0
        self.last_warn = {}
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.tick)

    # ---------------- inputs ----------------
    def on_lpos(self, msg):
        try:
            self._on_lpos(msg)
        except Exception as e:  # noqa: BLE001 -- a bad message must not kill the node
            self.get_logger().error(f'on_lpos error {type(e).__name__}: {e}')

    def _on_lpos(self, msg):
        self.lpos, self.lpos_t = msg, time.monotonic()
        # flight_interface takes its altitude band origin (0.3 m floor, max_altitude ceiling) from
        # the FIRST relayed sample. Right after boot PX4's height reads ~0.5 m off for a while
        # (drone 18, 2026-11-08: origin captured at 0.53 m -> floor 0.83 m, ceiling 2.53 m). So
        # start relaying only once PX4 reads ~0 height on the ground, then relay continuously.
        if not self.relay_ok:
            if not (msg.xy_valid and msg.z_valid and abs(float(msg.z)) <= RELAY_Z_MAX_M):
                self.relay_hist = []
                return
            self.relay_hist.append((self.lpos_t, float(msg.z)))
            self.relay_hist = [(t, z) for t, z in self.relay_hist if self.lpos_t - t <= RELAY_STEADY_S]
            zs = [z for _, z in self.relay_hist]
            if (self.lpos_t - self.relay_hist[0][0] < RELAY_STEADY_S - 0.1
                    or max(zs) - min(zs) > RELAY_STEADY_M):
                return
            self.relay_ok = True
            self.get_logger().info(f'PX4 height {-float(msg.z):+.2f} m on the ground: relaying PX4 '
                                   'position to flight_interface (its altitude band starts here)')
        if msg.xy_valid and msg.z_valid:          # relay: flight_interface's altitude band origin
            x, y, z = ned_to_enu(msg.x, msg.y, msg.z)
            o = Odometry()
            o.header.stamp = self.get_clock().now().to_msg()
            o.header.frame_id = 'px4_local_enu'
            o.child_frame_id = 'base_link'
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
            o.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(o)

    def on_state(self, msg):
        try:
            self._on_state(msg)
        except Exception as e:  # noqa: BLE001 -- a bad message must not kill the node
            self.get_logger().error(f'on_state error {type(e).__name__}: {e}')

    def _on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(s.get('offboard_active', False))
        self.armed = s.get('armed')
        clamped = [r for r in (s.get('clamped') or []) if r != 'min_altitude']
        if clamped and self.phase == 'FLY':
            self.event(f'flight_interface is CLAMPING our setpoint: {clamped}', warn=True, every=2.0)

    def position_enu(self):
        lp = self.lpos
        if lp is None or time.monotonic() - self.lpos_t > self.lpos_stale:
            return None
        if not (lp.xy_valid and lp.z_valid):
            return None
        return ned_to_enu(lp.x, lp.y, lp.z)

    # ---------------- outputs ----------------
    def send(self, p, yaw_enu=None):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        if yaw_enu is not None:                   # all-zero orientation = do not command yaw
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw_enu / 2), math.cos(yaw_enu / 2)
        self.pub.publish(m)

    def send_land(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.twist.linear.z = -self.land_mps
        self.twist_pub.publish(m)

    def event(self, text, warn=False, every=0.0):
        now = time.monotonic()
        if every and now - self.last_warn.get(text[:40], -1e9) < every:
            return
        self.last_warn[text[:40]] = now
        # One severity per call site: rclpy (Humble) raises "Logger severity cannot be changed
        # between calls" if the same line logs at two levels. That killed this node mid-flight.
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        if self.events:
            self.events.write(f'{time.strftime("%H:%M:%S")} {"WARN " if warn else ""}{text}\n')
            self.events.flush()

    def open_logs(self):
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stem = os.path.join(self.log_dir, 'route-' + time.strftime('%Y%m%d-%H%M%S'))
            self.csv = open(stem + '.csv', 'w')
            self.csv.write('t,phase,s,carrot_x,carrot_y,carrot_z,x,y,z,cross_track,speed,dist_bottom\n')
            self.events = open(stem + '.log', 'w')
        except OSError as e:
            self.get_logger().warn(f'cannot write flight logs to {self.log_dir}: {e}')

    def close_logs(self):
        for f in (self.csv, self.events):
            if f:
                f.close()
        self.csv = self.events = None

    def land(self, why):
        if self.phase not in ('LAND', 'DONE'):
            self.event(f'LAND: {why}', warn=why != 'route complete')
            self.phase = 'LAND'
        self.send_land()

    # ---------------- 10 Hz state machine ----------------
    def tick(self):
        """Never let an exception kill the node: a dead node leaves flight_interface holding the
        last pose, i.e. the drone hovering in Offboard until the pilot takes over (drone 18,
        2026-11-08 01:13). Any error in the air -> land straight down, every tick."""
        try:
            self._tick()
        except Exception as e:  # noqa: BLE001 -- deliberately everything
            self.crash_count += 1
            if self.crash_count <= 3:
                self.get_logger().error(f'INTERNAL ERROR {type(e).__name__}: {e} -- landing if flying')
            try:
                if self.phase in ('FLY', 'END_HOVER', 'LAND'):
                    self.phase = 'LAND'
                    self.send_land()
            except Exception:  # noqa: BLE001
                pass

    def _tick(self):
        now = time.monotonic()
        pos = self.position_enu()

        if self.phase in ('LAND', 'DONE') and self.armed is False:
            if self.phase == 'LAND':
                self.event('landed and disarmed: done')
            self.close_logs()
            self.phase, self.path, self.carrot = 'WAIT', None, None
            return
        if self.phase == 'LAND':
            if not self.offboard:
                self.event('Offboard ended while landing: the pilot has it', warn=True)
                self.phase = 'DONE'
                return
            self.send_land()                      # needs no position: keeps going if PX4's is lost
            self.log_row(now, pos)
            return
        if self.phase == 'DONE':
            return

        if pos is None:
            if self.phase in ('FLY', 'END_HOVER'):
                self.land('no fresh, valid PX4 local position')
            else:
                self.event('no fresh, valid PX4 local position: publishing nothing, so no takeover',
                           warn=True, every=2.0)
            return

        if self.phase == 'WAIT':
            self.wait(pos)
            return
        if not self.offboard:
            self.event('Offboard ended: the pilot has it. Not commanding anything more.', warn=True)
            self.phase = 'DONE'
            return
        if self.phase == 'FLY':
            self.fly(now, pos)
        elif self.phase == 'END_HOVER':
            self.send(self.path.point_at(self.path.length), self.yaw_cmd)
            if self.route.cfg['end'] == 'land' and now - self.t_end >= self.end_hover:
                self.land('route complete')
        self.log_row(now, pos)

    def wait(self, pos):
        if self.route is None:
            return                                # refused route: never offer a setpoint
        if not self.offboard:
            if not self.relay_ok:
                self.event(f'waiting for PX4 height to be steady near 0 (now {pos[2]:+.2f} m) before '
                           "flight_interface's altitude band is set: publishing nothing, so no takeover",
                           warn=True, every=2.0)
                return
            if abs(pos[2]) > self.ground_z_max:
                self.event(f'PX4 height {pos[2]:.2f} m is not ~0 on the ground: publishing nothing, '
                           'so no takeover', warn=True, every=2.0)
                return
            self.send(pos)                        # hold where we are; keeps the takeover possible
            return
        heading = float(self.lpos.heading)
        if not math.isfinite(heading):
            self.land('PX4 heading not valid at takeoff')
            return
        cfg = self.route.cfg
        self.start = pos
        enu = [start_frame_to_enu(p, pos, heading) for p in self.route.points]
        self.path = Path(enu, self.route.speeds, cfg['corner_speed'], cfg['corner_angle_deg'],
                         cfg['corner_radius'])
        self.carrot = Carrot(self.path, self.lead_base, self.kp)
        self.yaw_cmd = None
        if cfg['yaw'] == 'path':                  # start on the takeoff heading (ENU yaw)
            self.yaw_cmd = math.pi / 2 - heading
        self.t_fly = self.t_last = time.monotonic()
        self.xt_since, self.max_xt = None, 0.0
        self.phase = 'FLY'
        self.open_logs()
        self.event(f'Offboard active: route "{self.route.name}", {self.path.length:.1f} m, start (PX4 ENU) '
                   f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), heading {math.degrees(heading):.0f} deg')

    def fly(self, now, pos):
        cfg = self.route.cfg
        dt, self.t_last = now - self.t_last, now
        if now - self.t_fly > cfg['max_time_s']:
            self.land(f'{cfg["max_time_s"]:.0f} s in the air')
            return
        if pos[2] - self.start[2] > cfg['max_height'] + 0.3:
            self.land(f'height {pos[2] - self.start[2]:.2f} m above max_height {cfg["max_height"]:.2f} m')
            return
        # cross-track: distance from the path near where the carrot is (2 m back, 0.5 m ahead)
        _, xt = self.path.nearest(pos, self.carrot.s - 2.0, self.carrot.s + 0.5)
        self.max_xt = max(self.max_xt, xt)
        if xt > cfg['max_cross_track']:
            self.xt_since = self.xt_since or now
            if now - self.xt_since > self.xt_hold:
                self.land(f'{xt:.2f} m off the path (> {cfg["max_cross_track"]:.2f} m)')
                return
        else:
            self.xt_since = None

        target = self.carrot.step(dt, pos)
        if cfg['yaw'] == 'path':
            d = self.path.direction_at(self.carrot.s)
            if math.hypot(d[0], d[1]) > 0.3:      # not on a mostly vertical leg
                want = math.atan2(d[1], d[0])
                err = math.atan2(math.sin(want - self.yaw_cmd), math.cos(want - self.yaw_cmd))
                lim = math.radians(cfg['yaw_rate_dps']) * dt
                self.yaw_cmd += max(-lim, min(lim, err))
        self.send(target, self.yaw_cmd)

        if self.carrot.at_end() and math.dist(pos, target) < self.arrive_m:
            self.event(f'last waypoint reached in {now - self.t_fly:.1f} s, max cross-track '
                       f'{self.max_xt:.2f} m')
            self.phase, self.t_end = 'END_HOVER', now
            if cfg['end'] == 'hold':
                self.event('end: hold. Hovering at the last waypoint; the pilot lands.')

    def log_row(self, now, pos):
        if not self.csv:
            return
        c = self.path.point_at(self.carrot.s) if self.carrot else (math.nan,) * 3
        p = pos or (math.nan,) * 3
        xt = self.path.nearest(pos, self.carrot.s - 2.0, self.carrot.s + 0.5)[1] if pos and self.carrot else math.nan
        v = self.path.speed_at(self.carrot.s) if self.carrot else math.nan
        db = float(self.lpos.dist_bottom) if self.lpos is not None else math.nan
        s = self.carrot.s if self.carrot else math.nan
        self.csv.write(f'{now - self.t_fly:.2f},{self.phase},{s:.3f},{c[0]:.3f},{c[1]:.3f},{c[2]:.3f},'
                       f'{p[0]:.3f},{p[1]:.3f},{p[2]:.3f},{xt:.3f},{v:.2f},{db:.3f}\n')
        if now - self.last_flush > 1.0:
            self.csv.flush()
            self.last_flush = now


def main(args=None):
    rclpy.init(args=args)
    node = RouteMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
