#!/usr/bin/env python3
"""Run route_mission.py's state machine against a fake PX4 + fake rclpy, on a laptop (no ROS).

    python team_src/px4_route/tools/sim_mission.py            # all scenarios, prints PASS/FAIL

The fakes copy what bit us on drone 18:
  * rclpy's logger: one severity per call site, else ValueError (killed the node mid-flight);
  * PX4 stays in Offboard after landing + auto-disarm (COM_DISARM_LAND 2 s);
  * PX4 auto-disarms 10 s after arming if it never takes off (COM_DISARM_PRFLT);
  * PX4's height reads off for a while after boot (relay gate).
The "drone" is a first-order follower of the pose setpoint (0.95/s) or of the landing twist.
It is a logic test, not a flight-dynamics test.
"""
import inspect
import json
import math
import os
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROUTES = os.path.join(PKG, 'routes')


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def install_fakes(state):
    def mod(name, **a):
        m = types.ModuleType(name)
        m.__dict__.update(a)
        sys.modules[name] = m

    class Vec:
        def __init__(self):
            self.x = self.y = self.z = self.w = 0.0

    class PS:
        def __init__(self):
            self.header = Obj()
            self.pose = Obj(position=Vec(), orientation=Vec())

    class TS:
        def __init__(self):
            self.header = Obj()
            self.twist = Obj(linear=Vec(), angular=Vec())

    class OD:
        def __init__(self):
            self.header = Obj()
            self.child_frame_id = ''
            self.pose = Obj(pose=Obj(position=Vec(), orientation=Vec()))

    class VehicleCommand:
        VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        VEHICLE_CMD_DO_SET_MODE = 176

        def __init__(self):
            self.command = 0
            self.param1 = self.param2 = 0.0

    class VehicleStatus:
        MESSAGE_VERSION = 1

    class Pub:
        def __init__(self, topic):
            self.topic = topic
            state['pubs'][topic] = self

        def publish(self, m):
            state['sent'].append((state['now'], self.topic, m))

    class StrictLogger:
        def _log(self, sev, m):
            f = inspect.stack()[2]
            key = (f.filename, f.lineno)
            if state['sites'].setdefault(key, sev) != sev:
                raise ValueError('Logger severity cannot be changed between calls.')
            state['log'].append((round(state['now'], 1), sev, m))

        def info(self, m):
            self._log('I', m)

        def warn(self, m):
            self._log('W', m)

        def error(self, m):
            self._log('E', m)

    class Node:
        def __init__(self, name):
            self.subs = {}

        def declare_parameter(self, n, d):
            state['params'].setdefault(n, d)

        def get_parameter(self, n):
            return Obj(value=state['params'][n])

        def create_publisher(self, t, topic, q):
            return Pub(topic)

        def create_subscription(self, t, topic, cb, q):
            self.subs[topic] = cb

        def create_timer(self, p, cb):
            self.timer = cb

        def get_logger(self):
            return StrictLogger()

        def get_clock(self):
            return Obj(now=lambda: Obj(to_msg=lambda: None, nanoseconds=int(state['now'] * 1e9)))

    mod('rclpy')
    mod('rclpy.node', Node=Node)
    mod('rclpy.executors', ExternalShutdownException=Exception)
    mod('rclpy.qos', QoSProfile=lambda **k: None, ReliabilityPolicy=Obj(BEST_EFFORT=1),
        HistoryPolicy=Obj(KEEP_LAST=1), DurabilityPolicy=Obj(VOLATILE=1))
    mod('geometry_msgs')
    mod('geometry_msgs.msg', PoseStamped=PS, TwistStamped=TS)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=OD)
    mod('std_msgs')
    mod('std_msgs.msg', String=Obj)
    mod('px4_msgs')
    mod('px4_msgs.msg', VehicleLocalPosition=Obj, VehicleCommand=VehicleCommand, VehicleStatus=VehicleStatus)
    import time as _t
    _t.monotonic = lambda: state['now']


def run(route_file, px4_accepts_arm=True, pilot=None, t_end=400.0, boot_z=0.13):
    """pilot: dict time -> callable(px4) for scripted pilot actions."""
    state = dict(now=0.0, pubs={}, sent=[], log=[], sites={}, params={})
    install_fakes(state)
    sys.path.insert(0, PKG)
    import importlib
    import px4_route.route_mission as rm
    importlib.reload(rm)
    state['params'].update(route=route_file, log_dir=tempfile.mkdtemp())
    n = rm.RouteMission()
    px4 = dict(pos=[5.0, 7.0, boot_z], armed=False, offboard=False, landed_since=None, armed_at=None,
               yaw=0.0)                                   # NED heading (rad), turns at 60 deg/s
    takeoffs = []
    seen = 0
    pilot = pilot or {25.0: lambda p: p.update(armed=True, armed_at=25.0),
                      26.0: lambda p: p.update(offboard=True)}
    while state['now'] < t_end:
        state['now'] = round(state['now'] + 0.1, 1)
        t = state['now']
        if t in pilot:
            pilot[t](px4)
        # PX4 reacts to commands sent since last tick
        for (_, topic, m) in state['sent'][seen:]:
            if topic == '/fmu/in/vehicle_command':
                if m.command == 400 and m.param1 == 1.0 and px4_accepts_arm:
                    if not px4['armed']:
                        px4.update(armed=True, armed_at=t, landed_since=None)
                elif m.command == 400 and m.param1 == 0.0:
                    px4['armed'] = False
                elif m.command == 176 and px4_accepts_arm:
                    px4['offboard'] = True
        seen = len(state['sent'])
        # dynamics: follow the latest pose or twist while armed in offboard
        if px4['armed'] and px4['offboard']:
            last = None
            for (ts, topic, m) in reversed(state['sent']):
                if topic in ('/skybeta/setpoint/pose', '/skybeta/setpoint/twist'):
                    last = (ts, topic, m)
                    break
            if last and last[1] == '/skybeta/setpoint/twist' and t - last[0] < 1.0:
                px4['pos'][2] = max(0.0, px4['pos'][2] + last[2].twist.linear.z * 0.1)
            elif last and last[1] == '/skybeta/setpoint/pose':
                sp = last[2].pose.position
                q = last[2].pose.orientation
                if q.z or q.w:                            # yaw commanded (ENU) -> NED heading
                    want = math.pi / 2 - 2 * math.atan2(q.z, q.w)
                    e = math.atan2(math.sin(want - px4['yaw']), math.cos(want - px4['yaw']))
                    px4['yaw'] += max(-math.radians(6), min(math.radians(6), e))
                tgt = (sp.x, sp.y, max(sp.z, 0.3))       # flight_interface floor (origin ~0)
                for i in range(3):
                    px4['pos'][i] += 0.95 * (tgt[i] - px4['pos'][i]) * 0.1
        on_ground = px4['pos'][2] <= 0.02
        if px4['armed'] and on_ground and px4['armed_at'] is not None and t - px4['armed_at'] > 1.0:
            px4['landed_since'] = px4['landed_since'] or t
            if t - px4['landed_since'] >= 2.0:            # COM_DISARM_LAND
                px4.update(armed=False, landed_since=None)
        if px4['armed'] and px4['armed_at'] is not None and t - px4['armed_at'] > 10.0 \
                and px4['pos'][2] < 0.05 and not any(tt > px4['armed_at'] for tt in takeoffs):
            px4['armed'] = False                           # COM_DISARM_PRFLT
        if px4['armed'] and px4['pos'][2] > 0.2 and (not takeoffs or takeoffs[-1] < (px4['armed_at'] or 0)):
            takeoffs.append(t)
        z_rep = px4['pos'][2] if t > 3 else 0.5            # height reads off for 3 s after boot
        n.subs['/fmu/out/vehicle_local_position'](Obj(
            x=px4['pos'][1], y=px4['pos'][0], z=-z_rep, xy_valid=True, z_valid=True,
            heading=px4['yaw'], dist_bottom=max(0.1, px4['pos'][2])))
        if '/fmu/out/vehicle_status_v1' in n.subs:
            n.subs['/fmu/out/vehicle_status_v1'](Obj(system_id=1))
        n.subs['/skybeta/state'](Obj(data=json.dumps(dict(offboard_active=px4['offboard'], armed=px4['armed']))))
        n.timer()
    cmds = [(round(ts, 1), m.command, m.param1) for (ts, topic, m) in state['sent'] if topic == '/fmu/in/vehicle_command']
    return dict(node=n, px4=px4, log=state['log'], cmds=cmds, takeoffs=takeoffs,
                has_cmd_pub='/fmu/in/vehicle_command' in state['pubs'])


def events(r, words=('Offboard active', 'RETURN', 'LAND:', 'landed', 'last waypoint', 'INTERNAL', 'REFUSED')):
    return [f'{t:6.1f} {m[:110]}' for t, s, m in r['log'] if any(w in m for w in words)]


def main():
    ok = True

    def check(name, cond, r):
        nonlocal ok
        ok &= bool(cond)
        print(('PASS ' if cond else 'FAIL ') + name)
        if not cond:
            print('\n'.join('      ' + e for e in events(r)))

    hop = os.path.join(ROUTES, 'test_hop.yaml')

    r = run(os.path.join(ROUTES, 'test_line.yaml'), t_end=120)
    check('return OFF: no PX4 command channel, no commands, one takeoff',
          not r['has_cmd_pub'] and not r['cmds'] and len(r['takeoffs']) == 1, r)

    r = run(hop, t_end=150)
    back = math.dist(r['px4']['pos'][:2], (5.0, 7.0))
    check(f'hop + return: 2 takeoffs, ends disarmed {back:.2f} m from the start, 1 arm cmd',
          len(r['takeoffs']) == 2 and not r['px4']['armed'] and back < 0.3
          and sum(1 for c in r['cmds'] if c[1] == 400 and c[2] == 1.0) == 1, r)

    r = run(hop, px4_accepts_arm=False, t_end=150)
    check('PX4 refuses to arm: gives up (<= 3 arm cmds), never takes off again',
          len(r['takeoffs']) == 1 and sum(1 for c in r['cmds'] if c[1] == 400 and c[2] == 1.0) <= 3
          and any('RETURN CANCELLED' in m for _, _, m in r['log']), r)

    pilot = {25.0: lambda p: p.update(armed=True, armed_at=25.0), 26.0: lambda p: p.update(offboard=True),
             31.0: lambda p: p.update(offboard=False)}          # pilot takes over mid-hop
    r = run(hop, pilot=pilot, t_end=150)
    check('pilot takes over during the course: no return, no commands', not r['cmds'], r)

    def switch_on_pad(p):
        if not p['armed']:
            p['offboard'] = False
    pilot = {25.0: lambda p: p.update(armed=True, armed_at=25.0), 26.0: lambda p: p.update(offboard=True)}
    landed_at = next(t for t, s, m in run(hop, t_end=60)['log'] if 'landed and disarmed' in m)
    for t in [round(landed_at + 1.0 + 0.1 * i, 1) for i in range(10)]:   # 1 s after touchdown
        pilot.setdefault(t, switch_on_pad)
    r = run(hop, pilot=pilot, t_end=150)
    cancelled = any('RETURN CANCELLED' in m for _, _, m in r['log'])
    check(f'mode switch moved on the pad (1 s after landing at {landed_at}): return cancelled, no re-arm',
          cancelled and not any(c[1] == 400 and c[2] == 1.0 for c in r['cmds']) and len(r['takeoffs']) == 1, r)

    # the real course with the return switched on (temporary copy, the repo file stays OFF)
    d = tempfile.mkdtemp()
    src = open(os.path.join(ROUTES, 'course_walk_DRAFT.yaml')).read()
    open(os.path.join(d, 'course.yaml'), 'w').write(src + '\nreturn_route: return_to_x.yaml\n')
    open(os.path.join(d, 'return_to_x.yaml'), 'w').write(open(os.path.join(ROUTES, 'return_to_x.yaml')).read())
    r = run(os.path.join(d, 'course.yaml'), t_end=300)
    back = math.dist(r['px4']['pos'][:2], (5.0, 7.0))
    check(f'course + return_to_x: 2 takeoffs, lands {back:.2f} m from the X (< 0.61 m), disarmed',
          len(r['takeoffs']) == 2 and back < 0.61 and not r['px4']['armed'], r)
    print('\n'.join('      ' + e for e in events(r)))
    print('ALL PASS' if ok else 'SOME FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
