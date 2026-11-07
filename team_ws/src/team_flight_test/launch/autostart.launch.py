"""Boot-time entry point: what the stack's `competitor` window runs with nobody logged in.

drone.env points the stack here once (scripts/autostart.sh enable does it):
    COMPETITOR_PKG=team_flight_test
    COMPETITOR_LAUNCH=autostart.launch.py

Everything else comes from ~/team_ws/autostart.conf on the Jetson (/root/team_ws in the
container), written by scripts/autostart.sh, so a profile change never needs sudo:

    enabled=true          false: start nothing; the drone stays in Position under the pilot
    profile=hover         hover|yaw|out_back|box|course
    alt=1.0  speed=0.25  scale=1.0  max_legs=0  yaw_rate=20  run_s=60
    bag=false             true: also record record_bag.sh to ~/team_ws/flight_logs/bags/

A missing or unreadable file counts as enabled=false, so a broken config never flies.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

CONF = '/root/team_ws/autostart.conf'
PROFILES = ('hover', 'yaw', 'yaw45', 'strafe', 'out_back', 'box', 'course')
# conf key -> flight_test.launch.py argument
ARG_MAP = {
    'profile': 'profile',
    'alt': 'takeoff_altitude_m',
    'speed': 'speed',
    'scale': 'scale',
    'max_legs': 'max_legs',
    'yaw_rate': 'yaw_rate_dps',
    'run_s': 'max_run_s',
}


def read_conf(path):
    conf = {}
    with open(path) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if '=' in line:
                k, v = line.split('=', 1)
                conf[k.strip()] = v.strip()
    return conf


def off(reason):
    return LaunchDescription([LogInfo(msg=f'[autostart] mission NOT started: {reason}. '
                                          'The drone stays in Position under the pilot. '
                                          'Enable with: ~/team_ws/src/team_flight_test/scripts/autostart.sh enable')])


def generate_launch_description():
    try:
        conf = read_conf(CONF)
    except OSError as e:
        return off(f'cannot read {CONF} ({e.strerror})')
    if conf.get('enabled', 'false').lower() != 'true':
        return off(f'enabled={conf.get("enabled", "<missing>")} in {CONF}')
    profile = conf.get('profile', 'hover')
    if profile not in PROFILES:
        return off(f'bad profile "{profile}" in {CONF}')
    for k in ('alt', 'speed', 'scale', 'yaw_rate', 'run_s'):
        if k in conf:
            try:
                float(conf[k])
            except ValueError:
                return off(f'{k}={conf[k]} is not a number')
    if 'max_legs' in conf and not conf['max_legs'].isdigit():
        return off(f'max_legs={conf["max_legs"]} is not a whole number')

    args = {ARG_MAP[k]: conf[k] for k in ARG_MAP if k in conf}
    args['profile'] = profile
    if os.environ.get('DRONE_ID'):
        args['fc_system_id'] = os.environ['DRONE_ID']

    actions = [
        LogInfo(msg='[autostart] mission armed for takeoff: ' +
                    ' '.join(f'{k}={v}' for k, v in args.items()) +
                    '. TO FLY: pilot ARMS, then flips to POSITION. '
                    'ABORT: RC kill switch, or kill_flight.sh (lands).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('team_flight_test'), 'launch', 'flight_test.launch.py')),
            launch_arguments=args.items()),
    ]
    if conf.get('bag', 'false').lower() == 'true':
        out = f'/root/team_ws/flight_logs/bags/{time.strftime("%Y%m%d-%H%M%S")}_{profile}_autostart'
        os.makedirs(os.path.dirname(out), exist_ok=True)
        actions.append(ExecuteProcess(cmd=['record_bag.sh', '-o', out], output='screen'))
    return LaunchDescription(actions)
