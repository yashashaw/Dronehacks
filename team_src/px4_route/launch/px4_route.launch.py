"""Boot launch for px4_route. In /etc/skybeta/drone.env:

    COMPETITOR_PKG=px4_route
    COMPETITOR_LAUNCH=px4_route.launch.py

The ONE flight_interface, configured exactly as in px4_square (the stack that flew in PX4 log 26),
plus the route follower. The route file is read at start:

    /root/team_ws/routes/route.yaml   (= ~/team_ws/routes/route.yaml on the drone), if it exists,
    else this package's routes/test_line.yaml.

Swap routes by replacing ~/team_ws/routes/route.yaml and restarting the competitor window; no rebuild.
Or: ros2 launch px4_route px4_route.launch.py route:=/root/team_ws/routes/other.yaml

flight_interface's ceiling (max_altitude_m, above PX4's floor): 2.0 m, or 2.6 m automatically when
the route file turns the return on (return_route set), because the return flies over the walls.
The course part is still limited by its own route max_height (the node lands 0.3 m above it).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TEAM_ROUTE = '/root/team_ws/routes/route.yaml'
CEILING_M = '2.0'
CEILING_WITH_RETURN_M = '2.6'


def default_ceiling(route_path):
    try:
        with open(route_path) as f:
            raw = yaml.safe_load(f) or {}
        return CEILING_WITH_RETURN_M if raw.get('return_route') else CEILING_M
    except Exception:  # noqa: BLE001 -- the node itself reports a bad route file
        return CEILING_M


def generate_launch_description():
    default_route = TEAM_ROUTE if os.path.exists(TEAM_ROUTE) else os.path.join(
        get_package_share_directory('px4_route'), 'routes', 'test_line.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('route', default_value=default_route, description='route YAML file'),
        DeclareLaunchArgument('max_altitude_m', default_value=default_ceiling(default_route),
                              description="flight_interface ceiling above PX4's floor (m); 2.6 with a return"),
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{
                 'auto_offboard_on_arm': True,          # without this: no takeover
                 'anchor_setpoints': False,             # targets are already in PX4's frame
                 'odom_to_ned_yaw_offset_rad': 0.0,
                 'odometry_topic': '/px4_route/odom',   # altitude band from PX4, not VIO
                 'max_altitude_m': LaunchConfiguration('max_altitude_m'),
             }]),
        Node(package='px4_route', executable='route_mission',
             name='px4_route', output='screen', emulate_tty=True,
             parameters=[{'route': LaunchConfiguration('route')}]),
    ])
