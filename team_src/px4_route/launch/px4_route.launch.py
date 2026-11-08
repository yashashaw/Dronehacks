"""Boot launch for px4_route. In /etc/skybeta/drone.env:

    COMPETITOR_PKG=px4_route
    COMPETITOR_LAUNCH=px4_route.launch.py

The ONE flight_interface, configured exactly as in px4_square (the stack that flew in PX4 log 26),
plus the route follower. The route file is read at start:

    /root/team_ws/routes/route.yaml   (= ~/team_ws/routes/route.yaml on the drone), if it exists,
    else this package's routes/test_line.yaml.

Swap routes by replacing ~/team_ws/routes/route.yaml and restarting the competitor window; no rebuild.
Or: ros2 launch px4_route px4_route.launch.py route:=/root/team_ws/routes/other.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TEAM_ROUTE = '/root/team_ws/routes/route.yaml'


def generate_launch_description():
    default_route = TEAM_ROUTE if os.path.exists(TEAM_ROUTE) else os.path.join(
        get_package_share_directory('px4_route'), 'routes', 'test_line.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('route', default_value=default_route, description='route YAML file'),
        DeclareLaunchArgument('max_altitude_m', default_value='2.0',
                              description="flight_interface ceiling above PX4's floor (m)"),
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
