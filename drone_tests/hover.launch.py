"""The one flight_interface + hover.py. Run by `bg.sh hover` in the stack's competitor window.

    ros2 launch /root/team_ws/drone_tests/hover.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('square', default_value='0.0', description='square side m, 0 = hover only'),
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{
                 'auto_offboard_on_arm': True,   # without this: no takeover
                 'anchor_setpoints': True,       # only VIO's CHANGE in position reaches PX4 (drone 7 climb)
                 'max_altitude_m': 1.5,
                 'max_speed_mps': 0.5,
             }]),
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'hover.py'), '--square', LaunchConfiguration('square')],
                       output='screen', emulate_tty=True),
    ])
