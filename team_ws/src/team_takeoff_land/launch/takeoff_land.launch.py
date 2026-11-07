"""Start the takeoff/land node. Add it next to your planner in your team launch.

    ros2 launch team_takeoff_land takeoff_land.launch.py takeoff_altitude_m:=1.2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = [
    ('takeoff_altitude_m', '1.0', float),
    ('release_after_takeoff', 'false', bool),
    ('target_system', '-1', int),
]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d) for n, d, _ in ARGS] + [
            Node(
                package='team_takeoff_land',
                executable='takeoff_land',
                name='takeoff_land',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    n: ParameterValue(LaunchConfiguration(n), value_type=t)
                    for n, _, t in ARGS
                }],
            ),
        ])
