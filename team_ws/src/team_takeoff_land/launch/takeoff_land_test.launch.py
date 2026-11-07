"""Takeoff-then-land test: flight interface + takeoff_land + test mission.

Started by scripts/start_test.sh in the stack's competitor window, in place
of competitor_example. No planner runs.

    ros2 launch team_takeoff_land takeoff_land_test.launch.py fc_system_id:=7
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def arg(name, kind):
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description():
    return LaunchDescription([
        # Default from DRONE_ID (run.sh passes it into the container; the fleet
        # flasher sets MAV_SYS_ID to it). PX4 ignores commands for another id,
        # so a default of 1 meant Offboard never engaged on drone 7.
        DeclareLaunchArgument('fc_system_id',
                              default_value=EnvironmentVariable('DRONE_ID', default_value='1'),
                              description="PX4 MAV_SYS_ID of this drone's FC (default: $DRONE_ID)"),
        DeclareLaunchArgument('takeoff_altitude_m', default_value='1.0'),
        DeclareLaunchArgument('hover_s', default_value='1.0',
                              description='seconds at altitude before landing'),
        DeclareLaunchArgument('max_flight_s', default_value='30.0'),

        # Same settings competitor.launch.py gives it, plus target_system
        # (competitor.launch.py does not expose that one).
        Node(
            package='skybeta_flight_interface',
            executable='flight_interface',
            name='flight_interface',
            output='screen', emulate_tty=True,
            parameters=[{
                'max_altitude_m': 3.0,
                'min_altitude_m': 0.3,
                'max_speed_mps': 1.5,
                'setpoint_rate_hz': 10.0,
                'allow_mode_request': False,
                'auto_offboard_on_arm': True,
                'odometry_topic': '/visual_slam/tracking/odometry',
                'target_system': arg('fc_system_id', int),
            }],
        ),
        Node(
            package='team_takeoff_land',
            executable='takeoff_land',
            name='takeoff_land',
            output='screen', emulate_tty=True,
            parameters=[{
                'takeoff_altitude_m': arg('takeoff_altitude_m', float),
                'release_after_takeoff': False,
            }],
        ),
        Node(
            package='team_takeoff_land',
            executable='takeoff_land_mission',
            name='takeoff_land_mission',
            output='screen', emulate_tty=True,
            parameters=[{
                'hover_s': arg('hover_s', float),
                'max_flight_s': arg('max_flight_s', float),
                'expected_fc_system_id': arg('fc_system_id', int),
            }],
        ),
    ])
