"""flight_interface + takeoff_land + flight_test. Started by scripts/start_flight_test.sh
in the stack's competitor window, in place of competitor_example.

    ros2 launch team_flight_test flight_test.launch.py profile:=hover fc_system_id:=7
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, type) passed to flight_test
TEST_ARGS = [
    ('profile', 'hover', str),
    ('scale', '1.0', float),
    ('max_legs', '0', int),
    ('speed', '0.25', float),
    ('yaw_rate_dps', '20.0', float),
    ('settle_s', '1.0', float),
    ('max_run_s', '60.0', float),
    ('geofence_m', '1.0', float),
    ('max_track_err_m', '0.75', float),
    ('jump_land_m', '0.5', float),
]


def arg(name, kind):
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description():
    return LaunchDescription([
        # PX4 ignores commands for another system id; run.sh passes DRONE_ID in.
        DeclareLaunchArgument('fc_system_id',
                              default_value=EnvironmentVariable('DRONE_ID', default_value='1')),
        DeclareLaunchArgument('takeoff_altitude_m', default_value='1.0'),
        DeclareLaunchArgument('max_altitude_m', default_value='2.0'),
        DeclareLaunchArgument('max_speed_mps', default_value='0.75'),
    ] + [DeclareLaunchArgument(n, default_value=d) for n, d, _ in TEST_ARGS] + [
        Node(
            package='skybeta_flight_interface', executable='flight_interface',
            name='flight_interface', output='screen', emulate_tty=True,
            parameters=[{
                'max_altitude_m': arg('max_altitude_m', float),
                'min_altitude_m': 0.3,
                'max_speed_mps': arg('max_speed_mps', float),
                'setpoint_rate_hz': 10.0,
                'allow_mode_request': False,
                'auto_offboard_on_arm': True,
                'odometry_topic': '/visual_slam/tracking/odometry',
                'target_system': arg('fc_system_id', int),
            }],
        ),
        Node(
            package='team_takeoff_land', executable='takeoff_land',
            name='takeoff_land', output='screen', emulate_tty=True,
            parameters=[{
                'takeoff_altitude_m': arg('takeoff_altitude_m', float),
                'release_after_takeoff': False,
                'target_system': arg('fc_system_id', int),
            }],
        ),
        Node(
            package='team_flight_test', executable='flight_test',
            name='flight_test', output='screen', emulate_tty=True,
            parameters=[{n: arg(n, t) for n, _, t in TEST_ARGS}],
        ),
    ])
