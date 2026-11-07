"""Launch the flight interface, the pad detector, the guide and the placeholder planner.

This is what the stack's ``competitor`` window runs when drone.env has

    COMPETITOR_PKG=course_pipeline
    COMPETITOR_LAUNCH=course.launch.py

Keep the ``flight_interface`` node exactly as it is (it owns the Offboard
heartbeat and the safety envelope), as in skybeta's competitor_example launch
file. ``pad_guide`` is the only node here that publishes setpoints.

    ros2 launch course_pipeline course.launch.py
    ros2 launch course_pipeline course.launch.py weights_path:=/root/team_ws/pad.pt
    ros2 launch course_pipeline course.launch.py planner:=false   # real planner running elsewhere
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# name -> (default, help)
ARGUMENTS = [
    # flight interface envelope (its own defaults, repeated so one command
    # line can tighten them)
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('allow_mode_request', 'false',
     'SIM ONLY: on a real vehicle Offboard follows arming, then Position'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'ENU pose source'),
    # this package
    ('weights_path', '', 'YOLO weights inside the container; empty = no model'),
    ('cruise_altitude_m', '1.5', 'climb height above the start pose, metres'),
    ('step_m', '0.5', 'max distance of a published pose from the vehicle, metres'),
    ('planner', 'true', 'also start the PLACEHOLDER planner (/course/goal)'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    pad_detector = Node(
        package='course_pipeline',
        executable='pad_detector',
        name='pad_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'weights_path': ParameterValue(
                LaunchConfiguration('weights_path'), value_type=str),
        }],
    )

    pad_guide = Node(
        package='course_pipeline',
        executable='pad_guide',
        name='pad_guide',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'cruise_altitude_m': ParameterValue(
                LaunchConfiguration('cruise_altitude_m'), value_type=float),
            'step_m': ParameterValue(
                LaunchConfiguration('step_m'), value_type=float),
        }],
    )

    # PLACEHOLDER: delete this node (or pass planner:=false) when the real
    # pathfinder publishes /course/goal. See CONTRACT.md.
    path_placeholder = Node(
        package='course_pipeline',
        executable='path_placeholder',
        name='path_placeholder',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('planner')),
        parameters=[{
            'altitude_m': ParameterValue(
                LaunchConfiguration('cruise_altitude_m'), value_type=float),
        }],
    )

    return LaunchDescription(
        declared + [flight_interface, pad_detector, pad_guide, path_placeholder])
