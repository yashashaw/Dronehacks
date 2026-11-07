"""One launch for the whole chain: flight_interface + pad_detector + pad_guide (course_pipeline,
placeholder planner OFF) + team_planner. Point the stack's competitor window at this:

    COMPETITOR_PKG=team_planner
    COMPETITOR_LAUNCH=integrated.launch.py

Do NOT also launch team_takeoff_land: it is a second publisher of /skybeta/setpoint/*.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    course_launch = os.path.join(get_package_share_directory('course_pipeline'), 'launch', 'course.launch.py')
    share = get_package_share_directory('team_planner')
    config = os.path.join(share, 'config', 'cautious.yaml')
    hints = os.path.join(share, 'config', 'course_hints.json')
    args = [
        DeclareLaunchArgument('cruise_altitude_m', default_value='0.7',
                              description='guide climb height above the start pose; keep equal to flight_altitude_m'),
        DeclareLaunchArgument('step_m', default_value='1.0',
                              description='largest distance of a setpoint from the vehicle'),
        DeclareLaunchArgument('min_altitude_m', default_value='0.3',
                              description='flight interface floor. Landing needs it lowered: ask the organizers first'),
        DeclareLaunchArgument('weights_path', default_value='', description='YOLO weights for pad_detector'),
        DeclareLaunchArgument('planner_mode', default_value='general',
                              description='general | hybrid | scripted (hybrid and scripted use COURSE-SPECIFIC hints)'),
        DeclareLaunchArgument('hints_file', default_value=hints, description='COURSE-SPECIFIC waypoints JSON'),
        DeclareLaunchArgument('publish_pad_hint', default_value='false',
                              description='COURSE-SPECIFIC: announce the hinted pad near the pad (no detector yet)'),
    ]
    course = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(course_launch),
        launch_arguments={
            'planner': 'false',     # the real planner replaces the placeholder
            'cruise_altitude_m': LaunchConfiguration('cruise_altitude_m'),
            'step_m': LaunchConfiguration('step_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'weights_path': LaunchConfiguration('weights_path'),
        }.items())
    planner = Node(package='team_planner', executable='planner_node', name='team_planner',
                   output='screen', emulate_tty=True,
                   parameters=[config, {
                       'mode': ParameterValue(LaunchConfiguration('planner_mode'), value_type=str),
                       'hints_file': ParameterValue(LaunchConfiguration('hints_file'), value_type=str),
                       'publish_pad_hint': ParameterValue(LaunchConfiguration('publish_pad_hint'), value_type=bool),
                       'flight_altitude_m': ParameterValue(LaunchConfiguration('cruise_altitude_m'), value_type=float),
                   }])
    return LaunchDescription(args + [course, planner])
