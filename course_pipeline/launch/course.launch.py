"""Launch stub: only the flight interface.

The detector, guide and placeholder planner are added in later commits. Keep
the flight_interface node exactly as it is (it owns the Offboard heartbeat and
the safety envelope), as in skybeta's competitor_example launch file.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{'auto_offboard_on_arm': True}],
    )
    return LaunchDescription([flight_interface])
