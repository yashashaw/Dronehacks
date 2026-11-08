"""Boot launch for px4_square. In /etc/skybeta/drone.env:

    COMPETITOR_PKG=px4_square
    COMPETITOR_LAUNCH=px4_square.launch.py

Starts the ONE flight_interface (takeover on) and the square node (px4_square/square_mission.py,
the only mission file; it also relays PX4's position to /px4_square/odom).
No height in this pipeline comes from VIO:
  * the mission's start, targets and arrival checks are PX4's local position;
  * anchor_setpoints is FALSE explicitly, so targets reach PX4 with only the ENU->NED flip;
  * odometry_topic points flight_interface's altitude band at the relay (PX4's frame), not at
    /visual_slam/tracking/odometry.
VIO still reaches PX4 through the organisers' bridge, where EKF2 uses it for horizontal
position + yaw only (EKF2_EV_CTRL = 9; check on the drone).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', emulate_tty=True,
             parameters=[{
                 'auto_offboard_on_arm': True,          # without this: no takeover
                 'anchor_setpoints': False,             # targets are already in PX4's frame
                 'odom_to_ned_yaw_offset_rad': 0.0,
                 'odometry_topic': '/px4_square/odom',  # altitude band from PX4, not VIO
                 'max_altitude_m': 2.0,
             }]),
        Node(package='px4_square', executable='square_mission',
             name='px4_square', output='screen', emulate_tty=True,
             parameters=[{
                 'hover_altitude_m': 1.0,
                 'hover_seconds': 5.0,
                 'square_side_m': 0.5,
                 'arrival_radius_m': 0.3,
             }]),
    ])
