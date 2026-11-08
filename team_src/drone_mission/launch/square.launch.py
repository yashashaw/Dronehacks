"""Boot mission for this drone. Set in /etc/skybeta/drone.env:

    COMPETITOR_PKG=drone_mission
    COMPETITOR_LAUNCH=square.launch.py

The stack's competitor window then starts this at every boot: the ONE flight_interface
(auto_offboard_on_arm) + hover.py with the built-in 0.5 m square. Nothing flies until the
pilot arms on Altitude and flips to Position. The flight code itself stays in
/root/team_ws/drone_tests (hover.py, hover.launch.py), so an update is just install.sh.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HOVER_LAUNCH = '/root/team_ws/drone_tests/hover.launch.py'


def generate_launch_description():
    if not os.path.exists(HOVER_LAUNCH):
        # No flight code installed: start nothing, so Position stays with the pilot.
        return LaunchDescription([LogInfo(msg=f'[drone_mission] {HOVER_LAUNCH} missing: NO mission, pilot only')])
    return LaunchDescription([
        DeclareLaunchArgument('square', default_value='0.5', description='square side m (0 = hover only, max 1.0)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(HOVER_LAUNCH),
                                 launch_arguments={'square': LaunchConfiguration('square')}.items()),
    ])
