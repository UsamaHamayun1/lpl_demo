"""Walking human actor on a square loop. Wrapper around sim.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim = os.path.join(get_package_share_directory('lpl_demo'), 'launch', 'sim.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim),
            launch_arguments={'world': 'cafe_complex'}.items(),
        ),
    ])
