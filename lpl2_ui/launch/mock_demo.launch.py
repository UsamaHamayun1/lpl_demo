"""Dashboard preview with MOCK LPL data (not the real LPL).

    ros2 launch lpl2_ui mock_demo.launch.py [scenario:=lpl1_continue] [gui:=true]

Starts the simulation, the mock LPL publisher, a mock path follower and the dashboard.
Use it to look at and test the dashboard before the real LPL nodes exist.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = os.path.join(get_package_share_directory('lpl2_sim'), 'launch', 'sim.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='lpl1_continue'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={
            'scenario': LaunchConfiguration('scenario'), 'gui': LaunchConfiguration('gui'),
            'record': LaunchConfiguration('record')}.items()),
        Node(package='lpl2_ui', executable='dashboard', output='screen'),
        TimerAction(period=8.0, actions=[
            Node(package='lpl2_ui', executable='mock_lpl', output='screen', parameters=[{'use_sim_time': True}]),
            Node(package='lpl2_ui', executable='mock_follower', output='screen', parameters=[{'use_sim_time': True}]),
        ]),
    ])
