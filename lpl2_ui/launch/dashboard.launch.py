"""LPL 2.0 dashboard on its own (connects to whatever LPL / simulation is running).

    ros2 launch lpl2_ui dashboard.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='lpl2_ui', executable='dashboard', output='screen')])
