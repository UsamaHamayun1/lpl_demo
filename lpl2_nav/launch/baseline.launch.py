"""Plain autonomy in the LPL 2.0 corridor: simulation + Nav2 + mission, no LPL.

    ros2 launch lpl2_nav baseline.launch.py [scenario:=lpl1_continue] [gui:=true] [record:=false]

Nav2 keeps lane discipline (Lane 1 only inside the passing window) but never learns that the
evidence for borrowing Lane 1 has gone. With scenario:=nominal it is the control run; with a
disturbance (lpl1_continue / lpl3_handover) it is the "reckless" baseline that pulls into
Lane 1 blind.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, name, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments={k: v for k, v in args.items()}.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        include('lpl2_sim', 'sim.launch.py', scenario=LaunchConfiguration('scenario'),
                gui=LaunchConfiguration('gui'), record=LaunchConfiguration('record')),
        # let Gazebo, the robot and its TF come up before Nav2 activates
        TimerAction(period=8.0, actions=[include('lpl2_nav', 'nav.launch.py', mask='p3_available')]),
        TimerAction(period=12.0, actions=[Node(package='lpl2_nav', executable='mission', output='screen',
                                               parameters=[{'use_sim_time': True}])]),
    ])
