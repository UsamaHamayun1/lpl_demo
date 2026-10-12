"""TurtleBot3 Nav2 with its motor commands routed through the LPL guard.

    ros2 launch lpl_demo lpl_nav2.launch.py map:=/path/to/map.yaml

Nav2 on Jazzy: controller_server -> /cmd_vel_nav -> velocity_smoother
-> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel. The collision monitor's
output is redirected to /cmd_vel_auto, which lpl_manager_guard filters before
anything reaches /cmd_vel.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

# lpl_manager_guard subscribes here
AUTONOMY_CMD_TOPIC = '/cmd_vel_auto'


def generate_launch_description():
    tb3_nav_dir = get_package_share_directory('turtlebot3_navigation2')

    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'cmd_vel_out_topic': AUTONOMY_CMD_TOPIC},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the map yaml'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(tb3_nav_dir, 'param', 'waffle_pi.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_nav_dir, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map': LaunchConfiguration('map'),
                'params_file': params,
            }.items()
        ),
    ])
