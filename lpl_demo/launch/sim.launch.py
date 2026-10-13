"""Gazebo Harmonic + TurtleBot3 waffle_pi in any lpl_demo world.

    ros2 launch lpl_demo sim.launch.py world:=gauntlet2 [x_pose:=0.0 y_pose:=0.0] [gui:=false]
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = 'turtlebot3_waffle_pi'  # nodes look the robot up by this name

# Spawn pose per world when x_pose/y_pose are not given
DEFAULT_SPAWN = {
    'gauntlet2': (0.0, 0.0),
    'gauntlet1': (0.0, 0.0),
    'actor_puppet': (-3.0, 0.0),
    'cafe_complex': (-3.0, 0.0),
    'dynamic_tests': (0.0, 0.0),
    'three_boxes': (0.0, 0.0),
    'l_maze': (0.0, 0.0),
}


def launch_setup(context):
    pkg_lpl = get_package_share_directory('lpl_demo')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world').perform(context)
    world_file = os.path.join(pkg_lpl, 'worlds', world + '.world')
    world_name = ET.parse(world_file).getroot().find('world').get('name')

    x, y = DEFAULT_SPAWN.get(world, (0.0, 0.0))
    x = LaunchConfiguration('x_pose').perform(context) or str(x)
    y = LaunchConfiguration('y_pose').perform(context) or str(y)

    headless = LaunchConfiguration('gui').perform(context).lower() in ('false', '0')
    gz_args = f"-r -v2 {'-s ' if headless else ''}{world_file}"

    # turtlebot3_gazebo's robot_state_publisher launch reads this
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', ROBOT_NAME,
                       '-file', os.path.join(pkg_tb3, 'models', ROBOT_NAME, 'model.sdf'),
                       '-x', x, '-y', y, '-z', '0.01'],
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            parameters=[{'config_file': os.path.join(pkg_lpl, 'config', 'bridge.yaml'),
                         'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_image', executable='image_bridge', output='screen',
            arguments=['/camera/image_raw'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='lpl_demo', executable='gz_world_state', output='screen',
            parameters=[{'world': world_name}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='gauntlet2',
                              description='World file name in lpl_demo/worlds, without .world'),
        DeclareLaunchArgument('x_pose', default_value='', description='Robot spawn x (default per world)'),
        DeclareLaunchArgument('y_pose', default_value='', description='Robot spawn y (default per world)'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI'),
        # TurtleBot3 meshes are referenced as model://turtlebot3_common/...
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')),
        OpaqueFunction(function=launch_setup),
    ])
