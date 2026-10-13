"""LPL 2.0 corridor simulation: world, Robot R, Robot 2, scenario director, cameras.

    ros2 launch lpl2_sim sim.launch.py [scenario:=lpl1_continue] [gui:=true] [record:=false]

scenario: a file name in config/scenarios without .yaml (nominal, lpl1_continue,
lpl2_advisory, lpl3_handover), or 'none' for a static world.
record: write a split-screen mp4 (robot camera + top-down) to ~/lpl_recordings.
localization: truth (default) publishes map->odom so the believed pose equals the true
pose (simulated localization; stands in for AMCL). static uses a fixed map->odom at the
spawn pose, i.e. pure wheel odometry, which drifts.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lpl2_sim.geometry import Corridor

NVIDIA_GLX = '/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0'


def launch_setup(context):
    pkg = get_package_share_directory('lpl2_sim')
    geometry = os.path.join(pkg, 'config', 'corridor.yaml')
    corridor = Corridor(geometry)
    world = os.path.join(pkg, 'worlds', 'lpl2_corridor.sdf')
    x, y, yaw = corridor.robot_r['spawn']

    gui = LaunchConfiguration('gui').perform(context).lower() not in ('false', '0')
    scenario = LaunchConfiguration('scenario').perform(context)
    record = LaunchConfiguration('record').perform(context).lower() in ('true', '1')
    localization = LaunchConfiguration('localization').perform(context)
    common = {'use_sim_time': True, 'geometry_file': geometry}

    actions = []
    if gui and os.path.exists(NVIDIA_GLX):
        # Laptop PRIME is on-demand: without this the GUI renders on the Intel iGPU
        actions += [SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
                    SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')]
    # Headless server rendering through EGL picks the RTX on its own
    gz_args = f"-r -v2 {'' if gui else '-s --headless-rendering '}{world}"

    with open(os.path.join(pkg, 'urdf', 'robot_r.urdf')) as f:
        robot_description = f.read()

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', corridor.world_name, '-name', 'robot_r',
                        '-file', os.path.join(pkg, 'models', 'robot_r', 'model.sdf'),
                        '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),
        Node(package='ros_gz_image', executable='image_bridge', output='screen',
             arguments=['/camera/image_raw', '/topdown/image'],
             parameters=[{'use_sim_time': True}]),
        Node(package='lpl2_sim', executable='truth_localization', output='screen',
             parameters=[{'use_sim_time': True}]) if localization == 'truth' else
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                        '--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}]),
        Node(package='lpl2_sim', executable='world_state', output='screen',
             parameters=[{'use_sim_time': True, 'world': corridor.world_name}]),
        Node(package='lpl2_sim', executable='robot2_driver', output='screen', parameters=[common]),
    ]
    if scenario != 'none':
        actions.append(Node(
            package='lpl2_sim', executable='scenario_director', output='screen',
            parameters=[{**common, 'scenario_file':
                         os.path.join(pkg, 'config', 'scenarios', scenario + '.yaml')}]))
    if record:
        actions.append(Node(package='lpl2_sim', executable='recorder', output='screen',
                            parameters=[{**common, 'scenario': scenario}]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('lpl2_sim')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='none',
                              description='config/scenarios/<name>.yaml, or none'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI'),
        DeclareLaunchArgument('localization', default_value='truth',
                              description='truth (simulated localization) | static (odometry only)'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Record robot camera + top-down view to ~/lpl_recordings'),
        # model://turtlebot3_common meshes + our robot_r model
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        OpaqueFunction(function=launch_setup),
    ])
