"""Nav2 for Robot R in the LPL 2.0 corridor (the robot's ordinary autonomy).

    ros2 launch lpl2_nav nav.launch.py [mask:=p3_available] [cmd_vel_out:=/cmd_vel]

mask: which keepout mask to use: p3_available (Lane 1 only in the passing window),
      p3_unavailable (Lane 1 forbidden), none, a path to a mask yaml, or external
      (someone else publishes /keepout_mask: the LPL constraint compiler).
      Without LPL the mask never changes.
cmd_vel_out: where the final (collision-monitored) command goes.
feasibility: also start the physical-only planner in /feasibility (for LPL's decision
      space: "is this option possible?" via /feasibility/is_path_valid).

Only the servers the demo needs are started: nav2_bringup's navigation_launch.py also
starts docking_server, which publishes straight to /cmd_vel and would bypass any gate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def launch_setup(context):
    params = ParameterFile(
        os.path.join(get_package_share_directory('lpl2_nav'), 'config', 'nav2_params.yaml'),
        allow_substs=True)
    maps = os.path.join(get_package_share_directory('lpl2_sim'), 'maps')
    mask = LaunchConfiguration('mask').perform(context)
    if mask == 'external':
        mask_file = None
    elif mask == 'none':
        mask_file = os.path.join(maps, 'mask_none.yaml')
    elif mask.endswith('.yaml'):
        mask_file = mask
    else:
        mask_file = os.path.join(maps, f'mask_{mask}.yaml')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out').perform(context)
    feasibility = LaunchConfiguration('feasibility').perform(context).lower() in ('true', '1')
    feas_params = os.path.join(get_package_share_directory('lpl2_nav'), 'config', 'feasibility_params.yaml')

    def nav2(package, executable, name=None, extra=None, remaps=None):
        return Node(package=package, executable=executable, name=name or executable, output='screen',
                    parameters=[params] + ([extra] if extra else []), remappings=remaps or [])

    managed = ['map_server', 'keepout_mask_server', 'planner_server', 'controller_server',
               'velocity_smoother', 'collision_monitor', 'bt_navigator']
    if mask_file is None:
        managed.remove('keepout_mask_server')
    mask_server = [] if mask_file is None else [
        nav2('nav2_map_server', 'map_server', 'keepout_mask_server', {'yaml_filename': mask_file})]

    feasibility_nodes = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace='feasibility',
             output='screen', parameters=[feas_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_feasibility',
             namespace='feasibility', output='screen', parameters=[feas_params]),
    ] if feasibility else []

    return mask_server + feasibility_nodes + [
        nav2('nav2_map_server', 'map_server', extra={'yaml_filename': os.path.join(maps, 'corridor.yaml')}),
        nav2('nav2_planner', 'planner_server'),
        nav2('nav2_controller', 'controller_server', remaps=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_velocity_smoother', 'velocity_smoother', remaps=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_collision_monitor', 'collision_monitor', extra={'cmd_vel_out_topic': cmd_vel_out}),
        nav2('nav2_bt_navigator', 'bt_navigator'),
        nav2('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
             {'node_names': managed}),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mask', default_value='p3_available',
                              description='p3_available | p3_unavailable | none | path/to/mask.yaml | external'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel',
                              description='Topic for the final velocity command'),
        DeclareLaunchArgument('feasibility', default_value='false',
                              description='Start the physical-only feasibility planner (/feasibility)'),
        OpaqueFunction(function=launch_setup),
    ])
