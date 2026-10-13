"""LPL 2.0 run: simulation + Nav2 constrained by LPL + evidence + authority + dashboard.

    ros2 launch lpl2_core lpl.launch.py [scenario:=lpl1_continue] [mode:=lpl] [gui:=true]
                                        [record:=false] [dashboard:=true]

mode: lpl (LPL 2.0) | permissive (no LPL: evidence ignored) | global_revoke (any
      insufficient evidence stops everything). Same world, same autonomy; only the
      authority mode changes, for the comparison runs.
Pipeline:
  world events -> evidence_source -> lpl_authority (permissions) -> constraint_compiler -> /keepout_mask -> Nav2
  lpl_authority + /feasibility (physical map) -> option_planner (available actions, selection, escalation)
  -> option_executor -> Nav2 NavigateThroughPoses (on the LPL-constrained costmap)
  -> /lpl2/cmd_vel_auto -> cmd_gate (e-stop, lease, watchdog, geofence) -> /cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, name, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=args.items())


def lpl_node(executable, **params):
    return Node(package='lpl2_core', executable=executable, output='screen',
                parameters=[{'use_sim_time': True, **params}])


def recording(context):
    """record:=true records the dashboard (full LPL view) if it runs, else the plain camera recorder."""
    record = LaunchConfiguration('record').perform(context).lower() in ('true', '1')
    dashboard = LaunchConfiguration('dashboard').perform(context).lower() in ('true', '1')
    scenario = LaunchConfiguration('scenario').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    actions = [include('lpl2_sim', 'sim.launch.py', scenario=scenario, gui=LaunchConfiguration('gui'),
                       record=str(record and not dashboard).lower())]
    if dashboard:
        actions.append(Node(package='lpl2_ui', executable='dashboard', output='screen',
                            parameters=[{'record': record, 'scenario': f'{scenario}_{mode}'}]))
    return actions


def generate_launch_description():
    scenario, mode = LaunchConfiguration('scenario'), LaunchConfiguration('mode')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='lpl1_continue'),
        DeclareLaunchArgument('mode', default_value='lpl'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        OpaqueFunction(function=recording),
        lpl_node('evidence_source'),
        lpl_node('lpl_authority', mode=mode),
        lpl_node('constraint_compiler'),
        lpl_node('option_planner'),
        # the only /cmd_vel writer; if it dies, stop everything (DiffDrive keeps the last command)
        Node(package='lpl2_core', executable='cmd_gate', output='screen',
             parameters=[{'use_sim_time': True}], on_exit=Shutdown(reason='cmd_gate exited')),
        # let Gazebo, the robot and its TF come up before Nav2 activates
        TimerAction(period=8.0, actions=[include('lpl2_nav', 'nav.launch.py', mask='external', feasibility='true',
                                                         cmd_vel_out='/lpl2/cmd_vel_auto')]),
        TimerAction(period=12.0, actions=[lpl_node('option_executor')]),
    ])
