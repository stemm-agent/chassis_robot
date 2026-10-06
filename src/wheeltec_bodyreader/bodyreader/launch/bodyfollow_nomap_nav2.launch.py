"""Map-free (odom-frame) Nav2 for body-follow mode, isolated in the 'nofmap' namespace.

Why: when navigation is not running there is no map and no AMCL, so the map-based Nav2
cannot be used. This stack keeps both costmaps in the odom frame over a rolling 6x6 m
window, so MPPI + Smac give real local planning/obstacle avoidance with no map at all.

It shares the parameter framework with the map stack: param_nomap_odom.yaml is generated
from the live map-mode parameter file (same planners, controllers, inflation, and the
body-follow tuning baked in), only the frames/rolling window/static layer differ.

Interfaces (all private, so the map stack is untouched):
  actions : /nofmap/navigate_to_pose, /nofmap/compute_path_to_pose, /nofmap/follow_path
  velocity: /nofmap/cmd_vel        (the body follower forwards it while it owns cmd_vel)
  costmaps: /nofmap/{local,global}_costmap/costmap
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_PARAMS = os.path.join(
    '/home/hnh/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params',
    'param_nomap_odom.yaml')


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='nofmap'),
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        LogInfo(msg=['Starting map-free (odom) Nav2 in namespace ', namespace,
                     ' with ', params_file]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{'autostart': autostart,
                         'use_sim_time': use_sim_time,
                         'bond_timeout': 0.0,
                         'node_names': lifecycle_nodes}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
