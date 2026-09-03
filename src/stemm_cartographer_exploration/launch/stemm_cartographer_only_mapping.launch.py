"""Cartographer-only mapping probe with no Nav2, AMCL, RRT, or session manager.

The chassis, odometry, IMU, and static TF publishers must already be running.
Do not run alongside any node that publishes map->odom (AMCL or another mapper).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')

    package_dir = get_package_share_directory('stemm_cartographer_exploration')
    robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    config_dir = os.path.join(package_dir, 'config')

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_dir, 'launch', 'wheeltec_lidar.launch.py')
        ),
        condition=IfCondition(start_lidar),
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='stemm_cartographer_only_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'stemm_cartographer_2d.lua',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu/data_bias_compensated'),
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='stemm_cartographer_only_occupancy_grid',
        output='screen',
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '2.0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'start_lidar', default_value='true',
            description='Start the configured Wheeltec lidar driver.'),
        lidar,
        cartographer,
        occupancy_grid,
    ])
