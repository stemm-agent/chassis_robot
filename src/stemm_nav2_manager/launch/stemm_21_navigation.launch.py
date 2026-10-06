import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from stemm_nav2_manager.paths import map_basename


def generate_launch_description():
    nav_dir = get_package_share_directory('wheeltec_nav2')
    default_map = os.path.join(nav_dir, 'map', 'stemm_map.yaml')
    map_file = LaunchConfiguration('map')

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'wheeltec_nav2.launch.py')
        ),
        launch_arguments={'map': map_file}.items(),
    )

    manager = Node(
        package='stemm_nav2_manager',
        executable='stemm_nav2_manager',
        name='stemm_nav2_manager',
        output='screen',
        parameters=[{
            'map_save_path': str(map_basename()),
            'global_frame': 'map',
            'base_frame': 'base_footprint',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to saved map yaml file'
        ),
        nav,
        manager,
    ])
