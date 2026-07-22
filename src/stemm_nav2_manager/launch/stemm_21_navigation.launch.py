import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('wheeltec_nav2')
    map_file = LaunchConfiguration(
        'map',
        default='/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.yaml'
    )

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
            'map_save_path': '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map',
            'global_frame': 'map',
            'base_frame': 'base_footprint',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to saved map yaml file'
        ),
        nav,
        manager,
    ])
