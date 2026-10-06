from launch import LaunchDescription
from launch_ros.actions import Node

from stemm_nav2_manager.paths import map_basename


def generate_launch_description():
    return LaunchDescription([
        Node(
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
    ])
