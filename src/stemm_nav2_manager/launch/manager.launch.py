from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
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
    ])
