# ros2 run nav2_map_server map_saver_cli -f <map-prefix>
import os

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    map_dir = os.path.expanduser(
        os.getenv('STEMM_MAP_DIR', '~/.local/share/stemm/maps'))
    os.makedirs(map_dir, exist_ok=True)
    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', os.path.join(map_dir, 'WHEELTEC')],
        parameters=[
            {'save_map_timeout': 20000.0},
            {'free_thresh_default': 0.196},
        ],
    )
    return LaunchDescription([map_saver])
