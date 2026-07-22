"""External entry point: run the isolated session wrapper as one managed task."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'stemm_cartographer_exploration',
                'stemm_cartographer_session_runner',
            ],
            output='screen',
        ),
    ])
