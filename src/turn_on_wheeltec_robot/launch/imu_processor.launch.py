import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'),
        'config',
        'imu_bias_compensator.yaml'
    )

    imu_processor = launch_ros.actions.Node(
            package='turn_on_wheeltec_robot', 
            executable='ImuProcessor', 
            name='imu_processor',
            parameters=[param_file],
    )
    return LaunchDescription([
        imu_processor
    ])

