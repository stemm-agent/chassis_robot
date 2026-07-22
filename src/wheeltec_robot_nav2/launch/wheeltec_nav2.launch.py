import os,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_lasertracker = LaunchConfiguration('enable_lasertracker')

    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
        
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')
    cfg_params = load_yaml(os.path.join(get_package_share_directory('turn_on_wheeltec_robot'),'config','wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode:{car_mode}")

    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    # 如果 install 下仍断裂，可用主机本地路径替代：
    # map_dir = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map'
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'stemm_cartographer_map.yaml'))

    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, f'param_{car_mode}.yaml'))
    print(os.path.join(param_dir, f'param_{car_mode}.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'enable_lasertracker',
            default_value='true',
            description='Start the shared laser tracker with Nav2'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_launch_dir, '/wheeltec_lidar.launch.py']),
        ),
        Node(
            name='lasertracker',
            package='simple_follower_ros2',
            executable='lasertracker',
            output='screen',
            condition=IfCondition(enable_lasertracker),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])
