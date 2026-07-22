import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    auto_explore = LaunchConfiguration('auto_explore', default='true')
    slam_dir = get_package_share_directory('wheeltec_slam_toolbox')
    rrt_dir = get_package_share_directory('wheeltec_robot_rrt')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrt_dir, 'launch', 'navigation', 'nav2_bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'map': os.path.join(rrt_dir, 'map_data', 'image_map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(get_package_share_directory('stemm_nav2_manager'), 'config', 'stemm_safe_nav2_params.yaml'),
            'default_bt_xml_filename': os.path.join(rrt_dir, 'behaviour_trees', 'navigate_w_replanning_time.xml'),
            'autostart': 'True',
        }.items(),
    )

    rrt_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrt_dir, 'launch', 'rrt_exploration', 'rrt_exploration.launch.py')
        ),
        condition=IfCondition(auto_explore),
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
            'frontier_goal_topic': '/assigned_frontier_goal',
            'auto_explore': auto_explore,
            'min_explore_distance': 1.6,
            'max_explore_distance': 4.0,
            'explore_startup_delay_sec': 8.0,
            'return_home_quiet_sec': 2.0,
            'return_home_tf_max_age_sec': 0.20,
            'return_home_tf_stable_sec': 1.5,
            'return_home_max_retries': 3,
            'return_home_retry_delay_sec': 2.0,
            'return_home_start_clearance': 0.22,
            'home_known_radius': 0.45,
            'front_stop_distance': 0.42,
            'goal_clearance': 0.38,
            'front_emergency_distance': 0.28,
        }],
    )

    reset_odom = Node(
        package='stemm_nav2_manager',
        executable='mapping_odom_reset',
        name='mapping_odom_reset',
        output='screen',
        parameters=[{
            'odom_frame': 'odom_combined',
            'base_frame': 'base_footprint',
            'timeout_sec': 15.0,
            'position_tolerance': 0.03,
            'yaw_tolerance': 0.0523598776,
            'stable_sec': 0.8,
        }],
    )

    tf_guard = Node(
        package='stemm_nav2_manager',
        executable='mapping_tf_guard',
        name='mapping_tf_guard',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'map_topic': '/map',
            'timeout_sec': 30.0,
            'max_tf_age_sec': 0.80,
            'position_tolerance': 0.10,
            'yaw_tolerance': 0.0872664626,
            'stable_sec': 1.5,
        }],
    )

    def start_mapping_after_reset(event, _context):
        if event.returncode == 0:
            return [
                slam,
                TimerAction(period=0.5, actions=[tf_guard]),
            ]
        return [
            LogInfo(msg='ERROR: Mapping odom reset failed; SLAM and exploration will not start.'),
            EmitEvent(event=Shutdown(reason='mapping odom reset failed')),
        ]

    def start_navigation_after_tf_guard(event, _context):
        if event.returncode == 0:
            return [
                nav2,
                manager,
                TimerAction(period=5.0, actions=[rrt_exploration]),
            ]
        return [
            LogInfo(msg='ERROR: Mapping TF health check failed; stopping this mapping session.'),
            EmitEvent(event=Shutdown(reason='mapping TF health check failed')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_explore', default_value='true'),
        SetEnvironmentVariable('STEMM_AUTO_MAPPING_LAUNCH', '1'),
        RegisterEventHandler(OnProcessExit(
            target_action=reset_odom,
            on_exit=start_mapping_after_reset,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=tf_guard,
            on_exit=start_navigation_after_tf_guard,
        )),
        reset_odom,
    ])
