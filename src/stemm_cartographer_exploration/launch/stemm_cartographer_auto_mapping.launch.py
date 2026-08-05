"""Independent Cartographer replacement for stemm_21_auto_mapping.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    auto_explore = LaunchConfiguration('auto_explore', default='true')

    package_dir = get_package_share_directory(
        'stemm_cartographer_exploration')
    rrt_dir = get_package_share_directory('wheeltec_robot_rrt')
    robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    config_dir = os.path.join(package_dir, 'config')
    cartographer_config = 'stemm_cartographer_2d.lua'
    nav2_params = os.path.join(config_dir, 'stemm_safe_nav2_params.yaml')
    rrt_params = os.path.join(config_dir, 'exploration_params.yaml')

    system_preflight = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_preflight',
        name='stemm_cartographer_system_preflight',
        output='screen',
        parameters=[{
            'phase': 'system',
            'timeout_sec': 4.0,
        }],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )

    sensor_preflight = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_preflight',
        name='stemm_cartographer_sensor_preflight',
        output='screen',
        parameters=[{
            'phase': 'sensors',
            'timeout_sec': 8.0,
            'base_frame': 'base_footprint',
            'expected_imu_frame': 'gyro_link',
        }],
    )

    odom_session_guard = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_odom_session_guard',
        name='stemm_cartographer_odom_session_guard',
        output='screen',
        parameters=[{
            'odom_topic': '/odom_combined',
            'odom_frame': 'odom_combined',
            'base_frame': 'base_footprint',
            'cmd_vel_topic': '/cmd_vel',
            'timeout_sec': 10.0,
            'max_message_age_sec': 0.35,
            'max_tf_age_sec': 0.35,
            'stable_sec': 0.8,
            'position_drift_m': 0.025,
            'yaw_drift_rad': 0.0523598776,
            'linear_speed_mps': 0.02,
            'angular_speed_rps': 0.05,
            'position_step_m': 0.10,
            'yaw_step_rad': 0.1745329252,
            'pose_consistency_position_m': 0.03,
            'pose_consistency_yaw_rad': 0.0523598776,
            'required_advancing_messages': 5,
            'odom_queue_depth': 120,
            'ready_confirmation_sec': 0.15,
            'ready_confirmation_messages': 3,
            'use_sim_time': use_sim_time,
        }],
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='stemm_cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', cartographer_config,
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
        name='stemm_cartographer_occupancy_grid',
        output='screen',
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '2.0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_service_adapter = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_map_service_adapter',
        name='stemm_cartographer_map_service_adapter',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'map_service': '/stemm_cartographer/dynamic_map',
            'rrt_map_topic': '/stemm_cartographer/rrt_map',
            'rrt_map_service': '/stemm_cartographer/rrt_dynamic_map',
            'rrt_occupied_threshold': 50,
            'use_sim_time': use_sim_time,
        }],
    )

    state_saver = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_state_saver',
        name='stemm_cartographer_state_saver',
        output='screen',
        parameters=[{
            'state_filename':
                '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/'
                'map/stemm_cartographer_map.pbstream',
            'write_state_service': '/write_state',
            'timeout_sec': 30.0,
            'use_sim_time': use_sim_time,
        }],
    )

    tf_guard = Node(
        package='stemm_nav2_manager',
        executable='mapping_tf_guard',
        name='stemm_cartographer_mapping_tf_guard',
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrt_dir, 'launch', 'navigation',
                         'nav2_bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'map': os.path.join(rrt_dir, 'map_data', 'image_map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'True',
        }.items(),
    )

    manager = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_nav2_manager',
        name='stemm_cartographer_nav2_manager',
        output='screen',
        remappings=[
            ('/stemm/start_mapping',
             '/stemm_cartographer/start_mapping'),
            ('/stemm/save_map', '/stemm_cartographer/save_map'),
            ('/stemm/stop_navigation',
             '/stemm_cartographer/stop_navigation'),
            ('/stemm/cancel_mapping',
             '/stemm_cartographer/cancel_mapping'),
            ('/stemm/set_mapping_done',
             '/stemm_cartographer/set_mapping_done'),
            ('/stemm/nav_state', '/stemm_cartographer/nav_state'),
            ('/stemm/pose', '/stemm_cartographer/pose'),
            ('/stemm/nav_goal', '/stemm_cartographer/nav_goal'),
        ],
        parameters=[{
            'map_save_path':
                '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/'
                'map/stemm_cartographer_map',
            'global_frame': 'map',
            'base_frame': 'base_footprint',
            'goal_topic': '/stemm_cartographer/nav_goal',
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
            'mapping_finish_confirm_timeout_sec': 90.0,
            'return_home_hard_timeout_sec': 180.0,
            'mapping_timeout_cancel_delay_sec': 5.0,
            'return_home_start_clearance': 0.22,
            'home_known_radius': 0.45,
            'front_stop_distance': 0.42,
            'goal_clearance': 0.38,
            'front_emergency_distance': 0.28,
            'nav_lifecycle_request_timeout_sec': 1.5,
            'nav_lifecycle_retry_delay_sec': 0.5,
            'exploration_no_progress_timeout_sec': 18.0,
            'exploration_progress_delta_m': 0.12,
            'exploration_hard_timeout_sec': 50.0,
            'exploration_pose_stall_timeout_sec': 7.0,
            'exploration_pose_stall_radius_m': 0.10,
            'exploration_pose_stall_goal_tolerance_m': 0.30,
            'rrt_fallback_wait_sec': 0.0,
            'rrt_projection_radius_m': 1.0,
            'clear_costmaps_on_stall': True,
            'exploration_behavior_tree':
                '/home/wheeltec/wheeltec_ros2/install/'
                'stemm_cartographer_exploration/share/'
                'stemm_cartographer_exploration/config/'
                'stemm_cartographer_nav_to_pose.xml',
            'goal_acceptance_timeout_sec': 3.0,
            'feedback_zero_epsilon_m': 0.01,
            'feedback_zero_goal_tolerance_m': 0.30,
            'feedback_baseline_settle_sec': 2.0,
            'stall_cancel_response_timeout_sec': 2.0,
            'stall_cancel_result_timeout_sec': 5.0,
            'stall_cancel_max_attempts': 2,
            'stall_confirmation_sec': 0.8,
            'stall_spin_action_name': '/spin',
            'stall_spin_yaw_rad': 1.05,
            'stall_spin_escalated_yaw_rad': 1.30,
            'stall_spin_time_allowance_sec': 5.0,
            'stall_spin_acceptance_timeout_sec': 2.0,
            'stall_spin_result_timeout_sec': 6.0,
            'stall_spin_cancel_timeout_sec': 3.0,
            'costmap_clear_wait_sec': 0.6,
            'blocked_recovery_delay_sec': 1.5,
            'blocked_recovery_cooldown_sec': 5.0,
            'blocked_clearance_hysteresis_m': 0.08,
            'blocked_clearance_stable_sec': 0.5,
            'recovery_log_throttle_sec': 2.0,
            'cancel_zero_guard_period_sec': 0.05,
            'nav_lifecycle_retry_max_delay_sec': 2.0,
        }],
    )

    nav2_ready_guard = Node(
        package='stemm_cartographer_exploration',
        executable='stemm_cartographer_nav2_ready_guard',
        name='stemm_cartographer_nav2_ready_guard',
        output='screen',
        parameters=[{
            'timeout_sec': 35.0,
            'stable_sec': 0.8,
            'request_timeout_sec': 1.5,
            'retry_delay_sec': 0.5,
            'retry_max_delay_sec': 2.0,
            'use_sim_time': use_sim_time,
        }],
    )

    rrt_nodes = [
        Node(
            package='wheeltec_robot_rrt',
            executable=executable,
            name=executable,
            output='both',
            parameters=[rrt_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(auto_explore),
        )
        for executable in ('local_rrt', 'global_rrt', 'filter', 'assigner')
    ]

    def after_system_preflight(event, _context):
        if event.returncode == 0:
            return [
                lidar,
                sensor_preflight,
            ]
        return [
            LogInfo(msg='ERROR: System isolation preflight failed.'),
            EmitEvent(event=Shutdown(reason='system preflight failed')),
        ]

    def after_sensor_preflight(event, _context):
        if event.returncode == 0:
            return [odom_session_guard]
        return [
            LogInfo(msg='ERROR: Core sensor preflight failed.'),
            EmitEvent(event=Shutdown(reason='sensor preflight failed')),
        ]

    def after_odom_session_guard(event, _context):
        if event.returncode == 0:
            return [
                cartographer,
                occupancy_grid,
                map_service_adapter,
                state_saver,
                tf_guard,
            ]
        return [
            LogInfo(msg='ERROR: Mapping odometry session is not ready.'),
            EmitEvent(event=Shutdown(
                reason='mapping odometry session readiness failed')),
        ]

    def after_tf_guard(event, _context):
        if event.returncode == 0:
            return [
                nav2,
                nav2_ready_guard,
            ]
        return [
            LogInfo(msg='ERROR: Cartographer map/TF health check failed.'),
            EmitEvent(event=Shutdown(reason='Cartographer TF guard failed')),
        ]

    def after_nav2_ready(event, _context):
        if event.returncode == 0:
            # Keep the independent stable-ACTIVE gate even though the
            # Cartographer-specific manager can now recover from a transient
            # GetState timeout. Give its subscriptions a moment before RRT
            # publishes the first frontier goal.
            return [
                manager,
                TimerAction(period=1.0, actions=rrt_nodes),
            ]
        return [
            LogInfo(
                msg='ERROR: Nav2 did not become active; RRT will not start.'),
            EmitEvent(event=Shutdown(reason='Nav2 readiness guard failed')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_explore', default_value='true'),
        SetEnvironmentVariable('STEMM_AUTO_MAPPING_LAUNCH', '1'),
        RegisterEventHandler(OnProcessExit(
            target_action=system_preflight,
            on_exit=after_system_preflight,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=sensor_preflight,
            on_exit=after_sensor_preflight,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=odom_session_guard,
            on_exit=after_odom_session_guard,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=tf_guard,
            on_exit=after_tf_guard,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=nav2_ready_guard,
            on_exit=after_nav2_ready,
        )),
        system_preflight,
    ])
