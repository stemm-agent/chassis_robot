"""
Body follow through an already-running Nav2 stack.

This launch file does not start Nav2, lidar, chassis bringup, follower,
body_avoider, or collision_guard. It only starts body perception plus a
Nav2 profile guard and a NavigateToPose bridge node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bodyreader',
            executable='body_nav2_profile_guard',
            name='body_nav2_profile_guard',
            output='screen',
            parameters=[
                {'enabled': True},
                {'restore_on_shutdown': True},
                {'controller_profile_enabled': True},
                {'behavior_profile_enabled': True},
                {'controller_server_name': '/controller_server'},
                {'behavior_server_name': '/behavior_server'},
                {'wait_timeout_s': 6.0},

                # MPPI follow profile: smaller sampling jumps and a faster
                # controller tick for smoother UX, without reverse motion.
                {'controller_frequency': 20.0},
                {'controller_vx_min': 0.04},
                {'controller_vx_max': 0.55},
                {'controller_vx_std': 0.24},
                {'controller_wz_max': 1.05},
                {'controller_wz_std': 0.32},
                {'controller_retry_attempt_limit': 2},
                {'controller_failure_tolerance': 4.0},
                {'prefer_forward_enabled': True},
                {'path_angle_forward_preference': True},
                {'progress_required_movement_radius': 0.08},
                {'progress_movement_time_allowance': 25.0},

                # Recovery behaviors should rotate responsively but ramp gently.
                {'behavior_max_rotational_vel': 0.55},
                {'behavior_min_rotational_vel': 0.04},
                {'behavior_rotational_acc_lim': 0.28},
                {'behavior_simulate_ahead_time': 1.20},
            ],
        ),

        Node(
            package='bodyreader',
            executable='main',
            name='body_main',
            output='screen',
            parameters=[
                {'rgb_stream': True},
                {'body_stream': True},
            ],
        ),

        Node(
            package='bodyreader',
            executable='bodydata_process',
            name='bodydata_process',
            output='screen',
            parameters=[
                # Voice summon turns toward the speaker first; then keep
                # tracking the first visible body without akimbo calibration.
                {'require_akimbo_lock': False},
                {'auto_lock_first_body': True},
                {'initial_mode': 2},
                {'open_switch': False},
            ],
            remappings=[
                ('/cmd_vel', '/body_nav2/bodydata_process_cmd_vel_ignored'),
            ],
        ),

        Node(
            package='bodyreader',
            executable='body_nav2_follower',
            name='body_nav2_follower',
            output='screen',
            parameters=[
                {'nav_action_name': '/navigate_to_pose'},
                {'spin_action_name': '/spin'},
                {'scan_topic': '/scan'},
                {'visual_cmd_topic': '/cmd_vel'},
                {'global_frame': 'map'},
                {'base_frame': 'base_footprint'},

                # Bodyposture reports Astra body coordinates without a header.
                # By default, map Astra Z->base X and Astra X->base Y. If a
                # calibrated camera TF is available, set body_frame_id.
                {'body_frame_id': ''},
                {'camera_offset_x_m': 0.0},
                {'camera_offset_y_m': 0.0},
                {'camera_offset_z_m': 0.0},
                {'camera_yaw_offset_rad': 0.0},

                {'initial_mode': 2},
                {'follow_distance_m': 1.5},
                {'hold_distance_band_m': 0.12},
                {'min_goal_distance_m': 0.15},
                {'goal_update_period_s': 0.8},
                {'goal_position_tolerance_m': 0.25},
                {'goal_yaw_tolerance_rad': 0.30},
                {'lost_timeout_s': 1.5},
                {'tf_timeout_s': 0.20},
                {'enable_retreat_goal': False},
                {'max_retreat_goal_m': 0.8},

                # Hybrid decision policy:
                # clear: pure visual PD; slow band: visual PD with linear scale;
                # danger band: Nav2 takes over /cmd_vel until clearance recovers.
                {'require_scan_for_visual': True},
                {'scan_timeout_s': 0.6},
                {'obstacle_front_angle_rad': 0.45},
                {'obstacle_slow_distance_m': 1.45},
                {'obstacle_nav_distance_m': 0.85},
                {'obstacle_nav_release_distance_m': 1.50},
                {'visual_min_linear_scale': 0.45},
                {'target_exemption_angle_rad': 0.30},
                {'target_exemption_distance_margin_m': 0.45},
                {'enable_avoidance_side_goal': True},
                {'avoidance_forward_margin_m': 0.55},
                {'avoidance_min_forward_step_m': 0.90},
                {'avoidance_max_forward_step_m': 1.55},
                {'avoidance_occlusion_hold_s': 10.0},

                # Pure visual follow PD, matching follower_no_avoider behavior.
                {'visual_x_p': 0.5},
                {'visual_x_d': 0.33},
                {'visual_z_p': 1.2},
                {'visual_z_d': 0.5},
                {'visual_filter_alpha': 0.35},
                {'visual_angle_deadband': 0.025},
                {'visual_distance_deadband_mm': 80.0},
                {'visual_max_linear_mps': 0.5},
                {'visual_max_angular_rps': 1.0},
                {'visual_linear_accel_limit': 0.45},
                {'visual_angular_accel_limit': 0.9},
                {'visual_allow_reverse': True},

                {'keep_last_goal_when_occluded': True},
                {'target_memory_timeout_s': 12.0},
                {'reacquire_yaw_goal': False},
                {'reacquire_timeout_s': 18.0},
                {'nav2_retry_backoff_s': 1.5},
                {'reacquire_spin_min_yaw_rad': 0.35},
                {'reacquire_spin_max_yaw_rad': 1.2},
                {'spin_time_allowance_s': 8.0},
                {'spin_retry_period_s': 2.5},
                {'enable_cmd_watchdog': True},
                {'cmd_zero_timeout_s': 8.0},
                {'cmd_zero_linear_epsilon': 0.02},
                {'cmd_zero_angular_epsilon': 0.05},

                {'respect_mode_topic': True},
                {'mode_required': 2},
                {'cancel_on_lost': True},
            ],
        ),

        Node(
            package='bodyreader',
            executable='feedback',
            name='body_feedback',
            output='screen',
            remappings=[
                ('/cmd_vel', '/body_nav2/body_feedback_cmd_vel_ignored'),
            ],
            parameters=[
                {'voice_feedback': False},
                {'gesture_voice_feedback': False},
                {'interaction': 0},
            ],
        ),

        Node(
            package='bodyreader',
            executable='display.py',
            name='body_display',
        ),

        Node(
            package='bodyreader',
            executable='compressed.py',
            name='compressed',
            parameters=[
                {'input_image_topic': '/body/body_display'},
                {'output_image_topic': '/repub/body/body_display/compressed'},
            ],
        ),
    ])
