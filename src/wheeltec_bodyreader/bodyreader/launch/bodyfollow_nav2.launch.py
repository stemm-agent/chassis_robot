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
        # Voice and other controllers request follow state through this service;
        # it is the only publisher of the real /mode topic in this launch.
        Node(
            package='bodyreader',
            executable='bodyfollow_mode_controller',
            name='bodyfollow_mode_controller',
            output='screen',
            parameters=[
                {'service_name': '/voice_summon/set_body_follow_enabled'},
                {'legacy_service_name': '/bodyfollow/set_enabled'},
                {'status_service_name': '/voice_summon/get_body_follow_status'},
                {'mode_topic': '/mode'},
            ],
        ),

        Node(
            package='bodyreader',
            executable='body_nav2_profile_guard',
            name='body_nav2_profile_guard',
            output='screen',
            parameters=[
                {'enabled': True},
                {'restore_on_shutdown': True},
                {'mode_control_enabled': True},
                {'mode_required': 2},
                {'controller_profile_enabled': True},
                {'behavior_profile_enabled': True},
                {'amcl_profile_enabled': False},
                {'controller_server_name': '/controller_server'},
                {'behavior_server_name': '/behavior_server'},
                {'amcl_node_name': '/amcl'},
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
                {'controller_yaw_goal_tolerance': 0.20},
                {'prefer_forward_enabled': True},
                {'path_angle_forward_preference': True},
                {'progress_required_movement_radius': 0.08},
                {'progress_movement_time_allowance': 25.0},

                # Recovery behaviors should rotate responsively but ramp gently.
                {'behavior_max_rotational_vel': 0.55},
                {'behavior_min_rotational_vel': 0.04},
                {'behavior_rotational_acc_lim': 0.28},
                {'behavior_simulate_ahead_time': 1.20},

                # AMCL follow profile for 0.75 m/s visual motion and fast yaw.
                # The guard snapshots the running Nav2 values and restores them
                # when /mode leaves 2 or this process shuts down.
                {'amcl_alpha1': 0.10},
                {'amcl_alpha2': 0.10},
                {'amcl_alpha3': 0.10},
                {'amcl_alpha4': 0.10},
                {'amcl_update_min_d': 0.15},
                {'amcl_update_min_a': 0.10},
                {'amcl_laser_model_type': 'likelihood_field_prob'},
                {'amcl_do_beamskip': True},
            ],
        ),

        Node(
            package='bodyreader',
            executable='main',
            name='body_main',
            output='screen',
            respawn=True,
            respawn_delay=0.5,
            parameters=[
                {'rgb_stream': True},
                {'body_stream': True},
                {'mode_gated_body_stream': False},
                {'mode_required': 2},
                {'max_processing_rate_hz': 15.0},
            ],
        ),

        Node(
            package='bodyreader',
            executable='torso_trt_demo',
            name='torso_trt_demo',
            output='screen',
            parameters=[
                {
                    'engine_path': '/home/wheeltec/wheeltec_ros2/src/ultralytics_ros2/model/yolo11n_fp16.engine',
                    'image_topic': '/image_raw',
                    'annotated_topic': '/torso_trt_demo/image',
                    'detections_topic': '/torso_trt_demo/detections',
                    'state_topic': '/torso_trt_demo/state',
                    'features_topic': '/torso_trt_demo/features',
                    'locked_topic': '/torso_trt_demo/locked_target',
                    'lock_command_topic': '/torso_trt_demo/lock_command',
                    'depth_topic': '/camera/depth/image_raw',
                    'camera_info_topic': '/camera/color/camera_info',
                    'bodylist_topic': '/bodylist',
                    'bodyposture_topic': '/body_posture',
                    'input_width': 640,
                    'input_height': 640,
                    'conf_threshold': 0.50,
                    'nms_iou_threshold': 0.45,
                    'person_class_id': 0,
                    'max_detections': 12,
                    'min_person_area_px': 600.0,
                    'torso_side_crop_ratio': 0.18,
                    'torso_top_ratio': 0.22,
                    'torso_bottom_ratio': 0.68,
                    'upper_color_side_crop_ratio': 0.28,
                    'upper_color_band_top_ratio': 0.00,
                    'upper_color_band_bottom_ratio': 0.50,
                    'upper_color_inner_top_ratio': 0.34,
                    'upper_color_inner_bottom_ratio': 0.84,
                    'lower_color_side_crop_ratio': 0.26,
                    'lower_color_band_top_ratio': 0.50,
                    'lower_color_band_bottom_ratio': 1.00,
                    'lower_color_inner_top_ratio': 0.08,
                    'lower_color_inner_bottom_ratio': 0.44,
                    'respect_mode_topic': True,
                    'mode_required': 2,
                    'initial_mode': 1,
                    'max_inference_fps': 15.0,
                    'max_annotated_fps': 8.0,
                    'annotated_only_if_subscribed': True,
                    'opencv_num_threads': 2,
                    'publish_annotated': True,
                    'overlay_skeleton': True,
                    'skeleton_max_age_sec': 0.5,
                    'skeleton_base_width': 640.0,
                    'skeleton_base_height': 480.0,
                    'log_image_features': False,
                    'feature_log_period_sec': 2.0,
                    'max_logged_targets': 3,
                    'color_saturation_min': 22.0,
                    'color_value_min': 35.0,
                    'height_smoothing_alpha': 0.35,
                    'enable_depth_height': True,
                    'camera_fy_px': 0.0,
                    'depth_max_age_sec': 0.5,
                    'depth_unit_scale': 0.001,
                    'min_valid_depth_m': 0.20,
                    'max_valid_depth_m': 8.0,
                    'height_reference_m': 0.0,
                    'height_reference_px': 0.0,
                    'auto_lock_center': False,
                    'track_max_missed': 45,
                    'track_match_threshold': 0.82,
                    'track_iou_weight': 0.25,
                    'track_appearance_weight': 0.50,
                    'track_size_weight': 0.15,
                    'track_center_weight': 0.10,
                }
            ],
        ),

        Node(
            package='bodyreader',
            executable='body_identity_bridge',
            name='body_identity_bridge',
            output='screen',
            parameters=[
                {
                    'bodylist_topic': '/bodylist',
                    'bodyposture_topic': '/body_posture',
                    'features_topic': '/torso_trt_demo/features',
                    'lock_command_topic': '/torso_trt_demo/lock_command',
                    'recoveryid_topic': '/recoveryid',
                    'state_topic': '/body_identity_state',
                    'validated_bodyposture_topic': '/body_posture_yolo_validated',
                    'image_width': 640.0,
                    'horizontal_fov_rad': 1.05,
                    'yolo_max_age_s': 0.25,
                    'body_max_age_s': 0.25,
                    'initial_yolo_wait_s': 1.5,
                    'recovery_publish_period_s': 0.45,
                    'lock_command_period_s': 0.8,
                    'match_max_angle_rad': 0.18,
                    'match_max_depth_diff_m': 0.85,
                    'match_accept_score': 1.45,
                    'reacquire_accept_score': 1.65,
                    'appearance_accept_score': 0.80,
                    'bound_appearance_accept_score': 1.15,
                    'identity_validation_grace_s': 0.50,
                    'reacquire_candidate_margin': 0.18,
                    'initial_confirm_frames': 4,
                    'reacquire_confirm_frames': 4,
                    'require_yolo_for_initial_lock': True,
                }
            ],
        ),

        Node(
            package='bodyreader',
            executable='bodydata_process',
            name='bodydata_process',
            output='screen',
            parameters=[
                # The identity bridge owns automatic /recoveryid selection so
                # bodydata_process never blindly switches to the first body.
                {'require_akimbo_lock': False},
                {'auto_lock_first_body': False},
                {'initial_mode': 1},
                {'open_switch': False},
            ],
            remappings=[
                ('/cmd_vel', '/body_nav2/bodydata_process_cmd_vel_ignored'),
                ('/mode', '/body_nav2/bodydata_process_mode_ignored'),
            ],
        ),

        Node(
            package='bodyreader',
            executable='body_nav2_follower',
            name='body_nav2_follower',
            output='screen',
            parameters=[
                {'nav_action_name': '/navigate_to_pose'},
                {'behavior_tree':
                    '/home/wheeltec/wheeltec_ros2/src/wheeltec_bodyreader/bodyreader/behavior_trees/body_follow_fast_avoidance.xml'},
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

                {'follow_distance_m': 1.5},
                {'hold_distance_band_m': 0.12},
                {'min_goal_distance_m': 0.15},
                {'goal_update_period_s': 0.8},
                {'goal_position_tolerance_m': 0.25},
                {'goal_yaw_tolerance_rad': 0.15},
                {'lost_timeout_s': 1.5},
                {'body_invalid_grace_s': 0.45},
                {'tf_timeout_s': 0.20},
                {'enable_retreat_goal': False},
                {'max_retreat_goal_m': 0.8},

                # Hybrid decision policy:
                # clear: pure visual PD; slow band: visual PD with linear scale;
                # danger band: Nav2 takes over /cmd_vel until clearance recovers.
                {'require_scan_for_visual': True},
                {'scan_timeout_s': 0.6},
                {'obstacle_front_angle_rad': 0.45},
                {'side_obstacle_min_angle_rad': 0.35},
                {'side_obstacle_max_angle_rad': 1.57},
                {'obstacle_slow_distance_m': 0.80},
                {'obstacle_nav_distance_m': 0.50},
                {'obstacle_nav_release_distance_m': 0.80},
                {'side_obstacle_nav_distance_m': 0.40},
                {'side_obstacle_release_distance_m': 0.50},
                {'visual_min_linear_scale': 0.45},
                {'target_exemption_angle_rad': 0.20},
                {'target_exemption_distance_margin_m': 0.12},
                {'enable_avoidance_side_goal': True},
                {'avoidance_lateral_offset_m': 0.72},
                {'avoidance_forward_margin_m': 0.55},
                {'avoidance_min_forward_step_m': 0.90},
                {'avoidance_max_forward_step_m': 1.55},
                {'avoidance_side_angle_rad': 1.35},
                {'avoidance_side_clearance_min_m': 0.45},
                {'avoidance_target_side_bias_m': 0.20},
                {'avoidance_side_hold_s': 4.0},
                {'avoidance_occlusion_hold_s': 10.0},
                {'avoidance_gate_switch_distance_m': 0.45},
                {'avoidance_gate_timeout_s': 5.0},
                {'avoidance_anchor_blend_alpha': 0.20},
                {'visual_clear_takeover_front_clearance_m': 0.80},
                {'visual_clear_takeover_side_clearance_m': 0.50},
                {'visual_clear_takeover_min_avoidance_s': 0.60},
                {'visual_clear_takeover_body_max_age_s': 0.30},
                {'visual_clear_takeover_confirm_frames': 3},

                # Visual heading PD plus distance-proportional longitudinal speed.
                {'visual_x_p': 0.5},
                {'visual_x_d': 0.33},
                {'visual_z_p': 2.2},
                {'visual_z_d': 0.30},
                {'visual_filter_alpha': 0.62},
                {'visual_angle_deadband': 0.015},
                {'visual_distance_deadband_mm': 80.0},
                {'visual_max_linear_mps': 0.75},
                {'visual_max_angular_rps': 1.15},
                {'visual_linear_accel_limit': 0.90},
                {'visual_reverse_accel_limit': 1.80},
                {'visual_linear_decel_limit': 0.35},
                {'visual_angular_accel_limit': 2.2},
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
                {'voice_feedback': True},
                {'gesture_voice_feedback': False},
                {'interaction': 0},
            ],
        ),

    ])
