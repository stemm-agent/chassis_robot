"""
Body follow through an already-running Nav2 stack.

This launch file does not start the map-mode Nav2 stack, chassis bringup,
follower, body_avoider, or collision_guard.  It starts body perception, a Nav2
profile guard, a NavigateToPose bridge node and the always-on map-free Nav2
stack.  The LiDAR is owned by wheeltec-lidar.service and merely consumed here.
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('bodyreader')
    legacy_perception = LaunchConfiguration('legacy_perception')
    cpu_perception = LaunchConfiguration('cpu_perception')
    legacy_default = (
        'true' if platform.machine().lower() in ('aarch64', 'arm64') else 'false')
    cpu_default = (
        'false' if platform.machine().lower() in ('aarch64', 'arm64') else 'true')
    workspace_root = os.path.expanduser(
        os.getenv('WHEELTEC_ROS2_ROOT', '~/wheeltec_ros2'))
    engine_path = os.getenv(
        'STEMM_TENSORRT_ENGINE',
        os.path.join(
            workspace_root, 'src', 'ultralytics_ros2', 'model',
            'yolo11n_fp16.engine'))
    behavior_tree = os.path.join(
        package_share, 'behavior_trees', 'body_follow_fast_avoidance.xml')

    cpu_model_path = os.path.join(package_share, 'model', 'yolo11n.onnx')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_nomap_nav2',
            default_value='true',
            description=(
                'Start the map-free (odom frame, rolling window) Nav2 stack in the '
                'nofmap namespace so following works without a map.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_share, 'launch', 'bodyfollow_nomap_nav2.launch.py')),
            condition=IfCondition(LaunchConfiguration('enable_nomap_nav2')),
        ),
        # F4 + F2 arbiter: pauses the /nofmap Nav2 lifecycle while the map-mode
        # stack runs (CPU contention) and alarms when more than one publisher owns
        # /scan (two LiDAR drivers splitting the serial stream).
        ExecuteProcess(
            cmd=['python3', '/home/hnh/stemm_navstack_arbiter.py'],
            output='screen',
        ),
        DeclareLaunchArgument(
            'legacy_perception',
            default_value=legacy_default,
            description=(
                'Enable the Jetson-only Astra BodyTracking and TensorRT nodes. '                 'The x86 default is disabled.')),
        DeclareLaunchArgument(
            'cpu_perception',
            default_value=cpu_default,
            description=(
                'Enable the x86 ONNX Runtime CPU observation node. '
                'It never publishes robot motion commands.')),
        LogInfo(
            condition=UnlessCondition(legacy_perception),
            msg=(
                'Legacy Jetson perception is disabled; keep motion gated until '                 'the x86 perception backend is validated.')),
        # The coordinator owns the public follow-control service.  This node is
        # deliberately internal: it is the only publisher of the real /mode
        # topic, but no external caller can bypass the motion-gate transaction.
        Node(
            package='bodyreader',
            executable='bodyfollow_mode_controller',
            name='bodyfollow_mode_controller',
            output='screen',
            parameters=[
                {'service_name': '/bodyfollow/set_enabled'},
                {'legacy_service_name': ''},
                {'status_service_name': '/bodyfollow/get_mode_status'},
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

                # Keep the controller at the production Nav2 rate.  The clear
                # path follower publishes visual commands directly, so a 20 Hz
                # MPPI loop only doubles navigation work during obstacle
                # takeover without improving the visual tracking cadence.
                {'controller_frequency': 10.0},
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
            condition=IfCondition(legacy_perception),
            parameters=[
                {'rgb_stream': True},
                {'body_stream': True},
                {'mode_gated_body_stream': True},
                {'mode_required': 2},
                # Astra delivered about 10 useful visual frames per second in
                # production.  Capping the RGB/body conversion at that rate
                # avoids producing a second queue of frames that TensorRT
                # cannot consume while preserving the summon observation rate.
                {'max_processing_rate_hz': 10.0},
                {'min_body_distance_mm': 600.0},
                {'jump_rebase_confirm_frames': 3},
                {'jump_rebase_consistency_mm': 150.0},
                {'jump_rebase_max_interval_s': 0.30},
            ],
        ),

        Node(
            package='bodyreader',
            executable='torso_trt_demo',
            name='torso_trt_demo',
            output='screen',
            condition=IfCondition(legacy_perception),
            parameters=[
                {
                    'engine_path': engine_path,
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
                    'max_inference_fps': 10.0,
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
                    'auto_lock_center': True,
                  'inference_backend': 'openvino',
                  'openvino_device': 'CPU',
                  'openvino_performance_hint': 'LATENCY',
                  'relock_memory_s': 3.0,
                  'relock_max_distance': 0.35,
                  'relock_relax_after_s': 5.0,
                  'relock_relax_min_hits': 2,
                  'relock_relax_min_score': 0.6,
                  'diag_enabled': True,
                  'diag_heartbeat_sec': 1.0,
                  'diag_gap_threshold_s': 0.80,
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
            executable='torso_cpu_demo',
            name='torso_cpu_demo',
            output='screen',
            condition=IfCondition(cpu_perception),
            parameters=[{
                'model_path': cpu_model_path,
                'image_topic': '/camera/color/image_raw',
                'registered_depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'mode_topic': '/mode',
                'required_mode': 2,
                'initial_mode': 1,
                'max_inference_hz': 15.0,   # 2026-10-06: 10 -> 15（提高检测密度，稳住轨迹）
                'onnx_intra_op_threads': 6,   # 2026-10-06: 4 -> 6（CPU 有余量）
                'confidence_threshold': 0.40,   # 2026-10-06: 0.50 -> 0.40（实测更连续）
                'nms_iou_threshold': 0.45,
                'max_detections': 12,
                'min_box_area_px': 600.0,
                'min_box_area_ratio': 0.0015,   # 2026-10-06: 最小框面积改为画面比例（≈460px@640x480）
                'depth_roi_width_scale': 1.4,   # 2026-10-06 (B): 深度采样区放大
                'depth_roi_height_scale': 1.4,
                'depth_mode_lower_pct': 20.0,   # 2026-10-06 (B): 近邻主簇种子分位
                'depth_mode_tolerance_m': 0.50,
                'depth_mode_min_pixels': 20,
                'lock_min_hits': 3,   # 2026-10-06: 4 -> 3（实测每轮只积累 2-3 次命中）
                'track_max_missed': 30,   # 2026-10-06: 12 -> 30（轨迹保留约 3s）
                'track_iou_threshold': 0.10,   # 2026-10-06: 0.20 -> 0.10（移动时也能续轨）
                'track_center_threshold': 0.20,   # 2026-10-06: 0.12 -> 0.20（容忍更大位移）
                'auto_lock_center': True,
                'detections_topic': '/torso_trt_demo/detections',
                'features_topic': '/torso_trt_demo/features',
                'locked_topic': '/torso_trt_demo/locked_target',
                'state_topic': '/torso_trt_demo/state',
                'lock_command_topic': '/torso_trt_demo/lock_command',
                'bodyposture_topic': '/body_posture_yolo_validated',
            }],
        ),

        Node(
            package='bodyreader',
            executable='body_identity_bridge',
            name='body_identity_bridge',
            output='screen',
            condition=IfCondition(legacy_perception),
            parameters=[
                {
                    'bodylist_topic': '/bodylist',
                    'features_topic': '/torso_trt_demo/features',
                    'lock_command_topic': '/torso_trt_demo/lock_command',
                    'state_topic': '/body_identity_state',
                    'validated_bodyposture_topic': '/body_posture_yolo_validated',
                    'image_width': 640.0,
                    'horizontal_fov_rad': 1.05,
                    'yolo_max_age_s': 0.25,
                    'body_max_age_s': 0.25,
                    'body_yolo_max_skew_s': 0.15,
                    'initial_yolo_wait_s': 1.5,
                    'lock_command_period_s': 0.8,
                    'match_max_angle_rad': 0.18,
                    'match_max_depth_diff_m': 0.85,
                    'match_accept_score': 1.45,
                    'reacquire_accept_score': 1.65,
                    'appearance_accept_score': 0.80,
                    # The already locked YOLO track remains identity-authoritative.
                    # Permit moderate illumination/pose variation without weakening
                    # cross-track re-identification.
                    'bound_appearance_accept_score': 1.30,
                    # Keep the unbound skeleton/YOLO overlap margin at 8% in C++;
                    # only the currently bound identity gets the wider margin.
                    'bound_body_bbox_margin_ratio': 0.12,
                    'reacquire_candidate_margin': 0.18,
                    'body_pair_candidate_margin': 0.12,
                    'initial_confirm_frames': 4,
                    'reacquire_confirm_frames': 4,
                    'require_yolo_for_initial_lock': True,
                }
            ],
        ),

        Node(
            package='bodyreader',
            executable='body_nav2_follower',
            name='body_nav2_follower',
            output='screen',
            parameters=[
                # A follower process restart must come back observation-only.
                # The coordinator reopens this gate only after a current,
                # identity-validated body has been confirmed.
                {'motion_enabled': False},
                {'nav_action_name': '/navigate_to_pose'},
                {'behavior_tree': behavior_tree},
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
                {'hold_distance_band_m': 0.30},   # 2026-10-06: 0.12 -> 0.30 压掉逐帧状态抖动
                {'human_retreat_trigger_extra_m': 0.68},   # 2026-10-06: 0.50 -> 0.68 补偿 hold band，后退阈值仍为 1.88m
                {'human_retreat_exit_hysteresis_m': 0.50},   # 2026-10-06: 0.20 -> 0.50（退到 2.38m 才停）
                {'min_goal_distance_m': 0.15},
                {'goal_update_period_s': 0.8},
                {'goal_position_tolerance_m': 0.25},
                {'goal_yaw_tolerance_rad': 0.15},
                {'lost_timeout_s': 1.5},
                {'body_invalid_grace_s': 0.80},
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
                # 2026-09-28 (operator correction): ONLY the Nav2 takeover distance
                # moves. Slow clearance and release keep their original values, so the
                # takeover must stay at or below the release distance (0.80) or the
                # gate would release as soon as it engaged.
                {'obstacle_nav_distance_m': 0.80},
                {'obstacle_nav_release_distance_m': 0.80},
                {'side_obstacle_nav_distance_m': 0.40},
                {'side_obstacle_release_distance_m': 0.45},
                {'visual_min_linear_scale': 0.45},
                {'target_exemption_angle_rad': 0.20},   # 2026-10-06: 0.35 -> 0.20 收窄豁免扇区（防真实障碍被误豁免）
                {'target_exemption_distance_margin_m': 0.30},
                {'target_exemption_outer_margin_m': 0.25},   # 2026-10-06: 外侧（比人远）放宽
                {'target_exemption_inner_margin_m': 0.10},   # 2026-10-06: 内侧（比人近）=真实障碍，收紧
                {'human_retreat_stall_settle_s': 2.0},   # 2026-10-06: 堵转改累计判据（治后退不灵敏）
                # 2026-10-06: 目标豁免标定与门控（相机位姿变化后可在现场改这里或用 param set）
                {'body_nav2_depth_camera_behind_lidar_m': 0.13},
                {'body_nav2_person_map_depth_scale': 1.03},
                {'body_nav2_target_half_width_m': 0.22},
                {'obstacle_slow_distance_m': 0.55},
                # 2026-10-06: 盲区下限 0.60 -> 0.45（A 步验证：人在 0.6m 内也能触发受控后退）
                {'human_retreat_min_valid_m': 0.45},
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
                {'visual_clear_takeover_side_clearance_m': 0.45},
                {'visual_clear_takeover_min_avoidance_s': 0.60},
                {'visual_clear_takeover_body_max_age_s': 0.30},
                {'visual_clear_takeover_require_body_max_age_s': 0.30},
                {'visual_takeover_body_stale_timeout_s': 3.0},
                # Confirm physical clearance with two independent LaserScan
                # messages, keeping takeover responsive without trusting one ray set.
                {'visual_clear_takeover_confirm_frames': 2},
                # 2026-09-28: how long a continuously-confirmed clear handback may be held back
                # by G2 while Nav2 is still progressing. 1.2 s keeps a nearly-finished push
                # intact but stops the handback from waiting for the whole detour; 999 restores
                # the old always-protect behaviour.
                {'avoid_goal_protect_clear_grace_s': 1.2},
                {'visual_safety_release_confirm_frames': 2},
                {'visual_safety_min_finite_rays_per_sector': 1},
                {'nav_takeover_child_capture_slop_s': 0.10},
                {'nav_takeover_cancel_retry_s': 0.35},
                {'nav_takeover_status_quiet_s': 0.30},
                {'nav_takeover_cmd_quiet_s': 0.30},
                # 2026-09-28: 1.20 s of unconditional padding after the parent action went
                # terminal, on top of the 0.30 s quiet confirmation, added up to exactly the
                # 1.50 s drain timeout -- so every drain ended in the forced-timeout path.
                # Status freshness is verified separately, so the padding drops to 0.60 s
                # and the timeout becomes a real net at 2.00 s.
                {'nav_takeover_child_discovery_s': 0.60},
                {'takeover_drain_timeout_s': 2.00},

                # Visual heading PD plus distance-proportional longitudinal speed.
                {'visual_x_p': 0.5},
                {'visual_x_d': 0.33},
                {'visual_z_p': 2.2},
                {'visual_z_d': 0.60},   # 2026-10-06: 0.30 -> 0.60 阻尼翻倍，治过冲（越过人体中心）
                {'visual_filter_alpha': 0.75},   # 2026-10-06: 0.62 -> 0.75 滤波时间常数 107ms -> 67ms
                {'visual_angle_deadband': 0.008},   # 2026-10-06: 0.015 -> 0.008（1.55m 处 23mm -> 12mm 才响应）
                {'visual_distance_deadband_mm': 80.0},
                # 2026-09-28: forward follow ceiling raised by request; reverse stays at
                # its previous value (forward*1.00 == 0.75) so the reverse/retreat path
                # is unchanged.
                {'visual_max_linear_mps': 0.80},
                # 2026-09-28: operator request -- reverse ceiling lowered from 0.75 to
                # 0.50 m/s. Reverse is measured to reach exactly this value on the live
                # trace, so it is applied independently of the forward ceiling; the
                # human-approach retreat is distance-proportional and is capped by it too.
                {'visual_max_reverse_mps': 0.50},
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
