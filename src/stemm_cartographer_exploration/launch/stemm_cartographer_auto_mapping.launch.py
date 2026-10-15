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
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from stemm_nav2_manager.paths import map_basename


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    auto_explore = LaunchConfiguration('auto_explore', default='true')
    # 2026-10-10: 前沿点来源。'rrt' = 现有随机采样链（默认，保持现状）；
    # 'grid' = stemm_frontier_explorer 的栅格穷举 + 聚类 + 距离场打分。
    # 两者都发布 /assigned_frontier_goal (PoseStamped)，下游零改动。
    frontier_source = LaunchConfiguration('frontier_source', default='rrt')

    # 2026-10-10: 卡死脱困控制器开关（默认关闭）。
    escape_controller = LaunchConfiguration(
        'escape_controller', default='true')

    package_dir = get_package_share_directory(
        'stemm_cartographer_exploration')
    rrt_dir = get_package_share_directory('wheeltec_robot_rrt')
    robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    config_dir = os.path.join(package_dir, 'config')
    cartographer_config = 'stemm_cartographer_2d.lua'
    nav2_params = os.path.join(config_dir, 'stemm_safe_nav2_params.yaml')
    rrt_params = os.path.join(config_dir, 'exploration_params.yaml')
    cartographer_map_basename = str(map_basename('stemm_cartographer_map'))

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
            'state_filename': cartographer_map_basename + '.pbstream',
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
            'map_save_path': cartographer_map_basename,
            'global_frame': 'map',
            'base_frame': 'base_footprint',
            'goal_topic': '/stemm_cartographer/nav_goal',
            'frontier_goal_topic': '/assigned_frontier_goal',
            'auto_explore': auto_explore,
            'min_explore_distance': 1.6,
            # 2026-09-29: raised from 4.0 together with the RRT frontier fix -- the frontier
            # chain was dead code, so every goal came from the >=6 m fallback sampler and the
            # robot never left a ~4 m bubble.
            'max_explore_distance': 10.0,
            'explore_startup_delay_sec': 8.0,
            'return_home_quiet_sec': 2.0,
            'return_home_tf_max_age_sec': 0.20,
            'return_home_tf_stable_sec': 1.5,
            'return_home_max_retries': 3,
            # 2026-10-07（修正上一版的一次性关停）：倒车按生命周期拆成两个独立开关。
            #   唯一的反向速度发布点仍是 cartographer_manager._retreat_from_occupied_cells，
            #   但它的闸门现在按“当前阶段”取：
            #     探索期 R1b 清障后退 / R1c 发目标前净空后退  -> exploration_retreat_enabled
            #     返航   R1 起点被占用 / 返航重试前后退        -> return_home_retreat_enabled
            # 现场事实：雷达因安装角度看不到后方，返航倒车等于盲倒，已于 2026-10-07 13:38 撞障，
            # 因此返航倒车默认关闭；探索期脱困保持开启（上一版误将其一并关停）。
            'exploration_retreat_enabled': True,
            # 2026-10-13 P12-#1：escape 无效旋转换向（同一位置同方向失败过就反向）。
            'escape_flip_after_failed_budget': True,
            'escape_flip_max_per_spot': 2,
            'escape_flip_spot_radius_m': 0.30,
            # 2026-10-13 P11 方案4：探索专用后退参数（与返航解耦）。
            # 此前探索共用 return_home_retreat_*（max_dist=0.90）导致探测深度
            # probe = 0.90+0.25 = 1.15m，后方这么长的带状区内任一点被占即判
            # rear-blocked，实测频繁误判（14:03 轮 4 次后退里 2 次失败）。
            # 现探索用 0.35m/0.10m/s：probe 降到 0.60m，更短更慢但更可行。
            'exploration_retreat_speed_mps': 0.10,
            'exploration_retreat_max_distance_m': 0.35,
            'exploration_retreat_max_sec': 3.0,
            'exploration_retreat_rear_margin_m': 0.25,
            # 2026-10-13 P11 方案1：连续 rear-blocked >=2 次后原地转向 90 度。
            # 转向方向 = 让车头背离上次失败目标的方位（不再往同一扇区派发）。
            # 护栏：最多 2 次、累计 <=180 度，防止变成新的自转循环。
            'exploration_blocked_turn_deg': 90.0,
            'exploration_blocked_turn_max_tries': 2,
            'exploration_blocked_turn_total_limit_deg': 180.0,
            'exploration_blocked_turn_speed_radps': 0.40,
            'return_home_retreat_enabled': False,
            'return_home_retreat_speed_mps': 0.15,
            # 2026-10-06: 0.60 -> 0.90。返航失败常见形态是起点被近处障碍夹住
            # （planner: no valid path found），0.60m 有时不足以让出通道。
            'return_home_retreat_max_distance_m': 0.90,
            # 2026-10-06: 3.0 -> 5.0，配合 0.90m（限速仍为 0.15m/s）。
            'return_home_retreat_max_sec': 5.0,
            'return_home_retreat_rear_margin_m': 0.25,
            'return_home_retreat_settle_sec': 0.4,
            # 2026-10-06: 返航因 Nav2 失败而重试时，是否先做一次受控后退。
            # 2026-10-07: 该行为已由 return_home_retreat_enabled 统一管理；本开关保留仅为
            # 兼容旧配置，固定为 False（两者必须同时为真才会后退）。
            'return_home_failure_retreat_enabled': False,
            # R1b (2026-09-29): escape a blocked-clearance deadlock (front too close to
            # resume, and no backup/spin available) with one bounded reverse.
            'blocked_retreat_after_sec': 2.0,
            'blocked_retreat_min_interval_sec': 8.0,
            'blocked_retreat_max_per_session': 12,
            # A/B (2026-09-29): pull frontier goals into known-free space and validate the
            # approach corridor before dispatching.
            'frontier_pullback_m': 0.0,   # 2026-09-29: 0 = no pull-back.  1.0 m biased every frontier
                                        # goal back into KNOWN space (fired 135x in one run)
                                        # and made exploration hug known areas.
            # 2026-10-09 (operator): 1.2 -> 0.2.  This is the B approach-corridor probe:
            # the manager walks `approach_probe_m` metres from the robot toward the goal
            # (sampling -0.35 / 0 / +0.35 m laterally) and refuses the goal if ANY sampled
            # cell is occupied (>=50).  At 1.2 m x 0.7 m it vetoed almost everything in a
            # cramped area: on 2026-10-09 the map was saved fine at 11:43:28, but the
            # return-home goal was refused 4x in a row
            #   'B approach corridor rejected: obstacle within 0.94 m of the start'
            # and the run ended with 'return-home failed after 3 retries'.
            # NOTE the '0.73 m' printed in that message is NOT the threshold -- it is
            # front_stop_distance(0.65)+blocked_clearance_hysteresis_m(0.08), i.e. the front
            # gate band; the threshold is this probe length.  At 0.2 m the check is shorter
            # than the front gate itself, so it effectively no longer fires and Nav2 plus the
            # front gate take over.  Hot-tunable during a session:
            #   ros2 param set /stemm_cartographer_nav2_manager approach_probe_m 0.2
            # See also: the call site comment claims A/B are exploration-only, but B runs
            # unconditionally, so it also gates return-home.
            'approach_probe_m': 0.2,
            # 2026-10-10 (operator): 黑名单排斥半径 1.5 -> 0.8。
            # 见 cartographer_manager 中同名参数的说明。
            'failed_goal_radius': 1.5,
            'blocked_place_radius': 1.5,
            'return_home_retry_delay_sec': 2.0,
            # 2026-09-28: raised from 90.0 after task-mul2g8y4-002 reported
            # finish_mapping_timeout while the navigation stacks were still being
            # restored (the return-home goal had nowhere to go).  Stage 2 keeps its own
            # 180 s driving budget.
            'mapping_finish_confirm_timeout_sec': 300.0,
                        # 2026-09-29 (user decision): the return-home hard timeout is effectively
            # cancelled -- a far-away robot must not be aborted mid-way; the operator
            # decides when to stop (device restart / kill).  24 h is 'no timeout' here.
            'return_home_hard_timeout_sec': 86400.0,
            'mapping_timeout_cancel_delay_sec': 5.0,
            'return_home_start_clearance': 0.40,
            'home_known_radius': 0.45,
            'front_stop_distance': 0.65,
            # 2026-10-09 (operator): return-home target clearance, deliberately SEPARATE from
            # goal_clearance (0.50).  goal_clearance is shared with the exploration fallback
            # sampler, so lowering it there would also degrade exploration targets.  The home
            # target is the map origin == where the robot started, and requiring 0.50 m there
            # made return-home fail outright when the robot started near an obstacle
            # (2026-10-09 16:06: "R1d target rejected: less than 0.50 m clearance around
            # (0.00, 0.00)" x4 with "front obstacle too close: 0.39m").
            'return_home_goal_clearance': 0.20,
            # How far to look for a stand point that satisfies the above before giving up.
            'return_home_target_search_m': 1.5,
            # 2026-10-13 P9-A：0.50 → 0.35。
            # 实测：0.50 要求目标周围 1.0m 直径净空，而车足印内切半径 0.328m、
            # 实测通道仅 0.72~0.81m —— 物理上给不出 → R1d 连续拒发
            # （60s 窗口 18 次拒发 / 0 次派发）→ 车完全静止。
            # 0.35 仍大于内切半径 0.328，保留安全余量。回退：改回 0.50。
            'goal_clearance': 0.35,
            'front_emergency_distance': 0.45,
            'nav_lifecycle_request_timeout_sec': 1.5,
            'nav_lifecycle_retry_delay_sec': 0.5,
            'exploration_no_progress_timeout_sec': 18.0,
            'exploration_progress_delta_m': 0.12,
            'exploration_hard_timeout_sec': 50.0,
            'exploration_pose_stall_timeout_sec': 7.0,
            'exploration_pose_stall_radius_m': 0.10,
            'exploration_pose_stall_goal_tolerance_m': 0.30,
            'rrt_fallback_wait_sec': 0.0,
            'rrt_projection_radius_m': 1.0,   # reverted: 2.0 projected deeper into known space   # A: project deeper into known-free space
            'clear_costmaps_on_stall': True,
            'exploration_behavior_tree': os.path.join(
                config_dir, 'stemm_cartographer_nav_to_pose.xml'),
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
            # 2026-09-24: raised from 35 s after two mapping sessions failed because
            # the nav2 lifecycle bringup lost a change_state response under load
            # (controller_server/smoother_server 'failed to send response (timeout)').
            # The 14:49 session with the same code came up in ~4 s, so this is a
            # slow-machine allowance, not a functional timeout.
            'timeout_sec': 90.0,
            'cold_start_extension_sec': 30.0,
            'stable_sec': 0.8,
            'request_timeout_sec': 1.5,
            'retry_delay_sec': 0.5,
            'retry_max_delay_sec': 2.0,
            # Do not launch RRT until the normalized Cartographer map
            # can answer a real GetMap request.
            'rrt_map_service': '/stemm_cartographer/rrt_dynamic_map',
            'use_sim_time': use_sim_time,
        }],
    )

    # 2026-10-10: 外层必须用双引号。此前用单引号时，LaunchConfiguration
    # 求值后的值落进单引号内部，产生 'true'.lower() == ... 这种多余外层引号，
    # 求值时报 unterminated string literal。该错误只在条件被真正求值
    # （nav2_ready 事件）时暴露，静态门禁查不出来。
    # 2026-10-10: 照抄仓库既有可用模式（wheeltec_nav2.launch.py 的
    # both_publishers_enabled），只把内部字符串换成双引号：
    #   求值后 -> "true".lower() == "true" and "grid".lower() == "grid"
    # 用单引号会残留最外层引号（unterminated string literal），
    # 而丢掉外面那对引号会让 LaunchConfiguration 变成裸标识符（NameError）。
    rrt_enabled = IfCondition(PythonExpression([
        '"', auto_explore, '".lower() == "true" and "',
        frontier_source, '".lower() == "rrt"',
    ]))
    grid_enabled = IfCondition(PythonExpression([
        '"', auto_explore, '".lower() == "true" and "',
        frontier_source, '".lower() == "grid"',
    ]))

    # 2026-10-10: 脱困控制器开关，与探索源无关。
    escape_enabled = IfCondition(PythonExpression([
        '"', auto_explore, '".lower() == "true" and "',
        escape_controller, '".lower() == "true"',
    ]))

    rrt_nodes = [
        Node(
            package='wheeltec_robot_rrt',
            executable=executable,
            name=executable,
            output='both',
            parameters=[rrt_params, {'use_sim_time': use_sim_time}],
            condition=rrt_enabled,
        )
        for executable in ('local_rrt', 'global_rrt', 'filter', 'assigner')
    ]

    # 2026-10-10: 栅格式前沿点探索（方案切换的另一半）。
    # 输出契约与 RRT 链的 assigner 相同：/assigned_frontier_goal PoseStamped。
    grid_nodes = [
        Node(
            package='stemm_frontier_explorer',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='both',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=grid_enabled,
        )
    ]

    # 2026-10-10: 卡死后外部定步长旋转脱困（本包内节点，不改 Nav2 参数）。
    # 触发与退出都用 Nav2 自己的足印碰撞判定；节点内 enabled 默认 false，
    # 此处 escape_controller 也默认 'false'，两层闩锁都开才会真正动作。
    escape_nodes = [
        Node(
            package='stemm_cartographer_exploration',
            executable='stemm_cartographer_escape_controller',
            name='stemm_escape_controller',
            output='both',
            # 2026-10-13 P12-#1：原先只传 use_sim_time，导致 escape 的
            # direction(默认1.0=固定CCW) 与 escape_flip_* 全部取代码默认值，
            # 换向逻辑永不生效。此处显式传入。
            parameters=[{
                'use_sim_time': use_sim_time,
                # 2026-10-13 P14：启用自动选向。
                # direction 默认 1.0 = 固定 CCW，导致 _pick_direction()（按左右各
                # 10 度探测选代价更低的一侧）从未被调用 —— 实测 4 次 ESCAPE ARMED
                # 全部 rotating CCW，其中 3 次转满 2.10rad 仍无改善
                # （armed_cost=253 -> release_cost=253）。设为 0.0 即启用自动选向。
                # 回退：改回 1.0（固定 CCW）。
                'direction': 0.0,
                'escape_flip_after_failed_budget': True,
                'escape_flip_max_per_spot': 2,
                'escape_flip_spot_radius_m': 0.30,
            }],
            condition=escape_enabled,
        )
    ]

    # 2026-09-16: the LiDAR handover was removed (the resident owner keeps the
    # device and publishes /scan), and the sensor preflight now runs concurrently
    # with the odometry guard instead of behind it.  Cartographer still starts only
    # after BOTH succeeded, so the readiness contract is unchanged.
    _stage_results = {'sensors': None, 'odom': None}

    def _cartographer_stage_if_ready():
        if _stage_results['sensors'] == 0 and _stage_results['odom'] == 0:
            return [
                cartographer,
                occupancy_grid,
                map_service_adapter,
                state_saver,
                tf_guard,
            ]
        return []

    def after_system_preflight(event, _context):
        if event.returncode == 0:
            return [
                sensor_preflight,
                odom_session_guard,
            ]
        return [
            LogInfo(msg='ERROR: System isolation preflight failed.'),
            EmitEvent(event=Shutdown(reason='system preflight failed')),
        ]

    def after_sensor_preflight(event, _context):
        if event.returncode == 0:
            _stage_results['sensors'] = 0
            return _cartographer_stage_if_ready()
        return [
            LogInfo(msg='ERROR: Core sensor preflight failed.'),
            EmitEvent(event=Shutdown(reason='sensor preflight failed')),
        ]

    def after_odom_session_guard(event, _context):
        if event.returncode == 0:
            _stage_results['odom'] = 0
            return _cartographer_stage_if_ready()
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
                TimerAction(period=1.0,
                            actions=rrt_nodes + grid_nodes
                            + escape_nodes),
            ]
        return [
            LogInfo(
                msg='ERROR: Nav2 did not become active; RRT will not start.'),
            EmitEvent(event=Shutdown(reason='Nav2 readiness guard failed')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_explore', default_value='true'),
        # 2026-10-10: 'true' 才启动脱困控制器节点；
        # 节点内 enabled 参数（默认 false）是第二道闩锁。
        DeclareLaunchArgument(
            'escape_controller', default_value='true'),
        DeclareLaunchArgument(
            'frontier_source', default_value='rrt',
            description="frontier goal source: 'rrt' (existing random "
                        "sampling chain) or 'grid' (stemm_frontier_explorer)"),
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
