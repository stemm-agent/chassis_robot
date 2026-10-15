#!/usr/bin/env python3
"""Cartographer-only STEMM manager with recoverable Nav2 readiness checks."""

import json
import math
import os
import random
import time
from collections import deque
from functools import partial
from pathlib import Path
from threading import RLock

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from cartographer_ros_msgs.srv import FinishTrajectory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from stemm_nav2_manager.manager import StemmNav2Manager
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .exploration_watchdog import ExplorationProgressWatchdog
from .lifecycle_retry import LifecycleRetryState


class StemmCartographerNav2Manager(StemmNav2Manager):
    """Keep transient lifecycle service failures from wedging exploration."""

    RETURN_HOME_ORIENTATION_TIMEOUT_CONTEXT = (
        'return_home_orientation_timeout')

    def __init__(self):
        super().__init__()
        # Cancellation is only exposed for a failed return-home sequence.  The
        # outer session runner writes the final terminal state after the inner
        # launch exits and the fixed functional services have been restored.
        self._mapping_cancel_requested = False
        self._mapping_cancel_shutdown_pending = False
        self._mapping_cancel_shutdown_at = 0.0
        self._mapping_cancel_reason = ''
        self._mapping_cancel_detail = ''
        # This durable-in-process latch keeps voice pause semantics
        # stable while Nav2 reports cancellation/recovery transitions.
        self._voice_pause_latched = False
        # Monotonic control intent for external callers of the shared
        # ``start_mapping`` service.  The voice bridge uses this persisted
        # acknowledgement to distinguish a real mini-program resume from a
        # transient Nav2 state update.
        self._mapping_resume_accepted_at_epoch_ms = 0
        self.create_service(
            Trigger,
            '/stemm/cancel_mapping',
            self.cancel_mapping_callback,
        )
        state_home = Path(os.getenv(
            'XDG_STATE_HOME', Path.home() / '.local' / 'state')).expanduser()
        self.session_status_path = str(Path(os.getenv(
            'STEMM_CARTOGRAPHER_SESSION_STATUS_PATH',
            state_home / 'stemm-cartographer-session.json')).expanduser())
        self._session_status_write_error = ''
        self.session_task_path = str(Path(os.getenv(
            'STEMM_CARTOGRAPHER_TASK_PATH',
            state_home / 'stemm-cartographer-task.json')).expanduser())
        # 2026-10-09 (operator): return-home uses its own target clearance.  The home target is
        # the map ORIGIN (where the session started); requiring the exploration-grade 0.50 m disc
        # there made return-home fail outright when the robot had started near an obstacle
        # (measured 16:06: origin rejected 4x with "front obstacle too close: 0.39m").
        # 2026-10-12: 探索目标的连通性闸门。
        # 原选点只验证「目标格已知自由 + 圆盘净空 + 直线阻塞比」，从不检查
        # 「从当前位置到目标是否连通」，于是会把目标派发到被不连续障碍隔断的
        # 已知区域。实测 2026-10-12 14:02：4 个探索目标里 2 个耗满 50s 硬上限
        # （结束时仍剩 4.5m/8.5m），机器人 6 个位置全在原点 0.7m 内。
        # 现加连通性闸门：与机器人不同自由连通域的探索目标直接拒发并拉黑。
        # 无法判定（地图缺失/起点被占/异常）时一律放行，绝不阻断探索。
        self.declare_parameter('exploration_require_connected_route', True)
        # BFS 上限：正常地图约 28 万格；设上限防止异常情况下卡死主循环。
        self.declare_parameter('exploration_reach_max_cells', 400000)
        # 2026-10-12 N-9：探索目标可达性改用 Nav2 权威 costmap 判据。
        # E-1 只用地图占用（data<50）做 BFS，不含膨胀层，导致"地图自由但
        # 被膨胀隔断"的目标被放行（实测 3/3 目标规划失败、各耗满 50s）。
        self.declare_parameter('exploration_costmap_gate', True)
        self.declare_parameter(
            'exploration_costmap_service', '/global_costmap/get_costmap')
        self.declare_parameter('exploration_costmap_fetch_sec', 1.0)
        # 与 SmacPlannerHybrid 同判据：cost >= 253 (INSCRIBED_INFLATED_OBSTACLE)
        # 视为不可通行。
        self.declare_parameter('exploration_costmap_lethal_cost', 253)
        # 2026-10-13 P1：可达域判据是否要求"车物理上能过"。
        # 背景：运行期 /map 是 OccupancyGrid 尺度（-1=未知, 0~64=自由, >=65=占用），
        # 而旧判据 `value < 50` 会把 -1(未知) 判成自由 -> 可达域 = 整张图
        # （实测 598.79 m²，真实仅 21.66 m²），连通域闸门因此从不拒发。
        # True 时改用"已知自由 且 障碍距离 >= 车辆内切半径"判可通行。
        # 置 False 即回到旧行为（逐位等价）。
        self.declare_parameter('exploration_reach_respect_map', True)
        # 2026-10-13 P9-D：探索目标的"最小派发距离"。
        # 依据：实测派发过 dist=0.27m/turn=96°、dist=0.31m/turn=108° 的目标，
        # nav2_rotation_shim_controller 判"朝向差超阈值"→ 先原地旋转对准；
        # 0.3m 走完立刻又派极近目标 → 车原地旋转 60+ 秒
        # （escape trace 实测 stall_hold 累积到 36.3s，位置仅移动 0.47m）。
        # 置 0 关闭该过滤。
        self.declare_parameter('exploration_min_goal_distance_m', 0.6)
        # 2026-10-13 P21：探索目标的"转角门禁"。
        # 需求：把"方向粘性取消"的解锁逻辑泛化到全局采样点派发 ——
        #   未解锁时只放行 |turn| <= max_turn_deg 的目标；
        #   连续"确定不可达"拒发 >= exploration_direction_flip_after(2) 次
        #   才解锁，允许大角度转向。
        # 实测 15:23:45 在未解锁状态下派发了 turn=144° 的目标（车被迫掉头）。
        # 置 enabled=False 或 max_turn_deg=180 即回到旧行为。
        self.declare_parameter('exploration_dispatch_max_turn_deg', 80.0)
        self.declare_parameter('exploration_dispatch_max_turn_enabled', True)
        # 防死锁宽限：车头朝向不对时可能所有候选都超限 → 永久无候选。
        # 转角拦截累计超过本值时自动解锁并放行（等价于"小角度已试够"）。
        self.declare_parameter('exploration_dispatch_max_turn_grace_count', 3)
        # 2026-10-13 P27（方案A）：把上面的"硬拦截"补成"软约束"。
        # 在两条选点路径（fallback 采样器 / RRT 前沿投影）的排序里叠加
        # "相对当前车头的转角"惩罚，让前方候选自然优先，而不是先拦下
        # 再重选（实测重选还是同一个点，白费 2.2 秒）。
        # 转角 180° 的等效代价 = turn_penalty_weight_m（米），
        # 即"宁可多走这么多米也不掉头"。置 0 关闭软约束。
        self.declare_parameter('exploration_turn_penalty_enabled', True)
        # 2026-10-13 P31：2.0 -> 4.5。实测 17:55 轮 weight=2.0 时
        # "后方 2.66m + 惩罚 1.68m = 4.34" 仍优于 "前方 4.5m + 惩罚 0.02m
        # = 4.52"，于是继续选后方。4.5 使同一组变为 6.43 vs 4.52，前方胜出。
        self.declare_parameter('exploration_turn_penalty_weight_m', 4.5)
        # 2026-10-13 P3-1：R1d 拒发的目标点进入"拒发存根"，采样器跳过它们。
        # 背景：R1d（目标净空/后方空间）在派发侧拒发，但采样器侧的候选过滤
        # 用的是另一套判据（净空+直线+连通域），两者反馈断裂，导致同一批候选
        # 被反复选中、反复拒发（实测同一方向连续 10 次拒发）。
        self.declare_parameter('exploration_r1d_reject_ttl_sec', 60.0)
        self.declare_parameter('exploration_r1d_reject_radius_m', 1.2)
        # 2026-10-13 P3-2：连续拒发达阈值后重置方向粘性，并优先转向相对方向。
        self.declare_parameter('exploration_direction_flip_after', 2)
        # 2026-10-13 P4：两个标准距离窗都为空时不再停车，改用脱困式放宽窗。
        # 背景：车停在原点时可达域内所有自由格都 < 1.8m（最小候选窗），
        # 于是候选恒为 0 → holding position → 车不动 → 地图不扩展 → 死锁
        # （实测连续 7 秒无目标）。放宽后改为"可达域内最远点"，强制车移动。
        self.declare_parameter('exploration_fallback_relax_min_m', 0.5)
        self.declare_parameter('exploration_fallback_relax_enabled', True)
        # 2026-10-13 P6 缺陷1：放宽（脱困）窗是否忽略"失败记忆"。
        # 失败记忆半径 failed_goal_radius=1.5m；车停在可达域半径 <1.5m 处时，
        # 它会滤掉全部候选，使放宽窗也恒空 → 死锁。默认忽略，可置 False 回退。
        self.declare_parameter('exploration_fallback_relax_ignore_history', True)
        self.declare_parameter('exploration_direction_flip_hold_rejections', 2)
        self.declare_parameter('exploration_reach_clearance_m', 0.328)
        # 2026-10-13 P6 缺陷2：可达域判据的"占用"阈值（OccupancyGrid 概率百分比）。
        # 运行期 /map：-1=未知, 0~100=占用概率。map_saver trinary 把
        # p>0.65 判占用、p<0.25 判自由、其余判未知（不确定）。
        # 采样器 collect 一直用 value < 30（保守）；可达域原先用 < 65，
        # 把 17.8% 的"不确定"格也算成可通行 → 可达域虚大 70%。
        # 本参数统一两处口径，默认 30；置 65 即回到旧行为。
        # 2026-10-13 P9-B（回退缺陷2）：默认 30 → 65。
        # 实证：30 会把 /map 里 value 30~64 的"不确定格"（实测占 17.8%）当障碍
        # → 障碍距离场被压缩 → 大量空地格 field < 0.328 → 判"车过不去"
        # → 8 个探索目标全被拒 → 车完全静止。65 = 与 map_saver 的"非占用"口径一致。
        self.declare_parameter('exploration_reach_obstacle_cost', 65)
        # 2026-10-13 P31：采样器的"自由格"阈值（OccupancyGrid 概率百分比）。
        # 采样器三处（collect 与 RRT 投影两处）长期硬编码 `value >= 30`，
        # 把 30~64 的"不确定"格当障碍跳过。实测 17:42/17:55 两轮，车所在格
        # 的值恰为 38 → 三个窗口全空 → "no candidate cell" 连续 8 秒 →
        # 车原地空转。65 = 与 map_saver 的"非占用"口径一致。
        # 放宽后的安全性由后续判据把关（净空 / 直线遮挡比 / 可达域连通性）。
        # 置 30 即回到旧行为。
        self.declare_parameter('exploration_sampler_free_cost', 65)
        # 2026-10-12 N-11：兜底采样器也必须过可达性判据。
        # 实测 17:16 轮 8 个失败目标里 5 个在 costmap 上不可达，而它们
        # 全部来自 _pick_fallback_goal —— 该函数只查净空与直线通畅，
        # 从不问"能不能规划过去"，于是绕过了 N-9 闸门、每个耗满 50s 硬上限。
        self.declare_parameter(
            'exploration_fallback_require_reachable', True)
        self.declare_parameter('return_home_goal_clearance', 0.35)
        self.return_home_goal_clearance = max(
            0.0, float(self.get_parameter('return_home_goal_clearance').value))
        self.exploration_require_connected_route = bool(
            self.get_parameter('exploration_require_connected_route').value)
        self.exploration_costmap_gate = bool(
            self.get_parameter('exploration_costmap_gate').value)
        self.exploration_costmap_service = str(
            self.get_parameter('exploration_costmap_service').value)
        self.exploration_costmap_fetch_sec = max(
            0.2, float(self.get_parameter(
                'exploration_costmap_fetch_sec').value))
        self.exploration_costmap_lethal_cost = int(
            self.get_parameter('exploration_costmap_lethal_cost').value)
        self.exploration_reach_respect_map = bool(
            self.get_parameter('exploration_reach_respect_map').value)
        self.exploration_min_goal_distance_m = max(0.0, float(
            self.get_parameter('exploration_min_goal_distance_m').value))
        # 2026-10-13 P21：转角门禁
        self.exploration_dispatch_max_turn_deg = max(0.0, float(
            self.get_parameter('exploration_dispatch_max_turn_deg').value))
        self.exploration_dispatch_max_turn_enabled = bool(
            self.get_parameter('exploration_dispatch_max_turn_enabled').value)
        self.exploration_dispatch_max_turn_grace_count = max(0, int(
            self.get_parameter(
                'exploration_dispatch_max_turn_grace_count').value))
        # 2026-10-13 P27（方案A）：转角软约束
        self.exploration_turn_penalty_enabled = bool(
            self.get_parameter('exploration_turn_penalty_enabled').value)
        self.exploration_turn_penalty_weight_m = max(0.0, float(
            self.get_parameter(
                'exploration_turn_penalty_weight_m').value))
        self.exploration_reach_clearance_m = max(0.0, float(
            self.get_parameter('exploration_reach_clearance_m').value))
        self.exploration_reach_obstacle_cost = max(1, int(
            self.get_parameter('exploration_reach_obstacle_cost').value))
        # 2026-10-13 P31：仅作为兜底缓存；实际取值走 _sampler_free_cost()
        # 运行时读参数，这样 ros2 param set 能立即生效。
        self.exploration_sampler_free_cost = max(1, int(
            self.get_parameter('exploration_sampler_free_cost').value))
        self.exploration_r1d_reject_ttl_sec = max(1.0, float(
            self.get_parameter('exploration_r1d_reject_ttl_sec').value))
        self.exploration_r1d_reject_radius_m = max(0.1, float(
            self.get_parameter('exploration_r1d_reject_radius_m').value))
        self.exploration_direction_flip_after = max(1, int(
            self.get_parameter('exploration_direction_flip_after').value))
        self.exploration_fallback_relax_min_m = max(0.0, float(
            self.get_parameter('exploration_fallback_relax_min_m').value))
        self.exploration_fallback_relax_enabled = bool(
            self.get_parameter('exploration_fallback_relax_enabled').value)
        self.exploration_fallback_relax_ignore_history = bool(
            self.get_parameter('exploration_fallback_relax_ignore_history').value)
        self.exploration_direction_flip_hold_rejections = max(1, int(
            self.get_parameter('exploration_direction_flip_hold_rejections').value))
        self.exploration_fallback_require_reachable = bool(
            self.get_parameter(
                'exploration_fallback_require_reachable').value)
        self._costmap_msg = None
        self._costmap_at = 0.0
        self._costmap_client = None
        self._costmap_future = None
        self.exploration_reach_max_cells = max(
            1000, int(self.get_parameter(
                'exploration_reach_max_cells').value))
        # 2026-10-09 (operator): how far to look for a stand point that satisfies the above.
        self.declare_parameter('return_home_target_search_m', 1.5)
        # 2026-10-10 (operator): 返航派发前核对当前位姿与返航目标是否位于
        # 同一连通自由区。实测出现过 planner 反复 status 6 且误差越来越大，
        # 与其游荡 4 分钟不如一次明确拒绝并给出可行动原因。可现场关闭。
        self.declare_parameter('return_home_require_connected_route', True)
        self.return_home_require_connected_route = bool(
            self.get_parameter('return_home_require_connected_route').value)
        self._home_route_cache = None
        self._explore_reach_cache = None   # 2026-10-12: 探索目标的连通性闸门 的可达域缓存
        self._init_exploration_costmap_client()
        self.return_home_target_search_m = max(
            0.0, float(self.get_parameter('return_home_target_search_m').value))
        self.declare_parameter('nav_lifecycle_request_timeout_sec', 1.5)
        self.declare_parameter('nav_lifecycle_retry_delay_sec', 0.5)
        self.declare_parameter('nav_lifecycle_retry_max_delay_sec', 2.0)
        self._watchdog_trace_at = 0.0   # 2026-10-10 诊断 trace 节流
        self.declare_parameter('exploration_no_progress_timeout_sec', 18.0)
        self.declare_parameter('exploration_progress_delta_m', 0.12)
        # 2026-10-14 D3：进展判据改用“净推进量”而不是“到目标的瞬时距离”。
        # 实测（探针 probe31-1014-133517）：车在原地绕圈 50s（弧长 6.92m、
        # 转角 40rad）时，feedback 距离持续变化，stagnant 恒为 0.0，
        # 18s 看门狗与 7s 姿态停滞全部沉默，只剩 50s 硬上限兜底。
        # 置 False 即回到旧行为（用 feedback 瞬时距离）。
        self.declare_parameter('exploration_progress_path_aware', False)
        # 2026-10-14 D8：独立判据——"车一直在动却没靠近过目标"。
        # 实测（probe31-1014-133517）：50 秒里 stagnant 恒为 0.0，
        # 18s 看门狗与 7s 姿态停滞全部沉默，只剩 50s 硬上限；
        # 而车其实在原地绕圈（弧长 6.92m / 转角 40rad）。
        # 注意：D7 的"净推进折算"默认关闭——它会打断看门狗的滑动窗口，
        # 实测导致缓慢推进被误判（见 d3-test T3/T4）。
        self.declare_parameter('exploration_progress_no_net_timeout', True)
        self.declare_parameter('exploration_no_net_min_travel_m', 0.50)
        # 2026-10-14 D12：判据触发后进入短冷却再武装。
        # manager 的确认逻辑要求"再观察到一次 stalled"才取消
        # （cartographer_manager.py 的 _stall_confirmation 段），
        # 若触发后把锚点清空、重新等满 18s，收益会被 50s 硬上限吃掉
        # （实测 13:51:59 触发 → 13:52:06 仍由硬上限取消）。
        self.declare_parameter('exploration_no_net_rearm_sec', 3.0)
        # 锚点位移阈值：车真实移动超过该值后才把锚点前移，
        # 避免“缓慢绕圈”被误判成进展（默认 0.20 m）。
        self.declare_parameter('exploration_progress_anchor_m', 0.20)
        self.declare_parameter('exploration_hard_timeout_sec', 50.0)
        self.declare_parameter('exploration_pose_stall_timeout_sec', 7.0)
        self.declare_parameter('exploration_pose_stall_radius_m', 0.10)
        self.declare_parameter(
            'exploration_pose_stall_goal_tolerance_m', 0.30)
        # 2026-10-12: 停滞判据原先只看位移，把 Nav2 的原地转向（纯旋转、
        # 位移恒为 0，实测 |wz| 可达 0.61~1.01 rad/s）误判成卡死。
        # 现在"转了一点"也算在动，锚点随之重置。
        self.declare_parameter('exploration_pose_stall_yaw_rad', 0.15)
        self.declare_parameter('rrt_fallback_wait_sec', 0.0)
        self.declare_parameter('rrt_projection_radius_m', 1.0)
        # 2026-10-14 P46-C：单轮投影最多考察多少个候选（<=0 表示不限）。
        # 实测 999 个候选全部逐个投影会饿死 lifecycle 回调，导致误判
        # "Nav2 lifecycle is not active yet" 并让车长时间停住。
        self.declare_parameter('rrt_projection_max_candidates', 200)
        # 2026-10-06: 沿“前沿->小车”方向退让时的步长（米）。
        # 原实现只在以样本为心、半径 rrt_projection_radius_m 的圆内找已知空闲格，
        # 前沿样本位于“已探索/未知”交界，圆内多为未知格或被净空盘否掉，
        # 于是频繁报 no safe known-free projection 并退化为已知区域保守探索。
        self.declare_parameter('rrt_projection_backoff_step_m', 0.10)
        self.declare_parameter('clear_costmaps_on_stall', True)
        self.declare_parameter(
            'exploration_behavior_tree',
            os.path.join(
                get_package_share_directory('stemm_cartographer_exploration'),
                'config', 'stemm_cartographer_nav_to_pose.xml'))
        self.declare_parameter('goal_acceptance_timeout_sec', 3.0)
        self.declare_parameter(
            'cartographer_finish_trajectory_service', '/finish_trajectory')
        self.declare_parameter('cartographer_trajectory_id', 0)
        self.declare_parameter('cartographer_finish_timeout_sec', 10.0)
        # 2026-10-10: 3.0 -> 30.0。finish_trajectory 返回 != 轨迹优化完成；
        # 实测某一轮优化在 finish 受理后 39 秒才开始，3 秒预算必然判失败，
        # 导致最终地图从未落盘（磁盘上仍是上一轮产物）。
        # 2026-10-12: 等不到 finish 后的新栅格时，退回保存缓存地图。
        # 30 -> 45：finish 后的最终优化耗时不确定（实测 1.3s / 6.7s / 39s 都出现过），
        # 给慢优化更多机会走"保存真正最终栅格"的首选路径。
        self.declare_parameter('final_map_wait_timeout_sec', 45.0)
        # 超时兜底：缓存地图合法时照常保存，避免整轮建图成果丢失。
        self.declare_parameter('final_map_save_fallback_on_timeout', True)
        # 2026-10-10: 若等待窗口内地图仍在增长，则顺延截止时间，避免
        # "差一点点就到"时被硬切。
        self.declare_parameter('final_map_grace_after_growth_sec', 6.0)
        self.declare_parameter(
            'cartographer_finish_tf_translation_tolerance_m', 0.01)
        self.declare_parameter(
            'cartographer_finish_tf_rotation_tolerance_rad', 0.01)
        self.declare_parameter('feedback_zero_epsilon_m', 0.01)
        self.declare_parameter('feedback_zero_goal_tolerance_m', 0.30)
        self.declare_parameter('feedback_baseline_settle_sec', 2.0)
        self.declare_parameter('stall_cancel_response_timeout_sec', 2.0)
        self.declare_parameter('stall_cancel_result_timeout_sec', 5.0)
        self.declare_parameter('stall_cancel_max_attempts', 2)
        self.declare_parameter('stall_confirmation_sec', 0.8)
        self.declare_parameter('stall_spin_action_name', '/spin')
        self.declare_parameter('stall_spin_yaw_rad', 1.05)
        self.declare_parameter('stall_spin_escalated_yaw_rad', 1.30)
        self.declare_parameter('stall_spin_time_allowance_sec', 5.0)
        self.declare_parameter('stall_spin_acceptance_timeout_sec', 2.0)
        self.declare_parameter('stall_spin_result_timeout_sec', 6.0)
        self.declare_parameter('stall_spin_cancel_timeout_sec', 3.0)
        self.declare_parameter('costmap_clear_wait_sec', 0.6)
        self.declare_parameter('blocked_recovery_delay_sec', 1.5)
        self.declare_parameter('blocked_recovery_cooldown_sec', 5.0)
        self.declare_parameter('blocked_clearance_hysteresis_m', 0.08)
        self.declare_parameter('blocked_clearance_stable_sec', 0.5)
        self.declare_parameter('recovery_log_throttle_sec', 2.0)
        self.declare_parameter('cancel_zero_guard_period_sec', 0.05)
        # End-mapping has two bounded stages: Cartographer finalization before
        # a home goal, then the actual return-home navigation.
        # R1 (2026-09-29): bounded reverse when a return-home start is boxed in.
        # 2026-10-07: the reverse is gated per lifecycle stage instead of by one shared
        # switch.  Exploration (R1b blocked-clearance, R1c pre-goal) keeps its retreat;
        # return-home (R1 boxed-in, retry pre-retreat) is off by default because the
        # lidar cannot see behind the robot (field collision on 2026-10-07 13:38).
        self.declare_parameter('exploration_retreat_enabled', True)
        # 2026-10-13 P11 方案4：探索专用后退参数组。
        # 此前探索与返航共用 return_home_retreat_*，探索被迫采用返航尺度
        # （0.90m）导致探测深度 probe=0.90+0.25=1.15m，后方这么长的带状区里
        # 任一点被占就判 rear-blocked，实测频繁误判（14:03 轮 4 次里 2 次失败）。
        # 现按 context 分流：探索用本组（更慢更短），返航参数保持不动。
        self.declare_parameter('exploration_retreat_speed_mps', 0.10)
        self.declare_parameter('exploration_retreat_max_distance_m', 0.35)
        self.declare_parameter('exploration_retreat_max_sec', 3.0)
        self.declare_parameter('exploration_retreat_rear_margin_m', 0.25)
        # 2026-10-13 P11 方案1：连续 rear-blocked 后的原地转向（打破三向锁死）。
        # 前进被前向门禁拒、后退被 rear-blocked 拒、自转又 Collision Ahead 失败
        # 导致车在原地反复重试；改为转向换朝向后重试后退。
        self.declare_parameter('exploration_blocked_turn_deg', 90.0)
        self.declare_parameter('exploration_blocked_turn_max_tries', 2)
        self.declare_parameter('exploration_blocked_turn_total_limit_deg', 180.0)
        self.declare_parameter('exploration_blocked_turn_speed_radps', 0.40)
        self.declare_parameter('return_home_retreat_enabled', False)
        self.declare_parameter('return_home_retreat_speed_mps', 0.15)   # R1e
        self.declare_parameter('return_home_retreat_max_distance_m', 0.60)   # R1e
        self.declare_parameter('return_home_retreat_max_sec', 3.0)
        self.declare_parameter('return_home_retreat_rear_margin_m', 0.25)
        self.declare_parameter('return_home_retreat_settle_sec', 0.4)
        # R1b (2026-09-29): escape a clearance deadlock while exploring.
        self.declare_parameter('blocked_retreat_after_sec', 2.0)   # R1e: 2 s of no progress is enough
        self.declare_parameter('blocked_retreat_min_interval_sec', 8.0)   # R1e
        self.declare_parameter('blocked_retreat_max_per_session', 12)
        # 2026-10-06: 返航因 Nav2 规划/控制失败而重试时，是否先做一次受控后退。
        # 原 R1 后退只在“起点太靠近占用格”时触发；实测返航失败常见形态是
        # planner 报 no valid path found（起点被近处障碍夹住），此时也需要后退让出通道。
        self.declare_parameter('return_home_failure_retreat_enabled', True)
        # A/B (2026-09-29)
        self.declare_parameter('frontier_pullback_m', 1.0)
        # 2026-10-10: fallback 采样的距离窗与兜转检测。原为硬编码
        # collect(2.5, 5.5) / collect(1.8, 6.0)。抽成参数以便按探索区域
        # 实际尺寸调整（小区域里 2.5m 下限必然指向区域外）。
        self.declare_parameter('fallback_min_distance_m', 2.5)
        self.declare_parameter('fallback_max_distance_m', 5.5)
        self.declare_parameter('fallback_wide_min_distance_m', 1.8)
        self.declare_parameter('fallback_wide_max_distance_m', 6.0)
        # 连续多少次目标方向反转（>reversal_min_deg）判定为兜转。
        self.declare_parameter('fallback_reversal_min_deg', 120.0)
        self.declare_parameter('fallback_reversal_streak_alert', 3)
        # 2026-10-10: fallback 候选的"方向粘性"权重（弧度）。
        # 排序键 = 角度差(rad) * 本值 + 距离(m) * (1/fallback_max_distance_m)。
        # 越大越坚持原方向；0 = 退化为纯距离优先。
        self.declare_parameter('fallback_heading_weight', 1.0)
        # 2026-10-10 (operator): 探索目标的 yaw 取法。
        #   'tangent' = 保持机器人当前朝向（对前向-only 的 DUBIN 平凡可满足）
        #   'bearing' = 原来的"指向目标"方位角
        # 实测 'bearing' 会让规划器对近处空旷目标连续 'exceeded maximum
        # iterations'（30 秒 7 次），故默认改为 'tangent'。
        #
        # 2026-10-12: 探索目标朝向改为「抵达时的行进方向」。
        # 'tangent' 取的是**派发那一刻**的车头朝向，而那一刻机器人还没转
        # （run-1316 实测 turn 平均 85°、最大 162°），抵达时朝向已变成行进
        # 方向 → 与写入朝向差 109°/147°。Nav2 用 SimpleGoalChecker
        # (yaw=0.25rad≈14°) 把目标朝向当作「抵达时必须满足」的约束，
        # 于是机器人到点后必须再转回去 —— 这是「频繁掉头」的直接来源。
        #   'approach' = 指向目标的方位角（= 抵达时的行进方向，
        #                抵达时天然满足，终点无需再转）—— 现为默认。
        #   'keep'     = 不改写，沿用调用方给的朝向。
        self.declare_parameter('goal_yaw_mode', 'approach')
        self.declare_parameter('approach_probe_m', 1.2)   # R1e
        self._blocked_retreat_last_at = 0.0
        self._blocked_retreat_count = 0
        # 2026-10-13 P11 方案1：连续 rear-blocked 与转向预算
        self._blocked_rear_streak = 0
        self._blocked_turn_count = 0
        self._blocked_turn_total_rad = 0.0
        # 2026-10-13 P21：转角门禁的拦截计数 + 多类型"确定不可达"连续计数。
        # 注意：'r1d' 复用既有 _r1d_consecutive（不另设平行计数，否则不同步）；
        # planner/reach 各一个独立计数。异类互相清零 => 不跨类型求和。
        self._turn_blocked_count = 0
        self._planner_consecutive = 0
        self._reach_consecutive = 0
        self._retreat_used_for_attempt = False
        self.declare_parameter('mapping_finish_confirm_timeout_sec', 90.0)
        self.declare_parameter('return_home_hard_timeout_sec', 86400.0)   # 2026-09-29: no timeout
        # 2026-10-12: 位置容差必须大于 Nav2 的 goal_checker 容差。
        # 原值 0.15 与 launch 侧 goal_checker.xy_goal_tolerance=0.15 完全相同，
        # 零余量：Nav2 判到达后停在 0.18m，而本判据要 ≤0.15m，差 3.6cm 永远
        # 无法满足 —— 实测 2026-10-12 14:02 轮因此刷屏
        # 'return-home is outside the position-only tolerance: 0.186m > 0.150m'
        # 并把 returnHomeRetry 顶到 3/3 上限。现放宽到 0.25（比 Nav2 大 0.10m，
        # 留出停车抖动余量；建图起点的语义只需"回到原点附近"）。
        self.declare_parameter(
            'return_home_xy_completion_tolerance_m', 0.25)
        self.declare_parameter('return_home_orientation_grace_sec', 6.0)
        # 2026-10-10 (operator): 返航完成判据。位置是唯一前提，朝向不再作为
        # 完成前提：Nav2 的 stateful goal checker 要求 XY 与 yaw 同时满足，
        # 而旋转对齐会把位置抖出容差，导致精确到达却一直微调。
        self.declare_parameter('return_home_completion_requires_orientation', False)
        self.declare_parameter('return_home_completion_hold_sec', 1.5)
        self.declare_parameter('return_home_completion_move_epsilon_m', 0.03)
        self.declare_parameter('mapping_timeout_cancel_delay_sec', 5.0)
        self.declare_parameter('manual_takeover_auto_close_enabled', True)
        self.declare_parameter('manual_takeover_auto_close_distance_m', 0.35)
        self.declare_parameter('manual_takeover_auto_close_stable_sec', 12.0)
        self.nav_lifecycle_retry = LifecycleRetryState(
            self.nav_lifecycle_nodes,
            self.get_parameter(
                'nav_lifecycle_request_timeout_sec').value,
            self.get_parameter('nav_lifecycle_retry_delay_sec').value,
            self.get_parameter(
                'nav_lifecycle_retry_max_delay_sec').value,
        )
        # 2026-10-14 P46-B：容忍瞬时缺失所需的记账（见 _nav2_lifecycle_callback_impl）。
        # 实测 manager 忙时 GetState 回调会被饿死并误判为超时，若零容忍就会
        # 直接把 nav2_ready 打成 False 并让车停住。
        self.nav_lifecycle_missing_streak = {}
        self.nav_lifecycle_last_active = {}
        self.nav_lifecycle_missing_tolerance = 3
        self.get_logger().info(
            'Cartographer lifecycle retry enabled: '
            f'timeout={self.nav_lifecycle_retry.request_timeout_sec:.1f}s, '
            f'retry={self.nav_lifecycle_retry.retry_delay_sec:.1f}-'
            f'{self.nav_lifecycle_retry.retry_max_delay_sec:.1f}s')

        no_progress_timeout = max(
            1.0,
            float(self.get_parameter(
                'exploration_no_progress_timeout_sec').value),
        )
        progress_delta = max(
            0.01,
            float(self.get_parameter('exploration_progress_delta_m').value),
        )
        hard_timeout = max(
            no_progress_timeout,
            float(self.get_parameter('exploration_hard_timeout_sec').value),
        )
        # 2026-10-14 D3：净推进判据的开关与锚点状态（见 _path_aware_progress_distance）。
        self.exploration_progress_path_aware = bool(
            self.get_parameter('exploration_progress_path_aware').value)
        self.exploration_progress_anchor_m = max(
            0.05,
            float(self.get_parameter(
                'exploration_progress_anchor_m').value),
        )
        self._progress_path_anchor = None
        self.exploration_progress_no_net_timeout = bool(
            self.get_parameter('exploration_progress_no_net_timeout').value)
        self.exploration_no_net_min_travel_m = max(
            0.0, float(self.get_parameter(
                'exploration_no_net_min_travel_m').value))
        self._no_net_anchor = None
        self.exploration_no_net_rearm_sec = max(
            0.5, float(self.get_parameter(
                'exploration_no_net_rearm_sec').value))
        self._no_net_cooldown_until = 0.0
        self.exploration_pose_stall_timeout_sec = max(
            1.0,
            float(self.get_parameter(
                'exploration_pose_stall_timeout_sec').value),
        )
        self.exploration_pose_stall_radius_m = max(
            0.02,
            float(self.get_parameter(
                'exploration_pose_stall_radius_m').value),
        )
        self.exploration_pose_stall_goal_tolerance_m = max(
            self.exploration_pose_stall_radius_m,
            float(self.get_parameter(
                'exploration_pose_stall_goal_tolerance_m').value),
        )
        self.exploration_pose_stall_yaw_rad = max(
            0.0,
            float(self.get_parameter(
                'exploration_pose_stall_yaw_rad').value),
        )
        self.rrt_fallback_wait_sec = max(
            0.0, float(self.get_parameter('rrt_fallback_wait_sec').value))
        self.rrt_projection_max_candidates = int(
            self.get_parameter('rrt_projection_max_candidates').value)
        self.rrt_projection_radius_m = max(
            self.goal_clearance,
            float(self.get_parameter('rrt_projection_radius_m').value),
        )
        self.rrt_projection_backoff_step_m = max(
            self.last_map_resolution_floor(),
            float(self.get_parameter('rrt_projection_backoff_step_m').value),
        )
        self.clear_costmaps_on_stall = bool(
            self.get_parameter('clear_costmaps_on_stall').value)
        self.exploration_behavior_tree = str(
            self.get_parameter('exploration_behavior_tree').value)
        self.goal_acceptance_timeout_sec = max(
            0.5, float(self.get_parameter(
                'goal_acceptance_timeout_sec').value))
        self.cartographer_finish_trajectory_service = str(
            self.get_parameter(
                'cartographer_finish_trajectory_service').value)
        self.cartographer_trajectory_id = int(self.get_parameter(
            'cartographer_trajectory_id').value)
        self.cartographer_finish_timeout_sec = max(
            1.0, float(self.get_parameter(
                'cartographer_finish_timeout_sec').value))
        self.final_map_wait_timeout_sec = max(
            0.0, float(self.get_parameter(
                'final_map_wait_timeout_sec').value))
        self.final_map_grace_after_growth_sec = max(
            0.0, float(self.get_parameter(
                'final_map_grace_after_growth_sec').value))
        self.final_map_save_fallback_on_timeout = bool(
            self.get_parameter(
                'final_map_save_fallback_on_timeout').value)
        self.cartographer_finish_tf_translation_tolerance_m = max(
            0.0, float(self.get_parameter(
                'cartographer_finish_tf_translation_tolerance_m').value))
        self.cartographer_finish_tf_rotation_tolerance_rad = max(
            0.0, float(self.get_parameter(
                'cartographer_finish_tf_rotation_tolerance_rad').value))
        self.feedback_zero_epsilon_m = max(
            0.0, float(self.get_parameter(
                'feedback_zero_epsilon_m').value))
        self.feedback_zero_goal_tolerance_m = max(
            self.feedback_zero_epsilon_m,
            float(self.get_parameter(
                'feedback_zero_goal_tolerance_m').value),
        )
        self.feedback_baseline_settle_sec = max(
            0.0, float(self.get_parameter(
                'feedback_baseline_settle_sec').value))
        self.stall_cancel_response_timeout_sec = max(
            0.5, float(self.get_parameter(
                'stall_cancel_response_timeout_sec').value))
        self.stall_cancel_result_timeout_sec = max(
            self.stall_cancel_response_timeout_sec,
            float(self.get_parameter(
                'stall_cancel_result_timeout_sec').value))
        self.stall_cancel_max_attempts = max(
            1, int(self.get_parameter('stall_cancel_max_attempts').value))
        self.stall_confirmation_sec = max(
            0.0, float(self.get_parameter('stall_confirmation_sec').value))
        self.stall_spin_action_name = str(
            self.get_parameter('stall_spin_action_name').value)
        self.stall_spin_yaw_rad = min(
            math.pi, max(0.20, float(self.get_parameter(
                'stall_spin_yaw_rad').value)))
        self.stall_spin_escalated_yaw_rad = min(
            math.pi,
            max(self.stall_spin_yaw_rad, float(self.get_parameter(
                'stall_spin_escalated_yaw_rad').value)),
        )
        self.stall_spin_time_allowance_sec = max(
            1.0, float(self.get_parameter(
                'stall_spin_time_allowance_sec').value))
        self.stall_spin_acceptance_timeout_sec = max(
            0.5, float(self.get_parameter(
                'stall_spin_acceptance_timeout_sec').value))
        self.stall_spin_result_timeout_sec = max(
            self.stall_spin_time_allowance_sec,
            float(self.get_parameter(
                'stall_spin_result_timeout_sec').value),
        )
        self.stall_spin_cancel_timeout_sec = max(
            0.5, float(self.get_parameter(
                'stall_spin_cancel_timeout_sec').value))
        self.costmap_clear_wait_sec = max(
            0.0, float(self.get_parameter('costmap_clear_wait_sec').value))
        self.blocked_recovery_delay_sec = max(
            0.0, float(self.get_parameter(
                'blocked_recovery_delay_sec').value))
        self.blocked_recovery_cooldown_sec = max(
            0.0, float(self.get_parameter(
                'blocked_recovery_cooldown_sec').value))
        self.blocked_clearance_hysteresis_m = max(
            0.0, float(self.get_parameter(
                'blocked_clearance_hysteresis_m').value))
        self.blocked_clearance_stable_sec = max(
            0.0, float(self.get_parameter(
                'blocked_clearance_stable_sec').value))
        self.recovery_log_throttle_sec = max(
            0.2, float(self.get_parameter(
                'recovery_log_throttle_sec').value))
        self.cancel_zero_guard_period_sec = max(
            0.02, float(self.get_parameter(
                'cancel_zero_guard_period_sec').value))
        # 2026-10-10: 兜转埋点用到的六个参数必须赋成实例属性。
        # 事故教训——只 declare_parameter 不赋值，_pick_fallback_goal 会在
        # 读取 self.fallback_min_distance_m 时抛 AttributeError，导致探索
        # 回调每拍崩溃、机器人停住不动。
        self.fallback_min_distance_m = max(
            0.0, float(self.get_parameter(
                'fallback_min_distance_m').value))
        self.fallback_max_distance_m = max(
            self.fallback_min_distance_m, float(self.get_parameter(
                'fallback_max_distance_m').value))
        self.fallback_wide_min_distance_m = max(
            0.0, float(self.get_parameter(
                'fallback_wide_min_distance_m').value))
        self.fallback_wide_max_distance_m = max(
            self.fallback_wide_min_distance_m, float(self.get_parameter(
                'fallback_wide_max_distance_m').value))
        self.fallback_reversal_min_deg = max(
            0.0, float(self.get_parameter(
                'fallback_reversal_min_deg').value))
        self.fallback_reversal_streak_alert = max(
            0, int(self.get_parameter(
                'fallback_reversal_streak_alert').value))
        self.fallback_heading_weight = max(
            0.0, float(self.get_parameter(
                'fallback_heading_weight').value))
        self.goal_yaw_mode = str(
            self.get_parameter('goal_yaw_mode').value).strip().lower()
        self.mapping_finish_confirm_timeout_sec = max(
            15.0, float(self.get_parameter(
                'mapping_finish_confirm_timeout_sec').value))
        self.return_home_hard_timeout_sec = max(
            30.0, float(self.get_parameter(
                'return_home_hard_timeout_sec').value))
        self.return_home_xy_completion_tolerance_m = max(
            0.01, float(self.get_parameter(
                'return_home_xy_completion_tolerance_m').value))
        self.return_home_completion_requires_orientation = bool(
            self.get_parameter(
                'return_home_completion_requires_orientation').value)
        self.return_home_completion_hold_sec = max(
            0.0, float(self.get_parameter(
                'return_home_completion_hold_sec').value))
        self.return_home_completion_move_epsilon_m = max(
            0.0, float(self.get_parameter(
                'return_home_completion_move_epsilon_m').value))
        self.return_home_orientation_grace_sec = max(
            0.0, float(self.get_parameter(
                'return_home_orientation_grace_sec').value))
        self.mapping_timeout_cancel_delay_sec = max(
            2.0, float(self.get_parameter(
                'mapping_timeout_cancel_delay_sec').value))
        self.manual_takeover_auto_close_enabled = bool(
            self.get_parameter('manual_takeover_auto_close_enabled').value)
        self.manual_takeover_auto_close_distance_m = max(
            0.05, float(self.get_parameter(
                'manual_takeover_auto_close_distance_m').value))
        self.manual_takeover_auto_close_stable_sec = max(
            1.0, float(self.get_parameter(
                'manual_takeover_auto_close_stable_sec').value))
        self._manual_recovery_stable_since = 0.0
        self._reset_mapping_exit_timeout()
        self.exploration_watchdog = ExplorationProgressWatchdog(
            no_progress_timeout_sec=no_progress_timeout,
            min_progress_m=progress_delta,
            hard_timeout_sec=hard_timeout,
        )
        self._goal_kind_context = 'manual'
        self._pending_goal_kind = None
        self._active_goal_kind = None
        self._goal_generation_counter = 0
        self._pending_goal_generation = None
        self._active_goal_generation = None
        self._return_home_xy_grace_started_at = None
        self._return_home_xy_grace_generation = None
        self._return_home_xy_cancel_handle = None
        self._return_home_xy_cancel_generation = None
        self._return_home_xy_cancel_result_future = None
        self._return_home_xy_cancel_distance_m = None
        self._return_home_xy_finalizing = False
        self._last_return_home_xy_wait_log = 0.0
        # 2026-10-10: 位置容差内的累计时长与上一拍距离（判停稳用）。
        self._return_home_xy_accum_sec = 0.0
        self._return_home_xy_accum_at = 0.0
        self._return_home_xy_last_dist = None
        self._latest_feedback_distance = None
        self._distance_source = None
        self._latest_recovery_count = 0
        self._pose_stall_generation = None
        self._pose_stall_anchor_x = None
        self._pose_stall_anchor_y = None
        self._pose_stall_anchor_yaw = None
        self._pose_stall_anchor_at = 0.0
        self._feedback_state_lock = RLock()
        self._feedback_baseline_generation = None
        self._feedback_baseline_deadline = 0.0
        self._feedback_baseline_locked = True
        self._stall_cancel_requested = False
        self._recovery_after_cancel = False
        self._cancel_response_future = None
        self._cancel_started_at = 0.0
        self._cancel_last_attempt_at = 0.0
        self._cancel_attempts = 0
        self._cancel_acknowledged = False
        self._cancel_timeout_reported = False
        self._cancel_context = 'navigation_goal'
        self._cancel_on_accept_context = None
        self._last_cancel_zero_time = 0.0
        self._goal_send_started_at = 0.0
        self._goal_acceptance_timed_out = False
        self._timed_out_goal_future = None
        self._goal_transport_fault = False
        self._result_retry_attempted = False
        self._last_progress_log = 0.0
        self._stall_confirmation_started_at = None
        self._rrt_wait_started_at = None
        self._recovery_intent = None
        self._blocked_episode_started_at = None
        self._blocked_episode_attempted = False
        self._blocked_clear_since = None
        self._next_blocked_recovery_at = 0.0
        self._recovery_log_times = {}
        self._spin_phase = 'idle'
        self._spin_generation_counter = 0
        self._spin_generation = None
        self._spin_context = None
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None
        self._spin_cancel_future = None
        self._spin_cancel_requested = False
        self._spin_cancel_acknowledged = False
        self._spin_phase_started_at = 0.0
        self._spin_cancel_started_at = 0.0
        self._spin_cancel_on_accept_reason = None
        self._spin_cancel_reason = None
        self._spin_transport_fault = False
        self._cartographer_spin_active = False
        # Spin collision checking reads the local costmap.  Never clear it
        # immediately before rotating: its temporary empty window could make
        # obstacles appear absent.  A best-effort global clear is sufficient
        # to unblock the next exploration plan while local collision data
        # remains intact throughout the recovery.
        self._costmap_clear_clients = {
            'global': self.create_client(
                ClearEntireCostmap,
                '/global_costmap/clear_entirely_global_costmap'),
        }
        self._costmap_clear_futures = []
        self.spin_client = ActionClient(
            self, Spin, self.stall_spin_action_name)
        self._cancel_zero_guard_timer = self.create_timer(
            self.cancel_zero_guard_period_sec,
            self._cancel_zero_guard_callback,
        )
        self.cartographer_finish_client = self.create_client(
            FinishTrajectory, self.cartographer_finish_trajectory_service)
        self._cartographer_finish_future = None
        self._cartographer_finish_started_at = 0.0
        self._cartographer_trajectory_finished = False
        self._map_revision = 0
        self._final_map_finish_revision = None
        self._final_map_wait_started_at = 0.0
        self._final_map_save_attempted = False
        self._final_map_saved = False
        self._final_map_saved_at = ''
        self._final_map_save_error = ''
        # 2026-10-12: 等不到 finish 后的新栅格时，退回保存缓存地图：是否走了超时兜底保存（供状态文件追溯）。
        self._final_map_saved_fallback = False
        # 2026-10-10: 等待最终栅格的顺延截止时间与 revision 观测点。
        self._final_map_wait_deadline = 0.0
        self._final_map_wait_revision = 0
        self._cartographer_return_tf_anchor = None
        self._cartographer_return_tf_stable_since = 0.0
        self._last_cartographer_tf_motion_log = 0.0
        self._frozen_return_map_to_odom = None
        self._frozen_return_tf_broadcaster = TransformBroadcaster(self)
        self._frozen_return_tf_timer = self.create_timer(
            0.05, self._publish_frozen_return_tf)
        self.get_logger().info(
            'Cartographer exploration watchdog enabled: '
            f'no_net_progress={no_progress_timeout:.1f}s, '
            f'delta={progress_delta:.2f}m, hard_limit={hard_timeout:.1f}s, '
            f'pose_stall={self.exploration_pose_stall_timeout_sec:.1f}s/'
            f'{self.exploration_pose_stall_radius_m:.2f}m, '
            f'RRT fallback wait={self.rrt_fallback_wait_sec:.1f}s, '
            f'Nav2 spin={self.stall_spin_yaw_rad:.2f}-'
            f'{self.stall_spin_escalated_yaw_rad:.2f}rad')

    def map_callback(self, msg):
        super().map_callback(msg)
        self._map_revision += 1

    def _reset_final_map_save_state(self):
        self._final_map_finish_revision = None
        self._final_map_wait_started_at = 0.0
        self._final_map_save_attempted = False
        self._final_map_saved = False
        self._final_map_saved_at = ''
        self._final_map_save_error = ''
        self._final_map_saved_fallback = False

    def _warn_lifecycle_failure(self, node_name, message, now):
        if self.nav_lifecycle_retry.should_warn(node_name, now):
            failures = self.nav_lifecycle_retry.failure_count[node_name]
            self.get_logger().warn(
                f'Nav2 lifecycle check for {node_name} failed '
                f'({failures} consecutive): {message}; retrying')

    def _retire_pending_request(self, node_name, client, future):
        try:
            client.remove_pending_request(future)
        except (KeyError, RuntimeError, ValueError):
            pass
        try:
            future.cancel()
        except Exception:  # noqa: BLE001
            pass
        self.nav_lifecycle_futures[node_name] = None

    def _record_lifecycle_failure(self, node_name, message, now):
        self.nav_lifecycle_states[node_name] = None
        delay = self.nav_lifecycle_retry.mark_failure(node_name, now)
        self._warn_lifecycle_failure(
            node_name, f'{message}; next attempt in {delay:.1f}s', now)

    def _nav2_lifecycle_callback_impl(self):
        now = time.monotonic()
        for node_name, client in self.nav_lifecycle_clients.items():
            future = self.nav_lifecycle_futures.get(node_name)
            if future is not None:
                if future.done():
                    self.nav_lifecycle_futures[node_name] = None
                    try:
                        result = future.result()
                        if result is None:
                            raise RuntimeError('empty GetState response')
                        self.nav_lifecycle_states[node_name] = (
                            result.current_state.id)
                        self.nav_lifecycle_retry.mark_success(node_name, now)
                    except Exception as exc:  # noqa: BLE001
                        self._record_lifecycle_failure(
                            node_name, f'GetState response error: {exc}', now)
                elif self.nav_lifecycle_retry.request_expired(
                        node_name, now):
                    elapsed = (
                        now - self.nav_lifecycle_retry.started_at[node_name])
                    self._retire_pending_request(node_name, client, future)
                    self._record_lifecycle_failure(
                        node_name,
                        f'GetState timed out after {elapsed:.1f}s',
                        now,
                    )
                else:
                    continue

            if not self.nav_lifecycle_retry.can_retry(node_name, now):
                continue
            if not client.service_is_ready():
                self._record_lifecycle_failure(
                    node_name, 'GetState service is unavailable', now)
                continue
            try:
                self.nav_lifecycle_futures[node_name] = client.call_async(
                    GetState.Request())
                self.nav_lifecycle_retry.mark_started(node_name, now)
            except Exception as exc:  # noqa: BLE001
                self.nav_lifecycle_futures[node_name] = None
                self._record_lifecycle_failure(
                    node_name, f'could not send GetState request: {exc}', now)

        # 2026-10-14 P46-B：容忍瞬时缺失。
        # 实测 manager 忙于 RRT 投影（单轮 999 个候选）时，GetState 的响应
        # 回调会被单线程 executor 饿死，10.7s 后才被判定"超时"，于是
        # nav_lifecycle_states 被置 None；原来的 all(...) 零容忍判据立刻让
        # nav2_ready=False -> waiting_for_nav2 -> 不发目标 -> 车停
        # （实测一次停 228s），而 Nav2 实际是 active[3]、get_state 服务也都在。
        # 改法：状态拿不到时沿用上次已知状态，只有连续缺失达到容忍次数
        # 才真正判为不就绪。默认 3 次；置 1 即回到旧的零容忍行为。
        ready = True
        for node_name in self.nav_lifecycle_nodes:
            state = self.nav_lifecycle_states.get(node_name)
            if state is None:
                streak = self.nav_lifecycle_missing_streak.get(node_name, 0) + 1
                self.nav_lifecycle_missing_streak[node_name] = streak
                if (streak < self.nav_lifecycle_missing_tolerance
                        and self.nav_lifecycle_last_active.get(node_name)):
                    continue
                ready = False
                break
            self.nav_lifecycle_missing_streak[node_name] = 0
            if state == State.PRIMARY_STATE_ACTIVE:
                self.nav_lifecycle_last_active[node_name] = True
            else:
                self.nav_lifecycle_last_active[node_name] = False
                ready = False
                break
        self.nav2_ready = ready

    def _run_with_goal_kind(self, kind, callback, *args, **kwargs):
        previous = self._goal_kind_context
        self._goal_kind_context = kind
        try:
            return callback(*args, **kwargs)
        finally:
            self._goal_kind_context = previous

    def _is_exploration_goal(self, kind=None):
        return (kind or self._active_goal_kind or '').startswith(
            'exploration_')

    def scan_callback(self, msg):
        previous_count = self.front_blocked_count
        super().scan_callback(msg)

        # The inherited callback assigns count=3 on every emergency frame,
        # so a continuous close obstacle can never reach its >=5 trigger.
        # Confirm against the narrow centre sector for two consecutive frames;
        # this avoids treating a side-wall return as a forward emergency.
        emergency_range = self.front_center_min_range
        if (emergency_range is not None
                and emergency_range < self.front_emergency_distance):
            self.front_blocked_count = min(
                5, max(self.front_blocked_count, previous_count + 3))
            self.front_blocked = self.front_blocked_count >= 5

        self._update_blocked_episode(time.monotonic())

    def _log_recovery_event(self, key, message, error=False):
        now = time.monotonic()
        last_time = self._recovery_log_times.get(key)
        if (last_time is not None
                and now - last_time < self.recovery_log_throttle_sec):
            return
        self._recovery_log_times[key] = now
        if len(self._recovery_log_times) > 64:
            cutoff = now - max(10.0, 5.0 * self.recovery_log_throttle_sec)
            self._recovery_log_times = {
                saved_key: saved_time
                for saved_key, saved_time in self._recovery_log_times.items()
                if saved_time >= cutoff
            }
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().warn(message)

    def _clear_recovery_intent(self, clear_episode=False):
        self._recovery_intent = None
        self._blocked_clear_since = None
        if clear_episode:
            self._blocked_episode_started_at = None
            self._blocked_episode_attempted = False

    def _record_escape_ineffective_spot(self):
        """P12-#2：把"escape 转满预算仍无改善"的位置写成卡点。

        复用基类的 blocked_places（manager.py 已有写入语义：
        同点 0.50m 内则更新时间戳，否则追加）。写入后 goal_recently_failed()
        会在 failed_goal_ttl(90s) 内不再往该处派目标，避免
        "同一点反复成为卡点"（实测 (3.26,4.18)/(3.82,3.44) 跨轮重复）。
        """
        try:
            pose = self.update_pose()
            if pose is None:
                return False
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            now = time.monotonic()
            placed = list(getattr(self, 'blocked_places', []) or [])
            replaced = False
            for index, (bx, by, _t) in enumerate(placed):
                if math.hypot(x - bx, y - by) < 0.50:
                    placed[index] = (x, y, now)
                    replaced = True
                    break
            if not replaced:
                placed.append((x, y, now))
            self.blocked_places = placed
            self._p1_safe_log(
                'warn',
                'P12 stuck-spot recorded: escape rotated with no footprint-cost '
                'gain at (%.2f, %.2f); blocking it for %.0fs so exploration '
                'stops re-dispatching into the same deadlock (blocked_places=%d)'
                % (x, y, float(getattr(self, 'failed_goal_ttl', 90.0)),
                   len(placed)))
            return True
        except Exception:  # noqa: BLE001
            return False

    def _goal_turn_from_current(self, pose):
        """P21：目标相对"当前朝向"的转角（弧度 0~pi）。取不到位姿返回 None。"""
        try:
            cur = self.update_pose()
            if cur is None or pose is None:
                return None
            dx = pose.pose.position.x - cur.pose.position.x
            dy = pose.pose.position.y - cur.pose.position.y
            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                return 0.0
            bearing = math.atan2(dy, dx)
            q = cur.pose.orientation
            cur_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return abs(self._normalize_angle(bearing - cur_yaw))
        except Exception:  # noqa: BLE001
            return None

    def _sampler_free_cost(self):
        """P31：运行时读采样器的"自由格"阈值，使 ros2 param set 热调生效。

        采样器三处（collect / RRT 投影两处）原先硬编码 `value >= 30`，
        会把 OccupancyGrid 里 30~64 的"不确定"格当障碍跳过；车恰好停在这类
        格子上时（实测 robot_cell_value=38）就会"三个窗口全空"。
        默认 65 = 与 map_saver 的"非占用"口径一致。
        """
        try:
            return max(1, int(
                self.get_parameter('exploration_sampler_free_cost').value))
        except Exception:  # noqa: BLE001
            return int(getattr(self, 'exploration_sampler_free_cost', 65))

    def _turn_penalty_weight(self):
        """P31：运行时读转角软约束权重（米），使热调生效。"""
        try:
            return max(0.0, float(self.get_parameter(
                'exploration_turn_penalty_weight_m').value))
        except Exception:  # noqa: BLE001
            return float(getattr(
                self, 'exploration_turn_penalty_weight_m', 4.5))

    def _turn_penalty_on(self):
        """P31：运行时读转角软约束开关，使热调生效。"""
        try:
            return bool(self.get_parameter(
                'exploration_turn_penalty_enabled').value)
        except Exception:  # noqa: BLE001
            return bool(getattr(
                self, 'exploration_turn_penalty_enabled', True))

    def _turn_from_pose_to(self, cur, x, y):
        """P27：从给定位姿的车头朝向到 (x, y) 的转角（弧度 0~pi）。

        与 _goal_turn_from_current 同算法，但允许调用方传入已取好的位姿，
        避免在排序键里对每个候选重复调用 update_pose()。
        """
        try:
            if cur is None:
                return None
            dx = float(x) - cur.pose.position.x
            dy = float(y) - cur.pose.position.y
            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                return 0.0
            bearing = math.atan2(dy, dx)
            q = cur.pose.orientation
            cur_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return abs(self._normalize_angle(bearing - cur_yaw))
        except Exception:  # noqa: BLE001
            return None

    def _turn_penalty_m(self, cur, x, y):
        """P27（方案A）：把"相对车头的转角"折算成等效距离惩罚（米）。

        turn=180° 时返回 exploration_turn_penalty_weight_m（默认 2.0m）。
        取不到位姿或开关关闭时返回 0.0（等价于关闭软约束）。
        """
        if not self._turn_penalty_on():
            return 0.0
        weight = self._turn_penalty_weight()
        if weight <= 0.0:
            return 0.0
        turn = self._turn_from_pose_to(cur, x, y)
        if turn is None:
            return 0.0
        return weight * (turn / math.pi)

    def _unlock_direction(self, reason, streak):
        """P21：进入 flip 模式（清方向粘性 + 丢弃 A 方案缓存队列 + 解锁转角门禁）。

        原逻辑内联在 _r1d_reject_note 里；P21 抽出以便 r1d/planner/reach
        三条"确定不可达"路径共用。
        """
        prev = getattr(self, 'last_explore_heading', None)
        self._explore_direction_flip = True
        self.last_explore_heading = None
        # 2026-10-13 P13：丢弃 A 方案缓存的候选队列（否则翻转被过期队列架空）。
        try:
            self._fallback_candidate_order = []
            self._fallback_candidate_pose = (0.0, 0.0)
            self._prevalidate_index = 0
            self._prevalidate_reset()
        except Exception:  # noqa: BLE001 - 清缓存失败也不能阻断派发
            pass
        # 显示名映射：既有测试断言 'after N consecutive R1d rejections'（大写 R1d）
        _label = 'R1d' if reason == 'r1d' else reason
        self.get_logger().warn(
            'direction stickiness reset after %d consecutive %s '
            'rejections (was heading=%s); fallback will now prefer '
            'the OPPOSITE direction and the +/-%.0fdeg dispatch turn '
            'gate is unlocked'
            % (streak, _label,
               'None' if prev is None else '%.0fdeg' % math.degrees(prev),
               float(getattr(self,
                             'exploration_dispatch_max_turn_deg', 80.0))))

    def _note_unreachable_rejection(self, reason):
        """P21（丙）：把"确定不可达"的拒发统一计入方向解锁计数。

        reason: 'r1d'（净空不足）/ 'planner'（规划器无路径）/ 'reach'（不同自由域）。
        计数来源（不设平行计数，避免与既有状态不同步）：
            'r1d'     -> 读 self._r1d_consecutive（已由 _r1d_reject_note 递增）
            'planner' -> self._planner_consecutive
            'reach'   -> self._reach_consecutive
        规则：**异类互相清零**（不跨类型求和）→ 阈值仍是"连续
              exploration_direction_flip_after 次"，扩大的是"什么算一次
              拒发"的口径，而非放松阈值。
        """
        try:
            if reason == 'r1d':
                streak = int(getattr(self, '_r1d_consecutive', 0))
                self._planner_consecutive = 0
                self._reach_consecutive = 0
            else:
                attr = '_%s_consecutive' % reason
                streak = int(getattr(self, attr, 0)) + 1
                setattr(self, attr, streak)
                # 异类清零：含 r1d（保证每类都要"连续"达标）
                self._r1d_consecutive = 0
                for other in ('planner', 'reach'):
                    if other != reason:
                        setattr(self, '_%s_consecutive' % other, 0)
            threshold = int(getattr(
                self, 'exploration_direction_flip_after', 2))
            if streak >= threshold and not getattr(
                    self, '_explore_direction_flip', False):
                self._unlock_direction(reason, streak)
        except Exception:  # noqa: BLE001 - 状态绝不能阻断派发
            return

    def _prevalidate_remember(self, x, y):
        """覆盖基类：A 方案预验证否决候选时同步推进方向解锁计数（P21 丙）。

        基类 _prevalidate_poll 在 verdict=='rejected' 时调用本方法。
        """
        try:
            super()._prevalidate_remember(x, y)
        except Exception:  # noqa: BLE001
            pass
        self._note_unreachable_rejection('planner')

    def _normalize_angle(self, angle):
        """把角度归一化到 (-pi, pi]。P11 方案1 转向用。"""
        return math.remainder(float(angle), 2.0 * math.pi)

    def _blocked_turn_direction(self, pose):
        """P11 方案1：选一个让车头"离开上次失败目标方位"的转向方向。

        目的：不再往同一个（不可行的）扇区做无效派发。做法是把车头转向
        "与上次目标方位夹角 >= 90 度"的那一侧，左右转里取转角较小者。
        返回 +1（逆时针/左转）或 -1（顺时针/右转）。
        """
        try:
            target_bearing = getattr(self, 'last_explore_heading', None)
            if target_bearing is None:
                return 1
            q = pose.pose.orientation
            cur_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            rel = self._normalize_angle(float(target_bearing) - cur_yaw)
            if rel == 0.0:
                return 1
            # 目标在左侧（rel>0）时左转正好冲向目标，应改右转，使车头背离目标
            return -1 if rel > 0.0 else 1
        except Exception:  # noqa: BLE001
            return 1

    def _rotate_in_place(self, direction, angle_rad, speed_radps):
        """P11 方案1：原地开环转向（直接发 cmd_vel）。返回实际旋转弧度。

        与 escape_controller 同机制（直接发 Twist），但有明确角度上限，
        不依赖 Nav2 Spin（实测 Spin 紧贴障碍时必然 Collision Ahead 失败）。
        """
        if angle_rad <= 0.0 or speed_radps <= 0.0:
            return 0.0
        twist = Twist()
        twist.angular.z = float(direction) * abs(speed_radps)
        start = time.monotonic()
        rotated = 0.0
        last = start
        max_sec = angle_rad / abs(speed_radps) + 2.0
        try:
            while rotated < angle_rad:
                now = time.monotonic()
                self.cmd_vel_pub.publish(twist)
                dt = max(0.0, now - last)
                rotated += abs(speed_radps) * dt
                last = now
                if (now - start) >= max_sec:
                    break
                time.sleep(0.05)
        finally:
            self.publish_zero_velocity()
            time.sleep(0.3)
        return rotated

    def _turn_after_rear_blocked(self, now):
        """P11 方案1：连续 rear-blocked 后转向，打破三向锁死。

        触发：连续 rear-blocked >= 2 次（见 _maybe_retreat_when_blocked）。
        护栏：最多 exploration_blocked_turn_max_tries 次，累计不超过
              exploration_blocked_turn_total_limit_deg，防止变成新的自转循环。
        """
        max_tries = int(self.get_parameter(
            'exploration_blocked_turn_max_tries').value)
        if self._blocked_turn_count >= max_tries:
            return False
        total_limit = math.radians(abs(float(self.get_parameter(
            'exploration_blocked_turn_total_limit_deg').value)))
        angle = math.radians(abs(float(self.get_parameter(
            'exploration_blocked_turn_deg').value)))
        speed = abs(float(self.get_parameter(
            'exploration_blocked_turn_speed_radps').value))
        if angle <= 0.0 or speed <= 0.0:
            return False
        if self._blocked_turn_total_rad + angle > total_limit:
            angle = total_limit - self._blocked_turn_total_rad
            if angle < math.radians(30.0):
                return False
        pose = self.update_pose()
        if pose is None:
            return False
        direction = self._blocked_turn_direction(pose)
        rotated = self._rotate_in_place(direction, angle, speed)
        self._blocked_turn_total_rad += rotated
        self._blocked_turn_count += 1
        self.get_logger().warn(
            'P11 blocked-turn %d/%d: rear stayed blocked, rotating %s %.0fdeg '
            '(rotated %.0fdeg, total %.0fdeg/%.0fdeg) to leave the failed sector'
            % (self._blocked_turn_count, max_tries,
               'CCW' if direction > 0 else 'CW', math.degrees(angle),
               math.degrees(rotated), math.degrees(self._blocked_turn_total_rad),
               math.degrees(total_limit)))
        return True

    def _maybe_retreat_when_blocked(self, now):
        """R1b: bounded reverse out of a blocked-clearance deadlock (see the hook above).

        Rate-limited on purpose: at most blocked_retreat_max_per_session attempts, spaced at
        least blocked_retreat_min_interval_sec apart, and never more than the R1 envelope of
        0.50 m at 0.12 m/s with the rear strip verified in /map first.
        """
        # 2026-10-07: R1b is an EXPLORATION retreat -> it reads the exploration gate, not
        # the return-home one (the shared gate used to switch both off together).
        if not bool(self.get_parameter('exploration_retreat_enabled').value):
            return
        max_per_session = int(self.get_parameter('blocked_retreat_max_per_session').value)
        if self._blocked_retreat_count >= max_per_session:
            return
        interval = float(self.get_parameter('blocked_retreat_min_interval_sec').value)
        if now - self._blocked_retreat_last_at < interval:
            return
        self._blocked_retreat_last_at = now
        self._blocked_retreat_count += 1
        # the return-home path owns this latch; keep it intact across an exploration retreat
        saved_latch = getattr(self, '_retreat_used_for_attempt', False)
        self._retreat_used_for_attempt = False
        try:
            status, moved = self._retreat_from_occupied_cells('exploration')
        finally:
            self._retreat_used_for_attempt = saved_latch
        front = -1.0 if self.front_min_range is None else float(self.front_min_range)
        self.get_logger().warn(
            'R1b blocked-clearance retreat %d/%d: status=%s moved=%.2fm front_before=%.2fm'
            % (self._blocked_retreat_count, max_per_session, status, moved, front))
        # 2026-10-13 P11 方案1：连续 rear-blocked -> 转向换朝向。
        # 实测 14:03 轮 4 次后退里 2 次 rear-blocked：前进被前向门禁拒、
        # 后退被 rear 探测拒、自转又 Collision Ahead 失败，车在原地反复重试。
        if status == 'rear-blocked':
            self._blocked_rear_streak += 1
            if self._blocked_rear_streak >= 2:
                turned = self._safe_call(
                    'P11 blocked-turn',
                    lambda: self._turn_after_rear_blocked(now), False)
                if turned:
                    self._blocked_rear_streak = 0
                else:
                    self.get_logger().warn(
                        'P11 blocked-turn exhausted (%d tries, %.0fdeg total); '
                        'leaving the deadlock for the upper layer to re-plan'
                        % (self._blocked_turn_count,
                           math.degrees(self._blocked_turn_total_rad)))
        elif status == 'moved':
            # 后退成功 -> 脱困成立，重置连续计数与转向预算
            self._blocked_rear_streak = 0
            self._blocked_turn_count = 0
            self._blocked_turn_total_rad = 0.0
            # 2026-10-13 P12-#2：后退做过了、前方却仍被挡 => 双向死锁，
            # 把这个位置写成卡点，避免探索再往同一处派目标
            # （实测 (3.26,4.18)/(3.82,3.44) 跨轮反复成为卡点）。
            if self.front_blocked:
                self._safe_call(
                    'P12 stuck-spot record',
                    lambda: self._record_escape_ineffective_spot(), False)
        self._blocked_episode_started_at = None      # re-evaluate the episode from scratch

    def _safe_call(self, label, fn, fallback):
        """Run an exploration helper so that a bug in it can never kill the node.

        Today's crash proved the point: an exception in auto_explore_callback made the crash
        handler raise as well, the node died, and the car stood still with no cause in the log.
        Every helper I added (R1b/R1c/R1d/A/B) now goes through here.
        """
        try:
            return fn()
        except Exception as exc:   # noqa: BLE001 - deliberately broad: never fatal
            try:
                self.get_logger().warn('%s failed (%r); continuing without it' % (label, exc))
            except Exception:      # noqa: BLE001
                pass
            return fallback

    def _update_blocked_episode(self, now):
        if self.front_blocked:
            if self._blocked_episode_started_at is None:
                self._blocked_episode_started_at = now
            self._blocked_clear_since = None
            # R1b (2026-09-29): if the front stays too close for too long, back away once.
            # Without this the manager waits for a clearance nothing can restore: the
            # mapping behaviour tree has no backup behaviour and its spin is refused.
            if (now - self._blocked_episode_started_at
                    >= float(self.get_parameter('blocked_retreat_after_sec').value)):
                self._safe_call('R1b retreat', lambda: self._maybe_retreat_when_blocked(now), None)
            return
        if self._blocked_episode_started_at is None:
            return

        clearance_threshold = (
            self.front_stop_distance + self.blocked_clearance_hysteresis_m)
        if (self.front_min_range is None
                or self.front_min_range < clearance_threshold):
            self._blocked_clear_since = None
            return
        if self._blocked_clear_since is None:
            self._blocked_clear_since = now
            return
        if now - self._blocked_clear_since < self.blocked_clearance_stable_sec:
            return

        self._clear_recovery_intent(clear_episode=True)
        blocked_states = (
            'blocked_precheck_pending',
            'blocked_waiting_clearance',
            'recovery_cooldown',
            'spin_recovery_rejected',
            'spin_recovery_aborted',
        )
        if (self.navigation_state in blocked_states
                and self.auto_explore
                and not self.mapping_done
                and not self.base_charging
                and not self._goal_transport_fault
                and not self._spin_transport_fault):
            self.mode = 'mapping'
            self.navigation_state = 'clearance_restored'
            self.last_error = ''

    def _arm_exploration_precheck_recovery(self, now):
        if self._blocked_episode_started_at is None:
            self._blocked_episode_started_at = now
        if self._blocked_episode_attempted:
            return False
        self._recovery_intent = 'exploration_precheck'
        self.navigation_state = 'blocked_precheck_pending'
        return True

    def _precheck_recovery_is_ready(self, now):
        return (
            self._recovery_intent == 'exploration_precheck'
            and not self._blocked_episode_attempted
            and self.front_blocked
            and self._blocked_episode_started_at is not None
            and now - self._blocked_episode_started_at
            >= self.blocked_recovery_delay_sec
            and now >= self._next_blocked_recovery_at
            and self.goal_send_future is None
            and not self.nav_goal_active
            and self._spin_phase == 'idle'
            and not self.mapping_done
            and self.auto_explore
            and not self.base_charging
            and not self._goal_transport_fault
            and not self._spin_transport_fault
        )

    def _motion_is_inhibited(self):
        return (
            self.shutdown_started
            or self._mapping_cancel_requested
            or bool(self._mapping_timeout_failure_kind)
            or self._return_home_xy_finalizing
            or self._stall_cancel_requested
            or self._cancel_on_accept_context is not None
            or self._goal_transport_fault
            or self._spin_transport_fault
            or self._spin_cancel_on_accept_reason is not None
            or self._spin_phase not in ('idle', 'executing')
            or self.base_charging
        )

    def _cancel_zero_guard_callback(self):
        if self._motion_is_inhibited():
            self.publish_zero_velocity()

    def cmd_vel_callback(self, msg):
        super().cmd_vel_callback(msg)
        components = (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )
        if (self._motion_is_inhibited()
                and any(abs(value) > 1.0e-4 for value in components)):
            # Best-effort clamp until the exact action result arrives.
            # The 20 Hz guard above also bounds the interval between zeroes.
            self.publish_zero_velocity()

    def goal_callback(self, msg):
        return self._run_with_goal_kind(
            'manual', super().goal_callback, msg)

    def start_mapping_callback(self, request, response):
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'mapping session is finalizing after return-home completion; '
                'wait for the session runner to restore functional services')
            return response
        if self._mapping_timeout_failure_kind:
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'mapping exit has timed out; wait for controlled session '
                'cancellation and functional service restoration')
            return response
        if self._mapping_cancel_requested:
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'mapping cancellation is in progress; wait for the session '
                'runner to restore functional services')
            return response
        if (self._goal_transport_fault
                or self._spin_transport_fault
                or self._spin_phase != 'idle'
                or self._stall_cancel_requested
                or self._cancel_on_accept_context is not None):
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'mapping cannot resume while a Nav2 action state is '
                'uncertain; '
                'restart the Cartographer mapping session')
            return response
        response = super().start_mapping_callback(request, response)
        if response.success:
            self._reset_return_home_xy_completion()
            was_voice_paused = self._voice_pause_latched
            self._voice_pause_latched = False
            if was_voice_paused:
                self._mapping_resume_accepted_at_epoch_ms = int(
                    time.time() * 1000
                )
            self._cartographer_finish_future = None
            self._cartographer_finish_started_at = 0.0
            self._cartographer_trajectory_finished = False
            self._reset_final_map_save_state()
            self._cartographer_return_tf_anchor = None
            self._cartographer_return_tf_stable_since = 0.0
            self._frozen_return_map_to_odom = None
            self._reset_mapping_exit_timeout()
            if was_voice_paused:
                # Publish the acknowledgement before the next navigation goal
                # is visible, so the voice and mini-program observers can
                # converge without waiting for an arbitrary UI timeout.
                self._write_session_status()
        return response

    def _reset_return_home_xy_completion(self, clear_finalizing=True):
        """Forget the one-shot position gate and its exact cancel token."""
        self._return_home_xy_grace_started_at = None
        # 2026-10-10: 一并清空位置累计器，避免换代际后带着旧累计。
        self._return_home_xy_accum_sec = 0.0
        self._return_home_xy_accum_at = 0.0
        self._return_home_xy_last_dist = None
        self._return_home_xy_grace_generation = None
        self._return_home_xy_cancel_handle = None
        self._return_home_xy_cancel_generation = None
        self._return_home_xy_cancel_result_future = None
        self._return_home_xy_cancel_distance_m = None
        self._last_return_home_xy_wait_log = 0.0
        if clear_finalizing:
            self._return_home_xy_finalizing = False

    def _fresh_return_home_xy_distance(self):
        """Return current map-frame XY error only for a fresh, valid TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - an unavailable pose is not success
            return None

        stamp = getattr(getattr(transform, 'header', None), 'stamp', None)
        stamp_sec = int(getattr(stamp, 'sec', 0) or 0)
        stamp_nanosec = int(getattr(stamp, 'nanosec', 0) or 0)
        stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
        if stamp_ns <= 0:
            return None

        try:
            now_ns = int(self.get_clock().now().nanoseconds)
        except (AttributeError, TypeError, ValueError):
            return None
        age_sec = (now_ns - stamp_ns) / 1_000_000_000.0
        max_age_sec = max(
            0.05, float(getattr(self, 'return_home_tf_max_age_sec', 0.20)))
        if age_sec < -0.05 or age_sec > max_age_sec:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return None
        norm_sq = sum(float(value) * float(value) for value in (
            rotation.x, rotation.y, rotation.z, rotation.w))
        if abs(norm_sq - 1.0) > 0.10:
            return None

        return math.hypot(
            float(translation.x) - float(self.home_x),
            float(translation.y) - float(self.home_y),
        )

    def _return_home_xy_goal_is_current(self):
        return (
            self.mapping_done
            and self.shutdown_after_return_home
            and self.return_home_requested
            and not self.pending_home_goal
            and self.goal_send_future is None
            and self.nav_goal_active
            and self._active_goal_kind == 'home'
            and self._active_goal_generation is not None
            and self.current_goal_handle is not None
            and self.goal_result_future is not None
            and not self.shutdown_started
            and not self._mapping_cancel_requested
            and not self.base_charging
            and self._spin_phase == 'idle'
            and self._cancel_on_accept_context is None
            and not self._stall_cancel_requested
            and not self._goal_transport_fault
            and not self._spin_transport_fault
        )

    def _maybe_complete_return_home_by_position(self, now=None):
        """Give exact yaw six seconds, then cancel this home goal safely."""
        if self._return_home_xy_finalizing:
            return False
        if self._return_home_xy_cancel_handle is not None:
            return False
        if not self._return_home_xy_goal_is_current():
            self._reset_return_home_xy_completion()
            return False

        generation = self._active_goal_generation
        if (self._return_home_xy_grace_generation is not None
                and self._return_home_xy_grace_generation != generation):
            self._reset_return_home_xy_completion()

        distance_m = self._fresh_return_home_xy_distance()
        if distance_m is None:
            return False
        now = time.monotonic() if now is None else float(now)

        tolerance_m = self.return_home_xy_completion_tolerance_m
        if distance_m > tolerance_m:
            # 2026-10-10: 超出容差即清空累计器。旧实现在此只打日志，而 grace
            # 计时器一旦启动就不再复位，于是 6s 后每拍都落在该分支、永远
            # 等不到重新进入容差，只能无限微调（实测 10:38-10:39）。
            self._return_home_xy_accum_sec = 0.0
            self._return_home_xy_accum_at = now
            self._return_home_xy_last_dist = distance_m
            if now - self._last_return_home_xy_wait_log >= 2.0:
                self._last_return_home_xy_wait_log = now
                self.get_logger().warn(
                    'return-home is outside the position-only tolerance: '
                    f'{distance_m:.3f}m > {tolerance_m:.3f}m; hold reset')
            return False

        # 位于容差内：若与上一拍相比仍在挪动，累计器清零，确保被认可的是
        # 停稳而不是恰好路过。
        previous_distance = self._return_home_xy_last_dist
        self._return_home_xy_last_dist = distance_m
        if (previous_distance is not None
                and abs(previous_distance - distance_m)
                > self.return_home_completion_move_epsilon_m):
            self._return_home_xy_accum_sec = 0.0
            self._return_home_xy_accum_at = now
            return False

        if self._return_home_xy_accum_at <= 0.0:
            self._return_home_xy_accum_at = now
            if self._return_home_xy_grace_started_at is None:
                self._return_home_xy_grace_started_at = now
                self._return_home_xy_grace_generation = generation
            self.get_logger().info(
                'return-home entered the position-only tolerance: '
                f'distance={distance_m:.3f}m; holding '
                f'{self.return_home_completion_hold_sec:.1f}s before '
                'completing by position')
            return False

        self._return_home_xy_accum_sec += max(
            0.0, now - self._return_home_xy_accum_at)
        self._return_home_xy_accum_at = now

        if (self.return_home_completion_requires_orientation
                and self._return_home_xy_accum_sec
                < self.return_home_orientation_grace_sec):
            return False                   # 兼容开关：回到旧语义
        if self._return_home_xy_accum_sec < self.return_home_completion_hold_sec:
            return False

        self.get_logger().info(
            'return-home position reached and settled: '
            f'distance={distance_m:.3f}m, held '
            f'{self._return_home_xy_accum_sec:.2f}s (orientation is not a '
            'completion prerequisite)')
        if self.goal_result_future.done():
            # The action terminal callback already owns the outcome.
            return False

        # Install exact action identity before requesting cancellation. A
        # completed Future may invoke its callback synchronously.
        self._return_home_xy_cancel_handle = self.current_goal_handle
        self._return_home_xy_cancel_generation = generation
        self._return_home_xy_cancel_result_future = self.goal_result_future
        self._return_home_xy_cancel_distance_m = distance_m
        self._stop_manager_recovery_motion()
        self.mode = 'safety_stop'
        self.navigation_state = 'return_home_orientation_timeout_canceling'
        self.last_error = ''
        self.get_logger().warn(
            'return-home exact orientation was not reached within '
            f'{self.return_home_orientation_grace_sec:.1f}s; '
            f'position error={distance_m:.3f}m, stopping and waiting for the '
            'exact NavigateToPose terminal result')
        self.publish_zero_velocity()
        self._begin_monitored_cancel(
            self.RETURN_HOME_ORIENTATION_TIMEOUT_CONTEXT, False, now)
        self.publish_zero_velocity()
        return True

    @staticmethod
    def _shortest_angle_distance(first, second):
        return abs(math.atan2(
            math.sin(first - second), math.cos(first - second)))

    @staticmethod
    def _transform_yaw(transform):
        rotation = transform.transform.rotation
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    # ---------------------------------------------------------------- R1 (2026-09-29)
    def _retreat_context(self):
        """Which lifecycle stage owns the current tick: 'exploration' or 'return_home'.

        The base class sets return_home_requested immediately before it dispatches the
        home goal through send_nav_goal (stemm_nav2_manager/manager.py:1015-1017), which
        also runs R1c.  Without this check a return-home dispatch would still reverse.
        """
        if (getattr(self, 'return_home_requested', False)
                or getattr(self, 'pending_home_goal', False)):
            return 'return_home'
        return 'exploration'

    def _retreat_from_occupied_cells(self, context=None):
        """Bounded reverse; the gate depends on the lifecycle stage, not one shared switch.

        Callers:
          exploration : R1b blocked-clearance retreat, R1c pre-goal clearance retreat
          return_home : R1 boxed-in start (before dispatch), return-home retry pre-retreat

        The mapping stack has no Nav2 backup behaviour on purpose, so this issues the
        reverse directly and refuses to move at all unless /map shows a free strip behind
        the robot.  context=None falls back to the current lifecycle stage, so a call site
        that forgets to pass one can never silently inherit the wrong policy.
        """
        feature = context or self._retreat_context()
        gate = ('exploration_retreat_enabled' if feature == 'exploration'
                else 'return_home_retreat_enabled')
        if not bool(self.get_parameter(gate).value):
            return 'disabled', 0.0
        if getattr(self, '_retreat_used_for_attempt', False):
            return 'already-used', 0.0
        self._retreat_used_for_attempt = True

        # 2026-10-13 P11 方案4：探索与返航使用各自的后退参数组。
        # 此前两者共用 return_home_retreat_*，探索被迫用返航尺度
        # （max_dist=0.90 -> probe=1.15m），后方稍长一点有障碍就判 rear-blocked。
        # feature 已在上方由 context 或 _retreat_context() 决定。
        _pfx = ('exploration_retreat_' if feature == 'exploration'
                else 'return_home_retreat_')
        speed = abs(float(self.get_parameter(_pfx + 'speed_mps').value))
        max_dist = abs(float(self.get_parameter(_pfx + 'max_distance_m').value))
        max_sec = abs(float(self.get_parameter(_pfx + 'max_sec').value))
        margin = abs(float(self.get_parameter(_pfx + 'rear_margin_m').value))
        settle = abs(float(self.get_parameter('return_home_retreat_settle_sec').value))
        if speed <= 0.0 or max_dist <= 0.0 or max_sec <= 0.0:
            return 'misconfigured', 0.0

        current = self.update_pose()
        if current is None or self.last_map is None:
            return 'no-pose-or-map', 0.0

        # 1) is the strip behind the robot free in /map?
        info = self.last_map.info
        res = info.resolution
        if res <= 0.0:
            return 'bad-resolution', 0.0
        ox, oy = info.origin.position.x, info.origin.position.y
        x, y = current.pose.position.x, current.pose.position.y
        q = current.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        probe = max_dist + margin
        half_width = 0.30
        steps = max(3, int(probe / max(res, 0.02)) + 1)
        for i in range(1, steps + 1):
            back = -probe * i / steps                     # behind the robot
            for lateral in (-half_width, 0.0, half_width):
                px = x + back * math.cos(yaw) - lateral * math.sin(yaw)
                py = y + back * math.sin(yaw) + lateral * math.cos(yaw)
                mx = int((px - ox) / res)
                my = int((py - oy) / res)
                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    return 'rear-outside-map', 0.0
                if self.last_map.data[my * info.width + mx] >= 50:
                    return 'rear-blocked', 0.0

        # 2) bounded reverse
        start = time.monotonic()
        travelled = 0.0
        last = start
        twist = Twist()
        twist.linear.x = -speed
        try:
            while True:
                now = time.monotonic()
                self.cmd_vel_pub.publish(twist)
                travelled += speed * max(0.0, now - last)
                last = now
                if travelled >= max_dist or (now - start) >= max_sec:
                    break
                time.sleep(0.05)   # R1e: 20 Hz wins the cmd_vel race
        finally:
            self.publish_zero_velocity()
            time.sleep(settle)
        self.get_logger().warn(
            'R1 bounded reverse: context=%s moved back %.2f m at %.2f m/s in %.1f s '
            '(limit %.2f m / %.1f s)' % (feature, travelled, speed, time.monotonic() - start,
                                         max_dist, max_sec))
        return 'moved', travelled

    def _fail_return_home_before_dispatch(self, message):
        """Retry return-home setup failures before manual takeover.

        Cartographer finalization and the first Nav2 goal can experience short
        TF/service gaps while the robot is already near the origin. Those
        transient conditions are not evidence that the home target is
        unreachable, so preserve the active session and consume the same
        bounded retry budget used for terminal Nav2 failures.
        """
        # R1 (2026-09-29): if the start is refused for being too close to occupied
        # cells, back away a little and let the normal retry chain re-evaluate.
        if 'too close to occupied' in str(message):
            status, moved = self._retreat_from_occupied_cells('return_home')
            if status == 'moved':
                clear, why = self.current_start_is_map_clear()
                self.get_logger().warn(
                    'R1 post-reverse clearance check: clear=%s (%s)' % (clear, why))
                if clear:
                    self.return_home_requested = True
                    self.return_home_ready_after = time.monotonic()
                    self.navigation_state = 'return_home_retry_after_reverse'
                    return True, 'return-home retry after bounded reverse'
        self._reset_return_home_xy_completion()
        self.pending_home_goal = False
        self.return_home_requested = False
        self.publish_zero_velocity()
        self.return_home_retry_count += 1

        if self.return_home_retry_count > self.return_home_max_retries:
            self._cartographer_finish_future = None
            self._frozen_return_map_to_odom = None
            self.mode = 'stopped'
            self.navigation_state = 'return_home_failed_waiting_manual'
            self.last_error = (
                f'{message}; return-home failed after '
                f'{self.return_home_max_retries} retries, '
                'keeping launch alive for manual takeover')
            self.get_logger().error(self.last_error)
            return False, self.last_error

        # Keep a completed trajectory and an in-flight finish request intact:
        # resending FinishTrajectory after a successful response is unsafe.
        # Restart only the wait window for a still-pending finish request; all
        # other retry attempts re-check the prerequisite from scratch.
        now = time.monotonic()
        if self._cartographer_finish_future is not None:
            self._cartographer_finish_started_at = now
        elif not self._cartographer_trajectory_finished:
            self._cartographer_finish_started_at = 0.0
        self.return_home_ready_after = (
            now + self.return_home_retry_delay_sec)
        self.return_home_tf_stable_since = 0.0
        self._cartographer_return_tf_anchor = None
        self._cartographer_return_tf_stable_since = 0.0
        self.pending_home_goal = True
        self.mode = 'returning_home'
        self.navigation_state = 'return_home_retry_recovery'
        self.last_error = (
            f'{message}; retrying return-home '
            f'{self.return_home_retry_count}/{self.return_home_max_retries}')
        self.get_logger().warn(self.last_error)
        return False, self.last_error

    def _schedule_return_home_retry(self, reason):
        """2026-10-06: 返航失败重试前先清代价地图并做一次受控后退。

        原实现只是把失败转给父类的重试链；本轮实测表明：当起点被近处障碍夹住时，
        父类重试仍会在同一位置规划失败（planner -> no valid path found），
        于是 3 次预算耗尽、返航失败。这里复用既有 R1 有界后退（自带后方净空校验、
        限速 0.15m/s、距离与时长双上限），每次重试最多一次，让下一次规划有通道。
        """
        # 2026-10-07: the reverse is now owned by return_home_retreat_enabled (context
        # 'return_home'); this older per-feature flag is kept for configuration
        # compatibility and kept False.  Both must allow it before anything moves.
        if (bool(self.get_parameter('return_home_failure_retreat_enabled').value)
                and bool(self.get_parameter('return_home_retreat_enabled').value)):
            self._safe_call(
                'return-home failure retreat',
                self._retreat_before_return_home_retry,
                None,
            )
        self._reset_return_home_xy_completion()
        return super()._schedule_return_home_retry(reason)

    def _retreat_before_return_home_retry(self):
        """清一次 global costmap，然后做一次有界后退，给下一次返航让出通道。"""
        self._request_costmap_clears()
        # 给代价地图重建留出时间，避免在膨胀层尚未就绪时立刻重规划。
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            time.sleep(0.1)
        # 每次返航尝试允许一次后退（R1 的 per-attempt 闩锁按尝试复位）。
        self._retreat_used_for_attempt = False
        status, moved = self._retreat_from_occupied_cells('return_home')
        self.get_logger().warn(
            'return-home retry pre-retreat: status=%s moved=%.2fm '
            '(front=%.2fm)'
            % (status, moved,
               -1.0 if self.front_min_range is None else float(self.front_min_range)))
        return status, moved

    def _publish_frozen_return_tf(self):
        """Keep Cartographer's final map->odom transform fresh for Nav2."""
        frozen = self._frozen_return_map_to_odom
        if frozen is None:
            return
        frozen.header.stamp = self.get_clock().now().to_msg()
        self._frozen_return_tf_broadcaster.sendTransform(frozen)

    def _freeze_final_map_to_odom_transform(self):
        """Capture and republish Cartographer's final map->odom transform."""
        try:
            sampled = self.tf_buffer.lookup_transform(
                self.global_frame, 'odom_combined', rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            return False, (
                'could not capture Cartographer final map->odom transform: '
                f'{exc}')

        if not sampled.header.frame_id or not sampled.child_frame_id:
            return False, (
                'Cartographer final map->odom transform had empty frame IDs')

        frozen = TransformStamped()
        frozen.header.frame_id = sampled.header.frame_id
        frozen.child_frame_id = sampled.child_frame_id
        frozen.transform = sampled.transform
        self._frozen_return_map_to_odom = frozen
        self._publish_frozen_return_tf()
        self.get_logger().info(
            'frozen final map->odom_combined transform; rebroadcasting it '
            'with current timestamps for Nav2 return-home')
        return True, 'final map->odom transform frozen'

    def _finish_cartographer_trajectory_before_return(self):
        """Finish mapping once, then leave return-home pending for the TF gate."""
        if self._cartographer_trajectory_finished:
            if self._frozen_return_map_to_odom is None:
                frozen, message = self._freeze_final_map_to_odom_transform()
                if not frozen:
                    return self._fail_return_home_before_dispatch(message)
            if self._final_map_finish_revision is None:
                self._final_map_finish_revision = self._map_revision
                self._final_map_wait_started_at = time.monotonic()
            return True, 'Cartographer trajectory is already finished'

        now = time.monotonic()
        future = self._cartographer_finish_future
        if future is not None:
            if future.done():
                self._cartographer_finish_future = None
                try:
                    response = future.result()
                except Exception as exc:  # noqa: BLE001
                    return self._fail_return_home_before_dispatch(
                        'Cartographer finish_trajectory request failed: '
                        f'{exc}')
                status = getattr(response, 'status', None)
                if status is None or int(getattr(status, 'code', 1)) != 0:
                    detail = getattr(status, 'message', 'empty status')
                    return self._fail_return_home_before_dispatch(
                        'Cartographer finish_trajectory was rejected: '
                        f'{detail}')
                # A successful FinishTrajectory response is final even if
                # the immediate TF sampling attempt loses a frame. Record it
                # before retrying TF capture so subsequent retries never send
                # a duplicate FinishTrajectory request.
                self._cartographer_trajectory_finished = True
                frozen, message = self._freeze_final_map_to_odom_transform()
                if not frozen:
                    return self._fail_return_home_before_dispatch(message)
                self._cartographer_finish_started_at = 0.0
                self._final_map_finish_revision = self._map_revision
                self._final_map_wait_started_at = time.monotonic()
                self._cartographer_return_tf_anchor = None
                self._cartographer_return_tf_stable_since = 0.0
                self.get_logger().info(
                    'Cartographer trajectory %d finished; waiting for '
                    'the final map->odom transform to settle before '
                    'return-home' % self.cartographer_trajectory_id)
                return True, 'Cartographer trajectory finished'

            if (now - self._cartographer_finish_started_at
                    >= self.cartographer_finish_timeout_sec):
                return self._fail_return_home_before_dispatch(
                    'Cartographer finish_trajectory timed out after '
                    f'{self.cartographer_finish_timeout_sec:.1f}s')
            self.publish_zero_velocity()
            self.navigation_state = 'return_home_waiting_cartographer_finish'
            return False, (
                'mapping done acknowledged; waiting for Cartographer '
                'trajectory finish')

        if self._cartographer_finish_started_at <= 0.0:
            self._cartographer_finish_started_at = now
        if not self.cartographer_finish_client.service_is_ready():
            if (now - self._cartographer_finish_started_at
                    >= self.cartographer_finish_timeout_sec):
                return self._fail_return_home_before_dispatch(
                    'Cartographer finish_trajectory service is unavailable '
                    f'after {self.cartographer_finish_timeout_sec:.1f}s')
            self.publish_zero_velocity()
            self.navigation_state = 'return_home_waiting_cartographer_finish'
            return False, (
                'mapping done acknowledged; waiting for Cartographer '
                'finish_trajectory service')

        request = FinishTrajectory.Request()
        request.trajectory_id = self.cartographer_trajectory_id
        try:
            self._cartographer_finish_future = (
                self.cartographer_finish_client.call_async(request))
        except Exception as exc:  # noqa: BLE001
            return self._fail_return_home_before_dispatch(
                'could not request Cartographer finish_trajectory: '
                f'{exc}')
        self._cartographer_finish_started_at = now
        self.publish_zero_velocity()
        self.navigation_state = 'return_home_waiting_cartographer_finish'
        self.get_logger().info(
            'requested Cartographer finish_trajectory for trajectory %d '
            'before return-home' % self.cartographer_trajectory_id)
        return False, (
            'mapping done acknowledged; finishing Cartographer trajectory '
            'before returning home')

    def _tf_is_stably_recent(
            self, target_frame, source_frame, max_age, stable_sec):
        """After finishing, require map->odom values to stay within tolerance."""
        if (not self.mapping_done
                or not self._cartographer_trajectory_finished):
            return super()._tf_is_stably_recent(
                target_frame, source_frame, max_age, stable_sec)

        if not super()._tf_is_recent(
                target_frame, source_frame, max_age=max_age):
            self._cartographer_return_tf_anchor = None
            self._cartographer_return_tf_stable_since = 0.0
            return False

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self._cartographer_return_tf_anchor = None
            self._cartographer_return_tf_stable_since = 0.0
            self.get_logger().warn(
                'could not sample final return-home TF '
                f'({source_frame}->{target_frame}): {exc}')
            return False

        current = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self._transform_yaw(transform),
        )
        now = time.monotonic()
        anchor = self._cartographer_return_tf_anchor
        if anchor is None:
            self._cartographer_return_tf_anchor = current
            self._cartographer_return_tf_stable_since = now
            return False

        translation_delta = math.hypot(
            current[0] - anchor[0], current[1] - anchor[1])
        rotation_delta = self._shortest_angle_distance(current[2], anchor[2])
        if (translation_delta
                > self.cartographer_finish_tf_translation_tolerance_m
                or rotation_delta
                > self.cartographer_finish_tf_rotation_tolerance_rad):
            self._cartographer_return_tf_anchor = current
            self._cartographer_return_tf_stable_since = now
            if now - self._last_cartographer_tf_motion_log >= 1.0:
                self._last_cartographer_tf_motion_log = now
                self.get_logger().info(
                    'final map->odom changed after trajectory finish: '
                    f'dxy={translation_delta:.3f}m, '
                    f'dyaw={rotation_delta:.3f}rad; restarting '
                    'return-home TF settle window')
            return False

        stable_for = now - self._cartographer_return_tf_stable_since
        if stable_for < stable_sec:
            if now - self.last_return_home_wait_log > 1.0:
                self.last_return_home_wait_log = now
                self.get_logger().info(
                    'waiting for final return-home TF to remain fixed '
                    f'({stable_for:.1f}/{stable_sec:.1f}s)')
            return False
        return True

    @staticmethod
    def _map_origin_yaw(origin):
        rotation = origin.orientation
        return math.atan2(
            2.0 * (
                rotation.w * rotation.z
                + rotation.x * rotation.y
            ),
            1.0 - 2.0 * (
                rotation.y * rotation.y
                + rotation.z * rotation.z
            ),
        )

    @staticmethod
    def _write_temp_file(path, content, binary):
        temp_path = path.with_name(
            f'.{path.name}.tmp.{os.getpid()}')
        mode = 'wb' if binary else 'w'
        kwargs = {} if binary else {'encoding': 'utf-8'}
        try:
            with open(temp_path, mode, **kwargs) as handle:
                handle.write(content)
                handle.flush()
                os.fsync(handle.fileno())
            return temp_path
        except Exception:
            try:
                temp_path.unlink()
            except OSError:
                pass
            raise

    def _cached_final_map_msg_or_raise(self):
        """2026-10-12: 等不到 finish 后的新栅格时，退回保存缓存地图：校验并返回可用的缓存地图。

        主路径与超时兜底共用，保证两者落盘的地图质量判据完全一致。
        """
        msg = self.last_map
        if msg is None:
            raise RuntimeError('final OccupancyGrid is not available')

        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            raise RuntimeError(
                f'final OccupancyGrid has invalid size {width}x{height}')
        if len(msg.data) != width * height:
            raise RuntimeError(
                'final OccupancyGrid cell count does not match its size')
        return msg

    def _save_cached_final_map(self):
        msg = self._cached_final_map_msg_or_raise()

        width = int(msg.info.width)
        height = int(msg.info.height)

        pixels = bytearray(width * height)
        for target_row, source_row in enumerate(
                range(height - 1, -1, -1)):
            source_offset = source_row * width
            target_offset = target_row * width
            for col in range(width):
                value = int(msg.data[source_offset + col])
                if value < 0:
                    pixel = 205
                elif value >= 65:
                    pixel = 0
                elif value <= 35:
                    pixel = 254
                else:
                    pixel = 205
                pixels[target_offset + col] = pixel

        map_stem = Path(os.path.expanduser(self.map_save_path))
        map_stem.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = map_stem.with_suffix('.pgm')
        yaml_path = map_stem.with_suffix('.yaml')
        pgm = (
            f'P5\n# CREATOR: stemm_cartographer final map\n'
            f'{width} {height}\n255\n'
        ).encode('ascii') + bytes(pixels)

        origin = msg.info.origin
        yaml = (
            f'image: {pgm_path.name}\n'
            'mode: trinary\n'
            f'resolution: {float(msg.info.resolution):.9g}\n'
            'origin: ['
            f'{float(origin.position.x):.9g}, '
            f'{float(origin.position.y):.9g}, '
            f'{self._map_origin_yaw(origin):.9g}]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )

        pgm_temp = self._write_temp_file(pgm_path, pgm, binary=True)
        try:
            yaml_temp = self._write_temp_file(
                yaml_path, yaml, binary=False)
        except Exception:
            try:
                pgm_temp.unlink()
            except OSError:
                pass
            raise

        try:
            # YAML is the commit marker and is replaced last.
            os.replace(pgm_temp, pgm_path)
            os.replace(yaml_temp, yaml_path)
        finally:
            for temp_path in (pgm_temp, yaml_temp):
                try:
                    temp_path.unlink()
                except OSError:
                    pass

        return pgm_path, yaml_path, width, height

    def _save_final_map_before_return(self):
        if self._final_map_saved:
            return True, 'final map is already saved'
        if (
            self._final_map_save_attempted
            and self._final_map_save_error
        ):
            return True, (
                'final map save failed; continuing safe return-home: '
                f'{self._final_map_save_error}')

        now = time.monotonic()
        if self._final_map_wait_started_at <= 0.0:
            self._final_map_wait_started_at = now
        if self._final_map_finish_revision is None:
            self._final_map_finish_revision = self._map_revision

        elapsed = now - self._final_map_wait_started_at
        final_frame_arrived = (
            self._map_revision > self._final_map_finish_revision)
        # 2026-10-10: 轨迹优化期间栅格会陆续到达。只要地图还在增长，就把截止
        # 时间顺延一个 grace 窗口，避免"差一点点就到"被硬切（实测优化可在
        # finish 受理后 39 秒才开始）。
        if self._map_revision > self._final_map_wait_revision:
            self._final_map_wait_revision = self._map_revision
            self._final_map_wait_deadline = max(
                self._final_map_wait_deadline,
                now + self.final_map_grace_after_growth_sec)
        if self._final_map_wait_deadline <= 0.0:
            self._final_map_wait_deadline = (
                self._final_map_wait_started_at
                + self.final_map_wait_timeout_sec)
        if (
            (self.last_map is None or not final_frame_arrived)
            and now < self._final_map_wait_deadline
        ):
            self.publish_zero_velocity()
            self.mode = 'finalizing_map'
            self.navigation_state = 'return_home_waiting_final_map'
            return False, (
                'Cartographer trajectory finished; waiting for the final '
                f'OccupancyGrid (elapsed={elapsed:.1f}s, '
                f'remaining={max(0.0, self._final_map_wait_deadline - now):.1f}s, '
                f'revision {self._final_map_finish_revision}->'
                f'{self._map_revision})')

        if self.last_map is None or not final_frame_arrived:
            wait_age = (
                None if self.last_map_time is None
                else round(time.monotonic() - self.last_map_time, 2))
            self._final_map_save_error = (
                'no post-finish OccupancyGrid arrived within '
                f'{self.final_map_wait_timeout_sec:.1f}s '
                f'(waited={elapsed:.1f}s, revision '
                f'{self._final_map_finish_revision}->{self._map_revision}, '
                f'last_map_age={wait_age}s, map_present='
                f'{self.last_map is not None})')
            if self.final_map_save_fallback_on_timeout:
                try:
                    pgm_path, yaml_path, width, height = (
                        self._save_cached_final_map())
                except Exception as exc:  # noqa: BLE001
                    self._final_map_save_attempted = True
                    self.last_error = (
                        'final map save failed before return-home: '
                        f'{self._final_map_save_error} | fallback save '
                        f'also failed: {exc}')
                    self.mode = 'returning_home'
                    self.navigation_state = 'return_home_map_save_failed'
                    self.get_logger().error(self.last_error)
                    return True, self.last_error
                self._final_map_save_attempted = True
                self._final_map_saved = True
                self._final_map_saved_fallback = True
                self._final_map_saved_at = time.strftime(
                    '%Y-%m-%dT%H:%M:%S%z')
                self.mode = 'returning_home'
                self.navigation_state = 'return_home_final_map_saved'
                self.get_logger().warn(
                    'saved final Cartographer map from the cached '
                    'OccupancyGrid (fallback; no post-finish grid '
                    f'arrived): {pgm_path}, {yaml_path}, '
                    f'{width}x{height}; reason={self._final_map_save_error}')
                return True, (
                    'final map saved via timeout fallback; '
                    'return-home may begin')
            self._final_map_save_attempted = True
            self.last_error = (
                'final map save failed before return-home: '
                f'{self._final_map_save_error}')
            self.mode = 'returning_home'
            self.navigation_state = 'return_home_map_save_failed'
            self.get_logger().error(self.last_error)
            return True, self.last_error

        self._final_map_save_attempted = True
        self.publish_zero_velocity()
        self.mode = 'finalizing_map'
        self.navigation_state = 'return_home_saving_final_map'
        try:
            pgm_path, yaml_path, width, height = (
                self._save_cached_final_map())
        except Exception as exc:  # noqa: BLE001 - safe return still proceeds
            self._final_map_save_error = str(exc)
            self.last_error = (
                f'final map save failed before return-home: {exc}')
            self.mode = 'returning_home'
            self.navigation_state = 'return_home_map_save_failed'
            self.get_logger().error(self.last_error)
            return True, self.last_error

        self._final_map_saved = True
        self._final_map_saved_at = time.strftime(
            '%Y-%m-%dT%H:%M:%S%z')
        self._final_map_save_error = ''
        self.last_error = ''
        self.mode = 'returning_home'
        self.navigation_state = 'return_home_final_map_saved'
        self.get_logger().info(
            'saved final Cartographer map after finish_trajectory: '
            f'{pgm_path}, {yaml_path}, {width}x{height}')
        return True, 'final map saved; return-home may begin'

    def _dispatch_pending_home_goal(self):
        if (self._goal_transport_fault
                or self._spin_transport_fault
                or self._spin_phase != 'idle'):
            self.publish_zero_velocity()
            return (
                False,
                'return-home suppressed because a previous Nav2 action '
                'state is uncertain; restart the mapping session safely',
            )
        finished, message = self._finish_cartographer_trajectory_before_return()
        if not finished:
            return False, message
        saved, message = self._save_final_map_before_return()
        if not saved:
            return False, message
        return self._run_with_goal_kind(
            'home', super()._dispatch_pending_home_goal)
        # 2026-10-10: 返航起终点连通性门禁。实测 09:36-09:38 四次派发全部
        # Nav2 status 6 且误差不减（0.706 -> 0.873 m），机器人没朝家走反而
        # 越走越远，4 分钟预算耗尽。与其如此，不如在派发前用当前 /map 核对
        # 起点与原点是否属于同一连通自由区，一次性明确拒绝。
        # 走既有 _fail_return_home_before_dispatch，沿用同一重试预算与
        # 最终转人工接管的语义；检查本身出错则保守放行。
        if getattr(self, 'return_home_require_connected_route', False):
            try:
                current_pose = self.update_pose()
                if current_pose is not None and self.last_map is not None:
                    connected, route_detail = self._safe_call(
                        'home route connectivity',
                        lambda: self._home_route_connectivity(current_pose),
                        (None, {}))
                    self._log_home_route_evidence(connected, route_detail or {})
                    if connected is False:
                        self.publish_zero_velocity()
                        reason = (
                            'return-home route is blocked on the saved map: the '
                            'current pose and the origin are in different free '
                            'regions (static free cells reachable from here: '
                            '%s of %s). Drive the robot back manually, or check '
                            'that the navigation map matches this mapping session.'
                            % (route_detail.get('reachableCells'),
                               route_detail.get('totalCells')))
                        return self._fail_return_home_before_dispatch(reason)
            except Exception as exc:  # noqa: BLE001 - 绝不因检查本身阻断返航
                self.get_logger().warn(
                    'return-home connectivity precheck failed: %s' % exc)
        return self._run_with_goal_kind(
            'home', super()._dispatch_pending_home_goal)

    def _auto_explore_callback_impl(self):
        if self._goal_transport_fault or self._spin_transport_fault:
            self.publish_zero_velocity()
            self._rrt_wait_started_at = None
            return
        if self._spin_phase != 'idle':
            # Do not delegate into the base goal-selection timer while a Spin
            # action owns, or may still own, the BehaviorServer.
            self._rrt_wait_started_at = None
            return
        if self.goal_send_future is not None:
            # Never let the base timer send a second goal while the action
            # server is still deciding whether to accept the first one.
            self._rrt_wait_started_at = None
            return

        now = time.monotonic()
        elapsed = now - self.started_at
        eligible_for_rrt_wait = (
            self.auto_explore
            and not self.mapping_done
            and not self.base_charging
            and not self.pending_home_goal
            and not self.nav_goal_active
            and self.goal_send_future is None
            and not self.is_recovering()
            and not self.is_recovery_state()
            and self.map_is_fresh()
            and self.nav2_is_ready()
            and not self.front_blocked
            and elapsed >= self.explore_startup_delay_sec
            and self.navigation_state not in ('sending_goal', 'executing')
        )
        if not eligible_for_rrt_wait:
            self._rrt_wait_started_at = None
            return self._run_with_goal_kind(
                'exploration_manager',
                super()._auto_explore_callback_impl,
            )

        if eligible_for_rrt_wait and self.rrt_fallback_wait_sec > 0.0:
            if self._rrt_wait_started_at is None:
                self._rrt_wait_started_at = now
            wait_age = now - self._rrt_wait_started_at
            if wait_age < self.rrt_fallback_wait_sec:
                self.mode = 'mapping'
                self.navigation_state = 'waiting_for_rrt_frontier'
                if now - getattr(self, '_last_rrt_wait_log', 0.0) > 2.0:
                    self.get_logger().info(
                        'waiting for RRT frontier before manager fallback '
                        f'({wait_age:.1f}/{self.rrt_fallback_wait_sec:.1f}s)')
                    self._last_rrt_wait_log = now
                return

        result = self._run_with_goal_kind(
            'exploration_manager',
            super()._auto_explore_callback_impl,
        )
        if self.goal_send_future is None and not self.nav_goal_active:
            # A locally rejected/invalid manager candidate must not consume
            # the next full RRT-priority window.
            self._rrt_wait_started_at = None
        return result

    def _fallback_cell_has_clearance(self, mx, my, clearance=None):
        """Cheap target-clearance check before selecting a fallback goal.

        2026-10-09 (operator): `clearance` lets the caller use a different disc radius.  The
        return-home path passes return_home_goal_clearance; every other caller keeps the
        exploration-grade self.goal_clearance unchanged.
        """
        if clearance is None:
            clearance = self.goal_clearance
        if self.last_map is None:
            return False
        info = self.last_map.info
        resolution = info.resolution
        if resolution <= 0.0:
            return False
        clearance_cells = max(
            1, int(math.ceil(clearance / resolution)))
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if math.hypot(dx * resolution, dy * resolution) > clearance:
                    continue
                nx = mx + dx
                ny = my + dy
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    continue
                if self.last_map.data[ny * info.width + nx] >= 50:
                    return False
        return True

    def _pick_fallback_goal(self):
        """Pick a map-valid fallback without dispatching random unsafe cells."""
        current = self.update_pose()
        if current is None or self.last_map is None:
            return None
        info = self.last_map.info
        resolution = info.resolution
        if resolution <= 0.0:
            return None
        data = self.last_map.data
        width = info.width
        height = info.height
        origin = info.origin.position
        cx = current.pose.position.x
        cy = current.pose.position.y
        step = max(2, min(width, height) // 80)

        def collect(min_distance, max_distance, ignore_recent_failed=False):
            """扫出距离窗内的候选。

            2026-10-13 P6：新增 ignore_recent_failed。
            为什么需要它：goal_recently_failed 的半径是 failed_goal_radius=1.5m
            （launch 亦为 1.5），而车停在原点时可达域半径往往 <1.5m ——
            于是**所有候选都被失败记忆滤掉**，导致 collect() 恒为空、
            放宽窗也救不回来（实测 no candidate cell 11 次 / holding position 2 次）。
            放宽窗（脱困路径）必须能看到这些候选，否则就是死锁。
            """
            cells = []
            # P31：阈值改为运行时读取的参数（默认 65），热调自下一拍生效。
            free_cost = self._sampler_free_cost()
            for my in range(2, height - 2, step):
                row = my * width
                for mx in range(2, width - 2, step):
                    value = data[row + mx]
                    if value < 0 or value >= free_cost:
                        continue
                    x = origin.x + (mx + 0.5) * resolution
                    y = origin.y + (my + 0.5) * resolution
                    distance = math.hypot(x - cx, y - cy)
                    if distance < min_distance or distance > max_distance:
                        continue
                    if not ignore_recent_failed and self.goal_recently_failed(x, y):
                        continue
                    cells.append((x, y, distance, mx, my))
            return cells

        # 2026-10-10: 把两个距离窗的候选数记下来，便于判断"区域太小导致
        # 首选窗为空"还是"候选存在但全被验证拒掉"。
        window = 'preferred'
        # 2026-10-13 P4 修正：relax_window 必须在所有分支之前初始化。
        # 原写法只在"两窗皆空"分支里赋值，正常路径下后面 `if relax_window:`
        # 会抛 UnboundLocalError，而调用方用 _safe_call 包着 → 异常被吞
        # → 整个兜底采样静默失效。
        relax_window = False
        candidates = collect(self.fallback_min_distance_m,
                             self.fallback_max_distance_m)
        preferred_count = len(candidates)
        wide_count = 0
        if not candidates:
            window = 'wide'
            candidates = collect(self.fallback_wide_min_distance_m,
                                 self.fallback_wide_max_distance_m)
            wide_count = len(candidates)
        if not candidates:
            # 2026-10-13 P4：两个标准窗都空 → 不再直接停车（那会造成
            # "车不动 → 地图不扩展 → 候选仍为 0"的死锁），改用脱困式放宽窗：
            # 下限降到 exploration_fallback_relax_min_m，上限放开。
            if bool(getattr(self, 'exploration_fallback_relax_enabled', True)):
                relax_min = float(getattr(
                    self, 'exploration_fallback_relax_min_m', 0.5))
                # 2026-10-13 P6 缺陷1：脱困窗必须忽略失败记忆。
                # 否则 failed_goal_radius(1.5m) >= 车当前可达半径时，
                # collect() 恒为空 —— 失败记忆把车锁死在自己附近。
                _ignore_hist = bool(getattr(
                    self, 'exploration_fallback_relax_ignore_history', True))
                # 2026-10-13 P10 诊断：放宽窗为何仍为空？
                # 已知 no candidate cell 打印在放宽分支之后，说明放宽窗执行过
                # 但 collect(0.5, inf) 返回空。这里把三个可能的解释一次性打出：
                #   · relax_enabled / ignore_hist 的实际值
                #   · 标准窗候选数 vs 放宽窗候选数
                #   · 车所在格的 /map 值（若为 -1 说明车周围都还是未知）
                _dbg_exc = None
                try:
                    candidates = collect(relax_min, float('inf'),
                                         ignore_recent_failed=_ignore_hist)
                except Exception as _e:  # noqa: BLE001
                    _dbg_exc = repr(_e)
                    candidates = []
                # 按"结论"去重：只有结论变化时才打印，既能看趋势又不刷屏。
                # （_p1_safe_log 的 once 是按实例属性去重，只会打第一条，
                #   对"想看候选数随地图长大的变化"不够用。）
                _dbg_key = (bool(getattr(self, 'exploration_fallback_relax_enabled', None)),
                            _ignore_hist, preferred_count, wide_count,
                            len(candidates), width, height, _dbg_exc)
                if getattr(self, '_p10_relax_diag_key', None) != _dbg_key:
                    try:
                        self._p10_relax_diag_key = _dbg_key
                    except Exception:  # noqa: BLE001
                        pass
                    self._p1_safe_log(
                        'warn',
                        'P10 relax diag: enabled=%s ignore_hist=%s relax_min=%.2f '
                        'preferred=%d wide=%d relax=%d map=%dx%d robot=(%.2f,%.2f) '
                        'robot_cell_value=%s collect_exc=%s'
                        % (_dbg_key[0], _ignore_hist, relax_min,
                           preferred_count, wide_count, len(candidates),
                           width, height, cx, cy,
                           self._p4_cell_value(cx, cy), _dbg_exc))
                if candidates:
                    if _ignore_hist:
                        try:
                            _hist = list(getattr(self, 'failed_goals', [])) + \
                                list(getattr(self, 'blocked_places', []))
                            _near = ['(%.2f, %.2f)' % (gx, gy)
                                     for gx, gy, _t in _hist
                                     if math.hypot(gx - cx, gy - cy)
                                     <= float(getattr(self, 'failed_goal_radius', 1.5))]
                            self._p1_safe_log(
                                'warn',
                                'P6 relax window IGNORED %d recent-failure '
                                'marker(s) near the robot (%s); otherwise every '
                                'candidate would be filtered out and the robot '
                                'would be locked in place by its own history'
                                % (len(_near), ' '.join(_near[:6])),
                                once='_p6_relax_ignored_logged')
                        except Exception:  # noqa: BLE001
                            pass
                if candidates:
                    relax_window = True
                    window = 'relax(%.1f-∞)' % relax_min
                    _window_desc = (
                        'preferred %.1f-%.1fm=%d, wide %.1f-%.1fm=%d'
                        % (self.fallback_min_distance_m,
                           self.fallback_max_distance_m, preferred_count,
                           self.fallback_wide_min_distance_m,
                           self.fallback_wide_max_distance_m, wide_count))
                    self.get_logger().warn(
                        'fallback: both standard windows empty (%s) at '
                        'pose=(%.2f, %.2f); RELAXING to %.1f-infm and '
                        'heading for the farthest reachable cell to break '
                        'the standstill deadlock'
                        % (_window_desc, cx, cy, relax_min))
            if not candidates:
                self.get_logger().warn(
                    'fallback: no candidate cell in either window '
                    '(preferred %.1f-%.1fm=%d, wide %.1f-%.1fm=%d) at pose=(%.2f, %.2f)'
                    % (self.fallback_min_distance_m,
                       self.fallback_max_distance_m, preferred_count,
                       self.fallback_wide_min_distance_m,
                       self.fallback_wide_max_distance_m, wide_count,
                       cx, cy))
                return None

        # Randomize first so that equal-scoring candidates stay shuffled (keeps
        # coverage from collapsing onto one line), then rank by a CONTINUOUS key.
        #
        # 2026-10-10: 原实现只把候选分成"角度差<60 度 / >=60 度"两档，同档内
        # 仍是随机——于是"首选档"里可以出现 120 度扇形内任意方位，取第一个
        # 通过验证的就成了随机目标，实测 turn 均值 81-95 度、>120 度占 17-44%。
        # 现改为连续排序键：角度差(rad) * heading_weight + 距离(m) * 距离权重。
        # 效果是优先"与上次大致同向、且更近"的目标，从机制上压制掉头。
        random.shuffle(candidates)
        _heading_weight = max(0.0, float(
            self.get_parameter('fallback_heading_weight').value))
        if relax_window:
            # 脱困窗：距离项取负 → 优先"可达域内最远"的点，强制车离开原地
            _distance_weight = -1.0 / max(1e-3, self.fallback_max_distance_m)
        else:
            _distance_weight = (1.0 / max(1e-3, self.fallback_max_distance_m))

        # 2026-10-13 P3-2：连续拒发后优先转向"相对方向"。
        # 实现方式：把排序基准换成 last_explore_heading + π，原来的
        # "优先与基准同向" 逻辑即变为 "优先与上次方向相反（角度差接近 180°）"。
        _rank_basis = self.last_explore_heading
        if getattr(self, '_explore_direction_flip', False) and _rank_basis is not None:
            _rank_basis = math.atan2(math.sin(_rank_basis + math.pi),
                                     math.cos(_rank_basis + math.pi))

        # 2026-10-13 P27（方案A）：转角软约束。
        # 现有 heading 项用的是 last_explore_heading（**上次探索方向**，
        # 且 flip 时为 None → 整项失效），与"车现在朝哪"无关；这里补上
        # "相对当前车头的转角"惩罚。位姿只取一次，避免逐候选重复调用。
        _turn_cur = self.update_pose()
        # 惩罚项与距离项同量纲（归一化到 fallback_max_distance_m），
        # 且**恒为正向**：放宽窗里 _distance_weight 为负（优先最远点），
        # 若跟着取负会把"大转角"变成优先，反而制造掉头。
        # P31：每拍读一次参数（不是每个候选读），热调自下一拍生效。
        _turn_penalty_norm = (
            self._turn_penalty_weight()
            / max(1e-3, self.fallback_max_distance_m))
        if not self._turn_penalty_on():
            _turn_penalty_norm = 0.0

        def _rank_key(candidate):
            rank = candidate[2] * _distance_weight
            if _rank_basis is not None:
                heading = math.atan2(candidate[1] - cy, candidate[0] - cx)
                difference = abs(math.atan2(
                    math.sin(heading - _rank_basis),
                    math.cos(heading - _rank_basis)))
                rank += difference * _heading_weight
            if _turn_penalty_norm > 0.0:
                _turn = self._turn_from_pose_to(
                    _turn_cur, candidate[0], candidate[1])
                if _turn is not None:
                    rank += (_turn / math.pi) * _turn_penalty_norm
            return rank

        candidates.sort(key=_rank_key)
        # 2026-10-12 A：记录本拍排序后的候选，供规划器预验证逐个尝试。
        # 机制在父类 StemmNav2Manager._prevalidate_*，这里只负责记录。
        try:
            self._fallback_candidate_order = [
                (float(c[0]), float(c[1])) for c in candidates]
            self._fallback_candidate_pose = (float(cx), float(cy))
        except Exception:  # noqa: BLE001
            self._fallback_candidate_order = []

        rejected_clearance = 0
        rejected_path = 0
        rejected_unreachable = 0
        rejected_r1d = 0
        _require_reach = bool(getattr(
            self, 'exploration_fallback_require_reachable', True))
        for x, y, _distance, mx, my in candidates:
            # 2026-10-13 P3-1：跳过近期被 R1d 拒发过的候选（修反馈断裂）
            if self._r1d_reject_seen(x, y):
                rejected_r1d += 1
                continue
            if not self._fallback_cell_has_clearance(mx, my):
                rejected_clearance += 1
                continue
            if self.path_blocked_ratio(cx, cy, x, y) >= 0.10:
                rejected_path += 1
                continue
            # 2026-10-12 N-11：兜底候选同样必须与机器人同属一个可通行连通域。
            # 实测失败目标几乎全部出自这里 —— 旧实现只查净空与直线
            # 通畅，于是把"地图自由但被膨胀隔断"的点派了出去，
            # 每个耗满 50s 硬上限。三态判定：False 才拒，None 放行。
            if _require_reach:
                _reach = self._safe_call(
                    'fallback reachability',
                    lambda: self._explore_cell_is_reachable(x, y), None)
                if _reach is False:
                    rejected_unreachable += 1
                    continue
            bearing = math.atan2(y - cy, x - cx)
            self.get_logger().info(
                'fallback pick: window=%s candidates=%d (preferred=%d wide=%d) '
                'rejected(clearance=%d path=%.0f%%=%d unreachable=%d r1d=%d) '
                'chosen=(%.2f, %.2f) '
                'dist=%.2fm bearing=%.0fdeg robot=(%.2f, %.2f)%s'
                % (window, len(candidates), preferred_count, wide_count,
                   rejected_clearance, 10.0, rejected_path,
                   rejected_unreachable, rejected_r1d,
                   x, y, _distance, math.degrees(bearing), cx, cy,
                   ' [FLIP]' if getattr(self, '_explore_direction_flip', False) else ''))
            self._note_goal_bearing(bearing, x, y)
            self._note_explore_dispatch_success()
            return self.make_pose_goal(x, y, bearing)
        self.get_logger().warn(
            'fallback: all %d candidates rejected (clearance=%d, path=%.0f%%=%d), '
            'window=%s robot=(%.2f, %.2f)'
            % (len(candidates), rejected_clearance, 10.0, rejected_path,
               window, cx, cy))
        return None

    def _note_goal_bearing(self, bearing, x, y):
        """2026-10-10: 兜转检测。连续 N 次目标方位角反转超过阈值即告警。

        这是"小区域内兜转死循环"的核心可观测点：此前日志里只有 target 坐标，
        没有任何地方显式记录"机器人被反复派往相反方向"。
        """
        previous = getattr(self, '_fallback_last_bearing', None)
        streak = getattr(self, '_fallback_reversal_streak', 0)
        if previous is not None:
            delta = abs(math.atan2(math.sin(bearing - previous),
                                   math.cos(bearing - previous)))
            threshold = math.radians(
                float(self.get_parameter('fallback_reversal_min_deg').value))
            if delta >= threshold:
                streak += 1
            else:
                streak = 0
        self._fallback_last_bearing = bearing
        self._fallback_reversal_streak = streak
        alert_at = int(self.get_parameter(
            'fallback_reversal_streak_alert').value)
        if alert_at > 0 and streak >= alert_at:
            self.get_logger().warn(
                'fallback THRASHING: %d consecutive goal reversals >= %.0f deg; '
                'latest target=(%.2f, %.2f). The robot is being sent back and forth '
                'instead of making progress.'
                % (streak,
                   float(self.get_parameter('fallback_reversal_min_deg').value),
                   x, y))
            self._fallback_reversal_streak = 0      # 复位，避免刷屏

    def _pull_goal_into_known_free(self, pose):
        """A: move a frontier goal back towards the robot while the cells stay known-free.

        Only ever moves the goal CLOSER to the robot (never further), at most
        frontier_pullback_m, and stops at the first cell that is not known-free.  The result is a
        goal the vehicle can stand at without being pinned to the map boundary.
        """
        if self.last_map is None:
            return pose
        try:
            pull = abs(float(self.get_parameter('frontier_pullback_m').value))
        except Exception:
            pull = 1.0
        if pull <= 0.0:
            return pose
        info = self.last_map.info
        res = info.resolution
        if res <= 0.0:
            return pose
        ox, oy = info.origin.position.x, info.origin.position.y
        current = self.update_pose()
        if current is None:
            return pose
        rx, ry = current.pose.position.x, current.pose.position.y
        tx, ty = pose.pose.position.x, pose.pose.position.y
        dist = math.hypot(tx - rx, ty - ry)
        if dist <= 1e-3:
            return pose
        ux, uy = (tx - rx) / dist, (ty - ry) / dist
        moved = 0.0
        best_x, best_y = tx, ty
        steps = max(1, int(pull / max(res, 0.02)))
        for i in range(1, steps + 1):
            d = pull * i / steps
            if d >= dist:
                break
            px, py = tx - ux * d, ty - uy * d
            mx, my = int((px - ox) / res), int((py - oy) / res)
            if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                break
            value = self.last_map.data[my * info.width + mx]
            if value < 0 or value >= 50:          # unknown or occupied: stop pulling here
                break
            best_x, best_y, moved = px, py, d
        if moved <= 0.0:
            return pose
        self.get_logger().warn(
            'A frontier pull-back: goal (%.2f, %.2f) -> (%.2f, %.2f), %.2f m towards the robot'
            % (tx, ty, best_x, best_y, moved))
        new_pose = type(pose)()
        new_pose.header = pose.header
        new_pose.pose = pose.pose
        new_pose.pose.position.x = best_x
        new_pose.pose.position.y = best_y
        return new_pose

    def _approach_corridor_is_clear(self, pose):
        """B: the first part of the route must keep the front gate's clearance.

        The front gate refuses forward motion closer than
        front_stop_distance + blocked_clearance_hysteresis_m; a goal whose approach needs to be
        inside that band can never be driven to, so it is not dispatched at all.
        """
        if self.last_map is None:
            return True
        try:
            probe = abs(float(self.get_parameter('approach_probe_m').value))
        except Exception:
            probe = 1.2
        if probe <= 0.0:
            return True
        info = self.last_map.info
        res = info.resolution
        if res <= 0.0:
            return True
        ox, oy = info.origin.position.x, info.origin.position.y
        current = self.update_pose()
        if current is None:
            return True
        rx, ry = current.pose.position.x, current.pose.position.y
        tx, ty = pose.pose.position.x, pose.pose.position.y
        dist = math.hypot(tx - rx, ty - ry)
        if dist <= probe:
            return True                     # very short hop: the front gate itself already guards it
        ux, uy = (tx - rx) / dist, (ty - ry) / dist
        gate = self.front_stop_distance + self.blocked_clearance_hysteresis_m
        half_width = 0.35
        steps = max(2, int(probe / max(res, 0.02)))
        for i in range(1, steps + 1):
            d = probe * i / steps
            cx = rx + ux * d
            cy = ry + uy * d
            for lateral in (-half_width, 0.0, half_width):
                px = cx - uy * lateral
                py = cy + ux * lateral
                mx, my = int((px - ox) / res), int((py - oy) / res)
                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    self.get_logger().warn(
                        'B approach corridor rejected: route leaves the map at %.2f m' % d)
                    return False
                if self.last_map.data[my * info.width + mx] >= 50:
                    self.get_logger().warn(
                        'B approach corridor rejected: obstacle within %.2f m of the start '
                        '(needs %.2f m of clearance for %0.1f m)' % (d, gate, probe))
                    return False
        return True

    def _home_rear_is_clear(self, tx, ty, yaw, rear=0.50):
        """2026-10-09: the rear-space half of R1d, extracted so the home search can pre-check
        it; a candidate that fails it would only be rejected later by _goal_target_is_viable."""
        if self.last_map is None:
            return True
        info = self.last_map.info
        res = info.resolution
        if res <= 0.0:
            return True
        ox, oy = info.origin.position.x, info.origin.position.y
        bx = tx - rear * math.cos(yaw)
        by = ty - rear * math.sin(yaw)
        bmx, bmy = int((bx - ox) / res), int((by - oy) / res)
        if 0 <= bmx < info.width and 0 <= bmy < info.height:
            return self.last_map.data[bmy * info.width + bmx] < 50
        return True

    # ==== 2026-10-10: 返航起终点连通性门禁 + 证据采集 ====
    def _map_cell_index(self, x, y):
        """Return the flat index of (x, y) on the current /map, or None."""
        if self.last_map is None:
            return None
        info = self.last_map.info
        res = float(info.resolution)
        if res <= 0.0:
            return None
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        # 2026-10-10: 用 round 而非 int()。实测 y=0.95 / res=0.05 得到
        # 18.999999999999996，int() 截断为 18，落进相邻栅格；在一整格边界上
        # 会系统性偏移一格。round 取最近格，与 Nav2 的 worldToMap 一致。
        mx = int(round((float(x) - ox) / res))
        my = int(round((float(y) - oy) / res))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return my * info.width + mx


    # ==== 2026-10-12 N-9：探索目标的 Nav2 权威可达性判据 ====
    def _init_exploration_costmap_client(self):
        """建立 /global_costmap/get_costmap 客户端（失败不致命，判据会回退）。"""
        if not getattr(self, 'exploration_costmap_gate', False):
            return
        try:
            from nav2_msgs.srv import GetCostmap  # noqa: F401
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                'exploration costmap gate disabled: nav2_msgs/GetCostmap '
                'unavailable (%s); falling back to the map-only check' % exc)
            self.exploration_costmap_gate = False
            return
        try:
            self._costmap_client = self.create_client(
                GetCostmap, self.exploration_costmap_service)
            self.create_timer(
                self.exploration_costmap_fetch_sec, self._costmap_fetch_tick)
            self.create_timer(0.1, self._costmap_reap_future)
            self.get_logger().info(
                'exploration goal gate now uses the Nav2 global costmap via %s '
                '(traversable = cost < %d)'
                % (self.exploration_costmap_service,
                   self.exploration_costmap_lethal_cost))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                'exploration costmap client setup failed (%s); falling back to '
                'the map-only check' % exc)
            self._costmap_client = None
            self.exploration_costmap_gate = False

    def _costmap_reap_future(self):
        """非阻塞地取回上一拍的服务响应。"""
        fut = getattr(self, '_costmap_future', None)
        if fut is None or not fut.done():
            return
        self._costmap_future = None
        try:
            res = fut.result()
        except Exception:  # noqa: BLE001 - 判定绝不能阻断探索
            return
        msg = getattr(res, 'map', None)
        if msg is None or not getattr(msg, 'data', None):
            return
        self._costmap_msg = msg
        self._costmap_at = time.monotonic()

    def _costmap_fetch_tick(self):
        if not getattr(self, 'exploration_costmap_gate', False):
            return
        client = getattr(self, '_costmap_client', None)
        if client is None or getattr(self, '_costmap_future', None) is not None:
            return
        try:
            if not client.service_is_ready():
                return
            from nav2_msgs.srv import GetCostmap
            self._costmap_future = client.call_async(GetCostmap.Request())
        except Exception:  # noqa: BLE001
            self._costmap_future = None

    def _costmap_usable(self):
        """返回可用的 costmap 帧；无则 None（调用方必须回退）。"""
        if not getattr(self, 'exploration_costmap_gate', False):
            return None
        msg = getattr(self, '_costmap_msg', None)
        if msg is None:
            return None
        info = getattr(msg, 'metadata', None)
        if info is None or int(getattr(info, 'size_x', 0)) <= 0:
            return None
        if int(getattr(info, 'size_y', 0)) <= 0:
            return None
        if (time.monotonic() - float(getattr(self, '_costmap_at', 0.0))
                > 5.0 * self.exploration_costmap_fetch_sec):
            # 太久没刷新：宁可回退，也不要用陈旧代价地图误拒目标
            return None
        return msg

    def _costmap_cell_index(self, info, x, y):
        res = float(info.resolution)
        if res <= 0.0:
            return None
        size_x, size_y = int(info.size_x), int(info.size_y)
        mx = float(info.origin.position.x)
        my = float(info.origin.position.y)
        # nav2_msgs/GetCostmap.metadata 的原点位于左下角，行自下而上
        col = int((float(x) - mx) / res)
        row = int((float(y) - my) / res)
        if col < 0 or row < 0 or col >= size_x or row >= size_y:
            return None
        return row * size_x + col

    def _reachable_dist_field(self):
        """2026-10-13 P1：/map 上每个格到最近占用格的欧氏距离（米）。

        缓存键 = (地图签名, 车辆内切半径)。地图未变时直接复用，
        避免在 25 Hz 的决策循环里反复做距离变换。
        异常/无地图一律返回 None（调用方必须放行）。
        """
        try:
            if self.last_map is None:
                return None
            info = self.last_map.info
            data = self.last_map.data
            width, height = int(info.width), int(info.height)
            resolution = float(info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0 or not data:
                return None
            clearance = float(getattr(
                self, 'exploration_reach_clearance_m', 0.328))
            import numpy as _np
            # 注意：data 是 int8[]，含 -1（未知），**不能**用 bytes(data)——
            # bytes() 走的是"整数序列"语义，对负数抛 ValueError。
            # 必须用 ndarray.tobytes() 才能安全地拿字节做签名。
            arr = _np.asarray(data, dtype=_np.int8)
            if arr.size < width * height:
                return None
            arr = arr[:width * height].reshape(height, width)
            signature = (width, height, resolution,
                         int(round(clearance * 1e6)),
                         int(getattr(self, 'exploration_reach_obstacle_cost', 65)),
                         hash(arr.tobytes()))
            cached = getattr(self, '_reach_cache_field', None)
            if cached is not None and cached[0] == signature:
                return cached[1]
            # 占用/不可通行 = value >= exploration_reach_obstacle_cost
            # （默认 30，与采样器 collect 的 value<30 口径统一）。
            # 未知(-1) 不是占用格：它另由 _map_point_passable/passable 排除，
            # 不应参与距离场（否则未知区会把障碍距离压小）。
            _obstacle_cost = int(getattr(
                self, 'exploration_reach_obstacle_cost', 65))
            occ = (arr >= _obstacle_cost)
            field = self._exact_edt_cells(occ) * resolution
            self._reach_cache_field = (signature, field)
            return field
        except Exception:  # noqa: BLE001 - 判定绝不能阻断探索
            return None

    @staticmethod
    def _exact_edt_cells(occ):
        """精确欧氏距离变换（返回每个格到最近 True 格的欧氏距离，单位: 格）。

        标准做法：先沿一个轴做 1D DT，再沿另一个轴做 1D DT。
        纯 numpy，无 scipy 依赖；258k 格约 1 秒，且有缓存。
        """
        import numpy as _np
        height, width = occ.shape
        big = 10 ** 12
        # 第 1 遍：沿 width 方向做 1D DT（每次一批行）
        g = _np.where(occ, 0, big).astype(_np.int64)
        idx_w = _np.arange(width, dtype=_np.int64)
        d1 = _np.empty((height, width), dtype=_np.int64)
        batch = 64
        for start in range(0, height, batch):
            block = g[start:start + batch]                 # (b, w)
            vals = ((idx_w.reshape(1, 1, width)
                     - idx_w.reshape(1, width, 1)) ** 2) + block[:, None, :]
            d1[start:start + batch] = vals.min(axis=2)     # (b, w)
        # 第 2 遍：沿 height 方向做 1D DT（每次一批列）
        #   d2[r, c] = min_h ( (r-h)^2 + d1[h, c] )
        idx_h = _np.arange(height, dtype=_np.int64)
        d2 = _np.empty((height, width), dtype=_np.int64)
        for cs in range(0, width, batch):
            block = d1[:, cs:cs + batch]                   # (h, b)
            # (h, h, 1) 与 (1, h, b) 广播 -> (h, h, b)，对 h 轴取 min
            vals = ((idx_h.reshape(height, 1, 1)
                     - idx_h.reshape(1, height, 1)) ** 2) + block[None, :, :]
            d2[:, cs:cs + batch] = vals.min(axis=1)        # (h, b)
        out = _np.sqrt(d2.astype(_np.float64))
        # 饱和：没有障碍时 d2 会保留 big，开方后是 1e6 量级，必须换成 inf
        out[_np.sqrt(d2) > 1.0e5] = _np.inf
        return out

    # ==== 2026-10-13 P3：R1d 拒发反馈 + 方向粘性重置 ====

    def _r1d_reject_init(self):
        """惰性初始化拒发存根与方向状态（避免依赖 __init__ 改动）。"""
        if not hasattr(self, '_r1d_rejected'):
            self._r1d_rejected = {}          # key -> monotonic
            self._r1d_consecutive = 0
            self._r1d_reject_events = 0      # 本轮累计（诊断）
            self._explore_direction_flip = False

    def _r1d_reject_note(self, pose, kind):
        """记录一次 R1d 拒发（只对探索目标），并推进"方向粘性重置"状态机。

        连续拒发达 exploration_direction_flip_after（默认 2）次 → 重置 last_explore_heading
        并进入 flip 模式；之后每再拒发 exploration_direction_flip_hold_rejections 次
        仍然维持 flip（持续尝试相对方向），直到成功派发一次才退出。
        """
        try:
            self._r1d_reject_init()
            if not self._is_exploration_goal(kind):
                return
            self._r1d_reject_events += 1
            try:
                self._r1d_rejected[
                    (self._prevalidate_map_signature(),
                     round(float(pose.pose.position.x), 2),
                     round(float(pose.pose.position.y), 2))] = time.monotonic()
            except Exception:  # noqa: BLE001
                pass
            self._r1d_consecutive += 1
            # 2026-10-13 P21（丙）：解锁口径从"仅 R1d"扩为
            # r1d / planner / reach 三类"确定不可达"（见 _note_unreachable_rejection），
            # 但**不跨类型求和** —— 阈值仍是连续 2 次。
            self._note_unreachable_rejection('r1d')
        except Exception:  # noqa: BLE001 - 诊断/状态绝不能阻断派发
            return

    def _r1d_reject_seen(self, world_x, world_y):
        """该候选是否近期被 R1d 拒发过（按地图签名 + 0.25m 网格近似）。"""
        try:
            self._r1d_reject_init()
            if not self._r1d_rejected:
                return False
            ttl = getattr(self, 'exploration_r1d_reject_ttl_sec', 60.0)
            now = time.monotonic()
            # 先按 TTL 清理
            if len(self._r1d_rejected) > 4096:
                self._r1d_rejected = {
                    k: v for k, v in self._r1d_rejected.items() if now - v <= ttl}
            sig = self._prevalidate_map_signature()
            gx = round(float(world_x) * 4.0) / 4.0      # 0.25 m 网格
            gy = round(float(world_y) * 4.0) / 4.0
            for dx in (-0.25, 0.0, 0.25):
                for dy in (-0.25, 0.0, 0.25):
                    hit = self._r1d_rejected.get(
                        (sig, round(gx + dx, 2), round(gy + dy, 2)))
                    if hit is not None and now - hit <= ttl:
                        return True
            return False
        except Exception:  # noqa: BLE001
            return False

    def _note_explore_dispatch_success(self):
        """成功派发一次探索目标 → 退出 flip 模式、清空连续拒发计数。"""
        try:
            self._r1d_reject_init()
            self._r1d_consecutive = 0
            # 2026-10-13 P21：同步清零多类型连续计数与转角拦截计数
            self._planner_consecutive = 0
            self._reach_consecutive = 0
            self._turn_blocked_count = 0
            if getattr(self, '_explore_direction_flip', False):
                self._explore_direction_flip = False
                self.get_logger().info(
                    'direction flip mode cleared after a successful dispatch')
        except Exception:  # noqa: BLE001
            return

    def _p4_cell_value(self, world_x, world_y):
        """诊断用：读 /map 上该世界坐标的值（未知 -1 / 占用 >=65 / 自由 0~64）。"""
        try:
            if self.last_map is None:
                return 'no-map'
            info = self.last_map.info
            res = float(info.resolution)
            ox = float(info.origin.position.x)
            oy = float(info.origin.position.y)
            mx = int((float(world_x) - ox) / res)
            my = int((float(world_y) - oy) / res)
            if mx < 0 or my < 0 or mx >= int(info.width) or my >= int(info.height):
                return 'out-of-map'
            cell = my * int(info.width) + mx
            data = self.last_map.data
            if not (0 <= cell < len(data)):
                return 'bad-cell'
            return int(data[cell])
        except Exception:  # noqa: BLE001
            return 'err'

    def _map_point_passable(self, world_x, world_y):
        """2026-10-13 P1：该世界点"车能不能过"（三态）。

        True  = 地图已知自由，且到最近占用格 >= 车辆内切半径
        False = 未知格 / 占用格 / 离障碍太近
        None  = 无法判定（无地图/出图/异常）→ 调用方必须放行
        """
        try:
            if self.last_map is None:
                return None
            info = self.last_map.info
            data = self.last_map.data
            width, height = int(info.width), int(info.height)
            resolution = float(info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0:
                return None
            origin_x = float(info.origin.position.x)
            origin_y = float(info.origin.position.y)
            col = int(round((float(world_x) - origin_x) / resolution))
            row = int(round((float(world_y) - origin_y) / resolution))
            if col < 0 or row < 0 or col >= width or row >= height:
                return None
            cell = row * width + col
            if not (0 <= cell < len(data)):
                return None
            value = int(data[cell])
            if value < 0:
                return False                      # 未知：不认（这就是根因所在）
            if value >= 65:
                return False                      # 占用
            field = self._reachable_dist_field()
            if field is None:
                return None
            clearance = float(getattr(
                self, 'exploration_reach_clearance_m', 0.328))
            return float(field[row, col]) >= clearance
        except Exception:  # noqa: BLE001 - 判定绝不能阻断探索
            return None

    def _costmap_traversable(self, data, cell):
        """Nav2 全局 costmap 判据（+ 2026-10-13 P1：再要求"车物理上能过"）。

        cost >= lethal(253) -> 不可通行（含 254 LETHAL）
        cost == 255 (NO_INFORMATION) -> 与 allow_unknown: true 一致，放行
        其余（含 cost==0）-> 再问 _map_point_passable：
            未知格 / 占用格 / 离障碍 < 内切半径 一律判不可通行
        `exploration_reach_respect_map=False` 时跳过第二层（逐位回退旧行为）。
        """
        try:
            cost = int(data[cell])
        except Exception:  # noqa: BLE001
            return None
        lethal = int(getattr(self, 'exploration_costmap_lethal_cost', 253))
        if cost == 255:
            # NO_INFORMATION：与 allow_unknown: true 保持一致（可通行）
            return True
        if cost >= lethal:
            return False
        if not getattr(self, 'exploration_reach_respect_map', True):
            return True               # 旧行为：仅看 costmap
        msg = getattr(self, '_costmap_msg', None)
        info = getattr(msg, 'metadata', None) if msg is not None else None
        if info is None:
            return True               # 拿不到 costmap 元数据：fail-open
        size_x = int(getattr(info, 'size_x', 0))
        res = float(getattr(info, 'resolution', 0.0))
        if size_x <= 0 or res <= 0.0:
            return True
        row = cell // size_x
        col = cell - row * size_x
        world_x = float(info.origin.position.x) + (col + 0.5) * res
        world_y = float(info.origin.position.y) + (row + 0.5) * res
        verdict = self._map_point_passable(world_x, world_y)
        if verdict is None:
            return True               # 无法判定：fail-open
        return verdict

    def _reachable_costmap_cells(self):
        """在 Nav2 全局 costmap 上做 BFS，返回机器人所在可通行连通域的 cell 集合。

        costmap 不可用时返回 None（调用方回退到地图占用判据）。
        """
        msg = self._costmap_usable()
        if msg is None:
            return None
        data = msg.data
        info = msg.metadata
        size_x, size_y = int(info.size_x), int(info.size_y)
        if not data or len(data) < size_x * size_y:
            return None
        pose = self.update_pose()
        if pose is None:
            return None
        start_cell = self._costmap_cell_index(
            info, pose.pose.position.x, pose.pose.position.y)
        if start_cell is None:
            return None
        if self._costmap_traversable(data, start_cell) is not True:
            # 机器人当前格被判不可通行：无权判定，交回既有闸门
            return None
        # 注意：costmap data 是 uint8（0~255），bytes() 的"整数序列"重载
        # 在遇到负值时抛 ValueError；统一走 numpy 的 tobytes()。
        import numpy as _npc
        _cm_arr = _npc.asarray(data, dtype=_npc.uint8)
        signature = (
            'costmap', size_x, size_y, float(info.resolution),
            float(info.origin.position.x), float(info.origin.position.y),
            hash(_cm_arr.tobytes()), start_cell)
        cached = getattr(self, '_explore_reach_cache', None)
        if cached is not None and cached[0] == signature:
            return cached[1]
        cells = set()
        queue = deque([start_cell])
        seen = bytearray(size_x * size_y)
        seen[start_cell] = 1
        limit = max(1000, int(getattr(
            self, 'exploration_reach_max_cells', 400000)))
        while queue:
            cell = queue.popleft()
            cells.add(cell)
            if len(cells) >= limit:
                break
            row, col = divmod(cell, size_x)
            for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nrow, ncol = row + drow, col + dcol
                if nrow < 0 or ncol < 0 or nrow >= size_y or ncol >= size_x:
                    continue
                ncell = nrow * size_x + ncol
                if seen[ncell]:
                    continue
                seen[ncell] = 1
                if self._costmap_traversable(data, ncell) is True:
                    queue.append(ncell)
        self._explore_reach_cache = (signature, cells)
        return cells

    def _p1_safe_log(self, level, message, once=None):
        """2026-10-13 P1：诊断日志的安全包装。

        诊断代码绝不能影响判据：任何异常（节点未初始化、logger 不可用等）
        都必须被吞掉，且不影响调用方的返回值。
        `once` 给出实例属性名，用于同一条日志只打一次。
        """
        try:
            if once is not None:
                if getattr(self, once, False):
                    return
                try:
                    setattr(self, once, True)
                except Exception:  # noqa: BLE001
                    pass
            logger = None
            try:
                logger = self.get_logger()
            except Exception:  # noqa: BLE001
                logger = None
            if logger is None:
                return
            fn = getattr(logger, level, None)
            if fn is None:
                return
            fn(message)
        except Exception:  # noqa: BLE001 - 诊断绝不能抛出
            return

    def _reachable_map_cells(self):
        """2026-10-13 P1：以 /map 障碍距离场为主判据的"机器人可达域"。

        与 _reachable_costmap_cells 的区别：
          · 只用 /map（不依赖 Nav2 costmap 的轮询新鲜度）；
          · 通行判据 = 已知自由 且 到最近占用格 >= 车辆内切半径。
        costmap 判据的致命弱点在 2026-10-13 实测中暴露：global_costmap 建图期
        持续 resize（546→555→558 X 587），"5 秒未刷新即回退"被频繁触发，
        而回退路径（value<50）会把未知格(-1)当自由 → 闸门等于失效。
        返回 None 表示无法判定（调用方必须放行）。
        """
        try:
            if self.last_map is None:
                return None
            info = self.last_map.info
            data = self.last_map.data
            width, height = int(info.width), int(info.height)
            resolution = float(info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0 or not data:
                return None
            field = self._reachable_dist_field()
            if field is None:
                return None
            clearance = float(getattr(
                self, 'exploration_reach_clearance_m', 0.328))
            import numpy as _np
            arr = _np.asarray(data, dtype=_np.int8)
            if arr.size < width * height:
                return None
            arr = arr[:width * height].reshape(height, width)
            # 已知自由 = 0 <= value < 65（OccupancyGrid 尺度；-1 是未知，不算）
            _obstacle_cost = int(getattr(
                self, 'exploration_reach_obstacle_cost', 65))
            passable = ((arr >= 0) & (arr < _obstacle_cost)
                        & (field >= clearance))
            pose = self.update_pose()
            if pose is None:
                return None
            start_cell = self._map_cell_index(
                pose.pose.position.x, pose.pose.position.y)
            if start_cell is None:
                return None
            if not (0 <= start_cell < passable.size):
                return None
            if not bool(passable.reshape(-1)[start_cell]):
                # 起点本身不满足判据：无权判定，交给既有闸门
                self._p1_safe_log(
                    'warn',
                    'P1 reach gate: start_cell=%d NOT passable '
                    '(map=%dx%d value=%d dist=%.3f) -> fall back to '
                    'costmap/legacy gate'
                    % (start_cell, width, height,
                       int(arr.reshape(-1)[start_cell]),
                       float(field.reshape(-1)[start_cell])),
                    once='_reach_log_nostart')
                return None
            signature = (width, height, resolution,
                         int(round(clearance * 1e6)),
                         hash(passable.tobytes()), start_cell)
            cached = getattr(self, '_explore_reach_cache', None)
            if cached is not None and cached[0] == signature:
                return cached[1]
            flat = passable.reshape(-1)
            cells = set()
            queue = deque([start_cell])
            seen = bytearray(flat.size)
            seen[start_cell] = 1
            limit = max(1000, int(getattr(
                self, 'exploration_reach_max_cells', 400000)))
            while queue:
                cell = queue.popleft()
                cells.add(cell)
                if len(cells) >= limit:
                    break
                row, col = divmod(cell, width)
                for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nrow, ncol = row + drow, col + dcol
                    if nrow < 0 or ncol < 0 or nrow >= height or ncol >= width:
                        continue
                    ncell = nrow * width + ncol
                    if seen[ncell]:
                        continue
                    seen[ncell] = 1
                    if flat[ncell]:
                        queue.append(ncell)
            self._explore_reach_cache = (signature, cells)
            # 2026-10-13 P1 诊断（只打一次）。★日志绝不能抛异常★：
            # 否则会被外层 except 吞成 fail-open，等于诊断代码改变了判据行为。
            self._p1_safe_log(
                'info',
                'P1 reach gate ACTIVE (map-based): map=%dx%d clearance=%.3f '
                'reachable=%d cells (%.2f m2) start_cell=%d'
                % (width, height, clearance, len(cells),
                   len(cells) * resolution * resolution, start_cell),
                once='_reach_log_ok')
            return cells
        except Exception as exc:  # noqa: BLE001 - 判定绝不能阻断探索
            self._p1_safe_log('warn',
                              'P1 reach gate ERROR -> fail-open: %r' % (exc,),
                              once='_reach_log_err')
            return None

    def _reachable_free_cells(self):
        """2026-10-12: 探索目标的连通性闸门：机器人所在自由连通域。

        2026-10-13 P1 调整判据优先级：
          1) exploration_reach_respect_map=True（默认）→ 先用 _reachable_map_cells()
             （地图障碍距离场；与建图进度一致，不依赖 costmap 新鲜度）
          2) 否则 / 地图判据无法判定时 → 再用 _reachable_costmap_cells()
          3) 再不行 → 回退到旧的"地图占用 <50"BFS（value<50，会把未知当自由）
        返回 None 表示无法判定（调用方必须放行）。
        """
        try:
            if getattr(self, 'exploration_reach_respect_map', True):
                map_cells = self._reachable_map_cells()
                if map_cells is not None:
                    return map_cells
            # 2026-10-12 N-9：再试 Nav2 costmap 判据。
            costmap_cells = self._reachable_costmap_cells()
            if costmap_cells is not None:
                return costmap_cells
            if self.last_map is None:
                return None
            import numpy as _np
            data = self.last_map.data
            info = self.last_map.info
            width, height = int(info.width), int(info.height)
            if width <= 0 or height <= 0 or not data:
                return None
            pose = self.update_pose()
            if pose is None:
                return None
            start_cell = self._map_cell_index(
                pose.pose.position.x, pose.pose.position.y)
            if start_cell is None:
                return None
            if not (0 <= start_cell < len(data)):
                return None
            if int(data[start_cell]) >= 50:
                # 起点本身是占用格：无权判定，交给既有闸门
                return None
            _arr = _np.asarray(data, dtype=_np.int8)[:width * height]
            signature = (width, height, hash(_arr.tobytes()),
                         int(float(info.resolution) * 1e6), start_cell)
            cached = getattr(self, '_explore_reach_cache', None)
            if cached is not None and cached[0] == signature:
                return cached[1]
            cells = set()
            queue = deque([start_cell])
            seen = bytearray(len(data))
            seen[start_cell] = 1
            limit = max(1000, int(getattr(
                self, 'exploration_reach_max_cells', 400000)))
            while queue:
                cell = queue.popleft()
                cells.add(cell)
                if len(cells) >= limit:
                    break
                row, col = divmod(cell, width)
                for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nrow, ncol = row + drow, col + dcol
                    if nrow < 0 or ncol < 0 or nrow >= height or ncol >= width:
                        continue
                    ncell = nrow * width + ncol
                    if seen[ncell]:
                        continue
                    seen[ncell] = 1
                    if int(data[ncell]) < 50:
                        queue.append(ncell)
            self._explore_reach_cache = (signature, cells)
            return cells
        except Exception:  # noqa: BLE001 - 判定绝不能阻断探索
            return None

    def _explore_cell_is_reachable(self, x, y):
        """2026-10-12 N-11：兜底采样用的**布尔**可达性判定（**世界坐标**）。

        True  = 与机器人同属一个可通行连通域（放行）
        False = 可证明不可达（拒发该候选）
        None  = 无法判定（放行，交给既有闸门）

        ★ 必须传世界坐标：/map 与 global_costmap 的 origin 不同，格号体系
        不可互换（`_map_cell_index` 用 /map 的 origin，`_costmap_cell_index`
        用 costmap 的 origin）。这里统一在世界坐标入参，内部各自换算。

        优先用 Nav2 全局 costmap（cost<253，与规划器同判据），costmap 不可用时
        回退到地图占用判据 —— 与 `_reachable_free_cells()` 同一套判据，
        不引入新的语义。
        """
        try:
            if x is None or y is None:
                return None
            tx = float(x)
            ty = float(y)
            cm = self._costmap_usable()
            if cm is not None:
                info = cm.metadata
                cell = self._costmap_cell_index(info, tx, ty)
                if cell is None:
                    return None
                tv = self._costmap_traversable(cm.data, cell)
                if tv is None:
                    return None
                if tv is False:
                    return False
                cells = self._reachable_costmap_cells()
                if cells is None:
                    return None
                return cell in cells
            if self.last_map is None:
                return None
            cell = self._map_cell_index(tx, ty)
            if cell is None:
                return None
            data = self.last_map.data
            if not (0 <= cell < len(data)):
                return None
            if int(data[cell]) >= 50:
                # 目标格被占：交给既有 R1d/goal_is_traversable 判，不抢判定权
                return None
            cells = self._reachable_free_cells()
            if cells is None:
                return None
            return cell in cells
        except Exception:  # noqa: BLE001 - 判定绝不能阻断探索
            return None

    def _explore_target_reject_reason(self, pose):
        """2026-10-12 N-9：返回拒绝理由字符串；None 表示放行。

        与 _explore_target_is_connected 等价，但区分"不同自由域"与"Nav2
        cost 不可通行"，便于回归核对（costmap 判据下二者都表现为不可达）。
        """
        try:
            if pose is None:
                return None
            tx = float(pose.pose.position.x)
            ty = float(pose.pose.position.y)
            cm = self._costmap_usable()
            if cm is not None:
                info = cm.metadata
                cell = self._costmap_cell_index(info, tx, ty)
                if cell is None:
                    return ('target (%.2f, %.2f) is outside the Nav2 global '
                            'costmap' % (tx, ty))
                tv = self._costmap_traversable(cm.data, cell)
                if tv is False:
                    return ('target (%.2f, %.2f) is not traversable on the '
                            'Nav2 global costmap (cost=%d >= %d)'
                            % (tx, ty, int(cm.data[cell]),
                               int(self.exploration_costmap_lethal_cost)))
                if tv is None:
                    return None
                cells = self._reachable_costmap_cells()
                if cells is None:
                    return None
                if cell not in cells:
                    return ('target (%.2f, %.2f) is free on the costmap but '
                            'unreachable from the robot (no traversable '
                            'route, i.e. cut off by inflation)' % (tx, ty))
                return None
            if self.last_map is None:
                return None
            return None if self._explore_target_is_connected(pose) is not False \
                else ('target (%.2f, %.2f) is in a different free region than '
                      'the robot' % (tx, ty))
        except Exception:  # noqa: BLE001 - 判定绝不能阻断探索
            return None

    def _explore_target_is_connected(self, pose):
        """2026-10-12: 探索目标的连通性闸门：三态判定目标是否与机器人同属一个自由连通域。

        True  = 同域（放行）
        False = 可证明不同域（拒发并拉黑）
        None  = 无法判定（放行，交给既有闸门）
        """
        try:
            if pose is None or self.last_map is None:
                return None
            goal_cell = self._map_cell_index(
                pose.pose.position.x, pose.pose.position.y)
            if goal_cell is None:
                return None
            data = self.last_map.data
            if not (0 <= goal_cell < len(data)):
                return None
            if int(data[goal_cell]) >= 50:
                # 目标格被占：交给既有 R1d/goal_is_traversable 判，不抢判定权
                return None
            cells = self._reachable_free_cells()
            if cells is None:
                return None
            return goal_cell in cells
        except Exception:  # noqa: BLE001
            return None

    def _home_route_connectivity(self, pose):
        """Return (verdict, detail) for current-pose vs home-goal connectivity.

        verdict: True  = same free region (dispatch is reasonable)
                 False = provably different free regions (goal unreachable)
                 None  = cannot decide (defer to the existing gates)
        """
        detail = {}
        try:
            if self.last_map is None:
                return None, {'reason': 'no /map yet'}
            data = self.last_map.data
            info = self.last_map.info
            width, height = int(info.width), int(info.height)
            if width <= 0 or height <= 0 or not data:
                return None, {'reason': 'empty /map'}

            detail.update({
                'mapOriginX': round(float(info.origin.position.x), 3),
                'mapOriginY': round(float(info.origin.position.y), 3),
                'mapResolution': round(float(info.resolution), 6),
                'mapWidth': width,
                'mapHeight': height,
                'loadedMapSignature': getattr(
                    self, '_loaded_map_signature', None),
            })

            import numpy as _nph
            _hm_arr = _nph.asarray(data, dtype=_nph.int8)[:width * height]
            signature = (width, height, hash(_hm_arr.tobytes()),
                         int(float(info.resolution) * 1e6))
            if self._home_route_cache is not None \
                    and self._home_route_cache[0] == signature:
                cached = self._home_route_cache[1]
                detail.update(cached['detail'])
                return cached['verdict'], detail

            start_cell = self._map_cell_index(
                pose.pose.position.x, pose.pose.position.y)
            goal_cell = self._map_cell_index(self.home_x, self.home_y)
            detail['startIndex'] = start_cell
            detail['goalIndex'] = goal_cell
            if start_cell is None or goal_cell is None:
                self._home_route_cache = (signature, {
                    'verdict': None,
                    'detail': dict(detail, reason='start or goal outside /map')})
                return None, detail

            detail['startValue'] = int(data[start_cell])
            detail['goalValue'] = int(data[goal_cell])

            def is_free(value):
                return int(value) < 50

            if not is_free(data[start_cell]) or not is_free(data[goal_cell]):
                # 起点或终点本身是占用格：交给既有 R1c/R1d 判定，
                # 这里不抢它的判定权。
                self._home_route_cache = (signature, {
                    'verdict': None,
                    'detail': dict(detail, reason='start or goal is occupied')})
                return None, detail

            visited = bytearray(len(data))
            queue = deque([start_cell])
            visited[start_cell] = 1
            found = False
            reachable = 0
            while queue:
                cell = queue.popleft()
                reachable += 1
                if cell == goal_cell:
                    found = True
                    break
                row, col = divmod(cell, width)
                for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nrow, ncol = row + drow, col + dcol
                    if nrow < 0 or ncol < 0 or nrow >= height or ncol >= width:
                        continue
                    ncell = nrow * width + ncol
                    if visited[ncell]:
                        continue
                    visited[ncell] = 1
                    if is_free(data[ncell]):
                        queue.append(ncell)

            detail['reachableCells'] = reachable
            detail['totalCells'] = len(data)
            self._home_route_cache = (signature, {
                'verdict': bool(found), 'detail': dict(detail)})
            return bool(found), detail
        except Exception as exc:  # noqa: BLE001 - 检查本身绝不能阻断返航
            import traceback as _tb
            return None, {'reason': 'check failed: %s | %s'
                          % (exc, _tb.format_exc().splitlines()[-3:])}

    def _log_home_route_evidence(self, verdict, detail):
        self.get_logger().warn(
            'return-home route evidence: sameFreeRegion=%s origin=(%.3f,%.3f) '
            'res=%.3f size=%sx%s startIndex=%s(%s) goalIndex=%s(%s) '
            'reachable=%s/%s loadedMapSignature=%s extra=%s'
            % (verdict,
               float(detail.get('mapOriginX') or 0.0),
               float(detail.get('mapOriginY') or 0.0),
               float(detail.get('mapResolution') or 0.0),
               detail.get('mapWidth'), detail.get('mapHeight'),
               detail.get('startIndex'), detail.get('startValue'),
               detail.get('goalIndex'), detail.get('goalValue'),
               detail.get('reachableCells'), detail.get('totalCells'),
               detail.get('loadedMapSignature'),
               detail.get('reason') or '-'))

    def _nearest_viable_home_pose(self, pose, clearance):
        """2026-10-09 (operator): the nearest stand point near the origin that passes R1d.

        The return-home target is the map-frame origin, i.e. wherever the robot was when the
        session started.  If it started within the clearance disc of an obstacle the origin can
        NEVER satisfy R1d, and no reversal can change that: the occupancy around the origin was
        recorded at start time (measured 16:06: "less than 0.50 m clearance around (0.00, 0.00)"
        x4 with "front obstacle too close: 0.39m").  Rather than give up, walk outward ring by
        ring and take the closest cell that satisfies both halves of R1d.  Returns None when
        nothing qualifies, in which case the caller keeps the original target.
        """
        if self.last_map is None:
            return None
        info = self.last_map.info
        res = info.resolution
        if res <= 0.0:
            return None
        ox, oy = info.origin.position.x, info.origin.position.y
        cx, cy = pose.pose.position.x, pose.pose.position.y
        cmx, cmy = int((cx - ox) / res), int((cy - oy) / res)
        q = pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        max_ring = max(1, int(math.ceil(self.return_home_target_search_m / res)))
        best = None
        for ring in range(1, max_ring + 1):
            for dy in range(-ring, ring + 1):
                for dx in range(-ring, ring + 1):
                    if max(abs(dx), abs(dy)) != ring:
                        continue
                    mx, my = cmx + dx, cmy + dy
                    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                        continue
                    if not self._fallback_cell_has_clearance(mx, my, clearance):
                        continue
                    wx = ox + (mx + 0.5) * res
                    wy = oy + (my + 0.5) * res
                    if not self._home_rear_is_clear(wx, wy, yaw):
                        continue
                    dist = math.hypot(wx - cx, wy - cy)
                    if best is None or dist < best[0]:
                        best = (dist, wx, wy)
            if best is not None:
                break
        if best is None:
            self.get_logger().warn(
                'return-home: no cell within %.2f m of the origin satisfies the %.2f m '
                'clearance; keeping the original target'
                % (self.return_home_target_search_m, clearance))
            return None
        dist, wx, wy = best
        shifted = PoseStamped()
        shifted.header.frame_id = pose.header.frame_id
        shifted.header.stamp = pose.header.stamp
        shifted.pose.position.x = wx
        shifted.pose.position.y = wy
        shifted.pose.position.z = pose.pose.position.z
        shifted.pose.orientation = pose.pose.orientation
        self.get_logger().warn(
            'return-home target shifted to the nearest stand point (%.2f, %.2f), %.2f m from '
            'the origin, to satisfy the %.2f m clearance'
            % (wx, wy, dist, clearance))
        return shifted

    def _goal_target_is_viable(self, pose, clearance=None):
        """R1d: target clearance + rear space, applied to every goal source.

        2026-10-09 (operator): optional `clearance` so the return-home path can use its own,
        smaller disc; exploration keeps self.goal_clearance.

        * clearance: the same goal_clearance disc the fallback sampler always used;
        * rear space: the vehicle must have room behind it at the target, otherwise it arrives
          wedged and the next start clearance can never be satisfied.
        A rejected target is simply not dispatched; exploration then selects another one.
        """
        if self.last_map is None:
            return True
        info = self.last_map.info
        res = info.resolution
        if res <= 0.0:
            return True
        ox, oy = info.origin.position.x, info.origin.position.y
        tx, ty = pose.pose.position.x, pose.pose.position.y
        mx, my = int((tx - ox) / res), int((ty - oy) / res)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            self.get_logger().warn('R1d target rejected: (%.2f, %.2f) is outside /map' % (tx, ty))
            return False
        if clearance is None:
            clearance = self.goal_clearance
        if not self._fallback_cell_has_clearance(mx, my, clearance):
            self._r1d_last_reject = (float(tx), float(ty), 'clearance')
            self.get_logger().warn(
                'R1d target rejected: less than %.2f m clearance around (%.2f, %.2f)'
                % (clearance, tx, ty))
            return False
        q = pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rear = 0.50                      # room behind the vehicle at the target
        bx = tx - rear * math.cos(yaw)   # yaw points robot->target, so -yaw is behind
        by = ty - rear * math.sin(yaw)
        bmx, bmy = int((bx - ox) / res), int((by - oy) / res)
        if 0 <= bmx < info.width and 0 <= bmy < info.height:
            if self.last_map.data[bmy * info.width + bmx] >= 50:
                self.get_logger().warn(
                    'R1d target rejected: no %.2f m of room behind (%.2f, %.2f)'
                    % (rear, tx, ty))
                return False
        return True

    def _ensure_start_clearance_before_goal(self):
        """R1c (2026-09-29): never start a goal from a spot the vehicle cannot leave.

        The manager used to dispatch targets while the robot was wedged (wall 0.37 m ahead); the
        robot then could not progress, the spin recovery was refused and it stood still until the
        goal was blacklisted.  Here the start clearance is verified first and, when it fails
        because the front is too close, one bounded reverse restores it before the goal goes out.
        Rate-limited through the same blocked_retreat_* parameters used by R1b.
        """
        try:
            ready, reason = self.current_start_is_map_clear()
        except Exception:  # noqa: BLE001 - a gate must never break dispatching
            return True
        if ready:
            return True
        threshold = self.front_stop_distance + self.blocked_clearance_hysteresis_m
        if self.front_min_range is None or self.front_min_range >= threshold:
            self.get_logger().warn(
                'R1c start clearance not met (%s) but the front is open; dispatching anyway'
                % reason)
            return True
        if self._blocked_retreat_count >= int(
                self.get_parameter('blocked_retreat_max_per_session').value):
            return True
        now = time.monotonic()
        if now - self._blocked_retreat_last_at < float(
                self.get_parameter('blocked_retreat_min_interval_sec').value):
            return True
        self._blocked_retreat_last_at = now
        self._blocked_retreat_count += 1
        saved_latch = getattr(self, '_retreat_used_for_attempt', False)
        self._retreat_used_for_attempt = False
        try:
            status, moved = self._retreat_from_occupied_cells('exploration')
        finally:
            self._retreat_used_for_attempt = saved_latch
        ready_after, reason_after = self.current_start_is_map_clear()
        self.get_logger().warn(
            'R1c pre-goal retreat %d: status=%s moved=%.2fm start_clear=%s (%s)'
            % (self._blocked_retreat_count, status, moved, ready_after, reason_after))
        return bool(ready_after)

    def send_nav_goal(
            self, pose, validate_map=True, check_path_density=True):
        # R1c (2026-09-29): do not start a goal the vehicle cannot leave; see the helper.
        if not self._safe_call('R1c start clearance', lambda: self._ensure_start_clearance_before_goal(), True):
            self.publish_zero_velocity()
            return
        # R1d (2026-09-29): the root-cause fix -- EVERY goal source (frontier included) must
        # offer a target the vehicle can actually stand at.  Only the random fallback sampler
        # used to check this, so a frontier goal could leave the robot nose-first against a
        # wall: it arrived, the next start clearance failed, and nothing could restore it.
        # A (2026-09-29): RRT frontier goals sit on the boundary of the known-free
        # map, so reaching them means entering the front gate repeatedly (measured:
        # 196 'front obstacle too close' events in one run).  Pull the goal back into
        # known-free space first.
        # 2026-09-29: A/B are EXPLORATION-only.  Applied to every source they also
        # moved/rejected the return-home target.  home/manual keep the original path.
        pose = self._safe_call('A pull-back', lambda: self._pull_goal_into_known_free(pose), pose)
        # 2026-10-09 (operator): the return-home target is the map ORIGIN, so it must not be held
        # to the exploration-grade clearance disc -- see _nearest_viable_home_pose for the
        # measured failure.  Home gets its own threshold and, failing that, the nearest stand
        # point.  Exploration and manual goals take exactly the path they took before.
        is_home_goal = bool(getattr(self, 'return_home_requested', False)) or (
            self._goal_kind_context == 'home')
        r1d_clearance = (
            self.return_home_goal_clearance if is_home_goal else self.goal_clearance)
        if is_home_goal:
            shifted_home = self._safe_call(
                'R1d home relocalise',
                lambda: self._nearest_viable_home_pose(pose, r1d_clearance), None)
            if shifted_home is not None:
                pose = shifted_home
        self._r1d_last_reject = None
        if not self._safe_call(
                'R1d target viability',
                lambda: self._goal_target_is_viable(pose, r1d_clearance), True):
            # 2026-10-13 P3-1：把拒发结果反馈给采样器（只对探索目标）
            self._r1d_reject_note(
                pose, 'home' if is_home_goal else (self._goal_kind_context or 'manual'))
            self.publish_zero_velocity()
            return
        # B (2026-09-29): and validate the approach corridor, because a legal target can
        # still be unreachable without violating the front gate.
        if not self._safe_call('B corridor', lambda: self._approach_corridor_is_clear(pose), True):
            self.publish_zero_velocity()
            return
        kind = (
            'home' if self.return_home_requested
            else self._goal_kind_context or 'manual'
        )
        if (self._return_home_xy_finalizing
                or self.shutdown_started
                or self._mapping_cancel_requested
                or self._mapping_timeout_failure_kind):
            self.publish_zero_velocity()
            self.get_logger().info(
                f'navigation goal from {kind} ignored while the mapping '
                'session is shutting down')
            return
        if self._goal_transport_fault or self._spin_transport_fault:
            self.publish_zero_velocity()
            self.mode = 'safety_stop'
            self.get_logger().error(
                'navigation goal suppressed because an earlier action '
                'transport/cancel fault is still latched')
            return
        if (self.goal_send_future is not None
                or self.nav_goal_active
                or self._stall_cancel_requested):
            self.get_logger().warn(
                f'navigation goal from {kind} ignored while another goal is '
                'pending, executing, or canceling')
            return
        if (self._is_exploration_goal(kind)
                and (not self.auto_explore or self.mapping_done)):
            self.get_logger().info(
                f'exploration goal from {kind} ignored while exploration is '
                'disabled')
            return
        # 2026-10-12: 探索目标的连通性闸门：与机器人不同自由连通域的探索目标直接拒发。
        # 实测靶点被隔断时 Nav2 会连续报 no valid path found / exceeded
        # maximum iterations，直到 50s 硬上限才放弃 —— 这是"已知区域死锁"。
        # getattr 兜底：ManagerHarness 等测试替身不走真实 __init__，缺该属性
        # 时按"开启"处理（与生产默认一致）；判定本身另有三态兜底，
        # 属性缺失不会造成误拒。
        _require_conn = bool(getattr(
            self, 'exploration_require_connected_route', True))
        if _require_conn and self._is_exploration_goal(kind):
            _reason = self._safe_call(
                'explore connectivity',
                lambda: self._explore_target_reject_reason(pose), None)
            if _reason:
                # 拉黑该目标，使采样器下一拍换点而不是反复选同一死角
                try:
                    self.failed_goals.append((
                        float(pose.pose.position.x),
                        float(pose.pose.position.y),
                        time.monotonic()))
                except Exception:  # noqa: BLE001
                    pass
                self.publish_zero_velocity()
                self.get_logger().warn(
                    'exploration goal rejected: target (%.2f, %.2f) is '
                    'unreachable; blacklisting and re-selecting: %s'
                    % (pose.pose.position.x, pose.pose.position.y,
                       _reason))
                # 2026-10-13 P21（丙）：连通域"不可达"也计入方向解锁
                self._note_unreachable_rejection('reach')
                return
        if self.base_charging:
            self.pause_for_charging()
            return
        if not self.nav2_is_ready():
            self.report_nav2_not_ready()
            return
        if self.is_recovering() or self.is_recovery_state():
            self.mode = 'safety_stop'
            return
        if self.front_blocked:
            self.remember_blocked_place()
            self.publish_zero_velocity()
            self.mode = 'safety_stop'
            measured = (
                f'{self.front_min_range:.2f}m'
                if self.front_min_range is not None else 'unavailable')
            self.last_error = (
                f'front obstacle too close: {measured}')
            if self._is_exploration_goal(kind):
                armed = self._arm_exploration_precheck_recovery(
                    time.monotonic())
                if not armed:
                    self.navigation_state = 'blocked_waiting_clearance'
            else:
                self.navigation_state = 'blocked_waiting_clearance'
                super().start_escape_recovery()
            self._log_recovery_event(
                f'goal_precheck_blocked:{kind}', self.last_error)
            return

        if kind == 'home':
            # Even when the robot starts inside the XY tolerance, Nav2 must
            # own a real action so it gets the full orientation grace window.
            self._reset_return_home_xy_completion()

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.frame_id = pose.header.frame_id or self.global_frame
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        if self._is_exploration_goal(kind):
            goal.behavior_tree = self.exploration_behavior_tree

        self.last_goal = goal.pose
        self.last_error = ''

        if kind != 'home' and self.is_current_pose_goal(goal.pose):
            self.mode = 'arrived'
            self.navigation_state = 'succeeded'
            self._clear_recovery_intent(clear_episode=True)
            self.get_logger().info(
                'navigation goal is current pose; marking as arrived')
            return
        # 2026-10-13 P9-D：过滤"极近目标"，并埋点记录其来源。
        # 极近目标会让 nav2_rotation_shim_controller 先原地旋转对准；走完 0.3m
        # 后立刻再派一个极近目标 → 实测车原地旋转 60+ 秒（stall_hold 36.3s）。
        # 只对探索目标生效；返航/手动不受影响。被拒目标不写失败记忆
        # （它是"太近"而不是"去不了"），由候选队列顺序换下一个候选。
        if (kind != 'home'
                and self._is_exploration_goal(kind)
                and getattr(self, 'exploration_min_goal_distance_m', 0.0) > 0.0):
            _cur = self.update_pose()
            if _cur is not None:
                _dmin = float(self.exploration_min_goal_distance_m)
                # ★入参 `pose` 是 PoseStamped（见 send_nav_goal 签名），
                # 取位置必须走 pose.pose.position。2026-10-13 修：原写法用
                # goal.pose.position，而 goal 是后面才构造的 NavigateToPose.Goal，
                # 此处不存在 → AttributeError: 'PoseStamped' object has no
                # attribute 'position' → 冒泡到 _auto_explore_callback → 车不动。
                _d = math.hypot(
                    pose.pose.position.x - _cur.pose.position.x,
                    pose.pose.position.y - _cur.pose.position.y)
                if _d < _dmin:
                    # 来源埋点：kind / 调用栈 / A 方案上下文，用于定位谁派了极近点
                    _who = 'unknown'
                    try:
                        import inspect as _ins
                        _fr = _ins.stack()
                        _callers = [f.function for f in _fr[1:7]]
                        _who = '<-'.join(_callers)
                    except Exception:  # noqa: BLE001
                        pass
                    self._p1_safe_log(
                        'warn',
                        'exploration goal rejected: target (%.2f, %.2f) is only '
                        '%.2fm from the robot (< %.2fm minimum); kind=%s '
                        'prevalidate_phase=%s window=%s caller=%s'
                        % (pose.pose.position.x, pose.pose.position.y, _d,
                           _dmin, kind,
                           getattr(self, '_prevalidate_phase', 'n/a'),
                           getattr(self, '_fallback_window', 'n/a'),
                           _who))
                    self.publish_zero_velocity()
                    return

        # 2026-10-13 P21：探索目标的"转角门禁"。
        # 未解锁（_explore_direction_flip=False）时只放行 |turn| <= 上限的目标；
        # 连续"确定不可达"拒发达 exploration_direction_flip_after(2) 次即解锁。
        # 防死锁宽限：拦截累计超过 grace_count 时自动解锁并放行（否则车头朝向
        # 不对会导致"所有候选都被拦 → 车不动 → 朝向不变 → 永远不解锁"）。
        # ★兜底方向：按 max_turn_deg 是否为正值判定是否启用（与 P9-D 的
        #   getattr(self, 'exploration_min_goal_distance_m', 0.0) > 0.0 同模式）。
        #   属性缺失（测试替身不走 __init__）即视为不启用，避免误拦。
        _turn_limit_deg = float(getattr(
            self, 'exploration_dispatch_max_turn_deg', 0.0))
        if (kind != 'home'
                and self._is_exploration_goal(kind)
                and _turn_limit_deg > 0.0
                and bool(getattr(
                    self, 'exploration_dispatch_max_turn_enabled', True))
                and not getattr(self, '_explore_direction_flip', False)):
            _limit = math.radians(abs(_turn_limit_deg))
            _turn = self._goal_turn_from_current(pose)
            if _turn is not None and _turn > _limit:
                _grace = int(getattr(
                    self, 'exploration_dispatch_max_turn_grace_count', 3))
                self._turn_blocked_count = int(
                    getattr(self, '_turn_blocked_count', 0)) + 1
                if _grace > 0 and self._turn_blocked_count > _grace:
                    self._unlock_direction('turn_grace',
                                           self._turn_blocked_count)
                    self._p1_safe_log(
                        'warn',
                        'exploration turn gate: grace exhausted (%d blocks '
                        '> %d) -> direction unlocked, dispatching target '
                        '(%.2f, %.2f) with a %.0fdeg turn'
                        % (self._turn_blocked_count, _grace,
                           pose.pose.position.x, pose.pose.position.y,
                           math.degrees(_turn)))
                else:
                    self._p1_safe_log(
                        'warn',
                        'exploration goal rejected: target (%.2f, %.2f) needs '
                        'a %.0fdeg turn (> %.0fdeg limit while the direction is '
                        'locked; turn-blocks %d/%d; unreachable streak r1d=%d '
                        'planner=%d reach=%d of %d)'
                        % (pose.pose.position.x, pose.pose.position.y,
                           math.degrees(_turn), math.degrees(_limit),
                           self._turn_blocked_count, _grace,
                           int(getattr(self, '_r1d_consecutive', 0)),
                           int(getattr(self, '_planner_consecutive', 0)),
                           int(getattr(self, '_reach_consecutive', 0)),
                           int(getattr(
                               self, 'exploration_direction_flip_after', 2))))
                    self.publish_zero_velocity()
                    return

        if validate_map:
            traversable, reason = self.goal_is_traversable(
                goal.pose, check_path_density=check_path_density)
            if not traversable:
                self.mode = 'mapping' if self.auto_explore else 'error'
                self.navigation_state = 'goal_unreachable'
                self.last_error = reason
                self.remember_failed_goal()
                self._rrt_wait_started_at = None
                self.get_logger().warn(reason)
                return

        self.mode = 'navigating'
        self.navigation_state = 'sending_goal'
        self._pending_goal_kind = kind
        self._goal_generation_counter += 1
        goal_generation = self._goal_generation_counter
        self._pending_goal_generation = goal_generation
        self._goal_send_started_at = time.monotonic()
        self._goal_acceptance_timed_out = False
        self._timed_out_goal_future = None
        self._cancel_on_accept_context = None
        try:
            future = self.nav_client.send_goal_async(
                goal,
                feedback_callback=partial(
                    self._feedback_callback_for_goal, goal_generation),
            )
            if future is None:
                raise RuntimeError('send_goal_async returned no future')
        except Exception as exc:  # noqa: BLE001
            self.goal_send_future = None
            self._pending_goal_kind = None
            if self._pending_goal_generation == goal_generation:
                self._pending_goal_generation = None
            self._goal_send_started_at = 0.0
            self._clear_recovery_intent(clear_episode=True)
            self._goal_transport_fault = True
            self.auto_explore = False
            self.mode = 'safety_stop'
            self.navigation_state = 'goal_send_error'
            self.last_error = f'failed to dispatch navigation goal: {exc}'
            self.publish_zero_velocity()
            self.get_logger().error(self.last_error)
            return

        self.goal_send_future = future
        # 2026-10-10 (operator): 探索目标的 yaw 改为"保持当前朝向"。
        # 原实现把 yaw 设为"指向目标"的方位角，对 DUBIN（前向-only）是最难的
        # 约束：到达该位置时还必须朝向该方位，需要绕出再折回，实测导致
        # 连续 'exceeded maximum iterations'。只在探索目标上生效，
        # 返航/手动目标不受影响。
        if self._is_exploration_goal(kind):
            _cur = self.update_pose()
            if _cur is not None:
                _old_yaw = math.atan2(
                    2.0 * (pose.pose.orientation.w * pose.pose.orientation.z),
                    1.0 - 2.0 * (pose.pose.orientation.z ** 2))
                _new_yaw = None
                _tag = None
                if self.goal_yaw_mode == 'tangent':
                    _new_yaw = self.pose_yaw(_cur)
                    _tag = 'tangent (dispatch-time heading)'
                elif self.goal_yaw_mode == 'approach':
                    _dx = (pose.pose.position.x
                           - _cur.pose.position.x)
                    _dy = (pose.pose.position.y
                           - _cur.pose.position.y)
                    if math.hypot(_dx, _dy) > 1e-6:
                        _new_yaw = math.atan2(_dy, _dx)
                        _tag = 'approach (arrival heading)'
                # 'keep' 或退化情形（目标与当前位置重合）：不改写
                if _new_yaw is not None:
                    pose.pose.orientation.z = math.sin(_new_yaw * 0.5)
                    pose.pose.orientation.w = math.cos(_new_yaw * 0.5)
                    self.get_logger().info(
                        'goal yaw rewritten to %s: %.0fdeg -> %.0fdeg'
                        % (_tag, math.degrees(_old_yaw),
                           math.degrees(_new_yaw)))

        # 2026-10-10: 记录派发那一刻的机器人位姿与朝向差。"小区域内兜转"最直接
        # 的证据就是同一台车被反复派往相反方位；此前日志只记 target 坐标，
        # 看不出机器人当时在哪、要往哪个方向去。
        _robot = getattr(self, 'last_pose', None)
        _bearing = None      # 2026-10-10: 供下方落盘方向持续基准
        if _robot is not None:
            _rx = _robot.pose.position.x
            _ry = _robot.pose.position.y
            _tx = pose.pose.position.x
            _ty = pose.pose.position.y
            _q = _robot.pose.orientation
            _yaw = math.atan2(2.0 * (_q.w * _q.z + _q.x * _q.y),
                              1.0 - 2.0 * (_q.y * _q.y + _q.z * _q.z))
            _bearing = math.atan2(_ty - _ry, _tx - _rx)
            _delta = abs(math.atan2(math.sin(_bearing - _yaw),
                                    math.cos(_bearing - _yaw)))
            self.get_logger().info(
                'navigation goal dispatched: source=%s, '
                'target=(%.2f, %.2f), robot=(%.2f, %.2f, yaw=%.0fdeg), '
                'dist=%.2fm, turn=%.0fdeg, bt=%s'
                % (kind, _tx, _ty, _rx, _ry, math.degrees(_yaw),
                   math.hypot(_tx - _rx, _ty - _ry),
                   math.degrees(_delta),
                   goal.behavior_tree or 'default'))
        else:
            self.get_logger().info(
                f'navigation goal dispatched: source={kind}, '
                f'target=({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f}), robot=unknown, '
                f'bt={goal.behavior_tree or "default"}')
        # 2026-10-10: 每次派发探索目标时更新方向持续基准。
        # 原实现只在 STATUS_SUCCEEDED 分支更新（manager.py:915），目标被频繁
        # 取消时该值长期过期，使方向持续机制在低成功率下失效——实测 turn 序列
        # 前 7 个 108-175 度，后 3 个连续到达后降到 0-30 度。
        # 只对探索目标更新，不改返航/手动目标的语义。
        #
        # 2026-10-12: 方向持续基准对所有探索目标生效。
        # 原条件写死 'exploration_manager'，而 RRT 目标的 kind 是
        # 'exploration_rrt'（frontier_goal_callback → _run_with_goal_kind），
        # 于是方向基准在 RRT 派发后**完全不更新**：机器人已被 RRT 转到新朝向，
        # 基准却停留在上一次 fallback 的方位，下一次 fallback 选点必然大掉头。
        # 实测 8 目标序列里 出发大转(>120°) 3/7 次、走完朝向翻转(>90°) 2/7 次，
        # 其中『出发 -162° 去 -15° 方位』的 147° 掉头即由此产生。
        if self._is_exploration_goal(kind) and _bearing is not None:
            self.last_explore_heading = _bearing
        # The future may already be complete; kind/timing state is deliberately
        # installed before registering this callback.
        future.add_done_callback(self.goal_response_callback)

    def _clear_goal_acceptance_tracking(self, future=None):
        if future is None or self.goal_send_future is future:
            self.goal_send_future = None
        self._goal_send_started_at = 0.0
        if future is None or self._timed_out_goal_future is future:
            self._goal_acceptance_timed_out = False
            self._timed_out_goal_future = None

    def _monitor_goal_acceptance(self, now):
        future = self.goal_send_future
        if future is None or future.done():
            return
        if self._goal_send_started_at <= 0.0:
            self._goal_send_started_at = now
            return
        if self._goal_acceptance_timed_out:
            self.publish_zero_velocity()
            return
        elapsed = now - self._goal_send_started_at
        if elapsed < self.goal_acceptance_timeout_sec:
            return

        self._goal_acceptance_timed_out = True
        self._timed_out_goal_future = future
        self._goal_transport_fault = True
        self.auto_explore = False
        self._clear_recovery_intent(clear_episode=True)
        self._stop_manager_recovery_motion()
        self.mode = 'safety_stop'
        self.navigation_state = 'goal_acceptance_timeout'
        self.last_error = (
            'navigation action did not answer goal acceptance within '
            f'{self.goal_acceptance_timeout_sec:.1f}s; waiting for the exact '
            'goal handle so it can be canceled safely')
        self.publish_zero_velocity()
        self.get_logger().error(self.last_error)

    def _handle_goal_response_error(self, future, pending_kind, exc):
        self._reset_return_home_xy_completion()
        self._clear_goal_acceptance_tracking(future)
        self._pending_goal_kind = None
        self._pending_goal_generation = None
        self._cancel_on_accept_context = None
        self._stop_manager_recovery_motion()
        with self._feedback_state_lock:
            self._clear_feedback_tracking_locked(clear_generation=True)
            self.exploration_watchdog.clear()
        self._rrt_wait_started_at = None
        self._clear_recovery_intent(clear_episode=True)
        self._goal_transport_fault = True
        self.auto_explore = False
        self.mode = 'safety_stop'
        self.navigation_state = 'goal_response_error'
        self.last_error = (
            'navigation goal response failed for '
            f'{pending_kind or "unknown"}: '
            f'{exc}')
        self.publish_zero_velocity()
        self.get_logger().error(self.last_error)

        handle = self.current_goal_handle
        if handle is None:
            self._active_goal_kind = None
            self.goal_result_future = None
            self.nav_goal_active = False
            self._reset_stall_cancel_tracking()
            return

        if self._active_goal_kind is None:
            self._active_goal_kind = pending_kind
        self.nav_goal_active = True
        if self._stall_cancel_requested:
            self._recovery_after_cancel = False
            self._cancel_context = 'goal_response_error'
        else:
            self._begin_monitored_cancel(
                'goal_response_error', False, time.monotonic())
        self.navigation_state = 'goal_response_error_canceling'
        if not self._result_retry_attempted:
            self._result_retry_attempted = True
            try:
                self.goal_result_future = handle.get_result_async()
                self.goal_result_future.add_done_callback(
                    self.goal_result_callback)
            except Exception as retry_exc:  # noqa: BLE001
                self.goal_result_future = None
                self.get_logger().error(
                    f'could not reattach navigation result monitoring: '
                    f'{retry_exc}')

    def _stop_manager_recovery_motion(self):
        self.recovery_until = 0.0
        self.recovery_started_at = 0.0
        self.recovery_clear_count = 0
        self.recovery_twist = Twist()
        if self._spin_phase == 'idle':
            self._cartographer_spin_active = False
        self.publish_zero_velocity()

    def _monitor_accepted_goal_cancel(
            self, goal_handle, pending_kind, context):
        self._stop_manager_recovery_motion()
        self._active_goal_kind = pending_kind
        self.current_goal_handle = goal_handle
        self.nav_goal_active = True
        self.last_error = f'{context} requested exact goal cancellation'
        self.mode = 'safety_stop'
        self.navigation_state = f'{context}_canceling_accepted_goal'
        self._begin_monitored_cancel(context, False, time.monotonic())
        self.publish_zero_velocity()
        try:
            self.goal_result_future = goal_handle.get_result_async()
            self.goal_result_future.add_done_callback(
                self.goal_result_callback)
        except Exception as exc:  # noqa: BLE001
            self._handle_goal_response_error(None, pending_kind, exc)
            return False
        return True

    def goal_response_callback(self, future):
        pending_kind = self._pending_goal_kind
        timed_out = future is self._timed_out_goal_future
        cancel_on_accept = self._cancel_on_accept_context
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._handle_goal_response_error(future, pending_kind, exc)
            return

        if cancel_on_accept is not None:
            self._clear_goal_acceptance_tracking(future)
            self._cancel_on_accept_context = None
            if self._pending_goal_kind == pending_kind:
                self._pending_goal_kind = None
            if not goal_handle.accepted:
                self.current_goal_handle = None
                self.goal_result_future = None
                self.nav_goal_active = False
                self._pending_goal_generation = None
                self.navigation_state = f'{cancel_on_accept}_goal_rejected'
                if self.mapping_done and self.pending_home_goal:
                    self._goal_transport_fault = False
                    self._dispatch_pending_home_goal()
                elif cancel_on_accept == 'base_charging':
                    self.mode = 'paused'
                    self.navigation_state = 'base_charging'
                elif cancel_on_accept == 'stop_navigation':
                    self.mode = 'stopped'
                    self.navigation_state = 'canceled'
                else:
                    self.mode = 'stopped'
                return
            self._monitor_accepted_goal_cancel(
                goal_handle, pending_kind, cancel_on_accept)
            self._pending_goal_generation = None
            return

        if timed_out:
            self._clear_goal_acceptance_tracking(future)
            if self._pending_goal_kind == pending_kind:
                self._pending_goal_kind = None
            if not goal_handle.accepted:
                self.nav_goal_active = False
                self.current_goal_handle = None
                self.goal_result_future = None
                self._pending_goal_generation = None
                self.mode = 'safety_stop'
                self.navigation_state = 'timed_out_goal_rejected'
                self.get_logger().error(
                    f'{self.last_error}; late response rejected the goal')
                if self.mapping_done and self.pending_home_goal:
                    self._goal_transport_fault = False
                    self._dispatch_pending_home_goal()
                return

            self._monitor_accepted_goal_cancel(
                goal_handle, pending_kind, 'timed_out_goal')
            self._pending_goal_generation = None
            return

        if self.is_recovering() or self.is_recovery_state():
            self._clear_goal_acceptance_tracking(future)
            if self._pending_goal_kind == pending_kind:
                self._pending_goal_kind = None
            if not goal_handle.accepted:
                self.current_goal_handle = None
                self.goal_result_future = None
                self.nav_goal_active = False
                self._pending_goal_generation = None
                self.mode = 'safety_stop'
                self.navigation_state = (
                    'recovery_during_goal_acceptance_goal_rejected')
                return
            self._monitor_accepted_goal_cancel(
                goal_handle,
                pending_kind,
                'recovery_during_goal_acceptance',
            )
            self._pending_goal_generation = None
            return

        try:
            super().goal_response_callback(future)
        except Exception as exc:  # noqa: BLE001
            self._handle_goal_response_error(future, pending_kind, exc)
            return

        self._clear_goal_acceptance_tracking(future)
        if (self.nav_goal_active and self._active_goal_kind is None
                and pending_kind is not None):
            self._active_goal_kind = pending_kind
        if (not goal_handle.accepted
                and self._is_exploration_goal(pending_kind)):
            self._rrt_wait_started_at = None
        if not goal_handle.accepted:
            self._pending_goal_generation = None
        if self._pending_goal_kind == pending_kind:
            self._pending_goal_kind = None

    def _clear_pose_stall_tracking_locked(self):
        self._pose_stall_generation = None
        self._pose_stall_anchor_x = None
        self._pose_stall_anchor_y = None
        self._pose_stall_anchor_yaw = None
        self._pose_stall_anchor_at = 0.0

    def _pose_stall_anchor_yaw_value(self):
        """2026-10-12: 朝向锚点。

        用 getattr 读取，使「不调用真实 __init__ 的最小替身」也能工作
        （既有单测 ManagerHarness 就是这种替身）。
        """
        return getattr(self, '_pose_stall_anchor_yaw', None)

    def _stall_yaw_threshold(self):
        """2026-10-12: 判定「在转动」所需的朝向变化（弧度）。

        默认 0.15 rad 与 declare_parameter 的默认值一致；
        用 getattr 读取以兼容最小测试替身。
        """
        return float(getattr(self, 'exploration_pose_stall_yaw_rad', 0.15))

    def _reset_pose_stall_anchor_locked(self, now, x, y, yaw):
        """2026-10-12: 重置停滞锚点（位置 + 朝向 + 时间）。

        位置与朝向**同时**重置：判据要求二者都"没动"才算停滞，
        所以任何一项发生变化都意味着机器人在动。
        """
        self._pose_stall_anchor_x = x
        self._pose_stall_anchor_y = y
        # 2026-10-12 N-10a：yaw 为 None 时**保留原锚点朝向**，不要覆盖成 None ——
        # 否则一次取不到朝向就会让后续所有轮次的朝向判据永久失效。
        if yaw is not None:
            self._pose_stall_anchor_yaw = yaw
        self._pose_stall_anchor_at = now

    def _clear_feedback_tracking_locked(self, clear_generation=False):
        if clear_generation:
            self._active_goal_generation = None
        self._latest_feedback_distance = None
        self._distance_source = None
        self._latest_recovery_count = 0
        self._feedback_baseline_generation = None
        self._feedback_baseline_deadline = 0.0
        self._feedback_baseline_locked = True
        self._clear_pose_stall_tracking_locked()

    def reset_progress_watchdog(self):
        now = time.monotonic()
        with self._feedback_state_lock:
            kind = self._pending_goal_kind
            if kind is None:
                kind = 'home' if self.return_home_requested else 'manual'
            goal_generation = self._pending_goal_generation

            # Invalidate the previous action token before any inherited reset
            # work can synchronously yield to a queued feedback callback.
            self._active_goal_kind = kind
            self._active_goal_generation = goal_generation
            self._pending_goal_kind = None
            self._pending_goal_generation = None
            self._clear_feedback_tracking_locked()

            if self._is_exploration_goal(kind):
                self.exploration_watchdog.reset(now)
                # 2026-10-14 D3/D8/D12：新目标从当前位置重新立锚点。
                self._progress_path_anchor = None
                self._no_net_anchor = None
                self._no_net_cooldown_until = 0.0
                self._feedback_baseline_generation = goal_generation
                self._feedback_baseline_deadline = (
                    now + self.feedback_baseline_settle_sec)
                self._feedback_baseline_locked = False
            else:
                self.exploration_watchdog.clear()

            # The base reset only snapshots pose progress.  Keeping it under
            # the same re-entrant lock makes goal promotion and feedback state
            # reset one atomic transition for multi-threaded executors too.
            super().reset_progress_watchdog()

            if self._is_exploration_goal(kind):
                self._pose_stall_generation = goal_generation
                if self.progress_x is not None and self.progress_y is not None:
                    self._pose_stall_anchor_x = self.progress_x
                    self._pose_stall_anchor_y = self.progress_y
                    self._pose_stall_anchor_at = now

        self._reset_stall_cancel_tracking()
        self._result_retry_attempted = False
        self._last_progress_log = 0.0
        if self._is_exploration_goal(kind):
            self.get_logger().info(
                f'exploration progress tracking started: source={kind}, '
                f'no-progress window='
                f'{self.exploration_watchdog.no_progress_timeout_sec:.1f}s, '
                'hard limit='
                f'{self.exploration_watchdog.hard_timeout_sec:.1f}s')

    def _feedback_generation_is_current_locked(self, goal_generation):
        return (
            goal_generation is not None
            and goal_generation == self._active_goal_generation
            and self.nav_goal_active
        )

    def _feedback_callback_for_goal(self, goal_generation, feedback_msg):
        """Drop queued feedback that belongs to an older action goal."""
        with self._feedback_state_lock:
            if (not self._feedback_generation_is_current_locked(
                    goal_generation)
                    or self._stall_cancel_requested):
                return
        self._process_feedback_for_goal(goal_generation, feedback_msg)

    def _geometric_distance_to_active_goal(self, pose=None):
        if self.last_goal is None:
            return None
        if pose is None:
            pose = self.update_pose()
        if pose is None:
            return None
        return math.hypot(
            self.last_goal.pose.position.x - pose.pose.position.x,
            self.last_goal.pose.position.y - pose.pose.position.y,
        )

    def _feedback_distance_is_plausible(self, distance):
        if not math.isfinite(distance) or distance < 0.0:
            return False
        if distance > self.feedback_zero_epsilon_m:
            return True
        geometric_distance = self._geometric_distance_to_active_goal()
        return (
            geometric_distance is not None
            and geometric_distance <= self.feedback_zero_goal_tolerance_m
        )

    def _commit_feedback_distance_locked(
            self, goal_generation, now, distance, recovery_count):
        watchdog = self.exploration_watchdog
        best_distance = watchdog.best_distance
        rebased = False

        if recovery_count > watchdog.recovery_count:
            watchdog.rebase_distance(now, distance, recovery_count)
            rebased = True
            self._feedback_baseline_locked = True
        elif self._distance_source != 'feedback':
            # The first trustworthy path-distance sample replaces any earlier
            # geometric estimate and establishes a fresh, bounded baseline.
            watchdog.rebase_distance(now, distance, recovery_count)
            rebased = True
            if (goal_generation != self._feedback_baseline_generation
                    or now >= self._feedback_baseline_deadline):
                self._feedback_baseline_locked = True
        elif watchdog.best_distance is None:
            watchdog.rebase_distance(now, distance, recovery_count)
            rebased = True
            if (goal_generation != self._feedback_baseline_generation
                    or now >= self._feedback_baseline_deadline):
                self._feedback_baseline_locked = True
        elif (goal_generation == self._feedback_baseline_generation
                and not self._feedback_baseline_locked):
            if now >= self._feedback_baseline_deadline:
                self._feedback_baseline_locked = True
            elif distance <= best_distance - watchdog.min_progress_m:
                # A real decrease proves that Nav2 has published the new path;
                # lock immediately so later detours cannot refresh the clock.
                self._feedback_baseline_locked = True
            elif distance >= best_distance + watchdog.min_progress_m:
                # During the fixed startup window only, correct a stale low
                # sample such as 2.957 -> 5.200.  The deadline is never moved.
                watchdog.rebase_distance(now, distance, recovery_count)
                rebased = True

        if self._feedback_baseline_locked and not rebased:
            # Record progress at feedback cadence.  Otherwise a genuine low
            # sample followed by a brief rebound between watchdog timer ticks
            # would be overwritten in _latest_feedback_distance.
            watchdog.observe(now, distance, recovery_count)

        self._latest_feedback_distance = distance
        self._distance_source = 'feedback'

    def _process_feedback_for_goal(self, goal_generation, feedback_msg):
        with self._feedback_state_lock:
            if (not self._feedback_generation_is_current_locked(
                    goal_generation)
                    or self._stall_cancel_requested):
                return
            exploration_goal = self._is_exploration_goal()

        feedback = getattr(feedback_msg, 'feedback', None)
        distance = None
        recovery_count = None
        plausible_distance = False
        if exploration_goal and feedback is not None:
            now = time.monotonic()
            try:
                distance = float(feedback.distance_remaining)
            except (AttributeError, TypeError, ValueError):
                distance = None
            try:
                recovery_count = int(feedback.number_of_recoveries)
            except (AttributeError, TypeError, ValueError):
                recovery_count = None
            plausible_distance = (
                distance is not None
                and self._feedback_distance_is_plausible(distance)
            )

        # Parsing and TF lookup intentionally happen outside the lock.  This
        # second token check is the actual commit boundary: an old callback
        # that passed the entry check cannot write after a goal reset/result.
        with self._feedback_state_lock:
            if (not self._feedback_generation_is_current_locked(
                    goal_generation)
                    or self._stall_cancel_requested):
                return
            if exploration_goal and self._is_exploration_goal():
                if recovery_count is None:
                    recovery_count = self._latest_recovery_count
                if plausible_distance:
                    self._commit_feedback_distance_locked(
                        goal_generation, now, distance, recovery_count)
                self._latest_recovery_count = max(
                    self._latest_recovery_count, recovery_count)
            super().feedback_callback(feedback_msg)

    def feedback_callback(self, feedback_msg):
        """Compatibility path that still preserves the current goal token."""
        with self._feedback_state_lock:
            goal_generation = self._active_goal_generation
        self._process_feedback_for_goal(goal_generation, feedback_msg)

    def _distance_to_active_goal(self, pose=None):
        with self._feedback_state_lock:
            if (self._distance_source == 'feedback'
                    and self._latest_feedback_distance is not None):
                return self._latest_feedback_distance
        return self._geometric_distance_to_active_goal(pose)

    def _reset_progress_path_anchor(self, pose):
        """D3：把净推进锚点移到当前位姿。"""
        if pose is None:
            return
        try:
            self._progress_path_anchor = (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(self._geometric_distance_to_active_goal(pose)),
            )
        except Exception:  # noqa: BLE001 - 锚点绝不能阻断跟踪
            self._progress_path_anchor = None

    def _path_aware_progress_distance(self, distance, pose):
        """D3/D7：把"到目标的瞬时距离"折算成"自锚点以来的净推进量"。

        为什么需要它（2026-10-14 探针实测 probe31-1014-133517）：
          feedback 给的是"当前到目标还有多远"。车一旦开始原地绕圈，
          这个值每拍都在变，而 ExplorationProgressWatchdog 只看"相对上次
          最好值是否改善了 min_progress_m"——于是"距离在变"被当成"在前进"，
          stagnant 永远涨不到 no_progress_timeout_sec，18s 看门狗与 7s 姿态
          停滞两级软判据全部沉默，唯一兜底只剩 50s 硬上限。
          实测该轮：弧长 6.92 m / 转角 40 rad（≈6.4 圈）却在原地，
          watchdog trace 里 stagnant 恒为 0.0。

        语义（D7 修正，d3-test 抓出）：
          · 车与锚点的距离 < exploration_progress_anchor_m 时，锚点不动，
            返回 max(0, 锚点处距离 - 当前距离)；
          · 车移动超过阈值时，只把锚点前移到当前位置，**本身不产生进展**
            （返回 0.0）。
          于是：原地绕圈 ⇒ 恒为 0 ⇒ 判停滞；直行推进 ⇒ 每拍返回真实缩短量。

        取不到位姿或拿不到锚点时回退为原始距离（fail-open，绝不误杀）。
        """
        if not getattr(self, 'exploration_progress_path_aware', False):
            return distance
        if pose is None or distance is None:
            return distance
        try:
            anchor = self._progress_path_anchor
            cx = float(pose.pose.position.x)
            cy = float(pose.pose.position.y)
            if anchor is None:
                # 第一次观测：在当前位姿立锚点，本拍无进展。
                self._reset_progress_path_anchor(pose)
                return 0.0
            ax, ay, a_dist = anchor
            if (math.hypot(cx - ax, cy - ay)
                    >= self.exploration_progress_anchor_m):
                # 车真的移动了：锚点前移，本拍不产生进展。
                self._reset_progress_path_anchor(pose)
                return 0.0
            return max(0.0, float(a_dist) - float(distance))
        except Exception:  # noqa: BLE001 - 进展判据绝不能把自己搞崩
            return distance

    def _no_net_progress_timed_out(self, now, distance, pose):
        """D8：车"一直在动、却始终没有靠近过目标"时判停滞。

        与 ExplorationProgressWatchdog 的分工：
          · 看门狗管"到目标的距离有没有改善"——对绕圈太钝（距离一直在变）；
          · 本判据管"位姿在动、但目标距离没有净改善"，正是原地绕圈的形态。
        锚点规则：每次真实进展（改善 >= min_progress_m）把锚点重置到当前位姿，
        因此只有"持续动却没有进展"才会累积，正常推进与真静止都不受影响。
        """
        if not getattr(self, 'exploration_progress_no_net_timeout', False):
            return False
        if pose is None or distance is None:
            return False
        try:
            cx = float(pose.pose.position.x)
            cy = float(pose.pose.position.y)
            d = float(distance)
            min_progress = float(self.exploration_watchdog.min_progress_m)
            if now < getattr(self, '_no_net_cooldown_until', 0.0):
                # D12：刚触发过，冷却期内不重复告警，但计时从当前位姿重起，
                # 以便冷却结束后能立刻给出第二次观察。
                self._no_net_anchor = (float(now), cx, cy, d, 0.0)
                return False
            anchor = self._no_net_anchor
            if anchor is None:
                self._no_net_anchor = (float(now), cx, cy, d, 0.0)
                return False
            a_at, ax, ay, a_dist, travel = anchor
            travel += math.hypot(cx - ax, cy - ay)
            if a_dist - d >= min_progress:
                # 真的靠近了：锚点重置，计时与位移清零。
                self._no_net_anchor = (float(now), cx, cy, d, 0.0)
                return False
            self._no_net_anchor = (a_at, cx, cy, a_dist, travel)
            window = float(self.exploration_watchdog.no_progress_timeout_sec)
            if (now - a_at) >= window and travel >= self.exploration_no_net_min_travel_m:
                self.get_logger().warn(
                    'no net progress for %.1fs while moving %.2fm '
                    '(remaining=%.2fm) -> treating the goal as stalled'
                    % (now - a_at, travel, d))
                # D12：不置空锚点，改为短冷却再武装（见参数说明）。
                self._no_net_cooldown_until = (
                    float(now) + float(getattr(self, 'exploration_no_net_rearm_sec', 3.0)))
                self._no_net_anchor = (float(now), cx, cy, d, 0.0)
                return True
            return False
        except Exception:  # noqa: BLE001 - 判据绝不能把自己搞崩
            return False

    def _observe_exploration_progress(self, now, pose=None):
        with self._feedback_state_lock:
            if (not self.nav_goal_active
                    or not self._is_exploration_goal()):
                return None, None
            goal_generation = self._active_goal_generation
            if (not self._feedback_baseline_locked
                    and now >= self._feedback_baseline_deadline):
                self._feedback_baseline_locked = True
            if self._distance_source == 'feedback':
                distance = self._latest_feedback_distance
                # 2026-10-14 D3：改用净推进量送进看门狗（见
                # _path_aware_progress_distance 的说明）。
                distance = self._path_aware_progress_distance(distance, pose)
                decision = self.exploration_watchdog.observe(
                    now, distance, self._latest_recovery_count)
                # 2026-10-14 D8：独立判据——"一直在动却没靠近过目标"。
                if (decision in (None, 'tracking')
                        and self._no_net_progress_timed_out(now, distance, pose)):
                    decision = 'stalled'
                if decision in ('progress', 'recovery_started'):
                    self._feedback_baseline_locked = True
                return distance, decision

        geometric_distance = self._geometric_distance_to_active_goal(pose)
        with self._feedback_state_lock:
            if (goal_generation != self._active_goal_generation
                    or not self.nav_goal_active
                    or not self._is_exploration_goal()):
                return None, None
            # Feedback may have arrived while TF was queried.  In that case it
            # owns the baseline and the stale geometric snapshot is discarded.
            if self._distance_source == 'feedback':
                distance = self._latest_feedback_distance
            else:
                distance = geometric_distance
            if (not self._feedback_baseline_locked
                    and now >= self._feedback_baseline_deadline):
                self._feedback_baseline_locked = True
            # 2026-10-14 D3：同上，净推进口径。
            distance = self._path_aware_progress_distance(distance, pose)
            decision = self.exploration_watchdog.observe(
                now, distance, self._latest_recovery_count)
            # 2026-10-14 D8：同上。
            if (decision in (None, 'tracking')
                    and self._no_net_progress_timed_out(now, distance, pose)):
                decision = 'stalled'
            if decision in ('progress', 'recovery_started'):
                self._feedback_baseline_locked = True
            return distance, decision

    def _pose_stall_elapsed(self, now):
        with self._feedback_state_lock:
            if self._pose_stall_anchor_at <= 0.0:
                return 0.0
            return max(0.0, now - self._pose_stall_anchor_at)

    def _pose_stall_yaw_delta(self, yaw):
        """2026-10-12: 当前朝向与停滞锚点的夹角（仅用于诊断打印）。"""
        with self._feedback_state_lock:
            anchor = self._pose_stall_anchor_yaw_value()
        if anchor is None or yaw is None:
            return None
        return abs(math.atan2(math.sin(yaw - anchor),
                              math.cos(yaw - anchor)))

    def _exploration_pose_is_stalled(self, now, distance, pose):
        """Detect a goal that remains inside a small translational bubble."""
        if pose is None:
            return False
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = self.pose_yaw(pose)

        with self._feedback_state_lock:
            if (not self.nav_goal_active
                    or not self._is_exploration_goal()
                    or self._stall_cancel_requested):
                return False
            generation = self._active_goal_generation
            if self._pose_stall_generation != generation:
                self._pose_stall_generation = generation
                self._reset_pose_stall_anchor_locked(now, x, y, yaw)
                return False

            if (distance is not None
                    and distance
                    <= self.exploration_pose_stall_goal_tolerance_m):
                # Do not abandon a goal that is already inside its terminal
                # neighborhood merely because the goal checker is settling.
                self._reset_pose_stall_anchor_locked(now, x, y, yaw)
                return False

            if (self._pose_stall_anchor_x is None
                    or self._pose_stall_anchor_y is None
                    or self._pose_stall_anchor_at <= 0.0):
                self._reset_pose_stall_anchor_locked(now, x, y, yaw)
                return False

            displacement = math.hypot(
                x - self._pose_stall_anchor_x,
                y - self._pose_stall_anchor_y,
            )
            if displacement > self.exploration_pose_stall_radius_m:
                self._reset_pose_stall_anchor_locked(now, x, y, yaw)
                return False

            # 2026-10-12: 朝向变化同样是"在动"的证据。
            # 实测 Nav2 会按控制器指令长期原地纯旋转（|wz| 0.61~1.01 rad/s，
            # 位移恒为 0），原判据把它整段算作"停滞"，7s 后误取消目标。
            # 注意：**不设"累计旋转上限"** —— 正常调头可达数弧度，
            # 设上限会重新引入同类误判。真正卡死（连转都不转）由
            # 18s 看门狗与 50s 单目标硬上限兜底。
            anchor_yaw = self._pose_stall_anchor_yaw_value()
            if anchor_yaw is None:
                # 2026-10-12 N-10a：朝向锚点缺失。实测此时机器人可能正在原地
                # 旋转（cmd_vel.z=-0.476、yaw_delta=none、front_blocked=False），
                # 而原实现在这种情况下**整段跳过**朝向判据，直接落到
                # "超时即停滞"，7s 后就取消目标并黑名单，形成
                # "取消 -> 自转恢复(失败) -> 换目标"的循环。
                # 无法证明它是停着的，就按"在动"处理并补上锚点：
                # 最坏只是把一次真实停滞延后到 18s 看门狗 / 50s 硬上限兜底。
                if yaw is not None:
                    self._reset_pose_stall_anchor_locked(now, x, y, yaw)
                else:
                    self._pose_stall_anchor_at = now
                return False
            yaw_delta = abs(
                math.atan2(math.sin(yaw - anchor_yaw),
                           math.cos(yaw - anchor_yaw)))
            if yaw_delta > self._stall_yaw_threshold():
                self._reset_pose_stall_anchor_locked(now, x, y, yaw)
                return False

            return (
                now - self._pose_stall_anchor_at
                >= self.exploration_pose_stall_timeout_sec
            )

    def _rebase_watchdog_from_commanded_pose_progress(self, now, pose=None):
        """Recognize real TF motion without letting idle TF noise mask a stall."""
        commanded_motion = (
            abs(self.last_cmd_vel.linear.x) >= 0.03
            or abs(self.last_cmd_vel.angular.z) >= 0.10
        )
        if not commanded_motion:
            return False

        with self._feedback_state_lock:
            if (not self.nav_goal_active
                    or not self._is_exploration_goal()
                    or self._stall_cancel_requested):
                return False
            goal_generation = self._active_goal_generation

        if pose is None:
            pose = self.update_pose()
        if pose is None:
            return False
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = self.pose_yaw(pose)

        with self._feedback_state_lock:
            if (goal_generation != self._active_goal_generation
                    or not self.nav_goal_active
                    or not self._is_exploration_goal()
                    or self._stall_cancel_requested):
                return False
            if (self.progress_x is None
                    or self.progress_y is None
                    or self.progress_yaw is None):
                self.progress_x = x
                self.progress_y = y
                self.progress_yaw = yaw
                self.progress_time = now
                return False

            translation = math.hypot(
                x - self.progress_x, y - self.progress_y)
            rotation = abs(math.atan2(
                math.sin(yaw - self.progress_yaw),
                math.cos(yaw - self.progress_yaw)))
            if translation < 0.05 and rotation < 0.12:
                return False

            self.progress_x = x
            self.progress_y = y
            self.progress_yaw = yaw
            self.progress_time = now
            if (self._distance_source == 'feedback'
                    and self._latest_feedback_distance is not None):
                distance = self._latest_feedback_distance
            elif self.last_goal is not None:
                distance = math.hypot(
                    self.last_goal.pose.position.x - x,
                    self.last_goal.pose.position.y - y)
            else:
                distance = None
            self.exploration_watchdog.rebase_distance(
                now, distance, self._latest_recovery_count)
            self._feedback_baseline_locked = True
            self._stall_confirmation_started_at = None
            return True

    def _log_exploration_progress(self, now, decision, distance):
        if now - self._last_progress_log < 2.0:
            return
        distance_text = (
            f'{distance:.3f}m' if distance is not None else 'unavailable')
        best = self.exploration_watchdog.best_distance
        best_text = f'{best:.3f}m' if best is not None else 'unavailable'
        front_text = (
            f'{self.front_min_range:.2f}m'
            if self.front_min_range is not None else 'unavailable')
        pose_still = self._pose_stall_elapsed(now)
        self.get_logger().info(
            'exploration progress: '
            f'source={self._active_goal_kind}, '
            f'elapsed={self.exploration_watchdog.elapsed(now):.1f}s, '
            f'remaining={distance_text}, best={best_text}, '
            f'stagnant={self.exploration_watchdog.stagnant_for(now):.1f}s, '
            f'pose_still={pose_still:.1f}s, '
            f'recoveries={self._latest_recovery_count}, '
            f'front={front_text}, decision={decision}')
        self._last_progress_log = now

    def _progress_watchdog_callback_impl(self):
        now = time.monotonic()
        self._monitor_goal_acceptance(now)
        if self._stall_cancel_requested:
            self._monitor_stall_cancel(now)
            return
        if self._goal_transport_fault:
            self.publish_zero_velocity()
            return
        if not self._is_exploration_goal():
            return super()._progress_watchdog_callback_impl()
        if self.base_charging:
            self.pause_for_charging()
            return
        if (self.mapping_done or not self.auto_explore
                or not self.nav_goal_active):
            return
        if self.is_recovering():
            # 2026-10-12: 恢复动作本身要耗时（Nav2 BT 的 Spin + Wait 约 6~7s，
            # 与 7s 判据几乎重合），且多为纯旋转（位移为 0）。
            # 原实现只 return、**不重置锚点** —— 于是恢复一结束，
            # 计时器恰好到期，立刻被判"停滞"并取消目标（问题三根因）。
            # 这里把锚点推到"现在"，让恢复期间不计入停滞。
            recovering_pose = self.update_pose()
            if recovering_pose is not None:
                with self._feedback_state_lock:
                    if (self._pose_stall_generation
                            == self._active_goal_generation):
                        self._reset_pose_stall_anchor_locked(
                            now,
                            recovering_pose.pose.position.x,
                            recovering_pose.pose.position.y,
                            self.pose_yaw(recovering_pose))
            return

        pose = self.update_pose()
        distance_hint = self._distance_to_active_goal(pose)
        if self._exploration_pose_is_stalled(now, distance_hint, pose):
            self._log_exploration_progress(
                now, 'pose_stalled_switching_goal', distance_hint)
            # 2026-10-10 诊断：判定"7s 静止"到底由什么造成 ——
            # 前方闸门切断运动？Nav2 正在恢复？还是真的卡死？
            self.get_logger().warn(
                'pose_stalled ctx: pose=(%.2f, %.2f) remaining=%.2fm '
                'front=%.3f center=%.3f wide=%.3f left=%s right=%s '
                'front_blocked=%s recovery_active=%s recovery_turn_dir=%.1f '
                'recovery_clear_count=%d nav_state=%s mode=%s '
                'watchdog[elapsed=%.1fs stagnant=%.1fs best=%s recov=%d] '
                'cmd_vel=(%.3f, %.3f) pose_stall[yaw_delta=%s yaw_rad=%.2f]'
                % (pose.pose.position.x, pose.pose.position.y,
                   float(distance_hint) if distance_hint is not None else -1.0,
                   self._as_float(self.front_min_range),
                   self._as_float(self.front_center_min_range),
                   self._as_float(self.front_wide_min_range),
                   self._fmt_opt(self.left_min_range),
                   self._fmt_opt(self.right_min_range),
                   bool(self.front_blocked),
                   bool(self.is_recovering()),
                   float(getattr(self, 'recovery_turn_direction', 0.0) or 0.0),
                   int(getattr(self, 'recovery_clear_count', 0) or 0),
                   self.navigation_state or 'n/a', self.mode or 'n/a',
                   self.exploration_watchdog.elapsed(now),
                   self.exploration_watchdog.stagnant_for(now),
                   self._fmt_opt(self.exploration_watchdog.best_distance),
                   int(self.exploration_watchdog.recovery_count or 0),
                   self._as_float(self.last_cmd_vel.linear.x),
                   self._as_float(self.last_cmd_vel.angular.z),
                   self._fmt_opt(self._pose_stall_yaw_delta(
                       self.pose_yaw(pose))),
                   self._stall_yaw_threshold()))
            self._stall_confirmation_started_at = None
            self._trigger_stalled_exploration_recovery(
                'exploration goal stayed within '
                f'{self.exploration_pose_stall_radius_m:.2f}m for '
                f'{self.exploration_pose_stall_timeout_sec:.1f}s; '
                'blacklisting it and switching targets after recovery',
                distance_hint,
            )
            return

        self._rebase_watchdog_from_commanded_pose_progress(now, pose)
        distance, decision = self._observe_exploration_progress(now, pose)
        # 2026-10-10 诊断：实测 hard_timeout(50s) 触发 89 次而 stalled(18s
        # 无净改善) 触发 0 次，形态异常。此 1 Hz trace 打印看门狗的全部
        # 输入量，用于判定是"确实一直在改善"还是"看门狗被反复重置"。
        if now - getattr(self, '_watchdog_trace_at', 0.0) >= 1.0:
            self._watchdog_trace_at = now
            wd = self.exploration_watchdog
            if wd.active:
                self.get_logger().info(
                    'watchdog trace: elapsed=%.1f/%.1f stagnant=%.1f/%.1f '
                    'best=%s now_dist=%s decision=%s recov=%d '
                    'confirmation=%s'
                    % (wd.elapsed(now), wd.hard_timeout_sec,
                       wd.stagnant_for(now), wd.no_progress_timeout_sec,
                       self._fmt_opt(wd.best_distance),
                       self._fmt_opt(distance),
                       decision or 'none',
                       int(getattr(wd, 'recovery_count', 0) or 0),
                       ('pending' if self._stall_confirmation_started_at
                        else 'no')))
        if decision is None:
            return
        self._log_exploration_progress(now, decision, distance)

        if decision == 'recovery_started':
            self._stall_confirmation_started_at = None
            self.get_logger().warn(
                'Nav2 began its staged behavior-tree recovery after '
                f'{self.exploration_watchdog.elapsed(now):.1f}s; '
                f'granting at most another '
                f'{self.exploration_watchdog.no_progress_timeout_sec:.1f}s '
                'for net progress')
            return
        if decision not in ('stalled', 'hard_timeout'):
            self._stall_confirmation_started_at = None
            return

        if decision == 'stalled':
            if self._stall_confirmation_started_at is None:
                self._stall_confirmation_started_at = now
                self.navigation_state = 'confirming_exploration_stall'
                self.get_logger().warn(
                    'exploration no-progress window elapsed; requiring one '
                    'additional stagnant watchdog observation before cancel')
                return
            if (now - self._stall_confirmation_started_at
                    < self.stall_confirmation_sec):
                return

        if decision == 'hard_timeout':
            reason = (
                'exploration goal reached its '
                f'{self.exploration_watchdog.hard_timeout_sec:.0f}s '
                'hard limit')
        else:
            reason = (
                'exploration goal made no net remaining-distance improvement '
                f'of {self.exploration_watchdog.min_progress_m:.2f}m for '
                f'{self.exploration_watchdog.no_progress_timeout_sec:.0f}s')
        self._stall_confirmation_started_at = None
        self._trigger_stalled_exploration_recovery(reason, distance)

    @staticmethod
    def _as_float(value, default=0.0):
        """2026-10-10 诊断辅助：None / 非数安全转换（纯读取）。"""
        try:
            if value is None:
                return float(default)
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    @classmethod
    def _fmt_opt(cls, value):
        """诊断辅助：None 打印为 'none'，否则保留 3 位小数。"""
        if value is None:
            return 'none'
        return '%.3f' % cls._as_float(value)

    def _reset_stall_cancel_tracking(self):
        future = getattr(self, '_cancel_response_future', None)
        if future is not None and not future.done():
            try:
                future.cancel()
            except Exception:  # noqa: BLE001
                pass
        self._stall_cancel_requested = False
        self._recovery_after_cancel = False
        self._cancel_response_future = None
        self._cancel_started_at = 0.0
        self._cancel_last_attempt_at = 0.0
        self._cancel_attempts = 0
        self._cancel_acknowledged = False
        self._cancel_timeout_reported = False
        self._cancel_context = 'navigation_goal'
        self._last_cancel_zero_time = 0.0

    def _fail_stall_cancel(self, detail):
        self._recovery_after_cancel = False
        self._clear_recovery_intent(clear_episode=True)
        self._cancel_response_future = None
        self._cancel_timeout_reported = True
        self._goal_transport_fault = True
        self.auto_explore = False
        self._stop_manager_recovery_motion()
        self.mode = 'safety_stop'
        self.navigation_state = f'{self._cancel_context}_cancel_timeout'
        self.last_error = f'{self.last_error}; {detail}'
        self.publish_zero_velocity()
        self.get_logger().error(
            f'{self.last_error}; recovery and new goals are suppressed until '
            'the mapping session is restarted')

    def _begin_monitored_cancel(self, context, recover_after_cancel, now):
        self._reset_stall_cancel_tracking()
        self._stall_cancel_requested = True
        self._recovery_after_cancel = bool(recover_after_cancel)
        self._cancel_context = context
        self._cancel_started_at = now
        return self._send_stall_cancel_request(now)

    @staticmethod
    def _cancel_context_priority(context):
        return {
            'navigation_goal': 0,
            'stalled': 10,
            'recovery_during_goal_acceptance': 20,
            'return_home_orientation_timeout': 25,
            'base_charging': 30,
            'mapping_done': 40,
            'stop_navigation': 50,
            'timed_out_goal': 60,
            'goal_response_error': 60,
            'goal_result_error': 60,
        }.get(context, 0)

    def _retag_monitored_cancel(self, context, recover_after_cancel=False):
        """Change intent without resending cancel or extending its deadline."""
        self._recovery_after_cancel = (
            self._recovery_after_cancel and bool(recover_after_cancel))
        if (self._cancel_context_priority(context)
                >= self._cancel_context_priority(self._cancel_context)):
            self._cancel_context = context

    def _arm_cancel_on_accept(self, context):
        current = self._cancel_on_accept_context
        if (current is None
                or self._cancel_context_priority(context)
                >= self._cancel_context_priority(current)):
            self._cancel_on_accept_context = context

    def _send_stall_cancel_request(self, now):
        if self.current_goal_handle is None:
            self._fail_stall_cancel(
                'no action goal handle was available for exact cancellation')
            return False
        if self._cancel_attempts >= self.stall_cancel_max_attempts:
            return False

        self._cancel_attempts += 1
        self._cancel_last_attempt_at = now
        try:
            self._cancel_response_future = (
                self.current_goal_handle.cancel_goal_async())
        except Exception as exc:  # noqa: BLE001
            self._cancel_response_future = None
            self.get_logger().error(
                f'{self._cancel_context} cancel request '
                f'{self._cancel_attempts}/{self.stall_cancel_max_attempts} '
                f'could not be sent: {exc}')
            if self._cancel_attempts >= self.stall_cancel_max_attempts:
                self._fail_stall_cancel(
                    'all stalled-goal cancel requests failed to send')
            return False

        self.get_logger().warn(
            f'{self._cancel_context} cancel request sent: '
            f'attempt={self._cancel_attempts}/'
            f'{self.stall_cancel_max_attempts}')
        return True

    def _monitor_stall_cancel(self, now):
        # Continue asserting zero at the 1 Hz watchdog rate; the 5 Hz safety
        # callback below supplies the faster guard while cancellation settles.
        self.publish_zero_velocity()
        self._last_cancel_zero_time = now
        if self._cancel_timeout_reported:
            return
        if (self._cancel_started_at > 0.0
                and now - self._cancel_started_at
                >= self.stall_cancel_result_timeout_sec):
            result_future = self.goal_result_future
            if result_future is not None and result_future.done():
                # Humble may queue a Future's done callback behind a timer
                # that was ready in the same executor batch. Consume the exact
                # terminal result before declaring a cancel transport failure;
                # the strict Future identity guard makes its queued callback
                # harmless when it eventually runs.
                self.get_logger().warn(
                    'navigation result became ready at the cancel timeout '
                    'boundary; consuming the exact terminal result before '
                    'declaring a transport failure')
                self.goal_result_callback(result_future)
                return
            self._fail_stall_cancel(
                'Nav2 did not report the canceled goal result within '
                f'{self.stall_cancel_result_timeout_sec:.1f}s')
            return

        future = self._cancel_response_future
        if future is not None:
            if future.done():
                self._cancel_response_future = None
                try:
                    response = future.result()
                    accepted = bool(response.goals_canceling)
                except Exception as exc:  # noqa: BLE001
                    accepted = False
                    self.get_logger().error(
                        f'{self._cancel_context} cancel response failed: '
                        f'{exc}')
                if accepted:
                    self._cancel_acknowledged = True
                    self.get_logger().warn(
                        f'Nav2 accepted {self._cancel_context} cancellation; '
                        'waiting for the terminal action result before any '
                        'next motion')
                    return
                self.get_logger().warn(
                    f'Nav2 rejected {self._cancel_context} cancellation '
                    'request '
                    f'{self._cancel_attempts}/'
                    f'{self.stall_cancel_max_attempts}')
            elif (now - self._cancel_last_attempt_at
                  < self.stall_cancel_response_timeout_sec):
                return
            else:
                try:
                    future.cancel()
                except Exception:  # noqa: BLE001
                    pass
                self._cancel_response_future = None
                self.get_logger().warn(
                    f'{self._cancel_context} cancel response timed out after '
                    f'{self.stall_cancel_response_timeout_sec:.1f}s')

        if self._cancel_acknowledged:
            return
        if self._cancel_attempts < self.stall_cancel_max_attempts:
            self._send_stall_cancel_request(now)
            return
        self._fail_stall_cancel(
            f'Nav2 did not accept cancellation after '
            f'{self._cancel_attempts} attempts')

    def _trigger_stalled_exploration_recovery(self, reason, distance):
        if self._stall_cancel_requested:
            return
        self.remember_failed_goal()
        self.remember_blocked_place()
        self.publish_zero_velocity()
        self.mode = 'safety_stop'
        self.navigation_state = 'canceling_stalled_exploration_goal'
        self._recovery_intent = 'stalled_cancel'
        distance_text = (
            f'{distance:.3f}m' if distance is not None else 'unavailable')
        self.last_error = (
            f'{reason}; source={self._active_goal_kind}, '
            f'remaining={distance_text}')
        now = time.monotonic()
        self._begin_monitored_cancel('stalled', True, now)
        if self._cancel_timeout_reported:
            return
        self.get_logger().warn(
            f'{self.last_error}; zero velocity sent, waiting for Nav2 cancel '
            'acknowledgement before controlled recovery')

    def _safety_stop_callback_impl(self):
        now = time.monotonic()
        self._maybe_complete_return_home_by_position(now)
        if (self._stall_cancel_requested
                or self._cancel_on_accept_context is not None
                or self._goal_transport_fault
                or self._spin_transport_fault):
            self.publish_zero_velocity()
            return

        self._update_blocked_episode(now)
        if self._spin_phase != 'idle':
            # Nav2 BehaviorServer owns cmd_vel only while Spin is executing.
            # Pending/canceling phases remain clamped to zero.
            if self._spin_phase != 'executing':
                self.publish_zero_velocity()
            return

        if not self.front_blocked:
            blocked_states = (
                'blocked_precheck_pending',
                'blocked_waiting_clearance',
                'recovery_cooldown',
                'spin_recovery_rejected',
                'spin_recovery_aborted',
            )
            if (self._blocked_episode_started_at is not None
                    and self.navigation_state in blocked_states):
                self.publish_zero_velocity()
                return
            return super()._safety_stop_callback_impl()

        if self.current_goal_handle is not None and self.nav_goal_active:
            # Preserve the existing Nav2 BT/controller collision policy.  A
            # manager recovery is allowed only after the upper watchdog has
            # canceled this exact goal and received its terminal result.
            return super()._safety_stop_callback_impl()

        self.remember_blocked_place()
        self.mode = 'safety_stop'
        measured = (
            f'{self.front_min_range:.2f}m'
            if self.front_min_range is not None else 'unavailable')
        self.last_error = f'front obstacle too close: {measured}'
        if self._precheck_recovery_is_ready(now):
            # Consume the one-shot intent before attempting recovery so a
            # rejection or transport error cannot cause a timer-driven storm.
            self._blocked_episode_attempted = True
            self._recovery_intent = None
            self._next_blocked_recovery_at = (
                now + self.blocked_recovery_cooldown_sec)
            if self._start_stall_spin_recovery('exploration_precheck'):
                return

        self.publish_zero_velocity()
        if self.navigation_state not in (
                'blocked_precheck_pending',
                'spin_recovery_rejected',
                'spin_recovery_aborted'):
            self.navigation_state = 'blocked_waiting_clearance'
        self._log_recovery_event(
            'front_blocked_without_active_goal', self.last_error)

    def _request_costmap_clears(self):
        if not self.clear_costmaps_on_stall:
            return
        self._costmap_clear_futures = []
        requested = []
        for name, client in self._costmap_clear_clients.items():
            if not client.service_is_ready():
                self.get_logger().warn(
                    f'{name} costmap clear service unavailable; continuing '
                    'with bounded motion recovery')
                continue
            try:
                future = client.call_async(ClearEntireCostmap.Request())
                if future is None:
                    raise RuntimeError('clear service returned no future')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'failed to request {name} costmap clear: {exc}')
                continue
            self._costmap_clear_futures.append(future)
            requested.append(name)
        if requested:
            self.get_logger().info(
                'requested global costmap clear before controlled recovery; '
                'local collision costmap remains intact: '
                + ','.join(requested))

    def is_recovering(self):
        return (
            getattr(self, '_spin_phase', 'idle') != 'idle'
            or super().is_recovering()
        )

    def _reset_spin_tracking(self):
        self._spin_phase = 'idle'
        self._spin_generation = None
        self._spin_context = None
        self._spin_send_future = None
        self._spin_goal_handle = None
        self._spin_result_future = None
        self._spin_cancel_future = None
        self._spin_cancel_requested = False
        self._spin_cancel_acknowledged = False
        self._spin_phase_started_at = 0.0
        self._spin_cancel_started_at = 0.0
        self._spin_cancel_on_accept_reason = None
        self._spin_cancel_reason = None
        self._cartographer_spin_active = False
        self.recovery_until = 0.0
        self.recovery_started_at = 0.0
        self.recovery_clear_count = 0
        self.recovery_twist = Twist()

    @staticmethod
    def _spin_cancel_reason_priority(reason):
        return {
            'spin_acceptance_timeout': 10,
            'spin_result_timeout': 10,
            'spin_result_monitor_error': 20,
            'spin_transport_fault': 20,
            'navigation_transport_fault': 20,
            'base_charging': 30,
            'mapping_done': 40,
            'stop_navigation': 50,
        }.get(reason, 0)

    def _set_spin_cancel_reason(self, reason):
        current = self._spin_cancel_reason
        if (current is None
                or self._spin_cancel_reason_priority(reason)
                >= self._spin_cancel_reason_priority(current)):
            self._spin_cancel_reason = reason
        current_pending = self._spin_cancel_on_accept_reason
        if (current_pending is None
                or self._spin_cancel_reason_priority(reason)
                >= self._spin_cancel_reason_priority(current_pending)):
            self._spin_cancel_on_accept_reason = reason

    def _finish_spin_recovery(self, state, error=''):
        self._reset_spin_tracking()
        super().finish_recovery(state, error)

    def _spin_yaw_for_cycle(self):
        magnitude = (
            self.stall_spin_escalated_yaw_rad
            if self.stuck_recovery_cycles >= 2
            else self.stall_spin_yaw_rad
        )
        return magnitude * self.recovery_turn_direction

    def _start_stall_spin_recovery(self, context='stalled_cancel'):
        """Queue one collision-checked Nav2 Spin for an explicit intent."""
        if (self._spin_phase != 'idle'
                or super().is_recovering()
                or self.goal_send_future is not None
                or self.nav_goal_active
                or self._stall_cancel_requested
                or self._goal_transport_fault
                or self._spin_transport_fault
                or self.mapping_done
                or not self.auto_explore
                or self.base_charging):
            self.publish_zero_velocity()
            self._log_recovery_event(
                f'spin_start_suppressed:{context}',
                'Nav2 Spin recovery suppressed by the current navigation '
                f'safety state; context={context}',
            )
            return False
        if self.stuck_recovery_cycles >= 10:
            super().finish_recovery(
                'give_up_stuck',
                'robot remained stuck after 10 collision-checked Spin '
                'attempts; waiting for manual help',
            )
            return False

        now = time.monotonic()
        left_range = self.left_min_range or 0.0
        right_range = self.right_min_range or 0.0
        preferred = 1.0 if left_range >= right_range else -1.0
        if self.recovery_turn_direction == 0.0:
            self.recovery_turn_direction = preferred
        elif abs(left_range - right_range) < 0.25:
            self.recovery_turn_direction *= -1.0
        else:
            self.recovery_turn_direction = preferred

        self.stuck_recovery_cycles += 1
        self._spin_generation_counter += 1
        self._spin_generation = self._spin_generation_counter
        self._spin_context = context
        self._spin_phase = 'clearing'
        self._spin_phase_started_at = now
        self._cartographer_spin_active = True
        self.recovery_started_at = now
        self.recovery_until = 0.0
        self.recovery_twist = Twist()
        self.mode = 'safety_stop'
        self.navigation_state = 'clearing_costmaps_for_spin_recovery'
        self.last_error = ''
        self._request_costmap_clears()
        self.get_logger().warn(
            'queued collision-checked Nav2 Spin recovery: '
            f'context={context}, cycle={self.stuck_recovery_cycles}, '
            f'target_yaw={self._spin_yaw_for_cycle():+.2f}rad')
        return True

    def _dispatch_spin_goal(self, generation):
        if (generation != self._spin_generation
                or self._spin_phase != 'clearing'):
            return
        if not self.spin_client.server_is_ready():
            message = (
                f'Nav2 Spin action {self.stall_spin_action_name} is not ready')
            self._finish_spin_recovery('spin_recovery_rejected', message)
            self._log_recovery_event('spin_action_unavailable', message)
            return

        goal = Spin.Goal()
        goal.target_yaw = float(self._spin_yaw_for_cycle())
        whole_seconds = int(self.stall_spin_time_allowance_sec)
        goal.time_allowance.sec = whole_seconds
        goal.time_allowance.nanosec = int(
            (self.stall_spin_time_allowance_sec - whole_seconds) * 1.0e9)
        self._spin_phase = 'sending'
        self._spin_phase_started_at = time.monotonic()
        self.navigation_state = 'sending_spin_recovery'
        try:
            future = self.spin_client.send_goal_async(goal)
            if future is None:
                raise RuntimeError('send_goal_async returned no future')
        except Exception as exc:  # noqa: BLE001
            message = f'failed to dispatch Nav2 Spin recovery: {exc}'
            self._finish_spin_recovery('spin_recovery_rejected', message)
            self._log_recovery_event('spin_send_failed', message, error=True)
            return
        self._spin_send_future = future
        future.add_done_callback(partial(
            self._spin_goal_response_callback, generation=generation))

    def _spin_goal_response_callback(self, future, generation):
        if generation != self._spin_generation:
            return
        if self._spin_send_future is future:
            self._spin_send_future = None
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            message = f'Nav2 Spin goal response failed: {exc}'
            self._finish_spin_recovery('spin_recovery_rejected', message)
            self._log_recovery_event(
                'spin_response_failed', message, error=True)
            return
        if goal_handle is None or not goal_handle.accepted:
            message = 'Nav2 Spin recovery goal was rejected'
            cancel_reason = self._spin_cancel_reason
            self._spin_transport_fault = False
            self._finish_spin_recovery('spin_recovery_rejected', message)
            self._log_recovery_event('spin_goal_rejected', message)
            if cancel_reason in (
                    'mapping_done', 'stop_navigation', 'base_charging'):
                self._complete_spin_external_transition(cancel_reason)
            return

        self._spin_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            if result_future is None:
                raise RuntimeError('get_result_async returned no future')
        except Exception as exc:  # noqa: BLE001
            self._spin_transport_fault = True
            self.auto_explore = False
            self.mode = 'safety_stop'
            self.navigation_state = 'spin_result_monitor_error'
            self.last_error = f'could not monitor Nav2 Spin result: {exc}'
            self.publish_zero_velocity()
            self._log_recovery_event(
                'spin_result_monitor_error', self.last_error, error=True)
            self._request_spin_cancel('spin_result_monitor_error')
            return
        cancel_reason = self._spin_cancel_on_accept_reason
        self._spin_cancel_on_accept_reason = None
        self._spin_phase = 'executing'
        self._spin_phase_started_at = time.monotonic()
        self.navigation_state = 'turning_out_of_blocked_place'
        self._spin_result_future = result_future
        result_future.add_done_callback(partial(
            self._spin_result_callback, generation=generation))
        # A test double, or a very fast action server, can complete the result
        # future while the callback is being registered.  Never resurrect a
        # Spin state after that exact terminal result has reset its generation.
        if (generation != self._spin_generation
                or self._spin_phase == 'idle'):
            return
        if cancel_reason is not None:
            self._request_spin_cancel(cancel_reason)
            return
        self.get_logger().warn(
            'Nav2 Spin recovery accepted; BehaviorServer owns collision '
            'checking and cmd_vel until the exact terminal result')

    def _spin_cancel_response_callback(self, future, generation):
        if generation != self._spin_generation:
            return
        if self._spin_cancel_future is future:
            self._spin_cancel_future = None
        try:
            response = future.result()
            canceling = getattr(response, 'goals_canceling', ())
            if not canceling:
                raise RuntimeError('Spin action did not acknowledge cancel')
            self._spin_cancel_acknowledged = True
        except Exception as exc:  # noqa: BLE001
            self._spin_transport_fault = True
            self.auto_explore = False
            self.mode = 'safety_stop'
            self.navigation_state = 'spin_cancel_error'
            self.last_error = f'Nav2 Spin cancellation failed: {exc}'
            self.publish_zero_velocity()
            self._log_recovery_event(
                'spin_cancel_error', self.last_error, error=True)

    def _request_spin_cancel(self, reason):
        if self._spin_phase == 'idle':
            return False
        self._set_spin_cancel_reason(reason)
        self.publish_zero_velocity()

        if self._spin_phase == 'clearing':
            self._finish_spin_recovery('recovery_canceled')
            return True
        if self._spin_goal_handle is None:
            if self._spin_phase != 'acceptance_timeout':
                self._spin_phase = 'canceling_pending_acceptance'
            self.navigation_state = f'{reason}_canceling_pending_spin'
            return True
        if self._spin_cancel_requested:
            return True

        cancel_started_at = time.monotonic()
        self._spin_phase = 'canceling'
        self._spin_phase_started_at = cancel_started_at
        self._spin_cancel_started_at = cancel_started_at
        self.navigation_state = f'{reason}_canceling_spin'
        self._spin_cancel_requested = True
        self._spin_cancel_acknowledged = False
        try:
            future = self._spin_goal_handle.cancel_goal_async()
            if future is None:
                raise RuntimeError('cancel_goal_async returned no future')
        except Exception as exc:  # noqa: BLE001
            self._spin_transport_fault = True
            self.auto_explore = False
            self.mode = 'safety_stop'
            self.navigation_state = 'spin_cancel_error'
            self.last_error = f'could not cancel Nav2 Spin recovery: {exc}'
            self.publish_zero_velocity()
            self._log_recovery_event(
                'spin_cancel_dispatch_error', self.last_error, error=True)
            return False
        self._spin_cancel_future = future
        generation = self._spin_generation
        future.add_done_callback(partial(
            self._spin_cancel_response_callback, generation=generation))
        return True

    def _complete_spin_external_transition(self, reason):
        if reason == 'mapping_done' and self.pending_home_goal:
            self.publish_zero_velocity()
            started, message = self._dispatch_pending_home_goal()
            if started:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        elif reason == 'stop_navigation':
            self.mode = 'stopped'
            self.navigation_state = 'canceled'
            self.last_error = ''
        elif reason == 'base_charging':
            self.mode = 'paused'
            self.navigation_state = 'base_charging'
            self.last_error = 'base is charging; navigation paused'

    def _spin_result_callback(self, future, generation):
        if generation != self._spin_generation:
            return
        cancel_reason = self._spin_cancel_reason
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self._spin_transport_fault = True
            self.auto_explore = False
            self.mode = 'safety_stop'
            self.navigation_state = 'spin_result_error'
            self.last_error = f'Nav2 Spin result failed: {exc}'
            self.publish_zero_velocity()
            self._log_recovery_event(
                'spin_result_error', self.last_error, error=True)
            return

        self._spin_transport_fault = False
        if cancel_reason in (
                'mapping_done', 'stop_navigation', 'base_charging'):
            self._finish_spin_recovery('recovery_canceled')
            self._complete_spin_external_transition(cancel_reason)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_spin_recovery('recovery_clear')
            self.get_logger().info(
                'Nav2 Spin recovery completed with collision checking')
            return
        if status == GoalStatus.STATUS_CANCELED:
            message = (
                'Nav2 Spin recovery was canceled before completion'
                if cancel_reason is None else
                f'Nav2 Spin recovery canceled after {cancel_reason}')
            self._finish_spin_recovery('blocked_waiting_clearance', message)
            self._log_recovery_event('spin_result_canceled', message)
            return

        message = (
            'Nav2 Spin recovery aborted; BehaviorServer collision checking '
            'did not find a safe rotation')
        self._finish_spin_recovery('spin_recovery_aborted', message)
        self._log_recovery_event('spin_result_aborted', message)

    def finish_recovery(self, state, error=''):
        return super().finish_recovery(state, error)

    def _recovery_turn_callback_impl(self):
        if self._spin_phase == 'idle':
            return super()._recovery_turn_callback_impl()

        now = time.monotonic()
        if self.mapping_done:
            self._request_spin_cancel('mapping_done')
        elif self.base_charging:
            self._request_spin_cancel('base_charging')
        elif self._spin_transport_fault:
            self._request_spin_cancel(
                self._spin_cancel_reason or 'spin_transport_fault')
        elif self._goal_transport_fault:
            self._request_spin_cancel('navigation_transport_fault')
        elif not self.auto_explore:
            # stop_navigation_callback() installs the higher-priority explicit
            # stop reason synchronously.  A plain false value can also result
            # from an internal fault and must not erase its diagnostic state.
            self._request_spin_cancel('exploration_disabled')
        if self._spin_phase == 'idle':
            return

        if self._spin_phase == 'clearing':
            clears_done = all(
                future.done() for future in self._costmap_clear_futures)
            if (clears_done
                    or now - self._spin_phase_started_at
                    >= self.costmap_clear_wait_sec):
                self._dispatch_spin_goal(self._spin_generation)
            return
        if self._spin_phase in ('sending', 'canceling_pending_acceptance'):
            if (now - self._spin_phase_started_at
                    >= self.stall_spin_acceptance_timeout_sec):
                self._spin_transport_fault = True
                self.auto_explore = False
                self._set_spin_cancel_reason('spin_acceptance_timeout')
                self._spin_phase = 'acceptance_timeout'
                self.mode = 'safety_stop'
                self.navigation_state = 'spin_acceptance_timeout'
                self.last_error = (
                    'Nav2 Spin action did not answer goal acceptance within '
                    f'{self.stall_spin_acceptance_timeout_sec:.1f}s; waiting '
                    'for the exact goal handle before canceling')
                self.publish_zero_velocity()
                self._log_recovery_event(
                    'spin_acceptance_timeout', self.last_error, error=True)
            return
        if self._spin_phase == 'executing':
            if (now - self._spin_phase_started_at
                    >= self.stall_spin_result_timeout_sec):
                self.navigation_state = 'spin_result_timeout'
                self._request_spin_cancel('spin_result_timeout')
            return
        if self._spin_phase == 'canceling':
            if (now - self._spin_cancel_started_at
                    >= self.stall_spin_cancel_timeout_sec):
                self._spin_transport_fault = True
                self.auto_explore = False
                self.mode = 'safety_stop'
                self.navigation_state = 'spin_cancel_timeout'
                self.last_error = (
                    'Nav2 Spin cancellation did not reach an exact terminal '
                    f'result within {self.stall_spin_cancel_timeout_sec:.1f}s')
                self.publish_zero_velocity()
                self._log_recovery_event(
                    'spin_cancel_timeout', self.last_error, error=True)

    def start_escape_recovery(self):
        if (self._stall_cancel_requested
                or self._goal_transport_fault
                or self._spin_transport_fault
                or self.base_charging):
            self._stop_manager_recovery_motion()
            self.mode = 'safety_stop'
            return
        tracked_kind = (
            self._pending_goal_kind
            or self._active_goal_kind
            or self._goal_kind_context
        )
        if self._is_exploration_goal(tracked_kind):
            # Base auto-exploration reaches this method with no pending/active
            # goal but with an exploration goal-kind context.  Preserve that
            # context and arm the existing delayed, one-shot Nav2 Spin path.
            # A safety-timer tick on a static obstacle has no exploration
            # context and therefore cannot start motion by itself.
            if (self.front_blocked
                    and self.auto_explore
                    and not self.mapping_done
                    and not self.base_charging
                    and self.goal_send_future is None
                    and not self.nav_goal_active
                    and self._spin_phase == 'idle'
                    and not self._goal_transport_fault
                    and not self._spin_transport_fault):
                self._arm_exploration_precheck_recovery(time.monotonic())
            self.publish_zero_velocity()
            self.mode = 'safety_stop'
            if self.navigation_state not in (
                    'blocked_precheck_pending', 'blocked_waiting_clearance'):
                self.navigation_state = 'blocked_waiting_clearance'
            return
        if (tracked_kind == 'home'
                or self._goal_kind_context == 'home'):
            return super().start_escape_recovery()
        if (tracked_kind == 'manual'
                or self._goal_kind_context == 'manual'):
            return super().start_escape_recovery()
        self.publish_zero_velocity()

    def pause_for_charging(self):
        """Pause motion and cancel through the same bounded state machine."""
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            return
        self._reset_return_home_xy_completion()
        self._clear_recovery_intent(clear_episode=True)
        if self._spin_phase != 'idle':
            self._request_spin_cancel('base_charging')
            if self._spin_phase == 'idle':
                self._complete_spin_external_transition('base_charging')
            else:
                self.mode = 'paused'
                self.navigation_state = 'charging_canceling_spin'
                self.last_error = (
                    'base is charging; waiting for exact Spin cancellation')
            return
        self._stop_manager_recovery_motion()
        self.recovery_turn_direction = 0.0
        self.mode = 'paused'
        self.navigation_state = 'base_charging'
        self.last_error = 'base is charging; navigation paused'

        if self.goal_send_future is not None:
            self._arm_cancel_on_accept('base_charging')
            self.navigation_state = 'charging_waiting_for_goal_acceptance'
        elif self.current_goal_handle is not None and self.nav_goal_active:
            if self._stall_cancel_requested:
                self._retag_monitored_cancel('base_charging', False)
            else:
                self._begin_monitored_cancel(
                    'base_charging', False, time.monotonic())
            self.navigation_state = 'charging_canceling_active_goal'

        now = time.monotonic()
        if now - self.last_charging_log > 5.0:
            self.get_logger().warn(self.last_error)
            self.last_charging_log = now

    def save_map_callback(self, request, response):
        """Keep pre-finish callers from overwriting the final map."""
        del request
        if not self.mapping_done:
            response.success = False
            response.message = (
                'map save deferred: call set_mapping_done; the manager will '
                'save the final map after finish_trajectory')
            return response
        if self._final_map_saved:
            response.success = True
            response.message = (
                f'final map already saved: '
                f'{os.path.expanduser(self.map_save_path)}.yaml')
            return response
        response.success = False
        response.message = (
            'final map save is still pending in state '
            f'{self.navigation_state}')
        if self._final_map_save_error:
            response.message = (
                f'final map save failed: {self._final_map_save_error}')
        return response

    def _request_mapping_cancellation(self, reason, detail):
        """Begin the existing controlled session shutdown exactly once."""
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            return
        self._reset_return_home_xy_completion()
        self._mapping_cancel_requested = True
        self._mapping_cancel_reason = str(reason or 'return_home_failed')
        self._mapping_cancel_detail = str(detail or '')
        self._mapping_cancel_shutdown_pending = True
        # Let the service response/status frame leave the executor before the
        # launch process group is interrupted; this preserves observability.
        self._mapping_cancel_shutdown_at = time.monotonic() + 0.2
        self.pending_home_goal = False
        self.return_home_requested = False
        self.auto_explore = False
        self._clear_recovery_intent(clear_episode=True)
        self.publish_zero_velocity()
        self.mode = 'stopped'
        self.navigation_state = 'mapping_cancel_requested'
        self.last_error = ''
        self._write_session_status()

    def cancel_mapping_callback(self, request, response):
        # Safely leave a return-home failure without pretending it recovered.
        del request
        if self._mapping_cancel_requested:
            self.publish_zero_velocity()
            response.success = True
            response.message = (
                'mapping cancellation is already in progress; '
                'waiting for the session runner')
            return response
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'normal return-home completion is already finalizing; '
                'mapping cancellation was not started')
            return response

        if (
                not self.mapping_done or
                self.navigation_state != 'return_home_failed_waiting_manual'):
            response.success = False
            response.message = (
                'cancel_mapping is only available after return-home failure')
            return response

        self._request_mapping_cancellation(
            'return_home_failed', str(self.last_error or ''))
        response.success = True
        response.message = (
            'mapping cancellation accepted; waiting for safe session shutdown')
        return response

    def stop_navigation_callback(self, request, response):
        del request
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            response.success = True
            response.message = (
                'navigation is already stopped while the completed mapping '
                'session finalizes')
            return response
        self._reset_return_home_xy_completion()
        # A successful stop request is the mapping pause contract, not
        # merely an instantaneous Nav2 cancellation state.
        self._voice_pause_latched = True
        self.pending_home_goal = False
        self.return_home_requested = False
        self.return_home_retry_count = 0
        self.auto_explore = False
        self._clear_recovery_intent(clear_episode=True)
        if self._spin_phase != 'idle':
            self._request_spin_cancel('stop_navigation')
            if self._spin_phase == 'idle':
                self._complete_spin_external_transition('stop_navigation')
            response.success = True
            response.message = (
                'navigation stop requested; waiting for exact Spin terminal '
                'result' if self._spin_phase != 'idle' else
                'navigation stopped before Spin dispatch')
            return response
        self._stop_manager_recovery_motion()
        self.recovery_turn_direction = 0.0

        if self.goal_send_future is not None:
            self._arm_cancel_on_accept('stop_navigation')
            self.mode = 'safety_stop'
            self.navigation_state = 'stop_waiting_for_goal_acceptance'
            response.success = True
            response.message = (
                'navigation stop armed; pending goal will be canceled '
                'immediately if Nav2 accepts it')
            return response
        if self.current_goal_handle is not None and self.nav_goal_active:
            self.last_error = 'navigation stop requested'
            if self._stall_cancel_requested:
                self._retag_monitored_cancel('stop_navigation', False)
            else:
                self._begin_monitored_cancel(
                    'stop_navigation', False, time.monotonic())
            self.mode = 'safety_stop'
            self.navigation_state = 'stop_canceling_active_goal'
            response.success = True
            response.message = (
                'navigation stop requested; waiting for exact goal '
                'cancellation result')
            return response

        self.mode = 'stopped'
        self.navigation_state = 'canceled'
        response.success = True
        response.message = 'navigation stopped and zero velocity published'
        return response

    def set_mapping_done_callback(self, request, response):
        requested_done = bool(request.data)
        if self._mapping_cancel_requested:
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'mapping cancellation is in progress; wait for the session '
                'runner to restore functional services')
            return response
        if self._mapping_timeout_failure_kind:
            self.publish_zero_velocity()
            response.success = False
            response.message = (
                'mapping exit has timed out; wait for controlled session '
                'cancellation and functional service restoration')
            return response
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            response.success = requested_done
            response.message = (
                'mapping completion is already finalizing; wait for the '
                'session runner to restore functional services'
                if requested_done else
                'mapping cannot resume after return-home completion has '
                'entered finalization')
            return response
        if not requested_done:
            if (self._goal_transport_fault
                    or self._spin_transport_fault
                    or self._spin_phase != 'idle'
                    or self._stall_cancel_requested
                    or self._cancel_on_accept_context is not None):
                self.publish_zero_velocity()
                response.success = False
                response.message = (
                    'mapping cannot resume while a Nav2 action state is '
                    'uncertain; restart the Cartographer mapping session')
                return response
            response = super().set_mapping_done_callback(request, response)
            if response.success:
                self._reset_return_home_xy_completion()
                self._voice_pause_latched = False
            return response

        return_home_in_progress = (
            self.pending_home_goal
            or self.return_home_requested
            or self.navigation_state in (
                'canceling_late_exploration_goal',
                'mapping_done_canceling_active_goal',
                'mapping_done_canceling_spin',
            )
        )
        if self.mapping_done and return_home_in_progress:
            response.success = True
            response.message = (
                'mapping already marked done; return-home state: '
                f'{self.navigation_state}')
            return response

        self._reset_return_home_xy_completion()
        self._reset_final_map_save_state()
        self._voice_pause_latched = False
        was_recovering = self.is_recovering() or self.is_recovery_state()
        spin_was_active = self._spin_phase != 'idle'
        if not spin_was_active:
            self._stop_manager_recovery_motion()
        self.recovery_turn_direction = 0.0
        if was_recovering:
            self.get_logger().info(
                'mapping completion stopped Cartographer recovery motion '
                'before return-home processing')

        now = time.monotonic()
        # Keep lightweight unit-test harnesses and legacy direct callers safe:
        # production initialization always creates this field.
        if getattr(self, '_mapping_done_requested_at', 0.0) <= 0.0:
            self._mapping_done_requested_at = now
            self._mapping_done_requested_at_epoch_ms = int(time.time() * 1000)
            self._mapping_done_requested_at_iso = time.strftime(
                '%Y-%m-%dT%H:%M:%S%z')
            self._return_home_started_at = 0.0
            self._return_home_started_at_epoch_ms = 0
            self._return_home_started_at_iso = ''
            self._mapping_timeout_failure_kind = ''
            self._mapping_timeout_failure_reason = ''
            self._mapping_timeout_auto_cancel_at = 0.0
        self.mapping_done = True
        self.auto_explore = False
        self.pending_home_goal = True
        self.return_home_requested = False
        self.shutdown_after_return_home = True
        self.return_home_ready_after = (
            time.monotonic() + self.return_home_quiet_sec)
        self.return_home_tf_stable_since = 0.0
        self.return_home_retry_count = 0
        self.mode = 'returning_home'
        self.navigation_state = 'mapping_done_return_home_pending'
        self.last_error = ''
        self._clear_recovery_intent(clear_episode=True)
        self._kill_rrt_nodes()

        if spin_was_active:
            self._request_spin_cancel('mapping_done')
            if self._spin_phase == 'idle':
                _, message = self._dispatch_pending_home_goal()
                response.success = True
                response.message = message
                return response
            self.navigation_state = 'mapping_done_canceling_spin'
            response.success = True
            response.message = (
                'mapping marked done; waiting for exact Spin terminal result '
                'before returning home')
            return response

        if self.goal_send_future is not None:
            self._arm_cancel_on_accept('mapping_done')
            self.navigation_state = 'mapping_done_waiting_goal_acceptance'
            response.success = True
            response.message = (
                'mapping marked done; pending goal will be canceled '
                'immediately if Nav2 accepts it')
            return response

        if self.current_goal_handle is not None and self.nav_goal_active:
            if self._stall_cancel_requested:
                self._retag_monitored_cancel('mapping_done', False)
            else:
                self.last_error = 'mapping completion requested goal cancel'
                self._begin_monitored_cancel(
                    'mapping_done', False, time.monotonic())
            self.navigation_state = 'mapping_done_canceling_active_goal'
            response.success = True
            response.message = (
                'mapping marked done; waiting for exact active-goal '
                'cancellation before returning home')
            return response

        _, message = self._dispatch_pending_home_goal()
        response.success = True
        response.message = message
        return response

    def _return_home_xy_cancel_matches(
            self, future, completed_kind, completed_generation,
            monitored_cancel, cancel_context):
        return (
            monitored_cancel
            and cancel_context
            == self.RETURN_HOME_ORIENTATION_TIMEOUT_CONTEXT
            and completed_kind == 'home'
            and completed_generation is not None
            and completed_generation == self._active_goal_generation
            and completed_generation == self._return_home_xy_cancel_generation
            and self.current_goal_handle
            is self._return_home_xy_cancel_handle
            and self.goal_result_future is future
            and self._return_home_xy_cancel_result_future is future
            and self._return_home_xy_cancel_distance_m is not None
            and math.isfinite(self._return_home_xy_cancel_distance_m)
            and self._return_home_xy_cancel_distance_m
            <= self.return_home_xy_completion_tolerance_m
            and not self._cancel_timeout_reported
            and not self._goal_transport_fault
        )

    def _complete_return_home_by_position(self, future, distance_m):
        """Map this exact canceled home action to the normal success exit."""
        self.return_home_requested = False
        self.pending_home_goal = False
        self.nav_goal_active = False
        self.current_goal_handle = None
        self.last_safety_stop = 0.0
        self.last_cmd_vel = Twist()
        self.progress_x = None
        self.progress_y = None
        self.progress_yaw = None
        self.progress_time = 0.0
        self.stuck_recovery_cycles = 0
        if self.goal_result_future is future:
            self.goal_result_future = None
        self._active_goal_kind = None
        with self._feedback_state_lock:
            self._clear_feedback_tracking_locked(clear_generation=True)
            self.exploration_watchdog.clear()
        self._rrt_wait_started_at = None
        self._clear_recovery_intent(clear_episode=True)
        self._goal_transport_fault = False
        self._reset_return_home_xy_completion()
        # Keep the 20 Hz zero guard armed after the monitored-cancel state is
        # cleared and until the isolated launch actually exits.
        self._return_home_xy_finalizing = True
        self._reset_stall_cancel_tracking()
        self.mode = 'arrived'
        self.navigation_state = 'succeeded'
        self.last_error = ''
        self.publish_zero_velocity()
        self.get_logger().info(
            'return-home completed by position after orientation grace: '
            f'distance={distance_m:.3f}m; stopping the mapping session and '
            'returning control to the operator after service restoration')
        self.request_launch_shutdown(
            'return-home position reached after orientation grace expired')

    def _handle_goal_result_error(self, future, completed_kind, exc):
        self._reset_return_home_xy_completion()
        handle = self.current_goal_handle
        self._stop_manager_recovery_motion()
        self.return_home_requested = False
        if self.goal_result_future is future:
            self.goal_result_future = None
        with self._feedback_state_lock:
            self._clear_feedback_tracking_locked(clear_generation=True)
            self.exploration_watchdog.clear()
        self._rrt_wait_started_at = None
        self._clear_recovery_intent(clear_episode=True)
        self._goal_transport_fault = True
        self.auto_explore = False
        self.mode = 'safety_stop'
        self.navigation_state = 'goal_result_error'
        self.last_error = (
            f'navigation result failed for {completed_kind or "unknown"}: '
            f'{exc}; exact goal cancellation requested and recovery '
            'suppressed')
        self.get_logger().error(self.last_error)

        if handle is None:
            self.nav_goal_active = False
            if self._active_goal_kind == completed_kind:
                self._active_goal_kind = None
            self._reset_stall_cancel_tracking()
            return

        self.nav_goal_active = True
        if self._stall_cancel_requested:
            self._recovery_after_cancel = False
            self._cancel_context = 'goal_result_error'
        else:
            self._begin_monitored_cancel(
                'goal_result_error', False, time.monotonic())
        self.navigation_state = 'goal_result_error_canceling'
        if not self._result_retry_attempted:
            self._result_retry_attempted = True
            try:
                self.goal_result_future = handle.get_result_async()
                self.goal_result_future.add_done_callback(
                    self.goal_result_callback)
            except Exception as retry_exc:  # noqa: BLE001
                self.goal_result_future = None
                self.get_logger().error(
                    f'could not reattach navigation result monitoring: '
                    f'{retry_exc}')

    def goal_result_callback(self, future):
        if future is not self.goal_result_future:
            self.get_logger().warn(
                'ignored a stale navigation result Future that no longer '
                'owns the active goal')
            return
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self._handle_goal_result_error(
                future, self._active_goal_kind, exc)
            return
        completed_kind = self._active_goal_kind
        completed_generation = self._active_goal_generation
        monitored_cancel = self._stall_cancel_requested
        cancel_context = self._cancel_context if monitored_cancel else None
        transport_fault = self._goal_transport_fault
        fault_error = self.last_error if transport_fault else ''
        fault_state = self.navigation_state if transport_fault else ''
        recover_after_cancel = (
            self._recovery_after_cancel
            and self._is_exploration_goal(completed_kind)
            and status == GoalStatus.STATUS_CANCELED
        )
        exact_position_cancel = self._return_home_xy_cancel_matches(
            future,
            completed_kind,
            completed_generation,
            monitored_cancel,
            cancel_context,
        )
        if (status == GoalStatus.STATUS_CANCELED
                and exact_position_cancel):
            self._complete_return_home_by_position(
                future, self._return_home_xy_cancel_distance_m)
            return
        if self.mapping_done and self.pending_home_goal:
            # Any terminal result proves that the old goal no longer owns the
            # controller.  Clean it up without invoking the base failure
            # recovery, then begin the explicit return-home flow.
            self.return_home_requested = False
            self.nav_goal_active = False
            self.current_goal_handle = None
            self.last_safety_stop = 0.0
            self.last_cmd_vel = Twist()
            self.progress_x = None
            self.progress_y = None
            self.progress_yaw = None
            self.progress_time = 0.0
            if self.goal_result_future is future:
                self.goal_result_future = None
            self._active_goal_kind = None
            with self._feedback_state_lock:
                self._clear_feedback_tracking_locked(clear_generation=True)
                self.exploration_watchdog.clear()
            self._reset_stall_cancel_tracking()
            self._rrt_wait_started_at = None
            self._clear_recovery_intent(clear_episode=True)
            self._goal_transport_fault = False
            self.publish_zero_velocity()
            started, message = self._dispatch_pending_home_goal()
            if started:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
            return

        try:
            super().goal_result_callback(future)
        except Exception as exc:  # noqa: BLE001
            self._handle_goal_result_error(future, completed_kind, exc)
            return

        self._reset_return_home_xy_completion()

        if transport_fault:
            self._stop_manager_recovery_motion()
            self.mode = 'safety_stop'
            self.navigation_state = fault_state
            self.last_error = fault_error
        elif cancel_context == 'stop_navigation':
            self._stop_manager_recovery_motion()
            self.mode = 'stopped'
            self.navigation_state = 'canceled'
            self.last_error = ''
        elif cancel_context == 'base_charging':
            self._stop_manager_recovery_motion()
            self.mode = 'paused'
            self.navigation_state = 'base_charging'
            self.last_error = 'base is charging; navigation paused'

        if self._is_exploration_goal(completed_kind):
            self._rrt_wait_started_at = None
        if self._active_goal_kind == completed_kind:
            self._active_goal_kind = None
        if self.goal_result_future is future:
            self.goal_result_future = None
        with self._feedback_state_lock:
            self._clear_feedback_tracking_locked(clear_generation=True)
            self.exploration_watchdog.clear()
        self._reset_stall_cancel_tracking()

        if not recover_after_cancel:
            # A successful, aborted, manually canceled, stop, or charging
            # terminal result must never leave an old precheck/stall intent
            # available to a later safety-timer tick.
            self._clear_recovery_intent(clear_episode=True)

        safe_to_recover = (
            recover_after_cancel
            and self._recovery_intent == 'stalled_cancel'
            and not self.mapping_done
            and self.auto_explore
            and not self.base_charging
            and not self.pending_home_goal
            and not self.return_home_requested
            and self.goal_send_future is None
            and not self.nav_goal_active
            and not self._goal_transport_fault
            and not self._spin_transport_fault
        )
        if not safe_to_recover:
            self._clear_recovery_intent(clear_episode=True)
            return

        now = time.monotonic()
        if self._blocked_episode_started_at is None:
            self._blocked_episode_started_at = now
        self._blocked_episode_attempted = True
        self._next_blocked_recovery_at = (
            now + self.blocked_recovery_cooldown_sec)
        self._recovery_intent = None
        self.mode = 'safety_stop'
        self.navigation_state = 'starting_stall_recovery'
        self._start_stall_spin_recovery('stalled_cancel')
        if self.is_recovering():
            self.get_logger().warn(
                'stalled exploration goal cancel acknowledged; '
                f'controlled recovery started in state '
                f'{self.navigation_state}')
        else:
            self.get_logger().error(
                'stalled exploration goal cancel acknowledged, but controlled '
                f'recovery did not start; state={self.navigation_state}')

    def last_map_resolution_floor(self):
        """当前地图分辨率（无地图时退化为 0.05m，仅用于步长下限）。"""
        try:
            resolution = float(self.last_map.info.resolution)
        except Exception:
            return 0.05
        if resolution <= 0.0:
            return 0.05
        return resolution

    def _project_rrt_frontier_goal(self, goal_pose):
        """Project an RRT boundary sample onto a safe known-free map cell."""
        if self.last_map is None:
            return None, False, 'RRT frontier rejected: map is unavailable'
        info = self.last_map.info
        resolution = info.resolution
        if resolution <= 0.0:
            return None, False, 'RRT frontier rejected: map resolution is invalid'

        origin = info.origin.position
        target_x = goal_pose.pose.position.x
        target_y = goal_pose.pose.position.y
        target_mx = int((target_x - origin.x) / resolution)
        target_my = int((target_y - origin.y) / resolution)
        radius_cells = max(
            1, int(math.ceil(self.rrt_projection_radius_m / resolution)))
        current = self.update_pose()
        if current is not None:
            current_x = current.pose.position.x
            current_y = current.pose.position.y
        else:
            current_x = None
            current_y = None
        # 2026-10-13 P27（方案A）：本路径原先**完全没有**转角项，
        # 而实测那个 turn=99° 的点正来自这里。下面把它折算进
        # target_distance，使小转角候选自然排到前面。
        _turn_active = self._turn_penalty_on() and self._turn_penalty_weight() > 0.0

        candidates = []
        # P31：本函数内读一次（不是每个格读），热调自下一拍生效。
        _free_cost = self._sampler_free_cost()
        min_x = max(0, target_mx - radius_cells)
        max_x = min(info.width - 1, target_mx + radius_cells)
        min_y = max(0, target_my - radius_cells)
        max_y = min(info.height - 1, target_my + radius_cells)
        for my in range(min_y, max_y + 1):
            for mx in range(min_x, max_x + 1):
                x = origin.x + (mx + 0.5) * resolution
                y = origin.y + (my + 0.5) * resolution
                target_distance = math.hypot(x - target_x, y - target_y)
                if target_distance > self.rrt_projection_radius_m:
                    continue
                value = self.last_map.data[my * info.width + mx]
                if value < 0 or value >= _free_cost:
                    continue
                unknown_neighbors = 0
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        nx = mx + dx
                        ny = my + dy
                        if (0 <= nx < info.width
                                and 0 <= ny < info.height
                                and self.last_map.data[
                                    ny * info.width + nx] < 0):
                            unknown_neighbors += 1
                _rank_distance = target_distance
                if _turn_active:
                    _rank_distance += self._turn_penalty_m(current, x, y)
                candidates.append((
                    _rank_distance,
                    -unknown_neighbors,
                    x,
                    y,
                    mx,
                    my,
                ))

        candidates.sort()

        reject = {'bounds': 0, 'unknown_or_cost': 0, 'recent_failed': 0,
                  'clearance': 0, 'path_blocked': 0, 'ok': 0}

        def acceptable(mx, my, x, y):
            """Same safety standard as before: known-free, clearance, path ratio.

            2026-10-06: 仅增加否决原因计数（reject），判定结果与原先完全一致。
            """
            if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                reject['bounds'] += 1
                return False
            value = self.last_map.data[my * info.width + mx]
            if value < 0 or value >= _free_cost:
                reject['unknown_or_cost'] += 1
                return False
            if self.goal_recently_failed(x, y):
                reject['recent_failed'] += 1
                return False
            if not self._fallback_cell_has_clearance(mx, my):
                reject['clearance'] += 1
                return False
            if (current_x is not None
                    and self.path_blocked_ratio(
                        current_x, current_y, x, y) >= 0.10):
                reject['path_blocked'] += 1
                return False
            reject['ok'] += 1
            return True

        # 2026-10-06 (改动 1): 优先沿“前沿样本 -> 小车”连线向已知侧退让，
        # 取第一个满足安全标准的格。这样既能贴上可达前沿，又不会像把半径
        # 放大到 2.0m 那样“投进已知空间深处”。
        selected = None
        backoff_max = self.rrt_projection_radius_m
        backoff_min = min(self.rrt_projection_backoff_step_m * 4.0, backoff_max)
        if current_x is not None:
            direction_x = current_x - target_x
            direction_y = current_y - target_y
            norm = math.hypot(direction_x, direction_y)
        else:
            norm = 0.0
        if current_x is not None and norm > resolution:
            ux = direction_x / norm
            uy = direction_y / norm
            step = self.rrt_projection_backoff_step_m
            outer = min(backoff_max, norm)
            distance = min(backoff_min, outer)
            while distance <= outer + step * 0.5:
                px = target_x + ux * distance
                py = target_y + uy * distance
                mx = int((px - origin.x) / resolution)
                my = int((py - origin.y) / resolution)
                if acceptable(mx, my, px, py):
                    selected = (px, py)
                    break
                distance += step

        # 兜底：保留原有“圆内最近已知空闲格”搜索，不降低修复前的既有能力。
        if selected is None:
            # 2026-10-14 P46-C：只对排序后的前 N 个候选逐个投影。
            # 实测单轮 total=999 全部逐个投影，耗时足以饿死同 executor 上的
            # lifecycle 回调（见 P46-B 的现场），进而把车停成 228s。
            # candidates 已按 (rank_distance, -unknown_neighbors) 排序，
            # 取前 N 个不损失候选质量。置 0 或负数表示不限制（回到旧行为）。
            _limit = int(getattr(self, 'rrt_projection_max_candidates', 200))
            _pool = candidates if _limit <= 0 else candidates[:_limit]
            for _distance, _unknown_score, x, y, mx, my in _pool:
                if acceptable(mx, my, x, y):
                    selected = (x, y)
                    break

        if selected is None:
            # 2026-10-06 诊断：把各判据的否决计数带出来，便于定位瓶颈。
            self.get_logger().warn(
                'projection_reject_stats: total=%d unknown_or_cost=%d '
                'clearance=%d path_blocked=%d recent_failed=%d bounds=%d '
                'candidates=%d radius=%.2f goal_clearance=%.2f '
                'robot_known=%s'
                % (reject['unknown_or_cost'] + reject['clearance']
                   + reject['path_blocked'] + reject['recent_failed']
                   + reject['bounds'],
                   reject['unknown_or_cost'], reject['clearance'],
                   reject['path_blocked'], reject['recent_failed'],
                   reject['bounds'], len(candidates),
                   self.rrt_projection_radius_m, self.goal_clearance,
                   'yes' if current_x is not None else 'no'))
            return (
                None,
                False,
                'RRT frontier rejected: no safe known-free projection within '
                f'{self.rrt_projection_radius_m:.2f}m',
            )
        x, y = selected
        projected = math.hypot(x - target_x, y - target_y) > resolution * 0.51
        # 2026-10-06 (改动 1): 目标朝向统一取“朝目标点方向”，避免旧口径
        # （projected 时朝前沿、否则朝候选点）在相邻两次投影间给出相差 180°
        # 的 yaw，导致控制器原地来回对转而表现为高频自转。
        if current_x is not None:
            yaw = math.atan2(y - current_y, x - current_x)
        else:
            yaw = self.pose_yaw(goal_pose)
        projected_goal = self.make_pose_goal(x, y, yaw)
        projected_goal.header.frame_id = (
            goal_pose.header.frame_id or self.global_frame)
        return projected_goal, projected, ''

    def frontier_goal_callback(self, msg):
        # Do not run map validation or blacklist a candidate until Nav2 can
        # actually accept it.  Also close the action-acceptance window in which
        # nav_goal_active is still false but a goal request is already pending.
        if (not self.auto_explore or self.mapping_done
                or self._goal_transport_fault
                or self._spin_transport_fault
                or self._stall_cancel_requested
                or self._spin_phase != 'idle'
                or self.nav_goal_active
                or self.goal_send_future is not None):
            return
        if not self.nav2_is_ready():
            self.report_nav2_not_ready()
            return
        if self.front_blocked:
            now = time.monotonic()
            self.remember_blocked_place()
            self.publish_zero_velocity()
            self.mode = 'safety_stop'
            measured = (
                f'{self.front_min_range:.2f}m'
                if self.front_min_range is not None else 'unavailable')
            self.last_error = f'front obstacle too close: {measured}'
            armed = self._arm_exploration_precheck_recovery(now)
            if not armed:
                self.navigation_state = 'blocked_waiting_clearance'
            self._log_recovery_event(
                'goal_precheck_blocked:exploration_rrt', self.last_error)
            return
        candidate, projected, projection_error = (
            self._project_rrt_frontier_goal(msg))
        if candidate is None:
            self._log_recovery_event(
                'rrt_frontier_projection_failed', projection_error)
            return
        if projected:
            self.get_logger().info(
                'projected RRT boundary sample from '
                f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) to '
                f'known-free ({candidate.pose.position.x:.2f}, '
                f'{candidate.pose.position.y:.2f})')

        previous_future = self.goal_send_future
        self._run_with_goal_kind(
            'exploration_rrt', super().frontier_goal_callback, candidate)
        rrt_dispatched = (
            (self.goal_send_future is not None
             and self.goal_send_future is not previous_future)
            or (self.nav_goal_active
                and self._active_goal_kind == 'exploration_rrt')
        )
        if rrt_dispatched:
            self._rrt_wait_started_at = None
            self.get_logger().info(
                'accepted RRT frontier as the active exploration source')



    def _reset_mapping_exit_timeout(self):
        self._reset_return_home_xy_completion()
        self._mapping_done_requested_at = 0.0
        self._mapping_done_requested_at_epoch_ms = 0
        self._mapping_done_requested_at_iso = ''
        self._return_home_started_at = 0.0
        self._return_home_started_at_epoch_ms = 0
        self._return_home_started_at_iso = ''
        self._mapping_timeout_failure_kind = ''
        self._mapping_timeout_failure_reason = ''
        self._mapping_timeout_auto_cancel_at = 0.0

    def _report_mapping_exit_timeout(self, kind, reason):
        """Expose a failure briefly, then autonomously cancel safely."""
        if self.shutdown_started or self._return_home_xy_finalizing:
            self.publish_zero_velocity()
            return
        if self._mapping_timeout_failure_kind or self._mapping_cancel_requested:
            return
        self._reset_return_home_xy_completion()
        self._mapping_timeout_failure_kind = str(kind)
        self._mapping_timeout_failure_reason = str(reason)
        self.pending_home_goal = False
        self.return_home_requested = False
        self.auto_explore = False
        self._clear_recovery_intent(clear_episode=True)
        if self.current_goal_handle is not None and self.nav_goal_active:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - shutdown still proceeds
                self.get_logger().warn(
                    'best-effort return-home goal cancellation failed: '
                    f'{exc}')
        self.publish_zero_velocity()
        self.mode = 'stopped'
        self.navigation_state = 'return_home_failed_waiting_manual'
        self.last_error = self._mapping_timeout_failure_reason
        self._mapping_timeout_auto_cancel_at = (
            time.monotonic() + self.mapping_timeout_cancel_delay_sec)
        self.get_logger().error(
            f'mapping exit timeout ({kind}): {reason}; controlled '
            f'cancellation will start in '
            f'{self.mapping_timeout_cancel_delay_sec:.1f}s')
        self._write_session_status()

    def _return_home_xy_defers_hard_timeout(self, now):
        if (
                self._return_home_xy_cancel_handle is not None
                and self._stall_cancel_requested
                and self._cancel_context
                == self.RETURN_HOME_ORIENTATION_TIMEOUT_CONTEXT
                and not self._cancel_timeout_reported):
            return True
        if (
                self._return_home_xy_grace_started_at is None
                or self._return_home_xy_grace_generation is None
                or self._return_home_xy_grace_generation
                != self._active_goal_generation):
            return False
        if (self.goal_result_future is not None
                and self.goal_result_future.done()):
            return True
        return (
            now - self._return_home_xy_grace_started_at
            < self.return_home_orientation_grace_sec)

    def _maybe_enforce_mapping_exit_timeout(self):
        if (
                not self.mapping_done or self.shutdown_started or
                self._mapping_cancel_requested or
                getattr(self, '_mapping_timeout_failure_kind', '')):
            return
        now = time.monotonic()
        # A dispatched/active home goal marks the second, navigation stage.
        if self.return_home_requested or self.nav_goal_active:
            if self._return_home_started_at <= 0.0:
                self._return_home_started_at = now
                self._return_home_started_at_epoch_ms = int(time.time() * 1000)
                self._return_home_started_at_iso = time.strftime(
                    '%Y-%m-%dT%H:%M:%S%z')
                self.get_logger().info(
                    'return-home hard timeout armed: '
                    f'{self.return_home_hard_timeout_sec:.1f}s')
            if (now - self._return_home_started_at
                    >= self.return_home_hard_timeout_sec):
                result_future = self.goal_result_future
                if result_future is not None and result_future.done():
                    # As with cancel monitoring, Humble may run this timer
                    # before a done callback queued from the same executor
                    # batch. The exact current terminal result always owns the
                    # outcome ahead of the wall-clock hard timeout.
                    self.get_logger().warn(
                        'return-home result became ready at the hard timeout '
                        'boundary; consuming the exact terminal result before '
                        'declaring a mapping exit timeout')
                    self.goal_result_callback(result_future)
                    return
                # A robot that first reaches the XY tolerance at the end of
                # the ordinary return-home budget still receives the promised
                # six-second exact-orientation opportunity. Once the grace is
                # over, being outside XY does not mask the original hard
                # timeout; an in-range goal starts the bounded exact cancel.
                self._maybe_complete_return_home_by_position(now)
                if self._return_home_xy_defers_hard_timeout(now):
                    return
                self._report_mapping_exit_timeout(
                    'return_home_timeout',
                    'return-home navigation exceeded %.1fs' %
                    self.return_home_hard_timeout_sec)
            return
        if (
                self._mapping_done_requested_at > 0.0 and
                now - self._mapping_done_requested_at
                >= self.mapping_finish_confirm_timeout_sec):
            self._report_mapping_exit_timeout(
                'finish_mapping_timeout',
                'mapping finalization did not dispatch return-home within '
                '%.1fs' % self.mapping_finish_confirm_timeout_sec)

    def _maybe_request_mapping_timeout_cancel(self):
        if (
                not self._mapping_timeout_failure_kind or
                self._mapping_cancel_requested or
                self.shutdown_started or
                self._mapping_timeout_auto_cancel_at <= 0.0 or
                time.monotonic() < self._mapping_timeout_auto_cancel_at):
            return
        self._mapping_timeout_auto_cancel_at = 0.0
        self._request_mapping_cancellation(
            self._mapping_timeout_failure_kind,
            self._mapping_timeout_failure_reason)

    def _maybe_auto_close_manual_takeover(self, pose):
        if (
                not self.manual_takeover_auto_close_enabled
                or self._mapping_timeout_failure_kind
                or self.shutdown_started
                or not self.mapping_done
                or self.navigation_state != 'return_home_failed_waiting_manual'
                or self.mode != 'stopped'
                or self.nav_goal_active
                or self.goal_send_future is not None
                or self.pending_home_goal
                or self.return_home_requested
                or self.is_recovering()
                or self._spin_phase != 'idle'):
            self._manual_recovery_stable_since = 0.0
            return

        if pose is None:
            self._manual_recovery_stable_since = 0.0
            return

        position_error_m = math.hypot(
            pose.pose.position.x, pose.pose.position.y)
        if position_error_m > self.manual_takeover_auto_close_distance_m:
            self._manual_recovery_stable_since = 0.0
            return

        now = time.monotonic()
        if self._manual_recovery_stable_since <= 0.0:
            self._manual_recovery_stable_since = now
            self.get_logger().info(
                'manual recovery is near the start pose and stationary; '
                f'waiting {self.manual_takeover_auto_close_stable_sec:.1f}s '
                'before closing the mapping session')
            return

        stable_for = now - self._manual_recovery_stable_since
        if stable_for < self.manual_takeover_auto_close_stable_sec:
            return

        self.get_logger().info(
            'manual recovery confirmed by stable near-start pose '
            f'({position_error_m:.3f}m for {stable_for:.1f}s); '
            'closing Cartographer session')
        self.request_launch_shutdown(
            'manual recovery confirmed by vehicle state')

    def _maybe_request_mapping_cancel_shutdown(self):
        if (
                not self._mapping_cancel_shutdown_pending or
                self.shutdown_started or
                time.monotonic() < self._mapping_cancel_shutdown_at):
            return
        self._mapping_cancel_shutdown_pending = False
        self.get_logger().warn(
            'operator cancelled Cartographer after return-home failure; '
            'shutting down the isolated session')
        self.request_launch_shutdown(
            'operator cancelled mapping after return-home failure')

    def _before_launch_shutdown(self):
        # Persist a non-terminal transition before SIGINT.  The outer session
        # runner owns the final closed state because it also restores control
        # services after the inner Cartographer launch has exited.
        if self.mapping_done:
            self._write_session_status()

    def _publish_state_impl(self):
        # Publish a complete PAUSED nav-state while the goal
        # cancellation machinery runs, without changing the manager's
        # internal safety/recovery mode used by its callbacks.
        published_mode = None
        if (self._voice_pause_latched and not self.mapping_done
                and not self._mapping_cancel_requested):
            published_mode = self.mode
            self.mode = 'paused'
        try:
            super()._publish_state_impl()
        finally:
            if published_mode is not None:
                self.mode = published_mode
        pose = self.last_pose or self.update_pose()
        self._maybe_enforce_mapping_exit_timeout()
        self._maybe_auto_close_manual_takeover(pose)
        self._maybe_request_mapping_timeout_cancel()
        self._maybe_request_mapping_cancel_shutdown()
        self._write_session_status()

    def _read_session_task_id(self):
        expected_task_id = os.getenv(
            'STEMM_CARTOGRAPHER_EXPECTED_TASK_ID', ''
        ).strip()
        if expected_task_id:
            # A bridge restart or duplicate MQTT delivery may rewrite the shared
            # task file. The running session must keep the immutable identity
            # supplied by its outer session runner.
            return expected_task_id
        try:
            with open(self.session_task_path, 'r', encoding='utf-8') as handle:
                task_id = json.load(handle).get('taskId', '')
            return task_id if isinstance(task_id, str) else ''
        except (OSError, ValueError, json.JSONDecodeError):
            return ''

    def _write_session_status(self):
        pose = self.update_pose()
        position_error_m = None
        if pose is not None:
            position_error_m = math.hypot(pose.pose.position.x, pose.pose.position.y)
        state = str(self.navigation_state or '')
        failed_waiting = (
            state == 'return_home_failed_waiting_manual' and
            not self._mapping_cancel_requested)
        timeout_failure = bool(
            getattr(self, '_mapping_timeout_failure_kind', ''))
        finalizing = self.shutdown_started and self.mapping_done
        cancel_requested = bool(self._mapping_cancel_requested)
        if finalizing:
            phase = 'finalizing'
        elif cancel_requested:
            phase = 'canceling'
        elif failed_waiting:
            phase = 'return_home_failed'
        else:
            phase = 'returning_home' if self.mapping_done else 'mapping'
        termination_status = (
            'canceled' if cancel_requested else
            ('succeeded' if finalizing else '')
        )
        termination_reason = (
            self._mapping_cancel_reason if cancel_requested else
            ('return_home_complete' if finalizing else '')
        )
        payload = {
            'schemaVersion': 2,
            'taskId': self._read_session_task_id(),
            'updatedAt': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'updatedAtEpochMs': int(time.time() * 1000),
            'phase': phase,
            # Keep both fields during protocol migration. The mini-program
            # consumes status, while existing bridge consumers read taskStatus.
            'status': 'running',
            'taskStatus': 'running',
            'terminal': False,
            'sessionActive': True,
            'processExited': False,
            'functionalServicesRestored': False,
            'cancelRequested': cancel_requested,
            'terminationStatus': termination_status,
            'terminationReason': termination_reason,
            'returnHomeFailureReason': (
                self._mapping_timeout_failure_reason if timeout_failure else
                (self._mapping_cancel_detail if cancel_requested else
                 (str(self.last_error or '') if failed_waiting else ''))
            ),
            # A failure is published only after retry exhaustion or a bounded
            # end-mapping/return-home timeout, never for one transient abort.
            'returnHomeFailureConfirmed': failed_waiting or timeout_failure,
            'returnHomeFailureKind': (
                self._mapping_timeout_failure_kind if timeout_failure else
                ('retry_exhausted' if failed_waiting else '')
            ),
            # Wall-clock deadline origins let an online client display a
            # countdown from the car's authoritative timer.  The monotonic
            # counterparts remain the sole source for enforcement.
            'finishRequestedAt': getattr(
                self, '_mapping_done_requested_at_iso', ''),
            'finishRequestedAtEpochMs': int(getattr(
                self, '_mapping_done_requested_at_epoch_ms', 0) or 0),
            'returnHomeStartedAt': getattr(
                self, '_return_home_started_at_iso', ''),
            'returnHomeStartedAtEpochMs': int(getattr(
                self, '_return_home_started_at_epoch_ms', 0) or 0),
            'manualTakeoverRequired': failed_waiting,
            'mappingDone': bool(self.mapping_done),
            'navigationState': state,
            'mode': str(self.mode or ''),
            'voicePaused': bool(self._voice_pause_latched),
            'resumeAcceptedAtEpochMs': int(
                getattr(self, '_mapping_resume_accepted_at_epoch_ms', 0) or 0
            ),
            'returnHomeRetry': int(self.return_home_retry_count),
            'returnHomeMaxRetries': int(self.return_home_max_retries),
            'pendingHomeGoal': bool(self.pending_home_goal),
            'returnHomeRequested': bool(self.return_home_requested),
            'finalMapSaved': bool(self._final_map_saved),
            'finalMapSavedAt': self._final_map_saved_at,
            'finalMapSavedFallback': bool(
                getattr(self, '_final_map_saved_fallback', False)),
            'finalMapSaveError': self._final_map_save_error,
            'positionErrorMeters': position_error_m,
            'pose': self.pose_to_dict(pose),
            'lastError': str(self.last_error or ''),
        }
        try:
            os.makedirs(os.path.dirname(self.session_status_path), exist_ok=True)
            tmp = self.session_status_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, ensure_ascii=False)
            os.replace(tmp, self.session_status_path)
            self._session_status_write_error = ''
        except OSError as exc:
            message = f'cannot write Cartographer session status: {exc}'
            if message != self._session_status_write_error:
                self.get_logger().error(message)
            self._session_status_write_error = message


def main(args=None):
    rclpy.init(args=args)
    node = StemmCartographerNav2Manager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:               # 2026-10-09 (operator): rclpy shutdown race
        # The operator-cancel path (_maybe_request_mapping_cancel_shutdown ->
        # request_launch_shutdown) tears the rclpy context down while the executor is BETWEEN
        # iterations, so the next wait set cannot be built.  Measured 2026-10-09 16:06:59, 16 s
        # after 'return-home failed ... keeping launch alive for manual takeover':
        #   RCLError: failed to initialize wait set: the given context is not valid ...
        # That exception is not covered by the clause above, so the process died with exit 1,
        # which (a) voided the manual-takeover promise, (b) made the session runner mark the
        # session terminal and trigger a whole-device restart, and (c) skipped the final
        # publish_zero_velocity() because rclpy.ok() was already False.  Whether the SIGINT lands
        # inside rcl_wait (clean) or between iterations (this) is a race: 8 of the 9 recorded
        # nav2_manager deaths were clean -2; this one was the first 1.
        # Swallow ONLY this signature -- anything else must still propagate.
        if 'wait set' in str(exc) and 'not valid' in str(exc):
            print('shutdown race: rclpy context was torn down mid-spin; exiting cleanly '
                  'instead of dying with exit 1 (%s)' % exc, flush=True)
        else:
            raise
    finally:
        # 2026-10-13: teardown must not turn a successful session into exit 1.
        # rclpy.ok() only covers the global context; node/publisher-level
        # contexts can already be invalid, and destroy_node()/shutdown() have
        # their own races (see the 2026-10-09 'wait set not valid' note above).
        # Measured 2026-10-13 17:32: a real return-home (0.243 m) was reported
        # as a failed session purely because of this race.
        try:
            if rclpy.ok():
                node.publish_zero_velocity()
        except Exception as exc:  # noqa: BLE001 - teardown race
            print('shutdown race: zero-velocity publish failed during '
                  'teardown; ignoring (%s)' % exc, flush=True)
        try:
            node.destroy_node()
        except Exception as exc:  # noqa: BLE001 - teardown race
            print('shutdown race: destroy_node failed during teardown; '
                  'ignoring (%s)' % exc, flush=True)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001 - teardown race
            print('shutdown race: rclpy.shutdown failed during teardown; '
                  'ignoring (%s)' % exc, flush=True)


if __name__ == '__main__':
    main()
