#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""frontier_explorer.py — 栅格式前沿点探索节点

输出契约（与现有下游完全一致，零改动）：
    /assigned_frontier_goal   geometry_msgs/PoseStamped
    manager.py:174  create_subscription(PoseStamped, self.frontier_goal_topic,
                                        self.frontier_goal_callback, 10)
    manager.py:205  'frontier_goal_topic': '/assigned_frontier_goal'

下游仍然负责：R1d 前进净空预检、_project_rrt_frontier_goal 安全投影、
goal_recently_failed 去重、min_explore_distance 过滤。本节点只替换
"候选点从哪来"这一环。

显式上报"没有前沿点"（为后续"探索已收敛"终态做基础）：
    /stemm/frontier_status   std_msgs/String   JSON
      {state, goal, clusters, reason, pose, atEpochMs}
    state: 'waiting_map' | 'waiting_nav' | 'no_frontier' | 'goal_sent'

2026-10-10 新建。回滚：删除本文件。
"""
import json
import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .selector import GridMap, Pose2D, SelectionStats, select_coverage_goal


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        # ---- 输入 ----
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('nav_state_topic', '/stemm_cartographer/nav_state')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        # 2026-10-10: 拒绝非本栈发布的 costmap/map。根命名空间的
        # /global_costmap/costmap 在两套 Nav2 栈并存时可能被跟随栈占用，
        # 收到错误数据会静默污染可达性判断。留空 = 不校验。
        self.declare_parameter('require_global_frame_match', True)
        # 2026-10-10: nav_is_busy 的判定词表。只应包含**非探索态**；
        # 不要把 executing / sending_goal 放进来——那会把本节点饿死。
        self.declare_parameter('blocked_state_tokens', 'return_home,returning_home,mapping_done,saving_final_map,final_map,canceling,cancel_requested,blocked_,waiting_clearance,stuck_,spin_,clearing_,escaping,warming_up,charging,goal_send_error,goal_response_error,goal_result_error,goal_acceptance_timeout,goal_rejected,goal_unreachable,shutdown,timeout')
        # 2026-10-10: nav_state 是 JSON 字符串，必须先解析再匹配状态字段值，
        # 否则会命中 JSON 键名造成误报。
        self.declare_parameter('nav_state_is_json', True)
        self.declare_parameter('nav_state_fields', 'navigation_state,mode')
        self.declare_parameter('map_timeout_sec', 10.0)
        self.declare_parameter('costmap_timeout_sec', 3.0)

        # ---- 输出 ----
        self.declare_parameter('frontier_goal_topic', '/assigned_frontier_goal')
        self.declare_parameter('status_topic', '/stemm/frontier_status')

        # ---- 节奏 ----
        self.declare_parameter('goal_interval_sec', 3.0)
        self.declare_parameter('no_goal_log_interval_sec', 5.0)
        # 2026-10-10: 运行时可追溯。tick() 的早退分支原本全是静默 return，
        # 导致"节点活着但零产出"时无法定位原因。
        self.declare_parameter('trace_enabled', True)
        self.declare_parameter('trace_interval_sec', 5.0)

        # ---- 几何 ----
        self.declare_parameter('min_goal_distance', 0.6)
        self.declare_parameter('max_goal_distance', 8.0)
        self.declare_parameter('obstacle_clearance', 0.38)
        self.declare_parameter('known_clearance', 0.20)
        self.declare_parameter('corridor_clearance', 0.28)
        self.declare_parameter('frontier_radius', 0.7)
        self.declare_parameter('min_frontier_cells', 8)

        # ---- 打分 ----
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('potential_scale', 1.2)
        self.declare_parameter('clearance_scale', 0.8)
        self.declare_parameter('lookahead_scale', 0.35)

        # ---- 失败点记忆 ----
        self.declare_parameter('bad_goal_radius', 0.55)
        self.declare_parameter('blocked_place_radius', 0.55)
        self.declare_parameter('bad_ttl_sec', 240.0)

        # ---- 成本护栏 ----
        self.declare_parameter('search_window_radius_m', 8.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.global_frame = str(p('global_frame'))
        self.base_frame = str(p('base_frame'))
        self.require_global_frame_match = bool(
            p('require_global_frame_match'))
        self.blocked_state_tokens = tuple(
            t.strip().lower()
            for t in str(p('blocked_state_tokens')).split(',')
            if t.strip())
        self.nav_state_is_json = bool(p('nav_state_is_json'))
        self.nav_state_fields = tuple(
            f.strip() for f in str(p('nav_state_fields')).split(',')
            if f.strip())
        if not self.blocked_state_tokens:
            self.get_logger().warn(
                'blocked_state_tokens is empty; nav_is_busy() will never '
                'block and the node may publish goals during return-home')
        self.map_timeout_sec = float(p('map_timeout_sec'))
        self.costmap_timeout_sec = float(p('costmap_timeout_sec'))
        self.goal_interval_sec = max(0.0, float(p('goal_interval_sec')))
        self.no_goal_log_interval_sec = max(
            0.5, float(p('no_goal_log_interval_sec')))
        self.min_goal_distance = max(0.0, float(p('min_goal_distance')))
        self.max_goal_distance = max(
            self.min_goal_distance, float(p('max_goal_distance')))
        self.obstacle_clearance = max(0.0, float(p('obstacle_clearance')))
        self.known_clearance = max(0.0, float(p('known_clearance')))
        self.corridor_clearance = max(0.0, float(p('corridor_clearance')))
        self.frontier_radius = max(0.01, float(p('frontier_radius')))
        self.min_frontier_cells = max(1, int(p('min_frontier_cells')))
        self.gain_scale = float(p('gain_scale'))
        self.potential_scale = float(p('potential_scale'))
        self.clearance_scale = float(p('clearance_scale'))
        self.lookahead_scale = float(p('lookahead_scale'))
        self.bad_goal_radius = max(0.0, float(p('bad_goal_radius')))
        self.blocked_place_radius = max(0.0, float(p('blocked_place_radius')))
        self.bad_ttl_sec = max(0.0, float(p('bad_ttl_sec')))
        self.search_window_radius_m = max(
            0.0, float(p('search_window_radius_m')))

        self.goal_pub = self.create_publisher(
            PoseStamped, str(p('frontier_goal_topic')), 10)
        self.status_pub = self.create_publisher(String, str(p('status_topic')), 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_map: Optional[OccupancyGrid] = None
        self.last_map_at = 0.0
        self.global_costmap: Optional[OccupancyGrid] = None
        self.global_costmap_at = 0.0
        self.local_costmap: Optional[OccupancyGrid] = None
        self.local_costmap_at = 0.0
        self.nav_state = ''
        self.nav_busy = False
        self.bad_places: List[Tuple[float, float, float]] = []
        self.last_goal_at = 0.0
        self.last_no_goal_log_at = 0.0
        self.last_status_state = ''
        self._last_published_trace_total = 0
        self.rejected_frames = 0
        self.last_frame_reject_log_at = 0.0
        self.trace_enabled = bool(p('trace_enabled'))
        self.trace_interval_sec = max(
            0.5, float(p('trace_interval_sec')))
        self._trace_counts = {}
        self._trace_total = 0
        self._trace_last_log_at = time.monotonic()

        self.create_subscription(
            OccupancyGrid, str(p('map_topic')), self._on_map, 10)
        self.create_subscription(
            OccupancyGrid, str(p('global_costmap_topic')),
            self._on_global_costmap, 10)
        self.create_subscription(
            OccupancyGrid, str(p('local_costmap_topic')),
            self._on_local_costmap, 10)
        self.create_subscription(
            String, str(p('nav_state_topic')), self._on_nav_state, 10)

        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            'frontier_explorer up: map=%s -> goal=%s (grid frontier selection)'
            % (p('map_topic'), p('frontier_goal_topic')))

    # ------------------------------------------------------------------ 回调
    def _frame_ok(self, msg: OccupancyGrid, label: str) -> bool:
        """拒绝非本栈发布的栅格。

        2026-10-10: 根命名空间的 /global_costmap/costmap 不区分 Nav2 命名空间，
        两套栈并存时可能收到跟随栈（odom_combined 帧）的 costmap。用错数据做
        可达性判断会静默放行/拦截目标，比不判断更危险，因此这里直接丢弃。
        """
        if not self.require_global_frame_match:
            return True
        frame = str(msg.header.frame_id or '').lstrip('/')
        if frame == self.global_frame:
            return True
        self.rejected_frames += 1
        now = time.monotonic()
        if now - self.last_frame_reject_log_at >= 10.0:
            self.last_frame_reject_log_at = now
            self.get_logger().warn(
                'rejecting %s: frame_id=%r != %r (likely the /nofmap stack); '
                'rejected=%d total'
                % (label, frame, self.global_frame, self.rejected_frames))
        return False

    def _on_map(self, msg: OccupancyGrid) -> None:
        if not self._frame_ok(msg, 'map'):
            return
        self.last_map = msg
        self.last_map_at = time.monotonic()

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        if not self._frame_ok(msg, 'global_costmap'):
            return
        self.global_costmap = msg
        self.global_costmap_at = time.monotonic()

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        if not self._frame_ok(msg, 'local_costmap'):
            return
        self.local_costmap = msg
        self.local_costmap_at = time.monotonic()

    def _on_nav_state(self, msg: String) -> None:
        self.nav_state = str(msg.data or '')

    # ------------------------------------------------------------------ 工具
    def _fresh(self, msg: Optional[OccupancyGrid], at: float,
               timeout: float) -> Optional[OccupancyGrid]:
        if msg is None:
            return None
        if timeout > 0.0 and time.monotonic() - at > timeout:
            return None
        return msg

    def fresh_map(self) -> Optional[OccupancyGrid]:
        return self._fresh(self.last_map, self.last_map_at, self.map_timeout_sec)

    def fresh_global_costmap(self) -> Optional[OccupancyGrid]:
        return self._fresh(
            self.global_costmap, self.global_costmap_at, self.costmap_timeout_sec)

    def fresh_local_costmap(self) -> Optional[OccupancyGrid]:
        return self._fresh(
            self.local_costmap, self.local_costmap_at, self.costmap_timeout_sec)

    def current_pose(self) -> Optional[Pose2D]:
        """map 系下位姿，来自 TF map->base_footprint。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        return Pose2D(x=tf.transform.translation.x, y=tf.transform.translation.y)

    def prune_bad_places(self) -> None:
        now = time.monotonic()
        self.bad_places = [
            (x, y, until_at) for x, y, until_at in self.bad_places
            if until_at > now
        ]

    def mark_failed_goal(self, x: float, y: float) -> None:
        self.bad_places.append(
            (x, y, time.monotonic() + self.bad_ttl_sec))

    def to_grid_map(self, msg: OccupancyGrid) -> GridMap:
        return GridMap(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=msg.data,
        )

    def nav_is_busy(self) -> bool:
        """True only for NON-exploration navigation states.

        2026-10-10 两次修正，记录如下：

        (1) 原实现把 'executing' 与 'sending_goal' 也算作忙，但它们在本系统
        里就是探索的正常推进态；管理器的 fallback 采样器会在目标结束后
        立刻补位，使 nav_state 几乎永远停在 executing。已从词表移除。

        (2) 更关键：nav_state 是一个 **JSON 字符串**，直接对整串做子串匹配
        会命中 JSON 的键名，全部是误报：
            'return_home'  <- "return_home_retry_count"
            'mapping_done' <- "mapping_done"
            'blocked_'     <- "blocked_place_count"
            'charging'     <- "base_charging"
            'timeout'      <- "no_progress_timeout"
        必须先解析出**状态字段的值**再匹配。
        """
        raw = self.nav_state
        if not raw:
            return False
        state = ''
        if self.nav_state_is_json:
            try:
                parsed = json.loads(raw)
            except (ValueError, TypeError):
                parsed = None
            if isinstance(parsed, dict):
                for field in self.nav_state_fields:
                    value = parsed.get(field)
                    if isinstance(value, str) and value:
                        state = value
                        break
        if not state:
            # 非 JSON 或解析失败：回退到整串（保守，宁可挡住也不误发目标）
            state = raw
        state = state.lower()
        return any(token in state for token in self.blocked_state_tokens)

    # ------------------------------------------------------------------ 可追溯
    TRACE_BRANCHES = (
        'waiting_map', 'waiting_pose', 'waiting_nav',
        'interval_skip', 'selected', 'no_frontier',
    )

    def _trace(self, branch: str) -> None:
        """记录一次 tick 的分支归属，并按 trace_interval_sec 汇总输出。"""
        if not self.trace_enabled:
            return
        self._trace_counts[branch] = self._trace_counts.get(branch, 0) + 1
        self._trace_total += 1
        now = time.monotonic()
        if now - self._trace_last_log_at < self.trace_interval_sec:
            return
        span = now - self._trace_last_log_at
        self._trace_last_log_at = now
        parts = ', '.join(
            '%s=%d' % (b, self._trace_counts.get(b, 0))
            for b in self.TRACE_BRANCHES
            if self._trace_counts.get(b, 0))
        dominant = max(self._trace_counts.items(), key=lambda kv: kv[1])[0] \
            if self._trace_counts else 'none'
        self.get_logger().info(
            'tick trace (%.0fs): total=%d [%s] dominant=%s'
            % (span, self._trace_total, parts or 'none', dominant))
        self._trace_counts = {}
        self._trace_total = 0

    def _trace_snapshot(self):
        return {
            'intervalSec': round(self.trace_interval_sec, 1),
            'counts': dict(self._trace_counts),
            'total': self._trace_total,
            'mapPresent': self.last_map is not None,
            'mapAgeSec': (None if self.last_map is None else
                          round(time.monotonic() - self.last_map_at, 2)),
            'navState': self.nav_state,
            'globalCostmapAgeSec': (None if self.global_costmap is None else
                                    round(time.monotonic()
                                          - self.global_costmap_at, 2)),
            'localCostmapAgeSec': (None if self.local_costmap is None else
                                   round(time.monotonic()
                                         - self.local_costmap_at, 2)),
            'rejectedFrames': self.rejected_frames,
        }

    # ------------------------------------------------------------------ 主循环
    def tick(self) -> None:
        now = time.monotonic()
        grid_msg = self.fresh_map()
        if grid_msg is None:
            self._trace('waiting_map')
            self._publish_status('waiting_map')
            return
        pose = self.current_pose()
        if pose is None:
            self._trace('waiting_pose')
            self._publish_status('waiting_pose')
            return
        if self.nav_is_busy():
            self._trace('waiting_nav')
            self._publish_status('waiting_nav')
            return
        if now - self.last_goal_at < self.goal_interval_sec:
            self._trace('interval_skip')
            return

        self.prune_bad_places()
        grid = self.to_grid_map(grid_msg)
        if self.search_window_radius_m > 0.0:
            grid = self._windowed_grid(grid, pose)

        select_stats = SelectionStats()
        goal = select_coverage_goal(
            grid,
            pose,
            bad_places=[(x, y) for x, y, _ in self.bad_places],
            min_distance=self.min_goal_distance,
            max_distance=self.max_goal_distance,
            obstacle_clearance=self.obstacle_clearance,
            known_clearance=self.known_clearance,
            corridor_clearance=self.corridor_clearance,
            frontier_radius=self.frontier_radius,
            bad_radius=self.blocked_place_radius,
            global_costmap=(self.to_grid_map(self.fresh_global_costmap())
                            if self.fresh_global_costmap() else None),
            local_costmap=(self.to_grid_map(self.fresh_local_costmap())
                           if self.fresh_local_costmap() else None),
            min_frontier_cells=self.min_frontier_cells,
            gain_scale=self.gain_scale,
            potential_scale=self.potential_scale,
            clearance_scale=self.clearance_scale,
            lookahead_scale=self.lookahead_scale,
            stats=select_stats,
        )

        if goal is None:
            # 显式上报"当前没有可行前沿点"。这是后续"探索已收敛"终态的
            # 判定基础：由连续无目标时长 + 位姿停滞共同决定收敛。
            self._trace('no_frontier')
            self._publish_status('no_frontier')
            if now - self.last_no_goal_log_at >= self.no_goal_log_interval_sec:
                self.last_no_goal_log_at = now
                # 2026-10-10 诊断：把"候选被拒的原因"分类打出来，
                # 用于判定"选不出前沿点"到底是窗口/净空/代价地图哪一环。
                self.get_logger().warn(
                    'no selectable frontier: pose=(%.2f, %.2f) window=%.1fm '
                    'min_cells=%d bad_places=%d nav=%s | %s'
                    % (pose.x, pose.y, self.search_window_radius_m,
                       self.min_frontier_cells, len(self.bad_places),
                       self.nav_state or 'n/a', select_stats.summary()))
            return

        self._trace('selected')
        self._publish_goal(goal.x, goal.y, pose)
        self.last_goal_at = now
        self._publish_status('goal_sent', goal=(goal.x, goal.y, goal.score))
        self.get_logger().info(
            'frontier goal: (%.2f, %.2f) score=%.2f clusters_ok'
            % (goal.x, goal.y, goal.score))

    def _windowed_grid(self, grid: GridMap, pose: Pose2D) -> GridMap:
        """把搜索窗口外的格标为已占用，避免对全图做 O(格数) 的距离场与 BFS。

        2026-10-10: selector 的 build_distance_field / reachable_free_space 都是
        全图操作，507x587 约 30 万格，每拍全算在 NUC 上有实测到的 CPU 争用风险
        （EKF 曾因此错过更新率）。限制窗口后成本与窗口面积成正比。
        """
        mx, my = int(round((pose.x - grid.origin_x) / grid.resolution)), \
            int(round((pose.y - grid.origin_y) / grid.resolution))
        r = int(math.ceil(self.search_window_radius_m / grid.resolution))
        data = list(grid.data)
        x0, x1 = max(0, mx - r), min(grid.width, mx + r + 1)
        y0, y1 = max(0, my - r), min(grid.height, my + r + 1)
        for yy in range(grid.height):
            row = yy * grid.width
            if y0 <= yy < y1:
                # 行内只保留窗口段，两侧标为占用
                for xx in range(0, x0):
                    data[row + xx] = 100
                for xx in range(x1, grid.width):
                    data[row + xx] = 100
            else:
                for xx in range(grid.width):
                    data[row + xx] = 100
        return GridMap(width=grid.width, height=grid.height,
                       resolution=grid.resolution,
                       origin_x=grid.origin_x, origin_y=grid.origin_y,
                       data=data)

    def _publish_goal(self, x: float, y: float, pose: Pose2D) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        yaw = math.atan2(y - pose.y, x - pose.x)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(msg)

    def _publish_status(self, state: str, goal=None) -> None:
        # 2026-10-10: 只要 trace 计数有变化就发布，不再因 state 未变而完全
        # 静默——否则长时间卡住时外部观察不到任何心跳。
        trace_changed = self._trace_total != self._last_published_trace_total
        if (state == self.last_status_state and goal is None
                and not trace_changed):
            return
        self.last_status_state = state
        self._last_published_trace_total = self._trace_total
        payload = {
            'state': state,
            'goal': None if goal is None else {
                'x': round(goal[0], 3), 'y': round(goal[1], 3),
                'score': round(goal[2], 3)},
            'badPlaces': len(self.bad_places),
            'rejectedFrames': self.rejected_frames,
            'trace': self._trace_snapshot(),
            'globalCostmapAge': (
                None if self.global_costmap is None
                else round(time.monotonic() - self.global_costmap_at, 1)),
            'navState': self.nav_state,
            'atEpochMs': int(time.time() * 1000),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
