"""In-place rotation escape controller for the mapping stack.

Why this exists
---------------
In a corridor narrower than the robot's turning circle the mapping stack can
wedge:  Nav2 refuses to Spin ("Collision Ahead - Exiting Spin"), the planner
finds no path, and the R1b retreat refuses to move because the map cannot
confirm the rear is clear.  All three Nav2 recoveries are refused at once and
the robot needs a human push.

This node does not try to out-predict Nav2.  It keeps Nav2's safety judgement
exactly as it is and acts only *after* that judgement has already refused to
move, doing the one thing a human would do: rotate the base slowly in place
until the robot's own footprint no longer overlaps an obstacle.

Judgement reuse (the important part)
------------------------------------
Both the "stuck" and the "escaped" decision use Nav2's own data and Nav2's own
algorithm:

  * footprint : /local_costmap/published_footprint (Nav2's own footprint,
                including the deliberate enlargement - NOT the vision one)
  * costmap   : /local_costmap/costmap_raw (nav2_msgs/Costmap)
  * algorithm : replica of nav2_costmap_2d::FootprintCollisionChecker
                - pointCost(x, y) = raw costmap value, no smoothing
                - lineCost        = max over the rasterised edge
                - footprintCost   = max over all footprint edges
                - collision       = footprintCost >= LETHAL_OBSTACLE (254)
                Points off the grid count as LETHAL, matching footprintCost()'s
                worldToMap-failure path.

Exit therefore happens as soon as *Nav2 itself* would call the current heading
collision free - the requested hand-back condition.

Transforms
----------
published_footprint arrives already rotated and translated into the costmap
frame, so the robot-frame footprint is recovered by undoing the current pose
(the inverse of nav2_costmap_2d::transformFootprint), and each probe re-applies
it at the probed yaw.  This keeps the node independent of any footprint
parameter of its own.

What it deliberately does NOT do
--------------------------------
  * no change to any Nav2 parameter, costmap, footprint or recovery config
  * no reverse motion (this base has no rear sensing)
  * no clearing of the local costmap (Spin depends on it)
  * no use of the vision-stack footprint

Safety envelope
---------------
Every activation is bounded by all of: max angular speed, max total rotation,
max duration, a minimum number of steps before an exit is accepted, and an
optional lidar front abort.  It publishes only while armed, and always
publishes a final zero Twist before disarming.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# nav2_costmap_2d/cost_values.hpp
LETHAL_OBSTACLE = 254


def _yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def bresenham(x0, y0, x1, y1):
    """Yield integer cells on the line (x0,y0)-(x1,y1) inclusive.

    Mirrors nav2_util::LineIterator, which FootprintCollisionChecker uses to
    rasterise each footprint edge.
    """
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


# 2026-10-13 P33：manager 处于下列状态时，车"停着"是正常的（在等目标），
# 而不是被卡死。此时 Nav2 不会发 cmd_vel，_cmd_vel_is_quiet() 恒为 True，
# 现有武装判据会把"没事干"误判成"静默地卡死"并原地转圈。
# 取值清单来自 cartographer_manager.py 与 stemm_nav2_manager/manager.py 中
# navigation_state 的全部赋值点（共 70 个状态名）。
# 注意：returning_home / return_home_* 故意不在其中 —— 返航时必须允许脱困。
NAV_IDLE_STATES = frozenset((
    'idle',
    'waiting_for_frontier',
    'waiting_for_rrt_frontier',
    'waiting_for_nav2',
    'warming_up',
    'stuck_waiting_new_goal',
    'succeeded',
    'canceled',
    'goal_unreachable',
    'goal_rejected',
    'goal_acceptance_timeout',
    'goal_response_error',
    'goal_send_error',
    'timed_out_goal_rejected',
    'base_charging',
    'charging_canceling_active_goal',
    'charging_canceling_spin',
    'charging_waiting_for_goal_acceptance',
    'return_home_failed_waiting_manual',
    'shutdown_requested',
))


class EscapeController(Node):
    """Slow fixed-step in-place rotation until Nav2's footprint test passes."""

    def __init__(self):
        super().__init__('stemm_escape_controller')

        # ---- master switch: default OFF so deployment changes nothing ----
        self.declare_parameter('enabled', True)

        # ---- inputs ----
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('footprint_topic',
                               '/local_costmap/published_footprint')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('nav_state_topic',
                               '/stemm_cartographer/nav_state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ---- trigger ----
        self.declare_parameter('stall_radius_m', 0.05)
        self.declare_parameter('stall_sec', 3.0)
        self.declare_parameter('require_nav_refusal', True)
        self.declare_parameter('refusal_grace_sec', 15.0)
        # 2026-10-12 E-2：释放必须以"代价真的改善"为前提。
        # 原条件只要求 cost < LETHAL(254)，而 wedge 路径武装时就是 253，
        # 恒成立 —— 于是转够 min_steps 就撒手，实测 7/8 次 armed=253 ->
        # release=253，1 秒后同位置重新武装（起点附近高频掉头）。
        self.declare_parameter('escape_require_cost_improvement', True)
        # 2026-10-13 P12-#1：无效旋转换向。
        # 实测 14:29 轮 10 次 escape 里有 6 次 rotation_budget_exhausted
        # 且 armed_cost 253 -> release_cost 253（转满 2.10rad 毫无改善）。
        # 原因是 _pick_direction 每次都按当前 10 度探测重新选向、且失败后
        # 不记忆，于是同一位置可能反复选同一个方向，形成无效旋转循环。
        # 本开关：记住"该位置该方向失败过"，下次强制反向。
        self.declare_parameter('escape_flip_after_failed_budget', True)
        # 每个位置最多换向几次（防新循环）；位置按格点归并。
        self.declare_parameter('escape_flip_max_per_spot', 2)
        self.declare_parameter('escape_flip_spot_radius_m', 0.30)
        # arm_cooldown 8.0 -> 3.0：既然现在"没改善不放行"，长时间干等没有
        # 意义，尽快再试反而更有效。
        self.declare_parameter('arm_cooldown_sec', 3.0)

        # ---- motion envelope ----
        # 2026-10-12: min_steps_before_exit 5 -> 15, rotate_speed 0.25 -> 0.40。
        # 原值单次介入只转 5 × 0.25 × 0.1 = 0.125 rad ≈ 7.2°（日志实测
        # 'rotated 0.12 rad over 5 steps in 0.6s'）。7.2° 太小，转完 footprint
        # 仍在内切膨胀环内，释放后立刻又满足楔死条件，8s cooldown 后再来一次
        # —— 表现为"高频自转"，且 8 次触发里 4 次落在同一格 (0.04~0.05, 0.16)。
        # 现单次约 15 × 0.40 × 0.1 = 0.60 rad ≈ 34°，一次介入能真正改变几何；
        # 仍受 max_total_rotation_rad(120°) / return_home_max_rotation_rad(60°)
        # 预算与 max_duration_sec(20s)、front_abort 约束，不会无界旋转。
        self.declare_parameter('rotate_speed_radps', 0.40)
        self.declare_parameter('command_period_sec', 0.1)
        self.declare_parameter('max_total_rotation_rad', 2.10)   # ~120 deg
        self.declare_parameter('max_duration_sec', 20.0)
        self.declare_parameter('min_steps_before_exit', 15)
        self.declare_parameter('direction', 1.0)   # +1 CCW, -1 CW, 0 = auto

        # ---- front abort (lidar) ----
        self.declare_parameter('front_abort_enabled', True)
        self.declare_parameter('front_abort_m', 0.12)
        self.declare_parameter('front_cone_deg', 35.0)

        # ---- diagnostics ----
        self.declare_parameter('log_every_n_steps', 20)
        self.declare_parameter('input_stale_sec', 3.0)
        # 2026-10-10: 只允许在建图阶段介入。返航阶段同样会出现
        # front_blocked=True 与阻塞态 token，若不设闸门会把返航中的车原地转起来。
        self.declare_parameter('mapping_phase_only', True)
        # 2026-10-10 (operator): 返航阶段受限介入（方案 b）。
        # 返航途中旋转会改变朝向、可能让返航路径更差，故限幅更小，
        # 且释放前必须回到进入该阶段时的朝向附近。
        self.declare_parameter('return_home_max_rotation_rad', 1.05)   # 60 deg
        # 2026-10-12: 原 0.087（5°）与 return_home_completion_requires_orientation
        # =False 的设计意图矛盾 —— 系统本就只按位置判定返航完成，朝向不该
        # 阻塞收尾。实测 14:02 轮 escape 卡在返航分支出不去：
        # 'footprint clear but heading offset 35 deg > 5 deg; continuing'，
        # 转满 1.05rad 预算仍差 16-22° 后 'rotation_budget_exhausted'。
        # 放宽到 0.35（20°）：既不再卡住收尾，也仍要求基本维持原朝向。
        self.declare_parameter(
            'return_home_heading_tolerance_rad', 0.35)  # 20 deg
        # 2026-10-10: 独立条件（仅在返航阶段生效，方案 i）。
        # 原则：唯一目的是防止"Nav2 判卡死但车真静止无法脱困"的极端情况，
        # 允许稍放松但绝不能轻易触发，故该路径的停滞阈值更长。
        self.declare_parameter('return_home_stall_sec', 4.0)
        self.declare_parameter('return_home_require_collision_hold', True)
        self.declare_parameter('cmd_vel_quiet_threshold', 0.02)
        self.declare_parameter('trace_interval_sec', 5.0)
        # 2026-10-10 (operator, P0): 楔死判据。实测窄道卡死停在
        # footprint_cost=253（内切膨胀环）而非 254，导致原判据结构性失效。
        # 三个条件必须同时连续满足才接管，以服从"宁可漏抓、绝不误触"。
        self.declare_parameter('wedge_enabled', True)
        # 2026-10-13 P33：只有"确实有导航目标在执行"时才允许脱困。
        # 实测 18:04 轮开局：车在 waiting_for_frontier（等候选）时被
        # escape 反复武装并原地白转 3 次（rotation_budget_exhausted）。
        self.declare_parameter('escape_require_active_goal', True)
        self.declare_parameter('wedge_cost_threshold', 253)
        self.declare_parameter('wedge_hold_sec', 4.0)
        # 2026-10-13 P19：长时间停滞的独立脱困判据。
        # 实测 15:24:52~15:25:17：cost=207/181 且 stalled 持续 29.8s，
        # 但楔死判据要求 cost>=253，于是 escape 全程不介入（esc=False），
        # 车静止近 30 秒（视频 1:03 处）。
        # Nav2 公式 cost=253*exp(-3*(d-inscribed))：207->离障碍 0.395m、
        # 181->0.440m。故不能靠降 cost 阈值（降到 ~180 会在正常行驶中误触发），
        # 而应把"停滞时长"作为独立判据：停滞够久 + 足迹已进入膨胀带即可武装。
        self.declare_parameter('wedge_stall_override_enabled', True)
        # 停滞多久算真卡住。实测正常行驶的 stall_hold 多在 0~5s，
        # 15s 是明确异常；比 wedge_hold_sec(4s) 长得多以免误触。
        self.declare_parameter('wedge_stall_override_sec', 15.0)
        # 足迹代价下限：150 约等于"离障碍 0.46m"。
        # 用于排除"开阔地空转"（cost=0 时不该转），兼顾实测的 181/207。
        # 2026-10-14 A3：150 -> 253。
        # Nav2 膨胀公式 cost = 253*exp(-3.0*(d-0.328))，反算：
        #   cost=150 -> d=0.502 m > 车外接半径 0.469 m
        #   => 该状态下车辆任何朝向都不压障碍，根本没卡住；
        #      旧门槛让 stall_override 在"车有充分空间"时也武装旋转。
        #   cost=253 -> d=0.328 m，才是"障碍侵入内切圆"的真正卡死。
        # 回退：置回 150。
        self.declare_parameter('wedge_stall_override_min_cost', 253)
        # 2026-10-14 G2："静止且无代价"路径。
        # 实测 14:46：R1 后退后车停在 footprint_cost=0 的格子上（任何朝向
        # 都不压障碍），但 planner 报 "Starting point in lethal space" 而
        # 完全动不了。三条既有路径都以 cost 为必要条件，于是 escape 全程
        # 失明 51 秒（trace 里 stall_hold 涨到 29.1s、cmd_vel_quiet=True）。
        # 本条不再要求 cost："有活跃目标 + 零位移 >= 门槛 + 完全不发速度"
        # 已足以证明物理上动不了。A3 担心的"开阔地空转"发生在
        # waiting_for_frontier（无活跃目标），已被 escape_require_active_goal 挡掉。
        self.declare_parameter('wedge_no_cost_stall_enabled', True)
        # 该路径的停滞门槛，比 stall_override(15s) 更保守。
        self.declare_parameter('wedge_no_cost_stall_sec', 20.0)

        g = self.get_parameter
        self.enabled = bool(g('enabled').value)
        self.costmap_topic = str(g('costmap_topic').value)
        self.footprint_topic = str(g('footprint_topic').value)
        self.odom_topic = str(g('odom_topic').value)
        self.scan_topic = str(g('scan_topic').value)
        self.nav_state_topic = str(g('nav_state_topic').value)
        self.cmd_vel_topic = str(g('cmd_vel_topic').value)

        self.stall_radius_m = max(0.005, float(g('stall_radius_m').value))
        self.stall_sec = max(0.5, float(g('stall_sec').value))
        self.require_nav_refusal = bool(g('require_nav_refusal').value)
        self.refusal_grace_sec = max(1.0, float(g('refusal_grace_sec').value))
        self.escape_require_cost_improvement = bool(
            g('escape_require_cost_improvement').value)
        self.escape_flip_after_failed_budget = bool(
            g('escape_flip_after_failed_budget').value)
        self.escape_flip_max_per_spot = max(0, int(
            g('escape_flip_max_per_spot').value))
        self.escape_flip_spot_radius_m = max(0.05, float(
            g('escape_flip_spot_radius_m').value))
        self.arm_cooldown_sec = max(0.0, float(g('arm_cooldown_sec').value))

        self.rotate_speed_radps = max(
            0.02, float(g('rotate_speed_radps').value))
        self.command_period_sec = max(
            0.02, float(g('command_period_sec').value))
        self.max_total_rotation_rad = max(
            0.05, float(g('max_total_rotation_rad').value))
        self.max_duration_sec = max(1.0, float(g('max_duration_sec').value))
        self.min_steps_before_exit = max(
            0, int(g('min_steps_before_exit').value))
        self.direction = float(g('direction').value)

        self.front_abort_enabled = bool(g('front_abort_enabled').value)
        self.front_abort_m = max(0.0, float(g('front_abort_m').value))
        self.front_cone_deg = max(1.0, float(g('front_cone_deg').value))

        self.log_every_n_steps = max(1, int(g('log_every_n_steps').value))
        self.input_stale_sec = max(0.5, float(g('input_stale_sec').value))
        self.mapping_phase_only = bool(g('mapping_phase_only').value)
        self.return_home_max_rotation_rad = max(
            0.05, float(g('return_home_max_rotation_rad').value))
        self.return_home_heading_tolerance_rad = max(
            0.0, float(g('return_home_heading_tolerance_rad').value))
        self.return_home_stall_sec = max(
            1.0, float(g('return_home_stall_sec').value))
        self.return_home_require_collision_hold = bool(
            g('return_home_require_collision_hold').value)
        self.cmd_vel_quiet_threshold = max(
            0.0, float(g('cmd_vel_quiet_threshold').value))
        self.trace_interval_sec = max(
            1.0, float(g('trace_interval_sec').value))
        self.wedge_enabled = bool(g('wedge_enabled').value)
        self.escape_require_active_goal = bool(
            g('escape_require_active_goal').value)
        self.wedge_no_cost_stall_enabled = bool(
            g('wedge_no_cost_stall_enabled').value)
        self.wedge_no_cost_stall_sec = max(
            1.0, float(g('wedge_no_cost_stall_sec').value))
        self.wedge_cost_threshold = int(max(
            1, min(254, int(g('wedge_cost_threshold').value))))
        self.wedge_hold_sec = max(
            1.0, float(g('wedge_hold_sec').value))
        # 2026-10-13 P19：停滞超时独立判据
        self.wedge_stall_override_enabled = bool(
            g('wedge_stall_override_enabled').value)
        self.wedge_stall_override_sec = max(
            1.0, float(g('wedge_stall_override_sec').value))
        self.wedge_stall_override_min_cost = int(max(
            1, min(254, int(g('wedge_stall_override_min_cost').value))))

        # ---- state ----
        self.costmap = None
        self.costmap_stamp = 0.0
        self.footprint_poly = None       # list[(x, y)] in costmap frame
        self.footprint_robot = None      # list[(x, y)] in base frame
        self.footprint_stamp = 0.0
        self.pose = None                 # (x, y, yaw) in costmap frame
        self.pose_stamp = 0.0
        self.front_range = None
        self.front_stamp = 0.0
        self.nav_state_raw = ''
        self.mapping_phase_active = False   # 阶段闸门（见 _on_nav_state）
        self.nav_refused_at = None

        # ---- stall tracking ----
        self._stall_anchor = None
        self._stalled_since = None

        # ---- escape episode ----
        self._escaping = False
        self._escape_started = 0.0
        self._escape_rotated = 0.0
        self._escape_direction = 1.0
        self._escape_steps = 0
        self._last_arm_time = float('-inf')
        self._last_cmd_vel = 0.0
        self._last_cmd_vel_stamp = 0.0
        self._collision_hold_since = None   # 足印代价连续 >=254 的起点
        self._wedge_hold_since = None       # 足印代价连续 >=阈值(253) 的起点
        self._escape_yaw_ref = None         # 进入本阶段的参考朝向
        self._return_home_escape = False
        self._escape_budget = 2.10
        self._last_trace = 0.0
        # 2026-10-10 诊断：记录本轮的触发路径 / 真实阈值 / 触发与释放代价
        self._escape_path = ''
        self._armed_cost = None
        # 2026-10-13 P12-#1：失败方向记忆 {(格x, 格y): [方向, 次数]}
        self._failed_dir_memory = {}
        self._armed_threshold = None
        self._release_cost = None

        # ---- IO ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Costmap, self.costmap_topic, self._on_costmap, costmap_qos)
        self.create_subscription(
            PolygonStamped, self.footprint_topic, self._on_footprint, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            String, self.nav_state_topic, self._on_nav_state, 10)
        # 2026-10-10: 旁听 cmd_vel 作为"是否仍在被命令运动"的判据。
        self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)

        self.create_timer(self.command_period_sec, self._tick)

        self.get_logger().info(
            'escape controller up: enabled=%s speed=%.2f rad/s max_rot=%.2f '
            'rad max_dur=%.1fs direction=%s min_steps=%d'
            % (self.enabled, self.rotate_speed_radps,
               self.max_total_rotation_rad, self.max_duration_sec,
               self.direction, self.min_steps_before_exit))

    # ------------------------------------------------------------------ IO
    def _on_costmap(self, msg: Costmap):
        self.costmap = msg
        self.costmap_stamp = time.monotonic()

    def _on_footprint(self, msg: PolygonStamped):
        pts = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts) < 3:
            return
        self.footprint_poly = pts
        self.footprint_stamp = time.monotonic()
        if self.pose is not None:
            self._rebuild_robot_footprint()

    def _rebuild_robot_footprint(self):
        """Recover the base-frame footprint from the costmap-frame polygon.

        Inverse of nav2_costmap_2d::transformFootprint, which produced the
        polygon we subscribed to:
            world = pose + R(theta) * base
        so
            base  = R(-theta) * (world - pose)
        """
        if self.footprint_poly is None or self.pose is None:
            return
        x, y, yaw = self.pose
        cos_th = math.cos(-yaw)
        sin_th = math.sin(-yaw)
        robot = []
        for wx, wy in self.footprint_poly:
            dx = wx - x
            dy = wy - y
            robot.append((dx * cos_th - dy * sin_th,
                          dx * sin_th + dy * cos_th))
        self.footprint_robot = robot

    def _on_cmd_vel(self, msg: Twist):
        """旁听 /cmd_vel，用于判断"刚刚是否仍在被命令运动"。

        A robot that is still being commanded to move is not wedged - it may
        simply be slow or briefly blocked.  Refusing to take over in that case
        is one of the guards that keeps this node from firing easily.
        """
        self._last_cmd_vel = max(abs(msg.linear.x) + abs(msg.linear.y),
                                 abs(msg.angular.z))
        self._last_cmd_vel_stamp = time.monotonic()

    def _cmd_vel_is_quiet(self, now):
        if self._last_cmd_vel_stamp <= 0.0:
            return True          # no evidence of commanded motion
        if (now - self._last_cmd_vel_stamp) > 1.0:
            return True          # nobody has commanded anything recently
        return self._last_cmd_vel <= self.cmd_vel_quiet_threshold

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y,
                     _yaw_from_quaternion(msg.pose.pose.orientation))
        self.pose_stamp = time.monotonic()
        if self.footprint_poly is not None and self.footprint_robot is None:
            self._rebuild_robot_footprint()

    def _on_scan(self, msg: LaserScan):
        if not self.front_abort_enabled:
            return
        cone = math.radians(self.front_cone_deg)
        best = None
        angle = msg.angle_min
        for r in msg.ranges:
            if -cone <= angle <= cone and math.isfinite(r) and r > 0.0:
                if best is None or r < best:
                    best = r
            angle += msg.angle_increment
        if best is not None:
            self.front_range = best
            self.front_stamp = time.monotonic()

    def _on_nav_state(self, msg: String):
        self.nav_state_raw = msg.data or ''
        self.mapping_phase_active = self._mapping_phase_active(
            self.nav_state_raw)
        if self._nav_state_says_refused(self.nav_state_raw):
            self.nav_refused_at = time.monotonic()

    # 返航阶段会出现的 mode 取值；命中任一即认为已离开建图阶段。
    RETURN_HOME_MODES = (
        'returning_home', 'return_home', 'home', 'returning',
        'mapping_finishing', 'finishing_map', 'saving_final_map',
        'shutting_down', 'shutdown',
    )

    @staticmethod
    def _mapping_phase_active(raw: str) -> bool:
        """True only while the manager reports the MAPPING phase.

        Return-home reuses the same blocked states (front_blocked, blocking
        navigation_state tokens) that arm the escape, so without this gate the
        node would seize the controls mid-return-home and spin the robot.

        Fails closed: anything unparseable, or any phase flag that cannot be
        read, is treated as "not mapping" so the node stays hands-off rather
        than acting in the wrong phase.
        """
        if not raw:
            return False
        try:
            data = json.loads(raw)
        except (ValueError, TypeError):
            return False
        if not isinstance(data, dict):
            return False

        # Explicit phase flags published by the manager.  mapping_done and
        # pending_home_goal are both set at the exact moment mapping completes
        # and return-home is requested (cartographer_manager.py:4277-4281).
        if data.get('mapping_done') is True:
            return False
        if data.get('pending_home_goal') is True:
            return False

        mode = data.get('mode')
        if mode is None:
            # No phase information at all -> refuse to act.
            return False
        if str(mode).lower() in EscapeController.RETURN_HOME_MODES:
            return False
        return True

    def _nav_state_is_idle(self) -> bool:
        """P33：manager 是否处于"等目标/空闲"状态。

        这些状态下 Nav2 不会发 cmd_vel，_cmd_vel_is_quiet() 因而恒为 True，
        现有武装判据会把"没事干"误判成"静默地卡死"。
        任何解析失败都返回 False（不动门禁），以免削弱真实脱困能力。
        """
        # getattr 兜底：ManagerHarness 等测试替身不走真实 __init__，
        # 缺该属性时按"无信号"处理（不动门禁），而不是抛异常。
        raw = getattr(self, 'nav_state_raw', '')
        if not raw:
            return False
        try:
            data = json.loads(raw)
        except (ValueError, TypeError):
            return False
        if not isinstance(data, dict):
            return False
        state = str(data.get('navigation_state', '')).strip().lower()
        return state in NAV_IDLE_STATES

    @staticmethod
    def _nav_state_says_refused(raw: str) -> bool:
        """True when the manager reports a CURRENT blockage.

        Parsed from the JSON the manager publishes for diagnostics.  Any parse
        problem means 'no signal', never 'refused'.
        """
        if not raw:
            return False
        try:
            data = json.loads(raw)
        except (ValueError, TypeError):
            return False
        if not isinstance(data, dict):
            return False
        # (a) navigation_state reflects the CURRENT state, so matching it is
        #     safe.  (b) front_blocked is a live boolean from the manager.
        # Deliberately NOT using last_error: that field is sticky - the
        # manager writes 'front obstacle too close' once and never clears it,
        # so keying off it would arm the escape during ordinary slow
        # approaches long after the obstacle was gone.
        state = str(data.get('navigation_state', ''))
        for token in ('blocked_waiting_clearance', 'blocked_precheck_pending',
                      'starting_stall_recovery',
                      'clearing_costmaps_for_spin_recovery'):
            if token in state:
                return True
        return data.get('front_blocked') is True

    # ------------------------------------------------------- nav2 algorithm
    def _world_to_map(self, wx, wy):
        md = self.costmap.metadata
        res = md.resolution
        if res <= 0.0:
            return None
        mx = int((wx - md.origin.position.x) / res)
        my = int((wy - md.origin.position.y) / res)
        if mx < 0 or my < 0 or mx >= md.size_x or my >= md.size_y:
            return None
        return mx, my

    def footprint_cost(self, x, y, yaw, footprint):
        """Replica of FootprintCollisionChecker::footprintCost.

        Returns LETHAL_OBSTACLE if any footprint point falls off the grid, else
        the maximum raw cost along the rasterised footprint edges - exactly the
        number Nav2 compares against LETHAL_OBSTACLE.
        """
        if self.costmap is None or not footprint or len(footprint) < 3:
            return None
        cos_th = math.cos(yaw)
        sin_th = math.sin(yaw)
        cells = []
        for fx, fy in footprint:
            wx = x + (fx * cos_th - fy * sin_th)
            wy = y + (fx * sin_th + fy * cos_th)
            cell = self._world_to_map(wx, wy)
            if cell is None:
                return float(LETHAL_OBSTACLE)
            cells.append(cell)

        best = 0.0
        n = len(cells)
        size_x = self.costmap.metadata.size_x
        data = self.costmap.data
        for i in range(n):
            x0, y0 = cells[i]
            x1, y1 = cells[(i + 1) % n]
            for cx, cy in bresenham(x0, y0, x1, y1):
                c = data[cy * size_x + cx]
                if c >= LETHAL_OBSTACLE:
                    return float(LETHAL_OBSTACLE)
                if c > best:
                    best = float(c)
        return best

    def current_footprint_cost(self):
        if self.pose is None or self.footprint_robot is None:
            return None
        x, y, yaw = self.pose
        return self.footprint_cost(x, y, yaw, self.footprint_robot)

    # ------------------------------------------------------------- triggers
    def _update_collision_hold(self, now, cost):
        """Track continuity of BOTH the lethal state and the wedge state.

        lethal (>=254): the footprint overlaps an obstacle cell.
        wedge  (>=253): the footprint is pressed against the inscribed
        inflation ring - it cannot rotate, yet no obstacle cell is touched.
        2026-10-10: measured narrow-corridor wedges sit at 253, never 254,
        so the wedge timer is what actually detects them.
        """
        lethal = cost is not None and cost >= float(LETHAL_OBSTACLE)
        if lethal:
            if self._collision_hold_since is None:
                self._collision_hold_since = now
        else:
            self._collision_hold_since = None

        wedged = (cost is not None
                  and cost >= float(self.wedge_cost_threshold))
        if wedged:
            if self._wedge_hold_since is None:
                self._wedge_hold_since = now
        else:
            self._wedge_hold_since = None
        return self._collision_hold_since

    # ------------------------------------------------------------- triggers
    def _update_stall(self, now):
        if self.pose is None:
            self._stall_anchor = None
            self._stalled_since = None
            return False
        x, y, _ = self.pose
        if self._stall_anchor is None:
            self._stall_anchor = (x, y, now)
            return False
        ax, ay, _at = self._stall_anchor
        if math.hypot(x - ax, y - ay) > self.stall_radius_m:
            self._stall_anchor = (x, y, now)
            self._stalled_since = None
            return False
        if self._stalled_since is None:
            self._stalled_since = now
        return (now - self._stalled_since) >= self.stall_sec

    def _nav_refusal_recent(self, now):
        if self.nav_refused_at is None:
            return False
        return (now - self.nav_refused_at) <= self.refusal_grace_sec

    def _inputs_fresh(self, now):
        if self.costmap is None or self.footprint_robot is None:
            return False
        if self.pose is None:
            return False
        return ((now - self.costmap_stamp) <= self.input_stale_sec
                and (now - self.pose_stamp) <= self.input_stale_sec)

    # ---------------------------------------------------------------- tick
    def _emit_trace(self, now, cost, stalled):
        """每秒量级的诊断埋点：足印代价 + 是否停滞 + 阶段。

        2026-10-10 新增：此前的日志只含 front/left/right 标量，没有代价地图，
        导致"足印是否真的碰撞"只能靠推断。有了这行，下轮可直接量化误触率。
        """
        if (now - self._last_trace) < self.trace_interval_sec:
            return
        self._last_trace = now
        phase = 'mapping' if self.mapping_phase_active else 'return_home'
        hold = None
        if self._collision_hold_since is not None:
            hold = now - self._collision_hold_since
        wedge = None
        if self._wedge_hold_since is not None:
            wedge = now - self._wedge_hold_since
        self.get_logger().info(
            'escape trace: phase=%s footprint_cost=%s lethal=%s stalled=%s '
            'stall_hold=%.1fs collision_hold=%s wedge_hold=%s '
            'cmd_vel_quiet=%s escaping=%s'
            % (phase,
               ('%d' % cost) if cost is not None else 'n/a',
               (cost is not None and cost >= float(LETHAL_OBSTACLE)),
               stalled,
               (now - self._stalled_since) if self._stalled_since else 0.0,
               ('%.1fs' % hold) if hold is not None else 'no',
               ('%.1fs' % wedge) if wedge is not None else 'no',
               self._cmd_vel_is_quiet(now),
               self._escaping))

    def _tick(self):
        now = time.monotonic()
        if not self.enabled:
            return
        stalled = self._update_stall(now)
        if self.enabled:
            self._emit_trace(now, self.current_footprint_cost(), stalled)
        if not self._escaping:
            self._maybe_arm(now, stalled)
        else:
            self._step(now)

    def _maybe_arm(self, now, stalled):
        if (now - self._last_arm_time) < self.arm_cooldown_sec:
            return
        if not self._inputs_fresh(now):
            return

        # 2026-10-13 P33：没有活跃导航目标时不脱困。
        # 实测 18:05-18:06：车在 waiting_for_frontier（等候选）期间被 escape
        # 武装 4 次，其中 3 次原地转满 2.12rad 而 cost 恒为 253
        # （armed 253 -> release 253）—— 车并没有卡住，只是在等目标。
        if (getattr(self, 'escape_require_active_goal', True)
                and self._nav_state_is_idle()):
            return

        cost = self.current_footprint_cost()
        if cost is None:
            return
        hold_since = self._update_collision_hold(now, cost)

        # ---- P0：楔死路径（253 + 连续不动 + 无速度指令）----
        # 2026-10-10: 实测窄道卡死停在 253 而非 254，原判据抓不到。
        # 三个条件同时连续满足 wedge_hold_sec 才接管，服从"绝不误触"。
        if (self.wedge_enabled
                and cost >= float(self.wedge_cost_threshold)
                and stalled
                and self._stalled_since is not None
                and (now - self._stalled_since) >= self.wedge_hold_sec
                and self._wedge_hold_since is not None
                and (now - self._wedge_hold_since) >= self.wedge_hold_sec
                and self._cmd_vel_is_quiet(now)):
            self._start_escape(now, cost,
                               return_home=not self.mapping_phase_active,
                               path='wedge')
            return

        # ---- 2026-10-13 P19：停滞超时独立判据 ----
        # 不要求 cost 达 wedge_cost_threshold(253)：只要"停滞够久"且
        # "足迹已进入膨胀带"就武装。用于救"离障碍 0.4m 却静止 30s"这类
        # 楔死判据与 LETHAL 判据都覆盖不到的情形。
        # 仍保留 cmd_vel 静默与输入新鲜度约束，避免在 Nav2 正常推进时抢控制权。
        if (self.wedge_stall_override_enabled
                and stalled
                and cost >= float(self.wedge_stall_override_min_cost)
                and self._stalled_since is not None
                and (now - self._stalled_since) >= self.wedge_stall_override_sec
                and self._cmd_vel_is_quiet(now)):
            self._start_escape(now, cost,
                               return_home=not self.mapping_phase_active,
                               path='stall_override')
            return

        # ---- 2026-10-14 G2：静止且无代价（起点被判 lethal 型）----
        # 不要求 cost >= 253：实测"起点在 lethal space"时 footprint_cost 可为 0，
        # 但 Nav2 有目标却零位移、且完全不发速度，物理上就是动不了。
        if (self.wedge_no_cost_stall_enabled
                and stalled
                and self._stalled_since is not None
                and (now - self._stalled_since) >= self.wedge_no_cost_stall_sec
                and self._cmd_vel_is_quiet(now)):
            self._start_escape(now, cost,
                               return_home=not self.mapping_phase_active,
                               path='no_cost_stall')
            return

        # ---- 原路径：足印真的压到障碍格（>=254）----
        # 足印代价 <254 且未楔死时，Nav2 自己就能走，本节点不该动。
        if cost < float(LETHAL_OBSTACLE):
            return

        in_mapping = self.mapping_phase_active
        if not in_mapping:
            if self.mapping_phase_only is False:
                return
            # ---- 返航/收尾阶段：受限路径（方案 i，只在返航阶段放宽）----
            if not stalled:
                return
            if (now - self._stalled_since) < self.return_home_stall_sec:
                return
            if (self.return_home_require_collision_hold
                    and hold_since is not None
                    and (now - hold_since) < self.return_home_stall_sec):
                return
            if not self._cmd_vel_is_quiet(now):
                # Still being commanded to move -> not wedged yet.
                return
            self._start_escape(now, cost, return_home=True,
                               path='lethal_return')
            return

        # ---- 建图阶段：原判据，行为不变 ----
        if not stalled:
            return
        if self.require_nav_refusal and not self._nav_refusal_recent(now):
            return
        self._start_escape(now, cost, return_home=False,
                           path='lethal_mapping')

    def _start_escape(self, now, cost, return_home=False, path='?'):
        direction = self.direction
        if direction == 0.0:
            direction = self._pick_direction()
        else:
            # 2026-10-13 P12-#1 修正：direction 固定（默认 1.0=CCW）时，
            # 原代码不会调用 _pick_direction，flip 逻辑就成了死代码。
            # 这里显式查一次"本点该方向是否失败过"，是则强制反向。
            if self.pose is not None:
                direction = self._flip_if_failed(
                    self.pose[0], self.pose[1], direction)
        self._escape_direction = 1.0 if direction >= 0.0 else -1.0
        self._escaping = True
        self._return_home_escape = bool(return_home)
        self._escape_started = now
        self._escape_rotated = 0.0
        self._escape_steps = 0
        self._last_arm_time = now
        self._escape_yaw_ref = self.pose[2] if self.pose else None
        self._escape_budget = (self.return_home_max_rotation_rad
                               if return_home
                               else self.max_total_rotation_rad)
        # 2026-10-10 诊断：原文案把阈值写死成 LETHAL_OBSTACLE(254)，
        # 且预算打印 max_total_rotation_rad(2.10)，两者都可能与实际触发
        # 路径不符（wedge 走 253；返航预算只有 1.05）。此处打真实值。
        self._escape_path = str(path)
        self._armed_cost = float(cost)
        self._armed_threshold = float(
            self.wedge_cost_threshold
            if path in ('wedge', 'stall_override')
            else LETHAL_OBSTACLE)
        self.get_logger().warn(
            'ESCAPE ARMED [path=%s]: nav2 footprint test reports collision '
            '(cost=%.0f >= threshold=%.0f) while stalled at '
            '(%.2f, %.2f, yaw=%.0fdeg); rotating %s at %.2f rad/s, '
            'bounded to %.2f rad / %.1fs (actual budget; '
            'max_total=%.2f). Rear is unsensed: no reverse will be commanded.'
            % (self._escape_path, cost, self._armed_threshold,
               self.pose[0], self.pose[1],
               math.degrees(self.pose[2]),
               'CCW' if self._escape_direction > 0 else 'CW',
               self.rotate_speed_radps, self._escape_budget,
               self.max_duration_sec, self.max_total_rotation_rad))

    def _pick_direction(self):
        """Pick the rotation sense that clears the footprint soonest.

        Both senses are scored by the footprint collision state a small angle
        ahead; ties keep CCW.  The probe only reads the costmap.
        """
        probe = math.radians(10.0)
        x, y, yaw = self.pose
        ccw = self.footprint_cost(x, y, yaw + probe, self.footprint_robot)
        cw = self.footprint_cost(x, y, yaw - probe, self.footprint_robot)
        _pref = 1.0 if (ccw is None or cw is None or ccw <= cw) else -1.0
        # 2026-10-13 P12-#1：若此前在"同一位置同一方向"转满预算仍无改善，
        # 本次强制换相反方向 —— 避免在同一处反复做同样的无效旋转。
        if getattr(self, 'escape_flip_after_failed_budget', False):
            key = self._spot_key(x, y)
            mem = getattr(self, '_failed_dir_memory', {}).get(key)
            if mem is not None:
                _dir, _cnt = mem
                if _cnt < int(getattr(self, 'escape_flip_max_per_spot', 2)) \
                        and _pref == _dir:
                    self.get_logger().warn(
                        'P12 escape flip: direction %s already failed %d time(s) '
                        'at this spot; forcing %s instead'
                        % ('CCW' if _dir > 0 else 'CW', _cnt,
                           'CW' if _dir > 0 else 'CCW'))
                    return -1.0 * _dir
        # 2026-10-13 P12-#1 修正：无论是否走到上面的探测逻辑，
        # 只要"本点该方向失败过"就返回反向；否则返回探测结果。
        return self._flip_if_failed(x, y, _pref)

    def _flip_if_failed(self, x, y, direction):
        """P12-#1：若 (x,y) 点该方向曾转满预算仍无改善，返回相反方向。

        与 _pick_direction 内的判定同源，但可被 _start_escape 复用于
        "direction 固定"（默认 1.0）的场景 —— 那时代码不走 _pick_direction，
        若只把 flip 写在 _pick_direction 内部就永远不会执行。
        """
        try:
            if not getattr(self, 'escape_flip_after_failed_budget', False):
                return direction
            if x is None or y is None:
                return direction
            mem = getattr(self, '_failed_dir_memory', {}).get(self._spot_key(x, y))
            if mem is None:
                return direction
            _dir, _cnt = mem
            if _cnt >= int(getattr(self, 'escape_flip_max_per_spot', 2)):
                return direction
            _cur = 1.0 if direction >= 0.0 else -1.0
            if _cur != _dir:
                return direction
            self.get_logger().warn(
                'P12 escape flip: direction %s already failed %d time(s) at '
                'this spot; forcing %s instead'
                % ('CCW' if _dir > 0 else 'CW', _cnt,
                   'CW' if _dir > 0 else 'CCW'))
            return -1.0 * _dir
        except Exception:  # noqa: BLE001
            return direction

    def _spot_key(self, x, y):
        """P12-#1：把位置按 spot_radius 归并成格点键，用于失败方向记忆。"""
        try:
            r = max(0.05, float(getattr(self, 'escape_flip_spot_radius_m', 0.30)))
            return (int(round(float(x) / r)), int(round(float(y) / r)))
        except Exception:  # noqa: BLE001
            return (0, 0)

    def _remember_failed_direction(self, x, y, direction):
        """P12-#1：记下"该位置该方向转满预算仍无改善"。"""
        try:
            key = self._spot_key(x, y)
            mem = self._failed_dir_memory.get(key)
            if mem is not None and mem[0] == direction:
                self._failed_dir_memory[key] = (direction, mem[1] + 1)
            else:
                self._failed_dir_memory[key] = (direction, 1)
        except Exception:  # noqa: BLE001
            pass

    def _step(self, now):
        elapsed = now - self._escape_started
        if elapsed > self.max_duration_sec:
            self._finish('timeout', now)
            return
        budget = getattr(self, '_escape_budget', self.max_total_rotation_rad)
        if self._escape_rotated >= budget:
            # 2026-10-13 P12-#1：转满预算仍未改善 -> 记住该方向失败，
            # 下次在同一位置强制反向（见 _pick_direction）。
            if getattr(self, 'escape_flip_after_failed_budget', False):
                _cost_now = self.current_footprint_cost()
                _no_gain = (_cost_now is None
                            or self._armed_cost is None
                            or float(_cost_now) >= float(self._armed_cost))
                if _no_gain and self.pose is not None:
                    self._remember_failed_direction(
                        self.pose[0], self.pose[1], self._escape_direction)
                    self.get_logger().warn(
                        'P12 escape ineffective: rotated %.2f rad at (%.2f, %.2f) '
                        'with no footprint-cost gain (%.0f -> %.0f); '
                        'remembered direction %s as failed for this spot'
                        % (self._escape_rotated, self.pose[0], self.pose[1],
                           float(self._armed_cost) if self._armed_cost is not None else -1.0,
                           float(_cost_now) if _cost_now is not None else -1.0,
                           'CCW' if self._escape_direction > 0 else 'CW'))
            self._finish('rotation_budget_exhausted', now)
            return
        if (self.front_abort_enabled and self.front_range is not None
                and (now - self.front_stamp) <= 1.0
                and self.front_range < self.front_abort_m):
            self._finish('front_abort_%.3fm' % self.front_range, now)
            return

        cost = self.current_footprint_cost()
        if cost is None:
            self._finish('lost_inputs', now)
            return
        # 2026-10-12 E-2：先判断"代价是否真的改善"。
        # 不满足就继续转（受 budget / timeout / front_abort 约束），
        # 避免 armed=253 -> release=253 这种"毫无进展就撒手"的循环。
        _cost_improved = True
        if (self.escape_require_cost_improvement
                and self._armed_cost is not None):
            _cost_improved = float(cost) < float(self._armed_cost)
        if (self._escape_steps >= self.min_steps_before_exit
                and cost < float(LETHAL_OBSTACLE)
                and _cost_improved):
            # 返航阶段：释放前必须回到参考朝向附近（方案 b 的硬约束），
            # 否则不能就此撒手，继续转到预算用尽。
            if self._return_home_escape and self._escape_yaw_ref is not None:
                off = abs(math.atan2(
                    math.sin(self.pose[2] - self._escape_yaw_ref),
                    math.cos(self.pose[2] - self._escape_yaw_ref)))
                if off > self.return_home_heading_tolerance_rad:
                    self.get_logger().info(
                        'return-home escape: footprint clear but heading '
                        'offset %.0f deg > %.0f deg; continuing to rotate '
                        'back towards the original heading'
                        % (math.degrees(off),
                           math.degrees(
                               self.return_home_heading_tolerance_rad)))
                else:
                    self._finish(
                        'nav2_footprint_free_cost_%.0f_heading_ok' % cost,
                        now)
                    return
            else:
                self._finish(
                    'nav2_footprint_free_cost_%.0f_improved_from_%.0f'
                    % (cost, self._armed_cost
                       if self._armed_cost is not None else -1), now)
                return

        cmd = Twist()
        cmd.angular.z = self._escape_direction * self.rotate_speed_radps
        self.cmd_pub.publish(cmd)
        self._last_cmd_vel = cmd.angular.z
        self._escape_rotated += (
            self.rotate_speed_radps * self.command_period_sec)
        self._escape_steps += 1

        if self._escape_steps % self.log_every_n_steps == 0:
            self.get_logger().info(
                'escaping: step=%d rotated=%.2f/%.2f rad elapsed=%.1f/%.1fs '
                'footprint_cost=%.0f yaw=%.0fdeg'
                % (self._escape_steps, self._escape_rotated,
                   self.max_total_rotation_rad, elapsed,
                   self.max_duration_sec, cost, math.degrees(self.pose[2])))

    def _finish(self, reason, now):
        self.cmd_pub.publish(Twist())
        self.cmd_pub.publish(Twist())
        self._escaping = False
        self._last_cmd_vel = 0.0
        self._stalled_since = None
        self._stall_anchor = None
        self._last_arm_time = now       # cooldown starts from release
        logger = self.get_logger()
        self._release_cost = self.current_footprint_cost()
        # 2026-10-10 诊断：补上触发路径、触发代价、释放代价、实际预算，
        # 用于判定"释放是否等于局面真的解开"。
        text = ('ESCAPE RELEASED (%s) [path=%s armed_cost=%s -> release_cost=%s '
                'budget=%.2frad]: rotated %.2f rad over %d steps in '
                '%.1fs; zero velocity published, handing control back to Nav2'
                % (reason, self._escape_path or '?',
                   ('%.0f' % self._armed_cost)
                   if self._armed_cost is not None else 'n/a',
                   ('%.0f' % self._release_cost)
                   if self._release_cost is not None else 'n/a',
                   self._escape_budget, self._escape_rotated,
                   self._escape_steps, now - self._escape_started))
        if reason.startswith(('timeout', 'rotation_budget', 'lost_inputs',
                              'front_abort', 'mapping_phase_ended')):
            logger.warn(text)
        else:
            logger.info(text)

    def destroy_node(self):
        # Only emit a stop if we actually hold the controls.  Publishing a
        # zero Twist unconditionally would poke Nav2 for one control cycle
        # when this node is torn down during, say, a return-home.
        if self._escaping:
            try:
                self.cmd_pub.publish(Twist())
            except Exception:  # noqa: BLE001 - shutdown best effort
                pass
            self._escaping = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EscapeController()
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
