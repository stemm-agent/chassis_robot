#!/usr/bin/env python3
import json
import math
import os
import random
import signal
import subprocess
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class StemmNav2Manager(Node):
    def __init__(self):
        super().__init__('stemm_nav2_manager')

        self.declare_parameter('map_save_path', '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_timeout_sec', 30.0)
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('goal_topic', '/stemm/nav_goal')
        self.declare_parameter('frontier_goal_topic', '/assigned_frontier_goal')
        self.declare_parameter('auto_explore', False)
        self.declare_parameter('min_explore_distance', 0.8)
        self.declare_parameter('max_explore_distance', 2.5)
        self.declare_parameter('front_stop_distance', 0.40)
        self.declare_parameter('goal_clearance', 0.38)
        self.declare_parameter('front_emergency_distance', 0.30)
        self.declare_parameter('auto_explore_period_sec', 1.0)
        self.declare_parameter('explore_startup_delay_sec', 8.0)
        self.declare_parameter('return_home_quiet_sec', 2.0)
        self.declare_parameter('return_home_tf_max_age_sec', 0.20)
        self.declare_parameter('return_home_tf_stable_sec', 1.5)
        self.declare_parameter('return_home_max_retries', 3)
        self.declare_parameter('return_home_retry_delay_sec', 2.0)
        self.declare_parameter('return_home_start_clearance', 0.22)
        self.declare_parameter('home_known_radius', 0.45)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)

        self.map_save_path = self.get_parameter('map_save_path').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_timeout_sec = self.get_parameter('map_timeout_sec').get_parameter_value().double_value
        self.navigate_action = self.get_parameter('navigate_action').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.frontier_goal_topic = self.get_parameter('frontier_goal_topic').get_parameter_value().string_value
        self.auto_explore = self.get_parameter('auto_explore').get_parameter_value().bool_value
        self.min_explore_distance = self.get_parameter('min_explore_distance').get_parameter_value().double_value
        self.max_explore_distance = self.get_parameter('max_explore_distance').get_parameter_value().double_value
        self.front_stop_distance = self.get_parameter('front_stop_distance').get_parameter_value().double_value
        self.goal_clearance = self.get_parameter('goal_clearance').get_parameter_value().double_value
        self.front_emergency_distance = self.get_parameter('front_emergency_distance').get_parameter_value().double_value
        self.auto_explore_period_sec = self.get_parameter('auto_explore_period_sec').get_parameter_value().double_value
        self.explore_startup_delay_sec = self.get_parameter('explore_startup_delay_sec').get_parameter_value().double_value
        self.return_home_quiet_sec = self.get_parameter('return_home_quiet_sec').get_parameter_value().double_value
        self.return_home_tf_max_age_sec = self.get_parameter('return_home_tf_max_age_sec').get_parameter_value().double_value
        self.return_home_tf_stable_sec = self.get_parameter('return_home_tf_stable_sec').get_parameter_value().double_value
        self.return_home_max_retries = self.get_parameter('return_home_max_retries').get_parameter_value().integer_value
        self.return_home_retry_delay_sec = self.get_parameter('return_home_retry_delay_sec').get_parameter_value().double_value
        self.return_home_start_clearance = self.get_parameter('return_home_start_clearance').get_parameter_value().double_value
        self.home_known_radius = self.get_parameter('home_known_radius').get_parameter_value().double_value
        self.home_x = self.get_parameter('home_x').get_parameter_value().double_value
        self.home_y = self.get_parameter('home_y').get_parameter_value().double_value
        self.home_yaw = self.get_parameter('home_yaw').get_parameter_value().double_value

        self.last_map_time = None
        self.started_at = time.monotonic()
        self.last_explore_start_wait_log = 0.0
        self.last_map = None
        self.last_map_info = None
        self.last_pose = None
        self.mode = 'idle'
        self.navigation_state = 'idle'
        self.last_error = ''
        self.last_goal = None
        self.current_goal_handle = None
        self.goal_send_future = None
        self.goal_result_future = None
        self.nav_goal_active = False
        self.failed_goals = []
        self.blocked_places = []
        self.failed_goal_ttl = 240.0
        self.failed_goal_radius = 1.5
        self.blocked_place_radius = 1.5
        self.no_progress_timeout = 12.0
        self.no_progress_distance = 0.03
        self.no_progress_yaw = 0.12
        self.last_explore_heading = None
        self.progress_x = None
        self.progress_y = None
        self.progress_yaw = None
        self.progress_time = 0.0
        self.front_blocked = False
        self.front_min_range = None
        self.front_center_min_range = None
        self.front_wide_min_range = None
        self.last_safety_stop = 0.0
        self.front_blocked_count = 0
        self.last_cmd_vel = Twist()
        self.front_left_range = None
        self.front_right_range = None
        self.left_min_range = None
        self.right_min_range = None
        self.rear_min_range = None
        self.recovery_until = 0.0
        self.forward_escape_duration = 1.8
        self.reverse_escape_duration = 0.9
        self.reverse_escape_speed = -0.05
        self.recovery_started_at = 0.0
        self.recovery_clear_count = 0
        self.recovery_max_duration = 10.0
        self.recovery_twist = Twist()
        self.recovery_turn_direction = 0.0
        # Stuck recovery escalation: count consecutive failed recovery cycles
        self.stuck_recovery_cycles = 0
        self.last_stuck_clear_time = 0.0
        self.nav_lifecycle_nodes = (
            '/controller_server',
            '/planner_server',
            '/bt_navigator',
            '/waypoint_follower',
        )
        self.nav_lifecycle_clients = {}
        self.nav_lifecycle_futures = {}
        self.nav_lifecycle_states = {}
        self.nav2_ready = False
        self.last_nav2_wait_log = 0.0
        self.base_charging = False
        self.last_charging_log = 0.0
        self.mapping_done = False
        self.pending_home_goal = False
        self.return_home_requested = False
        self.shutdown_after_return_home = False
        self.shutdown_started = False
        self.return_home_ready_after = 0.0
        self.return_home_tf_stable_since = 0.0
        self.last_return_home_wait_log = 0.0
        self.return_home_retry_count = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)
        for node_name in self.nav_lifecycle_nodes:
            self.nav_lifecycle_clients[node_name] = self.create_client(GetState, f'{node_name}/get_state')
            self.nav_lifecycle_futures[node_name] = None

        self.state_pub = self.create_publisher(String, '/stemm/nav_state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/stemm/pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(PoseStamped, self.frontier_goal_topic, self.frontier_goal_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/robot_charging_flag', self.charging_callback, 10)

        self.create_service(Trigger, '/stemm/start_mapping', self.start_mapping_callback)
        self.create_service(Trigger, '/stemm/save_map', self.save_map_callback)
        self.create_service(Trigger, '/stemm/stop_navigation', self.stop_navigation_callback)
        self.create_service(SetBool, '/stemm/set_mapping_done', self.set_mapping_done_callback)

        self.create_timer(0.5, self.nav2_lifecycle_callback)
        self.create_timer(1.0, self.publish_state)
        self.create_timer(0.2, self.safety_stop_callback)
        self.create_timer(1.0, self.progress_watchdog_callback)
        self.create_timer(0.1, self.recovery_turn_callback)
        self.create_timer(self.auto_explore_period_sec, self.auto_explore_callback)
        self.get_logger().info('STEMM Nav2 manager ready')

    def map_callback(self, msg):
        self.last_map_time = time.monotonic()
        self.last_map = msg
        self.last_map_info = msg.info

    def scan_callback(self, msg):
        front_ranges = []
        front_center_ranges = []
        front_wide_ranges = []
        left_ranges = []
        right_ranges = []
        side_left_ranges = []
        side_right_ranges = []
        rear_ranges = []
        angle = msg.angle_min
        for value in msg.ranges:
            if math.isfinite(value):
                if -0.55 <= angle <= 0.55:
                    front_wide_ranges.append(value)
                    if angle >= 0.15:
                        left_ranges.append(value)
                    elif angle <= -0.15:
                        right_ranges.append(value)
                if -0.35 <= angle <= 0.35:
                    front_ranges.append(value)
                if -0.20 <= angle <= 0.20:
                    front_center_ranges.append(value)
                if 0.55 < angle <= 1.8:
                    side_left_ranges.append(value)
                elif -1.8 <= angle < -0.55:
                    side_right_ranges.append(value)
                elif abs(angle) >= 2.6:
                    rear_ranges.append(value)
            angle += msg.angle_increment

        if not front_ranges:
            self.front_blocked = False
            self.front_min_range = None
            self.front_center_min_range = None
            self.front_wide_min_range = min(front_wide_ranges) if front_wide_ranges else None
            self.front_left_range = None
            self.front_right_range = None
            self.left_min_range = min(side_left_ranges) if side_left_ranges else None
            self.right_min_range = min(side_right_ranges) if side_right_ranges else None
            self.rear_min_range = min(rear_ranges) if rear_ranges else None
            self.recovery_turn_direction = 0.0
            return

        self.front_min_range = min(front_ranges)
        self.front_center_min_range = min(front_center_ranges) if front_center_ranges else self.front_min_range
        self.front_wide_min_range = min(front_wide_ranges) if front_wide_ranges else self.front_min_range
        self.front_left_range = min(left_ranges) if left_ranges else self.front_min_range
        self.front_right_range = min(right_ranges) if right_ranges else self.front_min_range
        self.left_min_range = min(side_left_ranges) if side_left_ranges else None
        self.right_min_range = min(side_right_ranges) if side_right_ranges else None
        self.rear_min_range = min(rear_ranges) if rear_ranges else None
        if self.front_min_range < self.front_emergency_distance:
            self.front_blocked_count = 3
        elif self.front_min_range < self.front_stop_distance:
            self.front_blocked_count += 1
        else:
            self.front_blocked_count = 0
        self.front_blocked = self.front_blocked_count >= 5

    def safety_stop_callback(self):
        try:
            self._safety_stop_callback_impl()
        except Exception as e:
            self.get_logger().error(f'safety_stop_callback crashed: {e}', exc_info=True)

    def _safety_stop_callback_impl(self):
        if not self.front_blocked:
            self.last_stuck_clear_time = time.monotonic()
            self.stuck_recovery_cycles = 0
            if (self.mode == 'safety_stop'
                    and self.navigation_state == 'blocked_waiting_clearance'
                    and not self.nav_goal_active
                    and not self.is_recovering()):
                self.mode = 'mapping' if self.auto_explore else 'idle'
                self.navigation_state = 'clearance_restored'
                self.last_error = ''
            return

        self.remember_blocked_place()
        now = time.monotonic()
        # --- 关键修复：当Nav2 BT正在导航（含恢复Spin/Backup）时，信任BT自己处理，不抢控---
        # 之前这里直接 cancel_goal_async() 杀死了BT，导致BT的Spin90°恢复从未执行，
        # 而管理器的弱恢复（转69°）对半封闭区域的墙无效，形成死循环。
        if self.current_goal_handle is not None and self.nav_goal_active:
            # 记录日志但不取消 — BT正在用自己的恢复策略（Spin 90°）脱困
            self.last_error = f'front obstacle too close: {self.front_min_range:.2f}m'
            if now - self.last_safety_stop > 2.0:
                self.get_logger().warn(
                    f'front blocked ({self.front_min_range:.2f}m) but Nav2 BT is active — trusting BT recovery'
                )
                self.last_safety_stop = now
            return

        # 没有活跃导航目标时，管理器自主逃逸
        self.mode = 'safety_stop'
        self.last_error = f'front obstacle too close: {self.front_min_range:.2f}m'
        if self.is_recovering():
            return

        self.start_escape_recovery()
        if not self.is_recovering():
            self.publish_zero_velocity()
            self.navigation_state = 'blocked_waiting_clearance'

        if now - self.last_safety_stop > 2.0:
            self.get_logger().error(self.last_error)
            self.last_safety_stop = now

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def charging_callback(self, msg):
        self.base_charging = bool(msg.data)
        if self.base_charging:
            self.pause_for_charging()

    def pause_for_charging(self):
        if self.current_goal_handle is not None and self.nav_goal_active:
            self.current_goal_handle.cancel_goal_async()
        self.publish_zero_velocity()
        self.recovery_until = 0.0
        self.recovery_started_at = 0.0
        self.recovery_clear_count = 0
        self.recovery_turn_direction = 0.0
        self.mode = 'paused'
        self.navigation_state = 'base_charging'
        self.last_error = 'base is charging; navigation paused'
        now = time.monotonic()
        if now - self.last_charging_log > 5.0:
            self.get_logger().warn(self.last_error)
            self.last_charging_log = now

    def is_recovering(self):
        return self.recovery_until > 0.0

    def is_recovery_state(self):
        return self.navigation_state in (
            'forward_escaping_from_stuck_place',
            'reverse_escaping_from_stuck_place',
            'turning_out_of_blocked_place',
            'turning_out_of_stuck_place',
        )

    def finish_recovery(self, state, error=''):
        self.recovery_until = 0.0
        self.recovery_started_at = 0.0
        self.recovery_clear_count = 0
        self.recovery_twist = Twist()
        # DO NOT reset recovery_turn_direction — keep it so alternating logic works
        self.publish_zero_velocity()
        self.mode = 'mapping' if self.auto_explore and not error else 'safety_stop' if error else 'idle'
        self.navigation_state = state
        self.last_error = error
        # Escalation counter is now incremented in start_escape_recovery()
        if state == 'blocked_waiting_clearance':
            self.get_logger().warn(
                f'stuck recovery cycle {self.stuck_recovery_cycles} blocked, will escalate next attempt')
        if state == 'give_up_stuck':
            self.get_logger().error(
                f'GAVE UP after {self.stuck_recovery_cycles} recovery cycles; robot is truly stuck — waiting for human help')

    def nav2_lifecycle_callback(self):
        try:
            self._nav2_lifecycle_callback_impl()
        except Exception as e:
            self.get_logger().error(f'nav2_lifecycle_callback crashed: {e}', exc_info=True)

    def _nav2_lifecycle_callback_impl(self):
        for node_name, client in self.nav_lifecycle_clients.items():
            future = self.nav_lifecycle_futures.get(node_name)
            if future is not None:
                if not future.done():
                    continue
                try:
                    self.nav_lifecycle_states[node_name] = future.result().current_state.id
                except Exception:
                    self.nav_lifecycle_states[node_name] = None
                self.nav_lifecycle_futures[node_name] = None

            if self.nav_lifecycle_futures.get(node_name) is None and client.service_is_ready():
                self.nav_lifecycle_futures[node_name] = client.call_async(GetState.Request())

        self.nav2_ready = all(
            self.nav_lifecycle_states.get(node_name) == State.PRIMARY_STATE_ACTIVE
            for node_name in self.nav_lifecycle_nodes
        )

    def nav2_is_ready(self):
        return self.nav2_ready and self.nav_client.server_is_ready()

    def report_nav2_not_ready(self):
        self.mode = 'mapping' if self.auto_explore else 'idle'
        self.navigation_state = 'waiting_for_nav2'
        self.last_error = 'Nav2 lifecycle is not active yet'
        now = time.monotonic()
        if now - self.last_nav2_wait_log > 3.0:
            self.get_logger().warn(self.last_error)
            self.last_nav2_wait_log = now

    def recovery_turn_callback(self):
        try:
            self._recovery_turn_callback_impl()
        except Exception as e:
            self.get_logger().error(f'recovery_turn_callback crashed: {e}', exc_info=True)
            # Safety: zero velocity if recovery callback crashes
            try:
                self.publish_zero_velocity()
            except Exception:
                pass

    def _recovery_turn_callback_impl(self):
        if not self.is_recovering():
            return
        if self.base_charging:
            self.pause_for_charging()
            return

        now = time.monotonic()
        if self.recovery_started_at > 0.0 and now - self.recovery_started_at > self.recovery_max_duration:
            self.finish_recovery('blocked_waiting_clearance', 'recovery timed out; waiting for clearance')
            return

        if self.navigation_state == 'forward_escaping_from_stuck_place':
            front_center = self.front_center_min_range if self.front_center_min_range is not None else self.front_min_range
            front_margin = self.front_stop_distance + 0.05
            if front_center is None or front_center < front_margin:
                self.finish_recovery('blocked_waiting_clearance', 'forward escape blocked; waiting for clearance')
                return
            self.cmd_vel_pub.publish(self.recovery_twist)
            if now >= self.recovery_until:
                self.finish_recovery('recovery_clear')
            return

        if self.navigation_state == 'reverse_escaping_from_stuck_place':
            rear_margin = 0.22
            if self.rear_min_range is not None and self.rear_min_range < rear_margin:
                self.finish_recovery('blocked_waiting_clearance', 'reverse escape blocked; waiting for clearance')
                return
            self.cmd_vel_pub.publish(self.recovery_twist)
            if now >= self.recovery_until:
                self.finish_recovery('recovery_clear')
            return

        # Pure time-based exit for turning — front sensor cannot reliably
        # detect clearance mid-rotation because the lidar sweeps past
        # obstacles; after the full turn duration the robot re-evaluates.
        self.cmd_vel_pub.publish(self.recovery_twist)
        if now >= self.recovery_until:
            self.finish_recovery('recovery_clear')
            return

    def start_escape_recovery(self):
        if self.is_recovering():
            return

        # Hard cap: give up after 12 consecutive recovery cycles
        if self.stuck_recovery_cycles >= 10:
            self.finish_recovery('give_up_stuck',
                f'robot stuck after {self.stuck_recovery_cycles} recovery attempts; pausing for 60s')
            self.get_logger().error(self.last_error)
            return

        now = time.monotonic()
        self.recovery_started_at = now
        self.recovery_clear_count = 0
        
        # Count every recovery trigger (reset when front clears in safety_stop_callback)
        self.stuck_recovery_cycles += 1

        # Escalate: if stuck for many cycles, try more aggressive strategies
        front_open = self.front_center_min_range is not None and self.front_center_min_range >= 0.75
        rear_open = self.rear_min_range is None or self.rear_min_range >= 0.32
        left_range = self.left_min_range or 0.0
        right_range = self.right_min_range or 0.0
        left_open = left_range >= self.front_stop_distance + 0.10
        right_open = right_range >= self.front_stop_distance + 0.10
        # Level 3+: keep alternating aggressive turns until a path opens.
        if self.stuck_recovery_cycles >= 3 and not front_open and not left_open and not right_open:
            # Flip direction each cycle to sweep both sides without reversing.
            if self.recovery_turn_direction == 0.0:
                self.recovery_turn_direction = 1.0 if left_range >= right_range else -1.0
            elif self.stuck_recovery_cycles % 2 == 0:
                self.recovery_turn_direction *= -1.0
            self.recovery_twist = Twist()
            self.recovery_twist.angular.z = 1.10 * self.recovery_turn_direction
            self.recovery_until = now + 3.2
            self.navigation_state = 'turning_out_of_stuck_place'
            self.get_logger().warn(
                f'stuck escalation lv{self.stuck_recovery_cycles}: aggressive turn 3.2s @ 1.10rad/s '
                f'(dir={self.recovery_turn_direction})')
            return

        if front_open:
            self.recovery_twist = Twist()
            self.recovery_twist.linear.x = 0.05
            self.recovery_until = now + self.forward_escape_duration
            self.navigation_state = 'forward_escaping_from_stuck_place'
            return

        # A short, very slow reverse helps clear narrow dead-ends without letting
        # normal navigation plan long reverse segments.
        if rear_open and self.stuck_recovery_cycles >= 2:
            self.recovery_twist = Twist()
            self.recovery_twist.linear.x = self.reverse_escape_speed
            self.recovery_until = now + self.reverse_escape_duration
            self.navigation_state = 'reverse_escaping_from_stuck_place'
            self.get_logger().warn(
                f'stuck escalation lv{self.stuck_recovery_cycles}: reverse {self.reverse_escape_duration:.1f}s '
                f'@ {abs(self.reverse_escape_speed):.2f}m/s')
            return

        turn_clearance = self.front_stop_distance + 0.10
        left_clear = left_range >= turn_clearance
        right_clear = right_range >= turn_clearance
        if not left_clear and not right_clear:
            # Side clearances are both tight, so fall through to an in-place turn.
            pass

        if self.recovery_turn_direction == 0.0:
            self.recovery_turn_direction = 1.0 if left_range >= right_range else -1.0
        elif self.recovery_turn_direction > 0.0 and not left_clear and right_clear:
            self.recovery_turn_direction = -1.0
        elif self.recovery_turn_direction < 0.0 and not right_clear and left_clear:
            self.recovery_turn_direction = 1.0

        # Rotate long enough to sweep decisively away from the obstacle.
        # lv1: 2.8s × 0.85rad/s ≈ 136°;
        # lv2+: 3.6s × 0.95rad/s ≈ 196° for repeated stuck situations.
        turn_duration = 3.6 if self.stuck_recovery_cycles >= 2 else 2.8
        turn_speed = 0.95 if self.stuck_recovery_cycles >= 2 else 0.85
        self.recovery_twist = Twist()
        self.recovery_twist.angular.z = turn_speed * self.recovery_turn_direction
        self.recovery_until = now + turn_duration
        self.navigation_state = 'turning_out_of_blocked_place'
        if self.stuck_recovery_cycles >= 2:
            self.get_logger().warn(
            f'stuck escalation lv{self.stuck_recovery_cycles}: turning {turn_duration}s at {turn_speed:.2f}rad/s')

    def goal_callback(self, msg):
        if self.mapping_done:
            self.get_logger().info('ignoring manual nav goal while mapping_done is active')
            return
        self.send_nav_goal(msg, validate_map=True)

    def frontier_goal_callback(self, msg):
        if self.mapping_done:
            return
        if self.nav_goal_active or self.front_blocked:
            return
        current_pose = self.update_pose()
        if current_pose is None:
            return
        if self.pose_distance(msg, current_pose) < self.min_explore_distance:
            return
        if self.goal_recently_failed(msg.pose.position.x, msg.pose.position.y):
            return
        traversable, _ = self.goal_is_traversable(msg)
        if not traversable:
            self.failed_goals.append((msg.pose.position.x, msg.pose.position.y, time.monotonic()))
            return
        self.send_nav_goal(msg, validate_map=False)

    def start_mapping_callback(self, request, response):
        del request
        self.mapping_done = False
        self.pending_home_goal = False
        self.return_home_requested = False
        self.shutdown_after_return_home = False
        self.return_home_retry_count = 0
        self.auto_explore = True
        self.mode = 'mapping'
        if self.map_is_fresh():
            response.success = True
            response.message = 'mapping is active; /map is being updated'
        else:
            response.success = False
            response.message = 'mapping stack is not ready; /map has no recent data'
        return response

    def save_map_callback(self, request, response):
        del request
        if not self.map_is_fresh():
            response.success = False
            response.message = 'cannot save map: /map has no recent data'
            return response

        map_path = os.path.expanduser(self.map_save_path)
        os.makedirs(os.path.dirname(map_path), exist_ok=True)
        # Cartographer publishes /map with TRANSIENT_LOCAL durability.  The
        # saver is launched on demand, so request the latched map explicitly
        # instead of waiting for a future map update that may not arrive.
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path,
            '--ros-args',
            '-p', 'map_subscribe_transient_local:=true',
            '-p', 'save_map_timeout:=10.0',
            '-p', 'free_thresh_default:=0.25',
            '-p', 'occupied_thresh_default:=0.65',
        ]
        self.get_logger().info(f'saving map to {map_path}')
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=45)
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = 'map_saver_cli timed out'
            return response
        except OSError as exc:
            response.success = False
            response.message = f'map_saver_cli failed to start: {exc}'
            return response

        output = (result.stdout + result.stderr).strip()
        if result.returncode == 0:
            self.mode = 'map_saved'
            response.success = True
            response.message = f'map saved: {map_path}.yaml'
            return response

        response.success = False
        response.message = f'map save failed: {output[-500:]}'
        return response

    def stop_navigation_callback(self, request, response):
        del request
        self.pending_home_goal = False
        self.return_home_requested = False
        self.return_home_retry_count = 0
        if self.current_goal_handle is not None and self.nav_goal_active:
            self.current_goal_handle.cancel_goal_async()
        self.publish_zero_velocity()
        self.recovery_until = 0.0
        self.recovery_started_at = 0.0
        self.recovery_clear_count = 0
        self.recovery_turn_direction = 0.0
        self.auto_explore = False
        self.mode = 'stopped'
        self.navigation_state = 'canceled'
        response.success = True
        response.message = 'navigation stopped and zero velocity published'
        return response

    def set_mapping_done_callback(self, request, response):
        requested_done = bool(request.data)
        return_home_in_progress = (
            self.pending_home_goal
            or self.return_home_requested
            or self.navigation_state == 'canceling_late_exploration_goal'
        )
        if requested_done and self.mapping_done and return_home_in_progress:
            response.success = True
            response.message = f'mapping already marked done; return-home state: {self.navigation_state}'
            return response

        self.mapping_done = requested_done
        if not self.mapping_done:
            self.pending_home_goal = False
            self.return_home_requested = False
            self.shutdown_after_return_home = False
            self.shutdown_started = False
            self.return_home_ready_after = 0.0
            self.return_home_tf_stable_since = 0.0
            self.return_home_retry_count = 0
            self.auto_explore = True
            self.mode = 'mapping'
            self.navigation_state = 'mapping_resumed'
            self.last_error = ''
            response.success = True
            response.message = 'mapping_done cleared; auto exploration resumed'
            return response

        self.auto_explore = False
        self.pending_home_goal = True
        self.return_home_requested = False
        self.shutdown_after_return_home = True
        self.return_home_ready_after = time.monotonic() + self.return_home_quiet_sec
        self.return_home_tf_stable_since = 0.0
        self.return_home_retry_count = 0
        self.mode = 'returning_home'
        self.navigation_state = 'mapping_done_return_home_pending'
        self.last_error = ''
        self.publish_zero_velocity()

        # Free CPU resources: kill RRT exploration nodes so Nav2 return-home
        # navigation gets full compute budget for smooth MPPI control (20Hz).
        self._kill_rrt_nodes()

        # A goal may still be waiting for Nav2's acceptance callback. In that
        # window there is no goal handle to cancel yet; goal_response_callback
        # will cancel the late accepted exploration goal before it can continue.
        if self.goal_send_future is not None and not self.goal_send_future.done():
            response.success = True
            response.message = 'mapping marked done; waiting to cancel pending exploration goal before returning home'
            return response

        if self.current_goal_handle is not None and self.nav_goal_active:
            self.current_goal_handle.cancel_goal_async()
            response.success = True
            response.message = 'mapping marked done; stopping and canceling current goal before returning home'
            return response

        started, message = self._dispatch_pending_home_goal()
        response.success = True
        response.message = message
        return response

    def send_nav_goal(self, pose, validate_map=True, check_path_density=True):
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
            self.navigation_state = 'blocked_waiting_clearance'
            self.last_error = f'front obstacle too close: {self.front_min_range:.2f}m'
            self.start_escape_recovery()
            self.get_logger().warn(self.last_error)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.frame_id = pose.header.frame_id or self.global_frame
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0

        self.last_goal = goal.pose
        self.last_error = ''

        if self.is_current_pose_goal(goal.pose):
            self.mode = 'arrived'
            self.navigation_state = 'succeeded'
            self.get_logger().info('navigation goal is current pose; marking as arrived')
            return

        if validate_map:
            traversable, reason = self.goal_is_traversable(
                goal.pose, check_path_density=check_path_density)
            if not traversable:
                self.mode = 'mapping' if self.auto_explore else 'error'
                self.navigation_state = 'goal_unreachable'
                self.last_error = reason
                self.remember_failed_goal()
                self.get_logger().warn(reason)
                return

        self.mode = 'navigating'
        self.navigation_state = 'sending_goal'
        self.goal_send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        self.goal_send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_send_future = None
        goal_handle = future.result()

        # set_mapping_done may arrive while an exploration goal is still in
        # send_goal_async(). Such a goal had no handle at service time, so it
        # must be canceled here as soon as Nav2 accepts it.
        if self.mapping_done and self.pending_home_goal and not self.return_home_requested:
            if goal_handle.accepted:
                self.current_goal_handle = goal_handle
                self.nav_goal_active = True
                self.goal_result_future = goal_handle.get_result_async()
                self.goal_result_future.add_done_callback(self.goal_result_callback)
                goal_handle.cancel_goal_async()
                self.publish_zero_velocity()
                self.mode = 'returning_home'
                self.navigation_state = 'canceling_late_exploration_goal'
                self.get_logger().warn(
                    'mapping done while exploration goal was being accepted; canceling late goal')
            else:
                self.get_logger().info(
                    'pending exploration goal was rejected after mapping done')
            return

        if self.is_recovering() or self.is_recovery_state():
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.mode = 'safety_stop'
            return
        if not goal_handle.accepted:
            self.mode = 'error'
            self.navigation_state = 'goal_rejected'
            self.last_error = 'navigation goal was rejected'
            self.remember_failed_goal()
            self.get_logger().error(self.last_error)
            if self.return_home_requested and self.shutdown_after_return_home:
                self._schedule_return_home_retry('return-home goal was rejected')
            return

        self.current_goal_handle = goal_handle
        self.nav_goal_active = True
        self.reset_progress_watchdog()
        self.navigation_state = 'executing'
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        del feedback_msg
        if not self.is_recovering() and not self.is_recovery_state():
            self.navigation_state = 'executing'

    def reset_progress_watchdog(self):
        pose = self.update_pose()
        self.progress_time = time.monotonic()
        if pose is None:
            self.progress_x = None
            self.progress_y = None
            self.progress_yaw = None
            return
        self.progress_x = pose.pose.position.x
        self.progress_y = pose.pose.position.y
        self.progress_yaw = self.pose_yaw(pose)

    def progress_watchdog_callback(self):
        try:
            self._progress_watchdog_callback_impl()
        except Exception as e:
            self.get_logger().error(f'progress_watchdog_callback crashed: {e}', exc_info=True)

    def _progress_watchdog_callback_impl(self):
        if self.base_charging:
            self.pause_for_charging()
            return
        if not self.nav_goal_active:
            return
        pose = self.update_pose()
        if pose is None:
            return

        now = time.monotonic()
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = self.pose_yaw(pose)
        if self.progress_x is None or self.progress_y is None or self.progress_yaw is None:
            self.progress_x = x
            self.progress_y = y
            self.progress_yaw = yaw
            self.progress_time = now
            return

        moved = math.hypot(x - self.progress_x, y - self.progress_y) >= self.no_progress_distance
        turned = self.angle_distance(yaw, self.progress_yaw) >= self.no_progress_yaw
        if moved or turned:
            self.progress_x = x
            self.progress_y = y
            self.progress_yaw = yaw
            self.progress_time = now
            return

        commanded_motion = (
            abs(self.last_cmd_vel.linear.x) >= 0.03
            or abs(self.last_cmd_vel.angular.z) >= 0.10
        )
        if not commanded_motion:
            # Nav2 may intentionally output zero while changing sub-states; do not
            # let the manager cancel the goal and create an extra stop/restart cycle.
            self.progress_time = now
            return
        timeout = self.no_progress_timeout

        if now - self.progress_time < timeout:
            return

        self.remember_failed_goal()
        self.remember_blocked_place()
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        self.publish_zero_velocity()
        self.mode = 'safety_stop'
        self.navigation_state = 'stuck_waiting_new_goal'
        self.last_error = f'navigation made no progress for {timeout:.0f}s'
        self.start_escape_recovery()
        self.get_logger().warn(
            f'{self.last_error}; last cmd_vel linear.x={self.last_cmd_vel.linear.x:.3f}, '
            f'angular.z={self.last_cmd_vel.angular.z:.3f}, commanded_motion={commanded_motion}')
        self.progress_time = now

    def goal_result_callback(self, future):
        was_return_home_goal = self.return_home_requested
        self.return_home_requested = False
        self.nav_goal_active = False
        self.current_goal_handle = None
        self.last_safety_stop = 0.0
        self.last_cmd_vel = Twist()
        self.progress_x = None
        self.progress_y = None
        self.progress_yaw = None
        self.progress_time = 0.0
        result = future.result()
        status = result.status
        goal_summary = self.describe_goal_tracking()
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Reset stuck counter on successful navigation
            self.stuck_recovery_cycles = 0
            # Record exploration heading for direction persistence
            if self.last_goal is not None and self.auto_explore:
                current = self.update_pose()
                if current is not None:
                    self.last_explore_heading = math.atan2(
                        self.last_goal.pose.position.y - current.pose.position.y,
                        self.last_goal.pose.position.x - current.pose.position.x)
            self.mode = 'arrived'
            self.navigation_state = 'succeeded'
            self.last_error = ''
            self.get_logger().info(f'navigation goal reached; {goal_summary}')
            if was_return_home_goal and self.shutdown_after_return_home:
                self.request_launch_shutdown('return-home goal reached')
            return
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'navigation goal canceled; {goal_summary}')
            if self.pending_home_goal:
                started, message = self._dispatch_pending_home_goal()
                if started:
                    self.get_logger().info(message)
                else:
                    self.get_logger().warn(message)
                return
            if was_return_home_goal and self.shutdown_after_return_home:
                self._schedule_return_home_retry('return-home goal was canceled')
                return
            if self.is_recovering() or self.is_recovery_state():
                self.mode = 'safety_stop'
                return
            if self.last_error:
                self.mode = 'mapping' if self.auto_explore else 'stopped'
                if self.navigation_state not in ('blocked_waiting_clearance', 'stuck_waiting_new_goal'):
                    self.navigation_state = 'canceled'
            else:
                self.mode = 'stopped'
                self.navigation_state = 'canceled'
            return
        else:
            self.mode = 'safety_stop'
            self.navigation_state = f'failed:{status}'
            self.last_error = f'navigation failed with status {status}; {goal_summary}'
            self.remember_failed_goal()
            if was_return_home_goal and self.shutdown_after_return_home:
                self.publish_zero_velocity()
                self.get_logger().error(self.last_error)
                self._schedule_return_home_retry(
                    f'return-home goal failed with status {status}')
                return
            self.start_escape_recovery()
            if self.is_recovering():
                self.get_logger().warn(f'{self.last_error}; trying controlled recovery')
            else:
                self.get_logger().error(self.last_error)

    def _schedule_return_home_retry(self, reason):
        self.return_home_retry_count += 1
        self.return_home_requested = False
        self.pending_home_goal = False
        self.nav_goal_active = False
        self.current_goal_handle = None

        if self.return_home_retry_count > self.return_home_max_retries:
            self.publish_zero_velocity()
            self.mode = 'stopped'
            self.navigation_state = 'return_home_failed_waiting_manual'
            self.last_error = (
                f'{reason}; return-home failed after {self.return_home_max_retries} retries, '
                'keeping launch alive for manual takeover')
            self.get_logger().error(self.last_error)
            return

        self.pending_home_goal = True
        self.return_home_ready_after = time.monotonic() + self.return_home_retry_delay_sec
        self.return_home_tf_stable_since = 0.0
        self.mode = 'safety_stop'
        self.navigation_state = 'return_home_retry_recovery'
        self.last_error = (
            f'{reason}; retrying return-home '
            f'{self.return_home_retry_count}/{self.return_home_max_retries}')
        self.get_logger().warn(self.last_error)
        self.start_escape_recovery()
        if not self.is_recovering():
            self.publish_zero_velocity()

    def _dispatch_pending_home_goal(self):
        if not self.pending_home_goal:
            return False, 'no pending home goal'
        if self.base_charging:
            return False, 'mapping done acknowledged; waiting for base charging to finish before returning home'
        if not self.nav2_is_ready():
            self.report_nav2_not_ready()
            return False, 'mapping done acknowledged; waiting for Nav2 to become ready before returning home'

        now = time.monotonic()
        if now < self.return_home_ready_after:
            self.publish_zero_velocity()
            self.navigation_state = 'return_home_waiting_settle'
            return False, 'mapping done acknowledged; waiting for robot and Nav2 to settle before returning home'

        # Require a short continuous window of fresh TF. One fresh sample can
        # still be followed by lag during SLAM/costmap load spikes.
        if not self._tf_is_stably_recent('odom_combined', self.global_frame,
                                         max_age=self.return_home_tf_max_age_sec,
                                         stable_sec=self.return_home_tf_stable_sec):
            self.publish_zero_velocity()
            self.navigation_state = 'return_home_waiting_tf'
            return False, (
                'mapping done acknowledged; '
                'waiting for stable odom->map TF before returning home')

        clear, reason = self.current_start_is_map_clear()
        if not clear:
            self._schedule_return_home_retry(reason)
            return False, reason

        home_goal = self.make_pose_goal(self.home_x, self.home_y, self.home_yaw)
        self.pending_home_goal = False
        self.return_home_requested = True
        # Temporary policy: the mapping start area is operator-approved. Do not
        # reject the home pose because the occupancy grid has an occupied cell
        # inside the normal exploration-goal clearance radius. Front-laser
        # emergency checks and Nav2's own planning checks remain enabled.
        # TODO: replace this with a dedicated home clearance/staging-pose rule.
        self.send_nav_goal(home_goal, validate_map=False, check_path_density=False)
        if self.nav_goal_active or self.navigation_state == 'sending_goal':
            self.mode = 'returning_home'
            self.navigation_state = 'returning_home'
            return True, 'mapping done; returning to origin'

        reason = self.last_error or 'mapping done acknowledged; failed to start return-home navigation'
        self._schedule_return_home_retry(reason)
        return False, reason

    def map_is_fresh(self):
        return self.last_map_time is not None and time.monotonic() - self.last_map_time <= self.map_timeout_sec

    def auto_explore_callback(self):
        try:
            self._auto_explore_callback_impl()
        except Exception as e:
            self.get_logger().error(f'auto_explore_callback crashed: {e}', exc_info=True)

    def _auto_explore_callback_impl(self):
        if self.base_charging:
            self.pause_for_charging()
            return
        if self.is_recovering() or self.is_recovery_state():
            return
        if self.pending_home_goal and not self.nav_goal_active:
            self._dispatch_pending_home_goal()
            return
        if not self.auto_explore or self.nav_goal_active or not self.map_is_fresh():
            return
        elapsed = time.monotonic() - self.started_at
        if elapsed < self.explore_startup_delay_sec:
            now = time.monotonic()
            if now - self.last_explore_start_wait_log > 2.0:
                self.get_logger().info(
                    f'waiting for startup map/costmap warm-up before auto exploration '
                    f'({elapsed:.1f}/{self.explore_startup_delay_sec:.1f}s)')
                self.last_explore_start_wait_log = now
            self.mode = 'mapping'
            self.navigation_state = 'warming_up'
            return
        if not self.nav2_is_ready():
            self.report_nav2_not_ready()
            return
        if self.front_blocked:
            self.mode = 'safety_stop'
            self.last_error = f'front obstacle too close: {self.front_min_range:.2f}m'
            self.start_escape_recovery()
            if not self.is_recovering():
                self.publish_zero_velocity()
                self.navigation_state = 'blocked_waiting_clearance'
            return
        if self.navigation_state in ('sending_goal', 'executing'):
            return

        goal = self.select_frontier_goal()
        if goal is None:
            # Fallback: if all frontiers blocked, pick a random free cell 2-4m away
            # to move the robot and discover new frontiers from a fresh vantage
            fallback = self._pick_fallback_goal()
            if fallback is not None:
                self.get_logger().info(
                    f'no frontiers available; fallback goal: {fallback.pose.position.x:.2f}, {fallback.pose.position.y:.2f}')
                self.send_nav_goal(fallback, validate_map=True)
                return
            self.mode = 'mapping'
            self.navigation_state = 'waiting_for_frontier'
            return

        self.get_logger().info(
            f'auto exploration goal: {goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}')
        self.send_nav_goal(goal, validate_map=True)

    def select_frontier_goal(self):
        if self.last_map is None:
            return None
        current_pose = self.update_pose()
        if current_pose is None:
            return None

        info = self.last_map.info
        data = self.last_map.data
        width = info.width
        height = info.height
        resolution = info.resolution
        origin = info.origin.position
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y

        # Dynamic sampling step: coarser on large maps to bound computation
        step = max(2, min(width, height) // 80)

        # Phase 1: collect ALL frontier candidates (no cap bias)
        candidates = []
        for my in range(2, height - 2, step):
            row = my * width
            for mx in range(2, width - 2, step):
                value = data[row + mx]
                if value < 0 or value >= 30:
                    continue

                unknown_neighbors = 0
                unknown_cells_near_goal = 0
                occupied_distance = None
                blocked = False
                clearance_cells = max(2, int(self.goal_clearance / resolution))
                for dy in range(-clearance_cells, clearance_cells + 1):
                    for dx in range(-clearance_cells, clearance_cells + 1):
                        nx = mx + dx
                        ny = my + dy
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        neighbor = data[ny * width + nx]
                        radius = math.hypot(dx * resolution, dy * resolution)
                        if neighbor < 0 and abs(dx) <= 2 and abs(dy) <= 2:
                            unknown_neighbors += 1
                        if neighbor < 0:
                            unknown_cells_near_goal += 1
                        if neighbor >= 50 and radius <= self.goal_clearance:
                            blocked = True
                            if occupied_distance is None or radius < occupied_distance:
                                occupied_distance = radius
                if blocked or unknown_neighbors == 0:
                    continue

                x = origin.x + (mx + 0.5) * resolution
                y = origin.y + (my + 0.5) * resolution
                distance = math.hypot(x - cx, y - cy)
                if distance < self.min_explore_distance or distance > self.max_explore_distance:
                    continue
                if self.goal_recently_failed(x, y):
                    continue

                distance_score = max(0.25, 1.0 - abs(distance - 2.8) * 0.20)
                unknown_boost = 1.0 + min(1.2, 0.035 * unknown_cells_near_goal)
                clearance_score = 1.0 if occupied_distance is None else min(1.0, occupied_distance / self.goal_clearance)
                blocked_ratio = self.path_blocked_ratio(cx, cy, x, y)
                if blocked_ratio >= 0.10:
                    self.failed_goals.append((x, y, time.monotonic()))
                    continue
                path_score = max(0.35, 1.0 - blocked_ratio * 5.0)
                score = unknown_neighbors * distance_score * unknown_boost * path_score * max(0.4, clearance_score)
                candidates.append((score, x, y, distance))

        if not candidates:
            return None

        # Phase 2: shuffle for fairness, then pick top-scoring
        random.shuffle(candidates)

        best_score = -1.0
        best = None
        for score, x, y, distance in candidates:
            # Direction persistence: prefer frontiers in same general heading
            if self.last_explore_heading is not None:
                heading = math.atan2(y - cy, x - cx)
                heading_diff = abs(math.atan2(math.sin(heading - self.last_explore_heading),
                                              math.cos(heading - self.last_explore_heading)))
                # Boost score for frontiers within +/-60 deg of last heading.
                if heading_diff < 1.05:  # ~60 degrees
                    score *= 1.5

            if score > best_score:
                best_score = score
                best = (x, y)

        if best is None:
            return None

        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp.sec = 0
        goal.header.stamp.nanosec = 0
        goal.pose.position.x = best[0]
        goal.pose.position.y = best[1]
        yaw = math.atan2(best[1] - cy, best[0] - cx)
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def _pick_fallback_goal(self):
        """When all frontiers are blocked, pick a random free cell farther away
        to move the robot so SLAM can discover new frontiers."""
        current = self.update_pose()
        if current is None or self.last_map is None:
            return None
        info = self.last_map.info
        data = self.last_map.data
        width = info.width
        height = info.height
        resolution = info.resolution
        origin = info.origin.position
        cx = current.pose.position.x
        cy = current.pose.position.y

        # Collect free cells in a longer range, excluding blocked/failed places.
        # Short hops make Nav2 stop at each sub-goal, which looks like stuttering.
        fallback_cells = []
        step = max(2, min(width, height) // 80)
        for my in range(2, height - 2, step):
            row = my * width
            for mx in range(2, width - 2, step):
                value = data[row + mx]
                if value < 0 or value >= 30:
                    continue
                x = origin.x + (mx + 0.5) * resolution
                y = origin.y + (my + 0.5) * resolution
                d = math.hypot(x - cx, y - cy)
                if d < 2.5 or d > 5.5:
                    continue
                if self.goal_recently_failed(x, y):
                    continue
                fallback_cells.append((x, y, d))

        if not fallback_cells:
            # Try wider range if the preferred longer hop is unavailable.
            for my in range(2, height - 2, step):
                row = my * width
                for mx in range(2, width - 2, step):
                    value = data[row + mx]
                    if value < 0 or value >= 30:
                        continue
                    x = origin.x + (mx + 0.5) * resolution
                    y = origin.y + (my + 0.5) * resolution
                    d = math.hypot(x - cx, y - cy)
                    if d < 1.8 or d > 6.0:
                        continue
                    if self.goal_recently_failed(x, y):
                        continue
                    fallback_cells.append((x, y, d))

        if not fallback_cells:
            return None

        # Pick random direction-biased cell
        if self.last_explore_heading is not None:
            # Prefer cells roughly in the same heading
            weighted = []
            for x, y, d in fallback_cells:
                heading = math.atan2(y - cy, x - cx)
                diff = abs(math.atan2(math.sin(heading - self.last_explore_heading),
                                      math.cos(heading - self.last_explore_heading)))
                if diff < 1.05:
                    weighted.append((x, y))
            if weighted:
                choice = random.choice(weighted)
            else:
                choice = random.choice(fallback_cells)
        else:
            choice = random.choice(fallback_cells)

        return self.make_pose_goal(choice[0], choice[1], math.atan2(choice[1] - cy, choice[0] - cx))

    def make_pose_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp.sec = 0
        goal.header.stamp.nanosec = 0
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def pose_distance(self, pose, current_pose):
        dx = pose.pose.position.x - current_pose.pose.position.x
        dy = pose.pose.position.y - current_pose.pose.position.y
        return math.hypot(dx, dy)

    def pose_yaw(self, pose):
        q = pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def angle_distance(self, first, second):
        return abs(math.atan2(math.sin(first - second), math.cos(first - second)))

    def describe_goal_tracking(self):
        if self.last_goal is None:
            return 'goal=none'

        gx = self.last_goal.pose.position.x
        gy = self.last_goal.pose.position.y
        gyaw = self.pose_yaw(self.last_goal)
        cmd_linear = self.last_cmd_vel.linear.x
        cmd_angular = self.last_cmd_vel.angular.z

        current_pose = self.update_pose()
        if current_pose is None:
            return (
                f'goal=({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f}), '
                f'pose=unavailable, last_cmd_vel=({cmd_linear:.3f}, {cmd_angular:.3f})'
            )

        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y
        cyaw = self.pose_yaw(current_pose)
        xy_error = math.hypot(gx - cx, gy - cy)
        yaw_error = self.angle_distance(gyaw, cyaw)
        return (
            f'goal=({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f}), '
            f'current=({cx:.2f}, {cy:.2f}, yaw={cyaw:.2f}), '
            f'error=(xy={xy_error:.3f}m, yaw={yaw_error:.3f}rad), '
            f'last_cmd_vel=({cmd_linear:.3f}, {cmd_angular:.3f})'
        )

    def remember_failed_goal(self):
        if self.last_goal is None:
            return
        x = self.last_goal.pose.position.x
        y = self.last_goal.pose.position.y
        now = time.monotonic()
        for index, (gx, gy, _) in enumerate(self.failed_goals):
            if math.hypot(x - gx, y - gy) < 0.3:
                self.failed_goals[index] = (x, y, now)
                return
        self.failed_goals.append((x, y, now))

    def remember_blocked_place(self):
        pose = self.update_pose()
        if pose is None:
            return
        x = pose.pose.position.x
        y = pose.pose.position.y
        now = time.monotonic()
        for index, (bx, by, _) in enumerate(self.blocked_places):
            if math.hypot(x - bx, y - by) < 0.3:
                self.blocked_places[index] = (x, y, now)
                return
        self.blocked_places.append((x, y, now))

    def goal_recently_failed(self, x, y):
        now = time.monotonic()
        self.failed_goals = [goal for goal in self.failed_goals if now - goal[2] < self.failed_goal_ttl]
        self.blocked_places = [place for place in self.blocked_places if now - place[2] < self.failed_goal_ttl]
        near_failed_goal = any(math.hypot(x - gx, y - gy) < self.failed_goal_radius for gx, gy, _ in self.failed_goals)
        near_blocked_place = any(math.hypot(x - bx, y - by) < self.blocked_place_radius for bx, by, _ in self.blocked_places)
        return near_failed_goal or near_blocked_place

    def is_current_pose_goal(self, goal_pose):
        current_pose = self.update_pose()
        if current_pose is None:
            return False
        dx = goal_pose.pose.position.x - current_pose.pose.position.x
        dy = goal_pose.pose.position.y - current_pose.pose.position.y
        return math.hypot(dx, dy) <= 0.15

    def path_blocked_ratio(self, start_x, start_y, goal_x, goal_y):
        if self.last_map is None:
            return 0.0
        info = self.last_map.info
        resolution = info.resolution
        if resolution <= 0.0:
            return 0.0
        origin = info.origin.position
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        steps = max(1, int(distance / max(0.05, resolution)))
        blocked = 0
        checked = 0
        for step in range(1, steps + 1):
            ratio = step / steps
            x = start_x + (goal_x - start_x) * ratio
            y = start_y + (goal_y - start_y) * ratio
            mx = int((x - origin.x) / resolution)
            my = int((y - origin.y) / resolution)
            if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                continue
            checked += 1
            value = self.last_map.data[my * info.width + mx]
            if value >= 50:
                blocked += 1
        if checked == 0:
            return 0.0
        return blocked / checked

    def is_inside_home_known_area(self, x, y):
        return math.hypot(x - self.home_x, y - self.home_y) <= self.home_known_radius

    def current_start_is_map_clear(self):
        if self.last_map is None:
            return True, ''
        info = self.last_map.info
        resolution = info.resolution
        if resolution <= 0.0:
            return True, ''
        current = self.update_pose()
        if current is None:
            return True, ''

        origin = info.origin.position
        mx = int((current.pose.position.x - origin.x) / resolution)
        my = int((current.pose.position.y - origin.y) / resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False, 'return-home delayed: current pose is outside /map'

        clearance_cells = max(1, int(self.return_home_start_clearance / resolution))
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if math.hypot(dx * resolution, dy * resolution) > self.return_home_start_clearance:
                    continue
                nx = mx + dx
                ny = my + dy
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    continue
                if self.last_map.data[ny * info.width + nx] >= 50:
                    return False, 'return-home delayed: current pose is too close to occupied /map cells'
        return True, ''

    def goal_is_traversable(self, goal_pose, check_path_density=True):
        if self.last_map is None:
            return False, 'goal rejected: map is not available'

        info = self.last_map.info
        resolution = info.resolution
        origin = info.origin.position
        mx = int((goal_pose.pose.position.x - origin.x) / resolution)
        my = int((goal_pose.pose.position.y - origin.y) / resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False, 'goal rejected: target is outside the map'

        index = my * info.width + mx
        value = self.last_map.data[index]
        if value < 0:
            if not self.is_inside_home_known_area(goal_pose.pose.position.x, goal_pose.pose.position.y):
                return False, 'goal rejected: target is in unknown space'
            self.get_logger().info(
                'home goal cell is unknown in /map, accepting it inside startup known radius')
        if value >= 50:
            return False, 'goal rejected: target is occupied'

        current = self.update_pose()
        if check_path_density and current is not None:
            blocked_ratio = self.path_blocked_ratio(
                current.pose.position.x, current.pose.position.y,
                goal_pose.pose.position.x, goal_pose.pose.position.y)
            if blocked_ratio >= 0.10:
                return False, f'goal rejected: obstacle-dense path ({blocked_ratio:.0%} blocked cells)'

        clearance_cells = max(1, int(self.goal_clearance / resolution))
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                nx = mx + dx
                ny = my + dy
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    continue
                if math.hypot(dx * resolution, dy * resolution) > self.goal_clearance:
                    continue
                if self.last_map.data[ny * info.width + nx] >= 50:
                    return False, 'goal rejected: obstacle is too close to target'
        return True, ''

    def update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time())
        except TransformException as exc:
            self.last_error = f'pose unavailable: {exc}'
            return None

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.last_pose = pose
        self.pose_pub.publish(pose)
        return pose

    def _tf_is_recent(self, target_frame, source_frame, max_age=0.5):
        """Return True if the *latest* transform from source→target is
        no older than *max_age* seconds.  Used to gate navigation goal
        dispatch until slam_toolbox (or localization) is publishing
        timely transforms so the controller won't abort with
        "extrapolation into the future" errors.
        """
        try:
            # tf2::TimePointZero → get the most recent available transform
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'TF not yet available ({source_frame}→{target_frame}): {exc}')
            return False

        stamp = Time.from_msg(t.header.stamp, clock_type=self.get_clock().clock_type)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > max_age:
            self.get_logger().info(
                f'TF {source_frame}→{target_frame} is {age:.3f}s old '
                f'(max {max_age:.1f}s); waiting for a newer transform')
            return False
        return True

    def _tf_is_stably_recent(self, target_frame, source_frame, max_age, stable_sec):
        if not self._tf_is_recent(target_frame, source_frame, max_age=max_age):
            self.return_home_tf_stable_since = 0.0
            return False

        now = time.monotonic()
        if self.return_home_tf_stable_since <= 0.0:
            self.return_home_tf_stable_since = now
            return False

        stable_for = now - self.return_home_tf_stable_since
        if stable_for < stable_sec:
            if now - self.last_return_home_wait_log > 1.0:
                self.get_logger().info(
                    f'waiting for stable return-home TF ({stable_for:.1f}/{stable_sec:.1f}s)')
                self.last_return_home_wait_log = now
            return False
        return True

    def publish_state(self):
        try:
            self._publish_state_impl()
        except Exception as e:
            self.get_logger().error(f'publish_state crashed: {e}', exc_info=True)

        # Periodic memory monitoring (every 30s, in publish_state ≈ every 1s)
        now = time.monotonic()
        if not hasattr(self, '_last_mem_log_time'):
            self._last_mem_log_time = 0.0
        if now - self._last_mem_log_time > 30.0:
            self._last_mem_log_time = now
            try:
                self._log_memory_usage()
            except Exception:
                pass

    def _log_memory_usage(self):
        """Log current memory usage to help diagnose OOM crashes on Jetson"""
        try:
            with open('/proc/self/status', 'r') as f:
                for line in f:
                    if line.startswith('VmRSS:') or line.startswith('VmSize:'):
                        self.get_logger().info(f'memory: {line.strip()}')
        except Exception:
            pass

    def _publish_state_impl(self):
        pose = self.update_pose()
        payload = {
            'mode': self.mode,
            'navigation_state': self.navigation_state,
            'map_available': self.map_is_fresh(),
            'map_save_path': self.map_save_path,
            'pose_available': pose is not None,
            'pose': self.pose_to_dict(pose),
            'last_goal': self.pose_to_dict(self.last_goal),
            'last_error': self.last_error,
            'front_min_range': self.front_min_range,
            'front_center_min_range': self.front_center_min_range,
            'front_wide_min_range': self.front_wide_min_range,
            'front_blocked': self.front_blocked,
            'recovery_active': self.is_recovering(),
            'recovery_turn_direction': self.recovery_turn_direction,
            'recovery_clear_count': self.recovery_clear_count,
            'nav2_ready': self.nav2_ready,
            'nav2_lifecycle': self.nav_lifecycle_states,
            'base_charging': self.base_charging,
            'mapping_done': self.mapping_done,
            'pending_home_goal': self.pending_home_goal,
            'return_home_retry_count': self.return_home_retry_count,
            'left_min_range': self.left_min_range,
            'right_min_range': self.right_min_range,
            'rear_min_range': self.rear_min_range,
            'failed_goal_count': len(self.failed_goals),
            'blocked_place_count': len(self.blocked_places),
            'failed_goals': self.places_to_dicts(self.failed_goals, self.failed_goal_radius),
            'blocked_places': self.places_to_dicts(self.blocked_places, self.blocked_place_radius),
            'last_cmd_vel': self.twist_to_dict(self.last_cmd_vel),
            'no_progress_timeout': self.no_progress_timeout,
            'no_progress_distance': self.no_progress_distance,
            'no_progress_yaw': self.no_progress_yaw,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def places_to_dicts(self, places, radius):
        now = time.monotonic()
        return [
            {'x': x, 'y': y, 'age': now - timestamp, 'radius': radius}
            for x, y, timestamp in places
        ]

    def twist_to_dict(self, twist):
        return {
            'linear_x': twist.linear.x,
            'linear_y': twist.linear.y,
            'angular_z': twist.angular.z,
        }

    def pose_to_dict(self, pose):
        if pose is None:
            return None
        return {
            'frame_id': pose.header.frame_id,
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'z': pose.pose.position.z,
            'qx': pose.pose.orientation.x,
            'qy': pose.pose.orientation.y,
            'qz': pose.pose.orientation.z,
            'qw': pose.pose.orientation.w,
        }

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def _kill_rrt_nodes(self):
        """Kill RRT exploration nodes to free CPU for smooth Nav2 return-home.

        During auto-exploration the RRT chain (local_rrt / global_rrt / filter /
        assigner) runs alongside MPPI.  When mapping is marked done the RRT nodes
        are no longer needed but they keep consuming CPU — competing with MPPI's
        288 000 critic evaluations per 20 Hz control cycle.  On resource-
        constrained Jetson hardware this competition causes MPPI to occasionally
        miss its 50 ms deadline, producing physical stuttering.

        We send SIGTERM to the RRT executables so the launch continues running
        without them.  Resuming exploration (set_mapping_done=False) after this
        requires a full relaunch.
        """
        rrt_exes = ['local_rrt', 'global_rrt', 'filter', 'assigner']
        killed = 0
        for exe in rrt_exes:
            try:
                result = subprocess.run(
                    ['pgrep', '-x', exe],
                    capture_output=True, text=True, timeout=3)
                for pid_str in result.stdout.strip().split():
                    try:
                        os.kill(int(pid_str), signal.SIGTERM)
                        killed += 1
                        self.get_logger().info(
                            f'killed RRT node "{exe}" (pid {pid_str}) to free CPU for return-home')
                    except (OSError, ProcessLookupError):
                        pass
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        rrt_patterns = [
            'rrt_exploration.*(local_rrt|global_rrt|filter|assigner)',
            'wheeltec_robot_rrt.*(local_rrt|global_rrt|filter|assigner)',
        ]
        for pattern in rrt_patterns:
            try:
                result = subprocess.run(
                    ['pgrep', '-f', pattern],
                    capture_output=True, text=True, timeout=3)
                for pid_str in result.stdout.strip().split():
                    try:
                        pid = int(pid_str)
                        if pid == os.getpid():
                            continue
                        os.kill(pid, signal.SIGTERM)
                        killed += 1
                        self.get_logger().info(
                            f'killed RRT process matching "{pattern}" (pid {pid_str})')
                    except (ValueError, OSError, ProcessLookupError):
                        pass
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        if killed:
            self.get_logger().info(
                f'shut down {killed} RRT exploration process(es); '
                f'Nav2 return-home now has full CPU budget')
        else:
            self.get_logger().warn(
                'no RRT processes found to kill; they may already be stopped')

    def _before_launch_shutdown(self):
        """Allow derived managers to persist terminal state before SIGINT."""
        return

    def request_launch_shutdown(self, reason):
        if self.shutdown_started:
            return

        self.shutdown_started = True
        self.auto_explore = False
        self.pending_home_goal = False
        self.return_home_requested = False
        self.publish_zero_velocity()
        self.mode = 'shutdown'
        self.navigation_state = 'shutdown_requested'
        self.last_error = ''
        try:
            self._before_launch_shutdown()
        except Exception as exc:
            self.get_logger().error(
                f"failed to persist pre-shutdown state: {exc}")

        if os.environ.get('STEMM_AUTO_MAPPING_LAUNCH') == '1':
            parent_pid = os.getppid()
            self.get_logger().info(
                f'{reason}; sending SIGINT to launch process group via parent pid {parent_pid}')
            try:
                # Brutal: send SIGINT to the entire process group (like ^C)
                os.killpg(os.getpgid(parent_pid), signal.SIGINT)
                # Also send directly to parent in case process group differs
                os.kill(parent_pid, signal.SIGINT)
            except (OSError, ProcessLookupError) as exc:
                self.get_logger().error(
                    f'failed to signal parent launch for shutdown: {exc}; falling back to rclpy.shutdown')
                rclpy.shutdown()
            return

        self.get_logger().warn(
            f'{reason}; launch marker not present, shutting down manager process only')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StemmNav2Manager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_zero_velocity()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
