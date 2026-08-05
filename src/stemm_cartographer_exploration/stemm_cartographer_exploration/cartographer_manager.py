#!/usr/bin/env python3
"""Cartographer-only STEMM manager with recoverable Nav2 readiness checks."""

import json
import math
import os
import random
import time
from functools import partial
from pathlib import Path
from threading import RLock

from action_msgs.msg import GoalStatus
from cartographer_ros_msgs.srv import FinishTrajectory
from geometry_msgs.msg import TransformStamped, Twist
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
        self.create_service(
            Trigger,
            '/stemm/cancel_mapping',
            self.cancel_mapping_callback,
        )
        self.session_status_path = os.getenv(
            'STEMM_CARTOGRAPHER_SESSION_STATUS_PATH',
            '/home/wheeltec/.local/state/stemm-cartographer-session.json',
        )
        self._session_status_write_error = ''
        self.session_task_path = os.getenv(
            'STEMM_CARTOGRAPHER_TASK_PATH',
            '/home/wheeltec/.local/state/stemm-cartographer-task.json',
        )
        self.declare_parameter('nav_lifecycle_request_timeout_sec', 1.5)
        self.declare_parameter('nav_lifecycle_retry_delay_sec', 0.5)
        self.declare_parameter('nav_lifecycle_retry_max_delay_sec', 2.0)
        self.declare_parameter('exploration_no_progress_timeout_sec', 18.0)
        self.declare_parameter('exploration_progress_delta_m', 0.12)
        self.declare_parameter('exploration_hard_timeout_sec', 50.0)
        self.declare_parameter('exploration_pose_stall_timeout_sec', 7.0)
        self.declare_parameter('exploration_pose_stall_radius_m', 0.10)
        self.declare_parameter(
            'exploration_pose_stall_goal_tolerance_m', 0.30)
        self.declare_parameter('rrt_fallback_wait_sec', 0.0)
        self.declare_parameter('rrt_projection_radius_m', 1.0)
        self.declare_parameter('clear_costmaps_on_stall', True)
        self.declare_parameter(
            'exploration_behavior_tree',
            '/home/wheeltec/wheeltec_ros2/install/'
            'stemm_cartographer_exploration/share/'
            'stemm_cartographer_exploration/config/'
            'stemm_cartographer_nav_to_pose.xml')
        self.declare_parameter('goal_acceptance_timeout_sec', 3.0)
        self.declare_parameter(
            'cartographer_finish_trajectory_service', '/finish_trajectory')
        self.declare_parameter('cartographer_trajectory_id', 0)
        self.declare_parameter('cartographer_finish_timeout_sec', 10.0)
        self.declare_parameter('final_map_wait_timeout_sec', 3.0)
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
        self.declare_parameter('mapping_finish_confirm_timeout_sec', 90.0)
        self.declare_parameter('return_home_hard_timeout_sec', 180.0)
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
        self.rrt_fallback_wait_sec = max(
            0.0, float(self.get_parameter('rrt_fallback_wait_sec').value))
        self.rrt_projection_radius_m = max(
            self.goal_clearance,
            float(self.get_parameter('rrt_projection_radius_m').value),
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
        self.mapping_finish_confirm_timeout_sec = max(
            15.0, float(self.get_parameter(
                'mapping_finish_confirm_timeout_sec').value))
        self.return_home_hard_timeout_sec = max(
            30.0, float(self.get_parameter(
                'return_home_hard_timeout_sec').value))
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
        self._latest_feedback_distance = None
        self._distance_source = None
        self._latest_recovery_count = 0
        self._pose_stall_generation = None
        self._pose_stall_anchor_x = None
        self._pose_stall_anchor_y = None
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

        self.nav2_ready = all(
            self.nav_lifecycle_states.get(node_name)
            == State.PRIMARY_STATE_ACTIVE
            for node_name in self.nav_lifecycle_nodes
        )

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

    def _update_blocked_episode(self, now):
        if self.front_blocked:
            if self._blocked_episode_started_at is None:
                self._blocked_episode_started_at = now
            self._blocked_clear_since = None
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
            self._stall_cancel_requested
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
            self._cartographer_finish_future = None
            self._cartographer_finish_started_at = 0.0
            self._cartographer_trajectory_finished = False
            self._reset_final_map_save_state()
            self._cartographer_return_tf_anchor = None
            self._cartographer_return_tf_stable_since = 0.0
            self._frozen_return_map_to_odom = None
            self._reset_mapping_exit_timeout()
        return response

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

    def _fail_return_home_before_dispatch(self, message):
        """Retry return-home setup failures before manual takeover.

        Cartographer finalization and the first Nav2 goal can experience short
        TF/service gaps while the robot is already near the origin. Those
        transient conditions are not evidence that the home target is
        unreachable, so preserve the active session and consume the same
        bounded retry budget used for terminal Nav2 failures.
        """
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

    def _save_cached_final_map(self):
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
                elif value <= 25:
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
        if (
            (self.last_map is None or not final_frame_arrived)
            and elapsed < self.final_map_wait_timeout_sec
        ):
            self.publish_zero_velocity()
            self.mode = 'finalizing_map'
            self.navigation_state = 'return_home_waiting_final_map'
            return False, (
                'Cartographer trajectory finished; waiting for the final '
                f'OccupancyGrid ({elapsed:.1f}/'
                f'{self.final_map_wait_timeout_sec:.1f}s)')

        if self.last_map is None or not final_frame_arrived:
            self._final_map_save_attempted = True
            self._final_map_save_error = (
                'no post-finish OccupancyGrid arrived within '
                f'{self.final_map_wait_timeout_sec:.1f}s')
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

    def _fallback_cell_has_clearance(self, mx, my):
        """Cheap target-clearance check before selecting a fallback goal."""
        if self.last_map is None:
            return False
        info = self.last_map.info
        resolution = info.resolution
        if resolution <= 0.0:
            return False
        clearance_cells = max(
            1, int(math.ceil(self.goal_clearance / resolution)))
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if math.hypot(dx * resolution, dy * resolution) > self.goal_clearance:
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

        def collect(min_distance, max_distance):
            cells = []
            for my in range(2, height - 2, step):
                row = my * width
                for mx in range(2, width - 2, step):
                    value = data[row + mx]
                    if value < 0 or value >= 30:
                        continue
                    x = origin.x + (mx + 0.5) * resolution
                    y = origin.y + (my + 0.5) * resolution
                    distance = math.hypot(x - cx, y - cy)
                    if distance < min_distance or distance > max_distance:
                        continue
                    if self.goal_recently_failed(x, y):
                        continue
                    cells.append((x, y, distance, mx, my))
            return cells

        candidates = collect(2.5, 5.5)
        if not candidates:
            candidates = collect(1.8, 6.0)
        if not candidates:
            return None

        # Randomize peers for coverage, then stably move candidates that keep
        # the current exploration heading to the front of the validation queue.
        random.shuffle(candidates)
        if self.last_explore_heading is not None:
            def heading_group(candidate):
                heading = math.atan2(candidate[1] - cy, candidate[0] - cx)
                difference = abs(math.atan2(
                    math.sin(heading - self.last_explore_heading),
                    math.cos(heading - self.last_explore_heading)))
                return 0 if difference < 1.05 else 1
            candidates.sort(key=heading_group)

        for x, y, _distance, mx, my in candidates:
            if not self._fallback_cell_has_clearance(mx, my):
                continue
            if self.path_blocked_ratio(cx, cy, x, y) >= 0.10:
                continue
            return self.make_pose_goal(
                x, y, math.atan2(y - cy, x - cx))
        return None

    def send_nav_goal(
            self, pose, validate_map=True, check_path_density=True):
        kind = (
            'home' if self.return_home_requested
            else self._goal_kind_context or 'manual'
        )
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

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.frame_id = pose.header.frame_id or self.global_frame
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        if self._is_exploration_goal(kind):
            goal.behavior_tree = self.exploration_behavior_tree

        self.last_goal = goal.pose
        self.last_error = ''

        if self.is_current_pose_goal(goal.pose):
            self.mode = 'arrived'
            self.navigation_state = 'succeeded'
            self._clear_recovery_intent(clear_episode=True)
            self.get_logger().info(
                'navigation goal is current pose; marking as arrived')
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
        self.get_logger().info(
            f'navigation goal dispatched: source={kind}, '
            f'target=({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}), '
            f'bt={goal.behavior_tree or "default"}')
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
        self._pose_stall_anchor_at = 0.0

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
                decision = self.exploration_watchdog.observe(
                    now, distance, self._latest_recovery_count)
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
            decision = self.exploration_watchdog.observe(
                now, distance, self._latest_recovery_count)
            if decision in ('progress', 'recovery_started'):
                self._feedback_baseline_locked = True
            return distance, decision

    def _pose_stall_elapsed(self, now):
        with self._feedback_state_lock:
            if self._pose_stall_anchor_at <= 0.0:
                return 0.0
            return max(0.0, now - self._pose_stall_anchor_at)

    def _exploration_pose_is_stalled(self, now, distance, pose):
        """Detect a goal that remains inside a small translational bubble."""
        if pose is None:
            return False
        x = pose.pose.position.x
        y = pose.pose.position.y

        with self._feedback_state_lock:
            if (not self.nav_goal_active
                    or not self._is_exploration_goal()
                    or self._stall_cancel_requested):
                return False
            generation = self._active_goal_generation
            if self._pose_stall_generation != generation:
                self._pose_stall_generation = generation
                self._pose_stall_anchor_x = x
                self._pose_stall_anchor_y = y
                self._pose_stall_anchor_at = now
                return False

            if (distance is not None
                    and distance
                    <= self.exploration_pose_stall_goal_tolerance_m):
                # Do not abandon a goal that is already inside its terminal
                # neighborhood merely because the goal checker is settling.
                self._pose_stall_anchor_x = x
                self._pose_stall_anchor_y = y
                self._pose_stall_anchor_at = now
                return False

            if (self._pose_stall_anchor_x is None
                    or self._pose_stall_anchor_y is None
                    or self._pose_stall_anchor_at <= 0.0):
                self._pose_stall_anchor_x = x
                self._pose_stall_anchor_y = y
                self._pose_stall_anchor_at = now
                return False

            displacement = math.hypot(
                x - self._pose_stall_anchor_x,
                y - self._pose_stall_anchor_y,
            )
            if displacement > self.exploration_pose_stall_radius_m:
                self._pose_stall_anchor_x = x
                self._pose_stall_anchor_y = y
                self._pose_stall_anchor_at = now
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
            return

        pose = self.update_pose()
        distance_hint = self._distance_to_active_goal(pose)
        if self._exploration_pose_is_stalled(now, distance_hint, pose):
            self._log_exploration_progress(
                now, 'pose_stalled_switching_goal', distance_hint)
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
        if (self._stall_cancel_requested
                or self._cancel_on_accept_context is not None
                or self._goal_transport_fault
                or self._spin_transport_fault):
            self.publish_zero_velocity()
            return

        now = time.monotonic()
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
            response.success = True
            response.message = (
                'mapping cancellation is already in progress; '
                'waiting for the session runner')
            return response

        if self.shutdown_started:
            response.success = False
            response.message = 'Cartographer session shutdown is already underway'
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
            return super().set_mapping_done_callback(request, response)

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

        self._reset_final_map_save_state()
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

    def _handle_goal_result_error(self, future, completed_kind, exc):
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
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self._handle_goal_result_error(
                future, self._active_goal_kind, exc)
            return
        completed_kind = self._active_goal_kind
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

        candidates = []
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
                if value < 0 or value >= 30:
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
                candidates.append((
                    target_distance,
                    -unknown_neighbors,
                    x,
                    y,
                    mx,
                    my,
                ))

        candidates.sort()
        selected = None
        for _distance, _unknown_score, x, y, mx, my in candidates:
            if self.goal_recently_failed(x, y):
                continue
            if not self._fallback_cell_has_clearance(mx, my):
                continue
            if (current_x is not None
                    and self.path_blocked_ratio(
                        current_x, current_y, x, y) >= 0.10):
                continue
            selected = (x, y)
            break

        if selected is None:
            return (
                None,
                False,
                'RRT frontier rejected: no safe known-free projection within '
                f'{self.rrt_projection_radius_m:.2f}m',
            )
        x, y = selected
        projected = math.hypot(x - target_x, y - target_y) > resolution * 0.51
        if projected:
            yaw = math.atan2(target_y - y, target_x - x)
        elif current_x is not None:
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
        if self._mapping_timeout_failure_kind or self._mapping_cancel_requested:
            return
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
        super()._publish_state_impl()
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
            'returnHomeRetry': int(self.return_home_retry_count),
            'returnHomeMaxRetries': int(self.return_home_max_retries),
            'pendingHomeGoal': bool(self.pending_home_goal),
            'returnHomeRequested': bool(self.return_home_requested),
            'finalMapSaved': bool(self._final_map_saved),
            'finalMapSavedAt': self._final_map_saved_at,
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
    finally:
        if rclpy.ok():
            node.publish_zero_velocity()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
