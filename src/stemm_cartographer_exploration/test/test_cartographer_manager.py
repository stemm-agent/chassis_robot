from types import SimpleNamespace
from threading import RLock

import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Transform, Twist
from lifecycle_msgs.msg import State

import stemm_cartographer_exploration.cartographer_manager as manager_module
from stemm_cartographer_exploration.cartographer_manager import (
    StemmCartographerNav2Manager,
)
from stemm_cartographer_exploration.lifecycle_retry import LifecycleRetryState
from stemm_cartographer_exploration.exploration_watchdog import (
    ExplorationProgressWatchdog,
)


class FakeLogger:
    def __init__(self):
        self.warnings = []
        self.infos = []
        self.errors = []

    def warn(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeFuture:
    def __init__(
            self, done=False, state_id=None, result_value=None,
            exception=None):
        self._done = done
        self._state_id = state_id
        self._result_value = result_value
        self._exception = exception
        self.cancelled = False
        self.callbacks = []

    def done(self):
        return self._done

    def result(self):
        if self._exception is not None:
            raise self._exception
        if self._result_value is not None:
            return self._result_value
        if self._state_id is None:
            return None
        return SimpleNamespace(
            current_state=SimpleNamespace(id=self._state_id))

    def cancel(self):
        self.cancelled = True

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self._done:
            callback(self)

    def resolve(self, result_value=None):
        assert not self._done
        self._result_value = result_value
        self._done = True
        for callback in list(self.callbacks):
            callback(self)

    def fail(self, exception):
        assert not self._done
        self._exception = exception
        self._done = True
        for callback in list(self.callbacks):
            callback(self)


class FakeGoalHandle:
    def __init__(
            self, accepted=True, cancel_futures=None, result_future=None):
        self.accepted = accepted
        self.cancel_futures = list(cancel_futures or [FakeFuture()])
        self.cancel_count = 0
        self.result_future = result_future or FakeFuture()

    def cancel_goal_async(self):
        index = min(self.cancel_count, len(self.cancel_futures) - 1)
        self.cancel_count += 1
        return self.cancel_futures[index]

    def get_result_async(self):
        return self.result_future


class FakeNavClient:
    def __init__(self, goal_handle):
        self.goal_handle = goal_handle
        self.sent_goals = []

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent_goals.append((goal, feedback_callback))
        return FakeFuture(done=True, result_value=self.goal_handle)


class FakeSpinClient:
    def __init__(self, response_future=None, ready=True):
        self.ready = ready
        self.response_future = response_future
        self.sent_goals = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        if self.response_future is None:
            handle = FakeGoalHandle()
            self.response_future = FakeFuture(
                done=True, result_value=handle)
        return self.response_future


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTransformBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=Time)


class ManagerHarness(StemmCartographerNav2Manager):
    def __init__(self):
        pass

    def get_logger(self):
        return self.logger


def make_manager():
    manager = ManagerHarness()
    manager.logger = FakeLogger()
    manager._goal_kind_context = 'manual'
    manager._pending_goal_kind = None
    manager._active_goal_kind = None
    manager._goal_generation_counter = 0
    manager._pending_goal_generation = None
    manager._active_goal_generation = None
    manager._latest_feedback_distance = None
    manager._distance_source = None
    manager._latest_recovery_count = 0
    manager._pose_stall_generation = None
    manager._pose_stall_anchor_x = None
    manager._pose_stall_anchor_y = None
    manager._pose_stall_anchor_at = 0.0
    manager._feedback_state_lock = RLock()
    manager._feedback_baseline_generation = None
    manager._feedback_baseline_deadline = 0.0
    manager._feedback_baseline_locked = True
    manager._stall_cancel_requested = False
    manager._recovery_after_cancel = False
    manager._cancel_response_future = None
    manager._cancel_started_at = 0.0
    manager._cancel_last_attempt_at = 0.0
    manager._cancel_attempts = 0
    manager._cancel_acknowledged = False
    manager._cancel_timeout_reported = False
    manager._cancel_context = 'navigation_goal'
    manager._cancel_on_accept_context = None
    manager._last_cancel_zero_time = 0.0
    manager._goal_send_started_at = 0.0
    manager._goal_acceptance_timed_out = False
    manager._timed_out_goal_future = None
    manager._goal_transport_fault = False
    manager._result_retry_attempted = False
    manager._last_progress_log = 0.0
    manager._stall_confirmation_started_at = None
    manager._rrt_wait_started_at = None
    manager._recovery_intent = None
    manager._blocked_episode_started_at = None
    manager._blocked_episode_attempted = False
    manager._blocked_clear_since = None
    manager._next_blocked_recovery_at = 0.0
    manager._recovery_log_times = {}
    manager._spin_phase = 'idle'
    manager._spin_generation_counter = 0
    manager._spin_generation = None
    manager._spin_context = None
    manager._spin_send_future = None
    manager._spin_goal_handle = None
    manager._spin_result_future = None
    manager._spin_cancel_future = None
    manager._spin_cancel_requested = False
    manager._spin_cancel_acknowledged = False
    manager._spin_phase_started_at = 0.0
    manager._spin_cancel_started_at = 0.0
    manager._spin_cancel_on_accept_reason = None
    manager._spin_cancel_reason = None
    manager._spin_transport_fault = False
    manager._cartographer_spin_active = False
    manager.cartographer_finish_client = FakeClient()
    manager.cartographer_trajectory_id = 0
    manager.cartographer_finish_timeout_sec = 10.0
    manager.cartographer_finish_tf_translation_tolerance_m = 0.01
    manager.cartographer_finish_tf_rotation_tolerance_rad = 0.01
    manager._cartographer_finish_future = None
    manager._cartographer_finish_started_at = 0.0
    manager._cartographer_trajectory_finished = False
    manager._cartographer_return_tf_anchor = None
    manager._cartographer_return_tf_stable_since = 0.0
    manager._last_cartographer_tf_motion_log = 0.0
    manager._frozen_return_map_to_odom = None
    manager._frozen_return_tf_broadcaster = FakeTransformBroadcaster()
    manager.get_clock = lambda: FakeClock()
    manager.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args: _tf_sample())
    manager._costmap_clear_clients = {}
    manager._costmap_clear_futures = []
    manager.spin_client = FakeSpinClient()
    manager.exploration_watchdog = ExplorationProgressWatchdog(
        10.0, 0.12, 50.0)
    manager.exploration_behavior_tree = '/tmp/cartographer_exploration.xml'
    manager.goal_acceptance_timeout_sec = 3.0
    manager.feedback_zero_epsilon_m = 0.01
    manager.feedback_zero_goal_tolerance_m = 0.30
    manager.feedback_baseline_settle_sec = 2.0
    manager.stall_cancel_response_timeout_sec = 2.0
    manager.stall_cancel_result_timeout_sec = 5.0
    manager.stall_cancel_max_attempts = 2
    manager.stall_confirmation_sec = 0.8
    manager.exploration_pose_stall_timeout_sec = 7.0
    manager.exploration_pose_stall_radius_m = 0.10
    manager.exploration_pose_stall_goal_tolerance_m = 0.30
    manager.clear_costmaps_on_stall = True
    manager.stall_spin_action_name = '/spin'
    manager.stall_spin_yaw_rad = 1.05
    manager.stall_spin_escalated_yaw_rad = 1.30
    manager.stall_spin_time_allowance_sec = 5.0
    manager.stall_spin_acceptance_timeout_sec = 2.0
    manager.stall_spin_result_timeout_sec = 6.0
    manager.stall_spin_cancel_timeout_sec = 3.0
    manager.costmap_clear_wait_sec = 0.6
    manager.blocked_recovery_delay_sec = 1.5
    manager.blocked_recovery_cooldown_sec = 5.0
    manager.blocked_clearance_hysteresis_m = 0.08
    manager.blocked_clearance_stable_sec = 0.5
    manager.recovery_log_throttle_sec = 2.0
    manager.cancel_zero_guard_period_sec = 0.05
    manager.auto_explore = True
    manager.mapping_done = False
    manager.base_charging = False
    manager.pending_home_goal = False
    manager.return_home_requested = False
    manager.shutdown_after_return_home = False
    manager.return_home_quiet_sec = 2.0
    manager.return_home_ready_after = 0.0
    manager.return_home_tf_stable_since = 0.0
    manager.last_return_home_wait_log = 0.0
    manager.return_home_retry_count = 0
    manager.return_home_max_retries = 3
    manager.return_home_retry_delay_sec = 2.0
    manager._map_revision = 0
    manager._final_map_finish_revision = None
    manager._final_map_wait_started_at = 0.0
    manager._mapping_cancel_requested = False
    manager.nav_goal_active = False
    manager.goal_send_future = None
    manager.goal_result_future = None
    manager.current_goal_handle = None
    manager.last_goal = None
    manager.last_error = ''
    manager.last_safety_stop = 0.0
    manager.last_cmd_vel = Twist()
    manager.progress_x = None
    manager.progress_y = None
    manager.progress_yaw = None
    manager.progress_time = 0.0
    manager.mode = 'mapping'
    manager.navigation_state = 'idle'
    manager.recovery_until = 0.0
    manager.recovery_started_at = 0.0
    manager.recovery_clear_count = 0
    manager.recovery_twist = Twist()
    manager.recovery_turn_direction = 0.0
    manager.recovery_max_duration = 8.0
    manager.stuck_recovery_cycles = 0
    manager.left_min_range = 1.0
    manager.right_min_range = 0.8
    manager.front_blocked = False
    manager.front_blocked_count = 0
    manager.front_min_range = 1.0
    manager.front_center_min_range = 1.0
    manager.front_wide_min_range = 1.0
    manager.front_left_range = 1.0
    manager.front_right_range = 1.0
    manager.rear_min_range = 1.0
    manager.front_stop_distance = 0.42
    manager.front_emergency_distance = 0.25
    manager.global_frame = 'map'
    manager.started_at = 0.0
    manager.explore_startup_delay_sec = 8.0
    manager.rrt_fallback_wait_sec = 5.0
    manager.rrt_projection_radius_m = 1.0
    manager.last_charging_log = 0.0
    manager.cmd_vel_pub = FakePublisher()
    manager.zero_count = 0
    manager.publish_zero_velocity = lambda: setattr(
        manager, 'zero_count', manager.zero_count + 1)
    manager.remember_failed_goal = lambda: None
    manager.remember_blocked_place = lambda: None
    manager.nav2_is_ready = lambda: True
    manager.map_is_fresh = lambda: True
    manager.update_pose = lambda: make_pose(0.0, 0.0)
    manager.is_current_pose_goal = lambda _pose: False
    manager._kill_rrt_nodes = lambda: None
    return manager


def make_nav_feedback(distance, recoveries=0):
    return SimpleNamespace(feedback=SimpleNamespace(
        distance_remaining=distance,
        number_of_recoveries=recoveries,
    ))


def make_pose(x=0.0, y=0.0):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


class FakeClient:
    def __init__(self):
        self.ready = True
        self.call_count = 0
        self.removed = []
        self.last_future = None

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        self.call_count += 1
        self.last_request = _request
        self.last_future = FakeFuture()
        return self.last_future

    def remove_pending_request(self, future):
        self.removed.append(future)


class LifecycleHarness:
    _warn_lifecycle_failure = (
        StemmCartographerNav2Manager._warn_lifecycle_failure)
    _retire_pending_request = (
        StemmCartographerNav2Manager._retire_pending_request)
    _record_lifecycle_failure = (
        StemmCartographerNav2Manager._record_lifecycle_failure)
    poll = StemmCartographerNav2Manager._nav2_lifecycle_callback_impl

    def __init__(self, future):
        self.node_name = '/bt_navigator'
        self.client = FakeClient()
        self.nav_lifecycle_nodes = (self.node_name,)
        self.nav_lifecycle_clients = {self.node_name: self.client}
        self.nav_lifecycle_futures = {self.node_name: future}
        self.nav_lifecycle_states = {self.node_name: None}
        self.nav_lifecycle_retry = LifecycleRetryState(
            self.nav_lifecycle_nodes,
            request_timeout_sec=1.5,
            retry_delay_sec=0.5,
            retry_max_delay_sec=2.0,
        )
        self.nav2_ready = False
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


def test_pending_lifecycle_future_is_retired_and_retried(monkeypatch):
    now = [12.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    stuck_future = FakeFuture()
    manager = LifecycleHarness(stuck_future)
    manager.nav_lifecycle_retry.mark_started(manager.node_name, 10.0)

    manager.poll()
    assert manager.client.removed == [stuck_future]
    assert stuck_future.cancelled
    assert manager.nav_lifecycle_futures[manager.node_name] is None
    assert manager.nav_lifecycle_states[manager.node_name] is None
    assert manager.nav_lifecycle_retry.failure_count[manager.node_name] == 1
    assert manager.client.call_count == 0

    now[0] = 12.5
    manager.poll()
    assert manager.client.call_count == 1
    assert manager.nav_lifecycle_futures[manager.node_name] is not None

    manager.client.last_future._done = True
    manager.client.last_future._state_id = State.PRIMARY_STATE_ACTIVE
    now[0] = 12.6
    manager.poll()
    assert manager.nav_lifecycle_states[manager.node_name] == (
        State.PRIMARY_STATE_ACTIVE)
    assert manager.nav_lifecycle_retry.failure_count[manager.node_name] == 0
    assert manager.nav2_ready
    assert manager.client.call_count == 2


class FrontierHarness:
    frontier = StemmCartographerNav2Manager.frontier_goal_callback

    def __init__(self):
        self.auto_explore = True
        self.mapping_done = False
        self._goal_transport_fault = False
        self._spin_transport_fault = False
        self._spin_phase = 'idle'
        self._stall_cancel_requested = False
        self.nav_goal_active = False
        self.goal_send_future = None
        self.front_blocked = False
        self.not_ready_reports = 0

    def nav2_is_ready(self):
        return False

    def report_nav2_not_ready(self):
        self.not_ready_reports += 1


def test_frontier_is_ignored_without_polluting_state_while_nav2_is_unready():
    manager = FrontierHarness()
    manager.frontier(object())
    assert manager.not_ready_reports == 1

    manager.goal_send_future = object()
    manager.frontier(object())
    assert manager.not_ready_reports == 1


def test_stall_requests_exact_cancel_once_before_any_recovery(monkeypatch):
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()

    manager._trigger_stalled_exploration_recovery('no progress for 10s', 1.2)

    assert manager.current_goal_handle.cancel_count == 1
    assert manager._cancel_attempts == 1
    assert manager._stall_cancel_requested
    assert manager._recovery_after_cancel
    assert manager.recovery_until == 0.0
    assert manager.zero_count == 1
    assert manager.navigation_state == 'canceling_stalled_exploration_goal'

    manager._trigger_stalled_exploration_recovery(
        'duplicate watchdog tick', 1.2)
    assert manager.current_goal_handle.cancel_count == 1


def test_manager_watchdog_confirms_then_cancels_at_10_point_8_seconds(
        monkeypatch):
    now = [9.999]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._distance_source = 'feedback'
    manager._latest_feedback_distance = 2.0
    manager.exploration_watchdog.reset(0.0)
    manager.exploration_watchdog.best_distance = 2.0

    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 0

    now[0] = 10.0
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 0
    assert manager.navigation_state == 'confirming_exploration_stall'

    now[0] = 10.799
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 0

    now[0] = 10.8
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 1
    assert manager._stall_cancel_requested

    now[0] = 11.0
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 1


def test_pose_stall_cancels_at_seven_seconds_and_blacklists_goal(monkeypatch):
    now = [10.0]
    position = [0.0, 0.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager._active_goal_generation = 7
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._distance_source = 'feedback'
    manager._latest_feedback_distance = 2.0
    manager.exploration_watchdog = ExplorationProgressWatchdog(
        100.0, 0.12, 200.0)
    manager.exploration_watchdog.reset(now[0])
    manager.update_pose = lambda: make_pose(*position)
    remembered = []
    manager.remember_failed_goal = lambda: remembered.append('goal')
    manager.remember_blocked_place = lambda: remembered.append('place')

    manager._progress_watchdog_callback_impl()
    now[0] = 16.999
    position[:] = [0.04, -0.03]
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 0

    now[0] = 17.0
    manager._progress_watchdog_callback_impl()

    assert manager.current_goal_handle.cancel_count == 1
    assert manager._stall_cancel_requested
    assert manager._recovery_after_cancel
    assert remembered == ['goal', 'place']
    assert 'blacklisting it and switching targets' in manager.last_error


def test_pose_stall_window_resets_after_small_but_real_translation(
        monkeypatch):
    now = [10.0]
    position = [0.0, 0.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager._active_goal_generation = 3
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._distance_source = 'feedback'
    manager._latest_feedback_distance = 2.0
    manager.exploration_watchdog = ExplorationProgressWatchdog(
        100.0, 0.12, 200.0)
    manager.exploration_watchdog.reset(now[0])
    manager.update_pose = lambda: make_pose(*position)

    manager._progress_watchdog_callback_impl()
    now[0] = 16.0
    position[:] = [0.11, 0.0]
    manager._progress_watchdog_callback_impl()
    now[0] = 22.999
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 0

    now[0] = 23.0
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 1


def test_pose_stall_does_not_abandon_goal_inside_terminal_tolerance(
        monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager._active_goal_generation = 2
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._distance_source = 'feedback'
    manager._latest_feedback_distance = 0.20
    manager.exploration_watchdog = ExplorationProgressWatchdog(
        100.0, 0.12, 200.0)
    manager.exploration_watchdog.reset(now[0])

    manager._progress_watchdog_callback_impl()
    now[0] = 30.0
    manager._progress_watchdog_callback_impl()

    assert manager.current_goal_handle.cancel_count == 0
    assert not manager._stall_cancel_requested


def test_cancel_timeouts_have_finite_retries_and_safe_stop(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager.nav_goal_active = True
    first = FakeFuture()
    second = FakeFuture()
    manager.current_goal_handle = FakeGoalHandle(
        cancel_futures=[first, second])

    manager._trigger_stalled_exploration_recovery('no progress for 10s', 2.0)
    now[0] = 12.1
    manager._monitor_stall_cancel(now[0])
    assert first.cancelled
    assert manager.current_goal_handle.cancel_count == 2

    now[0] = 14.2
    manager._monitor_stall_cancel(now[0])
    assert second.cancelled
    assert manager._cancel_timeout_reported
    assert not manager.auto_explore
    assert manager._goal_transport_fault
    assert manager.navigation_state == 'stalled_cancel_timeout'
    assert manager.recovery_until == 0.0


def test_cancel_ack_without_result_faults_at_five_seconds(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    cancel_response = FakeFuture(
        done=True,
        result_value=SimpleNamespace(goals_canceling=[object()]),
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle(
        cancel_futures=[cancel_response])

    manager._trigger_stalled_exploration_recovery('no progress', 1.0)
    now[0] = 10.1
    manager._monitor_stall_cancel(now[0])
    assert manager._cancel_acknowledged
    assert manager.recovery_until == 0.0

    now[0] = 15.0
    manager._monitor_stall_cancel(now[0])
    assert manager._cancel_timeout_reported
    assert manager.navigation_state == 'stalled_cancel_timeout'
    assert manager.current_goal_handle is not None
    assert manager.nav_goal_active
    assert manager.recovery_until == 0.0


def test_canceled_result_requests_one_clear_and_queues_nav2_spin(monkeypatch):
    order = []

    def fake_base_result(self, _future):
        order.append('base_cleanup')
        self.nav_goal_active = False
        self.current_goal_handle = None

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_result_callback',
        fake_base_result,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager._stall_cancel_requested = True
    manager._recovery_after_cancel = True
    manager._cancel_context = 'stalled'
    manager._recovery_intent = 'stalled_cancel'
    manager._cancel_acknowledged = True
    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_CANCELED),
    )
    manager.goal_result_future = result
    manager._request_costmap_clears = lambda: order.append('clear_requested')

    manager.goal_result_callback(result)

    assert order == ['base_cleanup', 'clear_requested']
    assert manager._spin_phase == 'clearing'
    assert manager._spin_context == 'stalled_cancel'
    assert manager.stuck_recovery_cycles == 1
    assert manager._rrt_wait_started_at is None


def test_mapping_done_stops_recovery_before_home_dispatch():
    observed = {}
    manager = make_manager()
    manager.recovery_until = 20.0
    manager.recovery_started_at = 10.0
    manager.recovery_twist.angular.z = 0.8
    manager.navigation_state = 'turning_out_of_blocked_place'
    manager._dispatch_pending_home_goal = lambda: (
        observed.update({
            'recovery_until': manager.recovery_until,
            'linear': manager.recovery_twist.linear.x,
            'angular': manager.recovery_twist.angular.z,
        })
        or (True, 'home dispatched'))
    response = SimpleNamespace()

    returned = manager.set_mapping_done_callback(
        SimpleNamespace(data=True), response)

    assert returned is response
    assert observed == {'recovery_until': 0.0, 'linear': 0.0, 'angular': 0.0}
    assert manager.zero_count == 1


def _finish_response(code=0, message='finished'):
    return SimpleNamespace(
        status=SimpleNamespace(code=code, message=message))


def test_mapping_done_finishes_cartographer_before_home_dispatch(monkeypatch):
    manager = make_manager()
    manager._save_final_map_before_return = lambda: (
        True, 'final map already saved')
    dispatched = []
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_dispatch_pending_home_goal',
        lambda _self: (dispatched.append('home') or (True, 'home dispatched')),
    )

    manager.set_mapping_done_callback(
        SimpleNamespace(data=True), SimpleNamespace())

    assert manager.cartographer_finish_client.call_count == 1
    assert manager.cartographer_finish_client.last_request.trajectory_id == 0
    assert dispatched == []

    manager.cartographer_finish_client.last_future.resolve(_finish_response())
    started, message = manager._dispatch_pending_home_goal()

    assert started
    assert message == 'home dispatched'
    assert manager._cartographer_trajectory_finished
    assert dispatched == ['home']


def test_finished_cartographer_rebroadcasts_final_map_to_odom():
    manager = make_manager()
    sampled = _tf_sample(x=0.31, y=-0.27)
    manager.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args: sampled)
    manager._cartographer_finish_future = FakeFuture(
        done=True, result_value=_finish_response())

    finished, _ = manager._finish_cartographer_trajectory_before_return()

    assert finished
    assert manager._cartographer_trajectory_finished
    assert manager._frozen_return_map_to_odom.header.frame_id == 'map'
    assert manager._frozen_return_map_to_odom.child_frame_id == 'odom_combined'
    assert manager._frozen_return_map_to_odom.transform.translation.x == 0.31
    assert manager._frozen_return_map_to_odom.transform.translation.y == -0.27
    assert len(manager._frozen_return_tf_broadcaster.transforms) == 1


def test_cartographer_finish_failure_retries_before_home_dispatch(monkeypatch):
    manager = make_manager()
    dispatched = []
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_dispatch_pending_home_goal',
        lambda _self: (dispatched.append('home') or (True, 'home dispatched')),
    )

    manager.set_mapping_done_callback(
        SimpleNamespace(data=True), SimpleNamespace())
    manager.cartographer_finish_client.last_future.resolve(
        _finish_response(2, 'trajectory is not active'))
    started, message = manager._dispatch_pending_home_goal()

    assert not started
    assert 'was rejected' in message
    assert 'retrying return-home 1/3' in message
    assert manager.navigation_state == 'return_home_retry_recovery'
    assert manager.mode == 'returning_home'
    assert manager.pending_home_goal
    assert dispatched == []


def _tf_sample(x=0.0, y=0.0):
    transform = Transform()
    transform.translation.x = x
    transform.translation.y = y
    transform.rotation.w = 1.0
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        child_frame_id='odom_combined',
        transform=transform)


def test_finished_cartographer_requires_final_tf_settle_window(monkeypatch):
    manager = make_manager()
    manager.mapping_done = True
    manager._cartographer_trajectory_finished = True
    samples = [_tf_sample()]
    manager.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args: samples[0])
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_tf_is_recent',
        lambda *_args, **_kwargs: True,
    )

    assert not manager._tf_is_stably_recent(
        'odom_combined', 'map', max_age=0.2, stable_sec=1.5)
    now[0] += 1.4
    assert not manager._tf_is_stably_recent(
        'odom_combined', 'map', max_age=0.2, stable_sec=1.5)
    now[0] += 0.2
    assert manager._tf_is_stably_recent(
        'odom_combined', 'map', max_age=0.2, stable_sec=1.5)

    samples[0] = _tf_sample(x=0.02)
    assert not manager._tf_is_stably_recent(
        'odom_combined', 'map', max_age=0.2, stable_sec=1.5)


def test_mapping_done_waits_for_terminal_result_before_home(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    dispatched = []
    manager._dispatch_pending_home_goal = lambda: (
        dispatched.append('home') or (True, 'home dispatched'))
    response = SimpleNamespace()

    manager.set_mapping_done_callback(SimpleNamespace(data=True), response)
    assert manager.current_goal_handle.cancel_count == 1
    assert manager._stall_cancel_requested
    assert dispatched == []

    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_CANCELED),
    )
    manager.goal_result_future = result
    manager.goal_result_callback(result)
    assert dispatched == ['home']
    assert manager.current_goal_handle is None
    assert not manager.nav_goal_active


def test_home_precheck_recovery_never_uses_exploration_spin(monkeypatch):
    calls = []

    def fake_home_dispatch(self):
        self.start_escape_recovery()
        return False, 'home start blocked'

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_dispatch_pending_home_goal',
        fake_home_dispatch,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'start_escape_recovery',
        lambda _self: calls.append('inherited_home_recovery'),
    )
    manager = make_manager()
    manager._save_final_map_before_return = lambda: (
        True, 'final map already saved')
    manager.mapping_done = True
    manager._cartographer_trajectory_finished = True
    manager.auto_explore = False
    manager.pending_home_goal = True
    manager._active_goal_kind = 'exploration_manager'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._start_stall_spin_recovery = lambda *_args: calls.append(
        'exploration_spin')
    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_CANCELED),
    )
    manager.goal_result_future = result

    manager.goal_result_callback(result)

    assert calls == ['inherited_home_recovery']


def test_aborted_old_goal_dispatches_home_without_recovery():
    manager = make_manager()
    manager.mapping_done = True
    manager.auto_explore = False
    manager.pending_home_goal = True
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    calls = []
    manager._dispatch_pending_home_goal = lambda: (
        calls.append('home') or (True, 'home dispatched'))
    manager._start_stall_spin_recovery = lambda *_args: calls.append('spin')
    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_ABORTED),
    )
    manager.goal_result_future = result

    manager.goal_result_callback(result)

    assert calls == ['home']
    assert manager.current_goal_handle is None
    assert not manager.nav_goal_active


def test_sync_goal_response_keeps_source_and_exploration_bt(monkeypatch):
    def fake_base_response(self, future):
        self.goal_send_future = None
        goal_handle = future.result()
        self.current_goal_handle = goal_handle
        self.nav_goal_active = True
        self.reset_progress_watchdog()
        self.navigation_state = 'executing'
        self.goal_result_future = goal_handle.get_result_async()

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_response_callback',
        fake_base_response,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'reset_progress_watchdog',
        lambda _self: None,
    )
    manager = make_manager()
    goal_handle = FakeGoalHandle()
    manager.nav_client = FakeNavClient(goal_handle)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0

    manager._run_with_goal_kind(
        'exploration_rrt', manager.send_nav_goal, pose, False, False)

    sent_goal = manager.nav_client.sent_goals[0][0]
    assert sent_goal.behavior_tree == manager.exploration_behavior_tree
    assert manager._active_goal_kind == 'exploration_rrt'
    assert manager._pending_goal_kind is None
    assert manager._active_goal_generation == 1
    assert manager._pending_goal_generation is None
    assert manager.nav_goal_active


def test_distant_goal_zero_bootstrap_feedback_is_ignored(monkeypatch):
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager._active_goal_generation = 7
    manager.nav_goal_active = True
    manager.last_goal = make_pose(5.0, 0.0)
    manager.update_pose = lambda: make_pose(0.0, 0.0)
    manager.exploration_watchdog.reset(0.0)

    manager._feedback_callback_for_goal(7, make_nav_feedback(0.0))

    assert manager._latest_feedback_distance is None
    assert manager._distance_source is None
    assert manager.exploration_watchdog.best_distance is None

    manager._feedback_callback_for_goal(7, make_nav_feedback(5.373))

    assert manager._latest_feedback_distance == pytest.approx(5.373)
    assert manager._distance_source == 'feedback'
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.373)


def test_near_goal_zero_feedback_is_plausible(monkeypatch):
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager._active_goal_generation = 3
    manager.nav_goal_active = True
    manager.last_goal = make_pose(0.05, 0.0)
    manager.update_pose = lambda: make_pose(0.0, 0.0)
    manager.exploration_watchdog.reset(0.0)

    manager._feedback_callback_for_goal(3, make_nav_feedback(0.0))

    assert manager._latest_feedback_distance == 0.0
    assert manager.exploration_watchdog.best_distance == 0.0


def test_stale_previous_goal_feedback_cannot_seed_new_watchdog(monkeypatch):
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager._active_goal_generation = 1
    manager.nav_goal_active = True
    manager.exploration_watchdog.reset(0.0)
    manager._feedback_callback_for_goal(1, make_nav_feedback(1.347))
    assert manager.exploration_watchdog.best_distance == pytest.approx(1.347)

    manager._active_goal_generation = 2
    manager._latest_feedback_distance = None
    manager._distance_source = None
    manager._latest_recovery_count = 0
    manager.exploration_watchdog.reset(5.0)

    manager._feedback_callback_for_goal(1, make_nav_feedback(1.347))
    assert manager._latest_feedback_distance is None
    assert manager.exploration_watchdog.best_distance is None

    manager._feedback_callback_for_goal(2, make_nav_feedback(5.373))
    assert manager._latest_feedback_distance == pytest.approx(5.373)
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.373)

    manager._feedback_callback_for_goal(1, make_nav_feedback(0.5))
    assert manager._latest_feedback_distance == pytest.approx(5.373)
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.373)


def test_same_goal_feedback_progress_restarts_stagnation_clock(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager._active_goal_generation = 9
    manager.nav_goal_active = True
    manager.exploration_watchdog.reset(0.0)

    manager._feedback_callback_for_goal(9, make_nav_feedback(5.0))
    manager._progress_watchdog_callback_impl()

    now[0] = 4.0
    manager._feedback_callback_for_goal(9, make_nav_feedback(4.7))
    manager._progress_watchdog_callback_impl()

    assert manager.exploration_watchdog.best_distance == pytest.approx(4.7)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(4.0)


def test_reset_promotes_generation_before_inherited_reset(monkeypatch):
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_generation = 1
    manager._pending_goal_kind = 'exploration_manager'
    manager._pending_goal_generation = 2
    manager.nav_goal_active = True
    manager.last_goal = make_pose(5.2, 0.0)
    manager.update_pose = lambda: make_pose(0.0, 0.0)

    def fake_base_reset(_self):
        manager._feedback_callback_for_goal(1, make_nav_feedback(1.347))
        manager._feedback_callback_for_goal(2, make_nav_feedback(5.2))

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'reset_progress_watchdog',
        fake_base_reset,
    )

    manager.reset_progress_watchdog()

    assert manager._active_goal_generation == 2
    assert manager._pending_goal_generation is None
    assert manager._latest_feedback_distance == pytest.approx(5.2)
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.2)


def test_feedback_rechecks_generation_at_commit_boundary(monkeypatch):
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager._active_goal_generation = 4
    manager.nav_goal_active = True
    manager.exploration_watchdog.reset(0.0)

    def invalidate_during_parse(_distance):
        manager._active_goal_generation = 5
        return True

    manager._feedback_distance_is_plausible = invalidate_during_parse
    manager._feedback_callback_for_goal(4, make_nav_feedback(3.0))

    assert manager._latest_feedback_distance is None
    assert manager._distance_source is None
    assert manager.exploration_watchdog.best_distance is None


def test_same_generation_stale_baseline_is_corrected_then_locked(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'reset_progress_watchdog',
        lambda _self: None,
    )
    manager = make_manager()
    manager._pending_goal_kind = 'exploration_manager'
    manager._pending_goal_generation = 11
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager.last_goal = make_pose(5.3, 0.0)
    manager.update_pose = lambda: make_pose(0.0, 0.0)
    manager.reset_progress_watchdog()
    baseline_deadline = manager._feedback_baseline_deadline

    now[0] = 0.1
    manager._feedback_callback_for_goal(11, make_nav_feedback(2.957))
    assert manager.exploration_watchdog.best_distance == pytest.approx(2.957)

    now[0] = 0.2
    manager._feedback_callback_for_goal(11, make_nav_feedback(5.2))
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.2)
    assert manager._feedback_baseline_deadline == pytest.approx(
        baseline_deadline)

    now[0] = 0.5
    manager._feedback_callback_for_goal(11, make_nav_feedback(4.9))
    assert manager._feedback_baseline_locked
    assert manager.exploration_watchdog.best_distance == pytest.approx(4.9)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(0.5)
    _, decision = manager._observe_exploration_progress(now[0])
    assert decision == 'tracking'
    assert manager.exploration_watchdog.best_distance == pytest.approx(4.9)

    now[0] = 0.7
    manager._feedback_callback_for_goal(11, make_nav_feedback(5.6))
    assert manager.exploration_watchdog.best_distance == pytest.approx(4.9)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(0.5)

    now[0] = 1.2
    manager._feedback_callback_for_goal(11, make_nav_feedback(4.5))
    assert manager.exploration_watchdog.best_distance == pytest.approx(4.5)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(1.2)
    _, decision = manager._observe_exploration_progress(now[0])
    assert decision == 'tracking'
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(1.2)

    # The old false-cancel point is now only 9.6 s after real progress.
    now[0] = 10.8
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 0

    # A genuinely stagnant goal still confirms and cancels in a bounded time.
    now[0] = 11.2
    manager._progress_watchdog_callback_impl()
    assert manager.navigation_state == 'confirming_exploration_stall'
    now[0] = 12.0
    manager._progress_watchdog_callback_impl()
    assert manager.current_goal_handle.cancel_count == 1


def test_feedback_baseline_deadline_is_fixed_and_recovery_rebase_is_bounded(
        monkeypatch):
    now = [0.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'reset_progress_watchdog',
        lambda _self: None,
    )
    manager = make_manager()
    manager._pending_goal_kind = 'exploration_rrt'
    manager._pending_goal_generation = 12
    manager.nav_goal_active = True
    manager._feedback_distance_is_plausible = lambda _distance: True
    manager.reset_progress_watchdog()
    deadline = manager._feedback_baseline_deadline

    for timestamp, distance in ((0.1, 2.957), (0.3, 3.6), (1.0, 5.2)):
        now[0] = timestamp
        manager._feedback_callback_for_goal(12, make_nav_feedback(distance))
        assert manager._feedback_baseline_deadline == pytest.approx(deadline)

    assert manager.exploration_watchdog.best_distance == pytest.approx(5.2)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(1.0)
    assert manager.exploration_watchdog.started_at == pytest.approx(0.0)

    now[0] = 2.01
    manager._feedback_callback_for_goal(12, make_nav_feedback(6.0))
    assert manager._feedback_baseline_locked
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.2)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(1.0)

    now[0] = 3.0
    manager._feedback_callback_for_goal(12, make_nav_feedback(6.5, 1))
    assert manager.exploration_watchdog.best_distance == pytest.approx(6.5)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(3.0)
    assert manager.exploration_watchdog.recovery_count == 1
    assert manager.exploration_watchdog.started_at == pytest.approx(0.0)


def test_feedback_arriving_during_geometric_lookup_wins(monkeypatch):
    now = [0.2]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'feedback_callback',
        lambda _self, _feedback: None,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager._active_goal_generation = 13
    manager._feedback_baseline_generation = 13
    manager._feedback_baseline_deadline = 2.0
    manager._feedback_baseline_locked = False
    manager.nav_goal_active = True
    manager.exploration_watchdog.reset(0.0)
    manager.exploration_watchdog.observe(0.1, 2.957, 0)
    manager._feedback_distance_is_plausible = lambda _distance: True

    def geometric_lookup_with_feedback(_pose=None):
        manager._feedback_callback_for_goal(13, make_nav_feedback(5.2))
        return 2.957

    manager._geometric_distance_to_active_goal = geometric_lookup_with_feedback
    distance, decision = manager._observe_exploration_progress(now[0])

    assert distance == pytest.approx(5.2)
    assert decision == 'tracking'
    assert manager._distance_source == 'feedback'
    assert manager.exploration_watchdog.best_distance == pytest.approx(5.2)


def test_manual_goal_uses_default_bt(monkeypatch):
    def fake_base_response(self, future):
        self.goal_send_future = None
        goal_handle = future.result()
        self.current_goal_handle = goal_handle
        self.nav_goal_active = True
        self.reset_progress_watchdog()

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_response_callback',
        fake_base_response,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'reset_progress_watchdog',
        lambda _self: None,
    )
    manager = make_manager()
    manager.nav_client = FakeNavClient(FakeGoalHandle())
    pose = PoseStamped()
    pose.header.frame_id = 'map'

    manager.send_nav_goal(pose, validate_map=False)

    sent_goal = manager.nav_client.sent_goals[0][0]
    assert sent_goal.behavior_tree == ''
    assert manager._active_goal_kind == 'manual'


def test_home_goal_uses_default_bt(monkeypatch):
    def fake_base_response(self, future):
        self.goal_send_future = None
        goal_handle = future.result()
        self.current_goal_handle = goal_handle
        self.nav_goal_active = True
        self.reset_progress_watchdog()

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_response_callback',
        fake_base_response,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'reset_progress_watchdog',
        lambda _self: None,
    )
    manager = make_manager()
    manager.return_home_requested = True
    manager.auto_explore = False
    manager.mapping_done = True
    manager.nav_client = FakeNavClient(FakeGoalHandle())
    pose = PoseStamped()
    pose.header.frame_id = 'map'

    manager.send_nav_goal(pose, validate_map=False)

    sent_goal = manager.nav_client.sent_goals[0][0]
    assert sent_goal.behavior_tree == ''
    assert manager._active_goal_kind == 'home'


def test_acceptance_timeout_late_goal_is_canceled_exactly(monkeypatch):
    manager = make_manager()
    goal_handle = FakeGoalHandle()
    pending = FakeFuture(result_value=goal_handle)
    manager.goal_send_future = pending
    manager._pending_goal_kind = 'exploration_manager'
    manager._goal_send_started_at = 1.0

    manager._monitor_goal_acceptance(4.1)
    assert manager._goal_acceptance_timed_out
    assert manager._goal_transport_fault
    assert not manager.auto_explore
    assert manager.navigation_state == 'goal_acceptance_timeout'

    pending._done = True
    manager.goal_response_callback(pending)
    assert goal_handle.cancel_count == 1
    assert manager._stall_cancel_requested
    assert manager._cancel_response_future is goal_handle.cancel_futures[0]
    assert manager.nav_goal_active
    assert manager.current_goal_handle is goal_handle
    assert manager.navigation_state == (
        'timed_out_goal_canceling_accepted_goal')
    assert manager._active_goal_kind == 'exploration_manager'


def test_synchronous_terminal_result_cannot_restart_cancel(monkeypatch):
    def fake_base_result(self, _future):
        self.nav_goal_active = False
        self.current_goal_handle = None

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_result_callback',
        fake_base_result,
    )
    manager = make_manager()
    manager.auto_explore = False
    goal_handle = FakeGoalHandle()
    goal_handle.result_future = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_CANCELED),
    )

    manager._monitor_accepted_goal_cancel(
        goal_handle, 'exploration_rrt', 'stop_navigation')

    assert goal_handle.cancel_count == 1
    assert manager.current_goal_handle is None
    assert not manager.nav_goal_active
    assert not manager._stall_cancel_requested
    assert not manager._cancel_timeout_reported


def test_recovery_during_acceptance_uses_monitored_exact_cancel(monkeypatch):
    base_calls = []
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_response_callback',
        lambda _self, _future: base_calls.append('base'),
    )
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager.recovery_started_at = 9.0
    manager.recovery_until = 12.0
    manager.recovery_twist.angular.z = 0.75
    manager.navigation_state = 'turning_out_of_blocked_place'
    handle = FakeGoalHandle()
    pending = FakeFuture(done=True, result_value=handle)
    manager.goal_send_future = pending
    manager._pending_goal_kind = 'exploration_rrt'

    manager.goal_response_callback(pending)

    assert base_calls == []
    assert manager.goal_send_future is None
    assert manager._pending_goal_kind is None
    assert manager.current_goal_handle is handle
    assert manager.nav_goal_active
    assert manager._active_goal_kind == 'exploration_rrt'
    assert manager._stall_cancel_requested
    assert manager._cancel_context == 'recovery_during_goal_acceptance'
    assert not manager._recovery_after_cancel
    assert handle.cancel_count == 1
    assert manager.goal_result_callback in handle.result_future.callbacks
    assert manager.recovery_until == 0.0
    assert manager.recovery_twist.angular.z == 0.0
    assert manager.navigation_state == (
        'recovery_during_goal_acceptance_canceling_accepted_goal')


@pytest.mark.parametrize(
    'status',
    [
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_SUCCEEDED,
    ],
)
def test_stop_terminal_status_never_starts_recovery(monkeypatch, status):
    inherited_recovery_calls = []

    def fake_base_result(self, future):
        self.nav_goal_active = False
        self.current_goal_handle = None
        if future.result().status == GoalStatus.STATUS_ABORTED:
            self.start_escape_recovery()

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_result_callback',
        fake_base_result,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'start_escape_recovery',
        lambda _self: inherited_recovery_calls.append('base_recovery'),
    )
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    spin_calls = []
    manager._start_stall_spin_recovery = (
        lambda *_args: spin_calls.append('spin'))

    manager.stop_navigation_callback(SimpleNamespace(), SimpleNamespace())
    result = FakeFuture(
        done=True, result_value=SimpleNamespace(status=status))
    manager.goal_result_future = result
    manager.goal_result_callback(result)

    assert spin_calls == []
    assert inherited_recovery_calls == []
    assert manager.recovery_until == 0.0
    assert manager.recovery_twist.angular.z == 0.0
    assert manager.current_goal_handle is None
    assert not manager.nav_goal_active
    assert not manager._stall_cancel_requested
    assert not manager.auto_explore
    assert manager.mode == 'stopped'


def test_repeated_stop_keeps_original_cancel_deadline(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle(
        cancel_futures=[FakeFuture(), FakeFuture()])

    manager.stop_navigation_callback(SimpleNamespace(), SimpleNamespace())
    original_future = manager._cancel_response_future
    now[0] = 11.9
    manager.stop_navigation_callback(SimpleNamespace(), SimpleNamespace())

    assert manager.current_goal_handle.cancel_count == 1
    assert manager._cancel_started_at == 10.0
    assert manager._cancel_last_attempt_at == 10.0
    assert manager._cancel_response_future is original_future

    now[0] = 12.1
    manager._monitor_stall_cancel(now[0])
    assert original_future.cancelled
    assert manager.current_goal_handle.cancel_count == 2


def test_repeated_stop_after_ack_cannot_extend_result_timeout(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    accepted = FakeFuture(
        done=True,
        result_value=SimpleNamespace(goals_canceling=[object()]),
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle(
        cancel_futures=[accepted])

    manager.stop_navigation_callback(SimpleNamespace(), SimpleNamespace())
    now[0] = 10.1
    manager._monitor_stall_cancel(now[0])
    assert manager._cancel_acknowledged

    now[0] = 14.9
    manager.stop_navigation_callback(SimpleNamespace(), SimpleNamespace())
    assert manager._cancel_started_at == 10.0
    assert manager.current_goal_handle.cancel_count == 1

    now[0] = 15.0
    manager._monitor_stall_cancel(now[0])
    assert manager._cancel_timeout_reported
    assert manager._goal_transport_fault


def test_repeated_charging_pause_uses_one_cancel_request(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager.base_charging = True
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()

    manager.pause_for_charging()
    original_future = manager._cancel_response_future
    for stamp in (11.0, 12.0):
        now[0] = stamp
        manager.pause_for_charging()

    assert manager.current_goal_handle.cancel_count == 1
    assert manager._cancel_attempts == 1
    assert manager._cancel_started_at == 10.0
    assert manager._cancel_last_attempt_at == 10.0
    assert manager._cancel_response_future is original_future
    assert manager._cancel_context == 'base_charging'
    assert not manager._recovery_after_cancel


def test_charging_arms_exact_cancel_for_pending_acceptance(monkeypatch):
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager.base_charging = True
    handle = FakeGoalHandle()
    pending = FakeFuture(done=True, result_value=handle)
    manager.goal_send_future = pending
    manager._pending_goal_kind = 'exploration_rrt'

    manager.pause_for_charging()
    assert manager._cancel_on_accept_context == 'base_charging'

    manager.goal_response_callback(pending)
    assert handle.cancel_count == 1
    assert manager.current_goal_handle is handle
    assert manager._stall_cancel_requested
    assert manager._cancel_context == 'base_charging'
    assert manager.goal_result_callback in handle.result_future.callbacks


def test_charging_downgrades_stall_recovery_without_resending(monkeypatch):
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._trigger_stalled_exploration_recovery('stalled', 1.0)
    original_started_at = manager._cancel_started_at

    manager.base_charging = True
    manager.pause_for_charging()

    assert manager.current_goal_handle.cancel_count == 1
    assert manager._cancel_started_at == original_started_at
    assert manager._cancel_context == 'base_charging'
    assert not manager._recovery_after_cancel


def test_manual_watchdog_delegates_to_inherited_policy(monkeypatch):
    calls = []
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_progress_watchdog_callback_impl',
        lambda _self: calls.append('base'),
    )
    manager = make_manager()
    manager._active_goal_kind = 'manual'

    manager._progress_watchdog_callback_impl()

    assert calls == ['base']
    assert not manager._stall_cancel_requested


def test_result_future_exception_stops_without_recovery():
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    failed_result = FakeFuture(
        done=True, exception=RuntimeError('result transport lost'))
    manager.goal_result_future = failed_result

    manager.goal_result_callback(failed_result)

    assert manager.current_goal_handle is not None
    assert manager.nav_goal_active
    assert manager.current_goal_handle.cancel_count == 1
    assert manager._stall_cancel_requested
    assert not manager.auto_explore
    assert manager._goal_transport_fault
    assert manager.navigation_state == 'goal_result_error_canceling'
    assert manager.recovery_until == 0.0


@pytest.mark.parametrize(
    'status, expected_spin_count',
    [
        (GoalStatus.STATUS_CANCELED, 1),
        (GoalStatus.STATUS_ABORTED, 0),
        (GoalStatus.STATUS_SUCCEEDED, 0),
    ],
)
def test_only_canceled_stall_terminal_can_start_spin(
        monkeypatch, status, expected_spin_count):
    def fake_base_result(self, future):
        self.nav_goal_active = False
        self.current_goal_handle = None
        if future.result().status == GoalStatus.STATUS_ABORTED:
            self.start_escape_recovery()

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_result_callback',
        fake_base_result,
    )
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._stall_cancel_requested = True
    manager._recovery_after_cancel = True
    manager._cancel_context = 'stalled'
    manager._recovery_intent = 'stalled_cancel'
    spin_calls = []
    manager._start_stall_spin_recovery = (
        lambda *_args: spin_calls.append('spin'))
    result = FakeFuture(
        done=True, result_value=SimpleNamespace(status=status))
    manager.goal_result_future = result

    manager.goal_result_callback(result)

    assert len(spin_calls) == expected_spin_count
    assert not manager._stall_cancel_requested
    if status != GoalStatus.STATUS_CANCELED:
        assert manager.recovery_until == 0.0
        assert manager.recovery_twist.angular.z == 0.0


def test_faulted_aborted_terminal_cannot_start_recovery(monkeypatch):
    inherited_calls = []

    def fake_base_result(self, _future):
        self.nav_goal_active = False
        self.current_goal_handle = None
        self.start_escape_recovery()

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_result_callback',
        fake_base_result,
    )
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'start_escape_recovery',
        lambda _self: inherited_calls.append('base_recovery'),
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_rrt'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager._stall_cancel_requested = True
    manager._cancel_context = 'goal_result_error'
    manager._goal_transport_fault = True
    manager.auto_explore = False
    manager.navigation_state = 'goal_result_error_canceling'
    manager.last_error = 'result transport fault'
    spin_calls = []
    manager._start_stall_spin_recovery = (
        lambda *_args: spin_calls.append('spin'))
    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_ABORTED),
    )
    manager.goal_result_future = result

    manager.goal_result_callback(result)

    assert inherited_calls == []
    assert spin_calls == []
    assert manager.mode == 'safety_stop'
    assert manager.navigation_state == 'goal_result_error_canceling'
    assert manager.last_error == 'result transport fault'
    assert manager.recovery_until == 0.0
    assert manager.recovery_twist.angular.z == 0.0


def test_precheck_blocking_waits_then_attempts_once_per_episode(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager.front_blocked = True
    manager.front_min_range = 0.35
    calls = []
    manager._start_stall_spin_recovery = (
        lambda context: calls.append(context) or True)

    assert manager._arm_exploration_precheck_recovery(now[0])
    now[0] = 11.49
    manager._safety_stop_callback_impl()
    assert calls == []

    now[0] = 11.50
    manager._safety_stop_callback_impl()
    assert calls == ['exploration_precheck']
    assert manager._blocked_episode_attempted
    assert manager._recovery_intent is None

    for tick in range(300):
        now[0] = 11.51 + tick * 0.2
        manager._safety_stop_callback_impl()
    assert calls == ['exploration_precheck']
    assert len(manager.logger.warnings) < 40


def test_recovery_log_throttle_is_per_key(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()

    manager._log_recovery_event('safety', 'safety blocked')
    manager._log_recovery_event('rrt', 'rrt blocked')
    manager._log_recovery_event('safety', 'safety blocked again')
    manager._log_recovery_event('rrt', 'rrt blocked again')
    assert manager.logger.warnings == ['safety blocked', 'rrt blocked']

    now[0] = 12.0
    manager._log_recovery_event('safety', 'safety retry')
    manager._log_recovery_event('rrt', 'rrt retry')
    assert manager.logger.warnings[-2:] == ['safety retry', 'rrt retry']


def test_blocked_episode_requires_hysteresis_and_stable_clearance(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager._blocked_episode_started_at = 9.0
    manager._blocked_episode_attempted = True
    manager.front_blocked = False

    manager.front_min_range = 0.49
    manager._update_blocked_episode(now[0])
    assert manager._blocked_episode_started_at == 9.0

    manager.front_min_range = 0.51
    now[0] = 10.1
    manager._update_blocked_episode(now[0])
    now[0] = 10.59
    manager._update_blocked_episode(now[0])
    assert manager._blocked_episode_started_at == 9.0

    manager.front_min_range = 0.49
    now[0] = 10.6
    manager._update_blocked_episode(now[0])
    assert manager._blocked_clear_since is None

    manager.front_min_range = 0.51
    now[0] = 11.0
    manager._update_blocked_episode(now[0])
    now[0] = 11.5
    manager._update_blocked_episode(now[0])
    assert manager._blocked_episode_started_at is None
    assert not manager._blocked_episode_attempted


def test_success_with_static_front_block_never_auto_starts_recovery(
        monkeypatch):
    def fake_base_result(self, _future):
        self.nav_goal_active = False
        self.current_goal_handle = None
        self.navigation_state = 'succeeded'

    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        'goal_result_callback',
        fake_base_result,
    )
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager.nav_goal_active = True
    manager.current_goal_handle = FakeGoalHandle()
    manager.front_blocked = True
    manager.front_min_range = 0.39
    manager._recovery_intent = 'exploration_precheck'
    calls = []
    manager._start_stall_spin_recovery = (
        lambda context: calls.append(context) or True)
    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED),
    )

    manager.goal_result_callback(result)
    for _ in range(300):
        manager._safety_stop_callback_impl()

    assert calls == []
    assert manager._recovery_intent is None
    assert manager.navigation_state == 'blocked_waiting_clearance'
    assert len(manager.logger.warnings) <= 2


def _patch_base_finish(monkeypatch):
    def fake_finish(self, state, error=''):
        self.navigation_state = state
        self.last_error = error
    monkeypatch.setattr(
        manager_module.StemmNav2Manager, 'finish_recovery', fake_finish)


def test_nav2_spin_starts_with_one_clear_and_no_nonzero_twist(monkeypatch):
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    clears = []
    manager._request_costmap_clears = lambda: clears.append('clear')

    assert manager._start_stall_spin_recovery('stalled_cancel')

    assert clears == ['clear']
    assert manager._spin_phase == 'clearing'
    assert manager._cartographer_spin_active
    assert manager.recovery_twist.linear.x == 0.0
    assert manager.recovery_twist.linear.y == 0.0
    assert manager.recovery_twist.angular.z == 0.0
    assert manager.cmd_vel_pub.messages == []


def test_nav2_spin_goal_has_bounded_yaw_and_time_allowance(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager.spin_client = FakeSpinClient(response_future=FakeFuture())

    manager._start_stall_spin_recovery('stalled_cancel')
    manager._recovery_turn_callback_impl()

    assert manager._spin_phase == 'sending'
    assert len(manager.spin_client.sent_goals) == 1
    goal = manager.spin_client.sent_goals[0]
    assert abs(goal.target_yaw) == pytest.approx(1.05)
    assert goal.time_allowance.sec == 5
    assert goal.time_allowance.nanosec == 0
    assert manager.recovery_twist.linear.x == 0.0
    assert manager.recovery_twist.angular.z == 0.0


def test_nav2_spin_acceptance_and_success_return_to_idle(monkeypatch):
    _patch_base_finish(monkeypatch)
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    response = FakeFuture()
    result = FakeFuture()
    handle = FakeGoalHandle(result_future=result)
    manager = make_manager()
    manager.spin_client = FakeSpinClient(response_future=response)

    manager._start_stall_spin_recovery('stalled_cancel')
    manager._recovery_turn_callback_impl()
    response.resolve(handle)
    assert manager._spin_phase == 'executing'

    result.resolve(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert manager._spin_phase == 'idle'
    assert manager.navigation_state == 'recovery_clear'
    assert not manager._cartographer_spin_active


def test_synchronous_spin_result_cannot_resurrect_executing_phase(
        monkeypatch):
    _patch_base_finish(monkeypatch)
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    result = FakeFuture(
        done=True,
        result_value=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED),
    )
    handle = FakeGoalHandle(result_future=result)
    response = FakeFuture(done=True, result_value=handle)
    manager = make_manager()
    manager.spin_client = FakeSpinClient(response_future=response)

    manager._start_stall_spin_recovery('stalled_cancel')
    manager._recovery_turn_callback_impl()

    assert manager._spin_phase == 'idle'
    assert manager._spin_generation is None
    assert manager.navigation_state == 'recovery_clear'


def test_spin_acceptance_timeout_cancels_late_accepted_goal(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    response = FakeFuture()
    handle = FakeGoalHandle()
    manager = make_manager()
    manager.spin_client = FakeSpinClient(response_future=response)

    manager._start_stall_spin_recovery('stalled_cancel')
    manager._recovery_turn_callback_impl()
    now[0] = 12.0
    manager._recovery_turn_callback_impl()
    assert manager._spin_phase == 'acceptance_timeout'
    assert manager._spin_transport_fault
    assert not manager.auto_explore

    response.resolve(handle)
    assert handle.cancel_count == 1
    assert manager._spin_phase == 'canceling'


def test_spin_result_and_cancel_timeout_latch_transport_fault(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    response = FakeFuture()
    handle = FakeGoalHandle()
    manager = make_manager()
    manager.spin_client = FakeSpinClient(response_future=response)

    manager._start_stall_spin_recovery('stalled_cancel')
    manager._recovery_turn_callback_impl()
    response.resolve(handle)
    now[0] = 16.0
    manager._recovery_turn_callback_impl()
    assert handle.cancel_count == 1
    assert manager._spin_phase == 'canceling'

    now[0] = 19.0
    manager._recovery_turn_callback_impl()
    assert manager._spin_transport_fault
    assert not manager.auto_explore
    assert manager.navigation_state == 'spin_cancel_timeout'


def test_spin_cancel_ack_is_not_resent_or_allowed_to_extend_timeout(
        monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    cancel_ack = FakeFuture(
        done=True,
        result_value=SimpleNamespace(goals_canceling=[object()]),
    )
    handle = FakeGoalHandle(cancel_futures=[cancel_ack])
    manager = make_manager()
    manager._spin_phase = 'executing'
    manager._spin_generation = 1
    manager._spin_goal_handle = handle
    manager._cartographer_spin_active = True
    manager.mapping_done = True

    manager._recovery_turn_callback_impl()
    assert handle.cancel_count == 1
    assert manager._spin_cancel_acknowledged
    assert manager._spin_cancel_started_at == 10.0

    now[0] = 12.9
    manager._recovery_turn_callback_impl()
    assert handle.cancel_count == 1
    assert manager._spin_cancel_started_at == 10.0

    now[0] = 13.0
    manager._recovery_turn_callback_impl()
    assert handle.cancel_count == 1
    assert manager._spin_transport_fault
    assert manager.navigation_state == 'spin_cancel_timeout'


def test_spin_cancel_dispatch_exception_is_fail_closed_and_not_retried(
        monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    calls = []
    handle = FakeGoalHandle()

    def fail_cancel():
        calls.append('cancel')
        raise RuntimeError('transport unavailable')

    handle.cancel_goal_async = fail_cancel
    manager = make_manager()
    manager._spin_phase = 'executing'
    manager._spin_generation = 1
    manager._spin_goal_handle = handle
    manager._cartographer_spin_active = True

    assert not manager._request_spin_cancel('stop_navigation')
    assert calls == ['cancel']
    assert manager._spin_phase == 'canceling'
    assert manager._spin_cancel_requested
    assert manager._spin_transport_fault
    assert manager._motion_is_inhibited()

    now[0] = 12.9
    manager._recovery_turn_callback_impl()
    assert calls == ['cancel']
    assert manager._spin_cancel_started_at == 10.0

    now[0] = 13.0
    manager._recovery_turn_callback_impl()
    assert calls == ['cancel']
    assert manager.navigation_state == 'spin_cancel_timeout'


def test_mapping_done_waits_for_spin_terminal_before_home(monkeypatch):
    _patch_base_finish(monkeypatch)
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    response = FakeFuture()
    result = FakeFuture()
    handle = FakeGoalHandle(result_future=result)
    manager = make_manager()
    manager.spin_client = FakeSpinClient(response_future=response)
    home = []
    manager._dispatch_pending_home_goal = lambda: (
        home.append('home') or (True, 'home dispatched'))

    manager._start_stall_spin_recovery('stalled_cancel')
    manager._recovery_turn_callback_impl()
    response.resolve(handle)
    service_response = SimpleNamespace()
    manager.set_mapping_done_callback(
        SimpleNamespace(data=True), service_response)

    assert handle.cancel_count == 1
    assert manager._spin_phase == 'canceling'
    assert home == []

    result.resolve(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert manager._spin_phase == 'idle'
    assert home == ['home']


@pytest.mark.parametrize(
    'phase, should_clamp',
    [
        ('idle', False),
        ('clearing', True),
        ('sending', True),
        ('canceling_pending_acceptance', True),
        ('executing', False),
        ('canceling', True),
        ('acceptance_timeout', True),
    ],
)
def test_nonzero_cmd_vel_clamps_all_nonexecuting_spin_phases(
        phase, should_clamp):
    manager = make_manager()
    manager._spin_phase = phase
    command = Twist()
    command.linear.x = 0.2

    manager.cmd_vel_callback(command)

    assert manager.zero_count == int(should_clamp)


def test_disabled_exploration_ignores_rrt_frontiers():
    manager = FrontierHarness()
    manager.auto_explore = False

    manager.frontier(object())

    assert manager.not_ready_reports == 0


def test_rrt_wait_begins_only_after_recovery_is_eligible(monkeypatch):
    calls = []
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_auto_explore_callback_impl',
        lambda _self: calls.append('base'),
    )
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 100.0)
    manager = make_manager()
    manager._rrt_wait_started_at = 80.0
    manager.recovery_until = 101.0

    manager._auto_explore_callback_impl()
    assert manager._rrt_wait_started_at is None
    assert calls == ['base']

    manager.recovery_until = 0.0
    manager.navigation_state = 'recovery_clear'
    manager._auto_explore_callback_impl()
    assert manager._rrt_wait_started_at == 100.0
    assert manager.navigation_state == 'waiting_for_rrt_frontier'
    assert calls == ['base']


def test_zero_rrt_wait_delegates_on_first_eligible_tick(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    calls = []
    monkeypatch.setattr(
        manager_module.StemmNav2Manager,
        '_auto_explore_callback_impl',
        lambda _self: calls.append('base_selector'),
    )
    manager = make_manager()
    manager.rrt_fallback_wait_sec = 0.0

    manager._auto_explore_callback_impl()

    assert calls == ['base_selector']
    assert manager._rrt_wait_started_at is None
    assert manager.navigation_state != 'waiting_for_rrt_frontier'


def test_exploration_scheduler_arms_idle_blocked_precheck(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager.front_blocked = True
    manager.front_min_range = 0.35

    manager._run_with_goal_kind(
        'exploration_manager', manager.start_escape_recovery)

    assert manager._recovery_intent == 'exploration_precheck'
    assert manager._blocked_episode_started_at == pytest.approx(10.0)
    assert manager.navigation_state == 'blocked_precheck_pending'
    assert manager.zero_count == 1


@pytest.mark.parametrize('auto_explore,mapping_done', [(False, False), (True, True)])
def test_disabled_exploration_does_not_arm_idle_blocked_precheck(
        monkeypatch, auto_explore, mapping_done):
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: 10.0)
    manager = make_manager()
    manager.auto_explore = auto_explore
    manager.mapping_done = mapping_done
    manager.front_blocked = True

    manager._run_with_goal_kind(
        'exploration_manager', manager.start_escape_recovery)

    assert manager._recovery_intent is None


def _front_scan(distance):
    ranges = [1.0] * 7
    ranges[3] = distance
    return SimpleNamespace(
        angle_min=-0.30,
        angle_increment=0.10,
        ranges=ranges,
    )


def test_center_emergency_requires_two_frames_and_clear_resets():
    manager = make_manager()

    manager.scan_callback(_front_scan(0.20))
    assert manager.front_blocked_count == 3
    assert not manager.front_blocked

    manager.scan_callback(_front_scan(0.20))
    assert manager.front_blocked_count == 5
    assert manager.front_blocked

    manager.scan_callback(_front_scan(1.0))
    assert manager.front_blocked_count == 0
    assert not manager.front_blocked


def test_ordinary_stop_band_retains_five_frame_debounce():
    manager = make_manager()

    for _ in range(4):
        manager.scan_callback(_front_scan(0.35))
        assert not manager.front_blocked
    manager.scan_callback(_front_scan(0.35))

    assert manager.front_blocked_count == 5
    assert manager.front_blocked


def test_fallback_prefilter_skips_target_without_clearance(monkeypatch):
    manager = make_manager()
    width = 80
    height = 80
    resolution = 0.20
    origin = SimpleNamespace(x=-8.0, y=-8.0)
    data = [0] * (width * height)
    blocked_cell = (27, 40)
    data[blocked_cell[1] * width + blocked_cell[0]] = 100
    manager.last_map = SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(position=origin),
        ),
        data=data,
    )
    manager.goal_clearance = 0.38
    manager.last_explore_heading = None
    manager.update_pose = lambda: make_pose(0.0, 0.0)
    allowed = ((-2.7, 0.1), (2.9, 0.1))
    manager.goal_recently_failed = lambda x, y: not any(
        abs(x - ax) < 0.01 and abs(y - ay) < 0.01
        for ax, ay in allowed)
    manager.path_blocked_ratio = lambda *_args: 0.0
    monkeypatch.setattr(
        manager_module.random, 'shuffle',
        lambda cells: cells.sort(key=lambda item: item[0]),
    )

    goal = manager._pick_fallback_goal()

    assert goal is not None
    assert goal.pose.position.x == pytest.approx(2.9)
    assert goal.pose.position.y == pytest.approx(0.1)


def _prepare_tf_progress_manager():
    manager = make_manager()
    manager._active_goal_kind = 'exploration_manager'
    manager._active_goal_generation = 1
    manager.nav_goal_active = True
    manager.last_goal = make_pose(2.0, 0.0)
    manager.exploration_watchdog = ExplorationProgressWatchdog(
        18.0, 0.12, 50.0)
    manager.exploration_watchdog.reset(0.0)
    manager.exploration_watchdog.best_distance = 2.0
    manager._distance_source = 'feedback'
    manager._latest_feedback_distance = 2.0
    manager.progress_x = 0.0
    manager.progress_y = 0.0
    manager.progress_yaw = 0.0
    return manager


def test_commanded_tf_motion_rebases_exploration_watchdog():
    manager = _prepare_tf_progress_manager()
    manager.last_cmd_vel.linear.x = 0.10
    manager.update_pose = lambda: make_pose(0.05, 0.0)

    changed = manager._rebase_watchdog_from_commanded_pose_progress(10.0)

    assert changed
    assert manager.progress_x == pytest.approx(0.05)
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(10.0)
    assert manager.exploration_watchdog.started_at == pytest.approx(0.0)


def test_tf_change_without_motion_command_does_not_mask_stall():
    manager = _prepare_tf_progress_manager()
    manager.update_pose = lambda: make_pose(0.20, 0.0)

    changed = manager._rebase_watchdog_from_commanded_pose_progress(10.0)

    assert not changed
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(0.0)


def test_commanded_tf_noise_below_threshold_does_not_mask_stall():
    manager = _prepare_tf_progress_manager()
    manager.last_cmd_vel.linear.x = 0.10
    manager.update_pose = lambda: make_pose(0.01, 0.0)

    changed = manager._rebase_watchdog_from_commanded_pose_progress(10.0)

    assert not changed
    assert manager.exploration_watchdog.last_progress_at == pytest.approx(0.0)


def _prepare_rrt_projection_map(target_unknown=True):
    manager = make_manager()
    width = 30
    height = 30
    resolution = 0.10
    data = [0] * (width * height)
    target_mx = 15
    target_my = 15
    if target_unknown:
        data[target_my * width + target_mx] = -1
    manager.last_map = SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=data,
    )
    manager.goal_clearance = 0.15
    manager.rrt_projection_radius_m = 0.50
    manager.update_pose = lambda: make_pose(0.55, 0.55)
    manager.goal_recently_failed = lambda _x, _y: False
    manager.path_blocked_ratio = lambda *_args: 0.0
    goal = make_pose(
        (target_mx + 0.5) * resolution,
        (target_my + 0.5) * resolution,
    )
    goal.header.frame_id = 'map'
    return manager, goal, target_mx, target_my


def test_unknown_rrt_frontier_projects_to_known_free_cell():
    manager, goal, target_mx, target_my = _prepare_rrt_projection_map()

    projected, changed, error = manager._project_rrt_frontier_goal(goal)

    assert error == ''
    assert changed
    assert projected is not None
    info = manager.last_map.info
    mx = int(projected.pose.position.x / info.resolution)
    my = int(projected.pose.position.y / info.resolution)
    assert (mx, my) != (target_mx, target_my)
    assert manager.last_map.data[my * info.width + mx] == 0
    assert manager.goal_is_traversable(
        projected, check_path_density=False)[0]


def test_known_free_rrt_frontier_is_not_moved():
    manager, goal, _target_mx, _target_my = (
        _prepare_rrt_projection_map(target_unknown=False))

    projected, changed, error = manager._project_rrt_frontier_goal(goal)

    assert error == ''
    assert not changed
    assert projected.pose.position.x == pytest.approx(goal.pose.position.x)
    assert projected.pose.position.y == pytest.approx(goal.pose.position.y)


def test_rrt_projection_rejects_area_without_known_free_cell():
    manager, goal, _target_mx, _target_my = _prepare_rrt_projection_map()
    manager.last_map.data = [-1] * len(manager.last_map.data)

    projected, changed, error = manager._project_rrt_frontier_goal(goal)

    assert projected is None
    assert not changed
    assert 'no safe known-free projection' in error

def test_return_home_setup_failures_are_retried_before_manual_takeover(
        monkeypatch):
    now = [100.0]
    monkeypatch.setattr(manager_module.time, 'monotonic', lambda: now[0])
    manager = make_manager()
    manager.return_home_max_retries = 3
    manager.return_home_retry_delay_sec = 2.0
    manager._final_map_finish_revision = None
    manager._final_map_wait_started_at = 0.0

    started, message = manager._fail_return_home_before_dispatch(
        'temporary Cartographer TF gap')

    assert not started
    assert 'retrying return-home 1/3' in message
    assert manager.return_home_retry_count == 1
    assert manager.pending_home_goal
    assert not manager.return_home_requested
    assert manager.mode == 'returning_home'
    assert manager.navigation_state == 'return_home_retry_recovery'
    assert manager.return_home_ready_after == pytest.approx(102.0)

    manager.return_home_retry_count = manager.return_home_max_retries
    started, message = manager._fail_return_home_before_dispatch(
        'persistent Cartographer TF gap')

    assert not started
    assert 'failed after 3 retries' in message
    assert manager.return_home_retry_count == 4
    assert not manager.pending_home_goal
    assert manager.mode == 'stopped'
    assert manager.navigation_state == 'return_home_failed_waiting_manual'
