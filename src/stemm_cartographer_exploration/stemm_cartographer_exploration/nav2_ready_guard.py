#!/usr/bin/env python3
"""Wait for the existing Nav2 stack before allowing RRT exploration to start."""

import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .lifecycle_retry import LifecycleRetryState


LIFECYCLE_NODES = (
    '/controller_server',
    '/planner_server',
    '/bt_navigator',
    '/behavior_server',
    '/waypoint_follower',
)


class Nav2ReadyGuard(Node):
    def __init__(self) -> None:
        super().__init__('stemm_cartographer_nav2_ready_guard')
        self.declare_parameter('timeout_sec', 35.0)
        self.declare_parameter('stable_sec', 0.8)
        self.declare_parameter('request_timeout_sec', 1.5)
        self.declare_parameter('retry_delay_sec', 0.5)
        self.declare_parameter('retry_max_delay_sec', 2.0)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.stable_sec = float(self.get_parameter('stable_sec').value)
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'{name}/get_state')
            for name in LIFECYCLE_NODES
        }
        self.lifecycle_futures = {name: None for name in LIFECYCLE_NODES}
        self.lifecycle_states = {name: None for name in LIFECYCLE_NODES}
        self.lifecycle_retry = LifecycleRetryState(
            LIFECYCLE_NODES,
            self.get_parameter('request_timeout_sec').value,
            self.get_parameter('retry_delay_sec').value,
            self.get_parameter('retry_max_delay_sec').value,
        )
        self.action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.spin_action_client = ActionClient(self, Spin, '/spin')

    def _warn_failure(self, name, message, now) -> None:
        if self.lifecycle_retry.should_warn(name, now):
            failures = self.lifecycle_retry.failure_count[name]
            self.get_logger().warn(
                f'Nav2 readiness check for {name} failed '
                f'({failures} consecutive): {message}; retrying')

    def _record_failure(self, name, message, now) -> None:
        self.lifecycle_states[name] = None
        delay = self.lifecycle_retry.mark_failure(name, now)
        self._warn_failure(
            name, f'{message}; next attempt in {delay:.1f}s', now)

    def _retire_pending_request(self, name, client, future) -> None:
        try:
            client.remove_pending_request(future)
        except (KeyError, RuntimeError, ValueError):
            pass
        try:
            future.cancel()
        except Exception:  # noqa: BLE001
            pass
        self.lifecycle_futures[name] = None

    def _update_states(self) -> None:
        now = time.monotonic()
        for name, client in self.lifecycle_clients.items():
            future = self.lifecycle_futures[name]
            if future is not None:
                if future.done():
                    self.lifecycle_futures[name] = None
                    try:
                        result = future.result()
                        if result is None:
                            raise RuntimeError('empty GetState response')
                        self.lifecycle_states[name] = result.current_state.id
                        self.lifecycle_retry.mark_success(name, now)
                    except Exception as exc:  # noqa: BLE001
                        self._record_failure(
                            name, f'GetState response error: {exc}', now)
                elif self.lifecycle_retry.request_expired(name, now):
                    elapsed = now - self.lifecycle_retry.started_at[name]
                    self._retire_pending_request(name, client, future)
                    self._record_failure(
                        name,
                        f'GetState timed out after {elapsed:.1f}s',
                        now,
                    )
                else:
                    continue

            if not self.lifecycle_retry.can_retry(name, now):
                continue
            if not client.service_is_ready():
                self._record_failure(
                    name, 'GetState service is unavailable', now)
                continue
            try:
                self.lifecycle_futures[name] = client.call_async(
                    GetState.Request())
                self.lifecycle_retry.mark_started(name, now)
            except Exception as exc:  # noqa: BLE001
                self.lifecycle_futures[name] = None
                self._record_failure(
                    name, f'could not send GetState request: {exc}', now)

    def _ready(self) -> bool:
        return (
            all(state == State.PRIMARY_STATE_ACTIVE
                for state in self.lifecycle_states.values())
            and self.action_client.server_is_ready()
            and self.spin_action_client.server_is_ready()
        )

    def run(self) -> int:
        deadline = time.monotonic() + self.timeout_sec
        stable_since = None
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._update_states()
            now = time.monotonic()
            if self._ready():
                stable_since = stable_since or now
                if now - stable_since >= self.stable_sec:
                    self.get_logger().info(
                        'Nav2 lifecycle, NavigateToPose, and Spin are stably '
                        'active; RRT may start')
                    return 0
            else:
                stable_since = None
            if now - last_log >= 2.0:
                self.get_logger().info(
                    'waiting for Nav2 active states: '
                    f'{self.lifecycle_states}, '
                    'navigate_ready='
                    f'{self.action_client.server_is_ready()}, '
                    'spin_ready='
                    f'{self.spin_action_client.server_is_ready()}')
                last_log = now
        self.get_logger().error(
            f'Nav2 did not become active within {self.timeout_sec:.1f}s: '
            f'{self.lifecycle_states}, '
            f'navigate_ready={self.action_client.server_is_ready()}, '
            f'spin_ready={self.spin_action_client.server_is_ready()}')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2ReadyGuard()
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()
