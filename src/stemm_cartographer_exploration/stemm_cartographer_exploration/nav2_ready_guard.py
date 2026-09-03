#!/usr/bin/env python3
"""Wait for Nav2 and a non-empty RRT map before starting exploration."""

import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.srv import GetMap
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
        self.declare_parameter('cold_start_extension_sec', 20.0)
        self.declare_parameter(
            'rrt_map_service', '/stemm_cartographer/rrt_dynamic_map')
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.stable_sec = float(self.get_parameter('stable_sec').value)
        request_timeout_sec = self.get_parameter('request_timeout_sec').value
        retry_delay_sec = self.get_parameter('retry_delay_sec').value
        retry_max_delay_sec = self.get_parameter('retry_max_delay_sec').value
        cold_start_extension_sec = self.get_parameter(
            'cold_start_extension_sec'
        ).value
        try:
            self.cold_start_extension_sec = max(
                0.0, float(cold_start_extension_sec)
            )
        except (TypeError, ValueError):
            self.cold_start_extension_sec = 0.0
        self.cold_start_retry_used = False
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'{name}/get_state')
            for name in LIFECYCLE_NODES
        }
        self.lifecycle_futures = {name: None for name in LIFECYCLE_NODES}
        self.lifecycle_states = {name: None for name in LIFECYCLE_NODES}
        self.lifecycle_retry = LifecycleRetryState(
            LIFECYCLE_NODES,
            request_timeout_sec,
            retry_delay_sec,
            retry_max_delay_sec,
        )
        self.action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.spin_action_client = ActionClient(self, Spin, '/spin')
        self.rrt_map_service = str(
            self.get_parameter('rrt_map_service').value)
        self.map_client = self.create_client(GetMap, self.rrt_map_service)
        self.map_future = None
        self.map_ready = False
        self.map_retry = LifecycleRetryState(
            (self.rrt_map_service,),
            request_timeout_sec,
            retry_delay_sec,
            retry_max_delay_sec,
        )

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

    @staticmethod
    def _map_response_is_ready(response) -> bool:
        occupancy_grid = getattr(response, 'map', None)
        info = getattr(occupancy_grid, 'info', None)
        width = int(getattr(info, 'width', 0) or 0)
        height = int(getattr(info, 'height', 0) or 0)
        data = getattr(occupancy_grid, 'data', ())
        return width > 0 and height > 0 and len(data) == width * height

    def _warn_map_failure(self, message, now) -> None:
        name = self.rrt_map_service
        if self.map_retry.should_warn(name, now):
            failures = self.map_retry.failure_count[name]
            self.get_logger().warn(
                f'RRT map readiness check for {name} failed '
                f'({failures} consecutive): {message}; retrying')

    def _record_map_failure(self, message, now) -> None:
        self.map_ready = False
        delay = self.map_retry.mark_failure(self.rrt_map_service, now)
        self._warn_map_failure(
            f'{message}; next attempt in {delay:.1f}s', now)

    def _retire_map_request(self, future) -> None:
        try:
            self.map_client.remove_pending_request(future)
        except (KeyError, RuntimeError, ValueError):
            pass
        try:
            future.cancel()
        except Exception:  # noqa: BLE001
            pass
        self.map_future = None

    def _update_map(self) -> None:
        if self.map_ready:
            return
        now = time.monotonic()
        future = self.map_future
        if future is not None:
            if future.done():
                self.map_future = None
                try:
                    result = future.result()
                    if not self._map_response_is_ready(result):
                        raise RuntimeError('map is empty or has invalid geometry')
                    self.map_ready = True
                    self.map_retry.mark_success(self.rrt_map_service, now)
                    self.get_logger().info(
                        f'RRT map service is ready: {self.rrt_map_service}')
                    return
                except Exception as exc:  # noqa: BLE001
                    self._record_map_failure(
                        f'GetMap response error: {exc}', now)
            elif self.map_retry.request_expired(self.rrt_map_service, now):
                elapsed = now - self.map_retry.started_at[self.rrt_map_service]
                self._retire_map_request(future)
                self._record_map_failure(
                    f'GetMap timed out after {elapsed:.1f}s', now)
            else:
                return

        if not self.map_retry.can_retry(self.rrt_map_service, now):
            return
        if not self.map_client.service_is_ready():
            self._record_map_failure('GetMap service is unavailable', now)
            return
        try:
            self.map_future = self.map_client.call_async(GetMap.Request())
            self.map_retry.mark_started(self.rrt_map_service, now)
        except Exception as exc:  # noqa: BLE001
            self.map_future = None
            self._record_map_failure(
                f'could not send GetMap request: {exc}', now)

    def _ready(self) -> bool:
        return (
            all(state == State.PRIMARY_STATE_ACTIVE
                for state in self.lifecycle_states.values())
            and self.action_client.server_is_ready()
            and self.spin_action_client.server_is_ready()
            and self.map_ready
        )

    def _has_partial_start_evidence(self) -> bool:
        """Allow one extra window only when Nav2 is visibly booting.

        An entirely absent graph must fail on the normal timeout.  Partial
        lifecycle, action, or map-service evidence, however, proves this is the
        cold-start race that can finish inside the existing process tree without
        a second Cartographer launch or another service-stop cycle.
        """
        return (
            any(state is not None for state in self.lifecycle_states.values())
            or any(
                client.service_is_ready()
                for client in self.lifecycle_clients.values()
            )
            or self.action_client.server_is_ready()
            or self.spin_action_client.server_is_ready()
            or self.map_client.service_is_ready()
            or self.map_ready
        )

    def run(self) -> int:
        deadline = time.monotonic() + self.timeout_sec
        stable_since = None
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._update_states()
            self._update_map()
            now = time.monotonic()
            if self._ready():
                stable_since = stable_since or now
                if now - stable_since >= self.stable_sec:
                    self.get_logger().info(
                        'Nav2 lifecycle, NavigateToPose, Spin, and RRT map '
                        'are stably ready; RRT may start')
                    return 0
            else:
                stable_since = None
            if now - last_log >= 2.0:
                self.get_logger().info(
                    'waiting for Nav2/RRT readiness: '
                    f'lifecycle={self.lifecycle_states}, '
                    'navigate_ready='
                    f'{self.action_client.server_is_ready()}, '
                    'spin_ready='
                    f'{self.spin_action_client.server_is_ready()}, '
                    f'rrt_map_ready={self.map_ready}')
                last_log = now
            if now >= deadline:
                if (
                    not self.cold_start_retry_used
                    and self.cold_start_extension_sec > 0.0
                    and self._has_partial_start_evidence()
                ):
                    self.cold_start_retry_used = True
                    deadline = now + self.cold_start_extension_sec
                    stable_since = None
                    self.get_logger().warn(
                        'Nav2/RRT has partial cold-start evidence; granting one '
                        f'additional readiness window of '
                        f'{self.cold_start_extension_sec:.1f}s without relaunching '
                        'the mapping session.')
                    continue
                break
        self.get_logger().error(
            f'Nav2/RRT map did not become ready within {self.timeout_sec:.1f}s: '
            f'{self.lifecycle_states}, '
            f'navigate_ready={self.action_client.server_is_ready()}, '
            f'spin_ready={self.spin_action_client.server_is_ready()}, '
            f'rrt_map_ready={self.map_ready}')
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
