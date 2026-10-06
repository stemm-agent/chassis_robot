#!/usr/bin/env python3
# coding=utf-8

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


START_SERVICE = '/auto_recharge/start'
SERVICE_DISCOVERY_TIMEOUT_SEC = 30.0
SERVICE_DISCOVERY_POLL_SEC = 1.0
SERVICE_RESPONSE_TIMEOUT_SEC = 12.0
SERVICE_REQUEST_ATTEMPTS = 3
SERVICE_RETRY_INTERVAL_SEC = 1.0
ALREADY_RUNNING_TEXT = '已在执行'


class NavigateToChargerClient(Node):
    '''Compatibility CLI that asks the persistent manager to start recharging.'''

    def __init__(self):
        super().__init__('navigate_to_charger_client')
        self.client = self.create_client(Trigger, START_SERVICE)

    def _wait_for_manager(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        waiting_logged = False

        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False

            if self.client.wait_for_service(
                timeout_sec=min(SERVICE_DISCOVERY_POLL_SEC, remaining)
            ):
                return True

            if not waiting_logged:
                self.get_logger().info(
                    'Waiting for persistent recharge manager: %s '
                    '(up to %.0f seconds)...'
                    % (START_SERVICE, timeout_sec)
                )
                waiting_logged = True

        return False

    @staticmethod
    def _is_accepted_response(response):
        return bool(
            response.success or ALREADY_RUNNING_TEXT in (response.message or '')
        )

    def trigger(self):
        if not self._wait_for_manager(SERVICE_DISCOVERY_TIMEOUT_SEC):
            self.get_logger().error(
                'Persistent recharge manager was not discovered within %.0f '
                'seconds: %s'
                % (SERVICE_DISCOVERY_TIMEOUT_SEC, START_SERVICE)
            )
            return False

        for attempt in range(1, SERVICE_REQUEST_ATTEMPTS + 1):
            if attempt > 1 and not self._wait_for_manager(
                SERVICE_DISCOVERY_TIMEOUT_SEC
            ):
                self.get_logger().warn(
                    'Persistent recharge manager was not rediscovered before '
                    'request %d/%d.' % (attempt, SERVICE_REQUEST_ATTEMPTS)
                )
                continue

            future = self.client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=SERVICE_RESPONSE_TIMEOUT_SEC
            )

            response = None
            if future.done() and not future.cancelled():
                try:
                    response = future.result()
                except Exception as exc:
                    self.get_logger().warn(
                        'Recharge start request %d/%d failed: %s'
                        % (attempt, SERVICE_REQUEST_ATTEMPTS, exc)
                    )

            if response is not None:
                if self._is_accepted_response(response):
                    self.get_logger().info(response.message)
                    return True

                self.get_logger().warn(response.message)
                return False

            if not future.done():
                future.cancel()

            if attempt < SERVICE_REQUEST_ATTEMPTS:
                self.get_logger().warn(
                    'Recharge start request %d/%d did not receive a response '
                    'within %.0f seconds; retrying...'
                    % (
                        attempt,
                        SERVICE_REQUEST_ATTEMPTS,
                        SERVICE_RESPONSE_TIMEOUT_SEC,
                    )
                )
                time.sleep(SERVICE_RETRY_INTERVAL_SEC)

        self.get_logger().error(
            'Recharge start service did not respond after %d attempts.'
            % SERVICE_REQUEST_ATTEMPTS
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToChargerClient()
    success = False
    try:
        success = node.trigger()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
