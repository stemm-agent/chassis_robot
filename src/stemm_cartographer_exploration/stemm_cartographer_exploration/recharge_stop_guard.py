#!/usr/bin/env python3
"""Gracefully stop the current recharge task before systemd teardown."""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


SERVICE_NAME = '/auto_recharge/stop'
SERVICE_WAIT_SEC = 6.0
RESPONSE_TIMEOUT_SEC = 3.0


def request_recharge_stop() -> int:
    rclpy.init()
    node = Node('stemm_cartographer_recharge_stop_guard')
    client = node.create_client(Trigger, SERVICE_NAME)
    return_code = 1
    try:
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            node.get_logger().error(
                f'{SERVICE_NAME} was unavailable after {SERVICE_WAIT_SEC:.1f}s')
            return return_code

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + RESPONSE_TIMEOUT_SEC
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not future.done():
            future.cancel()
            node.get_logger().error(
                f'{SERVICE_NAME} did not respond within '
                f'{RESPONSE_TIMEOUT_SEC:.1f}s')
            return return_code

        response = future.result()
        if response is None or not response.success:
            message = '<no response>' if response is None else response.message
            node.get_logger().error(
                f'{SERVICE_NAME} rejected the stop request: {message}')
            return return_code

        node.get_logger().info(
            f'{SERVICE_NAME} confirmed recharge cancellation and zero motion')
        return_code = 0
        return return_code
    except Exception as exc:  # pragma: no cover - defensive ROS boundary
        node.get_logger().error(f'{SERVICE_NAME} call failed: {exc}')
        return return_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raise SystemExit(request_recharge_stop())


if __name__ == '__main__':
    main()
