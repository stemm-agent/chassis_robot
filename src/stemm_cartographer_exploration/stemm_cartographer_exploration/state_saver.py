#!/usr/bin/env python3
"""Save the live Cartographer trajectory to an independent PBStream file."""

from pathlib import Path
import time

from cartographer_ros_msgs.srv import WriteState
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class CartographerStateSaver(Node):
    def __init__(self) -> None:
        super().__init__('stemm_cartographer_state_saver')
        self.declare_parameter(
            'state_filename',
            '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/'
            'stemm_cartographer_map.pbstream')
        self.declare_parameter('write_state_service', '/write_state')
        self.declare_parameter('timeout_sec', 30.0)
        self.state_filename = str(self.get_parameter('state_filename').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        write_service = str(self.get_parameter('write_state_service').value)
        callback_group = ReentrantCallbackGroup()
        self.write_client = self.create_client(
            WriteState, write_service, callback_group=callback_group)
        self.create_service(
            Trigger,
            '/stemm_cartographer/save_state',
            self._save_state,
            callback_group=callback_group,
        )
        self.get_logger().info(
            f'PBStream saver ready: {self.state_filename}')

    def _save_state(self, request, response):
        del request
        if not self.write_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = 'Cartographer /write_state service is not ready'
            return response

        filename = str(Path(self.state_filename).expanduser())
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        write_request = WriteState.Request()
        write_request.filename = filename
        write_request.include_unfinished_submaps = True
        future = self.write_client.call_async(write_request)
        deadline = time.monotonic() + self.timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            response.success = False
            response.message = 'Cartographer write_state timed out'
            return response
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Cartographer write_state failed: {exc}'
            return response

        response.success = result.status.code == 0
        response.message = (
            f'Cartographer state saved: {filename}'
            if response.success
            else f'Cartographer write_state rejected: {result.status.message}'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartographerStateSaver()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
