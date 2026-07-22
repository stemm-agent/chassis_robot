#!/usr/bin/env python3
"""Expose raw and RRT-normalized Cartographer maps through GetMap."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def normalize_occupancy_values(values, occupied_threshold):
    """Convert Cartographer probabilities to the legacy RRT -1/0/100 model."""
    threshold = max(1, min(100, int(occupied_threshold)))
    return [
        -1 if value < 0 else 100 if value >= threshold else 0
        for value in values
    ]


class MapServiceAdapter(Node):
    def __init__(self) -> None:
        super().__init__('stemm_cartographer_map_service_adapter')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'map_service', '/stemm_cartographer/dynamic_map')
        self.declare_parameter(
            'rrt_map_topic', '/stemm_cartographer/rrt_map')
        self.declare_parameter(
            'rrt_map_service', '/stemm_cartographer/rrt_dynamic_map')
        self.declare_parameter('rrt_occupied_threshold', 50)
        map_topic = str(self.get_parameter('map_topic').value)
        map_service = str(self.get_parameter('map_service').value)
        rrt_map_topic = str(self.get_parameter('rrt_map_topic').value)
        rrt_map_service = str(self.get_parameter('rrt_map_service').value)
        self.rrt_occupied_threshold = int(
            self.get_parameter('rrt_occupied_threshold').value)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest_map = None
        self.latest_rrt_map = None
        self._reported_first_rrt_map = False
        self.rrt_map_publisher = self.create_publisher(
            OccupancyGrid, rrt_map_topic, qos)
        self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, qos)
        self.create_service(GetMap, map_service, self._get_map)
        self.create_service(GetMap, rrt_map_service, self._get_rrt_map)
        self.get_logger().info(
            f'caching raw {map_topic} at {map_service}; publishing normalized '
            f'RRT map {rrt_map_topic} at {rrt_map_service} '
            f'(occupied>={self.rrt_occupied_threshold})')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        normalized = OccupancyGrid()
        normalized.header = msg.header
        normalized.info = msg.info
        normalized.data = normalize_occupancy_values(
            msg.data, self.rrt_occupied_threshold)
        self.latest_rrt_map = normalized
        self.rrt_map_publisher.publish(normalized)
        if not self._reported_first_rrt_map:
            unknown = sum(value < 0 for value in normalized.data)
            occupied = sum(value == 100 for value in normalized.data)
            total = len(normalized.data)
            self.get_logger().info(
                'first RRT map normalized: '
                f'cells={total}, unknown={unknown}, occupied={occupied}, '
                f'free={max(0, total - unknown - occupied)}')
            self._reported_first_rrt_map = True

    def _get_map(self, request, response):
        del request
        if self.latest_map is None:
            self.get_logger().warn(
                'GetMap requested before the first raw /map update')
            return response
        response.map = self.latest_map
        return response

    def _get_rrt_map(self, request, response):
        del request
        if self.latest_rrt_map is None:
            self.get_logger().warn(
                'RRT GetMap requested before the first normalized map update')
            return response
        response.map = self.latest_rrt_map
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapServiceAdapter()
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
