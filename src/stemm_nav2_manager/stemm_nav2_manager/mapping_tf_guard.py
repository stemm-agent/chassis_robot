#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class MappingTfGuard(Node):
    def __init__(self):
        super().__init__('mapping_tf_guard')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('max_tf_age_sec', 0.30)
        self.declare_parameter('position_tolerance', 0.10)
        self.declare_parameter('yaw_tolerance', math.radians(5.0))
        self.declare_parameter('stable_sec', 1.5)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_topic = self.get_parameter('map_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.max_tf_age_sec = float(
            self.get_parameter('max_tf_age_sec').value)
        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(
            self.get_parameter('yaw_tolerance').value)
        self.stable_sec = float(self.get_parameter('stable_sec').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos)

        self.started_at = time.monotonic()
        self.stable_since = None
        self.map_ready = False
        self.last_stamp_ns = None
        self.saw_stamp_advance = False
        self.last_log_time = 0.0
        self.exit_code = None
        self.timer = self.create_timer(0.1, self.poll)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def map_callback(self, msg):
        self.map_ready = (
            msg.info.width > 0
            and msg.info.height > 0
            and len(msg.data) > 0)

    def finish(self, code, message):
        if self.exit_code is not None:
            return
        self.exit_code = code
        if code == 0:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        self.timer.cancel()

    def waiting(self, now, message):
        self.stable_since = None
        if now - self.last_log_time > 1.0:
            self.get_logger().info(message)
            self.last_log_time = now

    def poll(self):
        now = time.monotonic()
        if now - self.started_at > self.timeout_sec:
            self.finish(
                2,
                'mapping TF health check timed out; refusing to start '
                'navigation and exploration')
            return

        if not self.map_ready:
            self.waiting(now, f'waiting for a valid {self.map_topic} message')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.waiting(now, f'waiting for mapping TF: {exc}')
            return

        stamp = Time.from_msg(transform.header.stamp)
        stamp_ns = stamp.nanoseconds
        if self.last_stamp_ns is not None and stamp_ns > self.last_stamp_ns:
            self.saw_stamp_advance = True
        self.last_stamp_ns = stamp_ns
        tf_age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = self.yaw_from_quaternion(transform.transform.rotation)
        position_error = math.hypot(x, y)
        healthy = (
            self.saw_stamp_advance
            and -0.05 <= tf_age <= self.max_tf_age_sec
            and position_error <= self.position_tolerance
            and abs(yaw) <= self.yaw_tolerance)
        if not healthy:
            self.waiting(
                now,
                'waiting for fresh origin-aligned mapping TF: '
                f'x={x:.3f}m, y={y:.3f}m, '
                f'yaw={math.degrees(yaw):.1f}deg, age={tf_age:.3f}s, '
                f'advancing={self.saw_stamp_advance}')
            return

        if self.stable_since is None:
            self.stable_since = now
            self.get_logger().info(
                'mapping TF is healthy; verifying stability before startup')
            return
        if now - self.stable_since >= self.stable_sec:
            self.finish(
                0,
                'mapping TF health check passed: '
                f'x={x:.3f}m, y={y:.3f}m, '
                f'yaw={math.degrees(yaw):.1f}deg, age={tf_age:.3f}s')


def main(args=None):
    rclpy.init(args=args)
    node = MappingTfGuard()
    while rclpy.ok() and node.exit_code is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    code = node.exit_code if node.exit_code is not None else 1
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)
