#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import SetPose
from tf2_ros import Buffer, TransformException, TransformListener


class MappingOdomReset(Node):
    def __init__(self):
        super().__init__('mapping_odom_reset')
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('yaw_tolerance', math.radians(3.0))
        self.declare_parameter('stable_sec', 0.8)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.stable_sec = float(self.get_parameter('stable_sec').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_pose_client = self.create_client(SetPose, '/set_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.started_at = time.monotonic()
        self.request_future = None
        self.request_completed = False
        self.stable_since = None
        self.last_log_time = 0.0
        self.exit_code = None
        self.timer = self.create_timer(0.1, self.poll)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def finish(self, code, message):
        if self.exit_code is not None:
            return
        self.exit_code = code
        if code == 0:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        self.cmd_vel_pub.publish(Twist())
        self.timer.cancel()

    def send_reset_request(self):
        try:
            before = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.3))
            q = before.transform.rotation
            self.get_logger().info(
                'mapping session odom before reset: '
                f'x={before.transform.translation.x:.3f}m, '
                f'y={before.transform.translation.y:.3f}m, '
                f'yaw={math.degrees(self.yaw_from_quaternion(q)):.1f}deg')
        except TransformException:
            pass

        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = self.odom_frame
        request.pose.pose.pose.orientation.w = 1.0
        covariance = [0.0] * 36
        covariance[0] = 1.0e-6
        covariance[7] = 1.0e-6
        covariance[14] = 1.0e-6
        covariance[21] = 1.0e-6
        covariance[28] = 1.0e-6
        covariance[35] = 1.0e-6
        request.pose.pose.covariance = covariance
        self.cmd_vel_pub.publish(Twist())
        self.request_future = self.set_pose_client.call_async(request)
        self.get_logger().info(
            'requested mapping-session odom reset; waiting for stable zero TF')

    def poll(self):
        now = time.monotonic()
        if now - self.started_at > self.timeout_sec:
            self.finish(
                2,
                'mapping-session odom reset timed out; refusing to start SLAM')
            return

        if self.request_future is None:
            if not self.set_pose_client.service_is_ready():
                self.set_pose_client.wait_for_service(timeout_sec=0.0)
                if now - self.last_log_time > 1.0:
                    self.get_logger().info('waiting for /set_pose service')
                    self.last_log_time = now
                return
            self.send_reset_request()
            return

        if not self.request_completed:
            if self.request_future.done():
                try:
                    self.request_future.result()
                except Exception as exc:
                    self.finish(3, f'/set_pose failed: {exc}')
                    return
                self.request_completed = True
            # The EKF can apply /set_pose before the DDS service response is
            # delivered. Verify the resulting TF even while the reply is pending.

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            if now - self.last_log_time > 1.0:
                self.get_logger().info(f'waiting for reset TF: {exc}')
                self.last_log_time = now
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = self.yaw_from_quaternion(transform.transform.rotation)
        within_tolerance = (
            math.hypot(x, y) <= self.position_tolerance
            and abs(yaw) <= self.yaw_tolerance)
        if not within_tolerance:
            self.stable_since = None
            if now - self.last_log_time > 1.0:
                self.get_logger().info(
                    f'waiting for odom zero: x={x:.3f}m, y={y:.3f}m, '
                    f'yaw={math.degrees(yaw):.1f}deg')
                self.last_log_time = now
            return
        if self.stable_since is None:
            self.stable_since = now
            return
        if now - self.stable_since >= self.stable_sec:
            self.finish(
                0,
                f'mapping-session odom is stable at zero: x={x:.3f}m, '
                f'y={y:.3f}m, yaw={math.degrees(yaw):.1f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = MappingOdomReset()
    while rclpy.ok() and node.exit_code is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    code = node.exit_code if node.exit_code is not None else 1
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)
