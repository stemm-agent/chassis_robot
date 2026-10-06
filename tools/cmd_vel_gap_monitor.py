#!/usr/bin/env python3
"""Monitor /cmd_vel gaps and zero-command intervals during Nav2 runs."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelGapMonitor(Node):
    def __init__(self, topic: str, gap_sec: float, zero_sec: float) -> None:
        super().__init__('cmd_vel_gap_monitor')
        self.topic = topic
        self.gap_sec = gap_sec
        self.zero_sec = zero_sec
        self.last_msg_time = None
        self.last_nonzero_time = None
        self.gap_started_at = None
        self.zero_started_at = None
        self.msg_count = 0
        self.zero_count = 0
        self.create_subscription(Twist, topic, self.on_twist, 50)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f'monitoring {topic}; gap>{gap_sec:.2f}s, zero>{zero_sec:.2f}s')

    @staticmethod
    def speed_norm(msg: Twist) -> float:
        return math.hypot(msg.linear.x, msg.linear.y) + abs(msg.angular.z)

    def on_twist(self, msg: Twist) -> None:
        now = time.monotonic()
        self.msg_count += 1

        if self.last_msg_time is not None:
            dt = now - self.last_msg_time
            if dt > self.gap_sec:
                self.get_logger().warn(
                    f'CMD_GAP between messages: {dt:.3f}s; '
                    f'current=({msg.linear.x:.3f}, {msg.angular.z:.3f})')
        self.last_msg_time = now

        is_zero = self.speed_norm(msg) < 1e-4
        if is_zero:
            self.zero_count += 1
            if self.zero_started_at is None:
                self.zero_started_at = now
        else:
            if self.zero_started_at is not None:
                duration = now - self.zero_started_at
                if duration > self.zero_sec:
                    self.get_logger().warn(f'ZERO_END duration={duration:.3f}s')
            self.zero_started_at = None
            self.last_nonzero_time = now

        self.get_logger().info(
            f'cmd #{self.msg_count}: linear.x={msg.linear.x:.3f}, '
            f'angular.z={msg.angular.z:.3f}, zero={is_zero}')

    def on_timer(self) -> None:
        now = time.monotonic()
        if self.last_msg_time is None:
            return

        gap = now - self.last_msg_time
        if gap > self.gap_sec and self.gap_started_at is None:
            self.gap_started_at = self.last_msg_time
            self.get_logger().warn(f'CMD_GAP_START no message for {gap:.3f}s')
        elif gap <= self.gap_sec and self.gap_started_at is not None:
            duration = now - self.gap_started_at
            self.get_logger().warn(f'CMD_GAP_END duration={duration:.3f}s')
            self.gap_started_at = None

        if self.zero_started_at is not None:
            zero_duration = now - self.zero_started_at
            if zero_duration > self.zero_sec:
                self.get_logger().warn(
                    f'ZERO_ACTIVE duration={zero_duration:.3f}s; '
                    f'zero_msgs={self.zero_count}, total_msgs={self.msg_count}')
                self.zero_started_at = now


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--gap-sec', type=float, default=0.35)
    parser.add_argument('--zero-sec', type=float, default=0.35)
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelGapMonitor(args.topic, args.gap_sec, args.zero_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
