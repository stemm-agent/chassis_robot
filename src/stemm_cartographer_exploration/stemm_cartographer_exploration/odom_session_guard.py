#!/usr/bin/env python3
"""Gate Cartographer startup on fresh, continuous, stationary fused odometry."""

from collections import deque
from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .odom_session_state import (
    normalize_angle,
    OdomStabilityWindow,
    Pose2D,
)


@dataclass(frozen=True)
class PendingOdom:
    message: Odometry
    stamp_sec: float
    received_monotonic: float


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_from_quaternion(quaternion) -> float:
    norm = math.sqrt(sum(value * value for value in (
        quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return float('nan')
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_is_finite_and_normalized(quaternion) -> bool:
    values = (
        quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return False
    norm = math.sqrt(sum(value * value for value in values))
    return abs(norm - 1.0) <= 0.01


class OdomSessionGuard(Node):
    """Verify the existing EKF session without changing its absolute origin."""

    def __init__(self) -> None:
        super().__init__('stemm_cartographer_odom_session_guard')
        defaults = {
            'odom_topic': '/odom_combined',
            'odom_frame': 'odom_combined',
            'base_frame': 'base_footprint',
            'cmd_vel_topic': '/cmd_vel',
            'timeout_sec': 10.0,
            'max_message_age_sec': 0.35,
            'max_tf_age_sec': 0.35,
            'stable_sec': 0.8,
            'position_drift_m': 0.025,
            'yaw_drift_rad': 0.0523598776,
            'linear_speed_mps': 0.02,
            'angular_speed_rps': 0.05,
            'position_step_m': 0.10,
            'yaw_step_rad': 0.1745329252,
            'pose_consistency_position_m': 0.03,
            'pose_consistency_yaw_rad': 0.0523598776,
            'required_advancing_messages': 5,
            'odom_queue_depth': 120,
            'ready_confirmation_sec': 0.15,
            'ready_confirmation_messages': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value).lstrip('/')
        self.base_frame = str(self.get_parameter('base_frame').value).lstrip('/')
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.max_message_age_sec = float(
            self.get_parameter('max_message_age_sec').value)
        self.max_tf_age_sec = float(self.get_parameter('max_tf_age_sec').value)
        self.required_advancing_messages = int(
            self.get_parameter('required_advancing_messages').value)
        self.odom_queue_depth = int(
            self.get_parameter('odom_queue_depth').value)
        self.ready_confirmation_sec = float(
            self.get_parameter('ready_confirmation_sec').value)
        self.ready_confirmation_messages = int(
            self.get_parameter('ready_confirmation_messages').value)
        self.pose_consistency_position_m = float(
            self.get_parameter('pose_consistency_position_m').value)
        self.pose_consistency_yaw_rad = float(
            self.get_parameter('pose_consistency_yaw_rad').value)
        if self.required_advancing_messages <= 0:
            raise ValueError('required_advancing_messages must be positive')
        if self.odom_queue_depth < self.required_advancing_messages:
            raise ValueError(
                'odom_queue_depth must be at least required_advancing_messages')
        if self.ready_confirmation_sec <= 0.0:
            raise ValueError('ready_confirmation_sec must be positive')
        if self.ready_confirmation_messages <= 0:
            raise ValueError('ready_confirmation_messages must be positive')

        self.window = OdomStabilityWindow(
            stable_sec=float(self.get_parameter('stable_sec').value),
            position_drift_m=float(
                self.get_parameter('position_drift_m').value),
            yaw_drift_rad=float(self.get_parameter('yaw_drift_rad').value),
            linear_speed_mps=float(
                self.get_parameter('linear_speed_mps').value),
            angular_speed_rps=float(
                self.get_parameter('angular_speed_rps').value),
            position_step_m=float(
                self.get_parameter('position_step_m').value),
            yaw_step_rad=float(self.get_parameter('yaw_step_rad').value),
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pending_odom = deque()
        self.last_received_odom_stamp = None
        self.last_odom_advance_time = None
        self.odom_stamp_regressed = False
        self.odom_stamp_repeated = False
        self.odom_queue_overflow = False
        self.publisher_gid = None
        self.latest_tf_stamp = None
        self.latest_tf_advance_time = None
        self.last_validated_odom_stamp = None
        self.valid_advancing_messages = 0
        self.outer_cycle = 0
        self.ready_candidate = None
        self.ready_since = None
        self.ready_started_cycle = None
        self.ready_confirmation_count = 0
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos)
        self.zero_publisher = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_log_time = 0.0

    def _on_odom(self, message: Odometry) -> None:
        stamp = _stamp_seconds(message.header.stamp)
        now = time.monotonic()
        if (self.last_received_odom_stamp is None or
                stamp > self.last_received_odom_stamp):
            self.last_received_odom_stamp = stamp
            self.last_odom_advance_time = now
            if len(self.pending_odom) >= self.odom_queue_depth:
                self.odom_queue_overflow = True
                self.pending_odom.popleft()
            self.pending_odom.append(PendingOdom(message, stamp, now))
        elif stamp < self.last_received_odom_stamp:
            self.odom_stamp_regressed = True
        else:
            self.odom_stamp_repeated = True

    def _publish_zero(self) -> None:
        self.zero_publisher.publish(Twist())

    def _ros_now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _values_are_finite(message: Odometry) -> bool:
        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
            *message.pose.covariance,
            *message.twist.covariance,
        )
        return (
            all(math.isfinite(value) for value in values) and
            _quaternion_is_finite_and_normalized(pose.orientation)
        )

    @staticmethod
    def _gid(endpoint_info):
        return bytes(endpoint_info.endpoint_gid)

    def _publisher_ready(self):
        publishers = self.get_publishers_info_by_topic(self.odom_topic)
        if len(publishers) == 0:
            return False, f'waiting for publisher on {self.odom_topic}', False
        if len(publishers) != 1:
            return False, (
                f'{self.odom_topic} must have exactly one publisher, '
                f'found {len(publishers)}'), True
        gid = self._gid(publishers[0])
        if self.publisher_gid is None:
            self.publisher_gid = gid
        elif gid != self.publisher_gid:
            return False, (
                f'{self.odom_topic} publisher GID changed during startup'), True
        return True, '', False

    def _latest_tf_ready(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            return False, f'waiting for latest fused odometry TF: {exc}', False

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        if not _quaternion_is_finite_and_normalized(rotation):
            return False, 'latest fused odometry TF quaternion is invalid', True
        if not all(math.isfinite(value) for value in (
                translation.x, translation.y, translation.z)):
            return False, 'latest fused odometry TF translation is invalid', True

        stamp = _stamp_seconds(transform.header.stamp)
        now_ros = self._ros_now_seconds()
        age = now_ros - stamp
        if stamp <= 0.0:
            return False, (
                'latest fused odometry TF has a zero/static timestamp'), True
        if age < -0.10:
            return False, f'latest fused odometry TF is {-age:.3f}s in future', False
        if age > self.max_tf_age_sec:
            return False, f'latest fused odometry TF is stale by {age:.3f}s', False

        now_mono = time.monotonic()
        if self.latest_tf_stamp is None or stamp > self.latest_tf_stamp:
            self.latest_tf_stamp = stamp
            self.latest_tf_advance_time = now_mono
        elif stamp < self.latest_tf_stamp:
            return False, 'latest fused odometry TF timestamp regressed', True
        elif (self.latest_tf_advance_time is None or
              now_mono - self.latest_tf_advance_time > self.max_tf_age_sec):
            return False, 'latest fused odometry TF timestamp stopped advancing', False
        return True, '', False

    def _validate_pending(self, pending: PendingOdom):
        message = pending.message
        if message.header.frame_id.lstrip('/') != self.odom_frame:
            return None, (
                f'{self.odom_topic} frame is {message.header.frame_id!r}, '
                f'expected {self.odom_frame!r}'), True, False
        if message.child_frame_id.lstrip('/') != self.base_frame:
            return None, (
                f'{self.odom_topic} child frame is {message.child_frame_id!r}, '
                f'expected {self.base_frame!r}'), True, False
        if not self._values_are_finite(message):
            return None, (
                f'{self.odom_topic} contains invalid numeric data'), True, False

        message_age = self._ros_now_seconds() - pending.stamp_sec
        if message_age < -0.10:
            return None, (
                f'{self.odom_topic} timestamp is {-message_age:.3f}s in future'
            ), False, False
        if message_age > self.max_message_age_sec:
            return None, (
                f'dropping unpaired stale {self.odom_topic} message: '
                f'age={message_age:.3f}s'), False, True

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            return None, (
                f'waiting for timestamp-aligned fused odometry TF: {exc}'
            ), False, False

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        if not _quaternion_is_finite_and_normalized(rotation):
            return None, 'fused odometry TF quaternion is invalid', True, False
        if not all(math.isfinite(value) for value in (
                translation.x, translation.y, translation.z)):
            return None, 'fused odometry TF translation is invalid', True, False

        pose = Pose2D(
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(rotation),
        )
        message_pose = Pose2D(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _yaw_from_quaternion(message.pose.pose.orientation),
        )
        consistency_position = math.hypot(
            pose.x - message_pose.x,
            pose.y - message_pose.y,
        )
        consistency_yaw = abs(normalize_angle(pose.yaw - message_pose.yaw))
        if (consistency_position > self.pose_consistency_position_m or
                consistency_yaw > self.pose_consistency_yaw_rad):
            return None, (
                'timestamp-aligned odometry message/TF mismatch: '
                f'{consistency_position:.3f}m/'
                f'{math.degrees(consistency_yaw):.1f}deg'), True, False

        linear_speed = math.hypot(
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
        )
        return (
            pose,
            linear_speed,
            float(message.twist.twist.angular.z),
        ), '', False, False

    def _reset_stability(self) -> None:
        self.window.reset_stability()
        self.valid_advancing_messages = 0
        self._reset_ready_confirmation()

    def _reset_ready_confirmation(self) -> None:
        self.ready_candidate = None
        self.ready_since = None
        self.ready_started_cycle = None
        self.ready_confirmation_count = 0

    def _log_wait(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_log_time >= 1.0:
            self.get_logger().info(message)
            self.last_log_time = now

    def _spin_callbacks(self) -> None:
        # Drain enough callbacks to keep 30 Hz odom and TF streams aligned.
        rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(31):
            rclpy.spin_once(self, timeout_sec=0.0)

    def _fail(self, message: str) -> int:
        self._publish_zero()
        self.get_logger().error(message)
        return 1

    def run(self) -> int:
        deadline = time.monotonic() + self.timeout_sec
        last_message = 'waiting for fused odometry'
        while rclpy.ok() and time.monotonic() < deadline:
            self.outer_cycle += 1
            self._spin_callbacks()
            self._publish_zero()

            if self.odom_queue_overflow:
                return self._fail(
                    f'{self.odom_topic} validation queue overflowed')
            if self.odom_stamp_regressed:
                return self._fail(f'{self.odom_topic} timestamp regressed')
            if self.odom_stamp_repeated:
                return self._fail(f'{self.odom_topic} timestamp repeated')

            ready, reason, fatal = self._publisher_ready()
            if not ready:
                if fatal:
                    return self._fail(reason)
                self._reset_stability()
                last_message = reason
                self._log_wait(reason)
                continue

            ready, reason, fatal = self._latest_tf_ready()
            if not ready:
                if fatal:
                    return self._fail(reason)
                self._reset_stability()
                last_message = reason
                self._log_wait(reason)
                continue

            if (self.last_odom_advance_time is None or
                    time.monotonic() - self.last_odom_advance_time >
                    self.max_message_age_sec):
                self._reset_stability()
                last_message = f'{self.odom_topic} timestamp stopped advancing'
                self._log_wait(last_message)
                continue

            processed = 0
            while self.pending_odom and processed < 64:
                pending = self.pending_odom[0]
                sample, reason, fatal, drop = self._validate_pending(pending)
                if fatal:
                    return self._fail(reason)
                if drop:
                    self.pending_odom.popleft()
                    self._reset_stability()
                    last_message = reason
                    processed += 1
                    continue
                if sample is None:
                    last_message = reason
                    break

                self.pending_odom.popleft()
                processed += 1
                if (self.last_validated_odom_stamp is not None and
                        pending.stamp_sec <= self.last_validated_odom_stamp):
                    return self._fail(
                        'validated fused odometry timestamp did not advance')
                self.last_validated_odom_stamp = pending.stamp_sec

                pose, linear_speed, angular_speed = sample
                observation = self.window.observe(
                    now=pending.received_monotonic,
                    pose=pose,
                    linear_speed=linear_speed,
                    angular_speed=angular_speed,
                )
                last_message = observation.reason
                if observation.fatal:
                    return self._fail(observation.reason)
                if observation.window_restarted:
                    self.valid_advancing_messages = (
                        1 if observation.sample_accepted else 0)
                elif observation.sample_accepted:
                    self.valid_advancing_messages += 1

                if self.valid_advancing_messages < self.required_advancing_messages:
                    self._reset_ready_confirmation()
                    last_message = (
                        'waiting for validated advancing fused odometry messages: '
                        f'{self.valid_advancing_messages}/'
                        f'{self.required_advancing_messages}')
                elif observation.ready:
                    if self.ready_since is None:
                        self.ready_since = time.monotonic()
                        self.ready_started_cycle = self.outer_cycle
                    self.ready_confirmation_count += 1
                    self.ready_candidate = (pose, observation)
                else:
                    self._reset_ready_confirmation()

            confirmation_elapsed = (
                0.0 if self.ready_since is None
                else time.monotonic() - self.ready_since)
            confirmed_on_later_cycle = (
                self.ready_started_cycle is not None and
                self.outer_cycle > self.ready_started_cycle)
            if (self.ready_candidate is not None and
                    not self.pending_odom and
                    confirmed_on_later_cycle and
                    confirmation_elapsed >= self.ready_confirmation_sec and
                    self.ready_confirmation_count >=
                    self.ready_confirmation_messages):
                pose, observation = self.ready_candidate
                self._publish_zero()
                self.get_logger().info(
                    'fused odometry session ready without resetting origin: '
                    f'x={pose.x:.3f}m y={pose.y:.3f}m '
                    f'yaw={math.degrees(pose.yaw):.1f}deg; '
                    f'stable={observation.stable_for_sec:.2f}s')
                return 0

            self._log_wait(last_message)

        return self._fail(
            'fused odometry session readiness timed out after '
            f'{self.timeout_sec:.1f}s: {last_message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomSessionGuard()
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()
