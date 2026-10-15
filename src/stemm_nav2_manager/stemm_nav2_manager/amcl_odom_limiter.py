#!/usr/bin/env python3
"""Sole publisher of the map -> odom_combined transform for AMCL.

2026-09-13 design note
----------------------
AMCL runs with ``tf_broadcast: false`` and publishes ``/amcl_pose`` (RELIABLE +
TRANSIENT_LOCAL).  This node converts every AMCL pose into the
``map -> odom_combined`` transform implied by the odometry *at the pose stamp*:

    map -> odom_combined  =  (map -> base_footprint)_amcl  *  inverse(odom_combined -> base_footprint)

It then moves the published transform toward that value with a bounded rate
(``max_translation_rate`` m/s, ``max_yaw_rate`` rad/s).  A large single-step AMCL
correction - the long-corridor mode jump - therefore reaches Nav2 as a short
continuous ramp instead of a step, while the transform is still published on
every tick, so map->odom_combined is never held frozen and Nav2 can always
integrate it.

Odometry is treated as the trusted short-term displacement source: between AMCL
corrections the target is unchanged, so map->odom_combined stays constant and the
map pose follows odom exactly (the chassis odometry of the heavy base drifts
slowly, so this is the intended behaviour).

Deliberately *not* implemented here (separate decision, separate layer): anomaly
detection, correction veto, mode holds, interlocks, anchors and rebase auditing.
This node only limits the rate at which a correction may be applied.
"""

import math
import sys

import rclpy
try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    ExternalShutdownException = KeyboardInterrupt
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import (Buffer, TransformBroadcaster, TransformException,
                     TransformListener)


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def wrap_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def compose(a, b):
    """Return the 2D transform ``a * b`` (apply a, then b)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    cos_yaw, sin_yaw = math.cos(ayaw), math.sin(ayaw)
    return (ax + cos_yaw * bx - sin_yaw * by,
            ay + sin_yaw * bx + cos_yaw * by,
            wrap_angle(ayaw + byaw))


def inverse(a):
    """Return the inverse of the 2D transform ``a``."""
    ax, ay, ayaw = a
    cos_yaw, sin_yaw = math.cos(ayaw), math.sin(ayaw)
    return (-(cos_yaw * ax + sin_yaw * ay),
            -(-sin_yaw * ax + cos_yaw * ay),
            wrap_angle(-ayaw))


class AmclOdomLimiter(Node):
    def __init__(self):
        super().__init__('amcl_odom_limiter')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_translation_rate', 0.25)
        self.declare_parameter('max_yaw_rate', 0.35)
        # 2026-09-28: raised from 2.0; the stale-pose refresh keeps the age in the
        # 0.5-5 s band, so 2 s produced false alarms on a healthy system.
        # 2026-10-12 B：8.0 -> 15.0。AMCL 静止时没有内部定时发布
        # （publishAmclPose 只由 resampled/force_publication 触发），
        # 实测 /amcl_pose 周期恒为 5.08~5.17 s，对 8.0 s 只剩约 2.9 s
        # 余量；17:37 那轮 AMCL 稍慢即被判 stale，进而让 adaptor 锁死
        # interlock、有图导航栈 navigation_health_timeout 后被清掉。
        # 这里与 launch（wheeltec_nav2.launch.py）保持一致，
        # 避免 launch 传参丢失时退回边沿竞态。
        self.declare_parameter('warn_pose_age_sec', 15.0)
        self.declare_parameter('status_topic', '~/status')
        self.declare_parameter('nomotion_update_service', '/request_nomotion_update')
        self.declare_parameter('nomotion_first_delay_sec', 1.0)
        self.declare_parameter('nomotion_retry_sec', 2.0)
        # 2026-09-28: the budget must not run out while the pose is stale (the
        # request-age trigger below asks every nomotion_retry_sec).
        self.declare_parameter('nomotion_max_attempts', 100000)
        # 2026-09-28: the budget is no longer a one-shot allowance.
        self.declare_parameter('nomotion_rearm_sec', 10.0)
        # 2026-09-28: ask for a refresh when the last pose is older than this.
        # 2026-10-12 B：5.0 -> 1.5。实测把本节点看到的 pose_age 从
        # 3~5 s 压到 2.0~2.4 s（运行时改参实测，随后恢复）。
        # 与 launch 保持一致。
        self.declare_parameter('nomotion_request_age_sec', 1.5)
        self.declare_parameter('amcl_node_name', 'amcl')
        self.declare_parameter('amcl_absence_shutdown_sec', 15.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        pose_topic = self.get_parameter('pose_topic').value
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.max_translation_rate = max(
            0.0, float(self.get_parameter('max_translation_rate').value))
        self.max_yaw_rate = max(0.0, float(self.get_parameter('max_yaw_rate').value))
        self.warn_pose_age_sec = max(
            0.0, float(self.get_parameter('warn_pose_age_sec').value))
        status_topic = self.get_parameter('status_topic').value
        nomotion_service = self.get_parameter('nomotion_update_service').value
        self.nomotion_first_delay_sec = max(
            0.0, float(self.get_parameter('nomotion_first_delay_sec').value))
        self.nomotion_retry_sec = max(
            0.1, float(self.get_parameter('nomotion_retry_sec').value))
        self.nomotion_max_attempts = max(
            0, int(self.get_parameter('nomotion_max_attempts').value))
        self.nomotion_rearm_sec = max(
            0.0, float(self.get_parameter('nomotion_rearm_sec').value))
        self.nomotion_request_age_sec = max(
            0.0, float(self.get_parameter('nomotion_request_age_sec').value))
        self.amcl_node_name = self.get_parameter('amcl_node_name').value
        if not self.amcl_node_name.startswith('/'):
            self.amcl_node_name = '/' + self.amcl_node_name
        self.amcl_absence_shutdown_sec = max(
            0.0, float(self.get_parameter('amcl_absence_shutdown_sec').value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.published = None          # (x, y, yaw) map -> odom_combined, rate limited
        self.target = None             # (x, y, yaw) map -> odom_combined, from AMCL
        self.pose_stamp = None
        self.pose_received_at = None
        self.clipped = 0.0
        self.odom_lookup_fallbacks = 0
        self.last_tick = None
        self.last_warn_at = None
        self.started_at = self.get_clock().now()
        self.nomotion_attempts = 0
        self.last_nomotion_at = None
        self.amcl_seen = False
        self.amcl_missing_since = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self.pose_callback, latched)
        self.status_publisher = self.create_publisher(
            Float32MultiArray, status_topic, 10)
        self.nomotion_client = self.create_client(EmptySrv, nomotion_service)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)
        # A SIGKILLed nav2 leaves this node alive.  Without this check it would
        # keep broadcasting a stale map -> odom_combined for the next session.
        self.presence_timer = self.create_timer(2.0, self.check_amcl_presence)

        self.get_logger().info(
            '%s -> %s limiter ready: max %.2f m/s, %.2f rad/s, %s at %.0f Hz'
            % (self.map_frame, self.odom_frame, self.max_translation_rate,
               self.max_yaw_rate, pose_topic, self.publish_rate))

    def lookup_odom_base(self, stamp):
        """Return (x, y, yaw) of odom_frame -> base_frame at ``stamp``."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp)
        except TransformException:
            try:
                # Extrapolation or a stamp outside the buffer: fall back to the
                # newest odometry sample, which is only a few milliseconds newer.
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time())
            except TransformException:
                return None
            self.odom_lookup_fallbacks += 1
        return (transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(transform.transform.rotation))

    def pose_callback(self, message):
        odom_base = self.lookup_odom_base(Time.from_msg(message.header.stamp))
        if odom_base is None:
            self.get_logger().warn(
                'Cannot compute %s -> %s: no %s -> %s transform available yet.'
                % (self.map_frame, self.odom_frame, self.odom_frame,
                   self.base_frame),
                throttle_duration_sec=5.0)
            return
        map_base = (message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    yaw_from_quaternion(message.pose.pose.orientation))
        self.target = compose(map_base, inverse(odom_base))
        self.pose_stamp = message.header.stamp
        self.pose_received_at = self.get_clock().now()
        # 2026-09-28: a fresh pose restores the full nomotion budget.
        if self.nomotion_attempts:
            self.nomotion_attempts = 0

    def request_nomotion_update(self, now):
        """Ask AMCL for one filter update.

        While the robot is stationary AMCL does not run an update, so a freshly
        seeded filter may never publish /amcl_pose and the map frame would stay
        missing until the robot moves.  The nomotion service is the official way
        to force a single update.
        """
        if self.nomotion_attempts >= self.nomotion_max_attempts:
            # 2026-09-28: a stale pose must not become permanent. Once the per-spell
            # budget is spent, re-arm it on a slow period so AMCL is asked again.
            if self.nomotion_rearm_sec <= 0.0 or self.last_nomotion_at is None:
                return
            since_last = (now - self.last_nomotion_at).nanoseconds * 1e-9
            if since_last < self.nomotion_rearm_sec:
                return
            self.get_logger().info(
                'Re-arming the AMCL nomotion budget after %.0fs without a fresh pose.'
                % since_last)
            self.nomotion_attempts = 0
        elapsed = (now - self.started_at).nanoseconds * 1e-9
        if elapsed < self.nomotion_first_delay_sec:
            return
        if self.last_nomotion_at is not None:
            since = (now - self.last_nomotion_at).nanoseconds * 1e-9
            if since < self.nomotion_retry_sec:
                return
        self.last_nomotion_at = now
        if not self.nomotion_client.service_is_ready():
            return
        self.nomotion_attempts += 1
        self.nomotion_client.call_async(EmptySrv.Request())
        self.get_logger().info(
            'Requested a nomotion update from AMCL (attempt %d).'
            % self.nomotion_attempts)

    def check_amcl_presence(self):
        """Shut down instead of broadcasting a stale map -> odom_combined.

        nav2 is sometimes killed with SIGKILL, which leaves this node alive while
        the container (and AMCL) is gone.  A leftover publisher of map -> odom
        from a previous session is worse than no publisher at all.
        """
        now = self.get_clock().now()
        # Discovery of node *names* proved unreliable in this DDS setup
        # (AMCL was reported missing while it was demonstrably serving
        # requests), so treat a matched service as proof of presence too.
        if self.nomotion_client.service_is_ready() or \
                self.amcl_node_name in self.get_node_names():
            self.amcl_seen = True
            self.amcl_missing_since = None
            return
        elapsed = (now - self.started_at).nanoseconds * 1e-9
        grace = max(self.amcl_absence_shutdown_sec, 20.0)
        if not self.amcl_seen and elapsed < grace:
            return
        if self.amcl_missing_since is None:
            self.amcl_missing_since = now
            self.get_logger().warn(
                'AMCL node %s is gone; will stop publishing %s -> %s in %.0fs.'
                % (self.amcl_node_name, self.map_frame, self.odom_frame,
                   self.amcl_absence_shutdown_sec))
            return
        missing = (now - self.amcl_missing_since).nanoseconds * 1e-9
        if missing >= self.amcl_absence_shutdown_sec:
            self.get_logger().error(
                'AMCL node %s absent for %.0fs; shutting down so that no stale '
                '%s -> %s transform stays on the wire.'
                % (self.amcl_node_name, missing, self.map_frame, self.odom_frame))
            rclpy.shutdown()

    def tick(self):
        now = self.get_clock().now()
        if self.last_tick is None:
            self.last_tick = now
            dt = 1.0 / self.publish_rate
        else:
            dt = (now - self.last_tick).nanoseconds * 1e-9
            self.last_tick = now
        dt = min(max(dt, 1e-3), 0.5)

        # 2026-09-28: keep the pose fresh.  This request used to happen only while
        # self.target was still None (the cold start), so once AMCL had published
        # once the limiter never asked again and the pose age grew without bound
        # (1683 s observed).  request_nomotion_update() self-throttles with
        # nomotion_retry_sec and restores its budget whenever a fresh pose arrives.
        if self.nomotion_request_age_sec > 0.0:
            if self.pose_received_at is None:
                pose_age = float('inf')
            else:
                pose_age = (now - self.pose_received_at).nanoseconds * 1e-9
            if pose_age > self.nomotion_request_age_sec:
                self.request_nomotion_update(now)

        if self.target is None:
            if self.published is None:
                self.request_nomotion_update(now)
                self.get_logger().info(
                    'Waiting for an AMCL pose to publish %s -> %s.'
                    % (self.map_frame, self.odom_frame),
                    throttle_duration_sec=10.0)
                return
            # Target is unchanged while no new pose arrives: keep publishing the
            # same transform so odom carries the map pose between corrections.
        elif self.published is None:
            # First pose defines the frame; there is nothing to limit yet.
            self.published = self.target
            self.clipped = 0.0
        else:
            error = compose(inverse(self.published), self.target)
            distance = math.hypot(error[0], error[1])
            yaw_error = abs(error[2])
            allowed_distance = self.max_translation_rate * dt
            allowed_yaw = self.max_yaw_rate * dt
            if distance <= allowed_distance and yaw_error <= allowed_yaw:
                self.published = self.target
                self.clipped = 0.0
            else:
                ratio_distance = (1.0 if distance <= allowed_distance
                                  else allowed_distance / distance)
                ratio_yaw = 1.0 if yaw_error <= allowed_yaw else allowed_yaw / yaw_error
                self.published = compose(
                    self.published,
                    (error[0] * ratio_distance, error[1] * ratio_distance,
                     error[2] * ratio_yaw))
                self.clipped = 1.0
                if self.last_warn_at is None or (now - self.last_warn_at).nanoseconds > 2e9:
                    self.last_warn_at = now
                    self.get_logger().warn(
                        'Limiting %s -> %s correction: %.3f m / %.1f deg remaining '
                        '(applying %.2f m/s, %.2f rad/s).'
                        % (self.map_frame, self.odom_frame, distance,
                           math.degrees(yaw_error), self.max_translation_rate,
                           self.max_yaw_rate))

        self.publish_transform(now)
        self.publish_status(now)

    def publish_transform(self, now):
        x, y, yaw = self.published
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self, now):
        pose_age = -1.0
        if self.pose_received_at is not None:
            pose_age = (now - self.pose_received_at).nanoseconds * 1e-9
        target = self.target if self.target is not None else (0.0, 0.0, 0.0)
        error_distance = -1.0
        error_yaw = -1.0
        if self.target is not None and self.published is not None:
            error = compose(inverse(self.published), self.target)
            error_distance = math.hypot(error[0], error[1])
            error_yaw = error[2]
        message = Float32MultiArray()
        message.data = [
            self.clipped,
            float(error_distance),
            float(error_yaw),
            float(pose_age),
            float(self.odom_lookup_fallbacks),
            float(self.published[0]), float(self.published[1]), float(self.published[2]),
            float(target[0]), float(target[1]), float(target[2]),
        ]
        self.status_publisher.publish(message)
        if (self.target is not None and self.warn_pose_age_sec > 0.0
                and pose_age > self.warn_pose_age_sec):
            self.get_logger().warn(
                'Last AMCL pose is %.1fs old; map -> odom_combined is being held '
                'constant and the map pose follows odometry.' % pose_age,
                throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = AmclOdomLimiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
