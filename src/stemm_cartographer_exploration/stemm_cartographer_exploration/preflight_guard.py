#!/usr/bin/env python3
"""Fast, read-only safety checks for an isolated mapping session."""

import os
from pathlib import Path
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


STOPPED_SERVICES = (
    'low-battery-auto-recharge.service',
    'wheeltec-recharge-manager.service',
    'bodyfollow-nav2.service',
    'device-reporter.service',
)

CONFLICT_EXECUTABLES = {
    'slam_toolbox_async',
    'cartographer_node',
    'controller_server',
    'planner_server',
    'bt_navigator',
    'waypoint_follower',
    'local_rrt',
    'global_rrt',
    'filter',
    'assigner',
    'stemm_nav2_manager',
    'recharge_manager',
    'recharge_manage',
    'low_battery_auto_recharge',
    'low_battery_aut',
    'body_main',
    'torso_trt_demo',
    'body_nav2_follower',
    'body_nav2_profile_guard',
    'body_identity_bridge',
    'lasertracker',
}


def _service_is_active(unit: str) -> bool:
    result = subprocess.run(
        ['systemctl', 'is-active', '--quiet', unit],
        check=False,
        timeout=2.0,
    )
    return result.returncode == 0


def _conflicting_processes() -> list[str]:
    conflicts = []
    own_pid = os.getpid()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            comm = (entry / 'comm').read_text(encoding='utf-8').strip()
            argv = [
                item.decode('utf-8', errors='replace')
                for item in (entry / 'cmdline').read_bytes().split(b'\0')
                if item
            ]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        command_names = {comm}
        command_names.update(Path(item).name for item in argv)
        matched = sorted(command_names.intersection(CONFLICT_EXECUTABLES))
        if matched:
            conflicts.append(f'{entry.name}:{matched[0]}')
    return conflicts


class PreflightGuard(Node):
    def __init__(self) -> None:
        super().__init__('stemm_cartographer_preflight')
        self.declare_parameter('phase', 'system')
        self.declare_parameter('timeout_sec', 5.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('expected_imu_frame', 'gyro_link')

        self.phase = self.get_parameter('phase').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.expected_imu_frame = str(
            self.get_parameter('expected_imu_frame').value)

        self.first_stamps = {}
        self.last_stamps = {}
        self.message_counts = {'scan': 0, 'odom': 0, 'imu': 0}
        self.scan_frame = ''
        self.imu_frame = ''
        self.tf_buffer = None
        self.tf_listener = None

        if self.phase == 'sensors':
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                LaserScan, '/scan',
                lambda msg: self._record('scan', msg), qos)
            self.create_subscription(
                Odometry, '/odom',
                lambda msg: self._record('odom', msg), qos)
            self.create_subscription(
                Imu, '/imu/data_bias_compensated',
                lambda msg: self._record('imu', msg), qos)
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)

    def _record(self, key, msg) -> None:
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.message_counts[key] += 1
        self.first_stamps.setdefault(key, stamp)
        self.last_stamps[key] = stamp
        if key == 'scan':
            self.scan_frame = msg.header.frame_id
        elif key == 'imu':
            self.imu_frame = msg.header.frame_id

    def check_system(self) -> tuple[bool, str]:
        if not _service_is_active('wheeltec-robot.service'):
            return False, 'wheeltec-robot.service is not active'

        active = [unit for unit in STOPPED_SERVICES if _service_is_active(unit)]
        if active:
            return False, f'functional services still active: {active}'

        conflicts = _conflicting_processes()
        if conflicts:
            return False, f'conflicting mapping/navigation processes: {conflicts}'
        return True, 'system service and process isolation checks passed'

    def sensor_data_ready(self) -> tuple[bool, str]:
        if any(self.message_counts[key] < 2 for key in self.message_counts):
            return False, f'waiting for advancing sensor messages: {self.message_counts}'
        if any(self.first_stamps[key] == self.last_stamps[key]
               for key in self.message_counts):
            return False, 'one or more sensor timestamps are not advancing'
        if not self.scan_frame:
            return False, '/scan frame_id is empty'
        if self.imu_frame != self.expected_imu_frame:
            return False, (
                f'IMU frame is {self.imu_frame!r}, expected '
                f'{self.expected_imu_frame!r}')

        publishers = self.get_publishers_info_by_topic('/scan')
        if len(publishers) != 1:
            return False, f'/scan must have exactly one publisher, found {len(publishers)}'

        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.scan_frame, Time(),
                timeout=Duration(seconds=0.1))
            self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            return False, f'waiting for required sensor TF: {exc}'
        return True, (
            f'sensor checks passed: scan={self.scan_frame}, '
            f'imu={self.imu_frame}')

    def run(self) -> int:
        if self.phase == 'system':
            try:
                ok, message = self.check_system()
            except (OSError, subprocess.SubprocessError) as exc:
                ok, message = False, f'system preflight failed: {exc}'
            (self.get_logger().info if ok else self.get_logger().error)(message)
            return 0 if ok else 1

        if self.phase != 'sensors':
            self.get_logger().error(f'unsupported preflight phase: {self.phase}')
            return 2

        deadline = time.monotonic() + self.timeout_sec
        last_message = 'waiting for sensor data'
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ok, last_message = self.sensor_data_ready()
            if ok:
                self.get_logger().info(last_message)
                return 0
        self.get_logger().error(
            f'sensor preflight timed out after {self.timeout_sec:.1f}s: '
            f'{last_message}')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreflightGuard()
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()
