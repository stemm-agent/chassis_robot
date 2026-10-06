#!/usr/bin/env python3
# coding=utf-8

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from auto_recharge_ros2.low_battery_policy import (
    LowBatteryEvidence,
    RechargeEpisodeLatch,
)


STARTED_MANAGER_STATES = (
    'STARTING',
    'GOAL_PENDING',
    'NAVIGATING',
    'IR_SEARCH',
    'HANDOFF_',
    'REFINING_',
    'DOCKING',
    'CHARGING',
)


class LowBatteryAutoRecharge(Node):
    '''Monitor battery voltage and request the persistent recharge manager.'''

    def __init__(self):
        super().__init__('low_battery_auto_recharge')

        self.declare_parameter('voltage_topic', 'PowerVoltage')
        self.declare_parameter('charging_topic', 'robot_charging_flag')
        self.declare_parameter('low_voltage_threshold', 22.2)
        self.declare_parameter('low_voltage_confirm_count', 6)
        self.declare_parameter('trigger_cooldown_sec', 300.0)
        self.declare_parameter('minimum_valid_voltage', 5.0)
        self.declare_parameter('maximum_valid_voltage', 35.0)
        self.declare_parameter('voltage_stale_timeout_sec', 5.0)
        self.declare_parameter('charging_state_stale_timeout_sec', 5.0)
        self.declare_parameter('manager_request_timeout_sec', 12.0)
        self.declare_parameter('manager_start_confirmation_timeout_sec', 20.0)
        self.declare_parameter('failure_retry_sec', 15.0)
        self.declare_parameter('manager_service_discovery_wait_sec', 2.0)
        self.declare_parameter('manager_start_service', '/auto_recharge/start')
        self.declare_parameter('manager_status_topic', '/auto_recharge/status')
        self.declare_parameter('manager_result_topic', '/auto_recharge/result')
        self.declare_parameter('feedback_topic', 'feedback_audio')
        self.declare_parameter(
            'low_battery_audio_file', '/low_battery_recharge.wav'
        )
        # Retain the original configurable command as an opt-in compatibility
        # fallback.  The default path does not spawn a recharge worker.
        self.declare_parameter(
            'trigger_command', 'ros2 run auto_recharge_ros2 navigate_to_charger'
        )
        self.declare_parameter('fallback_to_trigger_command', False)

        self.voltage_topic = self.get_parameter('voltage_topic').value
        self.charging_topic = self.get_parameter('charging_topic').value
        self.low_voltage_threshold = float(
            self.get_parameter('low_voltage_threshold').value
        )
        self.low_voltage_confirm_count = int(
            self.get_parameter('low_voltage_confirm_count').value
        )
        self.trigger_cooldown_sec = float(
            self.get_parameter('trigger_cooldown_sec').value
        )
        self.manager_request_timeout_sec = float(
            self.get_parameter('manager_request_timeout_sec').value
        )
        self.manager_start_confirmation_timeout_sec = float(
            self.get_parameter('manager_start_confirmation_timeout_sec').value
        )
        self.failure_retry_sec = float(
            self.get_parameter('failure_retry_sec').value
        )
        self.manager_service_discovery_wait_sec = float(
            self.get_parameter('manager_service_discovery_wait_sec').value
        )
        self.manager_start_service = str(
            self.get_parameter('manager_start_service').value
        )
        self.manager_status_topic = str(
            self.get_parameter('manager_status_topic').value
        )
        self.manager_result_topic = str(
            self.get_parameter('manager_result_topic').value
        )
        self.feedback_topic = str(
            self.get_parameter('feedback_topic').value
        )
        self.low_battery_audio_file = str(
            self.get_parameter('low_battery_audio_file').value
        )
        self.trigger_command = str(self.get_parameter('trigger_command').value)
        self.fallback_to_trigger_command = bool(
            self.get_parameter('fallback_to_trigger_command').value
        )

        self.evidence = LowBatteryEvidence(
            low_voltage_threshold=self.low_voltage_threshold,
            confirm_count=self.low_voltage_confirm_count,
            minimum_valid_voltage=float(
                self.get_parameter('minimum_valid_voltage').value
            ),
            maximum_valid_voltage=float(
                self.get_parameter('maximum_valid_voltage').value
            ),
            voltage_stale_timeout_sec=float(
                self.get_parameter('voltage_stale_timeout_sec').value
            ),
            charging_state_stale_timeout_sec=float(
                self.get_parameter('charging_state_stale_timeout_sec').value
            ),
        )
        self.last_trigger_time = 0.0
        self.retry_not_before = 0.0
        self.recharge_process = None
        self.recharge_request_future = None
        self.recharge_request_deadline = None
        self.awaiting_start_confirmation = False
        self.start_confirmation_deadline = None
        self.manager_status = None
        self.recharge_active_seen = False
        self.episode = RechargeEpisodeLatch()
        self.manager_start_client = self.create_client(
            Trigger, self.manager_start_service
        )
        self.feedback_pub = self.create_publisher(
            String, self.feedback_topic, 10
        )

        self.create_subscription(
            Float32, self.voltage_topic, self._voltage_callback, 10
        )
        self.create_subscription(
            Bool, self.charging_topic, self._charging_callback, 10
        )
        manager_qos = QoSProfile(depth=10)
        manager_qos.reliability = ReliabilityPolicy.RELIABLE
        manager_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            self.manager_status_topic,
            self._manager_status_callback,
            manager_qos,
        )
        result_qos = QoSProfile(depth=10)
        result_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            String,
            self.manager_result_topic,
            self._manager_result_callback,
            result_qos,
        )
        self.create_timer(1.0, self._timer_callback)

        self.get_logger().info(
            'Low-battery monitor started: voltage_topic=%s, charging_topic=%s, '
            'threshold=%.2fV, confirm_samples=%d, manager_service=%s, '
            'request_timeout=%.1fs, command_fallback=%s'
            % (
                self.voltage_topic,
                self.charging_topic,
                self.low_voltage_threshold,
                self.low_voltage_confirm_count,
                self.manager_start_service,
                self.manager_request_timeout_sec,
                self.fallback_to_trigger_command,
            )
        )

    def _voltage_callback(self, msg):
        now = time.monotonic()
        if self.episode.voltage_frozen:
            return
        if not self.evidence.record_voltage(msg.data, now):
            self.get_logger().error(
                'Ignore invalid battery voltage sample: %r' % msg.data
            )
            return
        if self.evidence.current_voltage > self.low_voltage_threshold:
            self.episode.reset_audio()
        if (
            self.evidence.current_voltage <= self.low_voltage_threshold
            and self.evidence.charging_state_is_fresh(now)
            and not self.evidence.is_charging
            and self.evidence.low_voltage_count < self.low_voltage_confirm_count
        ):
            self.get_logger().warn(
                'Low voltage %.2fV detected from fresh sample (%d/%d).'
                % (
                    self.evidence.current_voltage,
                    self.evidence.low_voltage_count,
                    self.low_voltage_confirm_count,
                )
            )
        self._maybe_start_recharge(now)

    def _charging_callback(self, msg):
        self.evidence.record_charging_state(msg.data, time.monotonic())
        if self.evidence.is_charging:
            self.episode.reset_audio()

    def _timer_callback(self):
        self._reap_recharge_process()
        now = time.monotonic()
        self.evidence.discard_stale_evidence(now)
        self._check_request_timeout(now)
        self._check_start_confirmation_timeout(now)
        self._maybe_start_recharge(now)

    def _maybe_start_recharge(self, now):
        if self.episode.voltage_frozen:
            return
        if not self.evidence.ready_to_trigger(now):
            return
        if (
            self._is_recharge_request_pending()
            or self.awaiting_start_confirmation
            or self.recharge_active_seen
            or self._is_recharge_process_running()
        ):
            return
        if self._manager_state_is_active(self.manager_status):
            return
        if now < self.retry_not_before:
            return
        if now - self.last_trigger_time < self.trigger_cooldown_sec:
            return

        if self._start_recharge_task():
            self.evidence.reset_confirmation()

    def _start_recharge_task(self):
        self.get_logger().warn(
            'Battery voltage %.2fV is below %.2fV, requesting recharge manager.'
            % (self.evidence.current_voltage, self.low_voltage_threshold)
        )
        if (
            self.manager_start_client.service_is_ready()
            or self.manager_start_client.wait_for_service(
                timeout_sec=self.manager_service_discovery_wait_sec
            )
        ):
            self.recharge_request_future = self.manager_start_client.call_async(
                Trigger.Request()
            )
            self.recharge_request_deadline = (
                time.monotonic() + self.manager_request_timeout_sec
            )
            self.recharge_request_future.add_done_callback(
                self._manager_response_callback
            )
            return True

        self.get_logger().error(
            'Recharge manager service is unavailable: %s'
            % self.manager_start_service
        )
        self.retry_not_before = time.monotonic() + self.failure_retry_sec
        if not self.fallback_to_trigger_command:
            return False
        return self._start_legacy_trigger_command()

    def _publish_low_battery_audio_once(self):
        if not self.episode.mark_audio_announced():
            return
        msg = String()
        msg.data = self.low_battery_audio_file
        self.feedback_pub.publish(msg)
        self.get_logger().info(
            'Published low-battery audio prompt: %s'
            % self.low_battery_audio_file
        )

    def _manager_response_callback(self, future):
        if future is not self.recharge_request_future:
            return
        self.recharge_request_future = None
        self.recharge_request_deadline = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error('Recharge manager request failed: %s' % exc)
            self._schedule_failure_retry('manager request exception')
            return
        if response is not None and response.success:
            self.get_logger().warn('Recharge manager accepted request: %s' % response.message)
            frozen_voltage = self.evidence.current_voltage
            if frozen_voltage is None:
                self.get_logger().error(
                    'Recharge request was accepted without a valid trigger voltage.'
                )
                self._schedule_failure_retry('trigger voltage missing')
                return
            if self.episode.freeze(frozen_voltage):
                self.get_logger().warn(
                    'Low-battery episode frozen at %.2fV until recharge '
                    'reaches a terminal result.' % frozen_voltage
                )
            self.evidence.reset_confirmation()
            self._publish_low_battery_audio_once()
            self.awaiting_start_confirmation = True
            self.start_confirmation_deadline = (
                time.monotonic()
                + self.manager_start_confirmation_timeout_sec
            )
            if self._manager_state_is_started(self.manager_status):
                self._confirm_recharge_started(self.manager_status)
        else:
            self.get_logger().warn(
                'Recharge manager rejected request: %s'
                % (response.message if response is not None else 'empty response')
            )
            self._schedule_failure_retry('manager rejected request')

    def _manager_status_callback(self, msg):
        previous_status = self.manager_status
        self.manager_status = str(msg.data)
        if self._manager_state_is_started(self.manager_status):
            self._confirm_recharge_started(self.manager_status)
            return
        if (
            self.manager_status.startswith('IDLE')
            and self.recharge_active_seen
            and previous_status is not None
            and not previous_status.startswith('IDLE')
            and not self.evidence.is_charging
        ):
            self._schedule_failure_retry(
                'manager returned to IDLE before charging'
            )

    def _manager_result_callback(self, msg):
        result = str(msg.data)
        if result.startswith('FAILED:'):
            self._schedule_failure_retry(result)
        elif result.startswith('CANCELED:MAPPING'):
            self._schedule_failure_retry(result)
        elif result.startswith('CANCELED:'):
            self.awaiting_start_confirmation = False
            self.start_confirmation_deadline = None
            self.recharge_active_seen = False
            self.last_trigger_time = time.monotonic()
            self._release_voltage_freeze(
                'recharge canceled', reset_audio=False
            )
        elif result.startswith('SUCCEEDED:'):
            self.awaiting_start_confirmation = False
            self.start_confirmation_deadline = None
            self.recharge_active_seen = False
            self.last_trigger_time = time.monotonic()
            self._release_voltage_freeze(
                'recharge succeeded', reset_audio=True
            )

    def _confirm_recharge_started(self, status):
        if not self.awaiting_start_confirmation and self.recharge_active_seen:
            return
        self.awaiting_start_confirmation = False
        self.start_confirmation_deadline = None
        self.recharge_active_seen = True
        self.retry_not_before = 0.0
        self.last_trigger_time = time.monotonic()
        self.evidence.reset_confirmation()
        self.get_logger().warn(
            'Recharge start confirmed by manager state: %s' % status
        )

    def _schedule_failure_retry(self, reason):
        now = time.monotonic()
        self.awaiting_start_confirmation = False
        self.start_confirmation_deadline = None
        self.recharge_active_seen = False
        self.last_trigger_time = 0.0
        self.retry_not_before = now + self.failure_retry_sec
        self._release_voltage_freeze(reason, reset_audio=False)
        self.get_logger().error(
            'Recharge did not start or failed (%s); retry after %.1fs.'
            % (reason, self.failure_retry_sec)
        )

    def _release_voltage_freeze(self, reason, reset_audio):
        frozen_voltage = self.episode.release(reset_audio=reset_audio)
        self.evidence.clear_voltage_evidence()
        if frozen_voltage is not None:
            self.get_logger().warn(
                'Released low-battery voltage freeze %.2fV after %s; '
                'fresh voltage samples are required before another trigger.'
                % (frozen_voltage, reason)
            )

    def _check_request_timeout(self, now):
        if (
            not self._is_recharge_request_pending()
            or self.recharge_request_deadline is None
            or now < self.recharge_request_deadline
        ):
            return
        future = self.recharge_request_future
        self.recharge_request_future = None
        self.recharge_request_deadline = None
        try:
            future.cancel()
        except Exception:
            pass
        self._schedule_failure_retry(
            'manager service response timed out after %.1fs'
            % self.manager_request_timeout_sec
        )

    def _check_start_confirmation_timeout(self, now):
        if (
            not self.awaiting_start_confirmation
            or self.start_confirmation_deadline is None
            or now < self.start_confirmation_deadline
        ):
            return
        self._schedule_failure_retry(
            'manager did not enter an active state within %.1fs'
            % self.manager_start_confirmation_timeout_sec
        )

    @staticmethod
    def _manager_state_is_started(status):
        return bool(
            status
            and any(status.startswith(prefix) for prefix in STARTED_MANAGER_STATES)
        )

    @staticmethod
    def _manager_state_is_active(status):
        return bool(status and not status.startswith('IDLE'))

    def _is_recharge_request_pending(self):
        return (
            self.recharge_request_future is not None
            and not self.recharge_request_future.done()
        )

    def _start_legacy_trigger_command(self):
        self.get_logger().warn(
            'Using legacy recharge trigger command: %s' % self.trigger_command
        )
        try:
            self.recharge_process = subprocess.Popen(
                self.trigger_command,
                shell=True,
                preexec_fn=os.setsid,
            )
            return True
        except Exception as exc:
            self.recharge_process = None
            self.get_logger().error('Failed to start recharge command: %s' % exc)
            return False

    def _is_recharge_process_running(self):
        return self.recharge_process is not None and self.recharge_process.poll() is None

    def _reap_recharge_process(self):
        if self.recharge_process is None:
            return
        return_code = self.recharge_process.poll()
        if return_code is None:
            return
        self.get_logger().info('Recharge command exited with code %s.' % return_code)
        self.recharge_process = None

    def stop_recharge_process(self):
        if self._is_recharge_request_pending():
            try:
                self.recharge_request_future.cancel()
            except Exception:
                pass
            self.recharge_request_future = None
            self.recharge_request_deadline = None
        if not self._is_recharge_process_running():
            return
        self.get_logger().info('Stopping active legacy recharge command.')
        try:
            os.killpg(os.getpgid(self.recharge_process.pid), signal.SIGTERM)
            self.recharge_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.recharge_process.pid), signal.SIGKILL)
        except Exception as exc:
            self.get_logger().warn(
                'Failed to stop recharge command cleanly: %s' % exc
            )
        finally:
            self.recharge_process = None


def main(args=None):
    rclpy.init(args=args)
    node = LowBatteryAutoRecharge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recharge_process()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
