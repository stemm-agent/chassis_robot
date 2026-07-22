#!/usr/bin/env python3
# coding=utf-8

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger


class LowBatteryAutoRecharge(Node):
    '''Monitor battery voltage and request the persistent recharge manager.'''

    def __init__(self):
        super().__init__('low_battery_auto_recharge')

        self.declare_parameter('voltage_topic', 'PowerVoltage')
        self.declare_parameter('charging_topic', 'robot_charging_flag')
        self.declare_parameter('low_voltage_threshold', 21.0)
        self.declare_parameter('low_voltage_confirm_count', 6)
        self.declare_parameter('trigger_cooldown_sec', 300.0)
        self.declare_parameter('manager_start_service', '/auto_recharge/start')
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
        self.manager_start_service = str(
            self.get_parameter('manager_start_service').value
        )
        self.trigger_command = str(self.get_parameter('trigger_command').value)
        self.fallback_to_trigger_command = bool(
            self.get_parameter('fallback_to_trigger_command').value
        )

        self.current_voltage = None
        self.is_charging = False
        self.low_voltage_count = 0
        self.last_trigger_time = 0.0
        self.recharge_process = None
        self.recharge_request_future = None
        self.manager_start_client = self.create_client(
            Trigger, self.manager_start_service
        )

        self.create_subscription(
            Float32, self.voltage_topic, self._voltage_callback, 10
        )
        self.create_subscription(
            Bool, self.charging_topic, self._charging_callback, 10
        )
        self.create_timer(1.0, self._timer_callback)

        self.get_logger().info(
            'Low-battery monitor started: voltage_topic=%s, charging_topic=%s, '
            'threshold=%.2fV, manager_service=%s, command_fallback=%s'
            % (
                self.voltage_topic,
                self.charging_topic,
                self.low_voltage_threshold,
                self.manager_start_service,
                self.fallback_to_trigger_command,
            )
        )

    def _voltage_callback(self, msg):
        self.current_voltage = float(msg.data)
        self._evaluate_battery_state()

    def _charging_callback(self, msg):
        self.is_charging = bool(msg.data)
        if self.is_charging:
            self.low_voltage_count = 0

    def _timer_callback(self):
        self._reap_recharge_process()
        self._evaluate_battery_state()

    def _evaluate_battery_state(self):
        if self.current_voltage is None or self.is_charging:
            return
        if self._is_recharge_request_pending() or self._is_recharge_process_running():
            return
        if self.current_voltage > self.low_voltage_threshold:
            self.low_voltage_count = 0
            return

        self.low_voltage_count += 1
        if self.low_voltage_count < self.low_voltage_confirm_count:
            self.get_logger().warn(
                'Low voltage %.2fV detected (%d/%d).'
                % (
                    self.current_voltage,
                    self.low_voltage_count,
                    self.low_voltage_confirm_count,
                )
            )
            return

        now = time.monotonic()
        if now - self.last_trigger_time < self.trigger_cooldown_sec:
            return

        self.low_voltage_count = 0
        if self._start_recharge_task():
            self.last_trigger_time = now

    def _start_recharge_task(self):
        self.get_logger().warn(
            'Battery voltage %.2fV is below %.2fV, requesting recharge manager.'
            % (self.current_voltage, self.low_voltage_threshold)
        )
        if self.manager_start_client.service_is_ready() or self.manager_start_client.wait_for_service(
            timeout_sec=0.5
        ):
            self.recharge_request_future = self.manager_start_client.call_async(
                Trigger.Request()
            )
            self.recharge_request_future.add_done_callback(
                self._manager_response_callback
            )
            return True

        self.get_logger().error(
            'Recharge manager service is unavailable: %s'
            % self.manager_start_service
        )
        if not self.fallback_to_trigger_command:
            return False
        return self._start_legacy_trigger_command()

    def _manager_response_callback(self, future):
        self.recharge_request_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error('Recharge manager request failed: %s' % exc)
            return
        if response is not None and response.success:
            self.get_logger().warn('Recharge manager accepted request: %s' % response.message)
        else:
            self.get_logger().warn(
                'Recharge manager rejected request: %s'
                % (response.message if response is not None else 'empty response')
            )

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
