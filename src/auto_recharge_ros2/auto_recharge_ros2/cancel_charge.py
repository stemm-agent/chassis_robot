#!/usr/bin/env python3
# coding=utf-8

import sys
import time
import math

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, Float32, Int8, String, UInt8
from std_srvs.srv import Trigger

from auto_recharge_ros2.auto_recharger import print_and_fixRetract, settings


AUTO_RECHARGER_NODE_NAME = 'auto_recharger'
MANAGER_STOP_SERVICE = '/auto_recharge/stop'
MANAGER_STATUS_TOPIC = '/auto_recharge/status'


class CancelChargeNode(Node):
    def __init__(self):
        super().__init__('cancel_charge')
        self.declare_parameter('dock_hint', False)
        self.dock_hint = bool(self.get_parameter('dock_hint').value)
        self.charging = False
        self.was_charging = False
        self.charge_state_received = False
        self.charging_current = 0.0
        self.charging_current_received = False
        self.red_detected = False
        self.red_state_received = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_received = False
        self.manager_status = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.recharge_flag_pub = self.create_publisher(Int8, 'robot_recharge_flag', 5)
        self.security_pub = self.create_publisher(Int8, '/chassis_security', 5)
        self.create_subscription(Bool, 'robot_charging_flag', self._charging_callback, 10)
        self.create_subscription(
            Float32, 'robot_charging_current', self._charging_current_callback, 10
        )
        self.create_subscription(UInt8, 'robot_red_flag', self._red_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_cancel_client = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.manager_stop_client = self.create_client(Trigger, MANAGER_STOP_SERVICE)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, MANAGER_STATUS_TOPIC, self._manager_status_callback, status_qos
        )

    def _charging_callback(self, msg):
        self.charging = bool(msg.data)
        self.charge_state_received = True

    def _charging_current_callback(self, msg):
        self.charging_current = float(msg.data)
        self.charging_current_received = True

    def _red_callback(self, msg):
        self.red_detected = int(msg.data) > 0
        self.red_state_received = True

    def _odom_callback(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_received = True

    def _manager_status_callback(self, msg):
        self.manager_status = str(msg.data)

    def cancel_charge(self):
        self._wait_for_dock_state()
        manager_was_running = self._is_auto_recharger_running()
        task_was_active = bool(
            self.manager_status and not self.manager_status.startswith('IDLE')
        )
        self.was_charging = self.charging
        dock_evidence = (
            self.dock_hint
            or self.was_charging
            or self.red_detected
            or (self.charging_current_received and self.charging_current > 0.05)
        )

        print_and_fixRetract(
            '取消回充状态快照: task=%s dock_hint=%s charging=%s current=%.3fA red=%s odom=%s.'
            % (
                task_was_active,
                self.dock_hint,
                self.was_charging,
                self.charging_current,
                self.red_detected,
                self.odom_received,
            )
        )

        manager_stop_confirmed = self._request_manager_stop()
        # The manager now arms exact cancellation even while its goal response
        # is pending.  Broad cancellation is reserved for an active recharge
        # task whose manager stop request could not be confirmed, so an idle
        # "cancel recharge" command cannot cancel unrelated navigation.
        if task_was_active and not manager_stop_confirmed:
            self._cancel_navigation_goal()
        self._publish_chassis_security(True)
        self._stop_robot()
        # Exit the MCU's automatic-recharge mode before sending a normal
        # velocity command.  Publishing this at the end made detach commands
        # race with the docking controller and with navigate_to_charger exit.
        self._publish_stop_flag()
        self._wait_for_manager_idle(
            timeout_sec=2.0,
            manager_expected=manager_was_running and not manager_stop_confirmed,
        )
        self._settle_after_cancel()

        if dock_evidence:
            moved = self._drive_forward_for_exit()
            self._wait_for_disconnect()
            if not moved:
                print_and_fixRetract(
                    '脱桩速度已发送，但里程计未确认有效移动，请检查 /cmd_vel 竞争或底盘模式。'
                )
        else:
            print_and_fixRetract(
                '未检测到充电、充电电流或红外桩信号；已停止回充任务，'
                '为避免导航途中取消时突然前冲，不执行脱桩动作。'
            )

    def _wait_for_dock_state(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.charge_state_received
                and self.charging_current_received
                and self.red_state_received
            ):
                return

        missing = []
        if not self.charge_state_received:
            missing.append('robot_charging_flag')
        if not self.charging_current_received:
            missing.append('robot_charging_current')
        if not self.red_state_received:
            missing.append('robot_red_flag')
        print_and_fixRetract(
            '未在超时时间内收齐贴桩状态: %s；将使用已收到的传感器证据。'
            % ', '.join(missing)
        )

    def _spin_until_future_done(self, future: Future, timeout_sec):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def _request_manager_stop(self):
        if not self.manager_stop_client.wait_for_service(timeout_sec=3.0):
            print_and_fixRetract(
                'Persistent recharge manager stop service is unavailable; '
                'using navigation-cancel and recharge-flag fallbacks.'
            )
            return False

        future = self.manager_stop_client.call_async(Trigger.Request())
        if not self._spin_until_future_done(future, timeout_sec=3.0):
            print_and_fixRetract(
                'Persistent recharge manager stop service timed out; using fallbacks.'
            )
            return False
        try:
            response = future.result()
        except Exception as exc:
            print_and_fixRetract('Recharge manager stop service failed: %s' % exc)
            return False
        if response is None or not response.success:
            print_and_fixRetract(
                'Recharge manager did not confirm stop: %s'
                % (response.message if response is not None else 'empty response')
            )
            return False
        print_and_fixRetract('Recharge manager stop confirmed: %s' % response.message)
        return True

    def _cancel_navigation_goal(self):
        if not self.nav_cancel_client.wait_for_service(timeout_sec=3.0):
            print_and_fixRetract('未检测到导航动作服务，跳过导航取消.(Navigate action server not available, skip cancel.)')
            return

        request = CancelGoal.Request()
        request.goal_info = GoalInfo()

        cancel_future = self.nav_cancel_client.call_async(request)
        self._spin_until_future_done(cancel_future, timeout_sec=2.0)

        if cancel_future.done():
            response = cancel_future.result()
            if response is not None and response.return_code in (
                CancelGoal.Response.ERROR_NONE,
                CancelGoal.Response.ERROR_GOAL_TERMINATED,
            ):
                print_and_fixRetract('已发送导航取消请求.(Navigation cancel request sent.)')
            else:
                print_and_fixRetract('导航取消请求已发送，但返回状态异常.(Navigation cancel request sent but returned an unexpected status.)')
        else:
            print_and_fixRetract('导航取消请求超时.(Navigation cancel request timed out.)')

    def _publish_stop_flag(self):
        topic = Int8()
        topic.data = 0
        for _ in range(20):
            self.recharge_flag_pub.publish(topic)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _settle_after_cancel(self, timeout_sec=0.8):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_chassis_security(self, enabled):
        topic = Int8()
        topic.data = 1 if enabled else 0
        for _ in range(5):
            self.security_pub.publish(topic)
            rclpy.spin_once(self, timeout_sec=0.02)

    def _stop_robot(self):
        topic = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(topic)
            time.sleep(0.1)

    def _drive_forward_for_exit(self, speed=0.2, max_duration_sec=4.0, target_distance=0.45):
        publisher_count = self.count_publishers('/cmd_vel')
        print_and_fixRetract(
            '开始脱桩: %.2fm/s，最长%.1fs，目标位移%.2fm；/cmd_vel 发布者数=%d.'
            % (speed, max_duration_sec, target_distance, publisher_count)
        )
        topic = Twist()
        topic.linear.x = speed
        start_x = self.odom_x
        start_y = self.odom_y
        had_start_odom = self.odom_received
        end_time = time.time() + max_duration_sec
        distance = 0.0
        while rclpy.ok() and time.time() < end_time:
            self.cmd_vel_pub.publish(topic)
            rclpy.spin_once(self, timeout_sec=0.0)
            if had_start_odom and self.odom_received:
                distance = math.hypot(self.odom_x - start_x, self.odom_y - start_y)
                if distance >= target_distance:
                    break
            time.sleep(0.05)

        topic.linear.x = 0.0
        for _ in range(10):
            self.cmd_vel_pub.publish(topic)
            rclpy.spin_once(self, timeout_sec=0.02)

        if had_start_odom:
            print_and_fixRetract(
                '脱桩运动完成: odom_distance=%.3fm charging=%s red=%s.'
                % (distance, self.charging, self.red_detected)
            )
            return distance >= 0.10

        print_and_fixRetract(
            '脱桩运动完成，但未收到 /odom，无法核验实际位移。'
        )
        return False

    def _wait_for_disconnect(self, timeout_sec=3.0):
        if not self.was_charging:
            return
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.charging:
                print_and_fixRetract('检测到 charging disconnected，准备退出节点.(Charging disconnected detected, exiting node.)')
                return

        print_and_fixRetract('等待 charging disconnected 超时，按当前状态退出节点.(Timed out waiting for charging disconnected, exiting anyway.)')

    def _wait_for_manager_idle(self, timeout_sec=2.0, manager_expected=True):
        if not manager_expected:
            return
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if self.manager_status is not None and self.manager_status.startswith('IDLE'):
                print_and_fixRetract(
                    'Persistent recharge manager reached IDLE; continuing detach flow.'
                )
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        print_and_fixRetract(
            'Timed out waiting for recharge manager IDLE; '
            'continuing detach flow with fallbacks.'
        )

    def _wait_for_auto_recharger_exit(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if not self._is_auto_recharger_running():
                print_and_fixRetract('检测到 auto_recharger 节点已退出，取消回充节点准备退出.(auto_recharger node exited; cancel-charge node will exit now.)')
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        print_and_fixRetract('等待 auto_recharger 节点退出超时，按当前状态退出节点.(Timed out waiting for auto_recharger to exit, exiting anyway.)')

    def _is_auto_recharger_running(self):
        try:
            node_names = self.get_node_names()
        except Exception:
            return True

        active_names = [name for name in node_names if name != self.get_name()]
        return AUTO_RECHARGER_NODE_NAME in active_names


def main():
    node = None
    rclpy.init()

    try:
        node = CancelChargeNode()
        node.cancel_charge()
        print_and_fixRetract('已触发取消充电流程.(Cancel-charge flow triggered.)')
    except KeyboardInterrupt:
        print_and_fixRetract('取消充电节点已退出.(Cancel-charge node exited.)')
    except Exception as exc:
        print_and_fixRetract(str(exc))
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
        if settings is not None and sys.stdin.isatty():
            try:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            except Exception:
                pass


if __name__ == '__main__':
    main()
