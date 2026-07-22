#!/usr/bin/env python3
# coding=utf-8

import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from auto_recharge_ros2.auto_recharger import (
    AutoRecharger,
    print_and_fixRetract,
    yaml_file,
)


START_SERVICE = '/auto_recharge/start'
STOP_SERVICE = '/auto_recharge/stop'
STATUS_TOPIC = '/auto_recharge/status'


def _load_robot_config(autorecharger):
    autorecharger.declare_parameter('robot_BatteryCapacity', 5000)
    autorecharger.declare_parameter('car_mode', 'mini_mec')
    autorecharger.declare_parameter('diff_point', 1.2)
    autorecharger.declare_parameter('diff_angle', -15)
    autorecharger.declare_parameter('nav2_wait_timeout_sec', 12.0)

    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)

    robot_info = params['robot_info']
    autorecharger.robot['BatteryCapacity'] = robot_info['BatteryCapacity']
    autorecharger.robot['car_mode'] = robot_info['car_mode']
    autorecharger.diff_point = robot_info['diff_point']
    autorecharger.diff_angle = robot_info['diff_angle']
    if autorecharger.robot['car_mode'][0:4] != 'mini':
        autorecharger.robot['Type'] = 'Plus'
    else:
        autorecharger.robot['Type'] = 'Mini'


class RechargeManager(AutoRecharger):
    '''Long-lived recharge state machine controlled through ROS services.'''

    def __init__(self):
        super().__init__()
        _load_robot_config(self)

        self.nav2_wait_timeout_sec = float(
            self.get_parameter('nav2_wait_timeout_sec').value
        )
        self.manager_active = False
        self.waiting_for_nav2 = False
        self.nav2_wait_deadline = None
        self.last_terminal_state = 'startup'
        self.task_finished = True

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, status_qos)
        self.start_service = self.create_service(
            Trigger, START_SERVICE, self._start_callback
        )
        self.stop_service = self.create_service(
            Trigger, STOP_SERVICE, self._stop_callback
        )
        self.manager_timer = self.create_timer(0.1, self._manager_tick)
        self._status = None
        self._publish_status('IDLE', force=True)
        self.get_logger().info(
            'Persistent recharge manager ready: start=%s stop=%s status=%s; '
            'Nav2 and localization are not started or reset by this node.'
            % (START_SERVICE, STOP_SERVICE, STATUS_TOPIC)
        )

    def _publish_status(self, status, force=False):
        if not force and status == self._status:
            return
        self._status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info('Recharge manager state: %s' % status)

    def _start_callback(self, _request, response):
        if self.manager_active:
            response.success = False
            response.message = '回充任务已在执行，当前状态: %s' % self._status
            return response

        self.Prepare_New_Task()
        self.manager_active = True
        self.waiting_for_nav2 = True
        self.nav2_wait_deadline = time.monotonic() + self.nav2_wait_timeout_sec
        self._publish_status('WAITING_NAV2')
        response.success = True
        response.message = '回充请求已接收；管理器将异步等待 Nav2 并直接发送目标。'
        return response

    def _stop_callback(self, _request, response):
        was_active = self.manager_active
        exact_cancel_sent = False
        if was_active:
            exact_cancel_sent = self.Cancel_Current_Task()
        else:
            self.chargeflag = 0
            self.Stop_Motion(repeat=2, interval=0.02)
            self.Pub_Recharger_Flag()
            self.task_finished = True

        self.manager_active = False
        self.waiting_for_nav2 = False
        self.nav2_wait_deadline = None
        self.last_terminal_state = 'CANCELED' if was_active else 'already idle'
        self._publish_status('IDLE', force=True)
        response.success = True
        response.message = (
            '回充任务已停止；已发送当前 Nav2 goal 精确取消。'
            if exact_cancel_sent
            else '回充任务已停止；若 goal 尚未返回 handle，取消节点将执行兼容性兜底取消。'
        )
        return response

    def _begin_recharge(self):
        self.waiting_for_nav2 = False
        self.nav2_wait_deadline = None

        security = Int8()
        security.data = 1
        self.robot_security_off_pub.publish(security)
        self.Cmd_vel_pub.publish(Twist())

        self._publish_status('STARTING')
        self.autoRecharger('q')
        if self.task_finished:
            self.last_terminal_state = 'FAILED_TO_START'
            self.manager_active = False
            self._publish_status('IDLE', force=True)
            return
        self._update_active_status()

    def _update_active_status(self):
        if self.robot['Charging'] == 1:
            self._publish_status('CHARGING')
        elif self.start_turn == 1:
            self._publish_status('IR_SEARCH')
        elif self.near_goal_handoff_phase is not None:
            self._publish_status('HANDOFF_' + self.near_goal_handoff_phase.upper())
        elif self.strict_refine_phase is not None:
            self._publish_status('REFINING_' + self.strict_refine_phase.upper())
        elif self.star_getNav_Feedback_Flag == 1:
            if self.nav_goal_handle is None:
                self._publish_status('GOAL_PENDING')
            else:
                self._publish_status('NAVIGATING')
        elif self.chargeflag != 0:
            self._publish_status('DOCKING')
        else:
            self._publish_status('ACTIVE')

    def _manager_tick(self):
        if not self.manager_active:
            return

        if self.waiting_for_nav2:
            # High-strength infrared preserves the original direct-docking path
            # and does not require Nav2 to be online.
            if self.red_count >= 3 or self.Nav_Action_Server_Available():
                self._begin_recharge()
                return
            if (
                self.nav2_wait_deadline is not None
                and time.monotonic() >= self.nav2_wait_deadline
            ):
                print_and_fixRetract(
                    'Nav2 action server 在 %.1fs 内未上线，本次回充请求结束。'
                    % self.nav2_wait_timeout_sec
                )
                self.last_terminal_state = 'NAV2_UNAVAILABLE'
                self.Finish_Task()
                self.manager_active = False
                self.waiting_for_nav2 = False
                self.nav2_wait_deadline = None
                self._publish_status('IDLE', force=True)
            return

        self.autoRecharger('')
        if self.task_finished:
            self.last_terminal_state = self.nav_last_failure_reason or 'FINISHED'
            self.manager_active = False
            self._publish_status('IDLE', force=True)
            return
        self._update_active_status()

    def shutdown_manager(self):
        if self.manager_active:
            self.Cancel_Current_Task()
        try:
            self.nav_controller.destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RechargeManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_manager()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
