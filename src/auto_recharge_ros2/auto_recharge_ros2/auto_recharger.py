#!/usr/bin/env python3 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作
#2:Python.源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

#引用ros库
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin, DriveOnHeading
from rclpy.duration import Duration
from rclpy.time import Time

# 用到的变量定义
from std_msgs.msg import Bool 
from std_msgs.msg import Int8 
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from turtlesim.srv import Spawn

# 用于记录充电桩位置、发布导航点
from geometry_msgs.msg import PoseStamped

# rviz可视化相关
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# cmd_vel话题数据
from geometry_msgs.msg import Twist

# 里程计话题相关
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener

# 获取导航结果
# from move_base_msgs.msg import MoveBaseActionResult # ROS1

# 键盘控制相关
import sys, select, termios, tty

# 延迟相关
import time

# 读写充电桩位置文件
import json
import yaml

import math
import os
import subprocess

#存放充电桩位置的文件位置
json_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json'
yaml_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/robot_info.yaml'
#print_and_fixRetract相关，用于打印带颜色的信息
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

#圆周率
PI=3.1415926535897

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
	
settings = None
if os.name != 'nt' and sys.stdin.isatty():
	settings = list(termios.tcgetattr(sys.stdin))

def get_key(settings):
	if os.name == 'nt':
		return msvcrt.getch().decode('utf-8')
	if sys.stdin.isatty():
		tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	if sys.stdin.isatty():
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def print_and_fixRetract(str):
	global settings
	'''键盘控制会导致回调函数内使用print()出现自动缩进的问题，此函数可以解决该现象'''
	if sys.stdin.isatty():
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	print(str)

class AutoRecharger(Node):
	def __init__(self):
		
        #创建节点
		super().__init__("auto_recharger")

		print_and_fixRetract('Automatic charging node start!')

        #机器人状态变量：类型、电池容量、电池电压、充电状态、充电电流、红外信号状态、记录机器人姿态
		self.robot = {
        'Type':'Plus', 
        'BatteryCapacity':5000, 
        'Voltage':25, 
        'Charging':0, 
        'Charging_current':0, 
        'RED':0, 
        'Rotation_Z':0,
        'car_mode':'mini_mec'
        }

        #用于记录导航结束是的机器人Z轴姿态
		self.nav_end_z=0
		self.start_turn = 0
		self.find_redsignal = 0

		#红外信号的数量
		self.red_count=0
		self.last_red_seen_time = None
		self.red_recent_timeout_sec = 4.0
		#机器人自动回充模式标志位，0：关闭回充，1：导航回充，2：回充装备控制回充
		self.chargeflag=0
		#机器人时间戳记录变量
		self.last_time= self.get_clock().now()
		#机器人红外信号丢失的时间滤波2
		self.lost_red_flag=self.get_clock().now()
		#机器人电量过低(<12.5或者<25)计数
		self.power_lost_count=0
		#机器人低电量检测1次标志位
		self.lost_power_once=1
		#机器人充电完成标志位
		self.charge_complete=0
		#机器人充电完成标志位
		self.last_charge_complete=0
		#最新充电桩位置数据
		self.json_data=0
		#是否监听导航结果标志位
		self.star_getNav_Feedback_Flag=0
		# 语音触发的单次回充流程结束标志，避免失败后进程空转挡住下一次命令
		self.task_finished = False
		# 红外搜索使用 map->base_footprint 的实际 yaw 做左右递增闭环扫描。
		self.turn_start_time = None
		self.turn_timeout_sec = 120.0
		self.search_center_yaw = None
		self.search_target_yaw = None
		self.search_side = 1
		self.search_extent = math.radians(15.0)
		self.search_increment = math.radians(15.0)
		self.search_max_extent = math.radians(90.0)
		self.search_yaw_tolerance = math.radians(3.0)
		self.search_angular_speed = 0.2
		self.nav_goal_sent_time = None
		self.nav_no_feedback_timeout_sec = 45.0
		self.last_nav_distance_remaining = None
		# Recharge-only acceptance; this does not alter the shared Nav2 parameters.
		self.near_goal_dock_distance = 0.10
		self.near_goal_yaw_tolerance = math.radians(5.0)
		self.nav_goal_data = None
		self.last_nav_feedback_time = None
		self.nav_progress_reference_distance = None
		self.nav_progress_reference_time = None
		self.nav_feedback_stall_timeout_sec = 60.0
		self.nav_total_timeout_sec = 150.0
		self.nav_server_missing_since = None
		self.nav_server_missing_timeout_sec = 8.0
		self.nav_progress_epsilon = 0.05

		# 用户标记点和实际导航点直线偏移的距离，单位m
		self.diff_point = 1.2

		# 偏移的角度
		self.diff_angle = -15

		self.nav_controller =  BasicNavigator()
		self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
		self.strict_spin_client = ActionClient(self, Spin, 'spin')
		self.strict_drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
		self.strict_refine_phase = None
		self.strict_refine_goal_future = None
		self.strict_refine_goal_handle = None
		self.strict_refine_result_future = None
		self.strict_refine_deadline = None
		self.strict_refine_reason = ''
		self.strict_refine_max_distance = 0.60
		self.strict_refine_drive_speed = 0.06
		self.strict_refine_turn_threshold = math.radians(2.0)
		self.strict_refine_action_timeout_sec = 18.0
		self.nav_goal_future = None
		self.nav_goal_handle = None
		self.nav_result_future = None
		self.nav_goal_response_deadline = None
		self.nav_goal_response_timeout_sec = 12.0
		self.nav_last_failure_reason = ''
		# Nav2 may stop physically near the charger before its action result is
		# delivered.  A fresh TF that remains inside this radius allows the
		# recharge node to hand off to infrared search without waiting tens of
		# seconds for the final SUCCEEDED result.
		self.near_goal_handoff_distance = 0.22
		# Reuse the recharge-only final-pose tolerance; do not change shared Nav2 params.
		self.near_goal_handoff_yaw_tolerance = self.near_goal_yaw_tolerance
		self.near_goal_handoff_stable_sec = 0.5
		self.near_goal_handoff_tf_max_age_sec = 0.5
		self.near_goal_handoff_cancel_timeout_sec = 3.0
		self.near_goal_handoff_settle_sec = 0.4
		self.near_goal_handoff_stable_since = None
		self.near_goal_handoff_phase = None
		self.near_goal_handoff_cancel_future = None
		self.near_goal_handoff_deadline = None
		self.near_goal_handoff_settle_until = None
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		#读取json文件内保存的充电桩位置信息
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)

		self.robot_security_off_pub = self.create_publisher(Int8,'/chassis_security',   10) 

		#创建充电桩位置标记话题发布者
		self.Charger_marker_pub   = self.create_publisher(MarkerArray,'/goal_marker',   10) 

		#创建自动回充任务是否开启标志位话题发布者
		self.Recharger_Flag_pub = self.create_publisher(Int8,"robot_recharge_flag",  5)

		#速度话题用于不开启导航时，向底盘发送开启自动回充任务命令
		self.Cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",  5)

		#创建机器人电量话题订阅者
		self.Voltage_sub = self.create_subscription(Float32, "PowerVoltage", self.Voltage_callback,10)

		#创建机器人充电状态话题订阅者
		self.Charging_Flag_sub = self.create_subscription(Bool, "robot_charging_flag",self.Charging_Flag_callback,10)

		#创建机器人充电电流话题订阅者
		self.Charging_Current_sub = self.create_subscription(Float32,"robot_charging_current",  self.Charging_Current_callback,10)

		#创建机器人已发现红外信号话题订阅者
		self.RED_Flag_sub = self.create_subscription(UInt8,"robot_red_flag",  self.RED_Flag_callback,10)

		#创建充电桩位置更新话题订阅者
		self.Charger_Position_Update_sub = self.create_subscription( PoseStamped,"/charger_position_update", self.Position_Update_callback,10)

		#创建里程计话题订阅者
		self.Odom_sub = self.create_subscription(Odometry, '/odom', self.Odom_callback,10)

		# 创建服务调用者
		self.set_charge = self.create_client(Spawn,'/set_charge')

		self.server_set_state = None
		self.wait_server_done = None
		#按键控制说明
		self.tips = """
使用下面按键使用自动回充功能.       Press below Key to AutoRecharger.
Q/q:开启自动回充.                   Q/q:Start Navigation to find charger.
E/e:停止自动回充.                   E/e:Stop find charger.
Ctrl+C/c:关闭自动回充功能并退出.    Ctrl+C/c:Quit the program.
可使用话题"charger_position_update"更新充电桩的位置.
		"""
	def wait_server_callback(self,res):
		self.wait_server_done = 1
		response = res.result()
		self.server_set_state = response.name

	# 设置自动回充的状态
	def set_charge_mode(self,value,max_callcount=10):
		# 注:不可在回调函数调用服务,否则卡死
		try:
			if not self.set_charge.wait_for_service(2):
				raise TimeoutError("Service call time out")
	
		except TimeoutError as e:
			print_and_fixRetract(RED+'自动回充状态设置失败.原因:等待服务超时,请确认底盘节点是否被开启.'+RESET)
			return

		state = None  # 调用结果
		call_time = 0 # 调用失败后尝试调用的次数记录
		if round(value)==1 or round(value)==2:
			print("正在开启自动回充功能,等待响应,请确保底盘节点已开启...")
		else:
			print("正在关闭自动回充功能,等待响应,请确保底盘节点已开启...")
		while True:
			try:
				req = Spawn.Request()
				req.x = float(value)
				self.set_charge.call_async(req).add_done_callback(self.wait_server_callback)

				# 死循环等待响应结果
				while True:
					rclpy.spin_once(self)
					if self.wait_server_done==1:
						self.wait_server_done = 0
						break
				# 输出结果
				state = self.server_set_state
				
			except Exception  as e:
				print(e)
				state =  "false"

			if state=="true":
				self.server_set_state = None
				print("回充状态设置成功.")
				# 调用成功,跳出循环
				break
			else:
				# 记录失败的次数
				call_time = call_time + 1
				if call_time>max_callcount:
					print_and_fixRetract(RED+'尝试与底盘通信多次失败,无法开启自动回充功能,请检查底层设备是否正确.'+RESET)
					self.server_set_state = None
					break
			
			time.sleep(0.5)
				

	def Pub_Charger_Position(self):
		'''使用最新充电桩位置计算并发布实际回充导航目标。'''
		if not self.Charger_Position_Is_Valid():
			print_and_fixRetract(RED+'充电桩坐标格式异常，未发出导航目标；请检查 Charger_Position.json。'+RESET)
			self.Finish_Task()
			return False

		nav_goal_data = self.Build_Nav_Goal_Data()
		if nav_goal_data is None:
			print_and_fixRetract(RED+'无法根据充电桩坐标计算回充导航目标。'+RESET)
			self.Finish_Task()
			return False

		self.nav_goal_data = nav_goal_data
		self.star_getNav_Feedback_Flag=1
		self.last_nav_distance_remaining = None
		self.nav_goal_sent_time = time.time()
		self.last_nav_feedback_time = None
		self.nav_progress_reference_distance = None
		self.nav_progress_reference_time = self.nav_goal_sent_time
		self.nav_server_missing_since = None

		nav_goal=PoseStamped()
		nav_goal.header.frame_id = 'map'
		nav_goal.header.stamp = self.get_clock().now().to_msg()
		nav_goal.pose.position.x = nav_goal_data['p_x']
		nav_goal.pose.position.y = nav_goal_data['p_y']
		nav_goal.pose.orientation.z = nav_goal_data['orien_z']
		nav_goal.pose.orientation.w = nav_goal_data['orien_w']

		self.Pub_Charger_marker(
			self.json_data['p_x'],
			self.json_data['p_y'],
			self.json_data['orien_z'],
			self.json_data['orien_w'])

		print_and_fixRetract(
			'回充导航目标: x=%.3f, y=%.3f, yaw=%.1fdeg；充电桩坐标: x=%.3f, y=%.3f.' % (
				nav_goal_data['p_x'],
				nav_goal_data['p_y'],
				math.degrees(nav_goal_data['yaw']),
				float(self.json_data['p_x']),
				float(self.json_data['p_y'])))

		if not self.Send_Nav_Goal(nav_goal):
			print_and_fixRetract(
				RED+'导航目标发送失败，本次回充已退出，请重新发起回充。'+RESET)
			self.Finish_Task()
			return False
		return True

	def Pub_NavGoal_Cancel(self):
		"""异步取消 Nav2 目标；主线程不调用 cancel_goal_async，避免阻塞自旋找红外。"""
		self.star_getNav_Feedback_Flag = 0
		self.nav_goal_sent_time = None
		try:
			subprocess.Popen(
				[
					'/opt/ros/humble/bin/ros2',
					'service',
					'call',
					'/navigate_to_pose/_action/cancel_goal',
					'action_msgs/srv/CancelGoal',
					'{}',
				],
				stdout=subprocess.DEVNULL,
				stderr=subprocess.DEVNULL,
			)
			print_and_fixRetract(YELLOW+'已异步发送导航取消请求，继续进入回充末段判断。'+RESET)
		except Exception as exc:
			print_and_fixRetract(YELLOW+'异步发送导航取消请求失败或 Nav2 已不可用: '+str(exc)+RESET)

	def Charger_Position_Is_Valid(self):
		'''只校验充电桩坐标格式；原点允许作为固定充电桩坐标。'''
		return self.Get_Charger_Pose_Data() is not None

	def Get_Charger_Pose_Data(self):
		try:
			p_x = float(self.json_data['p_x'])
			p_y = float(self.json_data['p_y'])
			o_z = float(self.json_data['orien_z'])
			o_w = float(self.json_data['orien_w'])
		except Exception:
			return None

		if not all(math.isfinite(v) for v in (p_x, p_y, o_z, o_w)):
			return None

		q_norm = math.sqrt(o_z * o_z + o_w * o_w)
		if not 0.5 <= q_norm <= 1.5:
			return None
		if q_norm > 0.0:
			o_z = o_z / q_norm
			o_w = o_w / q_norm
		yaw = math.atan2(2*(o_w*o_z), 1-2*(o_z**2))
		return {'p_x': p_x, 'p_y': p_y, 'orien_z': o_z, 'orien_w': o_w, 'yaw': yaw}

	def Build_Nav_Goal_Data(self):
		charger = self.Get_Charger_Pose_Data()
		if charger is None:
			return None

		# 当前项目中充电桩坐标被固定为建图原点，Nav2 目标直接使用该坐标。
		# 末段对接交给底盘红外回充逻辑处理，避免 diff_point/diff_angle 造成目标偏移。
		return {
			'p_x': charger['p_x'],
			'p_y': charger['p_y'],
			'orien_z': charger['orien_z'],
			'orien_w': charger['orien_w'],
			'yaw': charger['yaw'],
		}

	def Clear_Nav_Action_State(self):
		self.nav_goal_future = None
		self.nav_goal_handle = None
		self.nav_result_future = None
		self.nav_goal_response_deadline = None
		self.nav_last_failure_reason = ''

	def Nav_Feedback_Callback(self, feedback_msg):
		try:
			self.Update_Nav_Feedback(feedback_msg.feedback)
		except Exception:
			pass

	def Send_Nav_Goal(self, nav_goal):
		if not self.Nav_Action_Server_Available():
			self.nav_last_failure_reason = 'Nav2 action server 不可用'
			return False
		goal_msg = NavigateToPose.Goal()
		goal_msg.pose = nav_goal
		goal_msg.behavior_tree = ''
		self.Reset_Near_Goal_Handoff()
		self.Clear_Nav_Action_State()
		try:
			self.nav_goal_future = self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.Nav_Feedback_Callback)
			self.nav_goal_response_deadline = time.time() + self.nav_goal_response_timeout_sec
			print_and_fixRetract('已非阻塞发送回充导航目标，等待 Nav2 goal response。')
			return True
		except Exception as exc:
			self.nav_last_failure_reason = '发送导航目标异常: ' + str(exc)
			return False

	def Get_Nav_Action_Result(self):
		if self.nav_goal_future is None:
			return None
		if self.nav_goal_handle is None:
			if not self.nav_goal_future.done():
				if self.nav_goal_response_deadline is not None and time.time() > self.nav_goal_response_deadline:
					self.nav_last_failure_reason = 'Nav2 goal response 超时'
					return TaskResult.FAILED
				return None
			try:
				self.nav_goal_handle = self.nav_goal_future.result()
			except Exception as exc:
				self.nav_last_failure_reason = 'Nav2 goal response 异常: ' + str(exc)
				return TaskResult.FAILED
			if not self.nav_goal_handle.accepted:
				self.nav_last_failure_reason = 'Nav2 拒绝回充导航目标'
				return TaskResult.FAILED
			self.nav_result_future = self.nav_goal_handle.get_result_async()
			goal_id = ''.join(
				format(int(value), '02x')
				for value in self.nav_goal_handle.goal_id.uuid)
			print_and_fixRetract(
				'Nav2 已接受回充导航目标，goal_id=' + goal_id + '。')
			return None
		if self.nav_result_future is None or not self.nav_result_future.done():
			return None
		try:
			status = self.nav_result_future.result().status
		except Exception as exc:
			self.nav_last_failure_reason = 'Nav2 result 异常: ' + str(exc)
			return TaskResult.FAILED
		goal_id = 'unknown'
		if self.nav_goal_handle is not None:
			goal_id = ''.join(
				format(int(value), '02x')
				for value in self.nav_goal_handle.goal_id.uuid)
		print_and_fixRetract('Nav2 action result: goal_id=%s, status=%d。' % (goal_id, status))
		if status == GoalStatus.STATUS_SUCCEEDED:
			return TaskResult.SUCCEEDED
		if status == GoalStatus.STATUS_CANCELED:
			return TaskResult.CANCELED
		self.nav_last_failure_reason = 'Nav2 导航失败，status=' + str(status)
		return TaskResult.FAILED

	def Reset_Nav_Tracking(self):
		self.nav_goal_sent_time = None
		self.last_nav_feedback_time = None
		self.last_nav_distance_remaining = None
		self.nav_progress_reference_distance = None
		self.nav_progress_reference_time = None
		self.nav_server_missing_since = None
		self.Reset_Near_Goal_Handoff()
		self.Clear_Nav_Action_State()

	def Wait_For_Nav2_Action(self, timeout_sec=12.0):
		'''只等待 Nav2 NavigateToPose action server，不调用 waitUntilNav2Active，避免重置 AMCL 位姿。'''
		deadline = time.time() + timeout_sec
		while rclpy.ok() and time.time() < deadline:
			try:
				if self.nav_action_client.wait_for_server(timeout_sec=0.5):
					return True
			except Exception:
				pass
			rclpy.spin_once(self, timeout_sec=0.05)
		return False

	def Nav_Action_Server_Available(self):
		try:
			return self.nav_action_client.server_is_ready()
		except Exception:
			# 老版本 simple commander 无该字段时不据此判故障。
			return True

	def Nav_Server_Missing_Too_Long(self):
		if self.Nav_Action_Server_Available():
			self.nav_server_missing_since = None
			return False
		now = time.time()
		if self.nav_server_missing_since is None:
			self.nav_server_missing_since = now
			return False
		return now - self.nav_server_missing_since > self.nav_server_missing_timeout_sec

	def Update_Nav_Feedback(self, nav_feedback):
		try:
			distance = float(nav_feedback.distance_remaining)
		except Exception:
			return
		now = time.time()
		self.last_nav_feedback_time = now
		self.last_nav_distance_remaining = distance
		if (self.nav_progress_reference_distance is None or
			distance < self.nav_progress_reference_distance - self.nav_progress_epsilon):
			self.nav_progress_reference_distance = distance
			self.nav_progress_reference_time = now

	def Nav_Feedback_Stalled_Too_Long(self):
		if self.nav_goal_sent_time is None:
			return False
		now = time.time()
		if now - self.nav_goal_sent_time > self.nav_total_timeout_sec:
			return True
		if self.last_nav_feedback_time is None:
			return now - self.nav_goal_sent_time > self.nav_no_feedback_timeout_sec
		if self.nav_progress_reference_time is None:
			self.nav_progress_reference_time = now
			return False
		return now - self.nav_progress_reference_time > self.nav_feedback_stall_timeout_sec

	def Reset_Near_Goal_Handoff(self):
		self.near_goal_handoff_stable_since = None
		self.near_goal_handoff_phase = None
		self.near_goal_handoff_cancel_future = None
		self.near_goal_handoff_deadline = None
		self.near_goal_handoff_settle_until = None

	def Near_Goal_Tf_Sample(self):
		'''返回新鲜 map->base_footprint TF 对回充目标的距离和 TF age。'''
		try:
			transform = self.tf_buffer.lookup_transform(
				'map', 'base_footprint', Time(),
				timeout=Duration(seconds=0.2))
			goal = self.nav_goal_data or self.Build_Nav_Goal_Data()
			if goal is None:
				return None
			now_sec = self.get_clock().now().nanoseconds / 1e9
			stamp = transform.header.stamp
			stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
			age = now_sec - stamp_sec
			if age < -0.1 or age > self.near_goal_handoff_tf_max_age_sec:
				return None
			dx = transform.transform.translation.x - float(goal['p_x'])
			dy = transform.transform.translation.y - float(goal['p_y'])
			rotation = transform.transform.rotation
			robot_yaw = math.atan2(
				2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
				1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
			yaw_error = abs(self.Normalize_Angle(float(goal['yaw']) - robot_yaw))
			return math.hypot(dx, dy), yaw_error, max(0.0, age)
		except (TransformException, LookupError, KeyError, TypeError, ValueError, AttributeError):
			return None

	def Begin_Near_Goal_Handoff(self, distance, yaw_error, tf_age):
		'''只取消当前回充 goal，等待 Nav2 完整结束后再接管速度。'''
		if self.nav_goal_handle is None or self.nav_result_future is None:
			return False
		if self.nav_result_future.done():
			return False
		goal_id = ''.join(
			format(int(value), '02x')
			for value in self.nav_goal_handle.goal_id.uuid)
		try:
			self.near_goal_handoff_cancel_future = self.nav_goal_handle.cancel_goal_async()
		except Exception as exc:
			print_and_fixRetract(
				RED+'近桩接管取消当前 Nav2 goal 失败: '+str(exc)+'；本次回充已退出。'+RESET)
			self.Finish_Task()
			return True
		self.star_getNav_Feedback_Flag = 0
		self.near_goal_handoff_phase = 'canceling'
		self.near_goal_handoff_deadline = (
			time.time() + self.near_goal_handoff_cancel_timeout_sec)
		self.Stop_Motion(repeat=1, interval=0.0)
		print_and_fixRetract(
			'Near-goal pose accepted: distance=%.3fm, yaw_error=%.1fdeg '
			'(limits %.3fm / %.1fdeg).' % (
				distance, math.degrees(yaw_error),
				self.near_goal_handoff_distance,
				math.degrees(self.near_goal_handoff_yaw_tolerance)))
		print_and_fixRetract(
			YELLOW+'近桩接管触发: goal_id=%s, distance=%.3fm, tf_age=%.3fs；'
			'仅取消当前回充 goal。'% (goal_id, distance, tf_age)+RESET)
		return True

	def Start_Near_Goal_Settle(self, result_status):
		self.near_goal_handoff_phase = 'settling'
		self.near_goal_handoff_settle_until = (
			time.time() + self.near_goal_handoff_settle_sec)
		self.Stop_Motion(repeat=8, interval=0.05)
		print_and_fixRetract(
			'当前回充 Nav2 goal 已结束(status=%d)，压零速度 %.1fs 后进入红外搜索。' % (
				result_status, self.near_goal_handoff_settle_sec))

	def Update_Near_Goal_Handoff(self):
		'''非阻塞推进近桩稳定判定、当前 goal 取消和停稳阶段。'''
		now = time.time()
		if self.near_goal_handoff_phase == 'canceling':
			self.Stop_Motion(repeat=1, interval=0.0)
			if self.nav_result_future is not None and self.nav_result_future.done():
				try:
					status = self.nav_result_future.result().status
				except Exception as exc:
					print_and_fixRetract(
						RED+'近桩接管读取 Nav2 结束状态失败: '+str(exc)+'；本次回充已退出。'+RESET)
					self.Finish_Task()
					return True
				if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
					self.Start_Near_Goal_Settle(status)
				else:
					print_and_fixRetract(
						RED+'近桩接管期间 Nav2 异常结束(status=%d)，本次回充已退出。'%status+RESET)
					self.Finish_Task()
				return True
			if (self.near_goal_handoff_cancel_future is not None and
				self.near_goal_handoff_cancel_future.done()):
				try:
					cancel_response = self.near_goal_handoff_cancel_future.result()
				except Exception as exc:
					print_and_fixRetract(
						RED+'近桩接管取消响应异常: '+str(exc)+'；本次回充已退出。'+RESET)
					self.Finish_Task()
					return True
				if (cancel_response is not None and
					len(cancel_response.goals_canceling) > 0):
					# The server has accepted cancellation for this exact handle.
					# Keep publishing zero during the settle window so the
					# controller cannot race the infrared-search command.
					self.Start_Near_Goal_Settle(GoalStatus.STATUS_CANCELING)
					return True
			if (self.near_goal_handoff_deadline is not None and
				now > self.near_goal_handoff_deadline):
				print_and_fixRetract(
					RED+'取消当前回充 Nav2 goal 超时，拒绝与 Nav2 争抢速度，本次回充已退出。'+RESET)
				self.Finish_Task()
			return True

		if self.near_goal_handoff_phase == 'settling':
			self.Stop_Motion(repeat=1, interval=0.0)
			if (self.near_goal_handoff_settle_until is not None and
				now >= self.near_goal_handoff_settle_until):
				print_and_fixRetract(
					GREEN+'近桩接管完成，立即开始左右递增搜索红外源。'+RESET)
				if not self.Start_Turn_Search():
					self.Finish_Task()
			return True

		if (self.star_getNav_Feedback_Flag != 1 or
			self.nav_goal_handle is None or
			self.nav_result_future is None):
			self.near_goal_handoff_stable_since = None
			return False
		# 正常 SUCCEEDED 已经就绪时交给原结果分支处理，不发送取消。
		if self.nav_result_future.done():
			self.near_goal_handoff_stable_since = None
			return False
		sample = self.Near_Goal_Tf_Sample()
		if sample is None:
			self.near_goal_handoff_stable_since = None
			return False
		distance, yaw_error, tf_age = sample
		if (distance > self.near_goal_handoff_distance or
			yaw_error > self.near_goal_handoff_yaw_tolerance):
			self.near_goal_handoff_stable_since = None
			return False
		if self.near_goal_handoff_stable_since is None:
			self.near_goal_handoff_stable_since = now
			print_and_fixRetract(
				'Near-goal candidate pose: distance=%.3fm, yaw_error=%.1fdeg; '
				'both must remain within tolerance for %.1fs.' % (
					distance, math.degrees(yaw_error),
					self.near_goal_handoff_stable_sec))
			print_and_fixRetract(
				'进入近桩候选区: distance=%.3fm, tf_age=%.3fs；等待稳定 %.1fs。' % (
					distance, tf_age, self.near_goal_handoff_stable_sec))
			return False
		if now - self.near_goal_handoff_stable_since < self.near_goal_handoff_stable_sec:
			return False
		return self.Begin_Near_Goal_Handoff(distance, yaw_error, tf_age)

	def Stop_Motion(self, repeat=8, interval=0.05):
		'''连续发布零速度，压住自转/对接切换瞬间的残留速度。'''
		topic=Twist()
		for _ in range(repeat):
			self.Cmd_vel_pub.publish(topic)
			time.sleep(interval)

	def Current_Robot_Yaw(self):
		try:
			transform = self.tf_buffer.lookup_transform(
				'map', 'base_footprint', Time(),
				timeout=Duration(seconds=0.2))
			robot_q = transform.transform.rotation
			return math.atan2(
				2 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y),
				1 - 2 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z))
		except (TransformException, LookupError, AttributeError):
			return None

	def Reset_Turn_Search_State(self):
		self.start_turn = 0
		self.turn_start_time = None
		self.search_center_yaw = None
		self.search_target_yaw = None
		self.search_side = 1
		self.search_extent = self.search_increment

	def Advance_Turn_Search_Target(self):
		if self.search_side > 0:
			self.search_side = -1
		else:
			self.search_side = 1
			self.search_extent = min(
				self.search_extent + self.search_increment,
				self.search_max_extent)
		self.search_target_yaw = self.Normalize_Angle(
			self.search_center_yaw + self.search_side * self.search_extent)
		print_and_fixRetract(
			'红外搜索下一目标: %s %.0fdeg（相对起始朝向）。' % (
				'左' if self.search_side > 0 else '右',
				math.degrees(self.search_extent)))

	def Start_Turn_Search(self):
		robot_yaw = self.Current_Robot_Yaw()
		if robot_yaw is None:
			print_and_fixRetract(RED+'无法读取机器人实际朝向，取消红外搜索并完整退出回充节点.'+RESET)
			self.Finish_Task()
			return False
		self.search_center_yaw = robot_yaw
		self.search_side = 1
		self.search_extent = self.search_increment
		self.search_target_yaw = self.Normalize_Angle(
			self.search_center_yaw + self.search_extent)
		self.start_turn = 1
		self.find_redsignal = 0
		self.Reset_Nav_Tracking()
		self.turn_start_time = time.time()
		print_and_fixRetract(
			YELLOW+'开始左右递增搜索红外源: 左15deg -> 右15deg -> 左30deg -> 右30deg ...'+RESET)
		self.Update_Turn_Search()
		return True

	def Update_Turn_Search(self):
		if self.start_turn != 1:
			return
		if (self.turn_start_time is None or
			time.time() - self.turn_start_time > self.turn_timeout_sec):
			self.Reset_Turn_Search_State()
			self.Stop_Charge()
			self.Stop_Motion()
			self.task_finished = True
			print_and_fixRetract(
				RED+'左右递增搜索红外超时,已停止自动回充.'+RESET)
			return
		robot_yaw = self.Current_Robot_Yaw()
		if robot_yaw is None or self.search_target_yaw is None:
			self.Stop_Motion(repeat=1, interval=0.0)
			return
		yaw_error = self.Normalize_Angle(self.search_target_yaw - robot_yaw)
		if abs(yaw_error) <= self.search_yaw_tolerance:
			self.Stop_Motion(repeat=2, interval=0.02)
			self.Advance_Turn_Search_Target()
			yaw_error = self.Normalize_Angle(
				self.search_target_yaw - robot_yaw)
		topic = Twist()
		topic.angular.z = math.copysign(
			self.search_angular_speed, yaw_error)
		self.Cmd_vel_pub.publish(topic)

	def Prepare_New_Task(self):
		'''Reset per-task state while keeping ROS subscriptions and action clients alive.'''
		self.task_finished = False
		self.chargeflag = 0
		self.find_redsignal = 0
		self.last_red_seen_time = (
			time.time() if self.robot['RED'] == 1 or self.red_count > 0 else None
		)
		self.charge_complete = 0
		self.last_charge_complete = 0
		self.power_lost_count = 0
		self.lost_power_once = 1
		self.last_time = self.get_clock().now()
		self.lost_red_flag = self.get_clock().now()
		self.nav_end_z = 0
		self.nav_goal_data = self.Build_Nav_Goal_Data()
		self.Clear_Strict_Refinement()
		self.Reset_Nav_Tracking()
		self.Reset_Turn_Search_State()

	def Cancel_Current_Task(self):
		'''Stop only the active recharge task; keep this long-lived node available.'''
		exact_cancel_sent = False
		if self.nav_goal_handle is not None:
			try:
				self.nav_goal_handle.cancel_goal_async()
				exact_cancel_sent = True
			except Exception:
				pass
		if self.strict_refine_goal_handle is not None:
			try:
				self.strict_refine_goal_handle.cancel_goal_async()
			except Exception:
				pass
		self.Finish_Task()
		return exact_cancel_sent

	def Finish_Task(self):
		self.star_getNav_Feedback_Flag = 0
		if self.strict_refine_goal_handle is not None:
			try:
				self.strict_refine_goal_handle.cancel_goal_async()
			except Exception:
				pass
		self.Clear_Strict_Refinement()
		self.Reset_Nav_Tracking()
		self.Reset_Turn_Search_State()
		self.chargeflag = 0
		self.Stop_Motion()
		self.Pub_Recharger_Flag()
		self.task_finished = True

	def Has_Recent_Red_Signal(self):
		if self.robot['RED']==1 or self.red_count>0:
			return True
		return self.last_red_seen_time is not None and time.time() - self.last_red_seen_time <= self.red_recent_timeout_sec

	def Normalize_Angle(self, angle):
		return math.atan2(math.sin(angle), math.cos(angle))

	def Goal_Error_To_Charger_Goal(self):
		try:
			transform = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(), timeout=Duration(seconds=0.5))
			goal = self.nav_goal_data or self.Build_Nav_Goal_Data()
			if goal is None:
				return None
			dx = transform.transform.translation.x - float(goal['p_x'])
			dy = transform.transform.translation.y - float(goal['p_y'])
			robot_q = transform.transform.rotation
			robot_yaw = math.atan2(
				2 * (robot_q.w * robot_q.z + robot_q.x * robot_q.y),
				1 - 2 * (robot_q.y * robot_q.y + robot_q.z * robot_q.z))
			yaw_error = abs(self.Normalize_Angle(robot_yaw - float(goal['yaw'])))
			distance = math.sqrt(dx * dx + dy * dy)
			return dx, dy, distance, yaw_error, robot_yaw, float(goal['yaw'])
		except (TransformException, LookupError, KeyError, TypeError, ValueError, AttributeError):
			return None

	def Distance_To_Charger_Goal(self):
		error = self.Goal_Error_To_Charger_Goal()
		if error is None:
			return None
		return error[2]

	def Is_Near_Charger_Goal(self):
		error = self.Goal_Error_To_Charger_Goal()
		if error is None:
			return False
		_, _, distance, yaw_error, _, _ = error
		return (
			distance <= self.near_goal_dock_distance and
			yaw_error <= self.near_goal_yaw_tolerance)

	def Clear_Strict_Refinement(self):
		self.strict_refine_phase = None
		self.strict_refine_goal_future = None
		self.strict_refine_goal_handle = None
		self.strict_refine_result_future = None
		self.strict_refine_deadline = None
		self.strict_refine_reason = ''

	def Send_Strict_Behavior(self, phase, client, goal_msg, timeout_sec):
		try:
			if not client.wait_for_server(timeout_sec=0.5):
				print_and_fixRetract(RED+'回充精调失败：Nav2行为服务未上线，完整退出回充节点.'+RESET)
				return False
			self.strict_refine_phase = phase
			self.strict_refine_goal_future = client.send_goal_async(goal_msg)
			self.strict_refine_goal_handle = None
			self.strict_refine_result_future = None
			self.strict_refine_deadline = time.time() + timeout_sec
			return True
		except Exception as exc:
			print_and_fixRetract(RED+'回充精调动作发送失败: '+str(exc)+RESET)
			return False

	def Start_Strict_Refinement(self, reason):
		error = self.Goal_Error_To_Charger_Goal()
		if error is None:
			print_and_fixRetract(RED+reason+'，无法读取位姿，完整退出回充节点.'+RESET)
			return False
		dx, dy, distance, yaw_error, robot_yaw, _ = error
		self.strict_refine_reason = reason
		print_and_fixRetract(
			'回充粗导航结果: dx=%+.3fm, dy=%+.3fm, distance=%.3fm, yaw_error=%.1fdeg。' % (
				dx, dy, distance, math.degrees(yaw_error)))
		if distance <= self.near_goal_dock_distance and yaw_error <= self.near_goal_yaw_tolerance:
			return self.Enter_End_Recharge_Phase(reason)
		if distance > self.strict_refine_max_distance:
			print_and_fixRetract(
				RED+reason+'，距离误差超过回充精调上限 %.2fm，完整退出回充节点.' %
				self.strict_refine_max_distance+RESET)
			return False
		goal_heading = math.atan2(-dy, -dx)
		turn_angle = self.Normalize_Angle(goal_heading - robot_yaw)
		if distance > self.near_goal_dock_distance and abs(turn_angle) > self.strict_refine_turn_threshold:
			goal_msg = Spin.Goal()
			goal_msg.target_yaw = float(turn_angle)
			goal_msg.time_allowance.sec = 12
			print_and_fixRetract('回充精调：先转向目标点 %.1fdeg。' % math.degrees(turn_angle))
			return self.Send_Strict_Behavior('turn_to_goal', self.strict_spin_client, goal_msg, 14.0)
		return self.Start_Strict_Drive()

	def Start_Strict_Drive(self):
		error = self.Goal_Error_To_Charger_Goal()
		if error is None:
			return False
		_, _, distance, _, _, _ = error
		if distance <= self.near_goal_dock_distance:
			return self.Start_Strict_Final_Turn()
		if distance > self.strict_refine_max_distance:
			return False
		goal_msg = DriveOnHeading.Goal()
		goal_msg.target.x = float(distance)
		goal_msg.target.y = 0.0
		goal_msg.target.z = 0.0
		goal_msg.speed = float(self.strict_refine_drive_speed)
		allowance = int(math.ceil(distance / self.strict_refine_drive_speed + 6.0))
		goal_msg.time_allowance.sec = max(8, allowance)
		print_and_fixRetract(
			'回充精调：带碰撞检测低速直行 %.3fm，速度 %.2fm/s。' %
			(distance, self.strict_refine_drive_speed))
		return self.Send_Strict_Behavior(
			'drive_to_goal', self.strict_drive_client, goal_msg,
			min(self.strict_refine_action_timeout_sec, goal_msg.time_allowance.sec + 2.0))

	def Start_Strict_Final_Turn(self):
		error = self.Goal_Error_To_Charger_Goal()
		if error is None:
			return False
		_, _, distance, yaw_error, robot_yaw, goal_yaw = error
		if distance > self.near_goal_dock_distance:
			print_and_fixRetract(
				RED+'回充精调直行后位置仍未进入 %.2fm 容差，完整退出回充节点.' %
				self.near_goal_dock_distance+RESET)
			return False
		if yaw_error <= self.near_goal_yaw_tolerance:
			return self.Finish_Strict_Refinement()
		turn_angle = self.Normalize_Angle(goal_yaw - robot_yaw)
		goal_msg = Spin.Goal()
		goal_msg.target_yaw = float(turn_angle)
		goal_msg.time_allowance.sec = 12
		print_and_fixRetract('回充精调：修正最终朝向 %.1fdeg。' % math.degrees(turn_angle))
		return self.Send_Strict_Behavior('turn_to_final_yaw', self.strict_spin_client, goal_msg, 14.0)

	def Finish_Strict_Refinement(self):
		reason = self.strict_refine_reason or '回充精调完成'
		self.strict_refine_phase = None
		if not self.Enter_End_Recharge_Phase(reason+'，精调完成'):
			self.Finish_Task()
			return False
		return True

	def Fail_Strict_Refinement(self, message):
		print_and_fixRetract(RED+message+'；停止本次任务并完整退出回充节点，请用户重新发起回充.'+RESET)
		self.Clear_Strict_Refinement()
		self.Finish_Task()

	def Poll_Strict_Refinement(self):
		if self.strict_refine_phase is None:
			return
		if self.strict_refine_deadline is not None and time.time() > self.strict_refine_deadline:
			if self.strict_refine_goal_handle is not None:
				try:
					self.strict_refine_goal_handle.cancel_goal_async()
				except Exception:
					pass
			self.Fail_Strict_Refinement('回充精调动作超时')
			return
		if self.strict_refine_goal_handle is None:
			if self.strict_refine_goal_future is None or not self.strict_refine_goal_future.done():
				return
			try:
				self.strict_refine_goal_handle = self.strict_refine_goal_future.result()
			except Exception as exc:
				self.Fail_Strict_Refinement('回充精调目标响应异常: '+str(exc))
				return
			if not self.strict_refine_goal_handle.accepted:
				self.Fail_Strict_Refinement('回充精调目标被拒绝')
				return
			self.strict_refine_result_future = self.strict_refine_goal_handle.get_result_async()
			return
		if self.strict_refine_result_future is None or not self.strict_refine_result_future.done():
			return
		phase = self.strict_refine_phase
		try:
			status = self.strict_refine_result_future.result().status
		except Exception as exc:
			self.Fail_Strict_Refinement('回充精调结果异常: '+str(exc))
			return
		self.strict_refine_phase = None
		self.strict_refine_goal_future = None
		self.strict_refine_goal_handle = None
		self.strict_refine_result_future = None
		self.strict_refine_deadline = None
		if status != GoalStatus.STATUS_SUCCEEDED:
			self.Fail_Strict_Refinement('回充精调动作失败，status='+str(status))
			return
		if phase == 'turn_to_goal':
			if not self.Start_Strict_Drive():
				self.Fail_Strict_Refinement('回充精调无法开始直行')
		elif phase == 'drive_to_goal':
			if not self.Start_Strict_Final_Turn():
				self.Fail_Strict_Refinement('回充精调位置未达标')
		elif phase == 'turn_to_final_yaw':
			self.Finish_Strict_Refinement()

	def Start_Red_Docking(self):
		self.find_redsignal = 0
		self.Reset_Turn_Search_State()
		self.Stop_Motion()
		self.chargeflag=1
		self.Pub_Recharger_Flag()

	def Enter_End_Recharge_Phase(self, reason):
		print_and_fixRetract(
			YELLOW+reason+'，不再执行位置、朝向严格验收或精细调整，立即搜索红外源.'+RESET)
		return self.Start_Turn_Search()

	def Pub_Charger_marker(self, p_x, p_y, o_z, o_w):
		'''发布充电桩标定点和当前实际回充导航点的可视化。'''
		markerArray = MarkerArray()

		charger = self.Get_Charger_Pose_Data()
		if charger is None:
			return

		marker_shape  = Marker()
		marker_shape.id = 0
		marker_shape.header.frame_id = 'map'
		marker_shape.type = Marker.ARROW
		marker_shape.action = Marker.ADD
		marker_shape.scale.x = 0.5
		marker_shape.scale.y = 0.05
		marker_shape.scale.z = 0.05
		marker_shape.pose.position.x = charger['p_x']
		marker_shape.pose.position.y = charger['p_y']
		marker_shape.pose.position.z = 0.1
		marker_shape.pose.orientation.z = charger['orien_z']
		marker_shape.pose.orientation.w = charger['orien_w']
		marker_shape.color.r = 1.0
		marker_shape.color.g = 0.0
		marker_shape.color.b = 0.0
		marker_shape.color.a = 1.0
		markerArray.markers.append(marker_shape)

		marker_string = Marker()
		marker_string.id = 1
		marker_string.header.frame_id = 'map'
		marker_string.type = Marker.TEXT_VIEW_FACING
		marker_string.action = Marker.ADD
		marker_string.scale.x = 0.5
		marker_string.scale.y = 0.5
		marker_string.scale.z = 0.5
		marker_string.color.a = 1.0
		marker_string.color.r = 1.0
		marker_string.color.g = 0.0
		marker_string.color.b = 0.0
		marker_string.pose.position.x = charger['p_x']
		marker_string.pose.position.y = charger['p_y']
		marker_string.pose.position.z = 0.1
		marker_string.pose.orientation.z = charger['orien_z']
		marker_string.pose.orientation.w = charger['orien_w']
		marker_string.text = 'Charger'
		markerArray.markers.append(marker_string)

		nav_goal = self.nav_goal_data or self.Build_Nav_Goal_Data()
		if nav_goal is not None:
			nav_marker = Marker()
			nav_marker.id = 2
			nav_marker.header.frame_id = 'map'
			nav_marker.type = Marker.ARROW
			nav_marker.action = Marker.ADD
			nav_marker.scale.x = 0.45
			nav_marker.scale.y = 0.05
			nav_marker.scale.z = 0.05
			nav_marker.pose.position.x = nav_goal['p_x']
			nav_marker.pose.position.y = nav_goal['p_y']
			nav_marker.pose.position.z = 0.12
			nav_marker.pose.orientation.z = nav_goal['orien_z']
			nav_marker.pose.orientation.w = nav_goal['orien_w']
			nav_marker.color.r = 0.0
			nav_marker.color.g = 0.35
			nav_marker.color.b = 1.0
			nav_marker.color.a = 1.0
			markerArray.markers.append(nav_marker)

		self.Charger_marker_pub.publish(markerArray)

	def Pub_Recharger_Flag(self,set_velflag=0):
		'''发布自动回充任务是否开启标志位话题'''
		topic=Int8()
		topic.data=self.chargeflag
		for i in range(10):
			self.Recharger_Flag_pub.publish(topic)

	# def Pub_Recharger_Flag(self,set_velflag=0):
	# 	'''发布自动回充任务是否开启标志位话题'''

    #     # 先开回充，再开导航的情况
	# 	if set_velflag==1:
	# 		topic=Int8()
	# 		topic.data=self.chargeflag
	# 		for i in range(10):
	# 			self.Recharger_Flag_pub.publish(topic)
		
	# 	self.set_charge_mode(self.chargeflag)

	def Voltage_callback(self, topic):
		'''更新机器人电池电量'''
		self.robot['Voltage']=topic.data

	def Charging_Flag_callback(self, topic):
		'''更新机器人充电状态'''
		if(self.robot['Charging']==0 and topic.data==1):
			print_and_fixRetract(GREEN+"Charging started!"+RESET)
		if(self.robot['Charging']==1 and topic.data==0):
			print_and_fixRetract(YELLOW+"Charging disconnected!"+RESET)
		self.robot['Charging']=topic.data

	def Charging_Current_callback(self, topic):
		'''更新机器人充电电流数据'''
		self.robot['Charging_current']=topic.data
		
	def RED_Flag_callback(self, topic):
		self.red_count = topic.data
		if topic.data>0:
			self.last_red_seen_time = time.time()
		'''更新是否寻找到红外信号(充电桩)状态'''
		if self.robot['Charging']==0:
			#如果是导航寻找充电桩模式，红外信号消失时
			if topic.data==0 and self.robot['RED']==1:
				if((self.get_clock().now()-self.lost_red_flag).to_msg()).sec>=2:
					print_and_fixRetract(YELLOW+"Infrared signal lost."+RESET)
				self.lost_red_flag = self.get_clock().now()
	
			#红外信号出现
			if topic.data==1 and self.robot['RED']==0:
				print_and_fixRetract(GREEN+"Infrared signal founded."+RESET) 

		if topic.data>0:
			self.robot['RED']=1
		else:
			self.robot['RED']=0

		# 自转寻找红外时处理逻辑
		if self.start_turn==1:
			if self.robot['RED']==1:
				self.find_redsignal = self.find_redsignal + 1 
				# print(self.find_redsignal)
				# if self.find_redsignal>=3: # 稳定识别一段时间
				# 	self.find_redsignal = 0 
				# 	self.start_turn=0
				# 	print_and_fixRetract(GREEN+'已通过自转发现红外信号,开始对接充电.(Infrared signals have been detected by rotation. Docking and charging has begun.)'+RESET)
				# 	vel_topic=Twist()
				# 	self.Cmd_vel_pub.publish(vel_topic) # 停止运动
				# 	self.chargeflag=1 # 开启自动回充
				# 	self.Pub_Recharger_Flag()
			else:
				self.find_redsignal = 0

	def Position_Update_callback(self, topic):
		'''更新json文件中的充电桩标定位置；实际导航偏移在发目标时动态计算。'''
		position_dic={'p_x':0, 'p_y':0, 'orien_z':0, 'orien_w':0 }
		position_dic['p_x']=topic.pose.position.x
		position_dic['p_y']=topic.pose.position.y
		position_dic['orien_z']=topic.pose.orientation.z
		position_dic['orien_w']=topic.pose.orientation.w

		# 保存的是充电桩标定坐标，不再把 diff_point/diff_angle 写入 JSON，
		# 避免下一次启动时被再次偏移。
		with open(json_file, 'w') as fp:
			json.dump(position_dic, fp, ensure_ascii=False)
			print_and_fixRetract('New charging pile position saved.')
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)
		self.nav_goal_data = self.Build_Nav_Goal_Data()

	def Odom_callback(self, topic):
		'''更新的机器人实时位姿'''
		self.robot['Rotation_Z']=topic.pose.pose.position.z	 

	def Stop_Charge(self):
		#如果在导航回充模式下，关闭导航
		self.Pub_NavGoal_Cancel() 

		# 停止监听导航结果
		self.star_getNav_Feedback_Flag = 0

		self.lost_power_once=1
		
		#切换为停止回充模式
		self.chargeflag=0
		self.Pub_Recharger_Flag()

	def autoRecharger(self, key):
		'''键盘控制开始自动回充:1-导航控制寻找充电桩,2-纯回充装备控制寻找充电桩
		'''
		if self.start_turn == 1:
			self.Update_Turn_Search()
		elif (self.star_getNav_Feedback_Flag == 1 or
			self.near_goal_handoff_phase is not None):
			self.Update_Near_Goal_Handoff()

		# 如果机器人在充电中,则检测充电是否已经完成.
		if self.robot['Charging']==1:
			if (self.robot['Type']=='Plus'and self.robot['Voltage']>25) or (self.robot['Type']=='Mini' and self.robot['Voltage']>12.5):
				self.charge_complete=self.charge_complete+1
			else:
				self.charge_complete=0

		#导航控制寻找充电桩
		if key=='q' or key=='Q':
			# 存在3路以上的红外信号,小车姿态接近于对准充电桩,无需导航	
			if self.red_count>=3:
				self.Pub_NavGoal_Cancel()
				self.chargeflag=1
				self.Pub_Recharger_Flag()
				print_and_fixRetract('已捕获到高强度红外信号,使用红外信号对接.(High-intensity infrared signals have been captured and are docked using infrared signals.)')
			else:
				if self.Pub_Charger_Position():
					print_and_fixRetract('开始导航到充电桩位置.(Start navigating to the charging post location.)')

		#关闭自动回充 
		elif key=='e' or key=='E':
			print_and_fixRetract('停止寻找充电桩或停止充电.(Stop finding charging pile or charging.)')
			self.Stop_Charge()

		# 测试用
		elif key=='t' or key=='T':
			self.set_charge_mode(1)
		elif key=='y' or key=='Y':
			self.set_charge_mode(0)

		#电压过低时开启导航自动回充
		if self.robot['Charging']==0:
			if (self.robot['Type']=='Plus'and self.robot['Voltage']<20) or (self.robot['Type']=='Mini' and self.robot['Voltage']<10):
				time.sleep(1)
				self.power_lost_count=self.power_lost_count+1 # 低电量滤波

				# 低电量状态超过5次
				if self.power_lost_count>5 and self.lost_power_once==1:
					self.power_lost_count=0

					# 电量低且小车不在回充模式,开启导航充电
					if self.chargeflag==0:
						self.Pub_NavGoal_Cancel() # 取消导航

						if 'akm' in self.robot['car_mode']:
							self.chargeflag=2
						else:
							self.chargeflag=1
						self.Pub_Recharger_Flag(1) # 出现要优先开启自动回充然后再导航的情况,需要进行标志位传递
						if self.Pub_Charger_Position():
							print_and_fixRetract(YELLOW+'检测到电池电量低,即将导航到充电桩进行充电.(Detects low battery level and will navigate to a charging station for charging.)'+RESET)
							self.lost_power_once=0
	
			else:
				self.power_lost_count=0		

		# #频率1hz的循环任务
		if ((self.get_clock().now()-self.last_time).to_msg()).sec>=1:
			#发布充电桩位置话题
			self.Pub_Charger_marker(
				self.json_data['p_x'], 
				self.json_data['p_y'], 
				self.json_data['orien_z'], 
				self.json_data['orien_w'])
			self.last_time=self.get_clock().now()


			# 需要监听导航结果
			res = None
			if self.star_getNav_Feedback_Flag==1:
				if self.Nav_Server_Missing_Too_Long():
					print_and_fixRetract(
						RED+'Nav2 action server 连续不可用，本次回充已退出。'+RESET)
					self.Pub_NavGoal_Cancel()
					self.Finish_Task()
				else:
					res = self.Get_Nav_Action_Result()
					if res is not None:
						self.star_getNav_Feedback_Flag = 0
						if res==TaskResult.SUCCEEDED:
							print_and_fixRetract(
								GREEN+'Nav2 已判定到达回充目标点，立即开始旋转搜索红外源。'+RESET)
							if not self.Start_Turn_Search():
								self.Finish_Task()
						elif res==TaskResult.CANCELED:
							print_and_fixRetract('nav was canceled.')
							self.Finish_Task()
						elif res==TaskResult.FAILED:
							reason = self.nav_last_failure_reason or '导航失败'
							print_and_fixRetract(
								RED+'goal failed: '+reason+'；本次回充已退出。'+RESET)
							self.Finish_Task()
						else:
							print_and_fixRetract(
								RED+'导航结束但结果未知，本次回充已退出。'+RESET)
							self.Finish_Task()
					else:
						if self.Nav_Feedback_Stalled_Too_Long():
							print_and_fixRetract(
								RED+'导航反馈长时间无有效进展，本次回充已退出。'+RESET)
							self.Pub_NavGoal_Cancel()
							self.Finish_Task()
			#充电期间打印电池电压、充电时间
			if self.robot['Charging']==1:
				self.lost_power_once=1
				percent=0
				percen_form=0
				if self.robot['Type']=='Plus':
					percent= (self.robot['Voltage']-20)/5 
					percent_form=format(percent, '.0%')
				if self.robot['Type']=='Mini':
					percent= (self.robot['Voltage']-10)/2.5
					percent_form=format(percent, '.0%')
				print_and_fixRetract("Robot is charging.")
				print_and_fixRetract("Robot battery: "+str(round(self.robot['Voltage'], 2))+"V = "+str(percent_form)+
									 ", Charging current: "+str(round(self.robot['Charging_current'], 2))+"A.")
				mAh_time=0
				try:
					mAh_time=1/self.robot['Charging_current']/1000
				except ZeroDivisionError:
					pass
				left_battery=round(self.robot['BatteryCapacity']*percent, 2)
				if percent<1:
					need_charge_battery=self.robot['BatteryCapacity']-left_battery
					need_percent_form=format(1-percent, '.0%')		
					print_and_fixRetract(str(self.robot['BatteryCapacity'])+"mAh*"+str(need_percent_form)+"="+str(need_charge_battery)+"mAh need to be charge, "+
										 "cost "+str(round(need_charge_battery*mAh_time, 2))+" hours.")
				else:	
					print_and_fixRetract(GREEN+"Robot battery is full."+RESET)
				print_and_fixRetract("\n")

		 # 自转寻找红外执行判断
		if self.find_redsignal>=3:
			self.find_redsignal = 0
			print_and_fixRetract(GREEN+'已通过自转发现红外信号,开始对接充电.(Infrared signals have been detected by rotation. Docking and charging has begun.)'+RESET)
			self.Start_Red_Docking()

		#机器人充电完成判断
		if self.charge_complete>10:
			self.charge_complete=0
			if self.last_charge_complete!=0:
				self.last_charge_complete=0
				self.Stop_Charge()			
			print_and_fixRetract(GREEN+'充电已完成.(Chrge complete.)'+RESET)#Charging complete
		self.last_charge_complete=self.charge_complete

	
def main():
	rclpy.init()
	try:
		autorecharger=AutoRecharger() #创建自动回充类

		print_and_fixRetract("请先开启导航功能；当前仅等待 Nav2 action server，不重置机器人定位。")
		if not autorecharger.Wait_For_Nav2_Action(timeout_sec=12.0):
			print_and_fixRetract('Nav2 action server 未上线，已取消本次回充；请先说“开启导航”或手动启动 Nav2。')
			autorecharger.Finish_Task()
			return
		print_and_fixRetract(autorecharger.tips)
		
		tmp_sec = Int8()
		tmp_sec.data = 1
		tmp_vel = Twist()
		autorecharger.robot_security_off_pub.publish(tmp_sec)
		autorecharger.Cmd_vel_pub.publish(tmp_vel)
		
		# 初始化参数
		autorecharger.declare_parameter('robot_BatteryCapacity',5000)
		autorecharger.declare_parameter('car_mode',"mini_mec")
		autorecharger.declare_parameter('diff_point',1.2)
		autorecharger.declare_parameter('diff_angle',-15)

		# 获取参数
		with open(yaml_file,'r') as file:
			params = yaml.safe_load(file)

		# 参数赋值
		autorecharger.robot['BatteryCapacity'] = params['robot_info']['BatteryCapacity']
		autorecharger.robot['car_mode']        = params['robot_info']['car_mode']
		autorecharger.diff_point = params['robot_info']['diff_point']
		autorecharger.diff_angle = params['robot_info']['diff_angle']

		if autorecharger.robot['car_mode'][0:4]!='mini':
			autorecharger.robot['Type'] = 'Plus'
		else:
			autorecharger.robot['Type'] = 'Mini'
		
		while rclpy.ok():
			key = get_key(settings) #获取键值，会导致终端打印自动缩进
			autorecharger.autoRecharger(key) #开始自动回充功能
			rclpy.spin_once(autorecharger)
			if (key == '\x03'):
				topic=Twist()
				autorecharger.Cmd_vel_pub.publish(topic) #发布速度0话题
				autorecharger.chargeflag=0
				autorecharger.Pub_Recharger_Flag()# 关闭自动回充
				print_and_fixRetract('自动回充功能已关闭.(Quit AutoRecharger.)')#Auto charging quit
				break #Ctrl+C退出自动回充功能
		
	except Exception as e:
		print_and_fixRetract(e)
	
	finally:
		autorecharger.chargeflag=0
		autorecharger.Pub_Recharger_Flag()# 关闭自动回充,并传递标志位到cmd_vel的callback函数
		# 恢复终端属性
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	
	print('over.')
