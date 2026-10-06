import os
import threading
import time
from datetime import datetime
from math import atan2
from math import cos
from math import sin
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
except ImportError:  # pragma: no cover - allows local editing without ROS2 installed
    rclpy = None
    Twist = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Node = object

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:  # pragma: no cover - Nav2 may be unavailable during local editing
    NavigateToPose = None
    ActionClient = None


DEVICE_ID = os.getenv('ROBOT_DEVICE_ID', 'robot-001')
CMD_VEL_TOPIC = os.getenv('CMD_VEL_TOPIC', '/cmd_vel')
LINEAR_SPEED = float(os.getenv('MANUAL_LINEAR_SPEED', '0.1'))
ANGULAR_SPEED = float(os.getenv('MANUAL_ANGULAR_SPEED', '0.5'))
PUBLISH_HZ = float(os.getenv('PUBLISH_HZ', '10'))
ROBOT_POSE_TOPIC = os.getenv('ROBOT_POSE_TOPIC', '/amcl_pose')
NAVIGATE_ACTION_NAME = os.getenv('NAVIGATE_ACTION_NAME', 'navigate_to_pose')
# Keep a safety stop, but make it long enough that normal press-and-hold use
# is not interrupted by a short timeout or minor network jitter.
AUTO_STOP_TIMEOUT = float(os.getenv('AUTO_STOP_TIMEOUT', '5.0'))

app = FastAPI(title='robot-agent', version='1.0.0')


class ManualControlPayload(BaseModel):
    deviceId: str = DEVICE_ID
    action: str
    linear: Optional[float] = None
    angular: Optional[float] = None


class NavigationGoalPayload(BaseModel):
    deviceId: str = DEVICE_ID
    x: float
    y: float
    yaw: float = 0.0


def _quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    half_yaw = yaw * 0.5
    return sin(half_yaw), cos(half_yaw)


def _yaw_from_quaternion(orientation) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return atan2(siny_cosp, cosy_cosp)


def _utc_now() -> str:
    return datetime.utcnow().isoformat() + 'Z'


def _duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0


class VelocityPublisher:
    def __init__(self) -> None:
        if rclpy is None or Twist is None:
            raise RuntimeError('ROS2 Python packages are not available. Please source ROS2 first.')

        rclpy.init(args=None)
        self.node = Node('robot_manual_control_agent')
        self.publisher = self.node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.pose_subscription = None
        self.navigation_client = None
        if PoseWithCovarianceStamped is not None:
            self.pose_subscription = self.node.create_subscription(
                PoseWithCovarianceStamped,
                ROBOT_POSE_TOPIC,
                self._on_pose,
                10,
            )
        if ActionClient is not None and NavigateToPose is not None:
            self.navigation_client = ActionClient(self.node, NavigateToPose, NAVIGATE_ACTION_NAME)
        self.linear = 0.0
        self.angular = 0.0
        self.latest_pose = None
        self.navigation_state = {
            'ok': True,
            'deviceId': DEVICE_ID,
            'status': 'idle',
            'goal': None,
            'feedback': None,
            'result': None,
            'message': 'No active navigation goal',
            'updatedAt': _utc_now(),
        }
        self.navigation_lock = threading.Lock()
        self._active_goal_handle = None
        self.last_command_time = 0.0
        self.manual_active = False
        self.publish_stop_once = False
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def set_motion(self, linear: float, angular: float) -> None:
        with self.lock:
            self.linear = linear
            self.angular = angular
            self.last_command_time = time.time()
            self.manual_active = abs(linear) > 1e-6 or abs(angular) > 1e-6
            self.publish_stop_once = not self.manual_active

    def stop(self) -> None:
        self.set_motion(0.0, 0.0)

    def _on_pose(self, message) -> None:
        pose = message.pose.pose
        with self.lock:
            self.latest_pose = {
                'ok': True,
                'deviceId': DEVICE_ID,
                'frameId': message.header.frame_id or 'map',
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': _yaw_from_quaternion(pose.orientation),
                'stamp': _utc_now(),
                'message': 'Robot pose ready',
            }

    def get_pose(self) -> dict:
        with self.lock:
            if self.latest_pose is None:
                return {
                    'ok': False,
                    'deviceId': DEVICE_ID,
                    'message': 'Robot pose is not available yet',
                }
            return dict(self.latest_pose)

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict:
        if self.navigation_client is None:
            return {
                'ok': False,
                'deviceId': DEVICE_ID,
                'message': 'Navigation action client is not available',
            }
        if not self.navigation_client.wait_for_server(timeout_sec=2.0):
            return {
                'ok': False,
                'deviceId': DEVICE_ID,
                'message': 'Navigation action server is not ready',
            }

        with self.navigation_lock:
            active_goal_handle = self._active_goal_handle
            if active_goal_handle is not None and self.navigation_state.get('status') == 'navigating':
                try:
                    active_goal_handle.cancel_goal_async()
                except Exception:
                    pass

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        z, w = _quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w

        goal_snapshot = {
            'frameId': 'map',
            'x': x,
            'y': y,
            'yaw': yaw,
            'sentAt': _utc_now(),
        }
        with self.navigation_lock:
            self.navigation_state.update({
                'ok': True,
                'status': 'navigating',
                'goal': goal_snapshot,
                'feedback': None,
                'result': None,
                'message': 'Navigation goal sent',
                'updatedAt': _utc_now(),
            })

        future = self.navigation_client.send_goal_async(goal, feedback_callback=self._on_navigation_feedback)
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        message = 'Navigation goal sent'
        if future.done():
            try:
                goal_handle = future.result()
            except Exception as exc:
                with self.navigation_lock:
                    self.navigation_state.update({
                        'ok': False,
                        'status': 'failed',
                        'result': {
                            'statusCode': None,
                            'errorCode': None,
                            'errorMsg': str(exc),
                            'completedAt': _utc_now(),
                        },
                        'message': f'Navigation goal failed: {exc}',
                        'updatedAt': _utc_now(),
                    })
                return {
                    'ok': False,
                    'deviceId': DEVICE_ID,
                    'frameId': 'map',
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'message': f'Navigation goal failed: {exc}',
                }
            if not goal_handle.accepted:
                with self.navigation_lock:
                    self._active_goal_handle = None
                    self.navigation_state.update({
                        'ok': False,
                        'status': 'failed',
                        'result': {
                            'statusCode': None,
                            'errorCode': None,
                            'errorMsg': 'Navigation goal rejected',
                            'completedAt': _utc_now(),
                        },
                        'message': 'Navigation goal rejected',
                        'updatedAt': _utc_now(),
                    })
                return {
                    'ok': False,
                    'deviceId': DEVICE_ID,
                    'frameId': 'map',
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'message': 'Navigation goal rejected',
                }
            self._register_navigation_result(goal_handle)
            message = 'Navigation goal accepted'
            with self.navigation_lock:
                self.navigation_state.update({
                    'ok': True,
                    'status': 'navigating',
                    'message': message,
                    'updatedAt': _utc_now(),
                })
        else:
            future.add_done_callback(self._on_navigation_goal_response)

        return {
            'ok': True,
            'deviceId': DEVICE_ID,
            'frameId': 'map',
            'x': x,
            'y': y,
            'yaw': yaw,
            'message': message,
        }

    def _on_navigation_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            with self.navigation_lock:
                self.navigation_state.update({
                    'ok': False,
                    'status': 'failed',
                    'result': {
                        'statusCode': None,
                        'errorCode': None,
                        'errorMsg': str(exc),
                        'completedAt': _utc_now(),
                    },
                    'message': f'Navigation goal failed: {exc}',
                    'updatedAt': _utc_now(),
                })
            return

        if not goal_handle.accepted:
            with self.navigation_lock:
                self._active_goal_handle = None
                self.navigation_state.update({
                    'ok': False,
                    'status': 'failed',
                    'result': {
                        'statusCode': None,
                        'errorCode': None,
                        'errorMsg': 'Navigation goal rejected',
                        'completedAt': _utc_now(),
                    },
                    'message': 'Navigation goal rejected',
                    'updatedAt': _utc_now(),
                })
            return

        self._register_navigation_result(goal_handle)
        with self.navigation_lock:
            self.navigation_state.update({
                'ok': True,
                'status': 'navigating',
                'message': 'Navigation goal accepted',
                'updatedAt': _utc_now(),
            })

    def _register_navigation_result(self, goal_handle) -> None:
        with self.navigation_lock:
            self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose
        feedback_snapshot = {
            'currentPose': {
                'frameId': feedback.current_pose.header.frame_id or 'map',
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': _yaw_from_quaternion(pose.orientation),
            },
            'navigationTimeSec': _duration_to_seconds(feedback.navigation_time),
            'estimatedTimeRemainingSec': _duration_to_seconds(feedback.estimated_time_remaining),
            'numberOfRecoveries': feedback.number_of_recoveries,
            'distanceRemaining': feedback.distance_remaining,
            'stamp': _utc_now(),
        }
        with self.navigation_lock:
            self.navigation_state.update({
                'ok': True,
                'status': 'navigating',
                'feedback': feedback_snapshot,
                'message': 'Navigation feedback received',
                'updatedAt': _utc_now(),
            })

    def _on_navigation_result(self, future) -> None:
        try:
            result = future.result()
            status_code = result.status
        except Exception as exc:
            with self.navigation_lock:
                self._active_goal_handle = None
                self.navigation_state.update({
                    'ok': False,
                    'status': 'failed',
                    'result': {
                        'statusCode': None,
                        'errorCode': None,
                        'errorMsg': str(exc),
                        'completedAt': _utc_now(),
                    },
                    'message': f'Navigation result failed: {exc}',
                    'updatedAt': _utc_now(),
                })
            return

        if status_code == 4:
            status = 'succeeded'
            ok = True
            message = 'Navigation succeeded'
        elif status_code == 5:
            status = 'canceled'
            ok = False
            message = 'Navigation canceled'
        else:
            status = 'failed'
            ok = False
            message = f'Navigation failed with status {status_code}'

        with self.navigation_lock:
            self._active_goal_handle = None
            self.navigation_state.update({
                'ok': ok,
                'status': status,
                'result': {
                    'statusCode': status_code,
                    'errorCode': None,
                    'errorMsg': '' if ok else message,
                    'completedAt': _utc_now(),
                },
                'message': message,
                'updatedAt': _utc_now(),
            })

    def get_navigation_status(self) -> dict:
        with self.navigation_lock:
            return dict(self.navigation_state)

    def _loop(self) -> None:
        interval = 1.0 / PUBLISH_HZ if PUBLISH_HZ > 0 else 0.1
        while self.running:
            with self.lock:
                if self.manual_active and AUTO_STOP_TIMEOUT > 0 and time.time() - self.last_command_time > AUTO_STOP_TIMEOUT:
                    self.linear = 0.0
                    self.angular = 0.0
                    self.manual_active = False
                    self.publish_stop_once = True

                if self.manual_active or self.publish_stop_once:
                    message = Twist()
                    message.linear.x = self.linear
                    message.angular.z = self.angular
                    self.publisher.publish(message)
                    if self.publish_stop_once:
                        self.publish_stop_once = False
            rclpy.spin_once(self.node, timeout_sec=0)
            time.sleep(interval)

    def shutdown(self) -> None:
        self.running = False
        self.thread.join(timeout=1.0)
        self.node.destroy_node()
        rclpy.shutdown()


publisher: Optional[VelocityPublisher] = None
publisher_error: Optional[str] = None


@app.on_event('startup')
def on_startup() -> None:
    global publisher, publisher_error
    try:
        publisher = VelocityPublisher()
    except Exception as exc:  # pragma: no cover - depends on target environment
        publisher_error = str(exc)
        publisher = None


@app.get('/api/status')
def get_status() -> dict:
    return {
        'ok': publisher is not None,
        'deviceId': DEVICE_ID,
        'topic': CMD_VEL_TOPIC,
        'poseTopic': ROBOT_POSE_TOPIC,
        'navigateActionName': NAVIGATE_ACTION_NAME,
        'linearSpeed': LINEAR_SPEED,
        'angularSpeed': ANGULAR_SPEED,
        'autoStopTimeout': AUTO_STOP_TIMEOUT,
        'publisherReady': publisher is not None,
        'message': publisher_error or 'robot-agent ready',
    }


@app.get('/api/robot/pose')
def get_robot_pose() -> dict:
    if publisher is None:
        return {'ok': False, 'deviceId': DEVICE_ID, 'message': publisher_error or 'Publisher is not ready'}
    return publisher.get_pose()


@app.post('/api/navigation/goal')
def send_navigation_goal(payload: NavigationGoalPayload) -> dict:
    if payload.deviceId != DEVICE_ID:
        return {'ok': False, 'message': f'Unexpected deviceId: {payload.deviceId}'}
    if publisher is None:
        return {'ok': False, 'deviceId': DEVICE_ID, 'message': publisher_error or 'Publisher is not ready'}
    return publisher.send_navigation_goal(float(payload.x), float(payload.y), float(payload.yaw))


@app.get('/api/navigation/status')
def get_navigation_status() -> dict:
    if publisher is None:
        return {'ok': False, 'deviceId': DEVICE_ID, 'message': publisher_error or 'Publisher is not ready'}
    return publisher.get_navigation_status()


@app.post('/api/manual-control/start')
def start_manual_control(payload: ManualControlPayload) -> dict:
    if payload.deviceId != DEVICE_ID:
        return {'ok': False, 'message': f'Unexpected deviceId: {payload.deviceId}'}
    if publisher is None:
        return {'ok': False, 'message': publisher_error or 'Publisher is not ready'}

    linear = LINEAR_SPEED if payload.linear is None else float(payload.linear)
    angular = ANGULAR_SPEED if payload.angular is None else float(payload.angular)
    publisher.set_motion(linear, angular)
    return {
        'ok': True,
        'deviceId': DEVICE_ID,
        'action': payload.action,
        'linear': linear,
        'angular': angular,
        'message': f'Manual control started: {payload.action}',
    }


@app.post('/api/manual-control/stop')
def stop_manual_control(payload: ManualControlPayload) -> dict:
    if payload.deviceId != DEVICE_ID:
        return {'ok': False, 'message': f'Unexpected deviceId: {payload.deviceId}'}
    if publisher is None:
        return {'ok': False, 'message': publisher_error or 'Publisher is not ready'}

    publisher.stop()
    return {
        'ok': True,
        'deviceId': DEVICE_ID,
        'action': payload.action,
        'linear': 0.0,
        'angular': 0.0,
        'message': 'Manual control stopped',
    }
