import copy
import json
import os
import subprocess
import threading
import time
import uuid
from collections import OrderedDict
from datetime import datetime
from math import atan2
from math import cos
from math import isfinite
from math import sin
from typing import Optional

from fastapi import FastAPI
from fastapi import HTTPException
from pydantic import BaseModel

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration as RosDuration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time as RosTime
    from sensor_msgs.msg import Imu
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from std_msgs.msg import Float32
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
except ImportError:  # pragma: no cover - allows local editing without ROS2 installed
    rclpy = None
    Twist = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Odometry = None
    RosDuration = None
    Node = object
    DurabilityPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    qos_profile_sensor_data = None
    RosTime = None
    Imu = None
    LaserScan = None
    Bool = None
    Float32 = None
    String = None
    Trigger = None
    TFMessage = None

try:
    from tf2_ros import Buffer as Tf2Buffer
    from tf2_ros import TransformListener as Tf2TransformListener
except ImportError:  # pragma: no cover - tf2 may be unavailable during local editing
    Tf2Buffer = None
    Tf2TransformListener = None

try:
    from lifecycle_msgs.srv import GetState
except ImportError:  # pragma: no cover - lifecycle services require ROS2
    GetState = None

try:
    from action_msgs.msg import GoalStatusArray
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from unique_identifier_msgs.msg import UUID as RosUuid
except ImportError:  # pragma: no cover - Nav2 may be unavailable during local editing
    GoalStatusArray = None
    NavigateToPose = None
    ActionClient = None
    RosUuid = None


DEVICE_ID = os.getenv('ROBOT_DEVICE_ID', 'robot-001')
CMD_VEL_TOPIC = os.getenv('CMD_VEL_TOPIC', '/cmd_vel')
LINEAR_SPEED = float(os.getenv('MANUAL_LINEAR_SPEED', '0.1'))
ANGULAR_SPEED = float(os.getenv('MANUAL_ANGULAR_SPEED', '0.5'))

# 2026-09-24 (F-36 / R11): operator-adjustable manual-control speed ceilings.
# Shared contract with the mini program (utils/manualSpeedLimits.ts): linear
# 0.05-1.0 step 0.05 m/s, angular 0.1-2.0 step 0.1 rad/s.  The car treats these
# as absolute ceilings and clamps every manual request; the manual path is the
# only one that reaches set_motion().
MANUAL_SPEED_LIMITS_PATH = os.getenv(
    'MANUAL_SPEED_LIMITS_PATH',
    os.path.expanduser('~/.local/state/stemm-manual-speed-limits.json'),
)
MANUAL_LINEAR_LIMIT_BOUNDS = (0.05, 1.0, 0.05)
MANUAL_ANGULAR_LIMIT_BOUNDS = (0.1, 2.0, 0.1)
# 2026-09-24 (F-36 follow-up): before this endpoint existed the mini program
# always sent its own hardcoded 0.3 m/s / 0.3 rad/s joystick ceiling, so that is
# the effective manual ceiling today and therefore the default here.
MANUAL_LINEAR_CEILING_DEFAULT = float(
    os.getenv('MANUAL_LINEAR_CEILING_DEFAULT', '0.3'))
MANUAL_ANGULAR_CEILING_DEFAULT = float(
    os.getenv('MANUAL_ANGULAR_CEILING_DEFAULT', '0.3'))
PUBLISH_HZ = float(os.getenv('PUBLISH_HZ', '10'))
ROBOT_POSE_TOPIC = os.getenv('ROBOT_POSE_TOPIC', '/amcl_pose')
NAVIGATE_ACTION_NAME = os.getenv('NAVIGATE_ACTION_NAME', 'navigate_to_pose')
NAVIGATION_ACTION_STATUS_TOPIC = os.getenv(
    'NAVIGATION_ACTION_STATUS_TOPIC',
    f'/{NAVIGATE_ACTION_NAME.strip("/")}/_action/status',
).strip()
NAVIGATION_GOAL_RESPONSE_TIMEOUT = max(
    1.0, float(os.getenv('NAVIGATION_GOAL_RESPONSE_TIMEOUT', '5.0'))
)
NAVIGATION_GOAL_ACTIVITY_TIMEOUT = max(
    NAVIGATION_GOAL_RESPONSE_TIMEOUT,
    float(os.getenv('NAVIGATION_GOAL_ACTIVITY_TIMEOUT', '10.0')),
)
NAVIGATION_GOAL_EXECUTION_TIMEOUT = max(
    NAVIGATION_GOAL_ACTIVITY_TIMEOUT,
    float(os.getenv('NAVIGATION_GOAL_EXECUTION_TIMEOUT', '900.0')),
)
NAVIGATION_SYSTEM_STARTUP_TIMEOUT = min(
    120.0,
    max(
        1.0,
        float(os.getenv('NAVIGATION_SYSTEM_STARTUP_TIMEOUT', '120.0')),
    ),
)
NAVIGATION_GUARD_TOPIC = os.getenv(
    'NAVIGATION_GUARD_TOPIC', '/amcl_tf_guard/anomaly'
).strip()
NAVIGATION_GUARD_MODE_TOPIC = os.getenv(
    'NAVIGATION_GUARD_MODE_TOPIC', '/amcl_tf_guard/mode'
).strip()
NAVIGATION_INTERLOCK_BLOCKED_TOPIC = os.getenv(
    'NAVIGATION_INTERLOCK_BLOCKED_TOPIC', '/nav2_guard_interlock/blocked'
).strip()
NAVIGATION_SAFETY_STATUS_STALE_TIMEOUT = max(
    0.5, float(os.getenv('NAVIGATION_SAFETY_STATUS_STALE_TIMEOUT', '2.0'))
)
NAVIGATION_POSE_STALE_TIMEOUT = float(
    os.getenv('NAVIGATION_POSE_STALE_TIMEOUT', '5.0')
)
NAVIGATION_GUARDED_POSE_PROBE_INTERVAL_SEC = max(
    0.05, float(os.getenv('NAVIGATION_GUARDED_POSE_PROBE_INTERVAL_SEC', '0.1'))
)
NAVIGATION_GOAL_HISTORY_LIMIT = max(
    4, int(os.getenv('NAVIGATION_GOAL_HISTORY_LIMIT', '32'))
)
NAVIGATION_GOAL_TOMBSTONE_LIMIT = max(
    NAVIGATION_GOAL_HISTORY_LIMIT,
    int(os.getenv('NAVIGATION_GOAL_TOMBSTONE_LIMIT', '64')),
)
NAVIGATION_REQUIRED_ACTIVE_LIFECYCLE_NODES = tuple(
    item.strip().strip('/')
    for item in os.getenv(
        'NAVIGATION_REQUIRED_ACTIVE_LIFECYCLE_NODES',
        'bt_navigator,controller_server,planner_server',
    ).split(',')
    if item.strip().strip('/')
)
NAVIGATION_LIFECYCLE_PROBE_TIMEOUT_SEC = max(
    0.1, float(os.getenv('NAVIGATION_LIFECYCLE_PROBE_TIMEOUT_SEC', '0.75'))
)
# A new AMCL message is useful diagnostics, but it is not a navigation-ready
# requirement: AMCL is permitted to retain its valid pose while a stationary
# robot does not cross its filter update thresholds.
NAVIGATION_LOCALIZATION_INITIAL_TIMEOUT = max(
    1.0,
    float(os.getenv('NAVIGATION_LOCALIZATION_INITIAL_TIMEOUT', '30.0')),
)
NAVIGATION_LOCALIZATION_SOURCE_LOSS_GRACE = max(
    1.0,
    float(os.getenv('NAVIGATION_LOCALIZATION_SOURCE_LOSS_GRACE', '12.0')),
)
NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT = max(
    0.5,
    float(os.getenv('NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT', '3.0')),
)
NAVIGATION_SOURCE_STAMP_FUTURE_TOLERANCE_SEC = max(
    0.0,
    float(os.getenv('NAVIGATION_SOURCE_STAMP_FUTURE_TOLERANCE_SEC', '0.5')),
)


def navigation_system_starting_allowed(
    active_observed: bool,
    startup_age_sec: float,
    timeout_sec: float,
) -> bool:
    """Keep startup pending only inside the absolute request deadline."""
    return active_observed and startup_age_sec < timeout_sec


def source_stamp_age_sec(now_ros_ns: int, stamp_ns: int) -> float | None:
    """Return source-message age without treating far-future stamps as fresh."""
    if now_ros_ns <= 0 or stamp_ns <= 0:
        return None
    age_sec = (now_ros_ns - stamp_ns) / 1_000_000_000.0
    if age_sec < -NAVIGATION_SOURCE_STAMP_FUTURE_TOLERANCE_SEC:
        return None
    return max(0.0, age_sec)


NAVIGATION_AMCL_NODE_NAMES = frozenset(
    item.strip()
    for item in os.getenv('NAVIGATION_AMCL_NODE_NAMES', 'amcl').split(',')
    if item.strip()
)
NAVIGATION_SCAN_TOPIC = os.getenv('NAVIGATION_SCAN_TOPIC', '/scan')
NAVIGATION_ODOM_TOPIC = os.getenv('NAVIGATION_ODOM_TOPIC', '/odom_combined')
NAVIGATION_TF_TOPIC = os.getenv('NAVIGATION_TF_TOPIC', '/tf')
NAVIGATION_TF_STATIC_TOPIC = os.getenv('NAVIGATION_TF_STATIC_TOPIC', '/tf_static')
NAVIGATION_GLOBAL_FRAME = os.getenv('NAVIGATION_GLOBAL_FRAME', 'map').strip().lstrip('/')
NAVIGATION_ODOM_FRAME = os.getenv('NAVIGATION_ODOM_FRAME', 'odom_combined').strip().lstrip('/')
NAVIGATION_BASE_FRAME = os.getenv('NAVIGATION_BASE_FRAME', 'base_footprint').strip().lstrip('/')
MAPPING_STARTUP_IMU_TOPIC = os.getenv(
    'MAPPING_STARTUP_IMU_TOPIC', '/imu/data_bias_compensated'
).strip()
MAPPING_STARTUP_IMU_FRAME = os.getenv(
    'MAPPING_STARTUP_IMU_FRAME', 'gyro_link'
).strip().lstrip('/')
MAPPING_STARTUP_BASE_SERVICE = os.getenv(
    'MAPPING_STARTUP_BASE_SERVICE', 'wheeltec-robot.service'
).strip()
MAPPING_STARTUP_FRESH_TIMEOUT_SEC = max(
    0.1, float(os.getenv('MAPPING_STARTUP_FRESH_TIMEOUT_SEC', '0.5'))
)
MAPPING_STARTUP_REQUIRED_SAMPLES = max(
    1, int(os.getenv('MAPPING_STARTUP_REQUIRED_SAMPLES', '5'))
)
MAPPING_STARTUP_STABLE_WINDOW_SEC = max(
    0.5, float(os.getenv('MAPPING_STARTUP_STABLE_WINDOW_SEC', '3.0'))
)
MAPPING_STARTUP_READY_GRACE_SEC = max(
    0.5, float(os.getenv('MAPPING_STARTUP_READY_GRACE_SEC', '5.0'))
)
MAPPING_STARTUP_TIMEOUT_SEC = max(
    MAPPING_STARTUP_STABLE_WINDOW_SEC,
    float(os.getenv('MAPPING_STARTUP_TIMEOUT_SEC', '90.0')),
)
MAPPING_STARTUP_SERVICE_PROBE_INTERVAL_SEC = max(
    0.2, float(os.getenv('MAPPING_STARTUP_SERVICE_PROBE_INTERVAL_SEC', '1.0'))
)
MAPPING_STARTUP_TF_PROBE_INTERVAL_SEC = max(
    0.1, float(os.getenv('MAPPING_STARTUP_TF_PROBE_INTERVAL_SEC', '0.5'))
)
MAPPING_STARTUP_TF_STALE_TIMEOUT_SEC = max(
    MAPPING_STARTUP_TF_PROBE_INTERVAL_SEC * 2.0,
    float(os.getenv('MAPPING_STARTUP_TF_STALE_TIMEOUT_SEC', '2.0')),
)
ROS_SPIN_BATCH_SIZE = max(1, int(os.getenv('ROS_SPIN_BATCH_SIZE', '32')))
STATUS_SNAPSHOT_CACHE_TTL_SEC = max(
    0.05, float(os.getenv('STATUS_SNAPSHOT_CACHE_TTL_SEC', '0.5'))
)
NAVIGATION_GRAPH_NODE_NAMES = frozenset(
    item.strip()
    for item in os.getenv(
        'NAVIGATION_GRAPH_NODE_NAMES',
        'bt_navigator,controller_server,planner_server,behavior_server,'
        'smoother_server,waypoint_follower',
    ).split(',')
    if item.strip()
)

# Internal namespaces (bare, no slashes) whose nodes must never be mistaken for
# the user-facing navigation stack.  The map-free body-follow avoidance Nav2
# runs at /nofmap and reuses the stock Nav2 base node names, while rclpy's
# get_node_names() reports bare names without namespaces.
NAVIGATION_INTERNAL_NODE_NAMESPACES = tuple(
    item.strip().strip('/')
    for item in os.getenv(
        'NAVIGATION_INTERNAL_NODE_NAMESPACES', 'nofmap'
    ).split(',')
    if item.strip().strip('/')
)


def _navigation_probe_node_is_internal(name: str, namespace: str) -> bool:
    """True when a graph entry belongs to an internal, non-feature stack."""
    ns = str(namespace or '').strip().strip('/')
    if ns and ns in NAVIGATION_INTERNAL_NODE_NAMESPACES:
        return True
    text = str(name or '').strip()
    for internal in NAVIGATION_INTERNAL_NODE_NAMESPACES:
        if text == internal or text.startswith(internal + '/'):
            return True
        if '/' + internal + '/' in text:
            return True
    return False

# These feature probes observe the same controllers used by voice and by the
# mini-program.  They never issue a follow/recharge command themselves.
BODY_FOLLOW_STATUS_SERVICE = os.getenv(
    'BODY_FOLLOW_STATUS_SERVICE', '/voice_summon/get_body_follow_status'
).strip()
BODY_FOLLOW_STATUS_PROBE_INTERVAL = max(
    0.25, float(os.getenv('BODY_FOLLOW_STATUS_PROBE_INTERVAL', '1.0'))
)
BODY_FOLLOW_STATUS_RESPONSE_TIMEOUT = max(
    BODY_FOLLOW_STATUS_PROBE_INTERVAL,
    float(os.getenv('BODY_FOLLOW_STATUS_RESPONSE_TIMEOUT', '4.0')),
)
BODY_FOLLOW_STATUS_STALE_TIMEOUT = max(
    BODY_FOLLOW_STATUS_RESPONSE_TIMEOUT,
    float(os.getenv('BODY_FOLLOW_STATUS_STALE_TIMEOUT', '8.0')),
)
BODY_FOLLOW_STATUS_FAILURE_THRESHOLD = max(
    1, int(os.getenv('BODY_FOLLOW_STATUS_FAILURE_THRESHOLD', '3'))
)
RECHARGE_STATUS_TOPIC = os.getenv(
    'RECHARGE_STATUS_TOPIC', '/auto_recharge/status'
).strip()
RECHARGE_RESULT_TOPIC = os.getenv(
    'RECHARGE_RESULT_TOPIC', '/auto_recharge/result'
).strip()
RECHARGE_CHARGING_FLAG_TOPIC = os.getenv(
    'RECHARGE_CHARGING_FLAG_TOPIC', '/robot_charging_flag'
).strip()
RECHARGE_CHARGING_CURRENT_TOPIC = os.getenv(
    'RECHARGE_CHARGING_CURRENT_TOPIC', '/robot_charging_current'
).strip()
RECHARGE_MAPPING_MOTION_GATE_TOPIC = os.getenv(
    'RECHARGE_MAPPING_MOTION_GATE_TOPIC', '/stemm_cartographer/motion_blocked'
).strip()
RECHARGE_MANAGER_NODE_NAMES = frozenset(
    item.strip()
    for item in os.getenv('RECHARGE_MANAGER_NODE_NAMES', 'auto_recharger').split(',')
    if item.strip()
)
RECHARGE_SIGNAL_STALE_TIMEOUT = max(
    0.5, float(os.getenv('RECHARGE_SIGNAL_STALE_TIMEOUT', '5.0'))
)
RECHARGE_RESULT_VISIBLE_TIMEOUT = max(
    1.0, float(os.getenv('RECHARGE_RESULT_VISIBLE_TIMEOUT', '20.0'))
)
# Cartographer has two complementary state authorities.  The voice bridge
# publishes a retained, high-frequency transition state while the session file
# is atomically written by Cartographer and proves terminal service recovery.
MAPPING_VOICE_STATE_TOPIC = os.getenv(
    'MAPPING_VOICE_STATE_TOPIC', '/stemm_cartographer/voice_state'
).strip()
MAPPING_SESSION_STATUS_PATH = os.getenv(
    'MAPPING_SESSION_STATUS_PATH',
    os.path.expanduser('~/.local/state/stemm-cartographer-session.json'),
).strip()
MAPPING_VOICE_STATE_STALE_TIMEOUT = max(
    1.0, float(os.getenv('MAPPING_VOICE_STATE_STALE_TIMEOUT', '3.0'))
)
MAPPING_ACTIVE_SESSION_STALE_TIMEOUT = max(
    MAPPING_VOICE_STATE_STALE_TIMEOUT,
    float(os.getenv('MAPPING_ACTIVE_SESSION_STALE_TIMEOUT', '15.0')),
)
# This topic is deliberately published only by command_recognition for the
# normal voice I/J/K targets.  It must never infer state from generic Nav2
# actions because Cartographer exploration, return-home and voice summon also
# use NavigateToPose.
VOICE_TARGET_NAVIGATION_STATUS_TOPIC = os.getenv(
    'VOICE_TARGET_NAVIGATION_STATUS_TOPIC', '/stemm_voice_navigation/status'
).strip()
VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH = os.getenv(
    'VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH',
    os.path.expanduser('~/.local/state/stemm-voice-target-navigation-session.json'),
).strip()
VOICE_TARGET_NAVIGATION_STATUS_STALE_TIMEOUT = max(
    1.0, float(os.getenv('VOICE_TARGET_NAVIGATION_STATUS_STALE_TIMEOUT', '4.0'))
)
# Keep a safety stop, but make it long enough that normal press-and-hold use
# is not interrupted by a short timeout or minor network jitter.
AUTO_STOP_TIMEOUT = float(os.getenv('AUTO_STOP_TIMEOUT', '5.0'))

app = FastAPI(title='robot-agent', version='1.1.0')


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


class ManualSpeedLimitsPayload(BaseModel):
    deviceId: str = DEVICE_ID
    maxLinear: Optional[float] = None
    maxAngular: Optional[float] = None


class MappingRecoverPayload(BaseModel):
    deviceId: str = DEVICE_ID


_manual_limits_lock = threading.Lock()
_manual_limits_state = {
    'maxLinear': 0.0,
    'maxAngular': 0.0,
    'source': 'default',
    'updatedAtEpochMs': 0,
}


def _round_manual_limit_to_step(value: float, step: float) -> float:
    """Snap to the shared slider step so 0.30000000004 never leaves the car."""
    text = f'{step}'
    decimals = len(text.split('.')[1]) if '.' in text else 0
    return round(round(value / step) * step, decimals)


def _normalize_manual_limit(value, bounds) -> tuple[float, bool, bool]:
    """Clamp a requested ceiling into the shared bounds.

    Returns (value, pinned_to_bound, usable).  Non-numeric, non-finite and
    non-positive input is reported as unusable so the caller can refuse instead
    of silently substituting a number the operator never chose.
    """
    minimum, maximum, step = bounds
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0.0, False, False
    if not isfinite(numeric) or numeric <= 0:
        return 0.0, False, False
    stepped = _round_manual_limit_to_step(numeric, step)
    pinned_value = min(maximum, max(minimum, stepped))
    return pinned_value, pinned_value != stepped, True


def _load_manual_speed_limits() -> None:
    """Restore the persisted ceilings; anything unreadable keeps the defaults."""
    default_linear, _, _ = _normalize_manual_limit(
        MANUAL_LINEAR_CEILING_DEFAULT, MANUAL_LINEAR_LIMIT_BOUNDS)
    default_angular, _, _ = _normalize_manual_limit(
        MANUAL_ANGULAR_CEILING_DEFAULT, MANUAL_ANGULAR_LIMIT_BOUNDS)
    state = {
        'maxLinear': default_linear,
        'maxAngular': default_angular,
        'source': 'default',
        'updatedAtEpochMs': 0,
    }
    try:
        with open(MANUAL_SPEED_LIMITS_PATH, 'r', encoding='utf-8') as handle:
            payload = json.load(handle)
    except FileNotFoundError:
        payload = None
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        payload = None
    if isinstance(payload, dict):
        linear, _, linear_ok = _normalize_manual_limit(
            payload.get('maxLinear'), MANUAL_LINEAR_LIMIT_BOUNDS)
        angular, _, angular_ok = _normalize_manual_limit(
            payload.get('maxAngular'), MANUAL_ANGULAR_LIMIT_BOUNDS)
        if linear_ok and angular_ok:
            try:
                updated_at = int(payload.get('updatedAtEpochMs') or 0)
            except (TypeError, ValueError):
                updated_at = 0
            state.update({
                'maxLinear': linear,
                'maxAngular': angular,
                'source': 'persisted',
                'updatedAtEpochMs': updated_at,
            })
    with _manual_limits_lock:
        _manual_limits_state.update(state)


def _persist_manual_speed_limits() -> str:
    """Atomically store the ceilings.  Returns '' on success, else a reason."""
    with _manual_limits_lock:
        payload = {
            'schemaVersion': 1,
            'maxLinear': float(_manual_limits_state['maxLinear']),
            'maxAngular': float(_manual_limits_state['maxAngular']),
            'updatedAtEpochMs': int(_manual_limits_state['updatedAtEpochMs']),
        }
    directory = os.path.dirname(MANUAL_SPEED_LIMITS_PATH) or '.'
    temporary_path = (
        f'{MANUAL_SPEED_LIMITS_PATH}.tmp.{os.getpid()}.{threading.get_ident()}'
    )
    try:
        os.makedirs(directory, exist_ok=True)
        with open(temporary_path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, ensure_ascii=False, separators=(',', ':'))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, MANUAL_SPEED_LIMITS_PATH)
        return ''
    except OSError as exc:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        return f'Manual speed limits cannot be written: {exc}'


def current_manual_speed_limits() -> tuple[float, float]:
    with _manual_limits_lock:
        return (
            float(_manual_limits_state['maxLinear']),
            float(_manual_limits_state['maxAngular']),
        )


def clamp_manual_motion(linear: float, angular: float) -> tuple[float, float, bool]:
    """Absolute car-side ceiling for manual teleop (F-36 / R11).

    Magnitudes are clamped and the sign is preserved, so reversing is unaffected.
    """
    max_linear, max_angular = current_manual_speed_limits()
    clamped_linear = max(-max_linear, min(max_linear, float(linear)))
    clamped_angular = max(-max_angular, min(max_angular, float(angular)))
    return (
        clamped_linear,
        clamped_angular,
        clamped_linear != linear or clamped_angular != angular,
    )


def _manual_speed_limits_payload(**extra) -> dict:
    max_linear, max_angular = current_manual_speed_limits()
    with _manual_limits_lock:
        updated_at = int(_manual_limits_state['updatedAtEpochMs'])
        source = str(_manual_limits_state['source'])
    payload = {
        'schemaVersion': 1,
        'ok': True,
        'deviceId': DEVICE_ID,
        'supported': True,
        'maxLinear': max_linear,
        'maxAngular': max_angular,
        'hardMaxLinear': MANUAL_LINEAR_LIMIT_BOUNDS[1],
        'hardMaxAngular': MANUAL_ANGULAR_LIMIT_BOUNDS[1],
        'minLinear': MANUAL_LINEAR_LIMIT_BOUNDS[0],
        'minAngular': MANUAL_ANGULAR_LIMIT_BOUNDS[0],
        'limits': {
            'supported': True,
            'maxLinear': max_linear,
            'maxAngular': max_angular,
        },
        'source': source,
        'updatedAtEpochMs': updated_at,
    }
    payload.update(extra)
    return payload


# 2026-09-24 (F-34): the functional-service allowlist the session runner stops
# for mapping.  Recovery only ever verifies it, or restarts it through the same
# NOPASSWD helper the runner uses.
FUNCTIONAL_SERVICE_UNITS = (
    'bodyfollow-nav2.service',
    'wheeltec-recharge-manager.service',
    'low-battery-auto-recharge.service',
)
CARTOGRAPHER_SERVICE_GUARD = '/usr/local/sbin/stemm-cartographer-service-guard'

# Diagnostic classification for sessions written before the runner carried its
# own reasonCode.  Backend/debug view only - never user-facing copy.
MAPPING_REASON_PATTERNS = (
    ('NAV2_LIFECYCLE_TIMEOUT', (
        'did not become active', 'lifecycle', 'change_state', 'nav2')),
    ('RECHARGE_STOP_TIMEOUT', (
        'service guard stop', 'service-guard', 'auto_recharge/stop',
        'recharge stop helper')),
    ('VEHICLE_NOT_READY', ('readiness', 'initializ', '初始化')),
    ('NAVIGATION_RELEASE_FAILED', (
        'navigation release', 'map-mode navigation')),
    ('START_CONFIRM_TIMEOUT', ('start confirmation', 'start confirm')),
    ('SESSION_CANCELED', ('cancel',)),
    ('RECOVERY_REQUIRED', ('manual recovery',)),
)


def _derive_mapping_reason_code(*, state_name, last_error, session,
                                session_terminal, mapping_done,
                                final_map_saved) -> tuple[str, str]:
    """Structured failure cause for diagnostics (F-34).

    The session file is authoritative when it carries a code; the pattern table
    only classifies older sessions.  The mini program must keep this out of
    user-visible text.
    """
    text = str(last_error or '').strip()
    if isinstance(session, dict):
        carried = str(session.get('reasonCode') or '').strip()
        if carried:
            return carried, str(session.get('reasonText') or text).strip()
    if state_name != 'failed':
        if session_terminal and mapping_done and final_map_saved:
            return 'MAPPING_SUCCEEDED', text
        return '', text
    lowered = text.lower()
    for code, needles in MAPPING_REASON_PATTERNS:
        if any(needle in lowered for needle in needles):
            return code, text
    return 'MAPPING_FAILED', text


def _read_mapping_session_file() -> tuple[dict | None, str]:
    """Read the runner's session state without touching a publisher instance."""
    if not MAPPING_SESSION_STATUS_PATH:
        return None, 'mapping session status path is not configured'
    try:
        with open(MAPPING_SESSION_STATUS_PATH, 'r', encoding='utf-8') as handle:
            payload = json.load(handle)
    except FileNotFoundError:
        return None, ''
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f'mapping session status cannot be read: {exc}'
    if not isinstance(payload, dict):
        return None, 'mapping session status is invalid'
    return payload, ''


def _functional_service_states() -> dict:
    states = {}
    for unit in FUNCTIONAL_SERVICE_UNITS:
        try:
            probe = subprocess.run(
                ['systemctl', 'is-active', unit],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=10.0,
            )
            states[unit] = (probe.stdout or '').strip() or 'unknown'
        except (OSError, subprocess.SubprocessError):
            states[unit] = 'unavailable'
    return states


def _recover_mapping_service() -> dict:
    """Free the vehicle after a failed mapping start (F-34 self-recovery).

    Never touches a live session: only a terminal, non-active state is cleared,
    and only after the functional-service allowlist is verified active.
    """
    actions = []
    session, session_error = _read_mapping_session_file()
    if session:
        still_running = (
            (bool(session.get('sessionActive')) and not bool(session.get('terminal')))
            or (str(session.get('phase') or '') == 'starting'
                and not bool(session.get('terminal')))
        )
        if still_running:
            return {
                'ok': True, 'accepted': False, 'recovering': False,
                'recovered': False, 'supported': True,
                'reasonCode': 'FEATURE_BUSY',
                'reasonText': 'A Cartographer mapping session is still active.',
                'lastError': str(session.get('lastError') or ''),
                'actions': [],
            }

    states = _functional_service_states()
    unavailable = {u: s for u, s in states.items() if s != 'active'}
    if unavailable:
        actions.append({'action': 'restart_functional_services',
                        'before': states})
        restart_rc = None
        try:
            completed = subprocess.run(
                ['sudo', '-n', CARTOGRAPHER_SERVICE_GUARD, 'restart'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=90.0,
            )
            restart_rc = completed.returncode
        except (OSError, subprocess.SubprocessError) as exc:
            actions.append({'action': 'restart_error', 'message': str(exc)})
        states = _functional_service_states()
        actions.append({'action': 'restart_result', 'returncode': restart_rc,
                        'after': states})
        still_unavailable = {u: s for u, s in states.items() if s != 'active'}
        if still_unavailable:
            return {
                'ok': True, 'accepted': True, 'recovering': False,
                'recovered': False, 'supported': True,
                'reasonCode': 'FUNCTIONAL_SERVICES_UNAVAILABLE',
                'reasonText': 'Functional services are not all active: '
                              + ', '.join(f'{u}={s}' for u, s in
                                          sorted(still_unavailable.items())),
                'lastError': '', 'actions': actions,
            }

    if session is None:
        return {
            'ok': True, 'accepted': True, 'recovering': False,
            'recovered': True, 'supported': True,
            'reasonCode': 'NOTHING_TO_RECOVER',
            'reasonText': session_error or 'No stale mapping session state.',
            'lastError': '', 'actions': actions,
        }

    archive_path = (
        f'{MAPPING_SESSION_STATUS_PATH}.recovered-'
        f'{time.strftime("%Y%m%d-%H%M%S")}'
    )
    try:
        os.replace(MAPPING_SESSION_STATUS_PATH, archive_path)
    except OSError as exc:
        return {
            'ok': True, 'accepted': True, 'recovering': False,
            'recovered': False, 'supported': True,
            'reasonCode': 'STATE_CLEAR_FAILED',
            'reasonText': f'Stale mapping state cannot be cleared: {exc}',
            'lastError': str(session.get('lastError') or ''),
            'actions': actions,
        }
    actions.append({'action': 'archive_stale_session_state',
                    'path': archive_path})
    if publisher is not None:
        try:
            publisher.feature_status_cache.invalidate()
        except Exception:  # noqa: BLE001 - cache invalidation is best effort
            pass
    return {
        'ok': True, 'accepted': True, 'recovering': False,
        'recovered': True, 'supported': True,
        'reasonCode': 'RECOVERED',
        'reasonText': 'Stale mapping failure state cleared; mapping can be '
                      'retried.',
        'lastError': str(session.get('lastError') or ''),
        'clearedTaskId': str(session.get('taskId') or ''),
        'actions': actions,
    }


_load_manual_speed_limits()


def _quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    half_yaw = yaw * 0.5
    return sin(half_yaw), cos(half_yaw)


def _goal_uuid_hex(goal_id) -> str:
    try:
        raw_uuid = bytes(goal_id.uuid)
    except (AttributeError, TypeError, ValueError):
        return ''
    return raw_uuid.hex() if len(raw_uuid) == 16 else ''


def _navigation_terminal_state(status_code: int) -> tuple[str, bool, str] | None:
    if status_code == 4:
        return 'succeeded', True, 'Navigation succeeded'
    if status_code == 5:
        return 'canceled', False, 'Navigation canceled'
    if status_code == 6:
        return 'failed', False, 'Navigation failed with status 6'
    return None


def _yaw_from_quaternion(orientation) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return atan2(siny_cosp, cosy_cosp)


def _utc_now() -> str:
    return datetime.utcnow().isoformat() + 'Z'


class TimedStatusSnapshotCache:
    """Bound expensive status builders and collapse concurrent refreshes.

    The cached sample is never handed to callers directly.  Every response is
    copied and receives a fresh top-level observation timestamp; nested source
    timestamps continue to describe when their underlying ROS data was seen.
    """

    def __init__(self, ttl_sec: float) -> None:
        self.ttl_sec = max(0.0, float(ttl_sec))
        self._condition = threading.Condition()
        self._snapshot = None
        self._expires_monotonic = 0.0
        self._refreshing = False

    @staticmethod
    def _response_copy(snapshot):
        response = copy.deepcopy(snapshot)
        if isinstance(response, dict):
            response['observedAtEpochMs'] = int(time.time() * 1000)
            if 'observedAt' in response:
                response['observedAt'] = _utc_now()
        return response

    def get(self, builder):
        while True:
            now_monotonic = time.monotonic()
            with self._condition:
                if (
                    self._snapshot is not None
                    and now_monotonic < self._expires_monotonic
                ):
                    return self._response_copy(self._snapshot)
                if not self._refreshing:
                    self._refreshing = True
                    break
                # One caller owns the ROS graph query.  Other HTTP workers wait
                # for that sample instead of issuing the same discovery calls.
                self._condition.wait(timeout=max(0.05, self.ttl_sec))

        try:
            snapshot = builder()
        except Exception:
            with self._condition:
                self._refreshing = False
                self._condition.notify_all()
            raise

        with self._condition:
            self._snapshot = copy.deepcopy(snapshot)
            self._expires_monotonic = time.monotonic() + self.ttl_sec
            self._refreshing = False
            self._condition.notify_all()
            return self._response_copy(self._snapshot)

    def invalidate(self) -> None:
        with self._condition:
            self._snapshot = None
            self._expires_monotonic = 0.0
            self._condition.notify_all()


def _duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0


def _linux_boot_epoch_ms() -> int:
    """Return the current Linux boot epoch, or zero when it cannot be read.

    The Cartographer session ledger persists across a reboot.  Its wall-clock
    update time is therefore the only non-mutating way for this observer to
    distinguish a previous-boot orphan from an in-process stale update.
    """
    try:
        with open('/proc/stat', 'r', encoding='utf-8') as handle:
            for line in handle:
                if line.startswith('btime '):
                    boot_epoch_seconds = int(line.split(maxsplit=1)[1])
                    return max(0, boot_epoch_seconds * 1000)
    except (OSError, ValueError):
        pass
    return 0


def _normalize_frame_id(frame_id: object) -> str:
    return str(frame_id or '').strip().lstrip('/')


class NavigationGoalLookupError(Exception):
    def __init__(self, status_code: int, goal_id: str, message: str) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.goal_id = goal_id


class VelocityPublisher:
    def __init__(self) -> None:
        if rclpy is None or Twist is None:
            raise RuntimeError('ROS2 Python packages are not available. Please source ROS2 first.')

        rclpy.init(args=None)
        self.node = Node('robot_manual_control_agent')
        # rclpy Humble registers a request's sequence number *after* writing the
        # request.  Serializing ActionClient request creation with spin_once()
        # prevents a fast response from being consumed before that registration.
        self.ros_action_lock = threading.RLock()
        self.publisher = self.node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.pose_subscription = None
        self.scan_subscription = None
        self.odom_subscription = None
        self.tf_subscription = None
        self.tf_static_subscription = None
        self.tf_buffer = None
        self.tf_listener = None
        self.imu_subscription = None
        self.navigation_client = None
        self.navigation_action_status_subscription = None
        self.navigation_lifecycle_clients = {}
        if PoseWithCovarianceStamped is not None:
            # AMCL offers /amcl_pose as RELIABLE + TRANSIENT_LOCAL. Matching
            # durability lets a restarted status agent immediately receive the
            # latest valid pose instead of waiting for the next motion-driven
            # AMCL update.
            pose_qos = 10
            if (
                QoSProfile is not None
                and ReliabilityPolicy is not None
                and DurabilityPolicy is not None
            ):
                pose_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            self.pose_subscription = self.node.create_subscription(
                PoseWithCovarianceStamped,
                ROBOT_POSE_TOPIC,
                self._on_pose,
                pose_qos,
            )
        localization_data_qos = (
            qos_profile_sensor_data if qos_profile_sensor_data is not None else 10
        )
        if LaserScan is not None:
            self.scan_subscription = self.node.create_subscription(
                LaserScan,
                NAVIGATION_SCAN_TOPIC,
                self._on_localization_scan,
                localization_data_qos,
            )
        if Odometry is not None:
            self.odom_subscription = self.node.create_subscription(
                Odometry,
                NAVIGATION_ODOM_TOPIC,
                self._on_localization_odom,
                localization_data_qos,
            )
        if Imu is not None and MAPPING_STARTUP_IMU_TOPIC:
            self.imu_subscription = self.node.create_subscription(
                Imu,
                MAPPING_STARTUP_IMU_TOPIC,
                self._on_mapping_startup_imu,
                localization_data_qos,
            )
        if TFMessage is not None:
            self.tf_subscription = self.node.create_subscription(
                TFMessage,
                NAVIGATION_TF_TOPIC,
                self._on_localization_tf,
                localization_data_qos,
            )
            tf_static_qos = 10
            if (
                QoSProfile is not None
                and ReliabilityPolicy is not None
                and DurabilityPolicy is not None
            ):
                tf_static_qos = QoSProfile(
                    # Multiple independent transient-local static publishers are
                    # present on this vehicle.  Keep enough retained samples for
                    # a late-starting agent instead of retaining only the last one.
                    depth=100,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            self.tf_static_subscription = self.node.create_subscription(
                TFMessage,
                NAVIGATION_TF_STATIC_TOPIC,
                self._on_localization_tf,
                tf_static_qos,
            )
        if (
            Tf2Buffer is not None
            and Tf2TransformListener is not None
            and RosDuration is not None
        ):
            # TransformListener owns a standards-compatible /tf + /tf_static
            # subscription and lets readiness verify the complete frame graph.
            # The direct subscriptions above remain for navigation diagnostics.
            self.tf_buffer = Tf2Buffer(cache_time=RosDuration(seconds=10.0))
            self.tf_listener = Tf2TransformListener(
                self.tf_buffer, self.node, spin_thread=False
            )
        if ActionClient is not None and NavigateToPose is not None:
            self.navigation_client = ActionClient(self.node, NavigateToPose, NAVIGATE_ACTION_NAME)
        if GetState is not None:
            self.navigation_lifecycle_clients = {
                node_name: self.node.create_client(
                    GetState, f'/{node_name}/get_state'
                )
                for node_name in NAVIGATION_REQUIRED_ACTIVE_LIFECYCLE_NODES
            }
        if GoalStatusArray is not None and NAVIGATION_ACTION_STATUS_TOPIC:
            action_status_qos = 10
            if (
                QoSProfile is not None
                and ReliabilityPolicy is not None
                and DurabilityPolicy is not None
            ):
                action_status_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            self.navigation_action_status_subscription = self.node.create_subscription(
                GoalStatusArray,
                NAVIGATION_ACTION_STATUS_TOPIC,
                self._on_navigation_action_status,
                action_status_qos,
            )
        # A persistent rclpy client avoids paying ROS CLI/DDS discovery cost on
        # every feature refresh.  The bridge only reads this agent over loopback,
        # so both voice- and mini-program-originated controller changes converge
        # on one cache without issuing any movement command.
        self.body_follow_status_client = None
        if Trigger is not None and BODY_FOLLOW_STATUS_SERVICE:
            self.body_follow_status_client = self.node.create_client(
                Trigger, BODY_FOLLOW_STATUS_SERVICE
            )

        recharge_status_qos = 10
        if (
            QoSProfile is not None
            and ReliabilityPolicy is not None
            and DurabilityPolicy is not None
        ):
            recharge_status_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        self.recharge_status_subscription = None
        self.recharge_result_subscription = None
        self.recharge_charging_flag_subscription = None
        self.recharge_charging_current_subscription = None
        self.recharge_mapping_gate_subscription = None
        self.mapping_voice_state_subscription = None
        self.voice_target_navigation_status_subscription = None
        self.navigation_guard_subscription = None
        self.navigation_guard_mode_subscription = None
        self.navigation_interlock_subscription = None
        if Bool is not None and NAVIGATION_GUARD_TOPIC:
            self.navigation_guard_subscription = self.node.create_subscription(
                Bool,
                NAVIGATION_GUARD_TOPIC,
                self._on_navigation_guard,
                recharge_status_qos,
            )
        if Bool is not None and NAVIGATION_INTERLOCK_BLOCKED_TOPIC:
            self.navigation_interlock_subscription = self.node.create_subscription(
                Bool,
                NAVIGATION_INTERLOCK_BLOCKED_TOPIC,
                self._on_navigation_interlock,
                recharge_status_qos,
            )
        if String is not None and NAVIGATION_GUARD_MODE_TOPIC:
            self.navigation_guard_mode_subscription = self.node.create_subscription(
                String,
                NAVIGATION_GUARD_MODE_TOPIC,
                self._on_navigation_guard_mode,
                recharge_status_qos,
            )
        if String is not None and RECHARGE_STATUS_TOPIC:
            self.recharge_status_subscription = self.node.create_subscription(
                String,
                RECHARGE_STATUS_TOPIC,
                self._on_recharge_status,
                recharge_status_qos,
            )
        if String is not None and RECHARGE_RESULT_TOPIC:
            self.recharge_result_subscription = self.node.create_subscription(
                String,
                RECHARGE_RESULT_TOPIC,
                self._on_recharge_result,
                10,
            )
        if Bool is not None and RECHARGE_CHARGING_FLAG_TOPIC:
            self.recharge_charging_flag_subscription = self.node.create_subscription(
                Bool,
                RECHARGE_CHARGING_FLAG_TOPIC,
                self._on_recharge_charging_flag,
                10,
            )
        if Float32 is not None and RECHARGE_CHARGING_CURRENT_TOPIC:
            self.recharge_charging_current_subscription = self.node.create_subscription(
                Float32,
                RECHARGE_CHARGING_CURRENT_TOPIC,
                self._on_recharge_charging_current,
                10,
            )
        if Bool is not None and RECHARGE_MAPPING_MOTION_GATE_TOPIC:
            self.recharge_mapping_gate_subscription = self.node.create_subscription(
                Bool,
                RECHARGE_MAPPING_MOTION_GATE_TOPIC,
                self._on_recharge_mapping_gate,
                recharge_status_qos,
            )
        if String is not None and MAPPING_VOICE_STATE_TOPIC:
            self.mapping_voice_state_subscription = self.node.create_subscription(
                String,
                MAPPING_VOICE_STATE_TOPIC,
                self._on_mapping_voice_state,
                recharge_status_qos,
            )
        if String is not None and VOICE_TARGET_NAVIGATION_STATUS_TOPIC:
            self.voice_target_navigation_status_subscription = (
                self.node.create_subscription(
                    String,
                    VOICE_TARGET_NAVIGATION_STATUS_TOPIC,
                    self._on_voice_target_navigation_status,
                    recharge_status_qos,
                )
            )
        self.linear = 0.0
        self.angular = 0.0
        self.latest_pose = None
        self.latest_pose_received_monotonic = 0.0
        self.latest_guarded_pose = None
        self.latest_guarded_pose_observed_monotonic = 0.0
        self.latest_guarded_pose_stamp_ns = 0
        self.latest_guarded_pose_probe_monotonic = 0.0
        self.latest_guarded_pose_probe_error = ''
        self.latest_scan_received_monotonic = 0.0
        self.latest_scan_stamp_ns = 0
        self.latest_odom_received_monotonic = 0.0
        self.latest_imu_received_monotonic = 0.0
        self.latest_map_to_base_received_monotonic = 0.0
        self.latest_map_to_odom_received_monotonic = 0.0
        self.latest_odom_to_base_received_monotonic = 0.0
        self.latest_odom_stamp_ns = 0
        self.latest_odom_to_base_stamp_ns = 0
        self.odom_advancing_samples = 0
        self.odom_to_base_advancing_samples = 0
        self.base_to_imu_transform_last_observed_monotonic = 0.0
        self.base_to_imu_transform_last_probe_monotonic = 0.0
        self.base_to_imu_transform_probe_error = ''
        self.mapping_startup_started_monotonic = time.monotonic()
        self.mapping_startup_started_epoch_ms = int(time.time() * 1000)
        self.mapping_startup_stable_since_monotonic = 0.0
        self.mapping_startup_ready_since_monotonic = 0.0
        self.mapping_startup_state = 'initializing'
        self.mapping_startup_base_service_active = False
        self.mapping_startup_service_probe_error = ''
        self.mapping_startup_service_probe_monotonic = 0.0
        self.navigation_state = {
            'schemaVersion': 1,
            'ok': True,
            'deviceId': DEVICE_ID,
            'status': 'idle',
            'goalId': '',
            'accepted': False,
            'goalResponsePending': False,
            'goal': None,
            'feedback': None,
            'result': None,
            'message': 'No active navigation goal',
            'updatedAt': _utc_now(),
        }
        self.navigation_goal_history = OrderedDict()
        self.navigation_goal_tombstones = OrderedDict()
        self.navigation_lock = threading.Lock()
        self.navigation_command_lock = threading.Lock()
        self.navigation_system_lock = threading.Lock()
        self.navigation_system_observed_since = None
        self.localization_health_missing_since = None
        self.navigation_guard_anomaly = None
        self.navigation_guard_received_monotonic = 0.0
        self.navigation_guard_mode = ''
        self.navigation_guard_mode_received_monotonic = 0.0
        self.navigation_interlock_blocked = None
        self.navigation_interlock_received_monotonic = 0.0
        self.body_follow_status = None
        self.body_follow_status_error = 'Body-follow status has not been observed yet.'
        self.body_follow_status_received_monotonic = 0.0
        self.body_follow_status_error_monotonic = 0.0
        self.body_follow_status_consecutive_failures = 0
        self.body_follow_status_request_future = None
        self.body_follow_status_request_started_monotonic = 0.0
        self.body_follow_status_last_requested_monotonic = 0.0
        self.body_follow_status_request_generation = 0
        self.body_follow_status_active_generation = 0
        self.recharge_status = None
        self.recharge_status_received_monotonic = 0.0
        self.recharge_result = ''
        self.recharge_result_received_monotonic = 0.0
        self.recharge_charging_flag = None
        self.recharge_charging_flag_received_monotonic = 0.0
        self.recharge_charging_current = None
        self.recharge_charging_current_received_monotonic = 0.0
        self.recharge_mapping_motion_blocked = False
        self.recharge_mapping_gate_received = False
        self.recharge_mapping_gate_received_monotonic = 0.0
        self.mapping_voice_state = None
        self.mapping_voice_state_received_monotonic = 0.0
        self.mapping_voice_state_error = (
            'Cartographer voice state has not been observed yet.'
        )
        self.mapping_voice_state_error_monotonic = 0.0
        self.voice_target_navigation_status = None
        self.voice_target_navigation_status_received_monotonic = 0.0
        self.voice_target_navigation_status_error = (
            'Voice target-navigation status has not been observed yet.'
        )
        self.voice_target_navigation_status_error_monotonic = 0.0
        self._active_goal_handle = None
        self._navigation_goal_generation = 0
        self._active_goal_uuid = ''
        self._active_goal_started_monotonic = 0.0
        self._active_goal_observed_monotonic = 0.0
        self._active_goal_response_future = None
        self.last_command_time = 0.0
        self.manual_active = False
        self.publish_stop_once = False
        self.lock = threading.Lock()
        self.navigation_system_status_cache = TimedStatusSnapshotCache(
            STATUS_SNAPSHOT_CACHE_TTL_SEC
        )
        self.feature_status_cache = TimedStatusSnapshotCache(
            STATUS_SNAPSHOT_CACHE_TTL_SEC
        )
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

    def _on_navigation_guard(self, message) -> None:
        changed = False
        with self.lock:
            next_anomaly = bool(getattr(message, 'data', True))
            changed = next_anomaly != self.navigation_guard_anomaly
            self.navigation_guard_anomaly = next_anomaly
            self.navigation_guard_received_monotonic = time.monotonic()
        if changed:
            self.navigation_system_status_cache.invalidate()

    def _on_navigation_guard_mode(self, message) -> None:
        mode = str(getattr(message, 'data', '') or '').strip().upper()
        if not mode:
            return
        with self.lock:
            changed = mode != self.navigation_guard_mode
            self.navigation_guard_mode = mode
            self.navigation_guard_mode_received_monotonic = time.monotonic()
        if changed:
            self.navigation_system_status_cache.invalidate()

    def _on_navigation_interlock(self, message) -> None:
        changed = False
        with self.lock:
            next_blocked = bool(getattr(message, 'data', True))
            changed = next_blocked != self.navigation_interlock_blocked
            self.navigation_interlock_blocked = next_blocked
            self.navigation_interlock_received_monotonic = time.monotonic()
        if changed:
            self.navigation_system_status_cache.invalidate()

    def _on_pose(self, message) -> None:
        pose = message.pose.pose
        coordinates = (
            pose.position.x,
            pose.position.y,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(isfinite(value) for value in coordinates):
            return
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
            self.latest_pose_received_monotonic = time.monotonic()

    def _on_localization_scan(self, message) -> None:
        stamp_ns = self._message_stamp_ns(message)
        with self.lock:
            self.latest_scan_received_monotonic = time.monotonic()
            self.latest_scan_stamp_ns = stamp_ns

    @staticmethod
    def _message_stamp_ns(message) -> int:
        try:
            stamp = message.header.stamp
            seconds = int(stamp.sec)
            nanoseconds = int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return 0
        if seconds < 0 or nanoseconds < 0:
            return 0
        return seconds * 1_000_000_000 + nanoseconds

    def _on_localization_odom(self, message) -> None:
        stamp_ns = self._message_stamp_ns(message)
        with self.lock:
            self.latest_odom_received_monotonic = time.monotonic()
            if stamp_ns > self.latest_odom_stamp_ns:
                self.latest_odom_stamp_ns = stamp_ns
                self.odom_advancing_samples = min(
                    MAPPING_STARTUP_REQUIRED_SAMPLES,
                    self.odom_advancing_samples + 1,
                )
            elif stamp_ns and stamp_ns < self.latest_odom_stamp_ns:
                # A clock/domain reset must earn a new stable window instead of
                # inheriting samples from the pre-reset odometry stream.
                self.latest_odom_stamp_ns = stamp_ns
                self.odom_advancing_samples = 1

    def _on_mapping_startup_imu(self, _message) -> None:
        with self.lock:
            self.latest_imu_received_monotonic = time.monotonic()

    def _on_localization_tf(self, message) -> None:
        now_monotonic = time.monotonic()
        with self.lock:
            for transform in message.transforms:
                parent = _normalize_frame_id(transform.header.frame_id)
                child = _normalize_frame_id(transform.child_frame_id)
                if parent == NAVIGATION_GLOBAL_FRAME and child == NAVIGATION_BASE_FRAME:
                    self.latest_map_to_base_received_monotonic = now_monotonic
                elif parent == NAVIGATION_GLOBAL_FRAME and child == NAVIGATION_ODOM_FRAME:
                    self.latest_map_to_odom_received_monotonic = now_monotonic
                elif parent == NAVIGATION_ODOM_FRAME and child == NAVIGATION_BASE_FRAME:
                    self.latest_odom_to_base_received_monotonic = now_monotonic
                    stamp_ns = self._message_stamp_ns(transform)
                    if stamp_ns > self.latest_odom_to_base_stamp_ns:
                        self.latest_odom_to_base_stamp_ns = stamp_ns
                        self.odom_to_base_advancing_samples = min(
                            MAPPING_STARTUP_REQUIRED_SAMPLES,
                            self.odom_to_base_advancing_samples + 1,
                        )
                    elif stamp_ns and stamp_ns < self.latest_odom_to_base_stamp_ns:
                        self.latest_odom_to_base_stamp_ns = stamp_ns
                        self.odom_to_base_advancing_samples = 1
                elif (
                    parent == NAVIGATION_BASE_FRAME
                    and child == MAPPING_STARTUP_IMU_FRAME
                ):
                    self.base_to_imu_transform_last_observed_monotonic = (
                        now_monotonic
                    )
                    self.base_to_imu_transform_probe_error = ''

    def _refresh_base_to_imu_transform(self) -> None:
        """Periodically verify base->IMU through tf2's assembled frame graph."""
        now_monotonic = time.monotonic()
        with self.lock:
            if (
                self.base_to_imu_transform_last_probe_monotonic > 0
                and now_monotonic
                - self.base_to_imu_transform_last_probe_monotonic
                < MAPPING_STARTUP_TF_PROBE_INTERVAL_SEC
            ):
                return
            self.base_to_imu_transform_last_probe_monotonic = now_monotonic

        if self.tf_buffer is None or RosTime is None:
            with self.lock:
                self.base_to_imu_transform_probe_error = (
                    'tf2 buffer is not available.'
                )
            return
        try:
            self.tf_buffer.lookup_transform(
                NAVIGATION_BASE_FRAME,
                MAPPING_STARTUP_IMU_FRAME,
                RosTime(),
            )
        except Exception as exc:  # noqa: BLE001 - readiness probe is fail-closed
            with self.lock:
                self.base_to_imu_transform_probe_error = str(exc)
            return
        with self.lock:
            self.base_to_imu_transform_last_observed_monotonic = now_monotonic
            self.base_to_imu_transform_probe_error = ''

    def _refresh_guarded_robot_pose(self) -> None:
        """Cache only the pose assembled through the guard-owned map TF chain.

        Raw AMCL is retained for diagnostics but never drives the mini-program's
        primary robot marker.  When the guard or interlock is unhealthy this
        method deliberately stops updating, leaving the last trusted pose
        frozen for HOLD presentation.
        """
        now_monotonic = time.monotonic()
        with self.lock:
            if (
                self.latest_guarded_pose_probe_monotonic > 0
                and now_monotonic - self.latest_guarded_pose_probe_monotonic
                < NAVIGATION_GUARDED_POSE_PROBE_INTERVAL_SEC
            ):
                return
            self.latest_guarded_pose_probe_monotonic = now_monotonic

        safety = self._get_navigation_safety_status(now_monotonic)
        if not safety['safetyReady']:
            return
        if self.tf_buffer is None or RosTime is None:
            with self.lock:
                self.latest_guarded_pose_probe_error = 'tf2 buffer is not available.'
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                NAVIGATION_GLOBAL_FRAME,
                NAVIGATION_BASE_FRAME,
                RosTime(),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = (
                translation.x,
                translation.y,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            if not all(isfinite(value) for value in values):
                raise ValueError('guarded map-to-base transform is not finite')
            transform_stamp_ns = self._message_stamp_ns(transform)
            try:
                now_ros_ns = int(self.node.get_clock().now().nanoseconds)
            except Exception as exc:  # noqa: BLE001 - trust requires source time
                raise ValueError(
                    f'guarded map-to-base source clock is unavailable: {exc}'
                ) from exc
            transform_age_sec = source_stamp_age_sec(
                now_ros_ns, transform_stamp_ns
            )
            if transform_age_sec is None:
                raise ValueError(
                    'guarded map-to-base transform has a zero or invalid-future '
                    'source stamp'
                )
            if transform_age_sec > NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT:
                raise ValueError(
                    'guarded map-to-base transform source stamp is stale '
                    f'({transform_age_sec:.3f}s)'
                )
        except Exception as exc:  # noqa: BLE001 - pose publication is fail-closed
            with self.lock:
                self.latest_guarded_pose_probe_error = str(exc)
            return

        with self.lock:
            self.latest_guarded_pose = {
                'ok': True,
                'deviceId': DEVICE_ID,
                'frameId': NAVIGATION_GLOBAL_FRAME,
                'x': float(translation.x),
                'y': float(translation.y),
                'yaw': _yaw_from_quaternion(rotation),
                'stamp': _utc_now(),
                'message': 'Guarded robot pose ready',
            }
            self.latest_guarded_pose_observed_monotonic = now_monotonic
            self.latest_guarded_pose_stamp_ns = transform_stamp_ns
            self.latest_guarded_pose_probe_error = ''

    def _on_recharge_status(self, message) -> None:
        status = str(getattr(message, 'data', '') or '').strip().upper()
        if not status:
            return
        with self.lock:
            self.recharge_status = status
            self.recharge_status_received_monotonic = time.monotonic()

    def _on_recharge_result(self, message) -> None:
        result = str(getattr(message, 'data', '') or '').strip()
        if not result:
            return
        with self.lock:
            self.recharge_result = result
            self.recharge_result_received_monotonic = time.monotonic()

    def _on_recharge_charging_flag(self, message) -> None:
        with self.lock:
            self.recharge_charging_flag = bool(getattr(message, 'data', False))
            self.recharge_charging_flag_received_monotonic = time.monotonic()

    def _on_recharge_charging_current(self, message) -> None:
        try:
            current = float(getattr(message, 'data', 0.0))
        except (TypeError, ValueError):
            return
        if not isfinite(current):
            return
        with self.lock:
            self.recharge_charging_current = current
            self.recharge_charging_current_received_monotonic = time.monotonic()

    def _on_recharge_mapping_gate(self, message) -> None:
        with self.lock:
            self.recharge_mapping_motion_blocked = bool(
                getattr(message, 'data', False)
            )
            self.recharge_mapping_gate_received = True
            self.recharge_mapping_gate_received_monotonic = time.monotonic()

    def _on_mapping_voice_state(self, message) -> None:
        """Cache Cartographer's retained, high-frequency voice state.

        The producer is intentionally separate from the session manager.  It
        confirms quick transitions such as STARTING and PAUSED; terminal truth
        still comes from the atomically written Cartographer session file.
        """
        now_monotonic = time.monotonic()
        try:
            payload = json.loads(str(getattr(message, 'data', '') or ''))
            if not isinstance(payload, dict):
                raise ValueError('Cartographer voice state must be a JSON object.')
            state_name = str(payload.get('state') or '').strip().upper()
            motion_blocked = payload.get('motion_blocked')
            lock_held = payload.get('lock_held')
            unit_state = payload.get('unit_state')
            pending_action = payload.get('pending_action')
            nav_mode = payload.get('nav_mode')
            status_error = payload.get('status_error')
            status_message = payload.get('message')
            if (
                state_name not in {'IDLE', 'STARTING', 'RUNNING', 'PAUSED', 'ENDING'}
                or type(motion_blocked) is not bool
                or type(lock_held) is not bool
                or not isinstance(unit_state, str)
                or not isinstance(pending_action, str)
                or (nav_mode is not None and not isinstance(nav_mode, str))
                or not isinstance(status_error, str)
                or not isinstance(status_message, str)
            ):
                raise ValueError('Cartographer voice state fields are invalid.')
        except Exception as exc:  # noqa: BLE001 - state observation is fail-closed
            with self.lock:
                self.mapping_voice_state_error = (
                    f'Cartographer voice state is invalid: {exc}'
                )
                self.mapping_voice_state_error_monotonic = now_monotonic
            return

        with self.lock:
            self.mapping_voice_state = {
                'state': state_name,
                'motionBlocked': motion_blocked,
                'unitState': unit_state,
                'lockHeld': lock_held,
                'pendingAction': pending_action,
                'navMode': nav_mode or '',
                'statusError': status_error,
                'message': status_message,
            }
            self.mapping_voice_state_received_monotonic = now_monotonic
            self.mapping_voice_state_error = ''
            self.mapping_voice_state_error_monotonic = 0.0

    def _normalize_voice_target_navigation_status_payload(self, payload: object) -> dict:
        """Validate the dedicated I/J/K lifecycle shared by ROS and disk.

        The persisted file is a restart-safety interlock, not a generic Nav2
        observation.  This validation deliberately rejects partial legacy data
        instead of allowing a new target to overwrite an un-attributed goal.
        """
        if not isinstance(payload, dict):
            raise ValueError('Voice target-navigation state must be a JSON object.')
        state_name = str(payload.get('state') or '').strip().lower()
        goal_id = payload.get('goalId')
        goal = payload.get('goal')
        active = payload.get('active')
        terminal = payload.get('terminal')
        interlock_active = payload.get('interlockActive')
        recovery_required = payload.get('recoveryRequired')
        can_cancel = payload.get('canCancel')
        action_uuid = payload.get('actionUuid')
        status_message = payload.get('message')
        last_error = payload.get('lastError')
        updated_at_epoch_ms = payload.get('updatedAtEpochMs')
        terminal_state = state_name in {'succeeded', 'failed', 'canceled'}
        active_state = state_name in {'starting', 'navigating'}
        if (
            payload.get('schemaVersion') != 1
            or payload.get('source') != 'voice_command_recognition'
            or state_name not in {
                'unknown', 'starting', 'navigating',
                'succeeded', 'failed', 'canceled',
            }
            or not isinstance(goal_id, str)
            or type(active) is not bool
            or type(terminal) is not bool
            or type(interlock_active) is not bool
            or type(recovery_required) is not bool
            or type(can_cancel) is not bool
            or not isinstance(action_uuid, str)
            or not isinstance(status_message, str)
            or not isinstance(last_error, str)
            or type(updated_at_epoch_ms) is not int
            or updated_at_epoch_ms <= 0
            or active != active_state
            or terminal != terminal_state
            or (active and not interlock_active)
            or (terminal and (interlock_active or recovery_required))
            or (recovery_required and (not interlock_active or active or terminal))
        ):
            raise ValueError('Voice target-navigation state fields are invalid.')

        normalized_goal = None
        if goal is not None:
            if not isinstance(goal, dict):
                raise ValueError('Voice target-navigation goal must be an object.')
            frame_id = goal.get('frameId')
            coordinates = (
                goal.get('x'),
                goal.get('y'),
                goal.get('orientationZ'),
                goal.get('orientationW'),
            )
            label = goal.get('label', '')
            if (
                not isinstance(frame_id, str)
                or not frame_id.strip()
                or not isinstance(label, str)
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not isfinite(float(value))
                    for value in coordinates
                )
            ):
                raise ValueError('Voice target-navigation goal fields are invalid.')
            normalized_goal = {
                'frameId': frame_id.strip().lstrip('/'),
                'x': float(coordinates[0]),
                'y': float(coordinates[1]),
                'orientationZ': float(coordinates[2]),
                'orientationW': float(coordinates[3]),
                'label': label,
            }
        if active and (not goal_id.strip() or normalized_goal is None):
            raise ValueError('An active voice target-navigation state needs goal identity.')
        return {
            'state': state_name,
            'active': active,
            'terminal': terminal,
            'interlockActive': interlock_active,
            'recoveryRequired': recovery_required,
            'goalId': goal_id,
            'goal': normalized_goal,
            'canCancel': can_cancel,
            'actionUuid': action_uuid,
            'message': status_message,
            'lastError': last_error,
            'sourceUpdatedAtEpochMs': updated_at_epoch_ms,
        }

    def _on_voice_target_navigation_status(self, message) -> None:
        """Accept only dedicated normal voice-target navigation observations."""
        now_monotonic = time.monotonic()
        try:
            payload = json.loads(str(getattr(message, 'data', '') or ''))
            normalized = self._normalize_voice_target_navigation_status_payload(payload)
        except Exception as exc:  # noqa: BLE001 - feature state must be fail-closed
            with self.lock:
                self.voice_target_navigation_status_error = (
                    f'Voice target-navigation state is invalid: {exc}'
                )
                self.voice_target_navigation_status_error_monotonic = now_monotonic
            return

        with self.lock:
            self.voice_target_navigation_status = normalized
            self.voice_target_navigation_status_received_monotonic = now_monotonic
            self.voice_target_navigation_status_error = ''
            self.voice_target_navigation_status_error_monotonic = 0.0

    def _read_voice_target_navigation_session_status(self) -> tuple[dict | None, str]:
        """Read the atomic I/J/K recovery session without treating it as Nav2 truth."""
        if not VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH:
            return None, 'Voice target-navigation session path is not configured.'
        try:
            with open(
                VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH,
                'r',
                encoding='utf-8',
            ) as handle:
                payload = json.load(handle)
        except FileNotFoundError:
            return None, ''
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            return None, f'Voice target-navigation session cannot be read: {exc}'
        try:
            return self._normalize_voice_target_navigation_status_payload(payload), ''
        except ValueError as exc:
            return None, f'Voice target-navigation session is invalid: {exc}'

    def _write_voice_target_navigation_session_status(self, payload: dict) -> str:
        """Atomically mark a persisted voice target terminal after safe shutdown."""
        if not VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH:
            return 'Voice target-navigation session path is not configured.'
        directory = os.path.dirname(VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH) or '.'
        temporary_path = (
            f'{VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH}.tmp.'
            f'{os.getpid()}.{threading.get_ident()}'
        )
        try:
            os.makedirs(directory, exist_ok=True)
            with open(temporary_path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, ensure_ascii=False, separators=(',', ':'))
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, VOICE_TARGET_NAVIGATION_SESSION_STATUS_PATH)
            return ''
        except OSError as exc:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass
            return f'Voice target-navigation session cannot be written: {exc}'

    def _clear_voice_target_navigation_session_after_confirmed_shutdown(
        self,
        session: dict,
    ) -> tuple[dict, str]:
        """Clear only an unresolved I/J/K session after Nav2 disappearance.

        The exact action UUID is otherwise canceled by command_recognition.  If
        that UUID was never recorded because of a crash, only this conservative
        process/action-server disappearance proof may release the interlock.
        """
        if session.get('terminal') is True and session.get('active') is False:
            return session, ''
        navigation = self.get_navigation_system_status()
        if not (
            navigation.get('state') == 'stopped'
            and navigation.get('launchObserved') is False
            and navigation.get('actionServerReady') is False
        ):
            return session, ''
        cleared = dict(session)
        cleared.update({
            'state': 'canceled',
            'active': False,
            'terminal': True,
            'interlockActive': False,
            'recoveryRequired': False,
            'canCancel': False,
            'message': (
                'Navigation system shutdown was confirmed; unresolved voice '
                'target-navigation session was cleared.'
            ),
            'lastError': '',
            'updatedAtEpochMs': int(time.time() * 1000),
        })
        write_error = self._write_voice_target_navigation_session_status(cleared)
        return (cleared if not write_error else session), write_error

    def _get_topic_publisher_observation(self, topic: str) -> dict:
        """Read graph presence without making a blocking ROS service call."""
        graph_probe_available = False
        publisher_observed = False
        try:
            get_publishers = getattr(self.node, 'get_publishers_info_by_topic', None)
            if callable(get_publishers):
                publisher_observed = bool(get_publishers(topic))
                graph_probe_available = True
        except Exception:
            pass
        return {
            'graphProbeAvailable': graph_probe_available,
            'publisherObserved': publisher_observed,
        }

    def _read_mapping_session_status(self) -> tuple[dict | None, str]:
        """Read Cartographer's atomic session summary without mutating it."""
        if not MAPPING_SESSION_STATUS_PATH:
            return None, 'Cartographer session status path is not configured.'
        try:
            with open(MAPPING_SESSION_STATUS_PATH, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)
        except FileNotFoundError:
            return None, 'Cartographer session status has not been created yet.'
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            return None, f'Cartographer session status cannot be read: {exc}'
        if not isinstance(payload, dict):
            return None, 'Cartographer session status must be a JSON object.'

        updated_at_epoch_ms = payload.get('updatedAtEpochMs')
        phase = payload.get('phase')
        task_id = payload.get('taskId', '')
        if (
            type(updated_at_epoch_ms) is not int
            or updated_at_epoch_ms <= 0
            or not isinstance(phase, str)
            or not phase.strip()
            or not isinstance(task_id, str)
        ):
            return None, 'Cartographer session status fields are invalid.'

        return {
            'taskId': task_id.strip(),
            'updatedAtEpochMs': updated_at_epoch_ms,
            'phase': phase.strip().lower(),
            'sessionActive': payload.get('sessionActive') is True,
            'terminal': payload.get('terminal') is True,
            'functionalServicesRestored': (
                payload.get('functionalServicesRestored') is True
            ),
            'mappingDone': payload.get('mappingDone') is True,
            'finalMapSaved': payload.get('finalMapSaved') is True,
            # ``False`` is meaningful: it proves the manager has accepted a
            # resume from a non-voice control path.  Keep ``None`` for older
            # session files so they cannot accidentally clear a voice pause.
            'voicePaused': (
                payload.get('voicePaused')
                if type(payload.get('voicePaused')) is bool else None
            ),
            'mode': str(payload.get('mode') or '').strip().lower(),
            'manualTakeoverRequired': payload.get('manualTakeoverRequired') is True,
            'status': str(
                payload.get('taskStatus') or payload.get('status') or ''
            ).strip().lower(),
            'lastError': str(payload.get('lastError') or '').strip(),
            'returnHomeFailureReason': str(
                payload.get('returnHomeFailureReason') or ''
            ).strip(),
        }, ''

    def _record_body_follow_probe_error(self, message: str) -> None:
        with self.lock:
            self.body_follow_status_error = str(
                message or 'Body-follow status is unavailable.'
            )
            self.body_follow_status_error_monotonic = time.monotonic()
            self.body_follow_status_consecutive_failures += 1

    def _on_body_follow_status_response(self, future, generation: int) -> None:
        now_monotonic = time.monotonic()
        with self.lock:
            if (
                generation != self.body_follow_status_active_generation
                or self.body_follow_status_request_future is not future
            ):
                # A timed-out request may complete after a newer request.  Its
                # answer belongs to an older controller observation and must
                # never overwrite the newer cache.
                return
        try:
            response = future.result()
            if response is None or response.success is not True:
                raise RuntimeError(
                    getattr(response, 'message', '')
                    or 'Body-follow status service returned success=false.'
                )
            raw_status = json.loads(str(response.message or ''))
            if not isinstance(raw_status, dict):
                raise ValueError('Body-follow status response must be a JSON object.')
            enabled = raw_status.get('enabled')
            controller_applied = raw_status.get('controllerApplied')
            pending = raw_status.get('pending')
            state_name = raw_status.get('state')
            subscriber_count = raw_status.get('modeSubscriberCount')
            last_error = raw_status.get('lastError')
            if (
                type(enabled) is not bool
                or type(controller_applied) is not bool
                or type(pending) is not bool
                or not isinstance(state_name, str)
                or not isinstance(subscriber_count, int)
                or subscriber_count < 0
                or not isinstance(last_error, str)
            ):
                raise ValueError('Body-follow status response fields are invalid.')
        except Exception as exc:  # noqa: BLE001 - a failed read must be fail-closed
            with self.lock:
                if (
                    generation != self.body_follow_status_active_generation
                    or self.body_follow_status_request_future is not future
                ):
                    return
                self.body_follow_status_request_future = None
                self.body_follow_status_error = f'Body-follow status probe failed: {exc}'
                self.body_follow_status_error_monotonic = now_monotonic
                self.body_follow_status_consecutive_failures += 1
            return

        with self.lock:
            if (
                generation != self.body_follow_status_active_generation
                or self.body_follow_status_request_future is not future
            ):
                return
            self.body_follow_status_request_future = None
            self.body_follow_status = {
                'enabled': enabled,
                'controllerApplied': controller_applied,
                'pending': pending,
                'state': state_name,
                'modeSubscriberCount': subscriber_count,
                'lastError': last_error,
            }
            self.body_follow_status_error = ''
            self.body_follow_status_error_monotonic = 0.0
            self.body_follow_status_consecutive_failures = 0
            self.body_follow_status_received_monotonic = now_monotonic

    def _schedule_body_follow_status_probe(self) -> None:
        """Refresh follow status with one long-lived ROS client request.

        This deliberately avoids repeatedly launching the ROS CLI.  Its process
        startup and DDS discovery were slower than the UI polling interval on
        the vehicle, which made a healthy controller appear permanently unknown.
        """
        now_monotonic = time.monotonic()
        with self.lock:
            future = self.body_follow_status_request_future
            if future is not None and not future.done():
                started_at = self.body_follow_status_request_started_monotonic
                if (
                    started_at > 0
                    and now_monotonic - started_at
                    > BODY_FOLLOW_STATUS_RESPONSE_TIMEOUT
                ):
                    # Do not cancel while holding the cache lock: some rclpy
                    # futures invoke callbacks synchronously on cancellation.
                    # Invalidating the generation is sufficient; a late reply
                    # is ignored and cannot overwrite the next observation.
                    self.body_follow_status_request_future = None
                    self.body_follow_status_active_generation += 1
                    self.body_follow_status_error = (
                        'Body-follow status service response timed out.'
                    )
                    self.body_follow_status_error_monotonic = now_monotonic
                    self.body_follow_status_consecutive_failures += 1
                return
            if (
                self.body_follow_status_last_requested_monotonic > 0
                and now_monotonic
                - self.body_follow_status_last_requested_monotonic
                < BODY_FOLLOW_STATUS_PROBE_INTERVAL
            ):
                return
            self.body_follow_status_last_requested_monotonic = now_monotonic

        client = self.body_follow_status_client
        if client is None:
            self._record_body_follow_probe_error(
                'Body-follow status client is not available.'
            )
            return
        try:
            if not client.service_is_ready():
                self._record_body_follow_probe_error(
                    'Body-follow status service is not available.'
                )
                return
            future = client.call_async(Trigger.Request())
            with self.lock:
                self.body_follow_status_request_generation += 1
                generation = self.body_follow_status_request_generation
                self.body_follow_status_active_generation = generation
                self.body_follow_status_request_future = future
                self.body_follow_status_request_started_monotonic = now_monotonic
            future.add_done_callback(
                lambda completed, request_generation=generation:
                self._on_body_follow_status_response(
                    completed, request_generation
                )
            )
        except Exception as exc:  # noqa: BLE001 - feature observation only
            self._record_body_follow_probe_error(
                f'Body-follow status request could not be started: {exc}'
            )

    def get_body_follow_feature_status(self) -> dict:
        now_monotonic = time.monotonic()
        now_epoch_ms = int(time.time() * 1000)
        with self.lock:
            raw_status = (
                dict(self.body_follow_status)
                if isinstance(self.body_follow_status, dict)
                else None
            )
            received_at = self.body_follow_status_received_monotonic
            error = self.body_follow_status_error
            consecutive_failures = self.body_follow_status_consecutive_failures

        if (
            raw_status is None
            or received_at <= 0
            or now_monotonic - received_at > BODY_FOLLOW_STATUS_STALE_TIMEOUT
            or consecutive_failures >= BODY_FOLLOW_STATUS_FAILURE_THRESHOLD
        ):
            return {
                'state': 'unknown',
                'effectiveEnabled': False,
                'canEnable': False,
                'canDisable': False,
                'source': 'ros_service',
                'requestedEnabled': False,
                'controllerApplied': False,
                'pending': False,
                'modeSubscriberCount': 0,
                'controllerState': 'unknown',
                'lastError': error or 'Body-follow status is not fresh.',
                'observedAtEpochMs': now_epoch_ms,
            }

        requested_enabled = raw_status['enabled']
        controller_applied = raw_status['controllerApplied']
        pending = raw_status['pending']
        controller_state = raw_status['state']
        reported_error = raw_status['lastError'].strip()
        if reported_error:
            state_name = 'failed'
            last_error = reported_error
        elif pending:
            state_name = 'starting' if requested_enabled else 'stopping'
            last_error = ''
        elif not controller_applied:
            state_name = 'failed'
            last_error = (
                'Body-follow controller has not applied the requested state.'
            )
        elif requested_enabled:
            state_name = 'ready'
            last_error = ''
        else:
            state_name = 'stopped'
            last_error = ''

        return {
            'state': state_name,
            'effectiveEnabled': state_name == 'ready',
            'canEnable': state_name == 'stopped',
            'canDisable': state_name == 'ready',
            'source': 'ros_service',
            'requestedEnabled': requested_enabled,
            'controllerApplied': controller_applied,
            'pending': pending,
            'modeSubscriberCount': raw_status['modeSubscriberCount'],
            'controllerState': controller_state,
            'lastError': last_error,
            'observedAtEpochMs': now_epoch_ms,
        }

    def _get_recharge_graph_status(self) -> dict:
        graph_probe_available = False
        manager_node_observed = False
        status_publisher_observed = False
        try:
            get_names = getattr(self.node, 'get_node_names', None)
            names = (
                get_names()
                if callable(get_names)
                else self.node.get_node_names_and_namespaces()
            )
            graph_probe_available = True
            for item in names:
                name = str(item[0] if isinstance(item, tuple) else item or '')
                if name in RECHARGE_MANAGER_NODE_NAMES:
                    manager_node_observed = True
                    break
        except Exception:
            pass
        try:
            get_publishers = getattr(
                self.node, 'get_publishers_info_by_topic', None
            )
            if callable(get_publishers):
                status_publisher_observed = bool(
                    get_publishers(RECHARGE_STATUS_TOPIC)
                )
                graph_probe_available = True
        except Exception:
            pass
        return {
            'graphProbeAvailable': graph_probe_available,
            'managerNodeObserved': manager_node_observed,
            'statusPublisherObserved': status_publisher_observed,
        }

    def get_recharge_feature_status(self) -> dict:
        now_monotonic = time.monotonic()
        now_epoch_ms = int(time.time() * 1000)
        graph_status = self._get_recharge_graph_status()
        with self.lock:
            manager_status = self.recharge_status
            charging_flag = self.recharge_charging_flag
            charging_flag_at = self.recharge_charging_flag_received_monotonic
            charging_current = self.recharge_charging_current
            charging_current_at = self.recharge_charging_current_received_monotonic
            mapping_motion_blocked = self.recharge_mapping_motion_blocked
            mapping_gate_received = self.recharge_mapping_gate_received
            last_result = self.recharge_result
            last_result_at = self.recharge_result_received_monotonic

        charging_signal_fresh = (
            charging_flag_at > 0
            and now_monotonic - charging_flag_at <= RECHARGE_SIGNAL_STALE_TIMEOUT
        )
        charging_current_fresh = (
            charging_current_at > 0
            and now_monotonic - charging_current_at
            <= RECHARGE_SIGNAL_STALE_TIMEOUT
        )
        charging_confirmed = bool(charging_flag) and charging_signal_fresh
        status_source_observed = bool(
            manager_status
            and graph_status['managerNodeObserved']
            and graph_status['statusPublisherObserved']
        )
        visible_result = (
            last_result
            if (
                last_result_at > 0
                and now_monotonic - last_result_at <= RECHARGE_RESULT_VISIBLE_TIMEOUT
            )
            else ''
        )
        failure_result = (
            visible_result
            if visible_result.startswith('FAILED:')
            else ''
        )
        manager_status = str(manager_status or '').strip().upper()

        if not status_source_observed:
            state_name = 'unknown'
            last_error = (
                'Recharge manager status source is not fully observable.'
            )
        elif manager_status == 'IDLE' and failure_result:
            state_name = 'failed'
            last_error = failure_result
        elif manager_status == 'IDLE' and charging_confirmed:
            # Docked and physically charging while the persistent manager has no task
            # running. This is still an active charging session: the mini program must be
            # able to cancel it (cancel = undock), so report it as ready + canDisable.
            # Reporting it as 'stopped' made the app show a disabled
            # "already charging, no need to recharge" button instead of "cancel recharge".
            state_name = 'ready'
            last_error = ''
        elif manager_status == 'IDLE':
            state_name = 'stopped'
            last_error = ''
        elif manager_status in {'WAITING_NAV2', 'STARTING', 'GOAL_PENDING'}:
            state_name = 'starting'
            last_error = ''
        elif manager_status in {'STOPPING', 'CANCELING'}:
            state_name = 'stopping'
            last_error = ''
        elif (
            manager_status == 'CHARGING'
            or manager_status == 'ACTIVE'
            or manager_status == 'NAVIGATING'
            or manager_status == 'IR_SEARCH'
            or manager_status == 'DOCKING'
            or manager_status.startswith('HANDOFF_')
            or manager_status.startswith('REFINING_')
        ):
            state_name = 'ready'
            last_error = ''
        else:
            state_name = 'unknown'
            last_error = (
                f'Unknown recharge manager state: {manager_status or "<empty>"}.'
            )

        can_start = (
            status_source_observed
            and mapping_gate_received
            and not mapping_motion_blocked
            and not charging_confirmed
        )
        if state_name == 'unknown':
            action_blocked_reason = last_error
        elif state_name in {'starting', 'ready', 'stopping'}:
            action_blocked_reason = ''
        elif mapping_motion_blocked:
            action_blocked_reason = (
                'Cartographer mapping currently owns vehicle motion.'
            )
        elif not mapping_gate_received:
            action_blocked_reason = (
                'Cartographer mapping interlock state has not been observed yet.'
            )
        elif charging_confirmed:
            action_blocked_reason = (
                'Vehicle is physically charging; a new recharge task is not allowed.'
            )
        else:
            action_blocked_reason = ''

        return {
            'state': state_name,
            'effectiveEnabled': state_name == 'ready',
            'canEnable': state_name in {'stopped', 'failed'} and can_start,
            'canDisable': state_name in {'starting', 'ready'},
            'source': 'recharge_manager',
            'managerStatus': manager_status or 'unknown',
            **graph_status,
            'statusSourceObserved': status_source_observed,
            'mappingMotionBlocked': mapping_motion_blocked,
            'mappingGateReceived': mapping_gate_received,
            'chargingSignalFresh': charging_signal_fresh,
            'chargingConfirmed': charging_confirmed,
            'chargingCurrentAmps': (
                charging_current if charging_current_fresh else None
            ),
            'chargingCurrentFresh': charging_current_fresh,
            'lastResult': visible_result,
            'lastError': last_error,
            'actionBlockedReason': action_blocked_reason,
            'observedAtEpochMs': now_epoch_ms,
        }

    def _mapping_startup_base_service_status(
        self, now_monotonic: float
    ) -> tuple[bool, str]:
        """Read the base-service state at a bounded rate without controlling it."""
        with self.lock:
            if (
                now_monotonic - self.mapping_startup_service_probe_monotonic
                < MAPPING_STARTUP_SERVICE_PROBE_INTERVAL_SEC
            ):
                return (
                    self.mapping_startup_base_service_active,
                    self.mapping_startup_service_probe_error,
                )

        active = False
        error = ''
        try:
            completed = subprocess.run(
                ['systemctl', 'is-active', MAPPING_STARTUP_BASE_SERVICE],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            active = completed.returncode == 0 and (
                completed.stdout.strip().lower() == 'active'
            )
            if not active:
                error = completed.stderr.strip() or completed.stdout.strip()
        except (OSError, subprocess.SubprocessError) as exc:
            error = str(exc)

        with self.lock:
            self.mapping_startup_base_service_active = active
            self.mapping_startup_service_probe_error = error
            self.mapping_startup_service_probe_monotonic = now_monotonic
        return active, error

    def get_vehicle_readiness(self) -> dict:
        """Return the read-only cold-start gate for beginning a mapping session.

        This intentionally observes only services and data that exist before a
        Cartographer/Nav2 session.  AMCL and map->base are excluded because they
        are not expected before mapping has started.  The lidar is also excluded
        from the hard gate: this vehicle deliberately starts its lidar inside
        the isolated Cartographer launch, where the existing preflight guard
        validates ``/scan`` before mapping can progress.
        """
        now_monotonic = time.monotonic()
        now_epoch_ms = int(time.time() * 1000)
        base_service_active, service_error = self._mapping_startup_base_service_status(
            now_monotonic
        )
        with self.lock:
            scan_age_sec = (
                now_monotonic - self.latest_scan_received_monotonic
                if self.latest_scan_received_monotonic > 0 else None
            )
            odom_age_sec = (
                now_monotonic - self.latest_odom_received_monotonic
                if self.latest_odom_received_monotonic > 0 else None
            )
            imu_age_sec = (
                now_monotonic - self.latest_imu_received_monotonic
                if self.latest_imu_received_monotonic > 0 else None
            )
            odom_to_base_age_sec = (
                now_monotonic - self.latest_odom_to_base_received_monotonic
                if self.latest_odom_to_base_received_monotonic > 0 else None
            )
            odom_advancing = (
                self.odom_advancing_samples >= MAPPING_STARTUP_REQUIRED_SAMPLES
            )
            odom_to_base_advancing = (
                self.odom_to_base_advancing_samples
                >= MAPPING_STARTUP_REQUIRED_SAMPLES
            )
            base_to_imu_last_observed = (
                self.base_to_imu_transform_last_observed_monotonic
            )
            base_to_imu_probe_available = self.tf_buffer is not None
            base_to_imu_probe_error = self.base_to_imu_transform_probe_error
            elapsed_since_start = (
                now_monotonic - self.mapping_startup_started_monotonic
            )

        scan_fresh = (
            scan_age_sec is not None
            and scan_age_sec <= MAPPING_STARTUP_FRESH_TIMEOUT_SEC
        )
        odom_fresh = (
            odom_age_sec is not None
            and odom_age_sec <= MAPPING_STARTUP_FRESH_TIMEOUT_SEC
        )
        imu_fresh = (
            imu_age_sec is not None
            and imu_age_sec <= MAPPING_STARTUP_FRESH_TIMEOUT_SEC
        )
        odom_to_base_fresh = (
            odom_to_base_age_sec is not None
            and odom_to_base_age_sec <= MAPPING_STARTUP_FRESH_TIMEOUT_SEC
        )
        base_to_imu_age_sec = (
            now_monotonic - base_to_imu_last_observed
            if base_to_imu_last_observed > 0 else None
        )
        base_to_imu_observed = (
            base_to_imu_age_sec is not None
            and base_to_imu_age_sec <= MAPPING_STARTUP_TF_STALE_TIMEOUT_SEC
        )
        candidate_ready = (
            base_service_active
            and odom_fresh
            and odom_advancing
            and imu_fresh
            and base_to_imu_observed
            and odom_to_base_fresh
            and odom_to_base_advancing
        )

        with self.lock:
            if candidate_ready:
                if self.mapping_startup_stable_since_monotonic <= 0:
                    self.mapping_startup_stable_since_monotonic = now_monotonic
                if (
                    now_monotonic - self.mapping_startup_stable_since_monotonic
                    >= MAPPING_STARTUP_STABLE_WINDOW_SEC
                ):
                    if self.mapping_startup_ready_since_monotonic <= 0:
                        self.mapping_startup_ready_since_monotonic = now_monotonic
                    self.mapping_startup_state = 'ready'
            else:
                self.mapping_startup_stable_since_monotonic = 0.0
                ready_grace_active = (
                    self.mapping_startup_state == 'ready'
                    and self.mapping_startup_ready_since_monotonic > 0
                    and now_monotonic - self.mapping_startup_ready_since_monotonic
                    <= MAPPING_STARTUP_READY_GRACE_SEC
                )
                if not ready_grace_active:
                    self.mapping_startup_state = (
                        'failed'
                        if elapsed_since_start >= MAPPING_STARTUP_TIMEOUT_SEC
                        else 'initializing'
                    )
            state_name = self.mapping_startup_state
            stable_since_monotonic = self.mapping_startup_stable_since_monotonic
            ready_since_monotonic = self.mapping_startup_ready_since_monotonic

        if state_name == 'ready':
            message = '车端已完成初始化，可以开始建图'
        elif state_name == 'failed':
            message = '车端初始化超时，请检查底盘、里程计、IMU 和坐标变换'
        else:
            message = '车端初始化中，正在等待底盘、里程计、IMU 和坐标变换稳定'

        def monotonic_to_epoch_ms(value: float) -> int:
            if value <= 0:
                return 0
            return max(0, int(now_epoch_ms - (now_monotonic - value) * 1000))

        return {
            'state': state_name,
            'mappingStartAllowed': state_name == 'ready',
            'message': message,
            'baseServiceActive': base_service_active,
            'serviceProbeError': service_error,
            'scanFresh': scan_fresh,
            'odomFresh': odom_fresh,
            'odomAdvancing': odom_advancing,
            'imuFresh': imu_fresh,
            'baseToImuTfObserved': base_to_imu_observed,
            'baseToImuTfAgeSec': base_to_imu_age_sec,
            'baseToImuTfProbeAvailable': base_to_imu_probe_available,
            'baseToImuTfProbeError': base_to_imu_probe_error,
            'odomToBaseTfFresh': odom_to_base_fresh,
            'odomToBaseTfAdvancing': odom_to_base_advancing,
            'stableSinceEpochMs': monotonic_to_epoch_ms(stable_since_monotonic),
            'readySinceEpochMs': monotonic_to_epoch_ms(ready_since_monotonic),
            'startedAtEpochMs': self.mapping_startup_started_epoch_ms,
            'observedAtEpochMs': now_epoch_ms,
        }

    def get_mapping_feature_status(
        self, vehicle_readiness: Optional[dict] = None
    ) -> dict:
        """Combine live voice state with Cartographer's persisted session truth.

        A retained voice ``RUNNING`` message by itself is never sufficient to
        claim the vehicle is still mapping: it must still be fresh and its
        publisher must be observable.  Conversely, a closed session only turns
        into ``stopped`` once the session runner confirms functional services
        have been restored.
        """
        now_monotonic = time.monotonic()
        now_epoch_ms = int(time.time() * 1000)
        if not isinstance(vehicle_readiness, dict):
            vehicle_readiness = self.get_vehicle_readiness()
        linux_boot_epoch_ms = _linux_boot_epoch_ms()
        voice_graph = self._get_topic_publisher_observation(
            MAPPING_VOICE_STATE_TOPIC
        )
        session, session_error = self._read_mapping_session_status()
        with self.lock:
            voice_state = (
                dict(self.mapping_voice_state)
                if isinstance(self.mapping_voice_state, dict)
                else None
            )
            voice_received_at = self.mapping_voice_state_received_monotonic
            voice_error = self.mapping_voice_state_error
            voice_error_at = self.mapping_voice_state_error_monotonic

        voice_fresh = (
            voice_state is not None
            and voice_received_at > 0
            and now_monotonic - voice_received_at
            <= MAPPING_VOICE_STATE_STALE_TIMEOUT
            and not (voice_error and voice_error_at >= voice_received_at)
        )
        # If graph inspection is unavailable, a locally received, fresh state
        # is still useful.  If it is available, require the publisher too so a
        # historical retained RUNNING value cannot claim an active session.
        voice_observable = voice_fresh and (
            not voice_graph['graphProbeAvailable']
            or voice_graph['publisherObserved']
        )
        voice_name = (
            str(voice_state.get('state') or 'unknown')
            if voice_state is not None else 'unknown'
        )

        session_fresh = False
        session_terminal = False
        session_active = False
        session_starting = False
        session_phase = 'idle'
        session_status = ''
        session_last_error = ''
        session_restored = False
        session_voice_paused = None
        session_mode = ''
        prior_boot_orphan_session = False
        mapping_done = False
        final_map_saved = False
        task_id = ''
        source_updated_at_epoch_ms = now_epoch_ms
        if session is not None:
            session_age_ms = now_epoch_ms - session['updatedAtEpochMs']
            session_fresh = (
                session_age_ms >= 0
                and session_age_ms
                <= int(MAPPING_ACTIVE_SESSION_STALE_TIMEOUT * 1000)
            )
            session_terminal = session['terminal']
            session_phase = session['phase']
            session_status = session['status']
            session_restored = session['functionalServicesRestored']
            session_voice_paused = session['voicePaused']
            session_mode = session['mode']
            mapping_done = session['mappingDone']
            final_map_saved = session['finalMapSaved']
            task_id = session['taskId']
            session_last_error = (
                session['lastError'] or session['returnHomeFailureReason']
            )
            source_updated_at_epoch_ms = session['updatedAtEpochMs']
            # The runner writes ``phase=starting`` before it has stopped any
            # functional service or launched Cartographer.  It is a real,
            # accepted request, but not evidence that mapping is running.
            # Keep it separate so a cold-start wait never exposes pause/finish
            # controls or a false "mapping active" UI state.
            session_starting = (
                not session_terminal
                and session_fresh
                and session['sessionActive']
                and session_phase in {'starting', 'idle', 'closed'}
            )
            session_active = (
                not session_terminal
                and session_fresh
                and (
                    session_phase in {
                        'mapping', 'paused', 'returning_home', 'finalizing',
                        'canceling', 'recovery_required', 'return_home_failed',
                    }
                    or (
                        session['sessionActive']
                        and session_phase not in {'starting', 'idle', 'closed'}
                    )
                )
            )
            # Do not let a non-terminal ledger from an earlier Linux boot
            # masquerade as a currently running Cartographer session.  This is
            # deliberately narrower than a generic stale-session override:
            # a stale record from *this* boot remains fail-closed.
            prior_boot_orphan_session = (
                not session_terminal
                and session['sessionActive']
                and not session_fresh
                and linux_boot_epoch_ms > 0
                and session['updatedAtEpochMs'] < linux_boot_epoch_ms
            )
        if voice_observable:
            source_updated_at_epoch_ms = now_epoch_ms

        voice_status_error = (
            voice_state['statusError'].strip()
            if voice_observable and voice_state is not None else ''
        )
        voice_motion_blocked = bool(
            voice_state['motionBlocked']
            if voice_observable and voice_state is not None else False
        )
        session_failure = (
            session is not None
            and session_fresh
            and not session_terminal
            and session_phase in {'return_home_failed', 'recovery_required'}
        )
        session_stopping = (
            session is not None
            and session_fresh
            and not session_terminal
            and session_phase in {'returning_home', 'finalizing', 'canceling'}
        )
        session_terminal_failure = (
            session is not None
            and session_terminal
            and session_status in {'failed', 'error'}
        )
        terminal_stopped = (
            session is not None
            and session_terminal
            and session_restored
            and not session_terminal_failure
        )

        state_name = 'unknown'
        last_error = ''
        if session_failure:
            state_name = 'failed'
            last_error = session_last_error or (
                'Cartographer session requires manual recovery.'
            )
        elif session_terminal_failure:
            state_name = 'failed'
            last_error = session_last_error or 'Cartographer session failed.'
        elif voice_status_error:
            state_name = 'failed'
            last_error = voice_status_error
        elif session_stopping or (voice_observable and voice_name == 'ENDING'):
            state_name = 'stopping'
        elif session_active or (
            voice_observable and voice_name in {'RUNNING', 'PAUSED'}
        ):
            # The atomically written session is fresh, active evidence that
            # Cartographer is already mapping.  A voice bridge may retain
            # STARTING while its nav-state probe lags at "idle"; it must not
            # keep the public feature state in a non-operational transition.
            state_name = 'ready'
        elif session_starting or (
            voice_observable and voice_name == 'STARTING'
        ):
            state_name = 'starting'
        elif terminal_stopped:
            state_name = 'stopped'
        elif (
            voice_observable
            and voice_name == 'IDLE'
            and not session_active
            and not (
                session is not None
                and not session_terminal
                and session['sessionActive']
                and not prior_boot_orphan_session
            )
        ):
            state_name = 'stopped'
        else:
            last_error = (
                voice_error
                or session_error
                or 'Cartographer mapping status is not fully observable.'
            )

        session_confirms_external_resume = (
            session_active
            and session_phase == 'mapping'
            and session_voice_paused is False
            and session_mode in {'mapping', 'navigating', 'exploring', 'exploration'}
        )
        paused = (
            state_name == 'ready'
            and (
                (
                    voice_observable
                    and voice_name == 'PAUSED'
                    and not session_confirms_external_resume
                )
                or session_phase == 'paused'
                or session_voice_paused is True
                or session_mode in {'paused', 'stopped', 'safety_stop'}
            )
        )
        phase = session_phase
        if state_name == 'ready' and paused:
            phase = 'paused'
        elif state_name == 'ready' and phase in {'idle', 'closed'}:
            phase = 'mapping'
        elif state_name == 'starting' and phase in {'idle', 'closed'}:
            phase = 'starting'
        elif state_name == 'stopping' and phase in {'idle', 'closed', 'mapping'}:
            phase = 'finalizing'
        elif state_name == 'stopped' and prior_boot_orphan_session:
            # The previous task remains available via taskId for diagnostics,
            # but it cannot describe an active phase in this Linux boot.
            phase = 'idle'

        status_source_observed = bool(
            voice_observable
            or session_active
            or session_starting
            or terminal_stopped
            or (session_terminal_failure and session_restored)
        )
        action_allowed = {
            'start': False,
            'pause': False,
            'resume': False,
            'finish': False,
        }
        if state_name == 'stopped' and status_source_observed:
            action_allowed['start'] = True
        elif (
            state_name == 'failed'
            and session_terminal
            and session_restored
        ):
            # A terminal failure with all services restored can safely be retried;
            # an in-progress recovery failure cannot.
            action_allowed['start'] = True
        elif state_name == 'ready':
            action_allowed.update({
                'pause': not paused,
                'resume': paused,
                'finish': True,
            })

        readiness_blocks_start = (
            state_name in {'stopped', 'failed', 'starting'}
            and vehicle_readiness.get('mappingStartAllowed') is not True
        )
        if readiness_blocks_start:
            # Initialization is a normal, non-error gate.  Keep the Cartographer
            # state truthful and only withhold the start permission so clients do
            # not turn a cold boot into a spurious mapping failure.
            action_allowed['start'] = False

        if voice_observable and session is not None:
            source = 'combined'
        elif session is not None:
            source = 'cartographer_session'
        elif voice_observable:
            source = 'voice_cartographer_bridge'
        else:
            source = 'unknown'

        action_blocked_reason = ''
        if state_name in {'unknown', 'failed'}:
            action_blocked_reason = last_error
        elif state_name in {'starting', 'ready', 'stopping'}:
            action_blocked_reason = (
                'Cartographer mapping session currently owns vehicle motion.'
            )
        if readiness_blocks_start:
            action_blocked_reason = str(
                vehicle_readiness.get('message')
                or '车端初始化中，暂不可开始建图'
            )

        reason_code, reason_text = _derive_mapping_reason_code(
            state_name=state_name,
            last_error=last_error,
            session=session,
            session_terminal=session_terminal,
            mapping_done=mapping_done,
            final_map_saved=final_map_saved,
        )

        return {
            'state': state_name,
            'effectiveEnabled': state_name == 'ready',
            'source': source,
            'taskId': task_id,
            'phase': phase,
            'voiceState': voice_name if voice_observable else 'unknown',
            'voiceStateObserved': voice_fresh,
            'voiceStatePublisherObserved': voice_graph['publisherObserved'],
            'sessionObserved': session is not None,
            'sessionFresh': session_fresh,
            'sessionActive': session_active,
            'sessionStarting': session_starting,
            'priorBootOrphanSession': prior_boot_orphan_session,
            'terminal': session_terminal,
            'functionalServicesRestored': session_restored,
            'motionBlocked': (
                voice_motion_blocked or session_active or session_starting
            ),
            'paused': paused,
            'mappingDone': mapping_done,
            'finalMapSaved': final_map_saved,
            'actionAllowed': action_allowed,
            'lastError': last_error,
            'actionBlockedReason': action_blocked_reason,
            # Diagnostics for the backend/debug view (F-34).  Not user-facing.
            'reasonCode': reason_code,
            'reasonText': reason_text,
            'sourceUpdatedAtEpochMs': source_updated_at_epoch_ms,
            'observedAtEpochMs': now_epoch_ms,
        }

    @staticmethod
    def _mapping_allows_navigation_start(mapping_session: object) -> tuple[bool, str]:
        """Require affirmative Cartographer-idle proof before new Nav2 work."""
        if not isinstance(mapping_session, dict):
            return False, 'Cartographer mapping status is unavailable.'
        state_name = str(mapping_session.get('state') or '').strip().lower()
        voice_idle = (
            mapping_session.get('voiceState') == 'IDLE'
            and mapping_session.get('voiceStateObserved') is True
            and mapping_session.get('voiceStatePublisherObserved') is True
        )
        terminal_recovered = (
            mapping_session.get('terminal') is True
            and mapping_session.get('functionalServicesRestored') is True
        )
        if state_name == 'stopped' and (voice_idle or terminal_recovered):
            return True, ''
        if state_name == 'failed' and terminal_recovered:
            return True, ''
        return False, (
            str(mapping_session.get('actionBlockedReason') or '').strip()
            or str(mapping_session.get('lastError') or '').strip()
            or (
                'Cartographer mapping status is not a confirmed idle or '
                'terminally recovered state.'
            )
        )

    def get_target_navigation_feature_status(self) -> dict:
        """Expose only the normal I/J/K voice target-navigation lifecycle.

        A fresh ROS message alone cannot clear an earlier non-terminal target:
        command_recognition may have restarted while Nav2 kept that exact goal.
        The atomic session therefore remains the recovery authority until its
        action reaches a terminal state or Nav2 shutdown is independently proven.
        """
        now_monotonic = time.monotonic()
        now_epoch_ms = int(time.time() * 1000)
        graph = self._get_topic_publisher_observation(
            VOICE_TARGET_NAVIGATION_STATUS_TOPIC
        )
        session, session_error = self._read_voice_target_navigation_session_status()
        if session is not None and not session['terminal']:
            session, clear_error = (
                self._clear_voice_target_navigation_session_after_confirmed_shutdown(
                    session
                )
            )
            if clear_error:
                session_error = clear_error

        with self.lock:
            raw_status = (
                dict(self.voice_target_navigation_status)
                if isinstance(self.voice_target_navigation_status, dict)
                else None
            )
            received_at = self.voice_target_navigation_status_received_monotonic
            status_error = self.voice_target_navigation_status_error
            error_at = self.voice_target_navigation_status_error_monotonic

        status_fresh = (
            raw_status is not None
            and received_at > 0
            and now_monotonic - received_at
            <= VOICE_TARGET_NAVIGATION_STATUS_STALE_TIMEOUT
            and not (status_error and error_at >= received_at)
        )
        publisher_observable = status_fresh and (
            not graph['graphProbeAvailable'] or graph['publisherObserved']
        )
        session_observed = session is not None
        session_terminal = bool(session and session['terminal'])
        session_unresolved = bool(session and not session['terminal'])

        def recovery_snapshot(reason: str) -> dict:
            source_status = session or raw_status or {}
            return {
                'state': 'unknown',
                'active': False,
                'source': 'voice_command_recognition',
                'goalId': str(source_status.get('goalId') or ''),
                'goal': source_status.get('goal'),
                'canCancel': False,
                'interlockActive': True,
                'recoveryRequired': True,
                'sessionObserved': session_observed,
                'sessionTerminal': session_terminal,
                'publisherObserved': graph['publisherObserved'],
                'statusFresh': status_fresh,
                'message': 'Voice target navigation is locked pending safe recovery.',
                'lastError': reason,
                'sourceUpdatedAtEpochMs': int(
                    source_status.get('sourceUpdatedAtEpochMs') or now_epoch_ms
                ),
                'observedAtEpochMs': now_epoch_ms,
            }

        if session_unresolved:
            same_goal = (
                raw_status is not None
                and raw_status.get('goalId') == session.get('goalId')
                and bool(session.get('goalId'))
            )
            session_uuid = str(session.get('actionUuid') or '')
            raw_uuid = str(raw_status.get('actionUuid') or '') if raw_status else ''
            same_action = not session_uuid or session_uuid == raw_uuid
            raw_is_live_same_session = bool(
                publisher_observable
                and raw_status is not None
                and raw_status.get('active') is True
                and raw_status.get('recoveryRequired') is False
                and same_goal
                and same_action
            )
            if not raw_is_live_same_session:
                return recovery_snapshot(
                    status_error
                    or session_error
                    or (raw_status or {}).get('lastError')
                    or session.get('lastError')
                    or (
                        'The persisted voice target-navigation session has not '
                        'reached a terminal state.'
                    )
                )

        if raw_status is None:
            if status_error or session_error:
                return recovery_snapshot(status_error or session_error)
            return {
                'state': 'unknown',
                'active': False,
                'source': 'unknown',
                'goalId': '',
                'goal': None,
                'canCancel': False,
                'interlockActive': False,
                'recoveryRequired': False,
                'sessionObserved': False,
                'sessionTerminal': False,
                'publisherObserved': graph['publisherObserved'],
                'statusFresh': False,
                'message': '',
                'lastError': '',
                'sourceUpdatedAtEpochMs': now_epoch_ms,
                'observedAtEpochMs': now_epoch_ms,
            }

        if (
            raw_status['recoveryRequired']
            or (raw_status['active'] and not session_observed)
            or (raw_status['active'] and not publisher_observable)
            or status_error
            or session_error
        ):
            return recovery_snapshot(
                status_error
                or session_error
                or raw_status['lastError']
                or (
                    'Voice target-navigation status cannot be safely matched to '
                    'a recoverable session.'
                )
            )

        state_name = raw_status['state']
        return {
            'state': state_name,
            'active': raw_status['active'],
            'source': 'voice_command_recognition',
            'goalId': raw_status['goalId'],
            'goal': raw_status['goal'],
            # No cancellation endpoint exists in the mini-program contract yet.
            'canCancel': False,
            'interlockActive': raw_status['interlockActive'],
            'recoveryRequired': False,
            'sessionObserved': session_observed,
            'sessionTerminal': session_terminal,
            'publisherObserved': graph['publisherObserved'],
            'statusFresh': status_fresh,
            'message': raw_status['message'],
            'lastError': raw_status['lastError'],
            'sourceUpdatedAtEpochMs': raw_status['sourceUpdatedAtEpochMs'],
            'observedAtEpochMs': now_epoch_ms,
        }

    def get_feature_status(self, *, _cache_bypass: bool = False) -> dict:
        if not _cache_bypass:
            return self.feature_status_cache.get(
                lambda: self.get_feature_status(_cache_bypass=True)
            )
        vehicle_readiness = self.get_vehicle_readiness()
        return {
            'ok': True,
            'deviceId': DEVICE_ID,
            'observedAtEpochMs': int(time.time() * 1000),
            'vehicleReadiness': vehicle_readiness,
            'bodyFollow': self.get_body_follow_feature_status(),
            'recharge': self.get_recharge_feature_status(),
            'mappingSession': self.get_mapping_feature_status(vehicle_readiness),
            'targetNavigation': self.get_target_navigation_feature_status(),
        }

    def get_pose(self) -> dict:
        now_monotonic = time.monotonic()
        safety = self._get_navigation_safety_status(now_monotonic)
        with self.lock:
            pose = copy.deepcopy(self.latest_guarded_pose)
            pose_observed = self.latest_guarded_pose_observed_monotonic
            pose_stamp_ns = self.latest_guarded_pose_stamp_ns
            odom_received = self.latest_odom_received_monotonic
            pose_error = self.latest_guarded_pose_probe_error
        try:
            now_ros_ns = int(self.node.get_clock().now().nanoseconds)
        except Exception:  # noqa: BLE001 - source freshness must fail closed
            now_ros_ns = 0
        tf_source_age_sec = source_stamp_age_sec(now_ros_ns, pose_stamp_ns)
        tf_received_age_sec = (
            now_monotonic - pose_observed if pose_observed > 0 else None
        )
        # Receipt age is diagnostic only.  A trusted dynamic map-to-base pose
        # must always have a valid ROS source stamp.
        tf_age_sec = tf_source_age_sec
        odom_age_sec = (
            now_monotonic - odom_received if odom_received > 0 else None
        )
        available = pose is not None
        held = available and not safety['safetyReady']
        override_active = safety.get('guardTrustedOverride') is True
        pose_fresh = (
            tf_age_sec is not None
            and tf_age_sec <= NAVIGATION_POSE_STALE_TIMEOUT
        )
        if not available:
            pose_state = 'unavailable'
        elif held:
            pose_state = 'held'
        elif override_active:
            pose_state = 'trusted_override'
        elif not pose_fresh:
            pose_state = 'stale'
        else:
            pose_state = 'trusted'
        trusted = available and (held or pose_fresh)
        diagnostics = {
            'poseSource': 'guarded_tf',
            'trusted': trusted,
            'available': available,
            'poseState': pose_state,
            'held': held,
            'tfAgeSec': tf_age_sec,
            'tfReceivedAgeSec': tf_received_age_sec,
            'odomAgeSec': odom_age_sec,
            'guardAnomaly': safety['guardAnomaly'],
            'interlockBlocked': safety['interlockBlocked'],
            'guardMode': safety.get('guardMode', ''),
        }
        if pose is None:
                return {
                    'ok': False,
                    'accepted': False,
                    'navigationState': 'failed',
                    'deviceId': DEVICE_ID,
                    'message': pose_error or 'Guarded robot pose is not available yet',
                    **diagnostics,
                }
        pose.update(diagnostics)
        if held:
            pose['message'] = 'Guarded robot pose is held at the last trusted value'
        elif pose_state == 'stale':
            pose['message'] = 'Guarded robot pose is stale'
        return pose

    @staticmethod
    def _decorate_navigation_state(state: dict) -> dict:
        snapshot = copy.deepcopy(state)
        status = str(snapshot.get('status') or 'idle').strip().lower()
        snapshot['schemaVersion'] = 1
        snapshot['active'] = status == 'navigating'
        snapshot['terminal'] = status in {'succeeded', 'canceled', 'failed'}
        snapshot.setdefault('superseded', False)
        return snapshot

    def _ensure_navigation_history_locked(self) -> None:
        if not hasattr(self, 'navigation_goal_history'):
            self.navigation_goal_history = OrderedDict()
        if not hasattr(self, 'navigation_goal_tombstones'):
            self.navigation_goal_tombstones = OrderedDict()

    def _remember_navigation_goal_locked(self, state: dict) -> None:
        goal_id = str(state.get('goalId') or '').strip().lower()
        if not goal_id:
            return
        self._ensure_navigation_history_locked()
        snapshot = self._decorate_navigation_state(state)
        self.navigation_goal_history.pop(goal_id, None)
        self.navigation_goal_history[goal_id] = snapshot
        while len(self.navigation_goal_history) > NAVIGATION_GOAL_HISTORY_LIMIT:
            expired_goal_id, _ = self.navigation_goal_history.popitem(last=False)
            self.navigation_goal_tombstones.pop(expired_goal_id, None)
            self.navigation_goal_tombstones[expired_goal_id] = int(time.time() * 1000)
        while len(self.navigation_goal_tombstones) > NAVIGATION_GOAL_TOMBSTONE_LIMIT:
            self.navigation_goal_tombstones.popitem(last=False)

    def _supersede_navigation_goal_locked(self, replacement_goal_id: str):
        active_goal_handle = self._active_goal_handle
        current_goal_id = str(self.navigation_state.get('goalId') or '').strip().lower()
        if (
            self.navigation_state.get('status') == 'navigating'
            and current_goal_id
        ):
            completed_at = _utc_now()
            self.navigation_state.update({
                'ok': False,
                'status': 'canceled',
                'accepted': self.navigation_state.get('accepted') is True,
                'goalResponsePending': False,
                'result': {
                    'statusCode': 5,
                    'errorCode': None,
                    'errorMsg': 'Navigation goal was superseded',
                    'completedAt': completed_at,
                    'outcome': 'superseded',
                },
                'superseded': True,
                'supersededByGoalId': replacement_goal_id,
                'message': 'Navigation goal canceled because it was superseded',
                'updatedAt': completed_at,
            })
            self._remember_navigation_goal_locked(self.navigation_state)
        return active_goal_handle

    def _navigation_goal_is_current(self, generation: int, goal_uuid: str) -> bool:
        return bool(
            goal_uuid
            and generation == self._navigation_goal_generation
            and goal_uuid == self._active_goal_uuid
        )

    def _complete_navigation_goal(
        self,
        generation: int,
        goal_uuid: str,
        *,
        status: str,
        ok: bool,
        message: str,
        status_code=None,
        error_code=None,
        error_message: str = '',
        accepted: bool = True,
    ) -> bool:
        response_future = None
        with self.navigation_lock:
            if not self._navigation_goal_is_current(generation, goal_uuid):
                return False
            response_future = self._active_goal_response_future
            self._active_goal_handle = None
            self._active_goal_uuid = ''
            self._active_goal_started_monotonic = 0.0
            self._active_goal_observed_monotonic = 0.0
            self._active_goal_response_future = None
            self.navigation_state.update({
                'ok': ok,
                'status': status,
                'accepted': accepted,
                'goalResponsePending': False,
                'result': {
                    'statusCode': status_code,
                    'errorCode': error_code,
                    'errorMsg': error_message,
                    'completedAt': _utc_now(),
                },
                'message': message,
                'updatedAt': _utc_now(),
            })
            self._remember_navigation_goal_locked(self.navigation_state)
        # A lost response must not leave an unbounded pending Future in rclpy.
        # Cancel only after releasing navigation_lock because its done callback
        # re-enters the generation check below.
        if response_future is not None and not response_future.done():
            response_future.cancel()
        return True

    def _on_navigation_action_status(self, message) -> None:
        """Track our explicit goal UUID even if a goal response is lost.

        The normal result Future remains authoritative.  This status path is a
        bounded fallback for transport races and also prevents a succeeded Nav2
        goal from remaining ``navigating`` forever in the mini-program.
        """
        for status_entry in getattr(message, 'status_list', ()):
            goal_uuid = _goal_uuid_hex(status_entry.goal_info.goal_id)
            status_code = int(status_entry.status)
            with self.navigation_lock:
                generation = self._navigation_goal_generation
                if not self._navigation_goal_is_current(generation, goal_uuid):
                    continue
                self._active_goal_observed_monotonic = time.monotonic()

            terminal = _navigation_terminal_state(status_code)
            if terminal is not None:
                status, ok, terminal_message = terminal
                self._complete_navigation_goal(
                    generation,
                    goal_uuid,
                    status=status,
                    ok=ok,
                    message=terminal_message,
                    status_code=status_code,
                    error_message='' if ok else terminal_message,
                )
                continue

            if status_code in {1, 2, 3}:
                with self.navigation_lock:
                    if not self._navigation_goal_is_current(generation, goal_uuid):
                        continue
                    self.navigation_state.update({
                        'ok': True,
                        'status': 'navigating',
                        'accepted': True,
                        'goalResponsePending': False,
                        'message': 'Navigation goal observed on action status',
                        'updatedAt': _utc_now(),
                    })

    def _expire_navigation_goal_if_needed(self) -> None:
        now_monotonic = time.monotonic()
        with self.navigation_lock:
            if self.navigation_state.get('status') != 'navigating':
                return
            generation = self._navigation_goal_generation
            goal_uuid = self._active_goal_uuid
            started = self._active_goal_started_monotonic
            observed = self._active_goal_observed_monotonic
            response_pending = self.navigation_state.get('goalResponsePending') is True
            goal_handle = self._active_goal_handle

        if not goal_uuid or started <= 0.0:
            return
        goal_age = now_monotonic - started
        if (
            response_pending
            and observed <= 0.0
            and goal_age >= NAVIGATION_GOAL_RESPONSE_TIMEOUT
        ):
            message = (
                'Navigation goal acceptance was not confirmed before the '
                f'{NAVIGATION_GOAL_RESPONSE_TIMEOUT:.1f}s timeout'
            )
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=message,
                accepted=False,
            )
            return
        # 2026-09-29: a goal nav2 ACCEPTED but that never produced action feedback
        # matched neither branch below and survived until the 900 s execution timeout;
        # that is the 'ghost navigation in progress' seen after the nav2 container died
        # right after accepting.  Fail it on the short activity clock instead.
        if (
            not response_pending
            and observed <= 0.0
            and goal_age >= NAVIGATION_GOAL_ACTIVITY_TIMEOUT
        ):
            message = (
                'Navigation goal was accepted but never produced action feedback for '
                f'{NAVIGATION_GOAL_ACTIVITY_TIMEOUT:.1f}s'
            )
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=message,
                accepted=True,
            )
            return
        if (
            observed > 0.0
            and now_monotonic - observed >= NAVIGATION_GOAL_ACTIVITY_TIMEOUT
        ):
            message = (
                'Navigation goal activity became stale for '
                f'{NAVIGATION_GOAL_ACTIVITY_TIMEOUT:.1f}s'
            )
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=message,
                accepted=True,
            )
            return
        if goal_age < NAVIGATION_GOAL_EXECUTION_TIMEOUT:
            return

        message = (
            'Navigation goal exceeded the '
            f'{NAVIGATION_GOAL_EXECUTION_TIMEOUT:.1f}s execution timeout'
        )
        completed = self._complete_navigation_goal(
            generation,
            goal_uuid,
            status='failed',
            ok=False,
            message=message,
            error_message=message,
        )
        if completed and goal_handle is not None:
            try:
                with self.ros_action_lock:
                    goal_handle.cancel_goal_async()
            except Exception:
                pass

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict:
        # FastAPI runs synchronous endpoints in a worker pool.  Serialize target
        # replacement so two simultaneous POSTs cannot interleave generations.
        with self.navigation_command_lock:
            return self._send_navigation_goal_locked(x, y, yaw)

    def _get_navigation_lifecycle_active_status(self) -> dict:
        clients = getattr(self, 'navigation_lifecycle_clients', {})
        if not NAVIGATION_REQUIRED_ACTIVE_LIFECYCLE_NODES:
            return {'lifecycleReady': True, 'lifecycleStates': {}, 'lifecycleError': ''}
        missing = [
            name
            for name in NAVIGATION_REQUIRED_ACTIVE_LIFECYCLE_NODES
            if name not in clients or not clients[name].service_is_ready()
        ]
        if missing:
            return {
                'lifecycleReady': False,
                'lifecycleStates': {},
                'lifecycleError': (
                    'Nav2 lifecycle state service is unavailable: '
                    + ', '.join(missing)
                ),
            }
        try:
            futures = {
                name: clients[name].call_async(GetState.Request())
                for name in NAVIGATION_REQUIRED_ACTIVE_LIFECYCLE_NODES
            }
            deadline = time.monotonic() + NAVIGATION_LIFECYCLE_PROBE_TIMEOUT_SEC
            while (
                any(not future.done() for future in futures.values())
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            states = {}
            for name, future in futures.items():
                if not future.done():
                    raise TimeoutError(f'{name} lifecycle state timed out')
                response = future.result()
                current_state = response.current_state
                state_id = int(getattr(current_state, 'id', 0))
                label = str(getattr(current_state, 'label', '') or '').strip().lower()
                states[name] = {'id': state_id, 'label': label}
            inactive = [
                name
                for name, state in states.items()
                if state['id'] != 3 and state['label'] != 'active'
            ]
            if inactive:
                return {
                    'lifecycleReady': False,
                    'lifecycleStates': states,
                    'lifecycleError': (
                        'Nav2 lifecycle nodes are not active: '
                        + ', '.join(inactive)
                    ),
                }
            return {
                'lifecycleReady': True,
                'lifecycleStates': states,
                'lifecycleError': '',
            }
        except Exception as exc:  # noqa: BLE001 - goal admission is fail-closed
            return {
                'lifecycleReady': False,
                'lifecycleStates': {},
                'lifecycleError': f'Nav2 lifecycle probe failed: {exc}',
            }

    def _navigation_goal_admission_status(self) -> dict:
        safety = self._get_navigation_safety_status(time.monotonic())
        if not safety['safetyReady']:
            return {
                'allowed': False,
                'reason': safety['safetyError'] or 'Navigation safety is not ready',
                **safety,
            }
        lifecycle = self._get_navigation_lifecycle_active_status()
        if not lifecycle['lifecycleReady']:
            return {
                'allowed': False,
                'reason': lifecycle['lifecycleError'],
                **safety,
                **lifecycle,
            }
        return {
            'allowed': True,
            'reason': '',
            **safety,
            **lifecycle,
        }

    def _send_navigation_goal_locked(self, x: float, y: float, yaw: float) -> dict:
        if self.navigation_client is None or RosUuid is None:
            return {
                'ok': False,
                'deviceId': DEVICE_ID,
                'message': 'Navigation action client is not available',
            }
        mapping_allowed, mapping_reason = self._mapping_allows_navigation_start(
            self.get_mapping_feature_status()
        )
        if not mapping_allowed:
            return {
                'ok': False,
                'deviceId': DEVICE_ID,
                'message': (
                    'Navigation target is blocked until Cartographer mapping is '
                    f'confirmed idle: {mapping_reason}'
                ),
            }
        target_navigation = self.get_target_navigation_feature_status()
        if target_navigation.get('interlockActive') is True:
            return {
                'ok': False,
                'deviceId': DEVICE_ID,
                'message': (
                    'Navigation target is blocked by an unresolved voice target '
                    'session: '
                    f'{target_navigation.get("lastError") or target_navigation.get("message")}'
                ),
            }
        admission = self._navigation_goal_admission_status()
        if not admission['allowed']:
            return {
                'ok': False,
                'accepted': False,
                'navigationState': 'failed',
                'deviceId': DEVICE_ID,
                'rejectionReason': admission['reason'],
                'guardAnomaly': admission.get('guardAnomaly'),
                'interlockBlocked': admission.get('interlockBlocked'),
                'guardMode': admission.get('guardMode', ''),
                'lifecycleReady': admission.get('lifecycleReady', False),
                'message': f'Navigation target is blocked: {admission["reason"]}',
            }
        with self.ros_action_lock:
            server_ready = self.navigation_client.wait_for_server(timeout_sec=2.0)
        if not server_ready:
            return {
                'ok': False,
                'deviceId': DEVICE_ID,
                'message': 'Navigation action server is not ready',
            }

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        z, w = _quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w

        goal_id = RosUuid()
        goal_id.uuid = list(uuid.uuid4().bytes)
        goal_uuid = _goal_uuid_hex(goal_id)
        goal_snapshot = {
            'frameId': 'map',
            'x': x,
            'y': y,
            'yaw': yaw,
            'sentAt': _utc_now(),
        }
        with self.navigation_lock:
            active_goal_handle = self._supersede_navigation_goal_locked(goal_uuid)
            self._navigation_goal_generation += 1
            generation = self._navigation_goal_generation
            self._active_goal_handle = None
            self._active_goal_uuid = goal_uuid
            self._active_goal_started_monotonic = time.monotonic()
            self._active_goal_observed_monotonic = 0.0
            self._active_goal_response_future = None
            self.navigation_state.update({
                'ok': True,
                'status': 'navigating',
                'goalId': goal_uuid,
                'accepted': False,
                'goalResponsePending': True,
                'goal': goal_snapshot,
                'feedback': None,
                'result': None,
                'superseded': False,
                'supersededByGoalId': '',
                'message': 'Navigation goal sent; waiting for acceptance',
                'updatedAt': _utc_now(),
            })

        if active_goal_handle is not None:
            try:
                with self.ros_action_lock:
                    active_goal_handle.cancel_goal_async()
            except Exception:
                pass

        feedback_callback = lambda feedback_msg: self._on_navigation_feedback(
            feedback_msg, generation, goal_uuid
        )
        try:
            # Keep request creation atomic with respect to spin_once().  Humble's
            # ActionClient writes first and only then records the pending sequence.
            with self.ros_action_lock:
                future = self.navigation_client.send_goal_async(
                    goal,
                    feedback_callback=feedback_callback,
                    goal_uuid=goal_id,
                )
                with self.navigation_lock:
                    if self._navigation_goal_is_current(generation, goal_uuid):
                        self._active_goal_response_future = future
                future.add_done_callback(
                    lambda completed: self._on_navigation_goal_response(
                        completed, generation, goal_uuid
                    )
                )
        except Exception as exc:
            message = f'Navigation goal failed: {exc}'
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=str(exc),
                accepted=False,
            )
            return {
                'ok': False,
                'accepted': False,
                'navigationState': 'failed',
                'deviceId': DEVICE_ID,
                'frameId': 'map',
                'x': x,
                'y': y,
                'yaw': yaw,
                'message': message,
            }

        deadline = time.monotonic() + min(3.0, NAVIGATION_GOAL_RESPONSE_TIMEOUT)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        with self.navigation_lock:
            state = dict(self.navigation_state)
        accepted = state.get('accepted') is True
        pending = state.get('goalResponsePending') is True
        return {
            'ok': state.get('ok') is True,
            'accepted': accepted,
            'pending': pending,
            'navigationState': state.get('status', 'failed'),
            'goalId': goal_uuid,
            'deviceId': DEVICE_ID,
            'frameId': 'map',
            'x': x,
            'y': y,
            'yaw': yaw,
            'message': state.get('message', ''),
        }

    def _on_navigation_goal_response(
        self,
        future,
        generation: int,
        goal_uuid: str,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            # A canceled pending Future belongs to a timeout or a status-driven
            # terminal transition that has already cleared this generation.
            with self.navigation_lock:
                current = self._navigation_goal_is_current(generation, goal_uuid)
            if not current:
                return
            message = f'Navigation goal failed: {exc}'
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=str(exc),
                accepted=False,
            )
            return

        with self.navigation_lock:
            current = self._navigation_goal_is_current(generation, goal_uuid)
        if not current:
            if goal_handle.accepted:
                try:
                    with self.ros_action_lock:
                        goal_handle.cancel_goal_async()
                except Exception:
                    pass
            return
        if not goal_handle.accepted:
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message='Navigation goal rejected',
                error_message='Navigation goal rejected',
                accepted=False,
            )
            return

        with self.navigation_lock:
            if self._navigation_goal_is_current(generation, goal_uuid):
                self._active_goal_response_future = None
                self._active_goal_observed_monotonic = time.monotonic()
                self.navigation_state.update({
                    'ok': True,
                    'status': 'navigating',
                    'accepted': True,
                    'goalResponsePending': False,
                    'message': 'Navigation goal accepted',
                    'updatedAt': _utc_now(),
                })
        self._register_navigation_result(goal_handle, generation, goal_uuid)

    def _register_navigation_result(
        self,
        goal_handle,
        generation: int,
        goal_uuid: str,
    ) -> None:
        with self.navigation_lock:
            if not self._navigation_goal_is_current(generation, goal_uuid):
                return
            self._active_goal_handle = goal_handle
        try:
            with self.ros_action_lock:
                result_future = goal_handle.get_result_async()
        except Exception as exc:
            message = f'Navigation result request failed: {exc}'
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=str(exc),
            )
            return
        result_future.add_done_callback(
            lambda completed: self._on_navigation_result(
                completed, generation, goal_uuid
            )
        )

    def _on_navigation_feedback(
        self,
        feedback_msg,
        generation: int,
        goal_uuid: str,
    ) -> None:
        if _goal_uuid_hex(feedback_msg.goal_id) != goal_uuid:
            return
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
            if not self._navigation_goal_is_current(generation, goal_uuid):
                return
            self._active_goal_observed_monotonic = time.monotonic()
            self.navigation_state.update({
                'ok': True,
                'status': 'navigating',
                'accepted': True,
                'goalResponsePending': False,
                'feedback': feedback_snapshot,
                'message': 'Navigation feedback received',
                'updatedAt': _utc_now(),
            })

    def _on_navigation_result(
        self,
        future,
        generation: int,
        goal_uuid: str,
    ) -> None:
        try:
            wrapped_result = future.result()
            status_code = int(wrapped_result.status)
            action_result = wrapped_result.result
            error_code = getattr(action_result, 'error_code', None)
            error_message = str(getattr(action_result, 'error_msg', '') or '')
        except Exception as exc:
            message = f'Navigation result failed: {exc}'
            self._complete_navigation_goal(
                generation,
                goal_uuid,
                status='failed',
                ok=False,
                message=message,
                error_message=str(exc),
            )
            return

        terminal = _navigation_terminal_state(status_code)
        if terminal is None:
            status = 'failed'
            ok = False
            message = f'Navigation failed with status {status_code}'
        else:
            status, ok, message = terminal
        self._complete_navigation_goal(
            generation,
            goal_uuid,
            status=status,
            ok=ok,
            message=message,
            status_code=status_code,
            error_code=error_code,
            error_message=error_message or ('' if ok else message),
        )

    def get_navigation_status(self, goal_id: str | None = None) -> dict:
        self._expire_navigation_goal_if_needed()
        requested_goal_id = str(goal_id or '').strip().lower()
        with self.navigation_lock:
            current = self._decorate_navigation_state(self.navigation_state)
            if not requested_goal_id:
                return current
            if str(current.get('goalId') or '').strip().lower() == requested_goal_id:
                return current
            self._ensure_navigation_history_locked()
            if requested_goal_id in self.navigation_goal_history:
                return copy.deepcopy(self.navigation_goal_history[requested_goal_id])
            if requested_goal_id in self.navigation_goal_tombstones:
                raise NavigationGoalLookupError(
                    410,
                    requested_goal_id,
                    f'Navigation goal status has expired: {requested_goal_id}',
                )
            raise NavigationGoalLookupError(
                404,
                requested_goal_id,
                f'Navigation goal was not found: {requested_goal_id}',
            )

    def _get_navigation_graph_nodes(self) -> list[str]:
        """Return only observed Nav2 node names; graph data is a startup hint.

        The graph is deliberately not used as the ready criterion.  A Nav2
        process can expose nodes before its NavigateToPose action server is
        usable, and voice control may have started it outside this process.

        Internal stacks live in their own namespaces (the map-free body-follow
        avoidance Nav2 runs at ``/nofmap``) and reuse the stock Nav2 base node
        names.  They are not the user-facing navigation feature, so they are
        excluded here; otherwise the mini program would report navigation as
        launched while the map-mode NavigateToPose server is absent.
        """
        observed = set()
        pairs_getter = getattr(self.node, 'get_node_names_and_namespaces', None)
        entries = []
        if callable(pairs_getter):
            try:
                entries = list(pairs_getter())
            except Exception:
                entries = []
        if entries:
            for item in entries:
                if not isinstance(item, (tuple, list)) or len(item) < 2:
                    continue
                name = str(item[0] or '')
                namespace = str(item[1] or '')
                if _navigation_probe_node_is_internal(name, namespace):
                    continue
                if name in NAVIGATION_GRAPH_NODE_NAMES:
                    observed.add(name)
            return sorted(observed)

        try:
            get_names = getattr(self.node, 'get_node_names', None)
            names = get_names() if callable(get_names) else []
        except Exception:
            return sorted(observed)

        for item in names:
            if isinstance(item, tuple):
                name = str(item[0] or '')
                namespace = str(item[1] or '') if len(item) > 1 else ''
            else:
                name = str(item or '')
                namespace = ''
            if _navigation_probe_node_is_internal(name, namespace):
                continue
            if name in NAVIGATION_GRAPH_NODE_NAMES:
                observed.add(name)
        return sorted(observed)
    def _get_localization_source_status(self) -> dict:
        """Observe AMCL's graph presence without invoking a ROS service.

        This deliberately avoids lifecycle RPCs in the HTTP path: some ROS
        graphs can make those service calls block during discovery.  It detects
        a real source disappearance while allowing a valid, stationary AMCL
        pose to remain usable.
        """
        graph_probe_available = False
        amcl_node_observed = False
        pose_publisher_observed = False

        try:
            get_names = getattr(self.node, 'get_node_names', None)
            if callable(get_names):
                names = get_names()
            else:
                names = self.node.get_node_names_and_namespaces()
            graph_probe_available = True
            for item in names:
                if isinstance(item, tuple):
                    name = str(item[0] or '')
                else:
                    name = str(item or '')
                if name in NAVIGATION_AMCL_NODE_NAMES:
                    amcl_node_observed = True
                    break
        except Exception:
            pass

        try:
            get_publishers = getattr(self.node, 'get_publishers_info_by_topic', None)
            if callable(get_publishers):
                pose_publisher_observed = bool(get_publishers(ROBOT_POSE_TOPIC))
                graph_probe_available = True
        except Exception:
            pass

        return {
            'localizationProbeAvailable': graph_probe_available,
            'amclNodeObserved': amcl_node_observed,
            'posePublisherObserved': pose_publisher_observed,
            'localizationSourceObserved': (
                amcl_node_observed or pose_publisher_observed
            ),
        }

    def _get_localization_transport_status(self, now_monotonic: float) -> dict:
        """Return freshness of the live sensor, odometry and TF chain.

        AMCL may hold a valid pose without republishing it, whereas these three
        streams continue to update on a healthy stationary vehicle.  The TF
        chain accepts either a direct map->base transform or the configured
        map->odom plus odom->base pair.
        """
        with self.lock:
            scan_received = self.latest_scan_received_monotonic
            scan_stamp_ns = self.latest_scan_stamp_ns
            odom_received = self.latest_odom_received_monotonic
            odom_stamp_ns = self.latest_odom_stamp_ns
            map_to_base_received = self.latest_map_to_base_received_monotonic
            map_to_odom_received = self.latest_map_to_odom_received_monotonic
            odom_to_base_received = self.latest_odom_to_base_received_monotonic

        def age(received_at: float) -> float | None:
            return (
                now_monotonic - received_at
                if received_at > 0
                else None
            )

        scan_age_sec = age(scan_received)
        odom_age_sec = age(odom_received)
        try:
            now_ros_ns = int(self.node.get_clock().now().nanoseconds)
        except Exception:  # noqa: BLE001 - absent clock must fail source freshness
            now_ros_ns = 0
        scan_source_stamp_age_sec = source_stamp_age_sec(now_ros_ns, scan_stamp_ns)
        odom_source_stamp_age_sec = source_stamp_age_sec(now_ros_ns, odom_stamp_ns)
        map_to_base_age_sec = age(map_to_base_received)
        map_to_odom_age_sec = age(map_to_odom_received)
        odom_to_base_age_sec = age(odom_to_base_received)
        scan_received_fresh = (
            scan_age_sec is not None
            and scan_age_sec <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
        )
        odom_received_fresh = (
            odom_age_sec is not None
            and odom_age_sec <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
        )
        scan_source_fresh = (
            scan_source_stamp_age_sec is not None
            and scan_source_stamp_age_sec
            <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
        )
        odom_source_fresh = (
            odom_source_stamp_age_sec is not None
            and odom_source_stamp_age_sec
            <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
        )
        scan_fresh = scan_received_fresh and scan_source_fresh
        odom_fresh = odom_received_fresh and odom_source_fresh
        direct_transform_fresh = (
            map_to_base_age_sec is not None
            and map_to_base_age_sec <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
        )
        chained_transform_fresh = (
            map_to_odom_age_sec is not None
            and map_to_odom_age_sec <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
            and odom_to_base_age_sec is not None
            and odom_to_base_age_sec <= NAVIGATION_LOCALIZATION_DATA_STALE_TIMEOUT
        )
        transform_fresh = direct_transform_fresh or chained_transform_fresh
        probe_available = all(
            subscription is not None
            for subscription in (
                self.scan_subscription,
                self.odom_subscription,
                self.tf_subscription,
            )
        )
        return {
            'localizationDataProbeAvailable': probe_available,
            'scanFresh': scan_fresh,
            'odomFresh': odom_fresh,
            'scanReceivedFresh': scan_received_fresh,
            'odomReceivedFresh': odom_received_fresh,
            'scanSourceStampFresh': scan_source_fresh,
            'odomSourceStampFresh': odom_source_fresh,
            'transformFresh': transform_fresh,
            'localizationTransportReady': (
                scan_fresh and odom_fresh and transform_fresh
            ),
            'scanAgeSec': scan_age_sec,
            'odomAgeSec': odom_age_sec,
            'scanReceivedAgeSec': scan_age_sec,
            'odomReceivedAgeSec': odom_age_sec,
            'scanSourceStampAgeSec': scan_source_stamp_age_sec,
            'odomSourceStampAgeSec': odom_source_stamp_age_sec,
            'mapToBaseTransformAgeSec': map_to_base_age_sec,
            'mapToOdomTransformAgeSec': map_to_odom_age_sec,
            'odomToBaseTransformAgeSec': odom_to_base_age_sec,
        }

    def _get_navigation_safety_status(self, now_monotonic: float) -> dict:
        with self.lock:
            guard_anomaly = self.navigation_guard_anomaly
            guard_received = self.navigation_guard_received_monotonic
            guard_mode = getattr(self, 'navigation_guard_mode', '')
            guard_mode_received = getattr(
                self, 'navigation_guard_mode_received_monotonic', 0.0
            )
            interlock_blocked = self.navigation_interlock_blocked
            interlock_received = self.navigation_interlock_received_monotonic

        guard_age_sec = (
            now_monotonic - guard_received if guard_received > 0 else None
        )
        interlock_age_sec = (
            now_monotonic - interlock_received if interlock_received > 0 else None
        )
        guard_mode_age_sec = (
            now_monotonic - guard_mode_received
            if guard_mode_received > 0
            else None
        )
        guard_status_fresh = (
            guard_age_sec is not None
            and guard_age_sec <= NAVIGATION_SAFETY_STATUS_STALE_TIMEOUT
        )
        interlock_status_fresh = (
            interlock_age_sec is not None
            and interlock_age_sec <= NAVIGATION_SAFETY_STATUS_STALE_TIMEOUT
        )
        guard_mode_fresh = (
            guard_mode_age_sec is not None
            and guard_mode_age_sec <= NAVIGATION_SAFETY_STATUS_STALE_TIMEOUT
        )
        guard_trusted_override = (
            guard_mode_fresh and guard_mode == 'TRUSTED_OVERRIDE'
        )
        # Mode is diagnostic only.  The guard must continue publishing a fresh
        # non-anomalous authority signal in TRUSTED_OVERRIDE; mode alone never
        # grants motion permission.
        guard_healthy = guard_status_fresh and guard_anomaly is False
        interlock_ready = (
            interlock_status_fresh and interlock_blocked is False
        )
        safety_error = ''
        if not guard_status_fresh:
            safety_error = 'AMCL TF guard status is missing or stale'
        elif guard_anomaly is not False:
            safety_error = 'AMCL TF guard anomaly is asserted'
        elif not interlock_status_fresh:
            safety_error = 'Navigation motion interlock status is missing or stale'
        elif interlock_blocked is not False:
            safety_error = 'Navigation motion interlock is blocked'

        return {
            'safetyProbeAvailable': (
                self.navigation_guard_subscription is not None
                and self.navigation_interlock_subscription is not None
            ),
            'safetyReady': guard_healthy and interlock_ready,
            'guardHealthy': guard_healthy,
            'guardStatusFresh': guard_status_fresh,
            'guardAnomalyStatusFresh': guard_status_fresh,
            'guardAnomaly': guard_anomaly,
            'guardStatusAgeSec': guard_age_sec,
            'guardMode': guard_mode,
            'guardModeStatusFresh': guard_mode_fresh,
            'guardModeStatusAgeSec': guard_mode_age_sec,
            'guardTrustedOverride': guard_trusted_override,
            'interlockReady': interlock_ready,
            'interlockStatusFresh': interlock_status_fresh,
            'interlockBlocked': interlock_blocked,
            'interlockStatusAgeSec': interlock_age_sec,
            'safetyError': safety_error,
        }

    def get_navigation_system_status(
        self, *, _cache_bypass: bool = False
    ) -> dict:
        """Probe Nav2 availability without issuing a goal or changing motion.

        `ready` requires a usable NavigateToPose Action Server and live
        localization transport (AMCL source, scan, odometry and map-to-base
        TF).  ``poseAvailable`` and ``poseFresh`` describe the guarded TF pose;
        raw AMCL remains diagnostic-only and cannot move the primary marker.
        This observes either voice- or
        mini-program-initiated Nav2 and is separate from an individual goal's
        status.
        """
        if not _cache_bypass:
            return self.navigation_system_status_cache.get(
                lambda: self.get_navigation_system_status(_cache_bypass=True)
            )
        now_monotonic = time.monotonic()
        now_epoch_ms = int(time.time() * 1000)
        navigation_nodes = self._get_navigation_graph_nodes()
        launch_observed = bool(navigation_nodes)

        action_server_ready = False
        action_error = ''
        if self.navigation_client is None:
            action_error = 'Navigation action client is not available'
        else:
            try:
                with self.ros_action_lock:
                    action_server_ready = bool(self.navigation_client.server_is_ready())
            except Exception as exc:
                action_error = f'Navigation action server probe failed: {exc}'

        with self.lock:
            pose_available = self.latest_guarded_pose is not None
            pose_age_sec = (
                now_monotonic - self.latest_guarded_pose_observed_monotonic
                if self.latest_guarded_pose_observed_monotonic > 0
                else None
            )
        pose_fresh = (
            pose_age_sec is not None
            and pose_age_sec <= NAVIGATION_POSE_STALE_TIMEOUT
        )

        active_observed = launch_observed or action_server_ready
        with self.navigation_system_lock:
            if active_observed:
                if self.navigation_system_observed_since is None:
                    self.navigation_system_observed_since = now_monotonic
                startup_age_sec = now_monotonic - self.navigation_system_observed_since
            else:
                self.navigation_system_observed_since = None
                startup_age_sec = 0.0

        localization_source = self._get_localization_source_status()
        localization_transport = self._get_localization_transport_status(
            now_monotonic
        )
        navigation_safety = self._get_navigation_safety_status(now_monotonic)
        localization_error = ''
        if not active_observed:
            localization_health = 'unknown'
            with self.navigation_system_lock:
                self.localization_health_missing_since = None
        else:
            source_missing = (
                localization_source['localizationProbeAvailable']
                and not localization_source['localizationSourceObserved']
            )
            transport_missing = (
                localization_transport['localizationDataProbeAvailable']
                and not localization_transport['localizationTransportReady']
            )
            pose_probe_unavailable = (
                not pose_available
                and not localization_transport['localizationDataProbeAvailable']
            )
            if (
                not source_missing
                and not transport_missing
                and not pose_probe_unavailable
            ):
                with self.navigation_system_lock:
                    self.localization_health_missing_since = None
                localization_health = 'ready'
            else:
                if source_missing:
                    localization_error = 'AMCL node and pose publisher are unavailable'
                elif transport_missing:
                    missing_signals = []
                    if not localization_transport['scanFresh']:
                        missing_signals.append('scan')
                    if not localization_transport['odomFresh']:
                        missing_signals.append('odometry')
                    if not localization_transport['transformFresh']:
                        missing_signals.append('map-to-base TF')
                    localization_error = (
                        'Localization transport is stale: '
                        + ', '.join(missing_signals)
                    )
                elif startup_age_sec < NAVIGATION_LOCALIZATION_INITIAL_TIMEOUT:
                    localization_error = 'Waiting for initial AMCL pose'
                else:
                    localization_error = (
                        'AMCL pose was not received and localization transport '
                        'is not observable'
                    )
                with self.navigation_system_lock:
                    if self.localization_health_missing_since is None:
                        self.localization_health_missing_since = now_monotonic
                    health_loss_age_sec = (
                        now_monotonic - self.localization_health_missing_since
                    )
                localization_health = (
                    'starting'
                    if health_loss_age_sec < NAVIGATION_LOCALIZATION_SOURCE_LOSS_GRACE
                    else 'failed'
                )

        # Keep poseReady as the legacy ready prerequisite consumed by existing
        # bridge/front-end code.  It means a usable live localization chain;
        # poseAvailable remains a separate diagnostic for the AMCL message
        # cache, while poseFresh reports whether the guarded TF was observed.
        pose_ready = localization_health == 'ready'

        last_error = ''
        if (
            action_server_ready
            and pose_ready
            and navigation_safety['safetyReady']
        ):
            state = 'ready'
            message = 'Navigation system is ready'
        elif navigation_system_starting_allowed(
            active_observed,
            startup_age_sec,
            NAVIGATION_SYSTEM_STARTUP_TIMEOUT,
        ):
            state = 'starting'
            message = localization_error or 'Navigation system is starting'
        elif active_observed:
            state = 'failed'
            if not action_server_ready:
                last_error = action_error or 'Navigation action server did not become ready before timeout'
            elif not navigation_safety['safetyReady']:
                last_error = navigation_safety['safetyError'] or (
                    'Navigation guard or motion interlock is not ready'
                )
            else:
                last_error = localization_error or (
                    'Navigation action server is ready but AMCL localization '
                    'is unavailable'
                )
            message = last_error
        elif action_error:
            state = 'unknown'
            last_error = action_error
            message = action_error
        else:
            state = 'stopped'
            message = 'Navigation system is not running'

        return {
            'ok': state not in {'failed', 'unknown'},
            'deviceId': DEVICE_ID,
            'state': state,
            'effectiveEnabled': state == 'ready',
            'launchObserved': launch_observed,
            'navigationNodes': navigation_nodes,
            'actionServerReady': action_server_ready,
            'poseReady': pose_ready,
            'poseAvailable': pose_available,
            'poseFresh': pose_fresh,
            'poseAgeSec': pose_age_sec,
            'localizationHealth': localization_health,
            **localization_source,
            **localization_transport,
            **navigation_safety,
            'startupAgeSec': startup_age_sec,
            'observedAt': _utc_now(),
            'observedAtEpochMs': now_epoch_ms,
            'lastError': last_error,
            'message': message,
        }

    def _loop(self) -> None:
        interval = 1.0 / PUBLISH_HZ if PUBLISH_HZ > 0 else 0.1
        while self.running:
            self._schedule_body_follow_status_probe()
            self._expire_navigation_goal_if_needed()
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
            # Sensor, odometry and TF callbacks together exceed the manual
            # publish rate. Drain a bounded callback batch so their freshness
            # reflects the live ROS graph without creating a second executor.
            for _ in range(ROS_SPIN_BATCH_SIZE):
                with self.ros_action_lock:
                    rclpy.spin_once(self.node, timeout_sec=0)
            self._refresh_base_to_imu_transform()
            self._refresh_guarded_robot_pose()
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
def get_navigation_status(goalId: Optional[str] = None) -> dict:
    if publisher is None:
        return {'ok': False, 'deviceId': DEVICE_ID, 'message': publisher_error or 'Publisher is not ready'}
    try:
        return publisher.get_navigation_status(goalId)
    except NavigationGoalLookupError as exc:
        raise HTTPException(
            status_code=exc.status_code,
            detail={
                'schemaVersion': 1,
                'ok': False,
                'deviceId': DEVICE_ID,
                'goalId': exc.goal_id,
                'message': str(exc),
            },
        ) from exc


@app.get('/api/navigation/system/status')
def get_navigation_system_status() -> dict:
    if publisher is None:
        return {
            'ok': False,
            'deviceId': DEVICE_ID,
            'state': 'unknown',
            'effectiveEnabled': False,
            'launchObserved': False,
            'actionServerReady': False,
            'poseReady': False,
            'poseAvailable': False,
            'poseFresh': False,
            'localizationHealth': 'unknown',
            'localizationProbeAvailable': False,
            'amclNodeObserved': False,
            'posePublisherObserved': False,
            'localizationSourceObserved': False,
            'localizationDataProbeAvailable': False,
            'scanFresh': False,
            'odomFresh': False,
            'transformFresh': False,
            'localizationTransportReady': False,
            'safetyProbeAvailable': False,
            'safetyReady': False,
            'guardHealthy': False,
            'guardStatusFresh': False,
            'guardAnomaly': None,
            'guardStatusAgeSec': None,
            'interlockReady': False,
            'interlockStatusFresh': False,
            'interlockBlocked': None,
            'interlockStatusAgeSec': None,
            'safetyError': publisher_error or 'robot-agent is not ready',
            'observedAt': _utc_now(),
            'observedAtEpochMs': int(time.time() * 1000),
            'lastError': publisher_error or 'robot-agent is not ready',
            'message': publisher_error or 'robot-agent is not ready',
        }
    return publisher.get_navigation_system_status()


def _queue_mapping_recovery() -> dict:
    """Bounded deferred recovery for a mapping session that is still finalizing.

    2026-09-29: the mini program calls recovery the moment the failure is reported, but
    the runner needs a few more seconds to close.  Answering FEATURE_BUSY made the app
    show '自动恢复未成功' and left the failure state in place, so the modal re-opened on
    every status poll.  Queue the request instead and apply it once the session turns
    terminal (or the map-file state disappears).
    """
    deadline = time.monotonic() + 90.0

    def _worker() -> None:
        while time.monotonic() < deadline:
            session, _error = _read_mapping_session_file()
            if session is None:
                break
            still_running = (
                (bool(session.get('sessionActive')) and not bool(session.get('terminal')))
                or (str(session.get('phase') or '') == 'starting'
                    and not bool(session.get('terminal')))
            )
            if not still_running:
                try:
                    _recover_mapping_service()
                except Exception:  # noqa: BLE001 - deferred best effort
                    pass
                return
            time.sleep(1.0)

    threading.Thread(target=_worker, daemon=True).start()
    return {
        'ok': True, 'accepted': True, 'recovering': True, 'recovered': False,
        'supported': True, 'deviceId': DEVICE_ID, 'reasonCode': 'QUEUED',
        'reasonText': ('Mapping session is still finalizing; recovery queued and will '
                       'be applied automatically.'),
        'lastError': '',
    }


@app.post('/api/feature/mapping/recover')
def recover_mapping_service(payload: MappingRecoverPayload) -> dict:
    """Restore the mapping service stack after a failed start (F-34).

    Answers accepted/recovering/recovered/supported plus a diagnostic
    reasonCode/reasonText.  A live session is never disturbed.
    """
    if payload.deviceId and payload.deviceId != DEVICE_ID:
        return {
            'ok': False, 'accepted': False, 'recovering': False,
            'recovered': False, 'supported': True, 'deviceId': DEVICE_ID,
            'reasonCode': 'FOREIGN_DEVICE',
            'reasonText': f'Unexpected deviceId: {payload.deviceId}',
            'lastError': '',
        }
    if publisher is None:
        return {
            'ok': False, 'accepted': False, 'recovering': False,
            'recovered': False, 'supported': True, 'deviceId': DEVICE_ID,
            'reasonCode': 'AGENT_NOT_READY',
            'reasonText': publisher_error or 'Publisher is not ready',
            'lastError': '',
        }
    result = _recover_mapping_service()
    if result.get('reasonCode') == 'FEATURE_BUSY':
        # still finalizing: queue instead of failing (see _queue_mapping_recovery)
        result = _queue_mapping_recovery()
    result.setdefault('deviceId', DEVICE_ID)
    return result


@app.get('/api/feature/status')
def get_feature_status() -> dict:
    """Read-only voice/mini-program shared feature state for the local bridge."""
    if publisher is None:
        return {
            'ok': False,
            'deviceId': DEVICE_ID,
            'observedAtEpochMs': int(time.time() * 1000),
            'message': publisher_error or 'robot-agent is not ready',
        }
    return publisher.get_feature_status()


@app.get('/api/manual-control/limits')
def get_manual_speed_limits() -> dict:
    """Operator-adjustable manual-control ceilings (F-36)."""
    return _manual_speed_limits_payload()


@app.post('/api/manual-control/limits')
def set_manual_speed_limits(payload: ManualSpeedLimitsPayload) -> dict:
    """Set the manual-control ceilings; the car clamps to its own hard bounds.

    The answer carries the *effective* values plus the request, so the mini
    program can tell the operator when its number was pinned to the hard ceiling.
    """
    if payload.deviceId and payload.deviceId != DEVICE_ID:
        return _manual_speed_limits_payload(
            ok=False, message=f'Unexpected deviceId: {payload.deviceId}')
    if payload.maxLinear is None and payload.maxAngular is None:
        return _manual_speed_limits_payload(
            ok=False, message='maxLinear or maxAngular is required')

    current_linear, current_angular = current_manual_speed_limits()
    requested_linear = (
        current_linear if payload.maxLinear is None else payload.maxLinear)
    requested_angular = (
        current_angular if payload.maxAngular is None else payload.maxAngular)
    linear, linear_pinned, linear_ok = _normalize_manual_limit(
        requested_linear, MANUAL_LINEAR_LIMIT_BOUNDS)
    angular, angular_pinned, angular_ok = _normalize_manual_limit(
        requested_angular, MANUAL_ANGULAR_LIMIT_BOUNDS)
    if not linear_ok:
        return _manual_speed_limits_payload(
            ok=False,
            message='maxLinear must be a positive finite number '
                    f'within {MANUAL_LINEAR_LIMIT_BOUNDS[0]}-'
                    f'{MANUAL_LINEAR_LIMIT_BOUNDS[1]}',
        )
    if not angular_ok:
        return _manual_speed_limits_payload(
            ok=False,
            message='maxAngular must be a positive finite number '
                    f'within {MANUAL_ANGULAR_LIMIT_BOUNDS[0]}-'
                    f'{MANUAL_ANGULAR_LIMIT_BOUNDS[1]}',
        )

    with _manual_limits_lock:
        _manual_limits_state['maxLinear'] = linear
        _manual_limits_state['maxAngular'] = angular
        _manual_limits_state['source'] = 'operator'
        _manual_limits_state['updatedAtEpochMs'] = int(time.time() * 1000)

    persist_error = _persist_manual_speed_limits()
    response = _manual_speed_limits_payload(
        requested={
            'maxLinear': float(requested_linear),
            'maxAngular': float(requested_angular),
        },
        clamped={
            'maxLinear': bool(linear_pinned),
            'maxAngular': bool(angular_pinned),
        },
        applied={'maxLinear': linear, 'maxAngular': angular},
        persisted=not persist_error,
    )
    if persist_error:
        response['persistError'] = persist_error
    return response


@app.post('/api/manual-control/start')
def start_manual_control(payload: ManualControlPayload) -> dict:
    if payload.deviceId != DEVICE_ID:
        return {'ok': False, 'message': f'Unexpected deviceId: {payload.deviceId}'}
    if publisher is None:
        return {'ok': False, 'message': publisher_error or 'Publisher is not ready'}

    requested_linear = (
        LINEAR_SPEED if payload.linear is None else float(payload.linear))
    requested_angular = (
        ANGULAR_SPEED if payload.angular is None else float(payload.angular))
    # F-36 / R11: the car is authoritative.  A caller that asks for more than the
    # operator's ceiling gets the ceiling, and the answer says so.
    linear, angular, clamped = clamp_manual_motion(
        requested_linear, requested_angular)
    publisher.set_motion(linear, angular)
    return {
        'ok': True,
        'deviceId': DEVICE_ID,
        'action': payload.action,
        'linear': linear,
        'angular': angular,
        'requestedLinear': requested_linear,
        'requestedAngular': requested_angular,
        'clampedByLimit': clamped,
        'limits': _manual_speed_limits_payload()['limits'],
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
