from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import re
import os
import signal
import stat
import subprocess
import threading
import time
import uuid
from urllib.error import HTTPError
from urllib.error import URLError
from urllib.request import urlopen

import paho.mqtt.client as mqtt

try:
    import fcntl
except ImportError:  # pragma: no cover - production target is Linux
    fcntl = None

BROKER_HOST = os.getenv('MQTT_BROKER_HOST', '0.0.0.0')
BROKER_PORT = int(os.getenv('MQTT_BROKER_PORT', '1883'))
DEVICE_ID = os.getenv('MQTT_DEVICE_ID', 'robot-001')
COMMAND_TOPIC = os.getenv('MQTT_COMMAND_TOPIC', f'robot/{DEVICE_ID}/launch/command')
STATUS_TOPIC = os.getenv('MQTT_STATUS_TOPIC', f'robot/{DEVICE_ID}/launch/status')
HEARTBEAT_TOPIC = os.getenv('MQTT_HEARTBEAT_TOPIC', f'robot/{DEVICE_ID}/heartbeat')
ERROR_TOPIC = os.getenv('MQTT_ERROR_TOPIC', f'robot/{DEVICE_ID}/error')
FEATURE_STATUS_TOPIC = os.getenv(
    'MQTT_FEATURE_STATUS_TOPIC', f'robot/{DEVICE_ID}/feature/status'
)
CARTOGRAPHER_TASK_PATH = os.getenv(
    'CARTOGRAPHER_TASK_PATH',
    os.path.expanduser('~/.local/state/stemm-cartographer-task.json'),
)
# 2026-10-09: 建图会话状态文件的权威来源。bridge 需要 terminationReason 才能区分
# "返航成功收尾" 与 "用户取消 / 设备重启 / 启动超时" —— 归一化后的 mappingSession
# 白名单里没有这个字段（只有 reasonCode=MAPPING_SUCCEEDED，无法区分后三者）。
CARTOGRAPHER_SESSION_STATUS_PATH = os.getenv(
    'CARTOGRAPHER_SESSION_STATUS_PATH',
    os.path.expanduser('~/.local/state/stemm-cartographer-session.json'),
)
# 自动拉起导航：总开关与触发原因白名单（可用环境变量覆盖，便于现场关停）
AUTO_NAVIGATION_AFTER_MAPPING = os.getenv(
    'MQTT_AUTO_NAVIGATION_AFTER_MAPPING', '1').strip().lower() not in {
        '0', 'false', 'no', 'off'}
AUTO_NAVIGATION_TRIGGER_REASONS = tuple(
    item.strip()
    for item in os.getenv(
        'MQTT_AUTO_NAVIGATION_TRIGGER_REASONS', 'return_home_complete',
    ).split(',')
    if item.strip()
)
# 单次尝试，不重试（用户决定）。重新武装只可能由"新一场建图会话"触发。
_auto_navigation_state = {'taskId': '', 'firedTaskId': ''}
_auto_navigation_lock = threading.Lock()
HEARTBEAT_INTERVAL = float(os.getenv('MQTT_HEARTBEAT_INTERVAL', '5'))
SERVICE_CALL_TIMEOUT = float(os.getenv('MQTT_SERVICE_CALL_TIMEOUT', '30'))
NAVIGATION_SYSTEM_STATUS_URL = os.getenv(
    'MQTT_NAVIGATION_SYSTEM_STATUS_URL',
    'http://127.0.0.1:8001/api/navigation/system/status',
).strip()
NAVIGATION_SYSTEM_STATUS_TIMEOUT = max(
    0.2, float(os.getenv('MQTT_NAVIGATION_SYSTEM_STATUS_TIMEOUT', '1.0'))
)
FEATURE_STATUS_URL = os.getenv(
    'MQTT_FEATURE_STATUS_URL',
    'http://127.0.0.1:8001/api/feature/status',
).strip()
FEATURE_STATUS_TIMEOUT = max(
    0.2, float(os.getenv('MQTT_FEATURE_STATUS_TIMEOUT', '1.0'))
)
# Voice commands bypass this MQTT command bridge, so the bridge keeps a bounded
# observation path for their state changes.  Idle truth does not need a 2 Hz
# ROS-graph probe; transitions use a one-second cadence and explicit command or
# health-monitor events still force immediate retained publications.
NAVIGATION_FEATURE_STATUS_INTERVAL = max(
    0.25, float(os.getenv('MQTT_NAVIGATION_FEATURE_STATUS_INTERVAL', '2.0'))
)
NAVIGATION_FEATURE_STATUS_ACTIVE_INTERVAL = max(
    0.25,
    min(
        NAVIGATION_FEATURE_STATUS_INTERVAL,
        float(os.getenv('MQTT_NAVIGATION_FEATURE_STATUS_ACTIVE_INTERVAL', '1.0')),
    ),
)
NAVIGATION_SYSTEM_PROBE_CACHE_TTL = max(
    0.05,
    float(os.getenv('MQTT_NAVIGATION_SYSTEM_PROBE_CACHE_TTL', '0.5')),
)
NAVIGATION_SYSTEM_STARTUP_TIMEOUT = min(
    120.0,
    max(
        1.0,
        float(os.getenv('MQTT_NAVIGATION_SYSTEM_STARTUP_TIMEOUT', '120.0')),
    ),
)
NAVIGATION_HEALTH_POLL_INTERVAL = max(
    0.25, float(os.getenv('MQTT_NAVIGATION_HEALTH_POLL_INTERVAL', '0.5'))
)
NAVIGATION_HEALTH_STABLE_SEC = max(
    0.5, float(os.getenv('MQTT_NAVIGATION_HEALTH_STABLE_SEC', '1.5'))
)
# This path is deliberately not configurable independently from the voice
# bridge.  A split-brain lock is worse than refusing to start: it permits two
# launch roots to own Nav2 at the same time.
NAVIGATION_LAUNCH_LOCK_PATH = '/tmp/wheeltec_nav2_launch.lock'


def bridge_singleton_lock_path(device_id: str, command_topic: str) -> str:
    """Return a stable per-role lock path without embedding MQTT path text."""
    role_key = f'{device_id}\n{command_topic}'.encode('utf-8')
    digest = hashlib.sha256(role_key).hexdigest()[:16]
    return f'/tmp/wheeltec_launch_mqtt_bridge_{digest}.lock'


BRIDGE_SINGLETON_LOCK_PATH = bridge_singleton_lock_path(
    DEVICE_ID, COMMAND_TOPIC
)
NAVIGATION_SIGKILL = getattr(signal, 'SIGKILL', 9)

# 2026-10-06: 导航 launch 包装的信号屏蔽库（与容器同一份）。
NAV_SIGNAL_GUARD_PATH = os.getenv(
    'NAV_SIGNAL_GUARD_PATH',
    '/usr/local/lib/nav_signal_guard/nav_signal_guard.so',
)
NAVIGATION_LIFECYCLE_SHUTDOWN_TIMEOUT = max(
    0.5,
    float(os.getenv('MQTT_NAVIGATION_LIFECYCLE_SHUTDOWN_TIMEOUT', '3.0')),
)
# 2026-09-17: user-approved fast path.  The per-node lifecycle shutdown cost up to
# len(services) x NAVIGATION_LIFECYCLE_SHUTDOWN_TIMEOUT (measured ~20 s end to end),
# while the whole launch process group is torn down by SIGINT anyway.  Default: skip
# the lifecycle shutdown and kill the group directly.  Set MQTT_NAVIGATION_STOP_FAST=0
# to restore the graceful per-node shutdown.
NAVIGATION_STOP_FAST_PATH = os.getenv(
    'MQTT_NAVIGATION_STOP_FAST', '1').strip().lower() not in {'0', 'false', 'no', 'off'}
# 2026-09-29 (stop-path crash workaround): the navigation container SIGSEGVs while it
# tears down the RotationShim/MPPI controller and the layered costmap after SIGINT
# (three cores, identical faulting instruction).  SIGKILL skips that destructor path.
# Default keeps the graceful ladder; set MQTT_NAVIGATION_STOP_SIGKILL_ONLY=1 to opt in.
NAVIGATION_STOP_SIGKILL_ONLY = os.getenv(
    'MQTT_NAVIGATION_STOP_SIGKILL_ONLY', '0').strip().lower() in {'1', 'true', 'yes', 'on'}
# 2026-09-29 (方案1): kill the composable containers first so they never receive a
# forwardable SIGINT/SIGTERM (which would run the crashing teardown).  Default on;
# set MQTT_NAVIGATION_STOP_CONTAINER_FIRST=0 to restore the plain ladder.
NAVIGATION_STOP_CONTAINER_FIRST = os.getenv(
    'MQTT_NAVIGATION_STOP_CONTAINER_FIRST', '1').strip().lower() not in {'0', 'false', 'no', 'off'}
NAVIGATION_STOP_SIGINT_TIMEOUT = max(
    0.5, float(os.getenv('MQTT_NAVIGATION_STOP_SIGINT_TIMEOUT', '10.0'))
)
NAVIGATION_STOP_SIGTERM_TIMEOUT = max(
    0.5, float(os.getenv('MQTT_NAVIGATION_STOP_SIGTERM_TIMEOUT', '5.0'))
)
NAVIGATION_STOP_SIGKILL_TIMEOUT = max(
    0.5, float(os.getenv('MQTT_NAVIGATION_STOP_SIGKILL_TIMEOUT', '5.0'))
)
NAVIGATION_LIFECYCLE_MANAGER_SERVICES = tuple(
    item.strip()
    for item in os.getenv(
        'MQTT_NAVIGATION_LIFECYCLE_MANAGER_SERVICES',
        '/lifecycle_manager_navigation/manage_nodes,'
        '/lifecycle_manager_localization/manage_nodes',
    ).split(',')
    if item.strip()
)
DEFAULT_PACKAGE = os.getenv('MQTT_LAUNCH_PACKAGE', 'wheeltec_nav2')
DEFAULT_LAUNCH_FILE = os.getenv('MQTT_LAUNCH_FILE', 'wheeltec_nav2.launch.py')
DEFAULT_NODE_PACKAGE = os.getenv('MQTT_NODE_PACKAGE', 'auto_recharge_ros2')
DEFAULT_NODE_EXECUTABLE = os.getenv('MQTT_NODE_EXECUTABLE', 'navigate_to_charger')


def build_target_key(package_name: str, entry_name: str) -> str:
    return f'{package_name}:{entry_name}'


def navigation_lock_stat_is_secure(lock_stat: object, effective_uid: int) -> bool:
    """Return whether an already-open lock inode is safe for this process."""
    return (
        stat.S_ISREG(lock_stat.st_mode)
        and lock_stat.st_uid == effective_uid
        and stat.S_IMODE(lock_stat.st_mode) == 0o600
        and lock_stat.st_nlink == 1
    )


def _open_navigation_launch_lock() -> tuple[object | None, str]:
    """Open the fixed lock without following links or blocking on a FIFO."""
    if not hasattr(os, 'O_NOFOLLOW'):
        return None, 'Secure O_NOFOLLOW file locking is unavailable.'
    flags = (
        os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
    )
    fd = None
    try:
        fd = os.open(NAVIGATION_LAUNCH_LOCK_PATH, flags, 0o600)
        effective_uid = os.geteuid()
        opened_stat = os.fstat(fd)
        if not stat.S_ISREG(opened_stat.st_mode):
            raise OSError('navigation lock is not a regular file')
        if opened_stat.st_uid != effective_uid:
            raise OSError(
                'navigation lock is owned by uid %s, expected uid %s'
                % (opened_stat.st_uid, effective_uid)
            )
        if opened_stat.st_nlink != 1:
            raise OSError('navigation lock has unexpected hard links')
        os.fchmod(fd, 0o600)
        opened_stat = os.fstat(fd)
        path_stat = os.stat(NAVIGATION_LAUNCH_LOCK_PATH, follow_symlinks=False)
        if not navigation_lock_stat_is_secure(opened_stat, effective_uid):
            raise OSError('navigation lock permissions are not 0600')
        if (
            not stat.S_ISREG(path_stat.st_mode)
            or path_stat.st_uid != effective_uid
            or path_stat.st_nlink != 1
            or path_stat.st_dev != opened_stat.st_dev
            or path_stat.st_ino != opened_stat.st_ino
        ):
            raise OSError('navigation lock path changed while it was opened')
        return os.fdopen(fd, 'r+', encoding='utf-8', closefd=True), ''
    except OSError as exc:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        return None, f'Cannot securely open navigation launch lock: {exc}'


def acquire_navigation_launch_lock(owner: str) -> tuple[object | None, str]:
    """Atomically reserve the one supported Nav2 launch across voice and MQTT."""
    if fcntl is None:
        return None, 'POSIX file locking is unavailable.'
    handle, open_error = _open_navigation_launch_lock()
    if handle is None:
        return None, open_error
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        locked_stat = os.fstat(handle.fileno())
        path_stat = os.stat(NAVIGATION_LAUNCH_LOCK_PATH, follow_symlinks=False)
        if (
            not navigation_lock_stat_is_secure(locked_stat, os.geteuid())
            or path_stat.st_dev != locked_stat.st_dev
            or path_stat.st_ino != locked_stat.st_ino
        ):
            raise OSError('navigation lock changed before metadata update')
        handle.seek(0)
        handle.truncate()
        handle.write(
            json.dumps({
                'owner': owner,
                'pid': os.getpid(),
                'acquiredAt': time.strftime('%Y-%m-%dT%H:%M:%S'),
            }) + '\n'
        )
        handle.flush()
        return handle, ''
    except BlockingIOError:
        handle.close()
        return None, 'Navigation startup is already owned by another launcher.'
    except OSError as exc:
        handle.close()
        return None, f'Cannot acquire navigation launch lock: {exc}'


def release_navigation_launch_lock(handle: object | None) -> None:
    if handle is None:
        return
    try:
        if fcntl is not None:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    except (AttributeError, OSError):
        pass
    try:
        handle.close()
    except (AttributeError, OSError):
        pass


def _open_bridge_singleton_lock() -> tuple[object | None, str]:
    """Open the process singleton without following links or blocking on a FIFO."""
    if not hasattr(os, 'O_NOFOLLOW'):
        return None, 'Secure O_NOFOLLOW file locking is unavailable.'
    flags = (
        os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
    )
    fd = None
    try:
        fd = os.open(BRIDGE_SINGLETON_LOCK_PATH, flags, 0o600)
        effective_uid = os.geteuid()
        opened_stat = os.fstat(fd)
        if not stat.S_ISREG(opened_stat.st_mode):
            raise OSError('bridge singleton lock is not a regular file')
        if opened_stat.st_uid != effective_uid:
            raise OSError(
                'bridge singleton lock is owned by uid %s, expected uid %s'
                % (opened_stat.st_uid, effective_uid)
            )
        if opened_stat.st_nlink != 1:
            raise OSError('bridge singleton lock has unexpected hard links')
        os.fchmod(fd, 0o600)
        opened_stat = os.fstat(fd)
        path_stat = os.stat(BRIDGE_SINGLETON_LOCK_PATH, follow_symlinks=False)
        if not navigation_lock_stat_is_secure(opened_stat, effective_uid):
            raise OSError('bridge singleton lock permissions are not 0600')
        if (
            not stat.S_ISREG(path_stat.st_mode)
            or path_stat.st_uid != effective_uid
            or path_stat.st_nlink != 1
            or path_stat.st_dev != opened_stat.st_dev
            or path_stat.st_ino != opened_stat.st_ino
        ):
            raise OSError('bridge singleton lock path changed while it was opened')
        return os.fdopen(fd, 'r+', encoding='utf-8', closefd=True), ''
    except OSError as exc:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        return None, f'Cannot securely open bridge singleton lock: {exc}'


def acquire_bridge_singleton_lock() -> tuple[object | None, str]:
    """Reserve the sole bridge allowed to poll and publish global feature truth."""
    if fcntl is None:
        return None, 'POSIX file locking is unavailable.'
    handle, open_error = _open_bridge_singleton_lock()
    if handle is None:
        return None, open_error
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        locked_stat = os.fstat(handle.fileno())
        path_stat = os.stat(BRIDGE_SINGLETON_LOCK_PATH, follow_symlinks=False)
        if (
            not navigation_lock_stat_is_secure(locked_stat, os.geteuid())
            or path_stat.st_dev != locked_stat.st_dev
            or path_stat.st_ino != locked_stat.st_ino
        ):
            raise OSError('bridge singleton lock changed before metadata update')
        handle.seek(0)
        handle.truncate()
        handle.write(json.dumps({
            'deviceId': DEVICE_ID,
            'pid': os.getpid(),
            'acquiredAt': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }) + '\n')
        handle.flush()
        return handle, ''
    except BlockingIOError:
        handle.close()
        return None, 'Another launch MQTT bridge already owns feature polling.'
    except OSError as exc:
        handle.close()
        return None, f'Cannot acquire bridge singleton lock: {exc}'


def release_bridge_singleton_lock(handle: object | None) -> None:
    release_navigation_launch_lock(handle)


def read_current_boot_id() -> str:
    try:
        with open('/proc/sys/kernel/random/boot_id', 'r', encoding='utf-8') as handle:
            boot_id = handle.read().strip()
        if boot_id:
            return boot_id
    except OSError:
        pass
    return f'bridge-{os.getpid()}'


def persist_cartographer_task_id(request_id: str) -> None:
    os.makedirs(os.path.dirname(CARTOGRAPHER_TASK_PATH), exist_ok=True)
    temporary_path = CARTOGRAPHER_TASK_PATH + '.tmp'
    with open(temporary_path, 'w', encoding='utf-8') as handle:
        json.dump({'taskId': request_id, 'updatedAt': time.time()}, handle)
    os.replace(temporary_path, CARTOGRAPHER_TASK_PATH)


def parse_allowed_targets(raw_targets: str, fallback_targets: list[str]) -> set[str]:
    source = raw_targets or ','.join(fallback_targets)
    return {
        item.strip()
        for item in source.split(',')
        if item.strip()
    }


ALLOWED_LAUNCH_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ALLOWED_LAUNCH_TARGETS', ''),
    [
        build_target_key('wheeltec_mic_ros2', 'mic_init.launch.py'),
        build_target_key('wheeltec_mic_ros2', 'base.launch.py'),
        # The mini program starts mapping with action=start_launch on this target.
        build_target_key('stemm_cartographer_exploration', 'stemm_cartographer_session.launch.py'),
        build_target_key('stemm_nav2_manager', 'stemm_21_auto_mapping.launch.py'),
        build_target_key('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch.py'),
        build_target_key('bodyreader', 'bodyfollow_no_avoider.launch.py'),
        build_target_key('wheeltec_nav2', 'wheeltec_nav2.launch.py'),
        build_target_key('turn_on_wheeltec_robot', 'wheeltec_camera.launch.py'),
        build_target_key('cloud_video_pusher', 'rtmp_pusher.launch.py'),
        build_target_key('cloud_video_pusher', 'local_hls_server.launch.py'),
    ],
)
VIDEO_LAUNCH_TARGETS = {
    build_target_key('cloud_video_pusher', 'rtmp_pusher.launch.py'),
    build_target_key('cloud_video_pusher', 'local_hls_server.launch.py'),
}
LOCAL_HLS_LAUNCH_TARGET = build_target_key('cloud_video_pusher', 'local_hls_server.launch.py')
RTMP_LAUNCH_TARGET = build_target_key('cloud_video_pusher', 'rtmp_pusher.launch.py')
NAVIGATION_LAUNCH_TARGET = build_target_key('wheeltec_nav2', 'wheeltec_nav2.launch.py')
_navigation_feature_status_enabled = os.getenv(
    'MQTT_NAVIGATION_FEATURE_STATUS_ENABLED', ''
).strip().lower()
NAVIGATION_FEATURE_STATUS_ENABLED = (
    NAVIGATION_LAUNCH_TARGET in ALLOWED_LAUNCH_TARGETS
    if not _navigation_feature_status_enabled
    else _navigation_feature_status_enabled not in {'0', 'false', 'no', 'off'}
)
# The local MJPEG producer is the car-side source used by /stream_session and
# the cloud video relay.  It is intentionally independent of individual
# viewer leases, so a bridge restart must restore it even when no new viewer
# launch command arrives.
AUTO_START_LOCAL_HLS = os.getenv('MQTT_AUTO_START_LOCAL_HLS', 'true').strip().lower() not in {
    '0', 'false', 'no', 'off',
}
LOCAL_HLS_PRODUCER_SESSION_ID = os.getenv(
    'MQTT_LOCAL_HLS_PRODUCER_SESSION_ID', f'{DEVICE_ID}-local-preview',
).strip()
LOCAL_HLS_RESTART_DELAY = max(1.0, float(os.getenv('MQTT_LOCAL_HLS_RESTART_DELAY', '3')))
# Persistent cloud publishing is opt-in.  When enabled for a vehicle, the
# publisher remains owned by the bridge rather than by any individual viewer.
AUTO_START_RTMP = os.getenv('MQTT_AUTO_START_RTMP', 'false').strip().lower() not in {
    '0', 'false', 'no', 'off',
}
RTMP_PRODUCER_SESSION_ID = os.getenv(
    'MQTT_RTMP_PRODUCER_SESSION_ID', f'{DEVICE_ID}-always-on-cloud',
).strip()
RTMP_RESTART_DELAY = max(1.0, float(os.getenv('MQTT_RTMP_RESTART_DELAY', '3')))

BODY_FOLLOW_STATUS_SERVICE_PATH = '/voice_summon/get_body_follow_status'
BODY_FOLLOW_STATUS_SERVICE_TYPE = 'std_srvs/srv/Trigger'
# A voice-owned Nav2 launch has a deliberately narrow, auditable process
# identity.  The bridge may only stop this exact independent process group;
# it must never infer permission from a generic `wheeltec_nav2` process name.
VOICE_NAVIGATION_PARENT_MARKER = os.getenv(
    'MQTT_VOICE_NAVIGATION_PARENT_MARKER', 'voice_recharge_bridge'
).strip()
NAVIGATION_STOP_TRANSITION_TIMEOUT = max(
    (
        len(NAVIGATION_LIFECYCLE_MANAGER_SERVICES)
        * NAVIGATION_LIFECYCLE_SHUTDOWN_TIMEOUT
        + NAVIGATION_STOP_SIGINT_TIMEOUT
        + NAVIGATION_STOP_SIGTERM_TIMEOUT
        + NAVIGATION_STOP_SIGKILL_TIMEOUT
    ),
    float(os.getenv('MQTT_NAVIGATION_STOP_TRANSITION_TIMEOUT', '30.0')),
)
FEATURE_STATUS_SCHEMA_VERSION = 1
# 2026-09-16: a strictly-increasing-revision consumer (the mini program) can pin
# itself to a stale frame when a frame is dropped, and then only recovers on a page
# re-entry.  Bumping the revision slowly even without a semantic change gives every
# consumer a bounded (<= this interval) self-healing window.
try:
    FEATURE_STATUS_REVISION_HEARTBEAT_SEC = max(
        0.0,
        float(os.getenv('MQTT_FEATURE_STATUS_REVISION_HEARTBEAT_SEC', '30')),
    )
except (TypeError, ValueError):
    FEATURE_STATUS_REVISION_HEARTBEAT_SEC = 30.0
# The retained status revision intentionally restarts from 1 whenever the
# bridge service restarts.  Therefore this needs to identify the publisher
# process generation, not merely the Linux kernel boot: otherwise consumers
# would reject every new low revision from a restarted bridge as stale.
BRIDGE_BOOT_ID = f'{read_current_boot_id()}-{uuid.uuid4().hex}'


def parse_body_follow_status(details: str) -> tuple[dict | None, str]:
    message_match = re.search(
        r"\bmessage=(?P<literal>'(?:\\.|[^'\\])*'|\"(?:\\.|[^\"\\])*\")",
        details,
        flags=re.DOTALL,
    )
    if not message_match:
        return None, 'ROS Trigger response does not contain a quoted message field.'

    try:
        message = ast.literal_eval(message_match.group('literal'))
    except (SyntaxError, ValueError) as error:
        return None, f'ROS Trigger message literal is invalid: {error}'

    if not isinstance(message, str):
        return None, 'ROS Trigger message is not a string.'

    try:
        payload = json.loads(message)
    except json.JSONDecodeError as error:
        return None, f'ROS Trigger message is not JSON: {error.msg}'

    if not isinstance(payload, dict):
        return None, 'Body-follow status JSON must be an object.'

    enabled = payload.get('enabled')
    state_name = payload.get('state')
    controller_applied = payload.get('controllerApplied')
    pending = payload.get('pending')
    subscriber_count = payload.get('modeSubscriberCount')
    last_error = payload.get('lastError')

    if (
        type(enabled) is not bool
        or not isinstance(state_name, str)
        or type(controller_applied) is not bool
        or type(pending) is not bool
        or type(subscriber_count) is not int
        or subscriber_count < 0
        or not isinstance(last_error, str)
    ):
        return None, 'Body-follow status JSON has invalid field types.'

    return {
        'enabled': enabled,
        'state': state_name,
        'controllerApplied': controller_applied,
        'pending': pending,
        'modeSubscriberCount': subscriber_count,
        'lastError': last_error,
    }, ''


def get_body_follow_status_metadata(service_path: str, service_type: str, details: str) -> dict:
    if (
        service_path != BODY_FOLLOW_STATUS_SERVICE_PATH
        or service_type != BODY_FOLLOW_STATUS_SERVICE_TYPE
    ):
        return {}

    follow_status, follow_status_error = parse_body_follow_status(details)
    return {
        'followStatus': follow_status,
        'followStatusError': follow_status_error,
    }


def build_unknown_body_follow_feature(last_error: str) -> dict:
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
        'lastError': last_error,
        'observedAtEpochMs': int(time.time() * 1000),
    }


def build_unknown_recharge_feature(last_error: str) -> dict:
    return {
        'state': 'unknown',
        'effectiveEnabled': False,
        'canEnable': False,
        'canDisable': False,
        'source': 'recharge_manager',
        'managerStatus': 'unknown',
        'graphProbeAvailable': False,
        'managerNodeObserved': False,
        'statusPublisherObserved': False,
        'statusSourceObserved': False,
        'mappingMotionBlocked': False,
        'mappingGateReceived': False,
        'chargingSignalFresh': False,
        'chargingConfirmed': False,
        'chargingCurrentAmps': None,
        'chargingCurrentFresh': False,
        'lastResult': '',
        'lastError': last_error,
        'actionBlockedReason': last_error,
        'observedAtEpochMs': int(time.time() * 1000),
    }


def build_unknown_mapping_feature(last_error: str) -> dict:
    return {
        'state': 'unknown',
        'effectiveEnabled': False,
        'source': 'unknown',
        'taskId': '',
        'phase': 'unknown',
        'voiceState': 'unknown',
        'voiceStateObserved': False,
        'voiceStatePublisherObserved': False,
        'sessionObserved': False,
        'sessionFresh': False,
        'sessionActive': False,
        'terminal': False,
        'functionalServicesRestored': False,
        'motionBlocked': False,
        'paused': False,
        'mappingDone': False,
        'finalMapSaved': False,
        'actionAllowed': {
            'start': False,
            'pause': False,
            'resume': False,
            'finish': False,
        },
        'lastError': last_error,
        'actionBlockedReason': last_error,
        'sourceUpdatedAtEpochMs': int(time.time() * 1000),
    }


def build_unknown_vehicle_readiness(last_error: str) -> dict:
    return {
        'state': 'unknown',
        'mappingStartAllowed': False,
        'message': '车端初始化状态暂不可用，暂不可开始建图',
        'baseServiceActive': False,
        'serviceProbeError': last_error,
        'scanFresh': False,
        'odomFresh': False,
        'odomAdvancing': False,
        'imuFresh': False,
        'baseToImuTfObserved': False,
        'odomToBaseTfFresh': False,
        'odomToBaseTfAdvancing': False,
        'stableSinceEpochMs': 0,
        'readySinceEpochMs': 0,
        'startedAtEpochMs': 0,
        'observedAtEpochMs': int(time.time() * 1000),
    }


def build_unknown_target_navigation_feature(last_error: str) -> dict:
    return {
        'state': 'unknown',
        'active': False,
        'interlockActive': True,
        'recoveryRequired': True,
        'sessionObserved': False,
        'sessionTerminal': False,
        'source': 'unknown',
        'goalId': '',
        'goal': None,
        'canCancel': False,
        'publisherObserved': False,
        'statusFresh': False,
        'message': '',
        'lastError': last_error,
        'sourceUpdatedAtEpochMs': int(time.time() * 1000),
    }


def normalize_body_follow_feature(payload: object) -> tuple[dict | None, str]:
    if not isinstance(payload, dict):
        return None, 'Body-follow feature observation must be an object.'
    state_name = str(payload.get('state') or '').strip().lower()
    expected_states = {'stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'}
    required_booleans = (
        'effectiveEnabled',
        'canEnable',
        'canDisable',
        'requestedEnabled',
        'controllerApplied',
        'pending',
    )
    if (
        state_name not in expected_states
        or payload.get('source') != 'ros_service'
        or any(type(payload.get(key)) is not bool for key in required_booleans)
        or type(payload.get('modeSubscriberCount')) is not int
        or payload.get('modeSubscriberCount') < 0
        or not isinstance(payload.get('controllerState'), str)
        or not isinstance(payload.get('lastError'), str)
    ):
        return None, 'Body-follow feature observation fields are invalid.'
    if (
        (state_name == 'ready') != (payload['effectiveEnabled'] is True)
        or (payload['canEnable'] and state_name != 'stopped')
        or (payload['canDisable'] and state_name != 'ready')
    ):
        return None, 'Body-follow feature observation has inconsistent state.'
    return {
        key: payload[key]
        for key in (
            'state',
            'effectiveEnabled',
            'canEnable',
            'canDisable',
            'source',
            'requestedEnabled',
            'controllerApplied',
            'pending',
            'modeSubscriberCount',
            'controllerState',
            'lastError',
        )
    }, ''


def normalize_recharge_feature(payload: object) -> tuple[dict | None, str]:
    if not isinstance(payload, dict):
        return None, 'Recharge feature observation must be an object.'
    state_name = str(payload.get('state') or '').strip().lower()
    expected_states = {'stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'}
    required_booleans = (
        'effectiveEnabled',
        'canEnable',
        'canDisable',
        'graphProbeAvailable',
        'managerNodeObserved',
        'statusPublisherObserved',
        'statusSourceObserved',
        'mappingMotionBlocked',
        'mappingGateReceived',
        'chargingSignalFresh',
        'chargingConfirmed',
        'chargingCurrentFresh',
    )
    current = payload.get('chargingCurrentAmps')
    if (
        state_name not in expected_states
        or payload.get('source') != 'recharge_manager'
        or any(type(payload.get(key)) is not bool for key in required_booleans)
        or not isinstance(payload.get('managerStatus'), str)
        or not isinstance(payload.get('lastResult'), str)
        or not isinstance(payload.get('lastError'), str)
        or not isinstance(payload.get('actionBlockedReason'), str)
        or (current is not None and not isinstance(current, (int, float)))
    ):
        return None, 'Recharge feature observation fields are invalid.'
    if (
        (state_name == 'ready') != (payload['effectiveEnabled'] is True)
        or (payload['canEnable'] and state_name not in {'stopped', 'failed'})
        or (payload['canDisable'] and state_name not in {'starting', 'ready'})
    ):
        return None, 'Recharge feature observation has inconsistent state.'
    return {
        key: payload[key]
        for key in (
            'state',
            'effectiveEnabled',
            'canEnable',
            'canDisable',
            'source',
            'managerStatus',
            'graphProbeAvailable',
            'managerNodeObserved',
            'statusPublisherObserved',
            'statusSourceObserved',
            'mappingMotionBlocked',
            'mappingGateReceived',
            'chargingSignalFresh',
            'chargingConfirmed',
            'chargingCurrentAmps',
            'chargingCurrentFresh',
            'lastResult',
            'lastError',
            'actionBlockedReason',
        )
    }, ''


def normalize_mapping_feature(payload: object) -> tuple[dict | None, str]:
    """Validate the combined Cartographer session/voice observation.

    The field is optional during a rolling deployment.  Its absence is not
    equivalent to ``stopped`` and must not change the old client behaviour.
    """
    if payload is None:
        return None, ''
    if not isinstance(payload, dict):
        return None, 'Mapping feature observation must be an object.'
    state_name = str(payload.get('state') or '').strip().lower()
    source = str(payload.get('source') or '').strip().lower()
    voice_state = str(payload.get('voiceState') or '').strip().upper()
    action_allowed = payload.get('actionAllowed')
    allowed_states = {'stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'}
    required_booleans = (
        'effectiveEnabled',
        'voiceStateObserved',
        'voiceStatePublisherObserved',
        'sessionObserved',
        'sessionFresh',
        'sessionActive',
        'terminal',
        'functionalServicesRestored',
        'motionBlocked',
        'paused',
        'mappingDone',
        'finalMapSaved',
    )
    if (
        state_name not in allowed_states
        or source not in {
            'combined', 'cartographer_session', 'voice_cartographer_bridge', 'unknown',
        }
        or voice_state not in {
            'IDLE', 'STARTING', 'RUNNING', 'PAUSED', 'ENDING', 'UNKNOWN',
        }
        or any(type(payload.get(key)) is not bool for key in required_booleans)
        or not isinstance(payload.get('taskId'), str)
        or not isinstance(payload.get('phase'), str)
        or not isinstance(payload.get('lastError'), str)
        or not isinstance(payload.get('actionBlockedReason'), str)
        or type(payload.get('sourceUpdatedAtEpochMs')) is not int
        or payload.get('sourceUpdatedAtEpochMs') <= 0
        or not isinstance(action_allowed, dict)
        or any(
            type(action_allowed.get(key)) is not bool
            for key in ('start', 'pause', 'resume', 'finish')
        )
    ):
        return None, 'Mapping feature observation fields are invalid.'
    if (
        (state_name == 'ready') != (payload['effectiveEnabled'] is True)
        or (action_allowed['start'] and state_name not in {'stopped', 'failed'})
        or (action_allowed['pause'] and (state_name != 'ready' or payload['paused']))
        or (action_allowed['resume'] and (state_name != 'ready' or not payload['paused']))
        or (action_allowed['finish'] and state_name != 'ready')
    ):
        return None, 'Mapping feature observation has inconsistent state.'
    if source == 'unknown' and any(action_allowed.values()):
        return None, 'Unknown mapping source cannot grant an action permission.'
    return {
        key: payload[key]
        for key in (
            'state',
            'effectiveEnabled',
            'source',
            'taskId',
            'phase',
            'voiceState',
            'voiceStateObserved',
            'voiceStatePublisherObserved',
            'sessionObserved',
            'sessionFresh',
            'sessionActive',
            'terminal',
            'functionalServicesRestored',
            'motionBlocked',
            'paused',
            'mappingDone',
            'finalMapSaved',
            'actionAllowed',
            'lastError',
            'actionBlockedReason',
            'sourceUpdatedAtEpochMs',
        )
    }, ''


def normalize_vehicle_readiness(payload: object) -> tuple[dict | None, str]:
    """Validate the pre-mapping cold-start gate from the long-lived agent."""
    if not isinstance(payload, dict):
        return None, 'Vehicle readiness observation must be an object.'
    state_name = str(payload.get('state') or '').strip().lower()
    required_booleans = (
        'mappingStartAllowed',
        'baseServiceActive',
        'scanFresh',
        'odomFresh',
        'odomAdvancing',
        'imuFresh',
        'baseToImuTfObserved',
        'odomToBaseTfFresh',
        'odomToBaseTfAdvancing',
    )
    required_timestamps = (
        'stableSinceEpochMs',
        'readySinceEpochMs',
        'startedAtEpochMs',
        'observedAtEpochMs',
    )
    if (
        state_name not in {'initializing', 'ready', 'failed'}
        or any(type(payload.get(key)) is not bool for key in required_booleans)
        or any(
            type(payload.get(key)) is not int or payload.get(key) < 0
            for key in required_timestamps
        )
        or not isinstance(payload.get('message'), str)
        or not isinstance(payload.get('serviceProbeError'), str)
        or (state_name == 'ready') != (payload.get('mappingStartAllowed') is True)
    ):
        return None, 'Vehicle readiness observation fields are invalid.'
    return {
        key: payload[key]
        for key in (
            'state',
            'mappingStartAllowed',
            'message',
            'baseServiceActive',
            'serviceProbeError',
            'scanFresh',
            'odomFresh',
            'odomAdvancing',
            'imuFresh',
            'baseToImuTfObserved',
            'odomToBaseTfFresh',
            'odomToBaseTfAdvancing',
            'stableSinceEpochMs',
            'readySinceEpochMs',
            'startedAtEpochMs',
            'observedAtEpochMs',
        )
    }, ''


def normalize_target_navigation_feature(payload: object) -> tuple[dict | None, str]:
    """Validate only dedicated I/J/K voice navigation, never generic Nav2."""
    if payload is None:
        return None, ''
    if not isinstance(payload, dict):
        return None, 'Target-navigation feature observation must be an object.'
    state_name = str(payload.get('state') or '').strip().lower()
    source = str(payload.get('source') or '').strip().lower()
    active = payload.get('active')
    interlock_active = payload.get('interlockActive')
    recovery_required = payload.get('recoveryRequired')
    goal = payload.get('goal')
    if (
        state_name not in {
            'unknown', 'starting', 'navigating', 'succeeded', 'failed', 'canceled',
        }
        or source not in {'voice_command_recognition', 'unknown'}
        or type(active) is not bool
        or type(interlock_active) is not bool
        or type(recovery_required) is not bool
        or type(payload.get('sessionObserved')) is not bool
        or type(payload.get('sessionTerminal')) is not bool
        or not isinstance(payload.get('goalId'), str)
        or type(payload.get('canCancel')) is not bool
        or payload.get('canCancel') is not False
        or type(payload.get('publisherObserved')) is not bool
        or type(payload.get('statusFresh')) is not bool
        or not isinstance(payload.get('message'), str)
        or not isinstance(payload.get('lastError'), str)
        or type(payload.get('sourceUpdatedAtEpochMs')) is not int
        or payload.get('sourceUpdatedAtEpochMs') <= 0
        or (state_name in {'starting', 'navigating'}) != active
        or (active and not interlock_active)
        or (recovery_required and not interlock_active)
        or (
            state_name in {'succeeded', 'failed', 'canceled'}
            and (interlock_active or recovery_required)
        )
    ):
        return None, 'Target-navigation feature observation fields are invalid.'
    if source == 'unknown' and state_name != 'unknown':
        return None, 'Unknown target-navigation source cannot report a task state.'
    if goal is not None:
        if not isinstance(goal, dict):
            return None, 'Target-navigation goal must be an object.'
        values = (
            goal.get('x'), goal.get('y'),
            goal.get('orientationZ'), goal.get('orientationW'),
        )
        if (
            not isinstance(goal.get('frameId'), str)
            or not goal.get('frameId').strip()
            or not isinstance(goal.get('label', ''), str)
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in values
            )
        ):
            return None, 'Target-navigation goal fields are invalid.'
    return {
        key: payload[key]
        for key in (
            'state', 'active', 'interlockActive', 'recoveryRequired',
            'sessionObserved', 'sessionTerminal', 'source', 'goalId', 'goal', 'canCancel',
            'publisherObserved', 'statusFresh', 'message', 'lastError',
            'sourceUpdatedAtEpochMs',
        )
    }, ''


def fetch_feature_status_probe() -> tuple[dict | None, str]:
    """Read cached follow/recharge truth from the long-lived robot-agent."""
    if not FEATURE_STATUS_URL:
        return None, 'MQTT_FEATURE_STATUS_URL is not configured.'
    try:
        with urlopen(FEATURE_STATUS_URL, timeout=FEATURE_STATUS_TIMEOUT) as response:
            body = response.read().decode('utf-8')
    except HTTPError as error:
        return None, f'Feature status probe returned HTTP {error.code}.'
    except URLError as error:
        return None, f'Feature status probe is unavailable: {error.reason}'
    except (OSError, UnicodeDecodeError) as error:
        return None, f'Feature status probe failed: {error}'
    try:
        payload = json.loads(body)
    except json.JSONDecodeError as error:
        return None, f'Feature status probe returned invalid JSON: {error.msg}'
    if not isinstance(payload, dict):
        return None, 'Feature status probe JSON must be an object.'
    if payload.get('ok') is not True:
        return None, str(payload.get('message') or 'robot-agent is not ready.')
    if str(payload.get('deviceId') or '').strip() != DEVICE_ID:
        return None, 'Feature status probe deviceId does not match this bridge.'
    return payload, ''


def auxiliary_feature_status() -> tuple[
    dict, dict, dict | None, dict | None, dict
]:
    """Read cached feature truth from the long-lived robot-agent.

    Mapping and voice target navigation are optional fields so a new bridge may
    still interoperate with an older robot-agent during a rolling deploy.
    """
    payload, probe_error = fetch_feature_status_probe()
    if payload is None:
        return (
            build_unknown_body_follow_feature(probe_error),
            build_unknown_recharge_feature(probe_error),
            build_unknown_mapping_feature(probe_error),
            build_unknown_target_navigation_feature(probe_error),
            build_unknown_vehicle_readiness(probe_error),
        )
    body_follow, body_error = normalize_body_follow_feature(payload.get('bodyFollow'))
    recharge, recharge_error = normalize_recharge_feature(payload.get('recharge'))
    mapping, mapping_error = normalize_mapping_feature(payload.get('mappingSession'))
    vehicle_readiness, vehicle_readiness_error = normalize_vehicle_readiness(
        payload.get('vehicleReadiness')
    )
    target_navigation, target_navigation_error = normalize_target_navigation_feature(
        payload.get('targetNavigation')
    )
    return (
        body_follow
        if body_follow is not None
        else build_unknown_body_follow_feature(body_error),
        recharge
        if recharge is not None
        else build_unknown_recharge_feature(recharge_error),
        mapping
        if mapping is not None or not mapping_error
        else build_unknown_mapping_feature(mapping_error),
        target_navigation
        if target_navigation is not None or not target_navigation_error
        else build_unknown_target_navigation_feature(target_navigation_error),
        vehicle_readiness
        if vehicle_readiness is not None
        else build_unknown_vehicle_readiness(vehicle_readiness_error),
    )


def _read_process_cmdline(pid: int) -> list[str]:
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as handle:
            raw = handle.read()
    except OSError:
        return []
    return [
        token.decode('utf-8', errors='replace')
        for token in raw.split(b'\0')
        if token
    ]


def _process_is_alive(pid: int) -> bool:
    identity = _read_process_identity(pid)
    return identity is not None and identity['state'] != 'Z'


def _read_process_identity(pid: int) -> dict | None:
    """Read /proc stat fields without breaking on spaces in process names."""
    try:
        with open(f'/proc/{pid}/stat', 'r', encoding='utf-8') as handle:
            raw_stat = handle.read()
        command_end = raw_stat.rfind(')')
        if command_end < 0:
            return None
        fields = raw_stat[command_end + 1:].split()
        if len(fields) < 4:
            return None
        return {
            'state': fields[0],
            'parentPid': int(fields[1]),
            'processGroupId': int(fields[2]),
            'sessionId': int(fields[3]),
        }
    except (OSError, ValueError):
        return None


def navigation_process_group_live_members(
    process_group_id: int,
    expected_session_id: int,
) -> tuple[list[int], str]:
    """Return non-zombie members, failing closed if the PGID was reused."""
    if (
        type(process_group_id) is not int
        or type(expected_session_id) is not int
        or process_group_id <= 1
        or expected_session_id <= 1
    ):
        return [], 'Navigation process-group identity is invalid.'
    try:
        proc_entries = os.listdir('/proc')
    except OSError as exc:
        return [], f'Cannot inspect navigation process group: {exc}'
    members = []
    for entry_name in proc_entries:
        if not entry_name.isdigit():
            continue
        pid = int(entry_name)
        identity = _read_process_identity(pid)
        if (
            identity is None
            or identity['state'] == 'Z'
            or identity['processGroupId'] != process_group_id
        ):
            continue
        if identity['sessionId'] != expected_session_id:
            return [], (
                'Navigation process-group identity changed; refusing to signal '
                f'PGID {process_group_id}.'
            )
        members.append(pid)
    return sorted(members), ''


def navigation_process_group_stopped(
    root_pid: int,
    process_group_id: int,
    expected_session_id: int,
    process: subprocess.Popen | None = None,
) -> tuple[bool, str]:
    """A stop is complete only after both launcher and its PGID are gone."""
    if process is not None:
        root_alive = process.poll() is None
    else:
        root_alive = _process_is_alive(root_pid)
    members, inspection_error = navigation_process_group_live_members(
        process_group_id, expected_session_id)
    if inspection_error:
        return False, inspection_error
    return not root_alive and not members, ''


def _wait_for_navigation_process_group_stop(
    root_pid: int,
    process_group_id: int,
    expected_session_id: int,
    timeout: float,
    process: subprocess.Popen | None = None,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        stopped, inspection_error = navigation_process_group_stopped(
            root_pid,
            process_group_id,
            expected_session_id,
            process,
        )
        if stopped or inspection_error:
            return stopped, inspection_error
        if time.monotonic() >= deadline:
            return False, ''
        time.sleep(0.1)


def _signal_navigation_process_group(
    process_group_id: int,
    expected_session_id: int,
    sig: int,
) -> tuple[bool, str]:
    members, inspection_error = navigation_process_group_live_members(
        process_group_id, expected_session_id)
    if inspection_error:
        return False, inspection_error
    if not members:
        return True, ''
    try:
        if process_group_id == os.getpgrp():
            return False, 'Refusing to signal the MQTT bridge process group.'
    except AttributeError:
        return False, 'POSIX process-group signaling is unavailable.'
    try:
        os.killpg(process_group_id, sig)
        return True, ''
    except ProcessLookupError:
        return True, ''
    except OSError as exc:
        return False, f'Cannot signal navigation PGID {process_group_id}: {exc}'


def _kill_nav2_containers_first(process_group_id: int, expected_session_id: int) -> str:
    """SIGKILL every composable container in the validated Nav2 process group.

    Containers must not receive SIGINT/SIGTERM: their teardown of the RotationShim/MPPI
    controller plus the layered costmap segfaults (verified in four cores).  SIGKILL
    cannot be handled or forwarded, so the destructor path never runs.
    """
    members, inspection_error = navigation_process_group_live_members(
        process_group_id, expected_session_id)
    if inspection_error or not members:
        return inspection_error
    containers = []
    for pid in members:
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as handle:
                argv0 = handle.read().split(b'\0', 1)[0].decode('utf-8', 'replace')
        except OSError:
            continue
        if os.path.basename(argv0).startswith('component_container'):
            containers.append(pid)
    if not containers:
        return ''
    for pid in containers:
        try:
            os.kill(pid, NAVIGATION_SIGKILL)
        except ProcessLookupError:
            pass
        except OSError as exc:
            return 'Cannot SIGKILL navigation container %d: %s' % (pid, exc)
    deadline = time.monotonic() + NAVIGATION_STOP_SIGKILL_TIMEOUT
    remaining = list(containers)
    while remaining and time.monotonic() < deadline:
        time.sleep(0.05)
        alive = []
        for pid in remaining:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                continue
            except OSError:
                continue
            identity = _read_process_identity(pid)
            if identity is not None and identity['state'] != 'Z':
                alive.append(pid)
        remaining = alive
    if remaining:
        return 'Navigation containers still alive after SIGKILL: %s' % remaining
    return ''


def stop_navigation_process_group(
    root_pid: int,
    process_group_id: int,
    expected_session_id: int,
    process: subprocess.Popen | None = None,
) -> tuple[bool, str]:
    """Boundedly escalate one validated Nav2 PGID and verify it is empty."""
    stopped, inspection_error = navigation_process_group_stopped(
        root_pid, process_group_id, expected_session_id, process)
    if stopped:
        return True, 'Navigation launcher and process group are already stopped.'
    if inspection_error:
        return False, inspection_error

    failure_reason = ''
    if NAVIGATION_STOP_CONTAINER_FIRST:
        container_error = _kill_nav2_containers_first(
            process_group_id, expected_session_id)
        if container_error:
            failure_reason = container_error
    if NAVIGATION_STOP_SIGKILL_ONLY:
        # Skip SIGINT/SIGTERM: the container's destructor path is what crashes.
        stages = (
            (NAVIGATION_SIGKILL, NAVIGATION_STOP_SIGKILL_TIMEOUT, 'SIGKILL'),
        )
    else:
            stages = (
                (signal.SIGINT, NAVIGATION_STOP_SIGINT_TIMEOUT, 'SIGINT'),
                (signal.SIGTERM, NAVIGATION_STOP_SIGTERM_TIMEOUT, 'SIGTERM'),
                (NAVIGATION_SIGKILL, NAVIGATION_STOP_SIGKILL_TIMEOUT, 'SIGKILL'),
            )
    # 2026-09-16 (FAST_STOP): a single Nav2 component container was observed
    # ignoring SIGINT while a healthy launcher stayed up, so the escalation must
    # never abandon the group halfway.  An inspection error on one stage is now
    # recorded and the escalation continues, and a final SIGKILL plus
    # verification runs before the group can be reported as unstoppable.
    for sig, timeout, label in stages:
        signaled, signal_error = _signal_navigation_process_group(
            process_group_id, expected_session_id, sig)
        if not signaled:
            failure_reason = failure_reason or signal_error
        else:
            stopped, inspection_error = _wait_for_navigation_process_group_stop(
                root_pid,
                process_group_id,
                expected_session_id,
                timeout,
                process,
            )
            if stopped:
                return True, (
                    'Navigation launcher exited and its process group is empty '
                    f'after {label}.'
                )
            if inspection_error:
                failure_reason = failure_reason or inspection_error

    _signal_navigation_process_group(
        process_group_id, expected_session_id, NAVIGATION_SIGKILL)
    stopped, _final_error = _wait_for_navigation_process_group_stop(
        root_pid,
        process_group_id,
        expected_session_id,
        NAVIGATION_STOP_SIGKILL_TIMEOUT,
        process,
    )
    if stopped:
        return True, (
            'Navigation launcher exited and its process group is empty after '
            'the final guaranteed SIGKILL.'
        )
    return False, failure_reason or (
        'Navigation process group still has live members after '
        'SIGINT, SIGTERM and SIGKILL.'
    )


def request_navigation_lifecycle_shutdown() -> list[str]:
    """Best-effort Nav2 lifecycle shutdown before process-group signaling."""
    outcomes = []
    for service_path in NAVIGATION_LIFECYCLE_MANAGER_SERVICES:
        command = [
            'ros2', 'service', 'call', service_path,
            'nav2_msgs/srv/ManageLifecycleNodes', '{command: 4}',
        ]
        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=NAVIGATION_LIFECYCLE_SHUTDOWN_TIMEOUT,
                check=False,
            )
            outcomes.append(
                f'{service_path}:exit={completed.returncode}')
        except subprocess.TimeoutExpired:
            outcomes.append(f'{service_path}:timeout')
        except OSError as exc:
            outcomes.append(f'{service_path}:error={exc}')
    return outcomes


def _is_navigation_launch_cmdline(tokens: list[str]) -> bool:
    for index, token in enumerate(tokens):
        if os.path.basename(token) != 'ros2':
            continue
        if tokens[index + 1:index + 4] == [
            'launch', 'wheeltec_nav2', 'wheeltec_nav2.launch.py',
        ]:
            return True
    return False


def find_navigation_launch_pids() -> tuple[list[int], str]:
    """Find live exact Nav2 launch roots without treating arbitrary nodes as owners."""
    try:
        proc_entries = os.listdir('/proc')
    except OSError as exc:
        return [], f'Cannot inspect process table: {exc}'
    candidates = []
    for entry_name in proc_entries:
        if not entry_name.isdigit():
            continue
        pid = int(entry_name)
        if pid == os.getpid() or not _process_is_alive(pid):
            continue
        if _is_navigation_launch_cmdline(_read_process_cmdline(pid)):
            candidates.append(pid)
    return sorted(candidates), ''


def find_voice_owned_navigation_launch() -> tuple[dict | None, str]:
    """Find exactly one safe-to-stop Nav2 launcher owned by voice control.

    A generic process-name match is intentionally not enough.  The root must
    have the exact ROS launch tokens, a direct voice bridge parent, and its own
    process group/session.  Any ambiguity fails closed and leaves the process
    untouched.
    """
    if not VOICE_NAVIGATION_PARENT_MARKER:
        return None, 'Voice navigation parent marker is not configured.'

    candidates = []
    try:
        proc_entries = os.listdir('/proc')
    except OSError as exc:
        return None, f'Cannot inspect process table: {exc}'

    for entry_name in proc_entries:
        if not entry_name.isdigit():
            continue
        pid = int(entry_name)
        tokens = _read_process_cmdline(pid)
        if not _is_navigation_launch_cmdline(tokens):
            continue
        identity = _read_process_identity(pid)
        if identity is None or identity['state'] == 'Z':
            continue
        parent_pid = identity['parentPid']
        process_group_id = identity['processGroupId']
        session_id = identity['sessionId']

        parent_tokens = _read_process_cmdline(parent_pid)
        parent_is_voice_bridge = any(
            VOICE_NAVIGATION_PARENT_MARKER in os.path.basename(token)
            for token in parent_tokens
        )
        if not parent_is_voice_bridge:
            continue
        if process_group_id != pid or session_id != pid:
            continue
        candidates.append({
            'pid': pid,
            'parentPid': parent_pid,
            'processGroupId': process_group_id,
        })

    if len(candidates) == 1:
        return candidates[0], ''
    if len(candidates) > 1:
        return None, 'Multiple voice-owned navigation launchers were observed.'
    return None, ''


def stop_voice_owned_navigation_launch(candidate: dict) -> tuple[bool, str]:
    """Gracefully stop a revalidated, voice-owned Nav2 process group.

    No fuzzy process-name kill or parent signal is used. The second identity
    check prevents a recycled PID or changed process tree from being signaled;
    success additionally requires the launcher and every live PGID member gone.
    """
    requested_pid = candidate.get('pid')
    if type(requested_pid) is not int or requested_pid <= 0:
        return False, 'Voice navigation launcher identity is invalid.'

    with voice_navigation_stop_lock:
        current, lookup_error = find_voice_owned_navigation_launch()
        if current is None:
            return False, lookup_error or 'Voice navigation launcher is no longer safely identifiable.'
        if current['pid'] != requested_pid:
            return False, 'Voice navigation launcher changed before the stop request.'
        lifecycle_outcomes = request_navigation_lifecycle_shutdown()
        stopped, message = stop_navigation_process_group(
            current['pid'],
            current['processGroupId'],
            current['pid'],
        )
        lifecycle_detail = ', '.join(lifecycle_outcomes) or 'no services configured'
        return stopped, f'{message} Lifecycle shutdown: {lifecycle_detail}'


class TimedProbeCache:
    """Share one short-lived HTTP probe across periodic and startup workers."""

    def __init__(self, ttl_sec: float) -> None:
        self.ttl_sec = max(0.0, float(ttl_sec))
        self._condition = threading.Condition()
        self._result = None
        self._expires_monotonic = 0.0
        self._refreshing = False

    def get(self, loader):
        while True:
            now_monotonic = time.monotonic()
            with self._condition:
                if (
                    self._result is not None
                    and now_monotonic < self._expires_monotonic
                ):
                    return copy.deepcopy(self._result)
                if not self._refreshing:
                    self._refreshing = True
                    break
                self._condition.wait(timeout=max(0.05, self.ttl_sec))

        try:
            result = loader()
        except Exception:
            with self._condition:
                self._refreshing = False
                self._condition.notify_all()
            raise
        with self._condition:
            self._result = copy.deepcopy(result)
            self._expires_monotonic = time.monotonic() + self.ttl_sec
            self._refreshing = False
            self._condition.notify_all()
            return copy.deepcopy(self._result)


navigation_system_probe_cache = TimedProbeCache(
    NAVIGATION_SYSTEM_PROBE_CACHE_TTL
)


def _fetch_navigation_system_probe_uncached() -> tuple[dict | None, str]:
    """Read the read-only Nav2 readiness probe exposed by robot-agent.

    The probe is intentionally independent of the MQTT bridge's launch process
    table so that a voice-triggered Nav2 instance is also observable.
    """
    if not NAVIGATION_SYSTEM_STATUS_URL:
        return None, 'MQTT_NAVIGATION_SYSTEM_STATUS_URL is not configured.'
    try:
        with urlopen(
            NAVIGATION_SYSTEM_STATUS_URL,
            timeout=NAVIGATION_SYSTEM_STATUS_TIMEOUT,
        ) as response:
            body = response.read().decode('utf-8')
    except HTTPError as error:
        return None, f'Navigation system probe returned HTTP {error.code}.'
    except URLError as error:
        return None, f'Navigation system probe is unavailable: {error.reason}'
    except (OSError, UnicodeDecodeError) as error:
        return None, f'Navigation system probe failed: {error}'

    try:
        payload = json.loads(body)
    except json.JSONDecodeError as error:
        return None, f'Navigation system probe returned invalid JSON: {error.msg}'
    if not isinstance(payload, dict):
        return None, 'Navigation system probe JSON must be an object.'
    return payload, ''


def fetch_navigation_system_probe() -> tuple[dict | None, str]:
    """Coalesce periodic snapshot and startup-health reads into one probe."""
    return navigation_system_probe_cache.get(
        _fetch_navigation_system_probe_uncached
    )


def normalize_navigation_system_probe(payload: dict) -> tuple[dict | None, str]:
    """Validate the robot-agent probe before it can become UI state."""
    if str(payload.get('deviceId') or '').strip() != DEVICE_ID:
        return None, 'Navigation system probe deviceId does not match this bridge.'

    state_name = str(payload.get('state') or '').strip().lower()
    allowed_states = {'stopped', 'starting', 'ready', 'failed', 'unknown'}
    effective_enabled = payload.get('effectiveEnabled')
    launch_observed = payload.get('launchObserved')
    action_server_ready = payload.get('actionServerReady')
    pose_ready = payload.get('poseReady')
    # These diagnostics were added after the first feature-status rollout.
    # Preserve compatibility with an older robot-agent during a rolling deploy.
    pose_available = payload.get('poseAvailable', pose_ready)
    pose_fresh = payload.get('poseFresh', pose_ready)
    localization_health = str(
        payload.get('localizationHealth') or (
            'ready' if pose_ready is True else 'unknown'
        )
    ).strip().lower()
    localization_probe_available = payload.get('localizationProbeAvailable', False)
    amcl_node_observed = payload.get('amclNodeObserved', False)
    pose_publisher_observed = payload.get('posePublisherObserved', False)
    localization_source_observed = payload.get(
        'localizationSourceObserved', pose_ready
    )
    safety_ready = payload.get('safetyReady', False)
    guard_healthy = payload.get('guardHealthy', False)
    guard_status_fresh = payload.get('guardStatusFresh', False)
    interlock_ready = payload.get('interlockReady', False)
    interlock_status_fresh = payload.get('interlockStatusFresh', False)
    safety_error = payload.get(
        'safetyError',
        'Navigation safety status is unavailable from robot-agent.',
    )
    last_error = payload.get('lastError')
    message = payload.get('message')
    observed_at_epoch_ms = payload.get('observedAtEpochMs')

    if state_name not in allowed_states:
        return None, f'Navigation system probe state is invalid: {state_name or "<empty>"}.'
    if any(
        type(value) is not bool
        for value in (
            effective_enabled,
            launch_observed,
            action_server_ready,
            pose_ready,
            pose_available,
            pose_fresh,
            localization_probe_available,
            amcl_node_observed,
            pose_publisher_observed,
            localization_source_observed,
            safety_ready,
            guard_healthy,
            guard_status_fresh,
            interlock_ready,
            interlock_status_fresh,
        )
    ):
        return None, 'Navigation system probe boolean fields are invalid.'
    if localization_health not in {'unknown', 'starting', 'ready', 'failed'}:
        return None, 'Navigation system probe localizationHealth is invalid.'
    if not isinstance(last_error, str):
        return None, 'Navigation system probe lastError is invalid.'
    if not isinstance(message, str):
        return None, 'Navigation system probe message is invalid.'
    if not isinstance(safety_error, str):
        return None, 'Navigation system probe safetyError is invalid.'
    if type(observed_at_epoch_ms) is not int or observed_at_epoch_ms <= 0:
        return None, 'Navigation system probe observedAtEpochMs is invalid.'

    ready_combination = (
        state_name == 'ready'
        and effective_enabled is True
        and action_server_ready is True
        and pose_ready is True
        and safety_ready is True
        and not last_error
    )
    if state_name == 'ready' and not ready_combination:
        return None, 'Navigation system probe reported an inconsistent ready state.'
    if state_name != 'ready' and effective_enabled is True:
        return None, 'Navigation system probe enabled flag conflicts with its state.'

    return {
        'state': state_name,
        'effectiveEnabled': effective_enabled,
        'launchObserved': launch_observed,
        'actionServerReady': action_server_ready,
        'poseReady': pose_ready,
        'poseAvailable': pose_available,
        'poseFresh': pose_fresh,
        'localizationHealth': localization_health,
        'localizationProbeAvailable': localization_probe_available,
        'amclNodeObserved': amcl_node_observed,
        'posePublisherObserved': pose_publisher_observed,
        'localizationSourceObserved': localization_source_observed,
        'safetyReady': safety_ready,
        'guardHealthy': guard_healthy,
        'guardStatusFresh': guard_status_fresh,
        'interlockReady': interlock_ready,
        'interlockStatusFresh': interlock_status_fresh,
        'safetyError': safety_error,
        'lastError': last_error,
        'message': message,
        'observedAtEpochMs': observed_at_epoch_ms,
    }, ''


def navigation_system_state_from_probe() -> tuple[dict, str]:
    payload, fetch_error = fetch_navigation_system_probe()
    if payload is None:
        return {
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
            'safetyReady': False,
            'guardHealthy': False,
            'guardStatusFresh': False,
            'interlockReady': False,
            'interlockStatusFresh': False,
            'safetyError': fetch_error,
            'lastError': fetch_error,
            'message': fetch_error,
            'observedAtEpochMs': int(time.time() * 1000),
        }, 'unknown'

    navigation_system, validation_error = normalize_navigation_system_probe(payload)
    if navigation_system is None:
        return {
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
            'safetyReady': False,
            'guardHealthy': False,
            'guardStatusFresh': False,
            'interlockReady': False,
            'interlockStatusFresh': False,
            'safetyError': validation_error,
            'lastError': validation_error,
            'message': validation_error,
            'observedAtEpochMs': int(time.time() * 1000),
        }, 'unknown'
    return navigation_system, 'ready'


ALLOWED_NODE_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ALLOWED_NODE_TARGETS', ''),
    [
        build_target_key('wheeltec_robot_keyboard', 'wheeltec_keyboard'),
        build_target_key('auto_recharge_ros2', 'navigate_to_charger'),
        build_target_key('auto_recharge_ros2', 'cancel_charge'),
        build_target_key('web_video_server', 'web_video_server'),
    ],
)
ONE_SHOT_NODE_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ONE_SHOT_NODE_TARGETS', ''),
    [],
)
ALLOWED_SERVICE_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ALLOWED_SERVICE_TARGETS', ''),
    [
        build_target_key('/stemm/set_mapping_done', 'std_srvs/srv/SetBool'),
        # The mini program addresses the modern service names.
        build_target_key('/stemm_cartographer/set_mapping_done', 'std_srvs/srv/SetBool'),
        build_target_key('/stemm_cartographer/save_map', 'std_srvs/srv/Trigger'),
        build_target_key('/stemm_cartographer/save_state', 'std_srvs/srv/Trigger'),
        build_target_key('/stemm_cartographer/cancel_mapping', 'std_srvs/srv/Trigger'),
        build_target_key('/stemm/save_map', 'std_srvs/srv/Trigger'),
        build_target_key('/stemm_cartographer/stop_navigation', 'std_srvs/srv/Trigger'),
        build_target_key('/stemm_cartographer/start_mapping', 'std_srvs/srv/Trigger'),
        build_target_key('/stemm_cartographer/cancel_mapping', 'std_srvs/srv/Trigger'),
        build_target_key('/voice_summon/set_body_follow_enabled', 'std_srvs/srv/SetBool'),
        build_target_key('/voice_summon/get_body_follow_status', 'std_srvs/srv/Trigger'),
    ],
)

state = {
    'launchStatus': 'idle',
    'currentTask': '-',
    'currentTaskType': '-',
    'lastError': '',
    'requestId': None,
    'runningLaunches': [],
    'runningNodes': [],
}
state_lock = threading.Lock()
feature_status_lock = threading.Lock()
feature_status_state = {
    'revision': 0,
    'signature': None,
    'lastPublishMonotonic': 0.0,
}
# Serializes the revalidation-and-signal sequence for an externally owned Nav2
# launcher.  This is intentionally separate from `state_lock`: process signal
# waits must not block normal status publishing.
voice_navigation_stop_lock = threading.Lock()
navigation_stop_transition_lock = threading.Lock()
navigation_stop_transition = {
    'active': False,
    'source': 'unknown',
    'startedAtMonotonic': 0.0,
}
launch_processes: dict[str, dict] = {}
node_processes: dict[str, dict] = {}
one_shot_processes: dict[str, dict] = {}
navigation_request_observers: dict[str, dict] = {}
stop_event = threading.Event()


def publish_json(
    client: mqtt.Client,
    topic: str,
    payload: dict,
    *,
    retain: bool = False,
) -> None:
    client.publish(
        topic,
        json.dumps(payload, ensure_ascii=True),
        qos=1,
        retain=retain,
    )


def build_navigation_feature_offline_snapshot() -> dict:
    with feature_status_lock:
        revision = int(feature_status_state['revision'] or 0)
    return {
        'schemaVersion': FEATURE_STATUS_SCHEMA_VERSION,
        'deviceId': DEVICE_ID,
        'bootId': BRIDGE_BOOT_ID,
        'revision': revision,
        'availability': 'offline',
        'observedAtEpochMs': int(time.time() * 1000),
        'navigationSystem': {
            'state': 'unknown',
            'effectiveEnabled': False,
            'canEnable': False,
            'canDisable': False,
            'source': 'unknown',
            'actionAllowed': {
                'start': False,
                'stop': False,
                'targetNavigation': False,
            },
            'actionBlockedReason': 'MQTT bridge is offline',
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
            'safetyReady': False,
            'guardHealthy': False,
            'guardStatusFresh': False,
            'interlockReady': False,
            'interlockStatusFresh': False,
            'safetyError': 'MQTT bridge is offline',
            'lastError': 'MQTT bridge is offline',
            'message': 'MQTT bridge is offline',
        },
        'bodyFollow': build_unknown_body_follow_feature('MQTT bridge is offline'),
        'recharge': build_unknown_recharge_feature('MQTT bridge is offline'),
        'mappingSession': build_unknown_mapping_feature('MQTT bridge is offline'),
        'targetNavigation': build_unknown_target_navigation_feature(
            'MQTT bridge is offline'
        ),
    }


def bridge_navigation_launch_status() -> tuple[bool, float | None]:
    with state_lock:
        entry = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
        if not entry or not navigation_process_entry_active(entry):
            return False, None
        started_at_monotonic = entry.get('startedAtMonotonic')
    if not isinstance(started_at_monotonic, (int, float)):
        return True, None
    return True, max(0.0, time.monotonic() - started_at_monotonic)


def bridge_navigation_health_status() -> dict:
    with state_lock:
        entry = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
        if not entry or not navigation_process_entry_active(entry):
            return {
                'terminal': False,
                'confirmed': False,
                'failure': '',
            }
        return {
            'terminal': entry.get('_navigationHealthTerminal') is True,
            'confirmed': entry.get('healthConfirmed') is True,
            'failure': str(entry.get('healthFailure') or ''),
        }


def set_navigation_stop_transition(active: bool, source: str = 'unknown') -> None:
    with navigation_stop_transition_lock:
        navigation_stop_transition['active'] = active
        navigation_stop_transition['source'] = source if active else 'unknown'
        navigation_stop_transition['startedAtMonotonic'] = (
            time.monotonic() if active else 0.0
        )


def get_navigation_stop_transition() -> dict | None:
    now_monotonic = time.monotonic()
    with navigation_stop_transition_lock:
        if not navigation_stop_transition['active']:
            return None
        started_at = navigation_stop_transition['startedAtMonotonic']
        if (
            not isinstance(started_at, (int, float))
            or now_monotonic - started_at > NAVIGATION_STOP_TRANSITION_TIMEOUT
        ):
            navigation_stop_transition['active'] = False
            navigation_stop_transition['source'] = 'unknown'
            navigation_stop_transition['startedAtMonotonic'] = 0.0
            return None
        return {
            'source': navigation_stop_transition['source'],
            'ageSec': max(0.0, now_monotonic - started_at),
        }


def mapping_owns_vehicle(mapping_session: object) -> bool:
    """True while a Cartographer session owns motion and the Nav2 graph.

    Added 2026-09-16.  The mapping session runs its own root-namespace Nav2, so the
    read-only navigation probe reports that graph as if navigation were running.
    Two consumers need this predicate:
      * the published snapshot, so the app is not told "navigation failed" while the
        vehicle is deliberately mapping (that also greyed out target navigation);
      * the start path, so a navigation launch cannot be started on top of a mapping
        session (AMCL and Cartographer would both publish map->odom and the TF tree
        broke: TF_OLD_DATA, planner/spin aborts, return-home timeouts).
    """
    if not isinstance(mapping_session, dict):
        return False
    # 2026-09-29: a terminally ended session can never own the vehicle.  Without this the
    # predicate latched on 'failed + not functionalServicesRestored' for a dead session and
    # the app showed a permanent mapping-start failure that survived a reboot.
    if mapping_session.get('terminal') is True:
        return False
    state_name = str(mapping_session.get('state') or '').strip().lower()
    return bool(
        mapping_session.get('motionBlocked') is True
        or state_name in {'starting', 'ready', 'stopping'}
        or (
            state_name == 'failed'
            and mapping_session.get('functionalServicesRestored') is not True
        )
    )


def mapping_allows_navigation_start(mapping_session: dict | None) -> tuple[bool, str]:
    """Require positive Cartographer-idle evidence before new Nav2 work."""
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
        or 'Cartographer mapping is not confirmed idle or terminally recovered.'
    )


def navigation_control_metadata(
    navigation_system: dict,
    availability: str,
    mapping_session: dict | None = None,
    target_navigation: dict | None = None,
) -> dict:
    """Describe which navigation actions are safe for the current owner.

    ``effectiveEnabled`` reports observed Nav2 readiness; it does not prove a
    particular launch owner can be stopped.  Keeping that distinction in the
    car-side snapshot makes a voice-started system visible without exposing a
    misleading enabled-but-unclickable UI action.
    """
    state_name = navigation_system.get('state')
    mapping_state = (
        str(mapping_session.get('state') or '').strip().lower()
        if isinstance(mapping_session, dict) else 'unknown'
    )
    mapping_motion_blocked = bool(
        mapping_session.get('motionBlocked')
        if isinstance(mapping_session, dict) else False
    )
    mapping_services_restored = bool(
        mapping_session.get('functionalServicesRestored')
        if isinstance(mapping_session, dict) else False
    )
    mapping_interlock_active = (
        mapping_motion_blocked
        or mapping_state in {'starting', 'ready', 'stopping'}
        or (mapping_state == 'failed' and not mapping_services_restored)
    )
    mapping_start_allowed, mapping_start_blocked_reason = (
        mapping_allows_navigation_start(mapping_session)
    )
    target_navigation_active = bool(
        isinstance(target_navigation, dict)
        and target_navigation.get('active') is True
        and str(target_navigation.get('state') or '').strip().lower()
        in {'starting', 'navigating'}
    )
    target_navigation_interlock = bool(
        isinstance(target_navigation, dict)
        and (
            target_navigation.get('interlockActive') is True
            or target_navigation.get('recoveryRequired') is True
        )
    )
    if mapping_interlock_active:
        mapping_reason = (
            str(mapping_session.get('actionBlockedReason') or '').strip()
            if isinstance(mapping_session, dict) else ''
        )
        return {
            'source': 'unknown',
            'canEnable': False,
            'canDisable': False,
            'actionAllowed': {
                'start': False,
                'stop': False,
                'targetNavigation': False,
            },
            'actionBlockedReason': mapping_reason or (
                'Cartographer mapping currently owns vehicle motion and navigation services.'
            ),
        }
    active_state = state_name in {'starting', 'ready', 'failed'}
    start_allowed = (
        availability == 'ready'
        and state_name == 'stopped'
        and mapping_start_allowed
    )
    target_navigation_allowed = (
        availability == 'ready'
        and state_name == 'ready'
        and navigation_system.get('effectiveEnabled') is True
        and navigation_system.get('actionServerReady') is True
        and navigation_system.get('poseReady') is True
        and navigation_system.get('safetyReady') is True
        and not str(navigation_system.get('lastError') or '').strip()
        and not target_navigation_active
        and not target_navigation_interlock
    )
    def navigation_is_proven_running() -> bool:
        """2026-09-29 (F-40): capability is what the ROS probe proves.

        The bridge's own process table only answers "did *this* bridge spawn it".
        A nav stack started from voice/hand control, or surviving a bridge restart,
        is still a running, stoppable navigation system: the stop request travels
        over the same topic it always did.  Reporting stop=false in that case made
        the mini program treat a healthy navigation as "not ready" and grey out both
        目标导航 and 小车回充.  Anything the probe cannot confirm still fails closed.
        """
        probe_state = str(navigation_system.get('state') or '').strip().lower()
        if probe_state not in {'starting', 'ready'}:
            return False
        if navigation_system.get('actionServerReady') is not True:
            return False
        return True

    managed_running, _ = bridge_navigation_launch_status()
    voice_launch = None
    lookup_error = ''
    if active_state:
        voice_launch, lookup_error = find_voice_owned_navigation_launch()
    if active_state and managed_running and voice_launch is not None:
        return {
            'source': 'unknown',
            'canEnable': False,
            'canDisable': False,
            'actionAllowed': {
                'start': False,
                'stop': False,
                'targetNavigation': False,
            },
            'actionBlockedReason': (
                'Multiple navigation launch owners were observed; shutdown is '
                'blocked until the vehicle is inspected.'
            ),
        }
    if active_state and managed_running:
        return {
            'source': 'bridge',
            'canEnable': False,
            'canDisable': True,
            'actionAllowed': {
                'start': False,
                'stop': True,
                'targetNavigation': target_navigation_allowed,
            },
            'actionBlockedReason': '',
        }

    if active_state:
        if voice_launch is not None:
            return {
                'source': 'voice',
                'canEnable': False,
                'canDisable': True,
                'actionAllowed': {
                    'start': False,
                    'stop': True,
                    'targetNavigation': target_navigation_allowed,
                },
                'actionBlockedReason': '',
            }
        proven_running = navigation_is_proven_running()
        return {
            'source': 'unknown',
            'canEnable': False,
            'canDisable': proven_running,
            'actionAllowed': {
                'start': False,
                'stop': proven_running,
                'targetNavigation': target_navigation_allowed,
            },
            'actionBlockedReason': lookup_error or (
                ''
                if proven_running
                else 'Navigation is active but its launcher cannot be safely identified for shutdown.'
            ),
        }

    if state_name == 'stopped':
        return {
            'source': 'none',
            'canEnable': start_allowed,
            'canDisable': False,
            'actionAllowed': {
                'start': start_allowed,
                'stop': False,
                'targetNavigation': False,
            },
            'actionBlockedReason': (
                ''
                if start_allowed
                else (
                    mapping_start_blocked_reason
                    if availability == 'ready'
                    else 'Navigation status is not available.'
                )
            ),
        }

    return {
        'source': 'unknown',
        'canEnable': False,
        'canDisable': False,
        'actionAllowed': {
            'start': False,
            'stop': False,
            'targetNavigation': False,
        },
        'actionBlockedReason': 'Navigation system state is still being confirmed.',
    }



_navigation_state_since = {'state': None, 'epochMs': 0}


def navigation_state_since_epoch_ms(state_name: object) -> int:
    """Epoch ms when the current navigation state began (stable across probes).

    Added 2026-09-16 (R3): lets consumers age a state (for example a transient
    ``failed`` raised while the managed launch is still terminating) instead of
    relying on revision monotonicity alone.  The value only changes when the
    state name changes, so it never churns the feature-status revision.
    """
    normalized = str(state_name or '').strip().lower()
    now_ms = int(time.time() * 1000)
    if (
        _navigation_state_since['state'] != normalized
        or not _navigation_state_since['epochMs']
    ):
        _navigation_state_since['state'] = normalized
        _navigation_state_since['epochMs'] = now_ms
    return int(_navigation_state_since['epochMs'])


# ---------------------------------------------------------------------------
# 2026-10-09 (operator): 返航完成后自动拉起导航栈。
#
# 用户诉求："建完图我就要直接使用导航，不想每次手动拉起导航栈"。
# 设计约束（均经车主确认）：
#   * 只在 terminationReason == 'return_home_complete' 时触发；
#     用户取消 / 设备重启 / 启动超时一律不触发——那几条路径要保留"不自动"
#     以便复盘 AMCL bootstrap。
#   * 单次尝试，不重试。失败只记录，不阻断用户手动再点。
#   * 返航完成时若确实在充电（chargingConfirmed 为 True），跳过。正常情况返航
#     停在建图原点（= 充电桩前约 0.6 m），并不在桩上；此分支只覆盖
#     "建图起点本身就在桩上" 的边界情形，避免撞上充电互锁。
# ---------------------------------------------------------------------------

def _read_mapping_session_termination() -> dict:
    """Read the runner's terminal reason. Fail-soft: any error means 'unknown'."""
    path = CARTOGRAPHER_SESSION_STATUS_PATH
    if not path:
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            payload = json.load(handle)
    except (OSError, ValueError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _autonav_should_arm(session: dict) -> bool:
    """A new mapping session is running: allow exactly one future after-session start."""
    if not AUTO_NAVIGATION_AFTER_MAPPING:
        return False
    if session.get('terminal') is True:
        return False
    if str(session.get('phase') or '').strip().lower() not in {
        'starting', 'mapping', 'returning_home',
    }:
        return False
    return bool(str(session.get('taskId') or '').strip())


def _autonav_idle_navigation_allowed(navigation_system: dict) -> tuple[bool, str]:
    state_name = str(navigation_system.get('state') or '').strip().lower()
    if state_name in {'starting', 'ready', 'stopping'}:
        return False, f'navigation is already {state_name or "busy"}'
    return True, ''


def _autonav_after_session_tick(
    client: 'mqtt.Client',
    mapping_session: object,
    recharge: object,
    navigation_system: object,
) -> None:
    """Arm on a live session; fire exactly once when that session ends successfully."""
    if not AUTO_NAVIGATION_AFTER_MAPPING:
        return
    session_file = _read_mapping_session_termination()
    task_id = str(session_file.get('taskId') or '').strip()

    if _autonav_should_arm(session_file):
        with _auto_navigation_lock:
            if _auto_navigation_state['taskId'] != task_id:
                _auto_navigation_state['taskId'] = task_id
                _auto_navigation_state['firedTaskId'] = ''
                print(
                    '[INFO] Auto-navigation armed after mapping session %s.'
                    % task_id, flush=True)
        return

    with _auto_navigation_lock:
        armed_task = _auto_navigation_state['taskId']
        already_fired = _auto_navigation_state['firedTaskId']
    if not armed_task or task_id != armed_task or already_fired == task_id:
        return

    # --- 会话已终态：检查触发条件 ---
    mapping = mapping_session if isinstance(mapping_session, dict) else {}
    if mapping.get('terminal') is not True:
        return
    if mapping.get('functionalServicesRestored') is not True:
        # 服务尚未恢复时 start_launch 的准入必然会拒绝，等下一拍。
        return
    reason = str(session_file.get('terminationReason') or '').strip()
    if reason not in AUTO_NAVIGATION_TRIGGER_REASONS:
        with _auto_navigation_lock:
            _auto_navigation_state['firedTaskId'] = task_id
        print(
            '[INFO] Auto-navigation skipped: terminationReason=%r taskId=%s'
            % (reason, task_id), flush=True)
        return

    charging = recharge if isinstance(recharge, dict) else {}
    if charging.get('chargingConfirmed') is True:
        with _auto_navigation_lock:
            _auto_navigation_state['firedTaskId'] = task_id
        print(
            '[INFO] Auto-navigation skipped: vehicle is charging '
            '(chargingConfirmed=true) taskId=%s' % task_id, flush=True)
        return

    nav = navigation_system if isinstance(navigation_system, dict) else {}
    allowed, why = _autonav_idle_navigation_allowed(nav)
    if not allowed:
        return

    # 只消费一次，无论结果如何。
    with _auto_navigation_lock:
        if _auto_navigation_state['firedTaskId'] == task_id:
            return
        _auto_navigation_state['firedTaskId'] = task_id

    request_id = 'auto-nav-%s' % task_id
    print(
        '[INFO] Auto-navigation: starting %s/%s after %s (requestId=%s).'
        % (DEFAULT_PACKAGE, DEFAULT_LAUNCH_FILE, reason, request_id), flush=True)
    try:
        start_launch(client, {
            'package': DEFAULT_PACKAGE,
            'launchFile': DEFAULT_LAUNCH_FILE,
            'requestId': request_id,
        })
    except Exception as exc:  # noqa: BLE001 - 自动路径绝不能让 bridge 挂掉
        print(
            '[ERROR] Auto-navigation failed to start: %s' % exc, flush=True)
        try:
            publish_json(
                client,
                ERROR_TOPIC,
                build_status_payload(
                    'Auto-navigation after mapping failed to start: %s' % exc,
                    status='failed',
                    action='auto_navigation',
                    requestId=request_id,
                ),
            )
        except Exception:  # noqa: BLE001
            pass


def publish_navigation_feature_status(
    client: mqtt.Client,
    reason: str,
    *,
    force: bool = False,
) -> dict:
    """Publish a retained, versioned navigation snapshot for local consumers."""
    if not NAVIGATION_FEATURE_STATUS_ENABLED:
        # Cartographer-only bridge roles keep their dedicated command/status
        # topics but do not duplicate the primary bridge's robot-agent probes.
        return {}
    navigation_system, availability = navigation_system_state_from_probe()
    # Immediately after this bridge spawns `ros2 launch`, the ROS graph may not
    # yet expose any Nav2 nodes.  Preserve the known managed start as a short
    # `starting` state, but never promote it to `ready` without the probe.
    managed_navigation_launch_running, managed_navigation_launch_age = (
        bridge_navigation_launch_status()
    )
    managed_navigation_health = bridge_navigation_health_status()
    if (
        availability == 'ready'
        and navigation_system['state'] == 'stopped'
        and managed_navigation_launch_running
    ):
        navigation_system = dict(navigation_system)
        if (
            managed_navigation_launch_age is None
            or managed_navigation_launch_age < NAVIGATION_SYSTEM_STARTUP_TIMEOUT
        ):
            navigation_system.update({
                'state': 'starting',
                'effectiveEnabled': False,
                'launchObserved': True,
                'lastError': '',
            })
        else:
            navigation_system.update({
                'state': 'failed',
                'effectiveEnabled': False,
                'launchObserved': True,
                'lastError': (
                    'Managed navigation launch did not become observable '
                    'before timeout.'
                ),
            })
    if (
        managed_navigation_launch_running
        and managed_navigation_health['terminal']
        and not managed_navigation_health['confirmed']
        and navigation_system['state'] != 'ready'
    ):
        navigation_system = dict(navigation_system)
        navigation_system.update({
            'state': 'failed',
            'effectiveEnabled': False,
            'launchObserved': True,
            'lastError': managed_navigation_health['failure'] or (
                'Navigation startup health confirmation failed.'
            ),
        })
    navigation_system = dict(navigation_system)
    stop_transition = get_navigation_stop_transition()
    if (
        stop_transition is not None
        and navigation_system['state'] == 'stopped'
        and navigation_system['launchObserved'] is False
    ):
        # Do not declare shutdown complete merely because the root launcher
        # exited.  The read-only robot-agent probe is the authority for Nav2
        # graph disappearance; until it confirms stopped, keep the UI in its
        # fail-closed stopping transition.
        set_navigation_stop_transition(False)
        stop_transition = None
    if stop_transition is not None and navigation_system['state'] in {
        'starting', 'ready', 'failed',
    }:
        navigation_system.update({
            'state': 'stopping',
            'effectiveEnabled': False,
            'lastError': '',
        })
    body_follow, recharge, mapping_session, target_navigation, vehicle_readiness = (
        auxiliary_feature_status()
    )
    navigation_control = navigation_control_metadata(
        navigation_system,
        availability,
        mapping_session,
        target_navigation,
    )
    if stop_transition is not None:
        navigation_control = {
            'source': stop_transition['source'],
            'canEnable': False,
            'canDisable': False,
            'actionAllowed': {
                'start': False,
                'stop': False,
                'targetNavigation': False,
            },
            'actionBlockedReason': 'Navigation shutdown is in progress.',
        }
    navigation_system.update(navigation_control)
    # 2026-10-09: 暴露自动拉起状态，便于审计与避免前端误判为"卡在 starting"。
    with _auto_navigation_lock:
        navigation_system['autostart'] = {
            'enabled': AUTO_NAVIGATION_AFTER_MAPPING,
            'armedTaskId': _auto_navigation_state['taskId'],
            'firedTaskId': _auto_navigation_state['firedTaskId'],
        }
    if mapping_owns_vehicle(mapping_session):
        # Report a coherent suspended state instead of the mapping session's own
        # Nav2 graph, which the probe can only describe as ready/failed.
        navigation_system['state'] = 'stopped'
        navigation_system['effectiveEnabled'] = False
        navigation_system['lastError'] = ''
        navigation_system['message'] = (
            'Cartographer mapping owns the vehicle; navigation is suspended.')
    navigation_system['stateSinceEpochMs'] = navigation_state_since_epoch_ms(
        navigation_system.get('state'))
    semantic_payload = {
        'availability': availability,
        'navigationSystem': {
            key: value
            for key, value in navigation_system.items()
            if key != 'observedAtEpochMs'
        },
        'bodyFollow': {
            key: value
            for key, value in body_follow.items()
            if key != 'observedAtEpochMs'
        },
        'recharge': {
            key: value
            for key, value in recharge.items()
            if key != 'observedAtEpochMs'
        },
        'vehicleReadiness': {
            key: value
            for key, value in vehicle_readiness.items()
            if key not in {
                'stableSinceEpochMs',
                'readySinceEpochMs',
                'startedAtEpochMs',
                'observedAtEpochMs',
            }
        },
    }
    if mapping_session is not None:
        semantic_payload['mappingSession'] = {
            key: value
            for key, value in mapping_session.items()
            if key not in {'observedAtEpochMs', 'sourceUpdatedAtEpochMs'}
        }
    if target_navigation is not None:
        semantic_payload['targetNavigation'] = {
            key: value
            for key, value in target_navigation.items()
            if key not in {'observedAtEpochMs', 'sourceUpdatedAtEpochMs'}
        }
    signature = json.dumps(semantic_payload, ensure_ascii=True, sort_keys=True)
    now_monotonic = time.monotonic()

    with feature_status_lock:
        signature_changed = feature_status_state['signature'] != signature
        now_wall = time.time()
        last_bump = float(feature_status_state.get('revisionBumpEpoch') or 0.0)
        if signature_changed:
            feature_status_state['revision'] = int(feature_status_state['revision']) + 1
            feature_status_state['signature'] = signature
            feature_status_state['revisionBumpEpoch'] = now_wall
        elif (
            FEATURE_STATUS_REVISION_HEARTBEAT_SEC > 0.0
            and now_wall - last_bump >= FEATURE_STATUS_REVISION_HEARTBEAT_SEC
        ):
            feature_status_state['revision'] = int(feature_status_state['revision']) + 1
            feature_status_state['revisionBumpEpoch'] = now_wall
        revision = int(feature_status_state['revision'])
        should_publish = (
            force
            or signature_changed
            or feature_status_state['lastPublishMonotonic'] == 0.0
            or now_monotonic - feature_status_state['lastPublishMonotonic']
            >= NAVIGATION_FEATURE_STATUS_INTERVAL
        )
        if should_publish:
            feature_status_state['lastPublishMonotonic'] = now_monotonic

    snapshot = {
        'schemaVersion': FEATURE_STATUS_SCHEMA_VERSION,
        'deviceId': DEVICE_ID,
        'bootId': BRIDGE_BOOT_ID,
        'revision': revision,
        'availability': availability,
        'observedAtEpochMs': int(time.time() * 1000),
        'navigationSystem': semantic_payload['navigationSystem'],
        'bodyFollow': semantic_payload['bodyFollow'],
        'recharge': semantic_payload['recharge'],
        'vehicleReadiness': {
            **semantic_payload['vehicleReadiness'],
            'stableSinceEpochMs': vehicle_readiness['stableSinceEpochMs'],
            'readySinceEpochMs': vehicle_readiness['readySinceEpochMs'],
            'startedAtEpochMs': vehicle_readiness['startedAtEpochMs'],
            'observedAtEpochMs': vehicle_readiness['observedAtEpochMs'],
        },
        **(
            {
                'mappingSession': {
                    **semantic_payload['mappingSession'],
                    'sourceUpdatedAtEpochMs': mapping_session['sourceUpdatedAtEpochMs'],
                }
            }
            if 'mappingSession' in semantic_payload and mapping_session is not None
            else {}
        ),
        **(
            {
                'targetNavigation': {
                    **semantic_payload['targetNavigation'],
                    'sourceUpdatedAtEpochMs': target_navigation['sourceUpdatedAtEpochMs'],
                }
            }
            if 'targetNavigation' in semantic_payload and target_navigation is not None
            else {}
        ),
    }
    if should_publish:
        publish_json(client, FEATURE_STATUS_TOPIC, snapshot, retain=True)
        if signature_changed or force:
            print(
                'Published feature status '
                f'(reason={reason}, revision={revision}, state='
                f"{navigation_system['state']}, follow={body_follow['state']}, "
                f"recharge={recharge['state']}, "
                f"mapping={mapping_session['state'] if mapping_session else 'omitted'}, "
                f"target={target_navigation['state'] if target_navigation else 'omitted'}, "
                f"readiness={vehicle_readiness['state']}, "
                f"availability={availability})."
            )
    # 2026-10-09: 会话收尾钩子。放在 return 之前，确保电量/会话状态已归一化。
    _autonav_after_session_tick(
        client, mapping_session, recharge, navigation_system)
    return snapshot


def refresh_body_follow_feature_after_service(
    client: mqtt.Client, service_path: str, reason: str
) -> None:
    """Publish a read-back request without treating a service ACK as final truth."""
    if service_path not in {
        '/voice_summon/set_body_follow_enabled',
        BODY_FOLLOW_STATUS_SERVICE_PATH,
    }:
        return
    publish_navigation_feature_status(client, reason, force=True)


def serialize_process_entry(entry: dict) -> dict:
    payload = {
        'taskId': entry['taskId'],
        'requestId': entry['requestId'],
        'package': entry['package'],
        'startedAt': entry['startedAt'],
        'targetKey': entry['targetKey'],
    }
    if entry['kind'] == 'launch':
        payload['launchFile'] = entry['entryName']
        if entry.get('videoSessionId'):
            # Keep videoSessionId for older cloud callers, but its semantics are
            # now the shared producer identity rather than the last viewer that
            # happened to request this launch.
            payload['videoSessionId'] = entry['videoSessionId']
            payload['producerSessionId'] = entry['videoSessionId']
            payload['videoLaunchMode'] = 'shared_producer'
    else:
        payload['nodeExecutable'] = entry['entryName']
        payload['oneShot'] = entry['oneShot']
    return payload


def refresh_state_locked() -> None:
    running_launches = [serialize_process_entry(entry) for entry in launch_processes.values()]
    running_nodes = [serialize_process_entry(entry) for entry in node_processes.values()]
    running_nodes.extend(serialize_process_entry(entry) for entry in one_shot_processes.values())

    state['runningLaunches'] = running_launches
    state['runningNodes'] = running_nodes

    if running_nodes:
        current = running_nodes[-1]
        state['currentTask'] = f"{current['package']}/{current['nodeExecutable']}"
        state['currentTaskType'] = 'node'
    elif running_launches:
        current = running_launches[-1]
        state['currentTask'] = f"{current['package']}/{current['launchFile']}"
        state['currentTaskType'] = 'launch'
    else:
        state['currentTask'] = '-'
        state['currentTaskType'] = '-'

    if running_launches or running_nodes or one_shot_processes:
        state['launchStatus'] = 'running'
    elif state['lastError']:
        state['launchStatus'] = 'failed'
    elif state['requestId']:
        state['launchStatus'] = 'stopped'
    else:
        state['launchStatus'] = 'idle'


def build_status_payload(message: str, **extra: object) -> dict:
    with state_lock:
        payload = {
            'deviceId': DEVICE_ID,
            'requestId': state['requestId'],
            'launchStatus': state['launchStatus'],
            'currentTask': state['currentTask'],
            'currentTaskType': state.get('currentTaskType', '-'),
            'lastError': state['lastError'],
            'runningLaunches': list(state.get('runningLaunches', [])),
            'runningNodes': list(state.get('runningNodes', [])),
            'message': message,
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
    payload.update(extra)
    return payload


def set_state(**kwargs: object) -> None:
    with state_lock:
        state.update(kwargs)
        refresh_state_locked()


def make_process_entry(
    kind: str,
    package_name: str,
    entry_name: str,
    request_id: str,
    process: subprocess.Popen,
    one_shot: bool = False,
    video_session_id: str = '',
    navigation_launch_lock: object | None = None,
) -> dict:
    entry = {
        'taskId': request_id,
        'requestId': request_id,
        'kind': kind,
        'package': package_name,
        'entryName': entry_name,
        'targetKey': build_target_key(package_name, entry_name),
        'process': process,
        'startedAt': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'startedAtMonotonic': time.monotonic(),
        'oneShot': one_shot,
        'videoSessionId': video_session_id,
        'healthConfirmed': False,
        '_navigationHealthTerminal': False,
        '_navigationPendingRequestIds': [],
        '_navigationLaunchLock': navigation_launch_lock,
    }
    if entry['targetKey'] == NAVIGATION_LAUNCH_TARGET:
        # Nav2 is always spawned with start_new_session=True. Persist that
        # identity so shutdown never falls back to a fuzzy process-name kill.
        entry.update({
            '_navigationProcessGroupId': process.pid,
            '_navigationSessionId': process.pid,
            '_navigationShutdownLock': threading.Lock(),
            '_navigationStopInProgress': False,
        })
    return entry


def release_process_entry_resources(entry: dict) -> None:
    release_navigation_launch_lock(entry.pop('_navigationLaunchLock', None))


def resolve_video_producer_session_id(payload: dict, request_id: object, *, allow_request_id: bool = True) -> str:
    """Return the stable shared-producer identity for a video launch.

    Cloud video viewers are intentionally not owners of the RTMP process. The
    bridge accepts the legacy videoSessionId/ownerTaskId fields so older cloud
    callers remain compatible, but stores exactly one producer identity for the
    lifetime of the launch.
    """
    return str(
        payload.get('producerSessionId') or payload.get('videoSessionId') or
        payload.get('ownerTaskId') or (request_id if allow_request_id else '') or ''
    ).strip()


def build_display_name(entry: dict) -> str:
    return f"{entry['package']}/{entry['entryName']}"


def _append_navigation_request_waiter_locked(entry: dict, request_id: str) -> None:
    pending = entry.setdefault('_navigationPendingRequestIds', [])
    if request_id not in pending:
        pending.append(request_id)


def add_navigation_request_waiter(entry: dict, request_id: str) -> bool:
    """Attach a request to a managed startup without racing its terminal state."""
    with state_lock:
        current = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
        if current is not entry or current.get('_navigationHealthTerminal') is True:
            return False
        _append_navigation_request_waiter_locked(current, request_id)
        return True


def _drain_navigation_request_waiters_locked(entry: dict) -> list[str]:
    request_ids = list(dict.fromkeys(
        str(item).strip()
        for item in entry.get('_navigationPendingRequestIds', [])
        if str(item).strip()
    ))
    entry['_navigationPendingRequestIds'] = []
    original_request_id = str(entry.get('requestId') or '').strip()
    if original_request_id and original_request_id not in request_ids:
        request_ids.append(original_request_id)
    return request_ids


def remove_process_entry(kind: str, target_key: str) -> dict | None:
    if kind == 'launch':
        return launch_processes.pop(target_key, None)
    if kind == 'node':
        return node_processes.pop(target_key, None)
    return one_shot_processes.pop(target_key, None)


def monitor_process(client: mqtt.Client, kind: str, target_key: str, pid: int) -> None:
    if kind == 'launch':
        registry = launch_processes
    elif kind == 'node':
        registry = node_processes
    else:
        registry = one_shot_processes

    with state_lock:
        entry = registry.get(target_key)
    if not entry or entry['process'].pid != pid:
        return

    return_code = entry['process'].wait()

    is_navigation_entry = (
        kind == 'launch' and target_key == NAVIGATION_LAUNCH_TARGET
    )
    navigation_stop_was_requested = False
    if is_navigation_entry:
        with state_lock:
            current_entry = registry.get(target_key)
            navigation_stop_was_requested = bool(
                current_entry is entry
                and current_entry.get('_navigationStopInProgress')
            )
        group_stopped, group_message = stop_navigation_process_entry(
            entry, request_lifecycle=False)
        if not group_stopped:
            entry['_navigationStopInProgress'] = False
            cleanup_error = (
                'Navigation launcher exited but its process group could not be '
                f'fully stopped: {group_message}'
            )
            terminal_request_ids = _mark_navigation_health_terminal(
                entry, ready=False, message=cleanup_error)
            with state_lock:
                current_entry = registry.get(target_key)
                if current_entry is entry:
                    current_entry['healthConfirmed'] = False
                    current_entry['healthFailure'] = cleanup_error
                    state['requestId'] = entry['requestId']
                    state['lastError'] = cleanup_error
                    refresh_state_locked()
            if terminal_request_ids is not None:
                publish_navigation_start_terminal(
                    client,
                    terminal_request_ids,
                    ready=False,
                    message=cleanup_error,
                    package_name=entry['package'],
                    launch_file=entry['entryName'],
                )
            else:
                publish_json(
                    client,
                    ERROR_TOPIC,
                    build_status_payload(
                        cleanup_error,
                        status='failed',
                        error=cleanup_error,
                        requestId=entry['requestId'],
                        requestTerminal=False,
                    ),
                )
            publish_navigation_feature_status(
                client, 'navigation_process_group_cleanup_failed', force=True)
            return

    navigation_exit_failure = ''
    navigation_exit_request_ids = []
    prior_navigation_health_failure = ''
    with state_lock:
        current_entry = registry.get(target_key)
        if not current_entry or current_entry['process'].pid != pid:
            refresh_state_locked()
            release_process_entry_resources(entry)
            return
        if is_navigation_entry:
            navigation_stop_was_requested = (
                navigation_stop_was_requested
                or current_entry.get('_navigationStopInProgress') is True
            )
        if (
            is_navigation_entry
            and current_entry.get('_navigationHealthTerminal') is True
            and current_entry.get('healthConfirmed') is not True
        ):
            prior_navigation_health_failure = str(
                current_entry.get('healthFailure')
                or state.get('lastError')
                or 'Navigation startup health confirmation failed.'
            )
        if (
            is_navigation_entry
            and current_entry.get('_navigationHealthTerminal') is not True
        ):
            navigation_exit_failure = (
                'Navigation launch exited before health confirmation '
                f'(code {return_code}).'
            )
            current_entry['_navigationHealthTerminal'] = True
            current_entry['healthConfirmed'] = False
            current_entry['healthConfirmedAt'] = ''
            current_entry['healthFailure'] = navigation_exit_failure
            navigation_exit_request_ids = (
                _drain_navigation_request_waiters_locked(current_entry)
            )
        registry.pop(target_key, None)
        state['requestId'] = entry['requestId']
        if navigation_exit_failure:
            state['lastError'] = navigation_exit_failure
        elif prior_navigation_health_failure:
            # Expected SIGINT/SIGTERM cleanup must not replace the health
            # blocker that caused the launch to be stopped.
            state['lastError'] = prior_navigation_health_failure
        elif return_code == 0:
            if kind == 'oneshot':
                state['lastError'] = ''
            elif not state['lastError']:
                state['lastError'] = ''
        else:
            state['lastError'] = f"{kind} exited with code {return_code}: {build_display_name(entry)}"
        refresh_state_locked()

    release_process_entry_resources(entry)

    if kind == 'launch' and target_key == NAVIGATION_LAUNCH_TARGET:
        if navigation_stop_was_requested:
            # The synchronous stop owner publishes the verified terminal after
            # it observes both the root exit and an empty PGID.
            return
        if navigation_exit_failure:
            publish_navigation_start_terminal(
                client,
                navigation_exit_request_ids,
                ready=False,
                message=navigation_exit_failure,
                package_name=entry['package'],
                launch_file=entry['entryName'],
            )
            publish_navigation_feature_status(
                client, 'navigation_launch_exited_before_ready', force=True
            )
        else:
            # Startup ready/failed is a one-shot request terminal.  A later
            # process exit must not overwrite it with a second terminal-looking
            # status; the feature snapshot still records the lifecycle change.
            publish_navigation_feature_status(
                client,
                'navigation_launch_exited'
                if return_code == 0 else 'navigation_launch_failed',
                force=True,
            )
        return

    if return_code == 0:
        if kind == 'oneshot':
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    f"One-shot node completed successfully: {build_display_name(entry)}",
                    status='completed',
                ),
            )
        else:
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    f"{kind.title()} exited successfully: {build_display_name(entry)}",
                    status='stopped',
                ),
            )
        if kind == 'launch' and target_key == LOCAL_HLS_LAUNCH_TARGET:
            schedule_local_hls_preview_restart(client, 'local_preview_exited')
        elif kind == 'launch' and target_key == RTMP_LAUNCH_TARGET:
            schedule_rtmp_publisher_restart(client, 'rtmp_publisher_exited')
        return

    publish_json(
        client,
        ERROR_TOPIC,
        build_status_payload(
            f"{kind.title()} exited with code {return_code}: {build_display_name(entry)}",
            error=state['lastError'],
            status='failed',
        ),
    )
    if kind == 'launch' and target_key == LOCAL_HLS_LAUNCH_TARGET:
        schedule_local_hls_preview_restart(client, 'local_preview_failed')
    elif kind == 'launch' and target_key == RTMP_LAUNCH_TARGET:
        schedule_rtmp_publisher_restart(client, 'rtmp_publisher_failed')


def start_process_monitor(client: mqtt.Client, kind: str, entry: dict) -> None:
    threading.Thread(
        target=monitor_process,
        args=(client, kind, entry['targetKey'], entry['process'].pid),
        daemon=True,
    ).start()


def navigation_start_health_timeout_reached(
    now_monotonic: float,
    started_at_monotonic: float,
    timeout_sec: float,
) -> bool:
    return now_monotonic - started_at_monotonic >= timeout_sec


# 2026-09-16 (TF_GUARD_GRACE): transient TF-guard states that the robot-agent
# reports while a freshly spawned navigation launch is still warming up AMCL.
# Both are retryable and must not terminate a healthy start on first sight.
TF_GUARD_TRANSIENT_GRACE_SEC = 25.0
TF_GUARD_TRANSIENT_MAX_AGE_SEC = 90.0
TF_GUARD_TRANSIENT_POLL_SEC = 2.0
TF_GUARD_TRANSIENT_MARKERS = (
    'tf guard anomaly',
    'tf guard status is missing or stale',
)


def _tf_guard_transient_failure(text: object) -> bool:
    """Return True when a terminal failure is a TF-guard warm-up transient."""
    lowered = str(text or '').strip().lower()
    if not lowered:
        return False
    return any(marker in lowered for marker in TF_GUARD_TRANSIENT_MARKERS)


def navigation_health_probe_sample() -> tuple[bool, str, str]:
    """Return (ready, terminal_failure, current_blocker)."""
    navigation_system, availability = navigation_system_state_from_probe()
    if availability != 'ready':
        blocker = str(
            navigation_system.get('lastError')
            or 'Navigation health probe is unavailable.'
        )
        return False, '', blocker
    health_ready = (
        navigation_system.get('state') == 'ready'
        and navigation_system.get('effectiveEnabled') is True
        and navigation_system.get('safetyReady') is True
    )
    if health_ready:
        return True, '', ''
    blocker = str(
        navigation_system.get('lastError')
        or navigation_system.get('safetyError')
        or navigation_system.get('message')
        or 'Navigation health checks are still pending.'
    )
    failure = (
        blocker if navigation_system.get('state') == 'failed' else ''
    )
    return False, failure, blocker


def publish_navigation_start_terminal(
    client: mqtt.Client,
    request_ids: list[str],
    *,
    ready: bool,
    message: str,
    package_name: str,
    launch_file: str,
) -> None:
    """Fan one authoritative terminal out to every accepted request ID."""
    for request_id in dict.fromkeys(request_ids):
        payload = build_status_payload(
            message,
            status='ready' if ready else 'failed',
            error='' if ready else message,
            action='start_launch',
            package=package_name,
            launchFile=launch_file,
            requestId=request_id,
            healthConfirmed=ready,
            requestTerminal=True,
        )
        publish_json(client, STATUS_TOPIC, payload)
        if not ready:
            publish_json(client, ERROR_TOPIC, payload)


def _mark_navigation_health_terminal(
    entry: dict,
    *,
    ready: bool,
    message: str,
) -> list[str] | None:
    with state_lock:
        current = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
        if (
            current is not entry
            or not navigation_process_entry_active(current)
            or current.get('_navigationHealthTerminal') is True
        ):
            return None
        current['_navigationHealthTerminal'] = True
        current['healthConfirmed'] = ready
        current['healthConfirmedAt'] = (
            time.strftime('%Y-%m-%dT%H:%M:%S') if ready else ''
        )
        current['healthFailure'] = '' if ready else message
        state['requestId'] = entry['requestId']
        state['lastError'] = '' if ready else message
        request_ids = _drain_navigation_request_waiters_locked(current)
        refresh_state_locked()
        return request_ids


def cleanup_navigation_entry_after_health_failure(
    client: mqtt.Client,
    entry: dict,
) -> None:
    """Stop only the managed launch that failed its startup health contract."""
    with state_lock:
        current = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
        if current is not entry or not navigation_process_entry_active(current):
            return
        current['_navigationHealthCleanupRequested'] = True
        current['_navigationStopInProgress'] = True
    try:
        stop_failed_navigation_process_entry(entry)
    except Exception as exc:
        cleanup_error = f'Navigation health cleanup failed: {exc}'
        with state_lock:
            state['requestId'] = entry['requestId']
            state['lastError'] = cleanup_error
            refresh_state_locked()
        publish_json(
            client,
            ERROR_TOPIC,
            build_status_payload(
                cleanup_error,
                status='failed',
                error=cleanup_error,
                requestId=entry['requestId'],
                requestTerminal=False,
            ),
        )
        return

    # The cleanup owner already verified both root exit and an empty PGID.
    # Remove the exact entry and release its inherited flock before returning,
    # so an immediate retry cannot observe a dead registry entry with a busy
    # lock while the process-monitor thread is still being scheduled.
    removed_entry = None
    with state_lock:
        current = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
        if current is entry:
            removed_entry = launch_processes.pop(
                NAVIGATION_LAUNCH_TARGET, None)
            refresh_state_locked()
    if removed_entry is not None:
        release_process_entry_resources(removed_entry)


def fail_managed_navigation_start_if_timed_out(
    client: mqtt.Client,
    entry: dict,
    now_monotonic: float,
    last_blocker: str,
) -> bool:
    if not navigation_start_health_timeout_reached(
        now_monotonic,
        entry['startedAtMonotonic'],
        NAVIGATION_SYSTEM_STARTUP_TIMEOUT,
    ):
        return False
    failure = (
        'Navigation startup health confirmation timed out after '
        f'{NAVIGATION_SYSTEM_STARTUP_TIMEOUT:.1f}s: {last_blocker}'
    )
    request_ids = _mark_navigation_health_terminal(
        entry, ready=False, message=failure
    )
    if request_ids is None:
        return True
    publish_navigation_start_terminal(
        client,
        request_ids,
        ready=False,
        message=failure,
        package_name=entry['package'],
        launch_file=entry['entryName'],
    )
    publish_navigation_feature_status(
        client, 'navigation_health_timeout', force=True
    )
    cleanup_navigation_entry_after_health_failure(client, entry)
    return True


def monitor_navigation_start_health(client: mqtt.Client, entry: dict) -> None:
    deadline = entry['startedAtMonotonic'] + NAVIGATION_SYSTEM_STARTUP_TIMEOUT
    last_blocker = 'Waiting for the navigation health probe.'
    ready_since = None
    while not stop_event.is_set():
        with state_lock:
            current = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
            if current is not entry or entry['process'].poll() is not None:
                return

        if fail_managed_navigation_start_if_timed_out(
            client, entry, time.monotonic(), last_blocker
        ):
            return
        health_ready, terminal_failure, blocker = navigation_health_probe_sample()
        if not health_ready:
            last_blocker = blocker
        if fail_managed_navigation_start_if_timed_out(
            client, entry, time.monotonic(), last_blocker
        ):
            return
        if health_ready:
            observed_ready_at = time.monotonic()
            if ready_since is None:
                ready_since = observed_ready_at
            if observed_ready_at - ready_since >= NAVIGATION_HEALTH_STABLE_SEC:
                message = (
                    'Navigation is ready: action server, localization, guard, '
                    'motion interlock and TF health are confirmed.'
                )
                request_ids = _mark_navigation_health_terminal(
                    entry, ready=True, message=message
                )
                if request_ids is None:
                    return
                publish_navigation_start_terminal(
                    client,
                    request_ids,
                    ready=True,
                    message=message,
                    package_name=entry['package'],
                    launch_file=entry['entryName'],
                )
                publish_navigation_feature_status(
                    client, 'navigation_health_ready', force=True
                )
                return
            last_blocker = (
                'Navigation health is ready but still inside the '
                f'{NAVIGATION_HEALTH_STABLE_SEC:.1f}s stability window.'
            )
        else:
            ready_since = None
        if terminal_failure:
            # 2026-09-16 (TF_GUARD_GRACE): right after this bridge spawns Nav2 the
            # robot-agent's TF guard reports 'AMCL TF guard anomaly is asserted' for
            # a few seconds until AMCL republishes map->odom.  Treat that specific
            # transient as retryable once instead of failing the start immediately
            # (observed 3x: the immediate retry always succeeded).
            if _tf_guard_transient_failure(terminal_failure):
                # 2026-09-16 (TF_GUARD_GRACE): a navigation launch spawned by this
                # bridge always passes through a TF-guard warm-up window.  The
                # agent reports either 'AMCL TF guard status is missing or stale'
                # (guard node not up yet) or 'AMCL TF guard anomaly is asserted'
                # (guard up, AMCL has not republished map->odom yet); both clear
                # on their own within a few seconds.  Failing the start on the
                # first sample killed a healthy launch (observed 3x) and forced
                # the operator to press start twice.  Hold the terminal verdict
                # for a bounded grace measured from launch start: safetyReady
                # stays False the whole time, so motion authority is unchanged.
                grace_deadline = min(
                    time.monotonic() + TF_GUARD_TRANSIENT_GRACE_SEC,
                    entry['startedAtMonotonic'] + TF_GUARD_TRANSIENT_MAX_AGE_SEC,
                )
                while not stop_event.is_set() and time.monotonic() < grace_deadline:
                    time.sleep(TF_GUARD_TRANSIENT_POLL_SEC)
                    retry_ready, retry_terminal, retry_blocker = navigation_health_probe_sample()
                    if retry_ready:
                        terminal_failure = None
                        blocker = retry_blocker or blocker
                        break
                    if retry_terminal and not _tf_guard_transient_failure(retry_terminal):
                        terminal_failure = retry_terminal
                        break
                    terminal_failure = retry_terminal or terminal_failure
        if terminal_failure:
            request_ids = _mark_navigation_health_terminal(
                entry, ready=False, message=terminal_failure
            )
            if request_ids is None:
                return
            publish_navigation_start_terminal(
                client,
                request_ids,
                ready=False,
                message=terminal_failure,
                package_name=entry['package'],
                launch_file=entry['entryName'],
            )
            publish_navigation_feature_status(
                client, 'navigation_health_failed', force=True
            )
            cleanup_navigation_entry_after_health_failure(client, entry)
            return

        now = time.monotonic()
        stop_event.wait(max(
            0.0, min(NAVIGATION_HEALTH_POLL_INTERVAL, deadline - now)
        ))


def start_navigation_health_monitor(client: mqtt.Client, entry: dict) -> None:
    threading.Thread(
        target=monitor_navigation_start_health,
        args=(client, entry),
        daemon=True,
    ).start()


def _finish_navigation_request_observer(
    observer: dict,
    *,
    ready: bool,
    message: str,
) -> bool:
    with state_lock:
        request_id = observer['requestId']
        if navigation_request_observers.get(request_id) is not observer:
            return False
        navigation_request_observers.pop(request_id, None)
        state['requestId'] = request_id
        state['lastError'] = '' if ready else message
        refresh_state_locked()
        return True


def fail_reused_navigation_start_if_timed_out(
    client: mqtt.Client,
    observer: dict,
    now_monotonic: float,
    last_blocker: str,
) -> bool:
    if not navigation_start_health_timeout_reached(
        now_monotonic,
        observer['startedAtMonotonic'],
        NAVIGATION_SYSTEM_STARTUP_TIMEOUT,
    ):
        return False
    failure = (
        'Navigation startup health confirmation timed out after '
        f'{NAVIGATION_SYSTEM_STARTUP_TIMEOUT:.1f}s: {last_blocker}'
    )
    if not _finish_navigation_request_observer(
        observer, ready=False, message=failure
    ):
        return True
    publish_navigation_start_terminal(
        client,
        [observer['requestId']],
        ready=False,
        message=failure,
        package_name=observer['package'],
        launch_file=observer['launchFile'],
    )
    return True


def monitor_reused_navigation_start_health(
    client: mqtt.Client,
    observer: dict,
) -> None:
    """Give an externally/previously owned start request its own terminal."""
    ready_since = None
    last_blocker = 'Waiting for the navigation health probe.'
    deadline = observer['startedAtMonotonic'] + NAVIGATION_SYSTEM_STARTUP_TIMEOUT
    while not stop_event.is_set():
        if fail_reused_navigation_start_if_timed_out(
            client, observer, time.monotonic(), last_blocker
        ):
            return
        health_ready, terminal_failure, blocker = navigation_health_probe_sample()
        now = time.monotonic()
        if not health_ready:
            last_blocker = blocker
        if fail_reused_navigation_start_if_timed_out(
            client, observer, now, last_blocker
        ):
            return
        if health_ready:
            if ready_since is None:
                ready_since = now
            if now - ready_since >= NAVIGATION_HEALTH_STABLE_SEC:
                ready_message = (
                    'Navigation is already running and health-confirmed.'
                )
                if not _finish_navigation_request_observer(
                    observer, ready=True, message=ready_message
                ):
                    return
                publish_navigation_start_terminal(
                    client,
                    [observer['requestId']],
                    ready=True,
                    message=ready_message,
                    package_name=observer['package'],
                    launch_file=observer['launchFile'],
                )
                return
            last_blocker = 'Navigation health is inside the stability window.'
        else:
            ready_since = None
        if terminal_failure:
            if not _finish_navigation_request_observer(
                observer, ready=False, message=terminal_failure
            ):
                return
            publish_navigation_start_terminal(
                client,
                [observer['requestId']],
                ready=False,
                message=terminal_failure,
                package_name=observer['package'],
                launch_file=observer['launchFile'],
            )
            return
        stop_event.wait(max(
            0.0, min(NAVIGATION_HEALTH_POLL_INTERVAL, deadline - now)
        ))


def start_reused_navigation_health_monitor(
    client: mqtt.Client,
    request_id: str,
    package_name: str,
    launch_file: str,
) -> None:
    observer = {
        'requestId': request_id,
        'package': package_name,
        'launchFile': launch_file,
        'startedAtMonotonic': time.monotonic(),
    }
    with state_lock:
        if request_id in navigation_request_observers:
            return
        navigation_request_observers[request_id] = observer
    threading.Thread(
        target=monitor_reused_navigation_start_health,
        args=(client, observer),
        daemon=True,
    ).start()


def stop_process_entry(entry: dict) -> None:
    """Bounded, process-group aware shutdown for one managed launch entry.

    2026-09-16 (FAST_STOP): the previous body had a hardcoded 10s + 5s ladder
    and signalled only the launcher process, so (a) the operator-visible stop
    transition stayed up for ~15s whenever a child ignored SIGINT, and (b)
    children that outlived the launcher were never signalled at all.  Escalate
    the already validated navigation process group with the same bounded
    schedule the navigation stop path uses, with SIGKILL as the final stage so
    no Nav2 node can survive a stop request.  Motion authority is unchanged:
    every branch tears the entry down, and a stopped Nav2 publishes no velocity.
    """
    process = entry['process']
    process_group_id = entry.get('_navigationProcessGroupId')
    session_id = entry.get('_navigationSessionId')
    if type(process_group_id) is int and type(session_id) is int:
        try:
            stopped, _message = stop_navigation_process_group(
                process.pid, process_group_id, session_id, process)
        except Exception as exc:  # noqa: BLE001 - never wedge the stop path
            stopped = False
            print(f'process entry group stop failed: {exc}', flush=True)
        if stopped or process.poll() is not None:
            return
        try:
            os.killpg(process_group_id, NAVIGATION_SIGKILL)
        except (OSError, ProcessLookupError):
            pass
        try:
            process.kill()
            process.wait(timeout=2)
        except (subprocess.TimeoutExpired, OSError):
            pass
        return

    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
                process.wait(timeout=2)
            except (subprocess.TimeoutExpired, OSError):
                pass


def navigation_process_entry_active(entry: dict) -> bool:
    """Fail closed while either the Nav2 root or any owned PGID member lives."""
    process = entry['process']
    if process.poll() is None:
        return True
    process_group_id = entry.get('_navigationProcessGroupId')
    session_id = entry.get('_navigationSessionId')
    if type(process_group_id) is not int or type(session_id) is not int:
        return False
    stopped, inspection_error = navigation_process_group_stopped(
        process.pid,
        process_group_id,
        session_id,
        process,
    )
    return not stopped or bool(inspection_error)


NAVIGATION_ZOMBIE_LOCK_PATH = '/tmp/wheeltec_nav2-%d.launch.lock'


def _nav2_container_children(pid: int) -> list:
    """PIDs of live component_container processes owned by this launch wrapper."""
    found = []
    try:
        with open('/proc/%d/task/%d/children' % (pid, pid), 'r', encoding='ascii') as handle:
            direct = handle.read().split()
    except OSError:
        direct = []
    for child in direct:
        try:
            with open('/proc/%s/cmdline' % child, 'rb') as handle:
                argv0 = handle.read().split(b'\0', 1)[0].decode('utf-8', 'replace')
        except OSError:
            continue
        if os.path.basename(argv0).startswith('component_container'):
            found.append(int(child))
    return found


def live_wrapper_container(pid: int) -> bool:
    """True when the navigation launch wrapper still owns a live container."""
    return bool(_nav2_container_children(pid))


def clean_orphaned_navigation_lock(pid: int) -> str:
    """Remove the singleton lock only when it still names a dead wrapper pid.

    Real shape (2026-10-06): the launch wrapper exited but
    /tmp/wheeltec_nav2-<uid>.launch.lock still contained its pid, so a later
    start had to be rescued by hand.  Deleting a lock that names a *live*
    process would be unsafe, hence the liveness re-check here.
    """
    if _process_is_alive(pid):
        return 'wrapper still alive; lock kept'
    lock_path = NAVIGATION_ZOMBIE_LOCK_PATH % os.getuid()
    try:
        with open(lock_path, 'r', encoding='ascii') as handle:
            holder = int(handle.read(64).strip())
    except Exception as exc:
        return 'no usable singleton lock (%s)' % exc
    if holder != pid:
        return 'singleton holder is %s, not %s' % (holder, pid)
    try:
        os.unlink(lock_path)
    except OSError as exc:
        return 'unlink failed: %s' % exc
    return 'removed orphaned lock naming dead pid %s' % pid


def reap_containerless_navigation_wrapper(pid: int) -> str:
    """Reclaim a wrapper that outlived its containers, then free the singleton.

    Only the process that actually owns the wheeltec_nav2 singleton lock is
    signalled, so an unrelated launcher can never be killed from here.
    """
    if os.environ.get('STEMM_NAV2_ZOMBIE_RECLAIM', '1') == '0':
        return 'reclaim disabled by STEMM_NAV2_ZOMBIE_RECLAIM'
    if not _process_is_alive(pid):
        return 'wrapper already gone'
    lock_path = NAVIGATION_ZOMBIE_LOCK_PATH % os.getuid()
    try:
        with open(lock_path, 'r', encoding='ascii') as handle:
            holder = int(handle.read(64).strip())
    except Exception as exc:
        return 'no usable singleton lock (%s)' % exc
    if holder != pid:
        return 'singleton holder is %s, not %s' % (holder, pid)
    try:
        os.kill(pid, signal.SIGKILL)
    except OSError as exc:
        return 'kill failed: %s' % exc
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not _process_is_alive(pid):
            break
        time.sleep(0.1)
    try:
        os.unlink(lock_path)
    except OSError:
        pass
    return 'reclaimed wrapper pid %s' % pid


def stop_navigation_process_entry(
    entry: dict,
    *,
    request_lifecycle: bool,
) -> tuple[bool, str]:
    """Serialize graceful shutdown and bounded escalation for owned Nav2."""
    process = entry['process']
    process_group_id = entry.get('_navigationProcessGroupId')
    session_id = entry.get('_navigationSessionId')
    shutdown_lock = entry.get('_navigationShutdownLock')
    if (
        type(process_group_id) is not int
        or type(session_id) is not int
        or shutdown_lock is None
    ):
        return False, 'Managed navigation process-group identity is unavailable.'
    with shutdown_lock:
        entry['_navigationStopInProgress'] = True
        lifecycle_outcomes = (
            request_navigation_lifecycle_shutdown()
            if request_lifecycle else []
        )
        stopped, message = stop_navigation_process_group(
            process.pid,
            process_group_id,
            session_id,
            process,
        )
        if lifecycle_outcomes:
            message = (
                f'{message} Lifecycle shutdown: '
                f'{", ".join(lifecycle_outcomes)}'
            )
        return stopped, message


def stop_failed_navigation_process_entry(entry: dict) -> None:
    """Bounded escalation for the dedicated owned Nav2 process group."""
    stopped, message = stop_navigation_process_entry(
        entry, request_lifecycle=not NAVIGATION_STOP_FAST_PATH)
    if not stopped:
        entry['_navigationStopInProgress'] = False
        raise RuntimeError(message)


def launch_command(package_name: str, launch_file: str, args: dict) -> list[str]:
    command = ['ros2', 'launch', package_name, launch_file]
    for key, value in args.items():
        command.append(f'{key}:={value}')
    return command


def node_command(package_name: str, executable_name: str, args: dict) -> list[str]:
    command = ['ros2', 'run', package_name, executable_name]
    for key, value in args.items():
        command.extend([f'--{key}', str(value)])
    return command


def service_command(service_path: str, service_type: str, service_args: dict) -> list[str]:
    return ['ros2', 'service', 'call', service_path, service_type, json.dumps(service_args, ensure_ascii=True)]


def parse_service_success(service_type: str, details: str) -> bool:
    if service_type not in {'std_srvs/srv/Trigger', 'std_srvs/srv/SetBool'}:
        return True
    normalized = details.lower()
    if 'success=false' in normalized or 'success: false' in normalized:
        return False
    return 'success=true' in normalized or 'success: true' in normalized


def start_launch(client: mqtt.Client, payload: dict) -> None:
    launch_file = (payload.get('launchFile') or DEFAULT_LAUNCH_FILE).strip()
    package_name = (payload.get('package') or DEFAULT_PACKAGE).strip()
    launch_args = payload.get('args') or {}
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, launch_file)
    is_video_launch = target_key in VIDEO_LAUNCH_TARGETS
    is_navigation_launch = target_key == NAVIGATION_LAUNCH_TARGET
    is_cartographer_session = (
        package_name == 'stemm_cartographer_exploration'
        and launch_file == 'stemm_cartographer_session.launch.py'
    )
    producer_session_id = resolve_video_producer_session_id(payload, request_id) if is_video_launch else ''

    if is_navigation_launch:
        request_id = str(request_id or '').strip() or (
            f'navigation-{uuid.uuid4()}'
        )
        # Preserve the canonical ID for the outer exception boundary. Validation
        # and Popen failures must still resolve this exact request terminally.
        payload['requestId'] = request_id

    if target_key not in ALLOWED_LAUNCH_TARGETS:
        raise ValueError(f'Launch target not allowed: {package_name}/{launch_file}')
    if not isinstance(launch_args, dict):
        raise ValueError('args must be an object')
    if is_video_launch and not producer_session_id:
        raise ValueError('producerSessionId or requestId is required for a video launch')

    if is_navigation_launch:
        _, _, mapping_session_now, _, _ = auxiliary_feature_status()
        if mapping_owns_vehicle(mapping_session_now):
            raise ValueError(
                'Cartographer mapping currently owns vehicle motion; '
                'stop mapping before starting navigation.')

    if is_cartographer_session:
        _, _, _, _, vehicle_readiness = auxiliary_feature_status()
        if vehicle_readiness.get('mappingStartAllowed') is not True:
            message = str(
                vehicle_readiness.get('message')
                or '车端初始化中，暂不可开始建图'
            )
            set_state(requestId=request_id, lastError=message)
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    message,
                    status='failed',
                    action='start_launch',
                    package=package_name,
                    launchFile=launch_file,
                    requestId=request_id,
                ),
            )
            publish_navigation_feature_status(
                client,
                'mapping_start_vehicle_initializing',
                force=True,
            )
            return

    if is_navigation_launch:
        _, _, mapping_session, _, _ = auxiliary_feature_status()
        mapping_start_allowed, mapping_blocked_reason = (
            mapping_allows_navigation_start(mapping_session)
        )
        if not mapping_start_allowed:
            message = (
                'Navigation launch was blocked because Cartographer mapping is '
                f'not confirmed idle: {mapping_blocked_reason}'
            )
            set_state(requestId=request_id, lastError=message)
            publish_navigation_start_terminal(
                client,
                [request_id],
                ready=False,
                message=message,
                package_name=package_name,
                launch_file=launch_file,
            )
            publish_navigation_feature_status(
                client,
                'navigation_start_mapping_interlock',
                force=True,
            )
            return
        stop_transition = get_navigation_stop_transition()
        with state_lock:
            stopping_entry = launch_processes.get(NAVIGATION_LAUNCH_TARGET)
            managed_stop_in_progress = bool(
                stopping_entry
                and stopping_entry.get('_navigationStopInProgress')
            )
        if stop_transition is not None or managed_stop_in_progress:
            message = (
                'Navigation launch was rejected because the previous Nav2 '
                'process group is still shutting down.'
            )
            set_state(requestId=request_id, lastError=message)
            publish_navigation_start_terminal(
                client,
                [request_id],
                ready=False,
                message=message,
                package_name=package_name,
                launch_file=launch_file,
            )
            publish_navigation_feature_status(
                client, 'navigation_start_during_shutdown', force=True)
            return
        navigation_system, availability = navigation_system_state_from_probe()
        navigation_ready = (
            availability == 'ready'
            and navigation_system['state'] == 'ready'
            and navigation_system['effectiveEnabled'] is True
            and navigation_system['safetyReady'] is True
        )
        navigation_starting = (
            availability == 'ready'
            and navigation_system['state'] == 'starting'
            and (
                navigation_system['launchObserved'] is True
                or navigation_system['actionServerReady'] is True
            )
        )
        navigation_unhealthy_observed = (
            availability == 'ready'
            and navigation_system['state'] == 'failed'
            and navigation_system['launchObserved'] is True
        )
        if navigation_ready or navigation_starting or navigation_unhealthy_observed:
            preflight_error = (
                str(navigation_system.get('lastError') or '')
                if navigation_unhealthy_observed else ''
            )
            set_state(requestId=request_id, lastError=preflight_error)
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    'Navigation system is already ready; launch was not duplicated.'
                    if navigation_ready
                    else (
                        'Navigation system is already starting; launch was not duplicated.'
                        if navigation_starting
                        else 'Navigation system is unhealthy; launch was not duplicated.'
                    ),
                    status=(
                        'failed' if navigation_unhealthy_observed
                        else ('ready' if navigation_ready else 'starting')
                    ),
                    action='start_launch',
                    package=package_name,
                    launchFile=launch_file,
                    requestId=request_id,
                    reused=True,
                    alreadyReady=navigation_ready,
                    alreadyStarting=navigation_starting,
                    alreadyFailed=navigation_unhealthy_observed,
                    healthConfirmed=navigation_ready,
                    requestTerminal=not navigation_starting,
                ),
            )
            if navigation_starting:
                managed_waiter_attached = False
                with state_lock:
                    managed_entry = launch_processes.get(
                        NAVIGATION_LAUNCH_TARGET)
                    if (
                        managed_entry is not None
                        and navigation_process_entry_active(managed_entry)
                        and managed_entry.get('_navigationHealthTerminal')
                        is not True
                    ):
                        _append_navigation_request_waiter_locked(
                            managed_entry, request_id
                        )
                        managed_waiter_attached = True
                if not managed_waiter_attached:
                    start_reused_navigation_health_monitor(
                        client, request_id, package_name, launch_file
                    )
            publish_navigation_feature_status(client, 'navigation_start_preflight', force=True)
            return

    reused_navigation_starting_ack_published = False
    with state_lock:
        existing_entry = launch_processes.get(target_key)
        if is_navigation_launch:
            # 2026-10-06: the launch root alone is not proof of life.  A wrapper
            # that outlived its containers used to look "active" here, so every
            # start_launch was answered with reused=True and no fresh Nav2 could
            # ever come up.  Treat container-less wrappers as dead and reclaim.
            existing_entry_active = bool(existing_entry) and (
                navigation_process_entry_active(existing_entry)
                and live_wrapper_container(existing_entry['process'].pid)
            )
            wrapper_exited = (
                bool(existing_entry) and existing_entry['process'].poll() is not None
            )
            if bool(existing_entry) and not existing_entry_active and (
                    existing_entry['process'].poll() is None or wrapper_exited):
                # 2026-10-06: two shapes of the same fault.
                #   (a) wrapper still alive, owns no container -> reap it.
                #   (b) wrapper already exited but the singleton lock still names
                #       it -> drop the orphaned lock.
                if wrapper_exited:
                    reap_message = clean_orphaned_navigation_lock(
                        existing_entry['process'].pid)
                else:
                    reap_message = reap_containerless_navigation_wrapper(
                        existing_entry['process'].pid)
                print(
                    '[bridge] Navigation launch wrapper pid %s owns no live '
                    'container (%s); starting a fresh Nav2 launch instead of '
                    'reusing it.' % (existing_entry['process'].pid, reap_message),
                    flush=True)
                try:
                    launch_processes.pop(target_key, None)
                except Exception:
                    pass
                existing_entry = None
        else:
            existing_entry_active = bool(existing_entry) and (
                existing_entry['process'].poll() is None
            )
        if existing_entry_active:
            if is_navigation_launch:
                navigation_terminal_failure = (
                    existing_entry.get('_navigationHealthTerminal') is True
                    and existing_entry.get('healthConfirmed') is not True
                )
                if existing_entry.get('_navigationHealthTerminal') is not True:
                    # The health thread uses the same lock to drain waiters. Send
                    # this request's nonterminal ACK before making it visible to
                    # that thread, so a ready/failed terminal can never overtake
                    # the request's starting state on STATUS_TOPIC.
                    publish_json(
                        client,
                        STATUS_TOPIC,
                        build_status_payload(
                            'Navigation launch is already managed: '
                            f'{package_name}/{launch_file}',
                            status='starting',
                            action='start_launch',
                            package=package_name,
                            launchFile=launch_file,
                            requestId=request_id,
                            reused=True,
                            alreadyStarting=True,
                            alreadyReady=False,
                            alreadyFailed=False,
                            healthConfirmed=False,
                            requestTerminal=False,
                        ),
                    )
                    _append_navigation_request_waiter_locked(
                        existing_entry, request_id
                    )
                    reused_navigation_starting_ack_published = True
                state['lastError'] = (
                    str(existing_entry.get('healthFailure') or '')
                    if navigation_terminal_failure else ''
                )
                refresh_state_locked()
                reused_navigation_entry = dict(existing_entry)
                reused_entry = None
            elif not is_video_launch:
                raise RuntimeError(f'Launch task already running: {package_name}/{launch_file}')
            else:
                # Video starts are idempotent. Do not replace the active producer
                # identity with a later viewer/request: all viewers attach to this
                # one RTMP publisher until an explicit management force-stop.
                state['lastError'] = ''
                refresh_state_locked()
                reused_entry = dict(existing_entry)
                reused_navigation_entry = None
        else:
            reused_entry = None
            reused_navigation_entry = None

    if reused_navigation_entry:
        health_confirmed = reused_navigation_entry.get('healthConfirmed') is True
        health_failed = (
            reused_navigation_entry.get('_navigationHealthTerminal') is True
            and not health_confirmed
        )
        reuse_message = (
            str(reused_navigation_entry.get('healthFailure') or '')
            if health_failed else (
                f'Navigation launch is already managed: {package_name}/{launch_file}'
            )
        )
        if not reused_navigation_starting_ack_published:
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    reuse_message,
                    status=(
                        'failed' if health_failed
                        else ('ready' if health_confirmed else 'starting')
                    ),
                    action='start_launch',
                    package=package_name,
                    launchFile=launch_file,
                    requestId=request_id,
                    reused=True,
                    alreadyStarting=not health_confirmed and not health_failed,
                    alreadyReady=health_confirmed,
                    alreadyFailed=health_failed,
                    healthConfirmed=health_confirmed,
                    requestTerminal=health_confirmed or health_failed,
                ),
            )
        publish_navigation_feature_status(client, 'navigation_bridge_launch_reused', force=True)
        return

    if reused_entry:
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(
                f'Video launch already running; reused: {package_name}/{launch_file}',
                status='running',
                reused=True,
                videoSessionId=reused_entry.get('videoSessionId', ''),
                producerSessionId=reused_entry.get('videoSessionId', ''),
                requestedProducerSessionId=producer_session_id,
                videoLaunchMode='shared_producer',
                requestId=request_id,
            ),
        )
        return

    navigation_launch_lock = None
    if is_navigation_launch:
        navigation_pids, process_lookup_error = find_navigation_launch_pids()
        if process_lookup_error:
            set_state(
                requestId=request_id,
                lastError=process_lookup_error,
                launchStatus='failed',
            )
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    process_lookup_error,
                    status='failed',
                    action='start_launch',
                    package=package_name,
                    launchFile=launch_file,
                    requestId=request_id,
                    healthConfirmed=False,
                    requestTerminal=True,
                ),
            )
            return
        if navigation_pids:
            message = (
                'An exact navigation launcher is already running; duplicate '
                f'launch was suppressed (pid(s) {navigation_pids}).'
            )
            set_state(requestId=request_id, lastError='')
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    message,
                    status='starting',
                    action='start_launch',
                    package=package_name,
                    launchFile=launch_file,
                    requestId=request_id,
                    reused=True,
                    alreadyStarting=True,
                    healthConfirmed=False,
                    requestTerminal=False,
                ),
            )
            start_reused_navigation_health_monitor(
                client, request_id, package_name, launch_file
            )
            publish_navigation_feature_status(
                client, 'navigation_exact_launch_reused', force=True
            )
            return
        navigation_launch_lock, lock_error = acquire_navigation_launch_lock(
            f'mqtt:{request_id or "unknown"}'
        )
        if navigation_launch_lock is None:
            lock_busy = 'already owned' in lock_error
            set_state(
                requestId=request_id,
                lastError='' if lock_busy else lock_error,
                launchStatus='failed' if not lock_busy else 'starting',
            )
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    lock_error,
                    status='starting' if lock_busy else 'failed',
                    action='start_launch',
                    package=package_name,
                    launchFile=launch_file,
                    requestId=request_id,
                    reused=lock_busy,
                    alreadyStarting=lock_busy,
                    healthConfirmed=False,
                    requestTerminal=not lock_busy,
                ),
            )
            if lock_busy:
                start_reused_navigation_health_monitor(
                    client, request_id, package_name, launch_file
                )
            publish_navigation_feature_status(
                client, 'navigation_single_instance_lock', force=True
            )
            return

    # Persist and pass the identity before spawning the runner.  This prevents
    # a rapid second mapping session from inheriting the prior task ID.
    if is_cartographer_session:
        persist_cartographer_task_id(request_id)

    command = launch_command(package_name, launch_file, launch_args)
    launch_environment = os.environ.copy()
    if is_cartographer_session:
        launch_environment['STEMM_CARTOGRAPHER_EXPECTED_TASK_ID'] = request_id
    # 2026-10-06: 导航 launch 包装同样忽略 SIGINT/SIGTERM，使“停止导航”只能
    # 来自桥的显式停止（SIGKILL 无法屏蔽，因此桥的停止路径依旧有效且更快）。
    # 背景：13:30 那次导航在到达目标后被一个来源不明、且非桥发出的 SIGINT 拆掉。
    if (target_key == NAVIGATION_LAUNCH_TARGET
            and os.path.exists(NAV_SIGNAL_GUARD_PATH)):
        existing_preload = launch_environment.get('LD_PRELOAD', '').strip()
        launch_environment['LD_PRELOAD'] = (
            NAV_SIGNAL_GUARD_PATH + (' ' + existing_preload if existing_preload else '')
        )
    popen_options = {'env': launch_environment}
    if navigation_launch_lock is not None:
        popen_options['pass_fds'] = (navigation_launch_lock.fileno(),)
        popen_options['start_new_session'] = True
    try:
        process = subprocess.Popen(command, **popen_options)
    except Exception:
        release_navigation_launch_lock(navigation_launch_lock)
        raise
    entry = make_process_entry(
        'launch', package_name, launch_file, request_id, process,
        video_session_id=producer_session_id,
        navigation_launch_lock=navigation_launch_lock,
    )

    with state_lock:
        launch_processes[target_key] = entry
        if is_navigation_launch:
            _append_navigation_request_waiter_locked(entry, request_id)
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    if is_navigation_launch:
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(
                'Navigation launch was accepted; waiting for health confirmation.',
                status='starting',
                action='start_launch',
                package=package_name,
                launchFile=launch_file,
                requestId=request_id,
                healthConfirmed=False,
                requestTerminal=False,
            ),
        )
        publish_navigation_feature_status(client, 'navigation_launch_spawned', force=True)
        start_navigation_health_monitor(client, entry)
        start_process_monitor(client, 'launch', entry)
    else:
        start_process_monitor(client, 'launch', entry)
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(
                f'Launch started successfully: {package_name}/{launch_file}',
                status='running',
                videoSessionId=producer_session_id if is_video_launch else '',
                producerSessionId=producer_session_id if is_video_launch else '',
                videoLaunchMode='shared_producer' if is_video_launch else '',
            ),
        )


def schedule_local_hls_preview_restart(client: mqtt.Client, reason: str) -> None:
    """Retry the local image producer without blocking MQTT callbacks."""
    if not AUTO_START_LOCAL_HLS or stop_event.is_set():
        return

    def restart_after_delay() -> None:
        if stop_event.wait(LOCAL_HLS_RESTART_DELAY):
            return
        ensure_local_hls_preview(client, reason)

    threading.Thread(target=restart_after_delay, daemon=True).start()


def ensure_local_hls_preview(client: mqtt.Client, reason: str) -> None:
    """Ensure the local camera-frame HTTP producer survives bridge restarts.

    This invokes the ordinary allow-listed launch path, so MQTT-triggered
    starts still reuse the same tracked process and shared producer identity.
    It never starts the RTMP publisher.
    """
    if not AUTO_START_LOCAL_HLS or stop_event.is_set():
        return

    try:
        start_launch(client, {
            'action': 'start_launch',
            'package': 'cloud_video_pusher',
            'launchFile': 'local_hls_server.launch.py',
            'requestId': LOCAL_HLS_PRODUCER_SESSION_ID,
            'producerSessionId': LOCAL_HLS_PRODUCER_SESSION_ID,
        })
        print(f'Local HLS preview ensure requested: {reason}')
    except Exception as exc:  # noqa: BLE001
        error_message = f'Automatic local HLS preview start failed ({reason}): {exc}'
        set_state(
            requestId=LOCAL_HLS_PRODUCER_SESSION_ID,
            lastError=error_message,
            launchStatus='failed',
        )
        publish_json(
            client,
            ERROR_TOPIC,
            build_status_payload(
                error_message,
                error=error_message,
                status='failed',
                action='ensure_local_hls_preview',
                producerSessionId=LOCAL_HLS_PRODUCER_SESSION_ID,
                videoLaunchMode='shared_producer',
            ),
        )
        schedule_local_hls_preview_restart(client, 'local_preview_start_failed')


def schedule_rtmp_publisher_restart(client: mqtt.Client, reason: str) -> None:
    """Retry the opt-in persistent cloud publisher without blocking MQTT callbacks."""
    if not AUTO_START_RTMP or stop_event.is_set():
        return

    def restart_after_delay() -> None:
        if stop_event.wait(RTMP_RESTART_DELAY):
            return
        ensure_rtmp_publisher(client, reason)

    threading.Thread(target=restart_after_delay, daemon=True).start()


def ensure_rtmp_publisher(client: mqtt.Client, reason: str) -> None:
    """Keep one bridge-owned cloud RTMP publisher running for this vehicle.

    This shares the ordinary allow-listed launch path with MQTT requests, so a
    viewer-triggered start reuses this publisher instead of creating a second
    encoder or replacing its stable producer identity.
    """
    if not AUTO_START_RTMP or stop_event.is_set():
        return

    try:
        start_launch(client, {
            'action': 'start_launch',
            'package': 'cloud_video_pusher',
            'launchFile': 'rtmp_pusher.launch.py',
            'requestId': RTMP_PRODUCER_SESSION_ID,
            'producerSessionId': RTMP_PRODUCER_SESSION_ID,
        })
        print(f'Always-on RTMP ensure requested: {reason}')
    except Exception as exc:  # noqa: BLE001
        error_message = f'Automatic RTMP publisher start failed ({reason}): {exc}'
        set_state(
            requestId=RTMP_PRODUCER_SESSION_ID,
            lastError=error_message,
            launchStatus='failed',
        )
        publish_json(
            client,
            ERROR_TOPIC,
            build_status_payload(
                error_message,
                error=error_message,
                status='failed',
                action='ensure_always_on_rtmp',
                producerSessionId=RTMP_PRODUCER_SESSION_ID,
                videoLaunchMode='shared_producer',
            ),
        )
        schedule_rtmp_publisher_restart(client, 'rtmp_publisher_start_failed')


def start_node(client: mqtt.Client, payload: dict) -> None:
    package_name = (payload.get('package') or DEFAULT_NODE_PACKAGE).strip()
    node_executable = (payload.get('nodeExecutable') or DEFAULT_NODE_EXECUTABLE).strip()
    node_args = payload.get('args') or {}
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, node_executable)

    if target_key not in ALLOWED_NODE_TARGETS:
        raise ValueError(f'Node target not allowed: {package_name}/{node_executable}')
    if not isinstance(node_args, dict):
        raise ValueError('args must be an object')

    with state_lock:
        existing_entry = node_processes.get(target_key)
        if existing_entry and existing_entry['process'].poll() is None:
            raise RuntimeError(f'Node task already running: {package_name}/{node_executable}')
        existing_one_shot = one_shot_processes.get(target_key)
        if existing_one_shot and existing_one_shot['process'].poll() is None:
            raise RuntimeError(f'One-shot node already running: {package_name}/{node_executable}')

    command = node_command(package_name, node_executable, node_args)
    process = subprocess.Popen(command)
    one_shot = target_key in ONE_SHOT_NODE_TARGETS
    kind = 'oneshot' if one_shot else 'node'
    entry = make_process_entry('node', package_name, node_executable, request_id, process, one_shot=one_shot)

    with state_lock:
        if one_shot:
            one_shot_processes[target_key] = entry
        else:
            node_processes[target_key] = entry
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    start_process_monitor(client, kind, entry)
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(
            f"{'One-shot node' if one_shot else 'Node'} started successfully: {package_name}/{node_executable}",
            status='running',
        ),
    )


# Business stop only targets the managed launch process instead of arbitrary commands.
def stop_launch(client: mqtt.Client, payload: dict) -> None:
    launch_file = (payload.get('launchFile') or DEFAULT_LAUNCH_FILE).strip()
    package_name = (payload.get('package') or DEFAULT_PACKAGE).strip()
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, launch_file)
    is_video_launch = target_key in VIDEO_LAUNCH_TARGETS
    producer_session_id = resolve_video_producer_session_id(
        payload, request_id, allow_request_id=False,
    ) if is_video_launch else ''
    # A string such as "false" must never become a destructive force stop.
    force_stop = payload.get('forceStop') is True or payload.get('force') is True

    with state_lock:
        entry = launch_processes.get(target_key)

    if (
        target_key == NAVIGATION_LAUNCH_TARGET
        and entry
        and navigation_process_entry_active(entry)
    ):
        voice_launch, lookup_error = find_voice_owned_navigation_launch()
        if voice_launch is not None or lookup_error:
            message = lookup_error or (
                'Both bridge-owned and voice-owned navigation launchers were '
                'observed; shutdown is blocked until the vehicle is inspected.'
            )
            set_state(requestId=request_id, lastError=message, launchStatus='failed')
            publish_json(
                client,
                ERROR_TOPIC,
                build_status_payload(
                    message,
                    status='failed',
                    action='stop_launch',
                    navigationSource='unknown',
                ),
            )
            publish_navigation_feature_status(
                client, 'navigation_stop_multiple_owners', force=True
            )
            return

    entry_is_active = bool(entry) and (
        navigation_process_entry_active(entry)
        if target_key == NAVIGATION_LAUNCH_TARGET
        else entry['process'].poll() is None
    )
    if not entry_is_active:
        # Nav2 can be started by the voice service rather than this bridge.  It
        # is still eligible for a stop request only when its exact launcher,
        # parent and independent process group can be revalidated.  All other
        # external launches remain deliberately unmanageable from MQTT.
        if target_key == NAVIGATION_LAUNCH_TARGET:
            navigation_system, availability = navigation_system_state_from_probe()
            voice_launch, lookup_error = find_voice_owned_navigation_launch()
            active_navigation = (
                availability == 'ready'
                and navigation_system['state'] in {'starting', 'ready', 'failed'}
            )
            if active_navigation and voice_launch is not None:
                set_state(requestId=request_id, lastError='')
                set_navigation_stop_transition(True, 'voice')
                publish_navigation_feature_status(
                    client, 'voice_navigation_stop_requested', force=True
                )
                publish_json(
                    client,
                    STATUS_TOPIC,
                    build_status_payload(
                        'Stopping safely identified voice-owned navigation launcher.',
                        status='stopping',
                        action='stop_launch',
                        navigationSource='voice',
                    ),
                )
                try:
                    stopped, message = stop_voice_owned_navigation_launch(voice_launch)
                except Exception as exc:  # noqa: BLE001 - preserve safe UI state
                    stopped = False
                    message = f'Voice navigation shutdown failed: {exc}'
                if not stopped:
                    set_navigation_stop_transition(False)
                    set_state(requestId=request_id, lastError=message, launchStatus='failed')
                    publish_json(
                        client,
                        ERROR_TOPIC,
                        build_status_payload(
                            message,
                            status='failed',
                            action='stop_launch',
                            navigationSource='voice',
                        ),
                    )
                    publish_navigation_feature_status(
                        client, 'voice_navigation_stop_failed', force=True
                    )
                    return

                set_state(requestId=request_id, lastError='')
                set_navigation_stop_transition(False)
                publish_json(
                    client,
                    STATUS_TOPIC,
                    build_status_payload(
                        message,
                        status='stopped',
                        action='stop_launch',
                        navigationSource='voice',
                    ),
                )
                publish_navigation_feature_status(
                    client, 'voice_navigation_stopped', force=True
                )
                return

            if active_navigation:
                message = lookup_error or (
                    'Navigation is active but its launcher cannot be safely '
                    'identified for shutdown.'
                )
                set_state(requestId=request_id, lastError=message, launchStatus='failed')
                publish_json(
                    client,
                    ERROR_TOPIC,
                    build_status_payload(
                        message,
                        status='failed',
                        action='stop_launch',
                        navigationSource='unknown',
                    ),
                )
                publish_navigation_feature_status(
                    client, 'navigation_stop_rejected', force=True
                )
                return

        set_state(requestId=request_id)
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(f'No active launch task to stop: {package_name}/{launch_file}', status='idle'),
        )
        return

    if is_video_launch:
        active_session_id = str(entry.get('videoSessionId') or '')
        if not force_stop:
            with state_lock:
                state['lastError'] = ''
                refresh_state_locked()
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    f'Ignored viewer video stop for shared producer: {package_name}/{launch_file}',
                    status='running',
                    ignored=True,
                    reason='shared_video_requires_force_stop',
                    videoSessionId=active_session_id,
                    producerSessionId=active_session_id,
                    requestedProducerSessionId=producer_session_id,
                    videoLaunchMode='shared_producer',
                    requestId=request_id,
                ),
            )
            return
        if not producer_session_id or producer_session_id != active_session_id:
            with state_lock:
                state['lastError'] = ''
                refresh_state_locked()
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    f'Ignored force stop for mismatched video producer: {package_name}/{launch_file}',
                    status='running',
                    ignored=True,
                    reason='video_producer_session_mismatch',
                    videoSessionId=active_session_id,
                    producerSessionId=active_session_id,
                    requestedProducerSessionId=producer_session_id,
                    videoLaunchMode='shared_producer',
                    requestId=request_id,
                ),
            )
            return

    if target_key == NAVIGATION_LAUNCH_TARGET:
        set_navigation_stop_transition(True, 'bridge')
        with state_lock:
            current_entry = launch_processes.get(target_key)
            if current_entry is entry:
                current_entry['_navigationStopInProgress'] = True
        cancellation_message = (
            'Navigation startup was cancelled by an explicit stop request.'
        )
        cancelled_request_ids = _mark_navigation_health_terminal(
            entry, ready=False, message=cancellation_message
        )
        if cancelled_request_ids is not None:
            publish_navigation_start_terminal(
                client,
                cancelled_request_ids,
                ready=False,
                message=cancellation_message,
                package_name=entry['package'],
                launch_file=entry['entryName'],
            )
        publish_navigation_feature_status(
            client, 'managed_navigation_stop_requested', force=True
        )
    if target_key == NAVIGATION_LAUNCH_TARGET:
        try:
            stopped, stop_message = stop_navigation_process_entry(
                entry, request_lifecycle=not NAVIGATION_STOP_FAST_PATH)
        except Exception as exc:  # noqa: BLE001 - preserve retryable ownership
            stopped = False
            stop_message = f'Navigation shutdown failed: {exc}'
        if not stopped:
            entry['_navigationStopInProgress'] = False
            set_navigation_stop_transition(False)
            set_state(
                requestId=request_id,
                lastError=stop_message,
                launchStatus='failed',
            )
            failure_payload = build_status_payload(
                stop_message,
                status='failed',
                error=stop_message,
                action='stop_launch',
                package=package_name,
                launchFile=launch_file,
                requestId=request_id,
                requestTerminal=True,
            )
            publish_json(client, STATUS_TOPIC, failure_payload)
            publish_json(client, ERROR_TOPIC, failure_payload)
            publish_navigation_feature_status(
                client, 'managed_navigation_stop_failed', force=True)
            return
    else:
        stop_process_entry(entry)
    release_process_entry_resources(entry)

    with state_lock:
        remove_process_entry('launch', target_key)
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    if target_key == NAVIGATION_LAUNCH_TARGET:
        set_navigation_stop_transition(False)
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(
            (
                stop_message
                if target_key == NAVIGATION_LAUNCH_TARGET
                else f'Launch stopped successfully: {package_name}/{launch_file}'
            ),
            status='stopped',
            requestId=request_id,
            requestTerminal=True,
        ),
    )
    if target_key == NAVIGATION_LAUNCH_TARGET:
        publish_navigation_feature_status(client, 'navigation_launch_stopped', force=True)


def stop_node(client: mqtt.Client, payload: dict) -> None:
    package_name = (payload.get('package') or DEFAULT_NODE_PACKAGE).strip()
    node_executable = (payload.get('nodeExecutable') or DEFAULT_NODE_EXECUTABLE).strip()
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, node_executable)

    with state_lock:
        entry = node_processes.get(target_key)

    if not entry or entry['process'].poll() is not None:
        set_state(requestId=request_id)
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(f'No active node task to stop: {package_name}/{node_executable}', status='idle'),
        )
        return

    stop_process_entry(entry)

    with state_lock:
        remove_process_entry('node', target_key)
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(f'Node stopped successfully: {package_name}/{node_executable}', status='stopped'),
    )


def call_service(client: mqtt.Client, payload: dict) -> None:
    service_path = str(payload.get('servicePath') or '').strip()
    service_type = str(payload.get('serviceType') or '').strip()
    service_args = payload.get('serviceArgs') or {}
    request_id = payload.get('requestId')
    target_key = build_target_key(service_path, service_type)

    if not service_path:
        raise ValueError('servicePath is required')
    if not service_type:
        raise ValueError('serviceType is required')
    if target_key not in ALLOWED_SERVICE_TARGETS:
        raise ValueError(f'Service target not allowed: {service_path}/{service_type}')
    if not isinstance(service_args, dict):
        raise ValueError('serviceArgs must be an object')

    set_state(requestId=request_id, lastError='')
    service_metadata = {
        'action': 'call_service',
        'servicePath': service_path,
        'serviceType': service_type,
    }
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(
            f'Service call started: {service_path}',
            status='starting',
            serviceSuccess=None,
            serviceResult='',
            **service_metadata,
        ),
    )

    completed = subprocess.run(
        service_command(service_path, service_type, service_args),
        capture_output=True,
        text=True,
        timeout=SERVICE_CALL_TIMEOUT,
        check=False,
    )

    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    details = stdout or stderr or 'Service call completed.'
    follow_status_metadata = get_body_follow_status_metadata(
        service_path,
        service_type,
        details,
    )

    if completed.returncode != 0:
        refresh_body_follow_feature_after_service(
            client, service_path, 'body_follow_service_process_failed'
        )
        raise RuntimeError(f'Service call failed ({completed.returncode}): {service_path}; {details}')

    if not parse_service_success(service_type, details):
        failure_message = f'Service response reported success=false: {service_path}; {details}'
        set_state(requestId=request_id, lastError=failure_message)
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(
                failure_message,
                status='failed',
                serviceSuccess=False,
                serviceResult=details,
                **service_metadata,
                **follow_status_metadata,
            ),
        )
        refresh_body_follow_feature_after_service(
            client, service_path, 'body_follow_service_rejected'
        )
        return

    set_state(requestId=request_id, lastError='')
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(
            f'Service call completed: {service_path}',
            status='completed',
            serviceSuccess=True,
            serviceResult=details,
            **service_metadata,
            **follow_status_metadata,
        ),
    )
    refresh_body_follow_feature_after_service(
        client, service_path, 'body_follow_service_completed'
    )


def heartbeat_loop(client: mqtt.Client) -> None:
    while not stop_event.wait(HEARTBEAT_INTERVAL):
        with state_lock:
            refresh_state_locked()
        publish_json(
            client,
            HEARTBEAT_TOPIC,
            {
                'deviceId': DEVICE_ID,
                'jetsonOnline': True,
                'launchStatus': state['launchStatus'],
                'currentTask': state['currentTask'],
                'currentTaskType': state.get('currentTaskType', '-'),
                'lastError': state['lastError'],
                'runningLaunches': list(state.get('runningLaunches', [])),
                'runningNodes': list(state.get('runningNodes', [])),
                'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
            },
        )


def feature_status_poll_interval(snapshot: dict | None) -> float:
    """Use a faster, bounded probe while any controlled feature is in flight.

    This only changes observation cadence.  It neither sends control commands
    nor changes the QoS-1/retained delivery semantics of feature snapshots.
    """
    if not isinstance(snapshot, dict):
        return NAVIGATION_FEATURE_STATUS_INTERVAL
    for key in (
        'vehicleReadiness',
        'navigationSystem',
        'bodyFollow',
        'recharge',
        'mappingSession',
        'targetNavigation',
    ):
        value = snapshot.get(key)
        if not isinstance(value, dict):
            continue
        state_name = str(value.get('state') or '').strip().lower()
        if state_name in {
            'starting',
            'initializing',
            'stopping',
            'navigating',
        }:
            return NAVIGATION_FEATURE_STATUS_ACTIVE_INTERVAL
        # Vehicle/Nav2 readiness commonly remains ready for the whole boot and
        # must not pin polling to the active rate.  In contrast, ready on these
        # three controllable features means an operation is actually enabled.
        if (
            state_name == 'ready'
            and key in {'bodyFollow', 'recharge', 'mappingSession'}
        ):
            return NAVIGATION_FEATURE_STATUS_ACTIVE_INTERVAL
    return NAVIGATION_FEATURE_STATUS_INTERVAL


def navigation_feature_status_loop(client: mqtt.Client) -> None:
    """Refresh retained state even when voice control bypasses MQTT commands."""
    interval = NAVIGATION_FEATURE_STATUS_INTERVAL
    while not stop_event.wait(interval):
        try:
            snapshot = publish_navigation_feature_status(client, 'periodic_probe')
            interval = feature_status_poll_interval(snapshot)
        except Exception as exc:  # noqa: BLE001
            # A bad probe must degrade the feature snapshot, not terminate the
            # command bridge or affect any motion process.
            print(f'Navigation feature status refresh failed: {exc}')
            interval = NAVIGATION_FEATURE_STATUS_INTERVAL


def on_connect(client: mqtt.Client, userdata, flags, reason_code, properties=None) -> None:
    print(f'MQTT connected: reason_code={reason_code}')
    client.subscribe(COMMAND_TOPIC, qos=1)
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload('Jetson MQTT bridge connected.', status='idle'),
    )
    publish_navigation_feature_status(client, 'bridge_connected', force=True)
    ensure_local_hls_preview(client, 'bridge_connected')
    ensure_rtmp_publisher(client, 'bridge_connected')


def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage) -> None:
    payload: dict = {}
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        action = payload.get('action')

        if action == 'start_launch':
            start_launch(client, payload)
            return
        if action == 'stop_launch':
            stop_launch(client, payload)
            return
        if action == 'start_node':
            start_node(client, payload)
            return
        if action == 'stop_node':
            stop_node(client, payload)
            return
        if action == 'call_service':
            call_service(client, payload)
            return

        raise ValueError(f'Unsupported action: {action}')
    except Exception as exc:  # noqa: BLE001
        error_message = str(exc)
        action = payload.get('action')
        package_name = str(payload.get('package') or DEFAULT_PACKAGE).strip()
        launch_file = str(payload.get('launchFile') or DEFAULT_LAUNCH_FILE).strip()
        is_navigation_start = (
            action == 'start_launch'
            and build_target_key(package_name, launch_file)
            == NAVIGATION_LAUNCH_TARGET
        )
        if is_navigation_start:
            request_id = str(payload.get('requestId') or '').strip() or (
                f'navigation-{uuid.uuid4()}'
            )
            payload['requestId'] = request_id
            set_state(
                requestId=request_id,
                lastError=error_message,
                launchStatus='failed',
            )
            publish_navigation_start_terminal(
                client,
                [request_id],
                ready=False,
                message=error_message,
                package_name=package_name,
                launch_file=launch_file,
            )
            publish_navigation_feature_status(
                client, 'navigation_start_exception', force=True)
            return
        service_path = str(payload.get('servicePath') or '').strip()
        service_type = str(payload.get('serviceType') or '').strip()
        is_service_call = payload.get('action') == 'call_service' and bool(service_path)
        set_state(requestId=payload.get('requestId'), lastError=error_message, launchStatus='failed')
        publish_json(
            client,
            ERROR_TOPIC,
            build_status_payload(
                error_message,
                error=error_message,
                action=action,
                servicePath=service_path,
                serviceType=service_type,
                serviceSuccess=False if is_service_call else None,
                serviceResult='',
            ),
        )


def main() -> None:
    singleton_lock, singleton_error = acquire_bridge_singleton_lock()
    if singleton_lock is None:
        raise SystemExit(f'Refusing duplicate launch MQTT bridge: {singleton_error}')
    client = None
    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'jetson-launch-{DEVICE_ID}',
        )
        client.on_connect = on_connect
        client.on_message = on_message
        if NAVIGATION_FEATURE_STATUS_ENABLED:
            client.will_set(
                FEATURE_STATUS_TOPIC,
                json.dumps(
                    build_navigation_feature_offline_snapshot(),
                    ensure_ascii=True,
                ),
                qos=1,
                retain=True,
            )
        client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)

        heartbeat_thread = threading.Thread(
            target=heartbeat_loop, args=(client,), daemon=True
        )
        heartbeat_thread.start()
        if NAVIGATION_FEATURE_STATUS_ENABLED:
            feature_status_thread = threading.Thread(
                target=navigation_feature_status_loop,
                args=(client,),
                daemon=True,
            )
            feature_status_thread.start()
        client.loop_forever()
    finally:
        stop_event.set()
        if client is not None:
            try:
                if NAVIGATION_FEATURE_STATUS_ENABLED:
                    publish_json(
                        client,
                        FEATURE_STATUS_TOPIC,
                        build_navigation_feature_offline_snapshot(),
                        retain=True,
                    )
                client.disconnect()
            except Exception:
                pass
        release_bridge_singleton_lock(singleton_lock)


if __name__ == '__main__':
    main()
