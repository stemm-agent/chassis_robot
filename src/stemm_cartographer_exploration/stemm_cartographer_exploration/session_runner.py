#!/usr/bin/env python3
"""Outer session runner that restores services before publishing terminal state."""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.error import URLError
from urllib.request import Request
from urllib.request import urlopen


SERVICE_GUARD = '/usr/local/sbin/stemm-cartographer-service-guard'
LOCK_FILE = '/tmp/stemm-cartographer-exploration.lock'
INNER_LAUNCH = (
    'ros2', 'launch', 'stemm_cartographer_exploration',
    'stemm_cartographer_auto_mapping.launch.py',
)

# 2026-10-10: 允许用环境变量选择前沿点来源，便于用 systemd 启动建图会话
# （命令行手工启动会打断收尾流程，且日志不进 journal）。
# 未设置时不追加任何参数，行为与既有完全一致（走 launch 的默认值 rrt）。
def _frontier_source_launch_args():
    value = os.getenv('STEMM_FRONTIER_SOURCE', '').strip().lower()
    if value in ('rrt', 'grid'):
        return ['frontier_source:=%s' % value]
    if value:
        print(
            '[WARN] ignoring STEMM_FRONTIER_SOURCE=%r '
            '(expected rrt or grid)' % value,
            file=sys.stderr, flush=True)
    return []
STATE_HOME = Path(os.getenv(
    'XDG_STATE_HOME', Path.home() / '.local' / 'state')).expanduser()
SESSION_STATUS_PATH = Path(os.getenv(
    'STEMM_CARTOGRAPHER_SESSION_STATUS_PATH',
    STATE_HOME / 'stemm-cartographer-session.json',
)).expanduser()
SESSION_TASK_PATH = Path(os.getenv(
    'STEMM_CARTOGRAPHER_TASK_PATH',
    STATE_HOME / 'stemm-cartographer-task.json',
)).expanduser()
VEHICLE_READINESS_URL = os.getenv(
    'STEMM_CARTOGRAPHER_VEHICLE_READINESS_URL',
    'http://127.0.0.1:8001/api/feature/status',
).strip()
try:
    VEHICLE_READINESS_WAIT_TIMEOUT_SEC = max(
        1.0, float(os.getenv('STEMM_CARTOGRAPHER_VEHICLE_READINESS_WAIT_TIMEOUT_SEC', '90'))
    )
except (TypeError, ValueError):
    VEHICLE_READINESS_WAIT_TIMEOUT_SEC = 90.0
try:
    VEHICLE_READINESS_POLL_INTERVAL_SEC = max(
        0.1, float(os.getenv('STEMM_CARTOGRAPHER_VEHICLE_READINESS_POLL_INTERVAL_SEC', '0.5'))
    )
except (TypeError, ValueError):
    VEHICLE_READINESS_POLL_INTERVAL_SEC = 0.5

# 2026-09-16 (F2): the map-mode navigation stack shares the root-namespace Nav2
# node names with this mapping session.  A concurrent copy made the mapping
# readiness guard fail and aborted the session into `recovery_required`, so the
# session releases navigation before it stops the functional services.
NAVIGATION_COMMAND_URL = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_COMMAND_URL',
    'http://127.0.0.1:3000/api/command',
).strip()
NAVIGATION_LAUNCH_PACKAGE = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_LAUNCH_PACKAGE', 'wheeltec_nav2').strip()
NAVIGATION_LAUNCH_FILE = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_LAUNCH_FILE', 'wheeltec_nav2.launch.py').strip()
NAVIGATION_LAUNCH_MARKER = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_LAUNCH_MARKER', NAVIGATION_LAUNCH_FILE).strip()
try:
    NAVIGATION_RELEASE_TIMEOUT_SEC = max(
        5.0,
        float(os.getenv(
            'STEMM_CARTOGRAPHER_NAVIGATION_RELEASE_TIMEOUT_SEC', '60')),
    )
except (TypeError, ValueError):
    NAVIGATION_RELEASE_TIMEOUT_SEC = 60.0
NAVIGATION_RELEASE_POLL_SEC = 0.5
# 2026-10-13 (P23): the release above must have a matching restore.  The
# session stops `wheeltec_nav2` so it cannot collide with this session's
# root-namespace Nav2 nodes; nothing put it back, so map-mode navigation
# stayed down after every session.
NAVIGATION_RESTORE_ENABLED = os.getenv(
    'STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_ENABLED', '1').strip().lower() \
    not in {'0', 'false', 'no', 'off'}
try:
    NAVIGATION_RESTORE_TIMEOUT_SEC = max(
        10.0,
        float(os.getenv(
            'STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_TIMEOUT_SEC', '25')),
    )
except (TypeError, ValueError):
    NAVIGATION_RESTORE_TIMEOUT_SEC = 25.0
# The functional-service restore starts the follow stack
# (bodyfollow-nav2 + /nofmap) first.  Starting `wheeltec_nav2` immediately
# afterwards made both fight over root-namespace / RTPS shared-memory ports
# (measured 2026-10-13 16:11:38: "[RTPS_TRANSPORT_SHM Error] Failed
# init_port fastrtps_port7423: open_and_lock_file failed"), so settle first.
try:
    NAVIGATION_RESTORE_SETTLE_SEC = max(
        0.0,
        float(os.getenv(
            'STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_SETTLE_SEC', '5')),
    )
except (TypeError, ValueError):
    NAVIGATION_RESTORE_SETTLE_SEC = 5.0
NAVIGATION_RESTORE_POLL_SEC = 2.0
# 2026-10-13 (P29): the helper must NOT inherit this process's stdout/stderr.
# The runner is spawned by `ros2 launch`, so its fd 1/2 are the launch's pipe;
# once the runner exits, launch closes the read end and the helper dies on its
# first print (BrokenPipeError/SIGPIPE) before writing a single line -- measured
# 17:58: helper spawned, zero [nav-restore] output, navigation never restored.
NAVIGATION_RESTORE_LOG_PATH = os.getenv(
    'STEMM_CARTOGRAPHER_NAV_RESTORE_LOG',
    os.path.expanduser('~/.local/state/stemm-nav-restore.log'),
).strip()
# 2026-10-13 (P24): a restore request can be accepted by the 3000 Node API
# (HTTP 200) and still never reach a running stack, because the MQTT bridge
# refuses `start_launch` while `mapping_owns_vehicle()` is still true
# (launch_mqtt_bridge.py:3751) -- it only observes the release on its next
# periodic probe.  Measured 17:32: the request at 17:32:28 was dropped and the
# bridge's own hook succeeded at 17:34:04 (96 s later).  So retry instead of
# waiting out one long timeout.
try:
    NAVIGATION_RESTORE_ATTEMPTS = max(
        1, int(os.getenv(
            'STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_ATTEMPTS', '4')))
except (TypeError, ValueError):
    NAVIGATION_RESTORE_ATTEMPTS = 4
try:
    NAVIGATION_RESTORE_RETRY_SEC = max(
        0.0,
        float(os.getenv(
            'STEMM_CARTOGRAPHER_NAVIGATION_RESTORE_RETRY_SEC', '12')),
    )
except (TypeError, ValueError):
    NAVIGATION_RESTORE_RETRY_SEC = 12.0
# 2026-09-17: bounded grace after a finished mapping session so the finally block
# (service restore + terminal status) always runs even if the inner launch lingers.
try:
    INNER_LAUNCH_EXIT_GRACE_SEC = max(
        5.0, float(os.getenv('STEMM_CARTOGRAPHER_INNER_LAUNCH_EXIT_GRACE_SEC', '60')))
except (TypeError, ValueError):
    INNER_LAUNCH_EXIT_GRACE_SEC = 60.0

# P0a-20261014: 首次观察到子进程后，至少经过这么久才允许把"无子进程"
# 判成僵尸。取值需大于 `ros2 launch` 启动期的子进程抖动窗口，远小于
# INNER_LAUNCH_EXIT_GRACE_SEC —— 它只推迟判定，不缩短收尾宽限。
INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC = max(
    1.0, float(os.getenv('STEMM_CARTOGRAPHER_ORPHAN_MIN_OBSERVE_SEC', '15')))
# 2026-09-24: mirrors the guard's STOP_ORDER allowlist.  Used only as a second
# opinion after a non-zero `stop` result: the runner verifies that the vehicle is
# actually free instead of trusting the guard helper's exit code.
FUNCTIONAL_SERVICE_UNITS = (
    'bodyfollow-nav2.service',
    'wheeltec-recharge-manager.service',
    'low-battery-auto-recharge.service',
)
# Same semantics as the guard's own unit_is_stopped(): the state must be exactly
# inactive/failed and no main pid may be left.  An unreadable state fails closed.
STOPPED_UNIT_STATES = frozenset(('inactive', 'failed'))


def _read_json_object(path: Path) -> dict:
    try:
        with path.open('r', encoding='utf-8') as handle:
            payload = json.load(handle)
        return payload if isinstance(payload, dict) else {}
    except (OSError, ValueError, json.JSONDecodeError):
        return {}


def _write_json_atomically(path: Path, payload: dict) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary_path = path.with_name(path.name + '.tmp')
    with temporary_path.open('w', encoding='utf-8') as handle:
        json.dump(payload, handle, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_path, path)


class SessionRunner:
    def __init__(self) -> None:
        self.child = None
        self.stop_attempted = False
        self.received_signal = None
        self.session_task_id = ''
        self.mqtt_launch = False
        self.process_termination_failed = False
        # P0a-20261014: inner-launch startup-window guard state.
        self.inner_launch_children_seen = False
        self.inner_launch_first_child_at = None
        self.inner_launch_childless_logged = False
        # 2026-10-13 (P23): map-mode navigation restore outcome.
        self.navigation_restore_failed = False
        self.navigation_restore_reason = ''
        self.session_started_at_epoch_ms = 0
        try:
            timeout_sec = float(os.getenv(
                'STEMM_CARTOGRAPHER_START_CONFIRM_TIMEOUT_SEC', '120'
            ))
        except (TypeError, ValueError):
            timeout_sec = 120.0
        self.start_confirm_timeout_sec = max(0.0, timeout_sec)
        self.start_confirm_deadline = None
        self.start_confirmed = False
        self.start_confirm_timeout_triggered = False
        self.vehicle_readiness_wait_failed = False
        self.navigation_release_failed = False
        self.navigation_release_reason = ''
        self.vehicle_readiness_wait_reason = ''

    def _arm_start_confirmation_timeout(self) -> None:
        # Only MQTT-launched sessions carry a verified task ID. Direct operator
        # launches stay unaffected by this mini-program safety watchdog.
        if (
                not self.mqtt_launch or not self.session_task_id or
                self.start_confirm_timeout_sec <= 0):
            return
        self.start_confirm_deadline = (
            time.monotonic() + self.start_confirm_timeout_sec
        )
        print(
            '[INFO] Cartographer start confirmation watchdog armed for '
            f'{self.start_confirm_timeout_sec:.1f}s (taskId={self.session_task_id}).',
            flush=True,
        )

    def _mapping_start_is_confirmed(self) -> bool:
        if self.start_confirmed:
            return True
        if not self.session_task_id:
            return False
        session = _read_json_object(SESSION_STATUS_PATH)
        if str(session.get('taskId') or '').strip() != self.session_task_id:
            return False
        phase = str(session.get('phase') or '').strip().lower()
        # returning_home necessarily means mapping became ready earlier. Do not
        # kill a session that has already completed its mapping phase quickly.
        if phase in {'mapping', 'returning_home'}:
            self.start_confirmed = True
            print(
                '[INFO] Cartographer mapping start confirmed; watchdog cleared '
                f'(taskId={self.session_task_id}, phase={phase}).',
                flush=True,
            )
            return True
        return False

    def _mark_start_confirmation_timeout_if_due(self) -> bool:
        if self.start_confirm_timeout_triggered:
            return True
        if self.start_confirm_deadline is None or self.start_confirmed:
            return False
        if self._mapping_start_is_confirmed():
            return False
        if time.monotonic() < self.start_confirm_deadline:
            return False
        self.start_confirm_timeout_triggered = True
        print(
            '[ERROR] Cartographer did not confirm phase=mapping before the '
            f'{self.start_confirm_timeout_sec:.1f}s deadline; canceling task '
            f'{self.session_task_id}.',
            file=sys.stderr,
            flush=True,
        )
        return True

    def _startup_abort_requested(self) -> bool:
        if self.received_signal is not None:
            return True
        return self._mark_start_confirmation_timeout_if_due()

    @staticmethod
    def _read_vehicle_readiness() -> tuple[dict | None, str]:
        if not VEHICLE_READINESS_URL:
            return None, 'vehicle readiness endpoint is not configured'
        try:
            with urlopen(VEHICLE_READINESS_URL, timeout=1.0) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except HTTPError as exc:
            return None, f'vehicle readiness endpoint returned HTTP {exc.code}'
        except URLError as exc:
            return None, f'vehicle readiness endpoint unavailable: {exc.reason}'
        except (OSError, UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
            return None, f'vehicle readiness endpoint failed: {exc}'
        if not isinstance(payload, dict) or payload.get('ok') is not True:
            return None, 'vehicle readiness endpoint returned an invalid payload'
        readiness = payload.get('vehicleReadiness')
        if not isinstance(readiness, dict):
            return None, 'vehicle readiness is unavailable'
        return readiness, ''

    def _wait_for_vehicle_readiness(self) -> tuple[bool, str]:
        """Hold a voice-accepted session before any mapping service is stopped."""
        deadline = time.monotonic() + VEHICLE_READINESS_WAIT_TIMEOUT_SEC
        last_log_at = 0.0
        last_reason = ''
        while True:
            if self.received_signal is not None:
                return False, 'mapping startup was canceled before vehicle initialization completed'
            readiness, probe_error = self._read_vehicle_readiness()
            if (
                isinstance(readiness, dict)
                and readiness.get('state') == 'ready'
                and readiness.get('mappingStartAllowed') is True
            ):
                print('[INFO] Vehicle initialization is stable; mapping session may start.', flush=True)
                return True, ''
            last_reason = str(
                (readiness or {}).get('message')
                or probe_error
                or 'vehicle initialization is still in progress'
            )
            now = time.monotonic()
            if now >= deadline:
                return False, last_reason
            if now - last_log_at >= 2.0:
                print(
                    '[INFO] Waiting for vehicle initialization before mapping: '
                    + last_reason,
                    flush=True,
                )
                last_log_at = now
            time.sleep(min(VEHICLE_READINESS_POLL_INTERVAL_SEC, max(0.0, deadline - now)))

    @staticmethod
    def _navigation_launch_running() -> bool:
        marker = NAVIGATION_LAUNCH_MARKER.encode('utf-8', 'replace')
        if not marker:
            return False
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / 'cmdline').read_bytes()
            except OSError:
                continue
            if marker in raw:
                return True
        return False

    def _release_map_navigation(self) -> None:
        """Stop the map-mode navigation stack before mapping takes over motion.

        Added 2026-09-16 (F2).  `wheeltec_nav2` runs the same root-namespace Nav2
        node names as this session, so a concurrent copy makes the mapping
        readiness guard fail and aborts the session into `recovery_required`.
        Nothing has been stopped yet when this runs, so a failure is reported
        without touching the functional services.
        """
        if not self._navigation_launch_running():
            print('[INFO] Map-mode navigation is not running; nothing to release.',
                  flush=True)
            return
        request_id = 'release-nav-%d' % int(time.time() * 1000)
        payload = json.dumps({
            'action': 'stop_launch',
            'package': NAVIGATION_LAUNCH_PACKAGE,
            'launchFile': NAVIGATION_LAUNCH_FILE,
            'requestId': request_id,
        }).encode('utf-8')
        print('[INFO] Map-mode navigation is running; requesting a clean release.',
              flush=True)
        try:
            request = Request(
                NAVIGATION_COMMAND_URL,
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urlopen(request, timeout=5.0) as response:
                print('[INFO] Navigation release accepted: HTTP %s'
                      % getattr(response, 'status', 'ok'), flush=True)
        except (HTTPError, URLError, OSError, ValueError) as exc:
            raise RuntimeError('navigation stop request failed: %s' % exc) from exc

        deadline = time.monotonic() + NAVIGATION_RELEASE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if not self._navigation_launch_running():
                print('[INFO] Map-mode navigation released.', flush=True)
                return
            time.sleep(NAVIGATION_RELEASE_POLL_SEC)
        raise RuntimeError(
            'map-mode navigation did not stop within %.0fs'
            % NAVIGATION_RELEASE_TIMEOUT_SEC)

    def _spawn_navigation_restore_helper(self) -> bool:
        """Start the detached restore helper; do not wait for it (P25).

        The helper is a separate module (`nav_restore`) launched with
        start_new_session=True, so it survives this runner's exit and starts
        only after the bridge can see the session as released.
        """
        if not NAVIGATION_RESTORE_ENABLED:
            print('[INFO] Map-mode navigation restore is disabled.',
                  flush=True)
            return True
        if self._navigation_launch_running():
            print('[INFO] Map-mode navigation is already running; '
                  'nothing to restore.', flush=True)
            return True
        # 2026-10-13 (P32): run the helper as a systemd transient unit.
        # `Popen(start_new_session=True)` was not enough: measured 18:04 the
        # helper wrote its first line and then died inside the 8 s settle
        # window (no further lines, navigation never restored, one orphan
        # nav2_lifecycle_manager left behind) -- teardown runs several
        # cleanup/restart paths at once and only a systemd-managed unit is
        # fully outside their reach.
        unit = 'stemm-nav-restore-%d' % int(time.time())
        pythonpath = os.environ.get('PYTHONPATH', '')
        command = [
            'sudo', '-n', 'systemd-run',
            '--unit=%s' % unit,
            '--collect',
            '--description=STEMM map-mode navigation restore helper',
        ]
        if pythonpath:
            # systemd-run starts with a clean environment; without this the
            # helper cannot import stemm_cartographer_exploration at all.
            command.append('--setenv=PYTHONPATH=%s' % pythonpath)
        command += [sys.executable, '-m',
                    'stemm_cartographer_exploration.nav_restore']
        try:
            completed = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=20.0,
                check=False,
            )
            output = (completed.stdout or '').strip().splitlines()
            output = output[-1] if output else ''
            if completed.returncode == 0:
                print('[INFO] Detached navigation restore helper started as '
                      'systemd unit %s (journalctl -u %s); this session exits '
                      'now and the helper retries on its own.'
                      % (unit, unit), flush=True)
                return True
            print('[WARNING] systemd-run refused the navigation restore '
                  'helper (rc=%s): %s; falling back to a plain detached '
                  'process.' % (completed.returncode, output),
                  file=sys.stderr, flush=True)
        except (OSError, subprocess.SubprocessError) as exc:
            print('[WARNING] could not start the navigation restore helper '
                  'via systemd-run: %s; falling back to a plain detached '
                  'process.' % exc, file=sys.stderr, flush=True)
        # --- 降级路径：P29 的独立日志文件 + 脱离进程组 ---
        fallback = [sys.executable, '-m',
                    'stemm_cartographer_exploration.nav_restore']
        helper_log = None
        try:
            os.makedirs(os.path.dirname(NAVIGATION_RESTORE_LOG_PATH),
                        mode=0o700, exist_ok=True)
            helper_log = open(NAVIGATION_RESTORE_LOG_PATH, 'ab', buffering=0)
        except OSError as exc:
            print('[WARNING] could not open the navigation restore log %s: %s'
                  % (NAVIGATION_RESTORE_LOG_PATH, exc),
                  file=sys.stderr, flush=True)
        try:
            handle = subprocess.Popen(
                fallback,
                start_new_session=True,
                stdin=subprocess.DEVNULL,
                stdout=helper_log if helper_log is not None else subprocess.DEVNULL,
                stderr=subprocess.STDOUT if helper_log is not None else subprocess.DEVNULL,
                close_fds=True,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            if helper_log is not None:
                try:
                    helper_log.close()
                except OSError:
                    pass
            print('[WARNING] could not start the navigation restore helper: '
                  '%s' % exc, file=sys.stderr, flush=True)
            return False
        if helper_log is not None:
            try:
                helper_log.close()
            except OSError:
                pass
        print('[INFO] Navigation restore helper started as a fallback detached '
              'process (pid=%s, log=%s); this session exits now.'
              % (getattr(handle, 'pid', '?'), NAVIGATION_RESTORE_LOG_PATH),
              flush=True)
        return True

    def _restore_map_navigation(self) -> bool:
        """Restore the map-mode navigation stack after a session (P23).

        Added 2026-10-13.  Pairs with _release_map_navigation(): because the
        session stops `wheeltec_nav2` before mapping takes over motion, the
        session must put it back when it is done.

        Why the session owns this instead of launch-mqtt-bridge: the bridge
        has an `auto-navigation` hook, but it keeps its armed task in process
        memory and has a silent failure path (not armed -> bare return, no
        log).  Measured 2026-10-13: it armed only for the 09:53 and 11:24
        sessions, then went silent for the remaining 16 -- leaving
        `wheeltec_nav2` down, `amcl_odom_limiter` stuck on "Waiting for an
        AMCL pose" and targetNavigation.state pinned to 'unknown'.  A restore
        that always reports its outcome is the fix.

        Returns True when the stack is (or already was) running.
        """
        if not NAVIGATION_RESTORE_ENABLED:
            print('[INFO] Map-mode navigation restore is disabled.',
                  flush=True)
            return True
        if self._navigation_launch_running():
            print('[INFO] Map-mode navigation is already running; '
                  'nothing to restore.', flush=True)
            return True
        if NAVIGATION_RESTORE_SETTLE_SEC > 0.0:
            print('[INFO] Waiting %.0fs for the restored functional services '
                  'to settle before starting map-mode navigation.'
                  % NAVIGATION_RESTORE_SETTLE_SEC, flush=True)
            time.sleep(NAVIGATION_RESTORE_SETTLE_SEC)
            if self._navigation_launch_running():
                print('[INFO] Map-mode navigation came back on its own '
                      'during the settle window.', flush=True)
                return True
        # P24: retry -- an HTTP 200 only means the 3000 Node API queued the
        # command; the bridge can still refuse it while it believes mapping
        # owns vehicle motion, and it only notices the release on a later
        # probe.  Each attempt arms a fresh request and waits briefly.
        for attempt in range(1, NAVIGATION_RESTORE_ATTEMPTS + 1):
            if self._navigation_launch_running():
                print('[INFO] Map-mode navigation restored (attempt %d).'
                      % attempt, flush=True)
                return True
            request_id = 'restore-nav-%d-%d' % (attempt, int(time.time() * 1000))
            payload = json.dumps({
                'action': 'start_launch',
                'package': NAVIGATION_LAUNCH_PACKAGE,
                'launchFile': NAVIGATION_LAUNCH_FILE,
                'requestId': request_id,
            }).encode('utf-8')
            print('[INFO] Requesting map-mode navigation restore '
                  '(%s/%s) attempt %d/%d.'
                  % (NAVIGATION_LAUNCH_PACKAGE, NAVIGATION_LAUNCH_FILE,
                     attempt, NAVIGATION_RESTORE_ATTEMPTS), flush=True)
            try:
                request = Request(
                    NAVIGATION_COMMAND_URL,
                    data=payload,
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urlopen(request, timeout=10.0) as response:
                    status = getattr(response, 'status', 'ok')
                print('[INFO] Navigation restore request accepted: HTTP %s'
                      % status, flush=True)
            except (HTTPError, URLError, OSError, ValueError) as exc:
                print('[WARNING] Navigation restore request failed: %s' % exc,
                      file=sys.stderr, flush=True)
            deadline = time.monotonic() + NAVIGATION_RESTORE_TIMEOUT_SEC
            while time.monotonic() < deadline:
                if self._navigation_launch_running():
                    print('[INFO] Map-mode navigation restored (attempt %d).'
                          % attempt, flush=True)
                    return True
                time.sleep(NAVIGATION_RESTORE_POLL_SEC)
            if attempt < NAVIGATION_RESTORE_ATTEMPTS:
                print('[WARNING] Map-mode navigation did not come up within '
                      '%.0fs (attempt %d/%d); retrying in %.0fs.'
                      % (NAVIGATION_RESTORE_TIMEOUT_SEC, attempt,
                         NAVIGATION_RESTORE_ATTEMPTS,
                         NAVIGATION_RESTORE_RETRY_SEC),
                      file=sys.stderr, flush=True)
                time.sleep(NAVIGATION_RESTORE_RETRY_SEC)
        print('[WARNING] Map-mode navigation did not come up after %d '
              'attempts; map-mode navigation (AMCL + map_server) stays '
              'unavailable until it is started manually.'
              % NAVIGATION_RESTORE_ATTEMPTS,
              file=sys.stderr, flush=True)
        return False

    def _write_navigation_release_failure_status(self) -> None:
        """Close only this task; no functional service has been stopped yet."""
        if not self.session_task_id:
            return
        session = _read_json_object(SESSION_STATUS_PATH)
        if str(session.get('taskId') or '').strip() != self.session_task_id:
            return
        now_ms = int(time.time() * 1000)
        terminal = {
            **session,
            'schemaVersion': 2,
            'taskId': self.session_task_id,
            'updatedAt': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'updatedAtEpochMs': now_ms,
            'phase': 'closed',
            'status': 'failed',
            'taskStatus': 'failed',
            'terminal': True,
            'sessionActive': False,
            'processExited': True,
            # Nothing was stopped, so the control services are untouched.
            'functionalServicesRestored': True,
            'manualTakeoverRequired': False,
            'cancelRequested': False,
            'terminationStatus': 'failed',
            'terminationReason': 'navigation_release_failed',
            'lastError': self.navigation_release_reason,
            'recoveryRequiredReason': '',
        }
        _write_json_atomically(SESSION_STATUS_PATH, terminal)
        print('[INFO] Closed mapping request before service stop: '
              'map-mode navigation could not be released.', flush=True)

    def _write_vehicle_readiness_timeout_status(self) -> None:
        """Close only this task; no functional service has been stopped yet."""
        if not self.session_task_id:
            return
        session = _read_json_object(SESSION_STATUS_PATH)
        if str(session.get('taskId') or '').strip() != self.session_task_id:
            return
        now_ms = int(time.time() * 1000)
        terminal = {
            **session,
            'schemaVersion': 2,
            'taskId': self.session_task_id,
            'updatedAt': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'updatedAtEpochMs': now_ms,
            'phase': 'closed',
            'status': 'canceled',
            'taskStatus': 'canceled',
            'terminal': True,
            'sessionActive': False,
            'processExited': True,
            # No stop guard ran, so the original control services were never
            # interrupted and are already in the required restored condition.
            'functionalServicesRestored': True,
            'manualTakeoverRequired': False,
            'cancelRequested': True,
            'terminationStatus': 'canceled',
            'terminationReason': 'vehicle_initialization_timeout',
            'lastError': '',
            'recoveryRequiredReason': '',
        }
        _write_json_atomically(SESSION_STATUS_PATH, terminal)
        print('[INFO] Closed mapping request before service stop: vehicle initialization timeout.', flush=True)

    @staticmethod
    def _signal_process_group(process, signum) -> None:
        if process is None or process.poll() is not None:
            return
        try:
            # All managed children are session leaders (start_new_session=True),
            # so this reaches only the current Cartographer/service-guard tree.
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass

    def _terminate_process_group(self, process) -> bool:
        if process is None or process.poll() is not None:
            return True
        for signum, wait_sec in (
            (signal.SIGINT, 15.0),
            (signal.SIGTERM, 10.0),
            (signal.SIGKILL, 5.0),
        ):
            if process.poll() is not None:
                return True
            self._signal_process_group(process, signum)
            try:
                process.wait(timeout=wait_sec)
                return True
            except subprocess.TimeoutExpired:
                continue
        return process.poll() is not None

    def _functional_services_stopped(self) -> bool:
        """Independently verify the functional-service stop (guard parity).

        2026-09-24: `service-guard stop` also returns 1 when its synchronous
        recharge stop helper cannot reach /auto_recharge/stop, even though the
        bounded systemd stop that follows is what actually frees the vehicle.
        That transient aborted a session before Cartographer started and left a
        terminal failure the app could not retry from.  This predicate mirrors
        the guard's own unit_is_stopped(); a unit that still owns the vehicle
        keeps the stop failure fatal, so nothing is relaxed.
        """
        for unit in FUNCTIONAL_SERVICE_UNITS:
            try:
                probe = subprocess.run(
                    ['systemctl', 'show', '--property=ActiveState',
                     '--property=MainPID', unit],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=10.0,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                print(
                    f'[WARNING] could not verify functional service {unit}: {exc}',
                    file=sys.stderr, flush=True)
                return False
            state = ''
            main_pid = ''
            for line in (probe.stdout or '').splitlines():
                key, _, value = line.partition('=')
                if key == 'ActiveState':
                    state = value.strip()
                elif key == 'MainPID':
                    main_pid = value.strip()
            if state not in STOPPED_UNIT_STATES or main_pid not in ('', '0'):
                print(
                    '[WARNING] functional service still owns the vehicle after '
                    f'guard stop: {unit} state={state or "unknown"} '
                    f'pid={main_pid or "unknown"}',
                    file=sys.stderr, flush=True)
                return False
        return True

    def _service_guard(self, action: str, *, interruptible: bool = False) -> bool:
        timeout_sec = 120.0 if action == 'stop' else 60.0
        command = ['sudo', '-n', SERVICE_GUARD, action]
        process = subprocess.Popen(command, start_new_session=True)
        deadline = time.monotonic() + timeout_sec
        try:
            while process.poll() is None:
                if interruptible and self._startup_abort_requested():
                    # A start timeout can occur before an inner ROS launch exists.
                    # Stop the guard, then let finally restore the fixed services.
                    self._signal_process_group(process, signal.SIGTERM)
                    try:
                        process.wait(timeout=10.0)
                    except subprocess.TimeoutExpired:
                        if not self._terminate_process_group(process):
                            self.process_termination_failed = True
                            raise RuntimeError(
                                'service guard did not exit after termination'
                            )
                    return False
                if time.monotonic() >= deadline:
                    if not self._terminate_process_group(process):
                        self.process_termination_failed = True
                        raise RuntimeError(
                            'service guard did not exit after termination'
                        )
                    raise subprocess.TimeoutExpired(command, timeout_sec)
                time.sleep(0.1)

            if interruptible and self._startup_abort_requested():
                return False
            if process.returncode != 0:
                if action == 'stop' and self._functional_services_stopped():
                    print(
                        '[WARNING] service guard stop exited '
                        f'{process.returncode}, but every allowlisted functional '
                        'service is verified stopped; continuing the mapping '
                        'session.', file=sys.stderr, flush=True)
                    return True
                raise subprocess.CalledProcessError(process.returncode, command)
            return True
        finally:
            if process.poll() is None:
                if not self._terminate_process_group(process):
                    self.process_termination_failed = True

    def _mapping_finished(self) -> bool:
        """True once the session file records a finished mapping result.

        2026-10-13 (P35): the taskId check must NOT be a hard gate when this
        runner never captured an identity.  _capture_session_identity()
        leaves session_task_id empty for direct/manual launches whose
        task.json is not fresh (it prints 'task identity is unavailable or
        stale' and returns).  Comparing the session file's real taskId
        against that empty string made this predicate permanently False,
        which in turn disabled the grace fallback in
        _wait_for_inner_launch(): measured 18:23, all twelve child
        processes exited at 18:25:44 while `ros2 launch` itself hung in
        ep_poll, so the whole teardown blocked forever (phase stuck at
        finalizing, functional services down, navigation never restored).
        With an empty identity we simply skip the comparison and rely on
        the mapping result fields alone.
        """
        session = _read_json_object(SESSION_STATUS_PATH)
        if self.session_task_id:
            if str(session.get('taskId') or '').strip() != self.session_task_id:
                return False
        return bool(
            session.get('mappingDone') is True
            and session.get('finalMapSaved') is True
            and str(session.get('terminationReason') or '').strip()
        )

    def _inner_launch_orphaned(self) -> bool:
        """P35: inner launch has no child processes left, so it is a zombie.

P0a-20261014 —— 启动窗口保护。
  实测（2026-10-14 09:34:03）：`ros2 launch` 刚 fork 出来、子进程尚未创建的
  几秒窗口里，`pgrep -P` 返回"无子进程"，于是被判成僵尸；而该次会话随后
  正常拉起了 23 个子进程并完整跑完建图。`pgrep -P` 只能证明"此刻没有子进程"，
  无法区分"从未有过"与"已全部退出"。
  修法：必须**先观察到过子进程**，且距首次观察超过
  INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC，才允许判 orphan。
  该保护只会让判定更保守（更晚）—— P35 的"僵尸看门狗"语义不变。

        `ros2 launch` occasionally fails to exit after every managed node has
        gone away (measured 18:25:44: twelve nodes printed 'process has
        finished cleanly', yet the launch process stayed in ep_poll with zero
        children).  Such a launch can never do useful work again, so it should
        start the same grace countdown as a finished mapping run -- this layer
        does not depend on the session file at all.
        """
        if self.child is None or self.child.poll() is not None:
            return False
        try:
            probe = subprocess.run(
                ['pgrep', '-P', str(self.child.pid)],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=5.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return False
        if probe.returncode == 0:
            # P0a-20261014: 至少出现过一次子进程 —— 之后才允许谈"孤儿"。
            self.inner_launch_children_seen = True
            if self.inner_launch_first_child_at is None:
                self.inner_launch_first_child_at = time.monotonic()
            return False
        # pgrep exits 1 when nothing matched -> no children left.
        if not getattr(self, 'inner_launch_children_seen', False):
            # 从未见到子进程：这更可能是"launch 还在启动"而不是僵尸。
            if not getattr(self, 'inner_launch_childless_logged', False):
                self.inner_launch_childless_logged = True
                print(
                    '[INFO] Inner launch has no child processes yet; '
                    'treating it as still starting (startup-window guard).',
                    flush=True)
            return False
        first_seen = getattr(self, 'inner_launch_first_child_at', None)
        if first_seen is not None:
            observed_for = time.monotonic() - first_seen
            if observed_for < INNER_LAUNCH_ORPHAN_MIN_OBSERVE_SEC:
                # 刚看到子进程全部退出，可能只是启动期的抖动。
                return False
        return True

    def _wait_for_inner_launch(self) -> int:
        if self.child is None:
            return 1
        finished_since = None
        while self.child.poll() is None:
            if self._startup_abort_requested():
                if not self._terminate_process_group(self.child):
                    self.process_termination_failed = True
                    return 1
                return self.child.returncode if self.child.returncode is not None else 0
            # 2026-10-13 (P35): also treat a childless launch as "done" -- it can
            # never do useful work again, and waiting on it blocks teardown forever.
            mapping_done = self._mapping_finished()
            orphaned = False if mapping_done else self._inner_launch_orphaned()
            if mapping_done or orphaned:
                if finished_since is None:
                    finished_since = time.monotonic()
                    print('[INFO] %s; waiting up to %.0fs for the inner launch to '
                          'exit on its own.'
                          % ('Mapping finished' if mapping_done
                             else 'Inner launch has no child processes left',
                             INNER_LAUNCH_EXIT_GRACE_SEC),
                          flush=True)
                elif time.monotonic() - finished_since > INNER_LAUNCH_EXIT_GRACE_SEC:
                    print('[WARN] Inner launch did not exit within the grace window; '
                          'terminating it so service restoration and the terminal '
                          'status still run.', flush=True)
                    if not self._terminate_process_group(self.child):
                        self.process_termination_failed = True
                    return self.child.returncode if self.child.returncode is not None else 0
            time.sleep(0.1)
        return self.child.wait()

    def _capture_session_identity(self) -> None:
        self.session_started_at_epoch_ms = int(time.time() * 1000)
        expected_task_id = os.getenv(
            'STEMM_CARTOGRAPHER_EXPECTED_TASK_ID', '').strip()
        if expected_task_id:
            self.session_task_id = expected_task_id
            self.mqtt_launch = True
            return

        # Direct/manual launches have no bridge-provided identity.  Fail closed:
        # accept only a task file written after this runner started, never an
        # earlier session's task ID.
        deadline = time.monotonic() + 3.0
        while True:
            task = _read_json_object(SESSION_TASK_PATH)
            task_id = task.get('taskId', '')
            try:
                updated_at_ms = int(float(task.get('updatedAt') or 0.0) * 1000)
            except (TypeError, ValueError):
                updated_at_ms = 0
            if (
                    isinstance(task_id, str) and task_id and
                    updated_at_ms >= self.session_started_at_epoch_ms):
                self.session_task_id = task_id
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)

        self.session_task_id = ''
        print(
            '[WARNING] Cartographer session task identity is unavailable or '
            'stale; terminal state will not be overwritten by the runner.',
            file=sys.stderr,
            flush=True,
        )

    def _write_starting_session_status(self) -> None:
        """Publish a task-scoped startup state before stopping control services.

        The state is intentionally written by the locked session runner rather
        than by the MQTT bridge: a command ACK alone must not overwrite a live
        or newer Cartographer session.  It is a non-terminal progress state,
        not evidence that Cartographer has reached its mapping phase.
        """
        if not self.mqtt_launch or not self.session_task_id:
            return

        previous = _read_json_object(SESSION_STATUS_PATH)
        previous_task_id = str(previous.get('taskId') or '').strip()
        previous_phase = str(previous.get('phase') or '').strip().lower()
        previous_terminal = (
            previous.get('terminal') is True or previous_phase == 'closed'
        )
        try:
            previous_updated_at_ms = int(
                previous.get('updatedAtEpochMs') or 0
            )
        except (TypeError, ValueError):
            previous_updated_at_ms = 0

        # The runner lock normally makes this impossible.  Keep the guard so a
        # stale/manual process can never hide another task that is still active,
        # nor a status written after this runner began.
        if (
                previous_task_id and
                previous_task_id != self.session_task_id and
                (not previous_terminal or
                 previous_updated_at_ms >= self.session_started_at_epoch_ms)):
            print(
                '[WARNING] A newer or active Cartographer session status exists; '
                'refusing to publish startup state for a different task.',
                file=sys.stderr,
                flush=True,
            )
            return

        now_ms = int(time.time() * 1000)
        starting = {
            **previous,
            'schemaVersion': 2,
            'taskId': self.session_task_id,
            'updatedAt': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'updatedAtEpochMs': now_ms,
            'phase': 'starting',
            'status': 'running',
            'taskStatus': 'running',
            'terminal': False,
            'sessionActive': True,
            'processExited': False,
            # Functional services are intentionally stopped while a mapping
            # session is active.  False is normal here, not a recovery error.
            'functionalServicesRestored': False,
            'manualTakeoverRequired': False,
            'cancelRequested': False,
            'terminationStatus': '',
            'terminationReason': '',
            'returnHomeFailureReason': '',
            'returnHomeFailureConfirmed': False,
            'returnHomeFailureKind': '',
            'finishRequestedAt': '',
            'finishRequestedAtEpochMs': 0,
            'returnHomeStartedAt': '',
            'returnHomeStartedAtEpochMs': 0,
            'mappingDone': False,
            'finalMapSaved': False,
            'finalMapSavedAt': '',
            'finalMapSaveError': '',
            'navigationState': '',
            'returnHomeRetry': 0,
            'returnHomeMaxRetries': 0,
            'positionErrorMeters': None,
            'mode': '',
            'lastError': '',
            'recoveryRequiredReason': '',
        }
        try:
            _write_json_atomically(SESSION_STATUS_PATH, starting)
        except OSError as exc:
            # Status delivery improves the UI but must not prevent a safe
            # Cartographer launch; the runner's terminal safeguards remain live.
            print(
                f'[WARNING] Failed to publish Cartographer startup state: {exc}',
                file=sys.stderr,
                flush=True,
            )
            return
        print(
            '[INFO] Published Cartographer startup state '
            f'(taskId={self.session_task_id}).',
            flush=True,
        )

    def _forward_signal(self, signum, _frame) -> None:
        self.received_signal = signum
        # The blocking service guard is checked by _service_guard() within 100ms.
        # If the inner launch exists, forward the graceful signal immediately.
        self._signal_process_group(self.child, signal.SIGINT)

    def _verified_terminal_outcome(self, session: dict) -> tuple[str, str]:
        termination_status = str(session.get('terminationStatus') or '').lower()
        termination_reason = str(session.get('terminationReason') or '').strip()

        if self.start_confirm_timeout_triggered:
            return 'canceled', 'mapping_start_timeout'
        if self.received_signal is not None:
            return 'canceled', termination_reason or 'session_runner_signal'
        if (
                session.get('cancelRequested') is True
                or termination_status == 'canceled'):
            return 'canceled', termination_reason or 'return_home_failed'
        if termination_status == 'succeeded':
            return 'succeeded', termination_reason or 'return_home_complete'
        return '', (
            'inner Cartographer launch exited without a verified mapping '
            'completion state'
        )

    def _write_final_session_status(self, restore_succeeded: bool) -> None:
        """Publish the only unlock-safe terminal state after service restore."""
        session = _read_json_object(SESSION_STATUS_PATH)
        if self.start_confirm_timeout_triggered and self.session_task_id:
            existing_task_id = str(session.get('taskId') or '').strip()
            if existing_task_id and existing_task_id != self.session_task_id:
                try:
                    existing_updated_at_ms = int(
                        session.get('updatedAtEpochMs') or 0
                    )
                except (TypeError, ValueError):
                    existing_updated_at_ms = 0
                # A stale status from an earlier session must not hide the
                # current task's 120-second timeout from the mini-program.
                # Only preserve another task if it wrote status after this
                # runner started.
                if existing_updated_at_ms >= self.session_started_at_epoch_ms:
                    print(
                        '[WARNING] Cartographer timeout belongs to an older '
                        'runner; refusing to overwrite a newer session status.',
                        file=sys.stderr,
                        flush=True,
                    )
                    return
            now_ms = int(time.time() * 1000)
            updated_at = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            restore_error = (
                'functional services were not restored after Cartographer '
                'start timeout'
            )
            terminal = {
                **session,
                'schemaVersion': 2,
                'taskId': self.session_task_id,
                'updatedAt': updated_at,
                'updatedAtEpochMs': now_ms,
                'phase': 'closed' if restore_succeeded else 'recovery_required',
                'status': 'canceled' if restore_succeeded else 'running',
                'taskStatus': 'canceled' if restore_succeeded else 'running',
                'terminal': restore_succeeded,
                'sessionActive': False,
                'processExited': True,
                'functionalServicesRestored': restore_succeeded,
                'manualTakeoverRequired': False,
                'cancelRequested': True,
                'terminationStatus': 'canceled',
                'terminationReason': 'mapping_start_timeout',
                'returnHomeFailureReason': '',
                'returnHomeFailureConfirmed': False,
                'returnHomeFailureKind': '',
                'finishRequestedAt': '',
                'finishRequestedAtEpochMs': 0,
                'returnHomeStartedAt': '',
                'returnHomeStartedAtEpochMs': 0,
                'mappingDone': False,
                'finalMapSaved': False,
                'finalMapSavedAt': '',
                'finalMapSaveError': '',
                'navigationState': '',
                'lastError': '' if restore_succeeded else restore_error,
                **({} if restore_succeeded else {
                    'recoveryRequiredReason': restore_error,
                }),
            }
            _write_json_atomically(SESSION_STATUS_PATH, terminal)
            print(
                '[INFO] Published Cartographer start-timeout terminal state.',
                flush=True,
            )
            return

        # MQTT-launched sessions provide the ID at start.  A direct/systemd
        # launch may not, but the live manager's status file remains the exact
        # same session after its timestamp guard below is satisfied.
        session_task_id = self.session_task_id
        requested_task_id = str(session.get('taskId') or '').strip()
        if not session_task_id:
            session_task_id = requested_task_id
        if not session_task_id:
            # 2026-10-13: 任务身份缺失时**不再放弃写终态**。
            # 实测（10-13 两轮）：task.json 被清成 {"taskId":"","status":"idle"}
            # 之后，runner 因身份为空直接 return，会话状态永久停在
            # phase=finalizing / terminal=false，小程序卡在"正在核验建图服务状态"，
            # 只能人工置终态。这里合成一个可追溯的 ID 继续写：
            # 会话文件此刻已是本次会话写的（下面还有 updatedAtEpochMs 与
            # process_exited/restore_succeeded 三重守卫），因此不会掩盖活跃会话。
            session_task_id = 'task-recovered-%s' % time.strftime('%Y%m%d-%H%M%S')
            print(
                '[WARNING] Cartographer session task identity is missing; '
                'publishing the terminal state under a synthesized task id '
                '(%s) so the mini program can unlock.' % session_task_id,
                file=sys.stderr,
                flush=True,
            )
        if requested_task_id and requested_task_id != session_task_id:
            print(
                '[WARNING] Cartographer session task ID changed; refusing to '
                'overwrite another session status.',
                file=sys.stderr,
                flush=True,
            )
            return

        updated_at_ms = int(session.get('updatedAtEpochMs') or 0)
        if updated_at_ms < self.session_started_at_epoch_ms:
            # 2026-09-24: a stale status is exactly what happens when the manager
            # never published one for this launch (e.g. nav2 startup failure).
            # Returning here left the mini program reading a stale non-terminal
            # state with no failure signal and no recovery path, so we warn and
            # carry on: this runner holds the session lock and the inner launch
            # has already exited, therefore a terminal write cannot hide an
            # active session.
            print(
                '[WARNING] Cartographer session state predates this runner '
                '(updatedAtEpochMs=%d < startedAtEpochMs=%d); a terminal failure '
                'state will still be published once the inner launch has exited.'
                % (updated_at_ms, self.session_started_at_epoch_ms),
                file=sys.stderr,
                flush=True,
            )

        process_exited = self.child is None or self.child.poll() is not None
        outcome, reason = self._verified_terminal_outcome(session)
        now_ms = int(time.time() * 1000)
        updated_at = time.strftime('%Y-%m-%dT%H:%M:%S%z')

        if restore_succeeded and process_exited and outcome:
            terminal = {
                **session,
                'schemaVersion': 2,
                'taskId': session_task_id,
                'updatedAt': updated_at,
                'updatedAtEpochMs': now_ms,
                'phase': 'closed',
                # Status is the public mini-program contract; retain
                # taskStatus for the bridge's older readers.
                'status': outcome,
                'taskStatus': outcome,
                'terminal': True,
                'sessionActive': False,
                'processExited': True,
                'functionalServicesRestored': True,
                'manualTakeoverRequired': False,
                'cancelRequested': session.get('cancelRequested') is True,
                'terminationStatus': outcome,
                'terminationReason': reason,
                # A return-home failure is diagnostic context, not an active
                # task error once the operator has safely canceled the session.
                'lastError': '',
            }
            _write_json_atomically(SESSION_STATUS_PATH, terminal)
            print(
                f'[INFO] Published Cartographer terminal state: {outcome}.',
                flush=True,
            )
            return

        if not process_exited:
            recovery_reason = 'inner Cartographer launch did not exit cleanly'
        elif not restore_succeeded:
            recovery_reason = (
                'functional services were not restored after Cartographer '
                'session exit'
            )
        else:
            recovery_reason = reason

        if process_exited and restore_succeeded:
            # 2026-09-24: the inner launch is gone and the functional services are
            # back, so there is nothing left to unlock later.  Publish an explicit
            # terminal *failure*: the mini program needs a failure signal to clear
            # its pending latches and offer retry / device restart.
            failed_terminal = {
                **session,
                'schemaVersion': 2,
                'taskId': session_task_id,
                'updatedAt': updated_at,
                'updatedAtEpochMs': now_ms,
                'phase': 'closed',
                'status': 'failed',
                'taskStatus': 'failed',
                'terminal': True,
                'sessionActive': False,
                'processExited': True,
                'functionalServicesRestored': True,
                'manualTakeoverRequired': False,
                'cancelRequested': session.get('cancelRequested') is True,
                'terminationStatus': 'failed',
                'terminationReason': recovery_reason,
                'lastError': recovery_reason,
                'recoveryRequiredReason': recovery_reason,
            }
            _write_json_atomically(SESSION_STATUS_PATH, failed_terminal)
            print(
                '[WARNING] Published Cartographer failed terminal state: '
                + recovery_reason,
                flush=True,
            )
            return

        recovery = {
            **session,
            'schemaVersion': 2,
            'taskId': session_task_id,
            'updatedAt': updated_at,
            'updatedAtEpochMs': now_ms,
            'phase': 'recovery_required',
            'status': 'running',
            'taskStatus': 'running',
            'terminal': False,
            'sessionActive': False,
            'processExited': process_exited,
            'functionalServicesRestored': False,
            'manualTakeoverRequired': False,
            'lastError': recovery_reason,
            'recoveryRequiredReason': recovery_reason,
        }
        _write_json_atomically(SESSION_STATUS_PATH, recovery)
        print(
            '[CRITICAL] Cartographer session is not safe to unlock: '
            + recovery_reason,
            file=sys.stderr,
            flush=True,
        )

    def run(self, launch_args: list[str]) -> int:
        Path(LOCK_FILE).touch(mode=0o600, exist_ok=True)
        with open(LOCK_FILE, 'r+', encoding='utf-8') as lock_handle:
            try:
                fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print('[ERROR] A STEMM Cartographer session is already running.',
                      file=sys.stderr, flush=True)
                return 73

            signal.signal(signal.SIGINT, self._forward_signal)
            signal.signal(signal.SIGTERM, self._forward_signal)
            self._capture_session_identity()
            self._write_starting_session_status()

            session_return_code = 1
            restore_succeeded = True
            try:
                readiness_ready, readiness_reason = self._wait_for_vehicle_readiness()
                if not readiness_ready:
                    self.vehicle_readiness_wait_reason = readiness_reason
                    if self.received_signal is not None:
                        print('[INFO] Mapping start canceled while waiting for vehicle initialization.',
                              flush=True)
                        session_return_code = 0
                    else:
                        self.vehicle_readiness_wait_failed = True
                        print(
                            '[ERROR] Vehicle initialization did not become ready before '
                            f'{VEHICLE_READINESS_WAIT_TIMEOUT_SEC:.1f}s: {readiness_reason}',
                            file=sys.stderr,
                            flush=True,
                        )
                        # Mark the unit failed so the voice bridge can report the
                        # result promptly instead of waiting for its longer normal
                        # Cartographer confirmation deadline.
                        session_return_code = 74
                else:
                    # The mapping watchdog measures only the actual mapping
                    # startup.  Cold-boot initialization must not consume it.
                    self._arm_start_confirmation_timeout()
                    # Release the map-mode navigation stack first; it cannot
                    # coexist with this session's Nav2 nodes (see F2 note above).
                    try:
                        self._release_map_navigation()
                    except RuntimeError as exc:
                        self.navigation_release_failed = True
                        self.navigation_release_reason = str(exc)
                        raise
                    self.stop_attempted = True
                    print('[INFO] Stopping the fixed functional-service allowlist.',
                          flush=True)
                    guard_completed = self._service_guard('stop', interruptible=True)
                    if not guard_completed or self._startup_abort_requested():
                        # Never start Cartographer after an operator stop or the
                        # start-confirmation timeout fired during service shutdown.
                        session_return_code = 0
                    else:
                        command = [
                            *INNER_LAUNCH,
                            *_frontier_source_launch_args(),
                            *launch_args,
                        ]
                        print(f'[INFO] Starting isolated mapping session: {command}',
                              flush=True)
                        self.child = subprocess.Popen(command, start_new_session=True)
                        return_code = self._wait_for_inner_launch()
                        if self.received_signal is not None and return_code in (-2, 130):
                            session_return_code = 0
                        elif self.start_confirm_timeout_triggered:
                            session_return_code = 0
                        # The manager SIGINTs the inner launch after either an approved
                        # return-home completion or the constrained cancellation flow.
                        elif return_code in (-2, 130):
                            session_return_code = 0
                        else:
                            session_return_code = return_code
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                print(f'[ERROR] Cartographer session failed: {exc}',
                      file=sys.stderr, flush=True)
                session_return_code = (
                    75 if self.navigation_release_failed else 1)
            finally:
                if self.child is not None and self.child.poll() is None:
                    if not self._terminate_process_group(self.child):
                        self.process_termination_failed = True
                if self.stop_attempted:
                    if self.process_termination_failed:
                        restore_succeeded = False
                        print(
                            '[CRITICAL] A managed Cartographer/service-guard '
                            'process did not exit; functional services will not '
                            'be restarted concurrently.',
                            file=sys.stderr,
                            flush=True,
                        )
                    else:
                        print('[INFO] Restarting the fixed functional-service '
                              'allowlist.', flush=True)
                        try:
                            self._service_guard('restart')
                        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                            restore_succeeded = False
                            print(
                                f'[CRITICAL] Failed to restart functional services: '
                                f'{exc}', file=sys.stderr, flush=True)
                if self.vehicle_readiness_wait_failed:
                    self._write_vehicle_readiness_timeout_status()
                elif self.navigation_release_failed:
                    self._write_navigation_release_failure_status()
                else:
                    self._write_final_session_status(restore_succeeded)
                # 2026-10-13 (P24): restore map-mode navigation AFTER the
                # terminal state is on disk.  The restore waits for a settle
                # window plus a bounded poll, so doing it first kept the
                # session in `finalizing` for up to ~100 s -- which the mini
                # program surfaces as a failed return home (measured 17:32:
                # return home had already settled at 0.243 m).  A restore
                # failure is reported but deliberately does NOT clear
                # restore_succeeded: the functional services (follow /
                # recharge) are already back and the vehicle is usable, so
                # degrading the session into recovery_required would only
                # stall the mini program.
                if self.stop_attempted and not self.process_termination_failed:
                    # 2026-10-13 (P25): hand the restore to a DETACHED helper
                    # instead of waiting for it here.  The bridge refuses
                    # `start_launch` while it still considers this mapping
                    # session to own vehicle motion, and it only observes the
                    # release after this runner (and the launch it owns) has
                    # exited -- so waiting deadlocks.  Measured 17:45: all four
                    # in-runner attempts were refused, the session stayed at
                    # mappingSession='stopping', map-mode navigation never came
                    # up, and the runner blocked for ~150 s while the mini
                    # program sat on "finishing".
                    self.navigation_restore_failed = False
                    self.navigation_restore_reason = ''
                    try:
                        if not self._spawn_navigation_restore_helper():
                            self.navigation_restore_failed = True
                            self.navigation_restore_reason = (
                                'could not spawn the detached navigation '
                                'restore helper')
                    except Exception as exc:  # noqa: BLE001 - 绝不阻断收尾
                        self.navigation_restore_failed = True
                        self.navigation_restore_reason = str(exc)
                        print(
                            f'[WARNING] Map-mode navigation restore helper '
                            f'could not be spawned: {exc}',
                            file=sys.stderr, flush=True)
            return session_return_code if restore_succeeded else 1


def main() -> None:
    runner = SessionRunner()
    raise SystemExit(runner.run(sys.argv[1:]))


if __name__ == '__main__':
    main()
