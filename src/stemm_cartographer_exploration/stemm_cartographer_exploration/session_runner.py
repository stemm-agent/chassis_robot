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


SERVICE_GUARD = '/usr/local/sbin/stemm-cartographer-service-guard'
LOCK_FILE = '/tmp/stemm-cartographer-exploration.lock'
INNER_LAUNCH = (
    'ros2', 'launch', 'stemm_cartographer_exploration',
    'stemm_cartographer_auto_mapping.launch.py',
)
SESSION_STATUS_PATH = Path(os.getenv(
    'STEMM_CARTOGRAPHER_SESSION_STATUS_PATH',
    '/home/wheeltec/.local/state/stemm-cartographer-session.json',
))
SESSION_TASK_PATH = Path(os.getenv(
    'STEMM_CARTOGRAPHER_TASK_PATH',
    '/home/wheeltec/.local/state/stemm-cartographer-task.json',
))


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
        self.session_started_at_epoch_ms = 0
        try:
            timeout_sec = float(os.getenv(
                'STEMM_CARTOGRAPHER_START_CONFIRM_TIMEOUT_SEC', '100'
            ))
        except (TypeError, ValueError):
            timeout_sec = 100.0
        self.start_confirm_timeout_sec = max(0.0, timeout_sec)
        self.start_confirm_deadline = None
        self.start_confirmed = False
        self.start_confirm_timeout_triggered = False

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
                raise subprocess.CalledProcessError(process.returncode, command)
            return True
        finally:
            if process.poll() is None:
                if not self._terminate_process_group(process):
                    self.process_termination_failed = True

    def _wait_for_inner_launch(self) -> int:
        if self.child is None:
            return 1
        while self.child.poll() is None:
            if self._startup_abort_requested():
                if not self._terminate_process_group(self.child):
                    self.process_termination_failed = True
                    return 1
                return self.child.returncode if self.child.returncode is not None else 1
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
                # current task's 100-second timeout from the mini-program.
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
        if not session_task_id:
            candidate = session.get('taskId')
            session_task_id = candidate.strip() if isinstance(candidate, str) else ''
        if not session_task_id:
            return
        if str(session.get('taskId') or '') != session_task_id:
            print(
                '[WARNING] Cartographer session task ID changed; refusing to '
                'overwrite another session status.',
                file=sys.stderr,
                flush=True,
            )
            return

        updated_at_ms = int(session.get('updatedAtEpochMs') or 0)
        if updated_at_ms < self.session_started_at_epoch_ms:
            print(
                '[WARNING] Cartographer session state predates this runner; '
                'refusing to publish a terminal state.',
                file=sys.stderr,
                flush=True,
            )
            return

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
            self._arm_start_confirmation_timeout()

            session_return_code = 1
            restore_succeeded = True
            try:
                self.stop_attempted = True
                print('[INFO] Stopping the fixed functional-service allowlist.',
                      flush=True)
                guard_completed = self._service_guard('stop', interruptible=True)
                if not guard_completed or self._startup_abort_requested():
                    # Never start Cartographer after an operator stop or the
                    # start-confirmation timeout fired during service shutdown.
                    session_return_code = 0
                else:
                    command = [*INNER_LAUNCH, *launch_args]
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
                session_return_code = 1
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
                self._write_final_session_status(restore_succeeded)
            return session_return_code if restore_succeeded else 1


def main() -> None:
    runner = SessionRunner()
    raise SystemExit(runner.run(sys.argv[1:]))


if __name__ == '__main__':
    main()
