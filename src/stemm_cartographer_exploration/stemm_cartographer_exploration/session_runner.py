#!/usr/bin/env python3
"""Outer session runner that always restores the fixed functional services."""

import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys


SERVICE_GUARD = '/usr/local/sbin/stemm-cartographer-service-guard'
LOCK_FILE = '/tmp/stemm-cartographer-exploration.lock'
INNER_LAUNCH = (
    'ros2', 'launch', 'stemm_cartographer_exploration',
    'stemm_cartographer_auto_mapping.launch.py',
)


class SessionRunner:
    def __init__(self) -> None:
        self.child = None
        self.stop_attempted = False
        self.received_signal = None

    def _service_guard(self, action: str) -> None:
        timeout_sec = 120.0 if action == 'stop' else 60.0
        subprocess.run(
            ['sudo', '-n', SERVICE_GUARD, action],
            check=True,
            timeout=timeout_sec,
        )

    def _forward_signal(self, signum, _frame) -> None:
        self.received_signal = signum
        if self.child is None or self.child.poll() is not None:
            return
        try:
            os.killpg(self.child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

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

            session_return_code = 1
            restore_succeeded = True
            try:
                self.stop_attempted = True
                print('[INFO] Stopping the fixed functional-service allowlist.',
                      flush=True)
                self._service_guard('stop')
                command = [*INNER_LAUNCH, *launch_args]
                print(f'[INFO] Starting isolated mapping session: {command}',
                      flush=True)
                self.child = subprocess.Popen(command, start_new_session=True)
                return_code = self.child.wait()
                if self.received_signal is not None and return_code in (-2, 130):
                    session_return_code = 0
                # The manager intentionally SIGINTs the inner launch after a
                # successful return-home sequence.
                elif return_code in (-2, 130):
                    session_return_code = 0
                else:
                    session_return_code = return_code
            except (OSError, subprocess.SubprocessError) as exc:
                print(f'[ERROR] Cartographer session failed: {exc}',
                      file=sys.stderr, flush=True)
                session_return_code = 1
            finally:
                if self.child is not None and self.child.poll() is None:
                    try:
                        os.killpg(self.child.pid, signal.SIGINT)
                        self.child.wait(timeout=15.0)
                    except (ProcessLookupError, subprocess.TimeoutExpired):
                        try:
                            os.killpg(self.child.pid, signal.SIGTERM)
                        except ProcessLookupError:
                            pass
                if self.stop_attempted:
                    print('[INFO] Restarting the fixed functional-service '
                          'allowlist.', flush=True)
                    try:
                        self._service_guard('restart')
                    except (OSError, subprocess.SubprocessError) as exc:
                        restore_succeeded = False
                        print(
                            f'[CRITICAL] Failed to restart functional services: '
                            f'{exc}', file=sys.stderr, flush=True)
            return session_return_code if restore_succeeded else 1


def main() -> None:
    runner = SessionRunner()
    raise SystemExit(runner.run(sys.argv[1:]))


if __name__ == '__main__':
    main()
