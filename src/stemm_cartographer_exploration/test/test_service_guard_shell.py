import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
UNITS = (
    'low-battery-auto-recharge.service',
    'wheeltec-recharge-manager.service',
    'bodyfollow-nav2.service',
    'device-reporter.service',
)


FAKE_SYSTEMCTL = r'''#!/usr/bin/env bash
set -u
command_name=$1
shift
unit=${!#}
state_file="${FAKE_STATE_DIR}/${unit}"
state=inactive
if [[ -f "${state_file}" ]]; then
  state=$(<"${state_file}")
fi
case "${command_name}" in
  is-active)
    [[ "${state}" == active ]]
    ;;
  show)
    if [[ "${state}" == active || "${state}" == deactivating ]]; then
      main_pid=1234
    else
      main_pid=0
    fi
    printf 'MainPID=%s\nActiveState=%s\n' "${main_pid}" "${state}"
    ;;
  stop)
    printf 'stop %s\n' "${unit}" >>"${FAKE_LOG}"
    if [[ "${unit}" == device-reporter.service &&
          ! -f "${FAKE_STATE_DIR}/device-reporter.killed" ]]; then
      printf 'deactivating\n' >"${state_file}"
    else
      printf 'inactive\n' >"${state_file}"
    fi
    ;;
  kill)
    printf 'kill %s\n' "${unit}" >>"${FAKE_LOG}"
    : >"${FAKE_STATE_DIR}/device-reporter.killed"
    printf 'inactive\n' >"${state_file}"
    ;;
  restart)
    printf 'restart %s\n' "${unit}" >>"${FAKE_LOG}"
    if [[ "${unit}" == "${FAKE_RESTART_FAIL_UNIT:-}" ]]; then
      printf 'failed\n' >"${state_file}"
      exit 1
    fi
    printf 'active\n' >"${state_file}"
    ;;
  *)
    printf 'unexpected fake systemctl command: %s\n' \
      "${command_name}" >&2
    exit 2
    ;;
esac
'''


def _write_executable(path: Path, text: str) -> None:
    path.write_text(text, encoding='utf-8')
    path.chmod(0o755)


def _make_harness(tmp_path: Path):
    fake_bin = tmp_path / 'bin'
    state_dir = tmp_path / 'state'
    fake_bin.mkdir()
    state_dir.mkdir()
    log_path = tmp_path / 'calls.log'
    log_path.write_text('', encoding='utf-8')

    _write_executable(fake_bin / 'systemctl', FAKE_SYSTEMCTL)
    _write_executable(
        fake_bin / 'sleep',
        '#!/usr/bin/env bash\nexit 0\n',
    )
    _write_executable(
        fake_bin / 'timeout',
        '#!/usr/bin/env bash\n'
        'printf "recharge_stop\\n" >>"${FAKE_LOG}"\n'
        'exit 0\n',
    )
    fake_helper = tmp_path / 'recharge-helper'
    _write_executable(fake_helper, '#!/usr/bin/env bash\nexit 0\n')

    source = (
        ROOT / 'deploy' / 'stemm-cartographer-service-guard'
    ).read_text(encoding='utf-8')
    source = source.replace(
        'RECHARGE_STOP_HELPER=/home/wheeltec/wheeltec_ros2/install/'
        'stemm_cartographer_exploration/lib/'
        'stemm_cartographer_exploration/'
        'stemm_cartographer_recharge_stop_guard',
        f'RECHARGE_STOP_HELPER={fake_helper}',
    )
    guard = tmp_path / 'service-guard'
    _write_executable(guard, source)

    environment = os.environ.copy()
    environment.update({
        'PATH': f'{fake_bin}:{environment["PATH"]}',
        'FAKE_STATE_DIR': str(state_dir),
        'FAKE_LOG': str(log_path),
    })
    return guard, state_dir, log_path, environment


def _set_all_states(state_dir: Path, state: str) -> None:
    for unit in UNITS:
        (state_dir / unit).write_text(f'{state}\n', encoding='utf-8')


def test_hung_feature_is_bounded_and_mic_remains_active(tmp_path):
    guard, state_dir, log_path, environment = _make_harness(tmp_path)
    _set_all_states(state_dir, 'active')
    (state_dir / 'wheeltec-mic.service').write_text(
        'active\n', encoding='utf-8')

    result = subprocess.run(
        ['/bin/bash', str(guard), 'stop'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
        env=environment,
    )

    assert result.returncode == 0, result.stderr
    assert 'graceful stop timed out' in result.stderr
    assert 'stopping feature service: device-reporter.service' in result.stdout
    assert 'stopped feature service: device-reporter.service' in result.stdout
    calls = log_path.read_text(encoding='utf-8').splitlines()
    assert calls.index('recharge_stop') < calls.index(
        'stop wheeltec-recharge-manager.service')
    assert calls.count('kill device-reporter.service') == 1
    assert all(
        (state_dir / unit).read_text(encoding='utf-8').strip() == 'inactive'
        for unit in UNITS
    )
    assert (state_dir / 'wheeltec-mic.service').read_text(
        encoding='utf-8').strip() == 'active'
    assert not any('wheeltec-mic.service' in call for call in calls)


def test_restart_attempts_every_unit_and_reports_aggregate_failure(tmp_path):
    guard, state_dir, log_path, environment = _make_harness(tmp_path)
    _set_all_states(state_dir, 'inactive')
    environment['FAKE_RESTART_FAIL_UNIT'] = 'wheeltec-recharge-manager.service'

    result = subprocess.run(
        ['/bin/bash', str(guard), 'restart'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
        env=environment,
    )

    assert result.returncode == 1
    calls = log_path.read_text(encoding='utf-8').splitlines()
    assert [line for line in calls if line.startswith('restart ')] == [
        'restart bodyfollow-nav2.service',
        'restart wheeltec-recharge-manager.service',
        'restart low-battery-auto-recharge.service',
        'restart device-reporter.service',
    ]
