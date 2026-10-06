#!/usr/bin/env python3
# coding=utf-8

import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name('low_battery_policy.py')
if not MODULE_PATH.exists():
    MODULE_PATH = (
        Path(__file__).resolve().parents[1]
        / 'auto_recharge_ros2'
        / 'low_battery_policy.py'
    )
SPEC = importlib.util.spec_from_file_location('low_battery_policy', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
LowBatteryEvidence = MODULE.LowBatteryEvidence
RechargeEpisodeLatch = MODULE.RechargeEpisodeLatch


def make_policy(confirm_count=6):
    return LowBatteryEvidence(
        low_voltage_threshold=22.2,
        confirm_count=confirm_count,
        minimum_valid_voltage=5.0,
        maximum_valid_voltage=35.0,
        voltage_stale_timeout_sec=5.0,
        charging_state_stale_timeout_sec=5.0,
    )


def test_only_fresh_voltage_messages_increment_confirmation():
    policy = make_policy()
    policy.record_charging_state(False, 0.0)
    policy.record_voltage(22.1, 0.1)

    assert policy.low_voltage_count == 1
    for now in (0.2, 0.5, 1.0, 2.0, 4.0):
        assert not policy.ready_to_trigger(now)
        assert policy.low_voltage_count == 1


def test_six_independent_low_voltage_samples_trigger():
    policy = make_policy()
    policy.record_charging_state(False, 0.0)

    for index in range(6):
        assert policy.record_voltage(22.1, 0.1 + index * 0.1)

    assert policy.low_voltage_count == 6
    assert policy.ready_to_trigger(0.7)


def test_exact_22_2v_threshold_counts_as_low():
    policy = make_policy(confirm_count=1)
    policy.record_charging_state(False, 0.0)

    assert policy.record_voltage(22.2, 0.1)
    assert policy.low_voltage_count == 1
    assert policy.ready_to_trigger(0.2)

    assert policy.record_voltage(22.21, 0.3)
    assert policy.low_voltage_count == 0
    assert not policy.ready_to_trigger(0.4)


def test_stale_voltage_or_charging_state_blocks_trigger():
    policy = make_policy(confirm_count=1)
    policy.record_charging_state(False, 0.0)
    policy.record_voltage(22.1, 0.1)

    assert policy.ready_to_trigger(0.2)
    assert not policy.ready_to_trigger(5.2)
    policy.discard_stale_evidence(5.2)
    assert policy.low_voltage_count == 0


def test_charging_state_blocks_and_resets_confirmation():
    policy = make_policy(confirm_count=2)
    policy.record_charging_state(False, 0.0)
    policy.record_voltage(22.1, 0.1)
    policy.record_charging_state(True, 0.2)
    policy.record_voltage(22.0, 0.3)

    assert policy.low_voltage_count == 0
    assert not policy.ready_to_trigger(0.4)


def test_invalid_voltage_is_rejected_and_clears_evidence():
    policy = make_policy(confirm_count=1)
    policy.record_charging_state(False, 0.0)
    policy.record_voltage(22.1, 0.1)

    assert not policy.record_voltage(float('nan'), 0.2)
    assert policy.current_voltage is None
    assert policy.low_voltage_count == 0


def test_episode_latch_freezes_voltage_and_audio_until_success():
    episode = RechargeEpisodeLatch()

    assert episode.freeze(22.2)
    assert episode.voltage_frozen
    assert episode.frozen_trigger_voltage == 22.2
    assert not episode.freeze(21.9)
    assert episode.frozen_trigger_voltage == 22.2

    assert episode.mark_audio_announced()
    assert not episode.mark_audio_announced()
    episode.reset_audio()
    assert not episode.mark_audio_announced()

    assert episode.release(reset_audio=True) == 22.2
    assert not episode.voltage_frozen
    assert episode.mark_audio_announced()


def test_failure_release_preserves_audio_deduplication():
    episode = RechargeEpisodeLatch()
    assert episode.freeze(22.1)
    assert episode.mark_audio_announced()

    assert episode.release(reset_audio=False) == 22.1
    assert not episode.voltage_frozen
    assert not episode.mark_audio_announced()

    episode.reset_audio()
    assert episode.mark_audio_announced()


def test_clearing_voltage_requires_fresh_samples_after_release():
    policy = make_policy(confirm_count=1)
    policy.record_charging_state(False, 0.0)
    policy.record_voltage(22.2, 0.1)
    assert policy.ready_to_trigger(0.2)

    policy.clear_voltage_evidence()
    assert policy.current_voltage is None
    assert not policy.ready_to_trigger(0.3)


if __name__ == '__main__':
    tests = sorted(
        (name, value)
        for name, value in globals().items()
        if name.startswith('test_') and callable(value)
    )
    for name, test in tests:
        test()
        print('PASS', name)
    print('TOTAL', len(tests))
