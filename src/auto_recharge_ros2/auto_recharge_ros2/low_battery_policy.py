#!/usr/bin/env python3
# coding=utf-8

import math


class RechargeEpisodeLatch:
    """Freeze one accepted low-battery episode until it reaches a terminal state."""

    def __init__(self):
        self.frozen_trigger_voltage = None
        self.audio_announced = False

    @property
    def voltage_frozen(self):
        return self.frozen_trigger_voltage is not None

    def freeze(self, trigger_voltage):
        value = float(trigger_voltage)
        if not math.isfinite(value):
            raise ValueError('Trigger voltage must be finite.')
        if self.voltage_frozen:
            return False
        self.frozen_trigger_voltage = value
        return True

    def mark_audio_announced(self):
        if self.audio_announced:
            return False
        self.audio_announced = True
        return True

    def reset_audio(self):
        if not self.voltage_frozen:
            self.audio_announced = False

    def release(self, reset_audio):
        previous_voltage = self.frozen_trigger_voltage
        self.frozen_trigger_voltage = None
        if reset_audio:
            self.audio_announced = False
        return previous_voltage


class LowBatteryEvidence:
    """Track fresh, independent battery samples used to trigger recharging."""

    def __init__(
        self,
        low_voltage_threshold,
        confirm_count,
        minimum_valid_voltage,
        maximum_valid_voltage,
        voltage_stale_timeout_sec,
        charging_state_stale_timeout_sec,
    ):
        self.low_voltage_threshold = float(low_voltage_threshold)
        self.confirm_count = int(confirm_count)
        self.minimum_valid_voltage = float(minimum_valid_voltage)
        self.maximum_valid_voltage = float(maximum_valid_voltage)
        self.voltage_stale_timeout_sec = float(voltage_stale_timeout_sec)
        self.charging_state_stale_timeout_sec = float(
            charging_state_stale_timeout_sec
        )
        self._validate_configuration()

        self.current_voltage = None
        self.voltage_updated_at = None
        self.is_charging = False
        self.charging_updated_at = None
        self.low_voltage_count = 0

    def _validate_configuration(self):
        numeric_values = (
            self.low_voltage_threshold,
            self.minimum_valid_voltage,
            self.maximum_valid_voltage,
            self.voltage_stale_timeout_sec,
            self.charging_state_stale_timeout_sec,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError('Low-battery policy parameters must be finite.')
        if self.confirm_count < 1:
            raise ValueError('low_voltage_confirm_count must be at least 1.')
        if self.minimum_valid_voltage >= self.maximum_valid_voltage:
            raise ValueError(
                'minimum_valid_voltage must be lower than maximum_valid_voltage.'
            )
        if not (
            self.minimum_valid_voltage
            <= self.low_voltage_threshold
            <= self.maximum_valid_voltage
        ):
            raise ValueError(
                'low_voltage_threshold must be inside the valid voltage range.'
            )
        if (
            self.voltage_stale_timeout_sec <= 0.0
            or self.charging_state_stale_timeout_sec <= 0.0
        ):
            raise ValueError('Freshness timeouts must be greater than zero.')

    def record_charging_state(self, is_charging, now):
        self.is_charging = bool(is_charging)
        self.charging_updated_at = float(now)
        if self.is_charging:
            self.low_voltage_count = 0

    def record_voltage(self, voltage, now):
        value = float(voltage)
        now = float(now)
        if (
            not math.isfinite(value)
            or value < self.minimum_valid_voltage
            or value > self.maximum_valid_voltage
        ):
            self.current_voltage = None
            self.voltage_updated_at = None
            self.low_voltage_count = 0
            return False

        self.current_voltage = value
        self.voltage_updated_at = now

        if not self.charging_state_is_fresh(now) or self.is_charging:
            self.low_voltage_count = 0
        elif value <= self.low_voltage_threshold:
            # This method is called only for a newly received voltage message,
            # so every increment represents an independent chassis sample.
            self.low_voltage_count += 1
        else:
            self.low_voltage_count = 0
        return True

    def voltage_is_fresh(self, now):
        return (
            self.current_voltage is not None
            and self.voltage_updated_at is not None
            and float(now) - self.voltage_updated_at
            <= self.voltage_stale_timeout_sec
        )

    def charging_state_is_fresh(self, now):
        return (
            self.charging_updated_at is not None
            and float(now) - self.charging_updated_at
            <= self.charging_state_stale_timeout_sec
        )

    def ready_to_trigger(self, now):
        return (
            self.voltage_is_fresh(now)
            and self.charging_state_is_fresh(now)
            and not self.is_charging
            and self.low_voltage_count >= self.confirm_count
        )

    def reset_confirmation(self):
        self.low_voltage_count = 0

    def clear_voltage_evidence(self):
        self.current_voltage = None
        self.voltage_updated_at = None
        self.low_voltage_count = 0

    def discard_stale_evidence(self, now):
        if not self.voltage_is_fresh(now) or not self.charging_state_is_fresh(now):
            self.low_voltage_count = 0
