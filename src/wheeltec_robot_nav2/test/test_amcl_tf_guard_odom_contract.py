#!/usr/bin/env python3
"""Executable source contract tests for amcl_tf_guard odom/AMCL safety logic.

Run directly with:
  python3 test/test_amcl_tf_guard_odom_contract.py

These tests intentionally need no ROS graph.  They protect the safety-critical
formulas and state/diagnostic wiring even when this patch is reviewed outside a
built ROS workspace.
"""

from pathlib import Path
import math
import unittest


SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "amcl_tf_guard.cpp"
).read_text(encoding="utf-8")
LAUNCH_SOURCE = (
    Path(__file__).resolve().parents[1] / "launch" / "wheeltec_nav2.launch.py"
).read_text(encoding="utf-8")


def section(start: str, end: str) -> str:
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


def allowed_step(
    dt: float,
    max_speed: float,
    max_acceleration: float,
    slack: float,
    discontinuity_floor: float,
) -> float:
    bounded_dt = max(0.0, dt)
    kinematic_limit = (
        slack
        + max_speed * bounded_dt
        + 0.5 * max_acceleration * bounded_dt * bounded_dt
    )
    return max(discontinuity_floor, kinematic_limit)


def silence_should_hold(
    validated_age: float,
    since_motion_crossing: float,
    timeout: float = 3.0,
    motion_grace: float = 1.0,
) -> bool:
    return validated_age > timeout and since_motion_crossing > motion_grace


def drop_historical_amcl_packets(
    last_validated_stamp: float, packet_stamps: list[float]
) -> tuple[float, int, int]:
    """Mirror the drop-only gate before guard state mutation.

    Returns (new last-validated stamp, committed fresh packets, dropped packets).
    """
    committed = 0
    dropped = 0
    for stamp in packet_stamps:
        if stamp <= last_validated_stamp:
            dropped += 1
            continue
        last_validated_stamp = stamp
        committed += 1
    return last_validated_stamp, committed, dropped


def bounded_recovery_nomotion_requests(
    timer_times: list[float],
    *,
    integrity_inhibit: bool,
    max_requests: int = 12,
    max_window_sec: float = 8.0,
) -> int:
    """Behavior mirror of one ACTIVE anomaly-recovery no-motion epoch."""
    if integrity_inhibit or not timer_times:
        return 0
    started = timer_times[0]
    requests = 0
    for now in timer_times:
        if requests >= max_requests or now - started >= max_window_sec:
            break
        requests += 1
    return requests


def timing_fault_should_hold(
    observations: list[tuple[float, float, int, str]],
    soft_limit: float,
    hard_limit: float,
    min_soft_duration: float = 0.25,
) -> bool:
    """Mirror the same-reason steady-duration soft/hard timing policy.

    Each tuple is (steady_time, offending_value, source_stamp, reason). Repeated
    timer callbacks may observe one source stamp, but they do not manufacture
    distinct source evidence.
    """
    active_reason = None
    first_seen = 0.0
    distinct_stamps: set[int] = set()
    for steady_time, value, source_stamp, reason in observations:
        if soft_limit <= 0.0 or value <= soft_limit:
            active_reason = None
            distinct_stamps.clear()
        elif hard_limit > 0.0 and value >= hard_limit:
            return True
        else:
            if active_reason != reason:
                active_reason = reason
                first_seen = steady_time
                distinct_stamps.clear()
            distinct_stamps.add(source_stamp)
            if steady_time - first_seen >= min_soft_duration:
                return True
    return False


def bounded_anchor_moves(
    event_times: list[float],
    requested_step: float = 0.03,
    window_sec: float = 30.0,
    window_budget: float = 0.12,
    session_budget: float = 2.0,
) -> float:
    """Executable mirror of the rolling-window plus lifetime fuse."""
    events: list[tuple[float, float]] = []
    session_used = 0.0
    for now in event_times:
        events = [(stamp, amount) for stamp, amount in events if now - stamp <= window_sec]
        window_used = sum(amount for _, amount in events)
        amount = min(
            requested_step,
            max(0.0, window_budget - window_used),
            max(0.0, session_budget - session_used),
        )
        if amount > 0.0:
            events.append((now, amount))
            session_used += amount
    return session_used


def dynamic_window_budget(
    base: float, odom_motion: float, gain: float, motion_cap: float
) -> float:
    return base + min(motion_cap, gain * odom_motion)


def directional_step_limit(
    net_x: float,
    net_y: float,
    direction_x: float,
    direction_y: float,
    budget: float,
    requested: float,
) -> float:
    norm = math.hypot(direction_x, direction_y)
    if budget <= 0.0 or requested <= 0.0 or norm <= 1.0e-12:
        return 0.0
    ux, uy = direction_x / norm, direction_y / norm
    projection = net_x * ux + net_y * uy
    discriminant = projection**2 + budget**2 - (net_x**2 + net_y**2)
    if discriminant < -1.0e-9:
        return 0.0
    return min(requested, max(0.0, -projection + math.sqrt(max(0.0, discriminant))))


def cumulative_overrun_outcome(
    observations: list[tuple[float, int, bool, bool]],
    *,
    required_samples: int = 3,
    min_duration: float = 0.5,
    max_duration: float = 1.5,
) -> tuple[str, int]:
    """Mirror ACTIVE cumulative-overrun evidence handling.

    Each tuple is (steady_time, source_stamp, eligible, back_inside).  Eligibility
    represents all per-sample pose age, same-stamp odom, scan, correction,
    continuity, session-fuse, and integrity checks.
    """
    started = None
    last_stamp = None
    count = 0
    for steady_time, source_stamp, eligible, back_inside in observations:
        if started is not None and steady_time - started >= max_duration:
            return "hold_deadline", count
        if back_inside:
            return "cleared", 0
        if not eligible:
            return "hold_immediate", count
        if started is None:
            started = steady_time
        if last_stamp is None or source_stamp > last_stamp:
            count += 1
            last_stamp = source_stamp
        if count >= required_samples and steady_time - started >= min_duration:
            return "hold_confirmed", count
    return "pending", count


def trusted_override_outcome(
    observations: list[tuple[float, int, str, bool, bool]],
    *,
    already_hold: bool = False,
) -> tuple[str, int]:
    """Small behavioral model of VERIFY_TRUSTED / TRUSTED_OVERRIDE.

    Each tuple is (steady_time, scan_stamp, trusted_quality, raw_inside,
    stationary).  trusted_quality is ``good``, ``soft`` or ``catastrophic``.
    Raw AMCL affects only the optional return to NORMAL; it cannot make a good
    trusted transform enter HARD_HOLD.
    """
    mode = "soft_hold" if already_hold else "verify"
    started: float | None = None
    last_stamp: int | None = None
    last_failure_stamp: int | None = None
    count = 0
    failures = 0
    for now, scan_stamp, quality, raw_inside, stationary in observations:
        if quality == "catastrophic":
            return "hard_hold", failures
        if quality == "soft":
            count = 0
            started = None
            last_stamp = None
            if last_failure_stamp is None or scan_stamp > last_failure_stamp:
                failures += 1
                last_failure_stamp = scan_stamp
            if failures >= 2:
                return "hard_hold", failures
            continue

        failures = 0
        last_failure_stamp = None
        if mode == "override":
            if raw_inside:
                return "normal_return_pending", count
            continue
        if mode == "soft_hold" and not stationary:
            count = 0
            started = None
            last_stamp = None
            continue
        if started is None:
            started = now
        if last_stamp is None or scan_stamp > last_stamp:
            count += 1
            last_stamp = scan_stamp
        required = 3 if mode == "soft_hold" else 2
        duration = 0.50 if mode == "soft_hold" else 0.25
        if count >= required and now - started >= duration:
            mode = "override"
            return mode, count
    return mode, count


def trusted_override_active_evidence_outcome(
    observations: list[tuple[float, int, bool]], timeout: float = 1.0
) -> str:
    """Mirror the ACTIVE new-scan steady deadline.

    Each tuple is (steady_time, scan_stamp, absolute_quality_good).  A good but
    repeated scan stamp is intentionally not fresh evidence.
    """
    last_good_time: float | None = None
    last_good_stamp: int | None = None
    for now, scan_stamp, good in observations:
        if last_good_time is not None and now - last_good_time >= timeout:
            return "hard_hold"
        if good and (last_good_stamp is None or scan_stamp > last_good_stamp):
            last_good_time = now
            last_good_stamp = scan_stamp
    return "active"


def rolling_catchup_step(
    *,
    candidate_gap_sec: float,
    desired_step: float,
    remaining_window_budget: float = 0.12,
    remaining_session_budget: float = 2.0,
    base_step: float = 0.03,
    nominal_gap_sec: float = 1.0,
    max_gap_sec: float = 4.0,
    max_step_scale: float = 4.0,
) -> float:
    """Mirror the bounded missed-callback catch-up amount."""
    if candidate_gap_sec <= 0.0 or candidate_gap_sec > max_gap_sec:
        return 0.0
    scale = min(
        max_step_scale,
        max(1.0, candidate_gap_sec / max(0.1, nominal_gap_sec)),
    )
    return min(
        desired_step,
        base_step * scale,
        max(0.0, remaining_window_budget),
        max(0.0, remaining_session_budget),
    )


def select_quality_scan_stamp(
    pose_stamp: float,
    scan_stamps: list[float],
    tolerance: float,
    allow_nearest: bool,
) -> float | None:
    """Executable mirror of exact-first, bounded nearest-scan selection."""
    for scan_stamp in reversed(scan_stamps):
        if scan_stamp == pose_stamp:
            return scan_stamp
    if not allow_nearest or not scan_stamps:
        return None
    nearest = min(reversed(scan_stamps), key=lambda stamp: abs(stamp - pose_stamp))
    return nearest if abs(nearest - pose_stamp) <= tolerance else None


def covariance_policy_accepts(
    xx: float,
    xy: float,
    yx: float,
    yy: float,
    yaw: float,
    strict: bool,
    max_xy_eigenvalue: float = 0.04,
    max_yaw: float = 0.0225,
) -> bool:
    """Executable mirror of structural-only versus strict covariance policy."""
    if not all(math.isfinite(value) for value in (xx, xy, yx, yy, yaw)):
        return False
    symmetric_xy = 0.5 * (xy + yx)
    determinant = xx * yy - symmetric_xy * symmetric_xy
    discriminant = max(0.0, (xx - yy) ** 2 + 4.0 * symmetric_xy**2)
    lambda_max = 0.5 * (xx + yy + math.sqrt(discriminant))
    zero = abs(xx) <= 1.0e-10 and abs(yy) <= 1.0e-10 and abs(yaw) <= 1.0e-10
    structural = (
        not zero
        and xx >= 0.0
        and yy >= 0.0
        and yaw >= 0.0
        and determinant >= -1.0e-8
        and abs(xy - yx) <= 1.0e-5
    )
    if not structural:
        return False
    return not strict or (lambda_max <= max_xy_eigenvalue and yaw <= max_yaw)


class OdomIntegrityContractTest(unittest.TestCase):
    def test_real_source_dt_and_gap_guards_are_used(self) -> None:
        monitor = section("bool monitor_odom_integrity", "bool accept_candidate")
        self.assertIn(
            "metrics.dt = (odom_stamp - last_odom_stamp_).seconds();", monitor
        )
        self.assertIn("metrics.dt <= 0.0", monitor)
        self.assertIn('consider_gap("source_stamp_gap", metrics.dt)', monitor)
        self.assertIn("hard_odom_observation_gap_sec_", monitor)
        self.assertIn('timing_reason = "non_monotonic_source_stamp"', monitor)
        self.assertNotIn("timing_fault = odom_timing_hold_", monitor)

    def test_dynamic_envelope_and_discontinuity_floor_are_both_required(self) -> None:
        helper = section("double allowed_odom_step", "tf2::Transform transform_from_pose")
        self.assertIn("max_speed * bounded_dt", helper)
        self.assertIn("0.5 * max_acceleration * bounded_dt * bounded_dt", helper)
        self.assertIn("return std::max(discontinuity_floor, kinematic_limit);", helper)

        monitor = section("bool monitor_odom_integrity", "bool accept_candidate")
        jump_assignment = monitor.rsplit("jump_detected =", 1)[1].split(
            "large_step_allowed_by_kinematics =", 1
        )[0]
        self.assertIn("metrics.translation > metrics.linear_limit", jump_assignment)
        self.assertIn("metrics.yaw_delta > metrics.angular_limit", jump_assignment)
        self.assertNotIn(
            "metrics.translation > max_odom_step_translation_", jump_assignment
        )
        self.assertNotIn("metrics.yaw_delta > max_odom_step_yaw_", jump_assignment)
        self.assertNotIn("timing_soft_violation", jump_assignment)
        self.assertIn("large step accepted by the dt-scaled kinematic envelope", monitor)

        # The incident's 0.356 rad step is not automatically a reset when the
        # real source interval is 100 ms: the configured 2 rad/s and 1.5 rad/s2
        # envelope permits 0.3575 rad.  A 0.360 rad step still exceeds it.
        angular_limit = allowed_step(0.1, 2.0, 1.5, 0.15, 0.35)
        self.assertAlmostEqual(angular_limit, 0.3575)
        self.assertLessEqual(0.356, angular_limit)
        self.assertGreater(0.360, angular_limit)

        # A longer but still valid source interval expands the physical
        # envelope; intervals beyond the configured gap are rejected before
        # this function is consulted.
        self.assertAlmostEqual(allowed_step(0.2, 2.0, 1.5, 0.15, 0.35), 0.58)

    def test_jump_diagnostic_contains_exact_stamp_pair_and_real_dt(self) -> None:
        monitor = section("bool monitor_odom_integrity", "bool accept_candidate")
        for token in (
            "source stamps %lld -> %lld ns",
            "real dt %.6f s",
            "kinematic %.3f / %.3f",
            "discontinuity floors %.3f / %.3f",
            "previous_integrity_stamp.nanoseconds()",
        ):
            self.assertIn(token, monitor)


class OdomTimingDebounceContractTest(unittest.TestCase):
    def test_single_near_threshold_age_sample_does_not_hold(self) -> None:
        # This is the measured false-pause case: 0.507 s is only 7 ms over
        # the configured 0.500 s soft threshold, and the following normal
        # sample clears the pending evidence.
        observations = [
            (0.0, 0.507, 100, "source_age"),
            (0.033, 0.043, 101, "source_age"),
        ]
        self.assertFalse(timing_fault_should_hold(observations, 0.5, 1.0))

    def test_same_stamp_timer_ticks_do_not_confirm_before_duration(self) -> None:
        observations = [
            (0.0, 0.507, 100, "source_age"),
            (0.033, 0.540, 100, "source_age"),
            (0.066, 0.573, 100, "source_age"),
        ]
        self.assertFalse(timing_fault_should_hold(observations, 0.5, 1.0))

    def test_sustained_same_reason_fault_confirms_by_steady_duration(self) -> None:
        observations = [
            (0.0, 0.507, 100, "source_age"),
            (0.20, 0.707, 100, "source_age"),
            (0.25, 0.757, 100, "source_age"),
        ]
        self.assertTrue(timing_fault_should_hold(observations, 0.5, 1.0))

    def test_healthy_new_stamp_clears_the_soft_window(self) -> None:
        observations = [
            (0.0, 0.507, 100, "source_age"),
            (0.20, 0.043, 101, "source_age"),
            (0.40, 0.600, 102, "source_age"),
        ]
        self.assertFalse(timing_fault_should_hold(observations, 0.5, 1.0))

    def test_hard_age_or_gap_fault_is_immediate(self) -> None:
        self.assertTrue(timing_fault_should_hold(
            [(0.0, 1.0, 100, "source_age")], 0.5, 1.0
        ))
        self.assertFalse(timing_fault_should_hold(
            [(0.0, 1.01, 100, "source_gap")], 1.0, 1.5
        ))
        self.assertTrue(timing_fault_should_hold(
            [(0.0, 1.5, 100, "source_gap")], 1.0, 1.5
        ))

    def test_source_policy_and_commit_freeze_are_wired(self) -> None:
        monitor = section("bool monitor_odom_integrity", "bool accept_candidate")
        callback = section("void on_amcl_pose", "void enter_hold")
        commit = section("bool accept_candidate", "void publish_last_trusted_transform")
        for token in (
            "OdomTimingSeverity::kSoft",
            "OdomTimingSeverity::kHard",
            "note_soft_odom_timing_fault_locked",
            'timing_hard_fault ? "hard-immediate" : "soft-confirmed"',
        ):
            self.assertIn(token, monitor)
        self.assertIn("odom_soft_timing_fault_samples_ > 0", callback)
        self.assertIn("odom_soft_timing_fault_samples_ > 0", commit)
        self.assertIn("!timing_soft_violation", monitor)
        self.assertIn('timing_reason = "non_monotonic_source_stamp"', monitor)
        self.assertIn("future_source_stamp", monitor)
        debounce = section(
            "bool note_soft_odom_timing_fault_locked",
            "int clear_soft_odom_timing_fault_locked",
        )
        self.assertIn("odom_soft_timing_fault_reason_ != reason", debounce)
        self.assertIn("source_stamp != odom_soft_timing_fault_last_stamp_", debounce)
        self.assertIn("now - odom_soft_timing_fault_started_at_", debounce)
        self.assertIn("odom_timing_soft_fault_min_duration_sec_", debounce)

    def test_launch_thresholds_encode_measured_jitter_margin(self) -> None:
        for token in (
            "'max_odom_tf_age_sec': 0.50",
            "'hard_odom_tf_age_sec': 1.0",
            "'max_odom_observation_gap_sec': 1.0",
            "'hard_odom_observation_gap_sec': 1.5",
            "'odom_timing_soft_fault_consecutive_samples': 3",
            "'odom_timing_soft_fault_min_duration_sec': 0.25",
        ):
            self.assertIn(token, LAUNCH_SOURCE)

    def test_build_type_is_optimized_but_honors_explicit_colcon_choice(self) -> None:
        cmake = (
            Path(__file__).resolve().parents[1] / "CMakeLists.txt"
        ).read_text(encoding="utf-8")
        self.assertIn("if(NOT CMAKE_BUILD_TYPE)", cmake)
        self.assertIn('set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING', cmake)
        self.assertNotIn('set(CMAKE_BUILD_TYPE "Debug")', cmake)
        self.assertNotIn("add_compile_options(-g)", cmake)


class AmclSilenceDiagnosticsContractTest(unittest.TestCase):
    def test_received_validated_and_accepted_clocks_are_independent(self) -> None:
        for token in (
            "last_amcl_receipt_",
            "last_validated_amcl_receipt_",
            "last_accepted_amcl_receipt_",
            "last_received_amcl_stamp_",
            "last_validated_amcl_stamp_",
            "last_accepted_amcl_stamp_",
        ):
            self.assertIn(token, SOURCE)
        self.assertIn("Ages are distinct: last received", SOURCE)
        self.assertIn("last basic timing/TF validated", SOURCE)
        self.assertIn("last trusted", SOURCE)

    def test_motion_crossing_grace_prevents_immediate_historical_age_hold(self) -> None:
        freshness = section("void monitor_amcl_freshness", "void request_nomotion_update_if_needed")
        self.assertIn(
            'declare_parameter<double>("amcl_silence_motion_grace_sec", 1.0)',
            SOURCE,
        )
        self.assertIn(
            "since_motion_crossing_sec > amcl_silence_motion_grace_sec_", freshness
        )
        self.assertIn(
            "validated_age_sec > amcl_silence_timeout_while_moving_sec_", freshness
        )
        self.assertIn("motion_grace_elapsed", freshness)
        self.assertIn(
            "validated_age_sec > amcl_silence_timeout_while_moving_sec_ &&\n"
            "          motion_grace_elapsed",
            freshness,
        )
        self.assertFalse(silence_should_hold(37.705, 0.5))
        self.assertTrue(silence_should_hold(37.705, 1.001))
        self.assertFalse(silence_should_hold(2.5, 5.0))

        monitor = section("bool monitor_odom_integrity", "bool accept_candidate")
        self.assertIn("if (!odom_moved_since_last_amcl_)", monitor)
        self.assertIn("amcl_motion_threshold_crossed_steady_ = steady_now", monitor)
        self.assertIn("amcl_motion_crossing_odom_stamp_ = odom_stamp", monitor)

        callback = section("void on_amcl_pose", "void enter_hold")
        self.assertIn("have_amcl_motion_threshold_crossing_ = false", callback)
        self.assertIn("odom_moved_since_last_amcl_ = false", callback)

    def test_historical_amcl_packet_is_drop_only_before_state_mutation(self) -> None:
        callback = section("void on_amcl_pose", "void enter_hold")
        historical_block = callback.split(
            "bool historical_amcl_pose = false;", 1
        )[1].split("const double pose_age_sec", 1)[0]
        historical_return = callback.split(
            "if (historical_amcl_pose) {", 1
        )[1].split("const double pose_age_sec", 1)[0]
        self.assertIn("stamp <= last_validated_amcl_stamp_", callback)
        self.assertIn("historical_amcl_drop_count_", historical_block)
        self.assertIn("return;", historical_return)
        for forbidden in (
            "last_validated_amcl_receipt_ =",
            "last_accepted_amcl_receipt_ =",
            "motion_reference_odom_base_ =",
            "last_trusted_map_odom_ =",
            "anomaly_active_ =",
            "set_auto_reanchor_integrity_inhibit_locked",
        ):
            self.assertNotIn(forbidden, historical_block)
        self.assertNotIn(
            'enter_amcl_timing_hold("non_monotonic_amcl_stamp"', callback
        )

        # One late packet is ignored and the following monotonic packet commits.
        stamp, committed, dropped = drop_historical_amcl_packets(
            100.0, [99.085, 100.1]
        )
        self.assertEqual((stamp, committed, dropped), (100.1, 1, 1))

    def test_only_historical_packets_cannot_mask_validated_stream_silence(self) -> None:
        stamp, committed, dropped = drop_historical_amcl_packets(
            100.0, [99.9, 99.8, 100.0, 99.7]
        )
        self.assertEqual(stamp, 100.0)
        self.assertEqual(committed, 0)
        self.assertEqual(dropped, 4)
        # The production watchdog keys off validated age, not receipt age.
        self.assertTrue(silence_should_hold(3.001, 1.001, timeout=3.0))


class RollingAnchorAndReanchorContractTest(unittest.TestCase):
    def test_map_base_motion_is_not_a_localization_fault(self) -> None:
        candidate = section(
            "bool candidate_is_reasonable", "bool candidate_is_within_anchor_locked"
        )
        decision = candidate
        self.assertNotIn(
            "metrics->translation > max_map_base_step_translation_", decision
        )
        self.assertNotIn("metrics->yaw_delta > max_map_base_step_yaw_", decision)
        self.assertIn(
            "metrics->correction_translation > max_odom_correction_", decision
        )
        self.assertIn(
            "metrics->correction_yaw > max_odom_yaw_correction_", decision
        )

    def test_rolling_anchor_is_post_commit_and_has_two_budgets(self) -> None:
        rolling = section(
            "bool maybe_advance_rolling_anchor_locked",
            "bool auto_reanchor_preconditions_locked",
        )
        callback = section("void on_amcl_pose", "void enter_hold")
        normal_branch = callback.split("} else if (!anomaly_active_) {", 1)[1].split(
            "} else if (!in_bounds_from_trusted)", 1
        )[0]
        self.assertLess(
            normal_branch.index("accepted = accept_candidate"),
            normal_branch.index("maybe_advance_rolling_anchor_locked"),
        )
        self.assertIn("if (accepted && trusted_quality_ok)", normal_branch)
        self.assertIn(
            'clear_auto_reanchor_integrity_inhibit_locked("scan_qualified_active_commit")',
            normal_branch,
        )
        self.assertIn("prune_rolling_anchor_window_locked(now)", rolling)
        self.assertIn("rolling_anchor_adjustments_.front().receipt", SOURCE)
        self.assertIn("rolling_anchor_window_translation_budget_", rolling)
        self.assertIn("rolling_anchor_session_translation_budget_", rolling)
        self.assertIn("rolling_anchor_session_translation_used_", rolling)

        # Bucket-boundary bursts cannot exceed the true sliding window, and
        # samples spaced farther apart than the window still cannot ratchet the
        # anchor forever because the session fuse never resets on pruning.
        self.assertAlmostEqual(bounded_anchor_moves([29.9] * 4 + [30.1] * 4), 0.12)
        low_frequency_total = bounded_anchor_moves(
            [31.0 * index for index in range(100)]
        )
        self.assertGreater(low_frequency_total, 0.30)
        self.assertAlmostEqual(low_frequency_total, 2.0)

    def test_sparse_candidate_catchup_is_quality_gated_and_transactional(self) -> None:
        callback = section("void on_amcl_pose", "void enter_hold")
        rolling = section(
            "bool maybe_advance_rolling_anchor_locked",
            "bool auto_reanchor_preconditions_locked",
        )

        # Reproduce the incident shape: after roughly three missed nominal
        # intervals a 0.336 m raw drift can use a 0.093 m bounded catch-up and
        # finishes inside the unchanged 0.30 m envelope.
        amount = rolling_catchup_step(
            candidate_gap_sec=3.1,
            desired_step=0.336 - 0.12,
        )
        self.assertAlmostEqual(amount, 0.093)
        self.assertLessEqual(0.336 - amount, 0.30)

        # A gap outside the freshness bound receives no catch-up. A depleted
        # window also cannot partially ratchet the anchor: the production helper
        # performs the residual-envelope check before assigning the anchor.
        self.assertEqual(
            rolling_catchup_step(candidate_gap_sec=4.001, desired_step=0.20), 0.0
        )
        insufficient = rolling_catchup_step(
            candidate_gap_sec=3.1,
            desired_step=0.216,
            remaining_window_budget=0.02,
        )
        self.assertGreater(0.336 - insufficient, 0.30)
        self.assertLess(
            rolling.index("const tf2::Transform proposed_anchor"),
            rolling.index("authorized_anchor_map_odom_ = proposed_anchor"),
        )
        self.assertIn("require_candidate_inside_limits", rolling)
        self.assertIn("catchup_step_insufficient_for_hard_envelope", rolling)

        # The exceptional pre-commit path needs independent local continuity,
        # a bounded source gap, scan/map quality, explicit nearest-scan motion
        # compensation, and a score that is not materially worse than frozen TF.
        catchup = callback.split(
            "if (!trusted_override_handled && !post_reanchor_verification_active_", 1
        )[1].split("if (trusted_override_handled)", 1)[0]
        for token in (
            "continuous_from_trusted",
            "trusted_quality_ok",
            "scan_temporally_valid",
            "trusted_quality.scan_motion_compensated",
            "candidate_score_not_worse",
            "rolling_anchor_catchup_max_gap_sec_",
            "rolling_anchor_catchup_max_step_scale_",
            "candidate_gap_sec / nominal_gap_sec",
            "catchup_step_scale, true",
        ):
            self.assertIn(token, catchup)
        self.assertIn(
            "bool in_bounds_from_trusted = in_bounds_from_anchor && "
            "continuous_from_trusted",
            callback,
        )
        self.assertIn("metrics->dt = (stamp - reference_stamp).seconds()", SOURCE)

    def test_active_quality_uses_bounded_nearest_scan_with_odom_compensation(self) -> None:
        selector = section(
            "sensor_msgs::msg::LaserScan::SharedPtr select_quality_scan",
            "bool evaluate_bootstrap_quality",
        )
        evaluator = section(
            "bool evaluate_bootstrap_quality", "bool is_spatial_fault_reason"
        )
        callback = section("void on_amcl_pose", "void enter_hold")

        self.assertEqual(
            select_quality_scan_stamp(10.0, [9.85, 10.0, 10.08], 0.20, True),
            10.0,
        )
        self.assertEqual(
            select_quality_scan_stamp(10.0, [9.84, 10.07], 0.20, True), 10.07
        )
        self.assertIsNone(
            select_quality_scan_stamp(10.0, [9.79, 10.22], 0.20, True)
        )
        self.assertIsNone(
            select_quality_scan_stamp(10.0, [9.99, 10.01], 0.20, False)
        )

        self.assertLess(
            selector.index("scan_stamp == pose_stamp"),
            selector.index("nearest_delta_sec <= active_quality_scan_tolerance_sec_"),
        )
        self.assertIn(
            '"active_quality_no_scan_within_tolerance"', selector
        )
        self.assertIn("odom_frame_, base_frame_, scan_stamp", evaluator)
        self.assertIn("scoring_map_base = candidate_map_odom *", evaluator)
        self.assertIn(
            "base_frame_, scan->header.frame_id, scan_stamp", evaluator
        )
        self.assertIn("metrics->scan_motion_compensated = true", evaluator)
        self.assertGreaterEqual(callback.count("trusted_scan"), 3)
        self.assertIn(
            "trusted_reference_map_odom, trusted_reference_map_base, trusted_scan",
            callback,
        )
        self.assertIn("'active_quality_scan_tolerance_sec': 0.20", LAUNCH_SOURCE)

    def test_active_covariance_policy_is_structural_but_reanchor_remains_strict(self) -> None:
        evaluator = section(
            "bool evaluate_bootstrap_quality", "bool is_spatial_fault_reason"
        )
        callback = section("void on_amcl_pose", "void enter_hold")

        # A valid but not yet bootstrap-converged covariance must reach ACTIVE
        # beam scoring, while bootstrap and discontinuous re-anchor remain strict.
        self.assertTrue(covariance_policy_accepts(
            0.09, 0.0, 0.0, 0.08, 0.04, strict=False
        ))
        self.assertFalse(covariance_policy_accepts(
            0.09, 0.0, 0.0, 0.08, 0.04, strict=True
        ))

        # Structural invalidity is rejected under both policies.
        invalid_cases = (
            (math.nan, 0.0, 0.0, 0.01, 0.01),
            (-0.01, 0.0, 0.0, 0.01, 0.01),
            (0.01, 0.02, 0.02, 0.01, 0.01),  # non-PSD
            (0.01, 0.01, 0.0, 0.01, 0.01),  # asymmetric
            (0.0, 0.0, 0.0, 0.0, 0.0),  # uninitialized/zero
        )
        for values in invalid_cases:
            self.assertFalse(covariance_policy_accepts(*values, strict=False))
            self.assertFalse(covariance_policy_accepts(*values, strict=True))

        self.assertIn("enum class QualityCovariancePolicy", SOURCE)
        self.assertIn("QualityCovariancePolicy::kStrictConverged", evaluator)
        self.assertIn("QualityCovariancePolicy::kStructuralOnly", evaluator)
        self.assertIn("covariance_structurally_valid", evaluator)
        self.assertIn("metrics->covariance_converged", evaluator)
        self.assertLess(
            evaluator.index("covariance_policy == QualityCovariancePolicy::kStrictConverged"),
            evaluator.index("const std::size_t range_count"),
        )

        bootstrap_call = callback[callback.index("bootstrap_quality_ok ="):
                                  callback.index("ScanMatchMetrics trusted_quality")]
        self.assertIn("QualityCovariancePolicy::kStrictConverged", bootstrap_call)
        active_call = callback[callback.index("trusted_quality_ok ="):
                               callback.index("bool accepted = false")]
        self.assertIn("QualityCovariancePolicy::kStructuralOnly", active_call)
        self.assertIn("trusted_quality.covariance_converged", active_call)

        normal_branch = callback.split("} else if (!anomaly_active_) {", 1)[1].split(
            "} else if (!in_bounds_from_trusted)", 1
        )[0]
        self.assertIn("accepted && trusted_quality_ok", normal_branch)
        self.assertNotIn("trusted_reanchor_quality_ok", normal_branch)
        self.assertGreaterEqual(callback.count("!trusted_reanchor_quality_ok"), 2)

    def test_rolling_skip_diagnostics_and_far_quality_advantage_remain_strict(self) -> None:
        rolling = section(
            "bool maybe_advance_rolling_anchor_locked",
            "bool auto_reanchor_preconditions_locked",
        )
        callback = section("void on_amcl_pose", "void enter_hold")
        for reason in (
            'skip("inside_deadband")',
            'skip("sliding_window_budget_exhausted")',
            'skip("session_budget_exhausted")',
        ):
            self.assertIn(reason, rolling)
        self.assertIn("Rolling anchor not advanced (%s)", callback)
        self.assertIn("selected scan delta %.3f s", callback)
        self.assertIn("odom motion compensation=%s", callback)

        # The corridor far-reanchor safeguard remains unchanged: a candidate
        # must beat the frozen trusted pose by at least 0.08 likelihood.
        self.assertIn(
            "trusted_quality.likelihood_mean - trusted_reference_quality.likelihood_mean <",
            callback,
        )
        self.assertIn("auto_reanchor_min_likelihood_improvement_", callback)
        self.assertIn("'auto_reanchor_min_likelihood_improvement': 0.08", LAUNCH_SOURCE)

    def test_far_reanchor_requires_independent_fresh_safety_evidence(self) -> None:
        preconditions = section(
            "bool auto_reanchor_preconditions_locked",
            "bool advance_recovery_cluster_locked",
        )
        callback = section("void on_amcl_pose", "void enter_hold")
        for token in (
            "interlock_is_freshly_blocked_locked(now)",
            "recovery_is_stationary_locked(now)",
            "auto_reanchor_integrity_inhibit_",
            "last_new_odom_observation_steady_",
            "auto_reanchor_max_odom_receipt_age_sec_",
            "auto_reanchors_used_ >= max_auto_reanchors_per_session_",
            "auto_reanchor_absolute_limit",
        ):
            self.assertIn(token, preconditions)
        for token in (
            "trusted_reanchor_quality_ok",
            "auto_reanchor_min_likelihood_improvement_",
            "recovery_cluster_min_duration_sec_",
            "post_reanchor_verification_active_ = true",
            "anomaly_active_ = true",
        ):
            self.assertIn(token, callback)

        first_far = callback.split(
            "} else if (!in_bounds_from_trusted && !anomaly_active_) {", 1
        )[1].split("} else if (!anomaly_active_) {", 1)[0]
        self.assertIn("anomaly_active_ = true", first_far)
        self.assertNotIn("advance_recovery_cluster_locked", first_far)

        far = callback.rsplit("} else if (!in_bounds_from_trusted) {", 1)[1].split(
            "} else {", 1
        )[0]
        ordered_tokens = (
            "if (!trusted_reanchor_quality_ok)",
            "auto_reanchor_min_likelihood_improvement_",
            "auto_reanchor_preconditions_locked",
            "advance_recovery_cluster_locked",
            "post_reanchor_verification_active_ = true",
            "anomaly_active_ = true",
        )
        positions = [far.index(token) for token in ordered_tokens]
        self.assertEqual(positions, sorted(positions))
        self.assertNotIn("auto_reanchor_integrity_inhibit_ = false", far)

        blocked_callback = section("void on_interlock_blocked", "void on_map")
        self.assertIn("ended_blocked_epoch", blocked_callback)
        self.assertIn("reset_pending_locked()", blocked_callback)

    def test_post_reanchor_release_needs_a_second_quality_duration_window(self) -> None:
        callback = section("void on_amcl_pose", "void enter_hold")
        post = callback.split("if (post_reanchor_verification_active_) {", 1)[1].split(
            "} else if (!in_bounds_from_trusted && !anomaly_active_)", 1
        )[0]
        for token in (
            "if (!trusted_reanchor_quality_ok)",
            "interlock_is_freshly_blocked_locked(steady_now)",
            "recovery_is_stationary_locked(steady_now)",
            "post_reanchor_consecutive_valid_samples_",
            "post_reanchor_min_confirm_duration_sec_",
            "post_reanchor_verification_active_ = false",
            "anomaly_active_ = false",
        ):
            self.assertIn(token, post)

    def test_far_cluster_is_fixed_seed_duration_and_strict_odom(self) -> None:
        cluster = section(
            "bool advance_recovery_cluster_locked", "void store_pending_locked"
        )
        self.assertIn("recovery_cluster_map_odom_.inverse() * candidate_map_odom", cluster)
        self.assertIn("recovery_cluster_odom_translation_tolerance_", cluster)
        self.assertIn("recovery_cluster_odom_yaw_tolerance_", cluster)
        self.assertIn("now - recovery_cluster_started_", cluster)
        self.assertIn("cluster_duration_sec >= required_duration_sec", cluster)
        immutable_comment = "Keep the first sample as an immutable seed"
        self.assertIn(immutable_comment, cluster)

    def test_launch_enables_bounded_policy_not_a_larger_fixed_limit(self) -> None:
        for token in (
            "'max_cumulative_map_odom_translation': 0.30",
            "'max_cumulative_map_odom_yaw': 0.20",
            "'rolling_anchor_window_translation_budget': 0.12",
            "'rolling_anchor_session_translation_budget': 2.0",
            "'rolling_anchor_catchup_max_gap_sec': 4.0",
            "'rolling_anchor_catchup_max_step_scale': 4.0",
            "'rolling_anchor_catchup_max_likelihood_drop': 0.03",
            "'interlock_blocked_max_age_sec': 0.25",
            "'recovery_cluster_min_duration_sec': 3.0",
            "'max_auto_reanchors_per_session': 1",
            "'post_reanchor_min_confirm_duration_sec': 2.0",
            "'recovery_nomotion_max_requests': 12",
            "'recovery_nomotion_max_window_sec': 8.0",
        ):
            self.assertIn(token, LAUNCH_SOURCE)

    def test_recovery_nomotion_is_bounded_and_disabled_by_integrity_inhibit(self) -> None:
        request = section(
            "void request_nomotion_update_if_needed", "void on_publish_timer"
        )
        reset = section(
            "void reset_pending_locked", "void reset_recovery_nomotion_budget_locked"
        )
        self.assertIn("!auto_reanchor_integrity_inhibit_", request)
        self.assertIn(
            "recovery_nomotion_requests_sent_ >= recovery_nomotion_max_requests_",
            request,
        )
        self.assertIn(
            "recovery_nomotion_elapsed_sec >= recovery_nomotion_max_window_sec_",
            request,
        )
        self.assertIn("recovery_cluster_reported_ = true", request)
        self.assertIn("++recovery_nomotion_requests_sent_", request)
        self.assertNotIn("recovery_cluster_reported_ = false", reset)
        timer_times = [0.5 * index for index in range(100)]
        self.assertEqual(
            bounded_recovery_nomotion_requests(
                timer_times, integrity_inhibit=True
            ),
            0,
        )
        self.assertEqual(
            bounded_recovery_nomotion_requests(
                timer_times, integrity_inhibit=False
            ),
            12,
        )

    def test_integrity_inhibit_preserves_first_reason_and_reports_absolute_limit(self) -> None:
        setter = section(
            "void set_auto_reanchor_integrity_inhibit_locked",
            "void clear_auto_reanchor_integrity_inhibit_locked",
        )
        preconditions = section(
            "bool auto_reanchor_preconditions_locked",
            "bool advance_recovery_cluster_locked",
        )
        self.assertIn("if (auto_reanchor_integrity_inhibit_)", setter)
        self.assertIn("auto_reanchor_integrity_inhibit_reason_ = reason", setter)
        self.assertIn("auto_reanchor_integrity_inhibit_source_stamp_ = source_stamp", setter)
        self.assertIn("first_reason=%s", setter)
        self.assertIn(
            '"auto_reanchor_inhibited_after_" + auto_reanchor_integrity_inhibit_reason_',
            preconditions,
        )
        self.assertIn('reason += "_and_absolute_limit"', preconditions)


class MotionLinkedRollingBudgetContractTest(unittest.TestCase):
    def test_dynamic_budget_is_motion_linked_and_capped(self) -> None:
        self.assertAlmostEqual(dynamic_window_budget(0.12, 0.0, 0.15, 0.18), 0.12)
        self.assertAlmostEqual(dynamic_window_budget(0.12, 1.0, 0.15, 0.18), 0.27)
        self.assertAlmostEqual(dynamic_window_budget(0.12, 10.0, 0.15, 0.18), 0.30)

        rolling = section(
            "bool maybe_advance_rolling_anchor_locked",
            "bool auto_reanchor_preconditions_locked",
        )
        monitor = section("bool monitor_odom_integrity", "bool accept_candidate")
        for token in (
            "rolling_anchor_translation_budget_per_odom_meter_ * odom_window_translation",
            "rolling_anchor_yaw_budget_per_odom_rad_ * odom_window_yaw",
            "rolling_anchor_window_translation_motion_cap_",
            "rolling_anchor_window_yaw_motion_cap_",
            "directional_vector_step_limit",
            "window_translation_net_x",
            "window_yaw_net",
        ):
            self.assertIn(token, rolling)
        self.assertIn("OdomMotionSegment motion", monitor)
        self.assertIn("!timing_fault && !timing_soft_violation", monitor)
        self.assertIn("!jump_detected", monitor)

    def test_direction_reversal_reclaims_window_but_not_session_fuse(self) -> None:
        budget = 0.12
        net = 0.0
        gross = 0.0
        for _ in range(4):
            step = directional_step_limit(net, 0.0, 1.0, 0.0, budget, 0.03)
            net += step
            gross += abs(step)
        self.assertAlmostEqual(net, 0.12)
        self.assertEqual(directional_step_limit(net, 0.0, 1.0, 0.0, budget, 0.03), 0.0)

        reverse = directional_step_limit(net, 0.0, -1.0, 0.0, budget, 0.03)
        net -= reverse
        gross += abs(reverse)
        self.assertAlmostEqual(reverse, 0.03)
        self.assertAlmostEqual(net, 0.09)
        self.assertAlmostEqual(gross, 0.15)
        self.assertIn("rolling_anchor_session_translation_used_ += translation_step", SOURCE)

    def test_launch_keeps_hard_envelope_and_exposes_effective_values(self) -> None:
        for token in (
            "'max_cumulative_map_odom_translation': 0.30",
            "'max_cumulative_map_odom_yaw': 0.20",
            "'rolling_anchor_translation_budget_per_odom_meter': 0.15",
            "'rolling_anchor_yaw_budget_per_odom_rad': 0.15",
            "'rolling_anchor_window_translation_motion_cap': 0.18",
            "'rolling_anchor_window_yaw_motion_cap': 0.10",
        ):
            self.assertIn(token, LAUNCH_SOURCE)


class CumulativeOverrunDeferContractTest(unittest.TestCase):
    def test_one_good_overrun_is_pending_without_hold(self) -> None:
        self.assertEqual(
            cumulative_overrun_outcome([(0.0, 100, True, False)]),
            ("pending", 1),
        )

    def test_duplicate_stamp_does_not_confirm(self) -> None:
        self.assertEqual(
            cumulative_overrun_outcome(
                [(0.0, 100, True, False), (0.3, 100, True, False), (0.6, 101, True, False)]
            ),
            ("pending", 2),
        )

    def test_three_new_stamps_over_minimum_duration_confirm(self) -> None:
        self.assertEqual(
            cumulative_overrun_outcome(
                [(0.0, 100, True, False), (0.25, 101, True, False), (0.51, 102, True, False)]
            ),
            ("hold_confirmed", 3),
        )

    def test_bad_evidence_is_immediate_and_return_inside_clears(self) -> None:
        self.assertEqual(
            cumulative_overrun_outcome([(0.0, 100, False, False)]),
            ("hold_immediate", 0),
        )
        self.assertEqual(
            cumulative_overrun_outcome(
                [(0.0, 100, True, False), (0.2, 101, True, True)]
            ),
            ("cleared", 0),
        )

    def test_steady_clock_deadline_confirms_even_without_enough_stamps(self) -> None:
        self.assertEqual(
            cumulative_overrun_outcome(
                [(0.0, 100, True, False), (1.51, 100, True, False)]
            ),
            ("hold_deadline", 1),
        )

    def test_source_wiring_freezes_without_new_anomaly_then_cleans_state(self) -> None:
        callback = section("void on_amcl_pose", "void enter_hold")
        pending = callback.rsplit("if (cumulative_overrun_deferred) {", 1)[1].split(
            "publish_anomaly(true);", 1
        )[0]
        eligibility = section(
            "bool cumulative_overrun_defer_eligible_locked",
            "bool advance_cumulative_overrun_defer_locked",
        )
        reset = section("void reset_cumulative_overrun_defer_locked", "void reset_recovery_nomotion_budget_locked")
        timer = section("void on_publish_timer", "bool monitor_publish_timer_health")

        self.assertIn("publish_current_anomaly();", pending)
        self.assertNotIn("accept_candidate", pending)
        self.assertIn("Candidate and rolling anchor are frozen", pending)
        for token in (
            "cumulative_overrun_defer_max_correction_translation_",
            "cumulative_overrun_defer_max_correction_yaw_",
            "cumulative_overrun_defer_max_source_gap_sec_",
            "cumulative_overrun_defer_max_odom_receipt_age_sec_",
            "cumulative_overrun_defer_pose_odom_stamp_tolerance_sec_",
            "cumulative_overrun_session_fuse_exhausted",
            "cumulative_overrun_scan_quality_ok_locked",
        ):
            self.assertIn(token, eligibility)
        self.assertIn("reset_cumulative_overrun_defer_locked();", reset)
        self.assertIn("monitor_cumulative_overrun_defer(steady_now);", timer)
        self.assertIn("cumulative_overrun_defer_last_stamp_", SOURCE)
        self.assertLess(
            callback.index("mark_new_amcl_stamp(stamp)"),
            callback.index("advance_cumulative_overrun_defer_locked"),
        )

        catchup_condition = callback.split(
            "if (!trusted_override_handled && !post_reanchor_verification_active_", 1
        )[1].split("{", 1)[0]
        self.assertIn("!cumulative_overrun_defer_active_", catchup_condition)

        defer_branch = callback.split(
            "const bool cumulative_only_overrun", 1
        )[1].split("if (defer_eligible)", 1)[0]
        self.assertIn("else if (!trusted_quality_ok)", defer_branch)
        self.assertIn("else if (!candidate_score_not_worse)", defer_branch)
        self.assertIn("cumulative_overrun_defer_eligible_locked", defer_branch)

        scan_gate = section(
            "bool cumulative_overrun_scan_quality_ok_locked",
            "bool cumulative_overrun_defer_eligible_locked",
        )
        self.assertIn(
            "quality.best_neighbor_likelihood_gain > score_max_neighbor_likelihood_gain_",
            scan_gate,
        )

        for token in (
            "'cumulative_overrun_defer_consecutive_samples': 3",
            "'cumulative_overrun_defer_min_duration_sec': 0.50",
            "'cumulative_overrun_defer_max_duration_sec': 1.50",
            "'cumulative_overrun_defer_max_translation_excess': 0.30",
            "'cumulative_overrun_defer_max_yaw_excess': 0.15",
            "'cumulative_overrun_defer_max_correction_translation': 0.10",
            "'cumulative_overrun_defer_max_correction_yaw': 0.08",
        ):
            self.assertIn(token, LAUNCH_SOURCE)


class TrustedOverrideContractTest(unittest.TestCase):
    def test_active_override_needs_fresh_scan_stamps_and_quarter_second(self) -> None:
        self.assertEqual(
            trusted_override_outcome(
                [
                    (0.00, 100, "good", False, False),
                    (0.10, 100, "good", False, False),
                    (0.26, 101, "good", False, False),
                ]
            ),
            ("override", 2),
        )

    def test_far_raw_amcl_does_not_lock_when_trusted_scan_is_good(self) -> None:
        self.assertEqual(
            trusted_override_outcome(
                [
                    (0.00, 100, "good", False, False),
                    (0.26, 101, "good", False, False),
                    (4.00, 140, "good", False, False),
                ]
            )[0],
            "override",
        )

    def test_hold_release_is_longer_and_requires_stationary(self) -> None:
        moving = trusted_override_outcome(
            [
                (0.00, 100, "good", False, False),
                (0.30, 101, "good", False, False),
                (0.60, 102, "good", False, False),
            ],
            already_hold=True,
        )
        self.assertEqual(moving, ("soft_hold", 0))
        self.assertEqual(
            trusted_override_outcome(
                [
                    (0.00, 100, "good", False, True),
                    (0.25, 101, "good", False, True),
                    (0.51, 102, "good", False, True),
                ],
                already_hold=True,
            ),
            ("override", 3),
        )

    def test_only_distinct_persistent_soft_failures_or_catastrophe_hard_hold(self) -> None:
        self.assertEqual(
            trusted_override_outcome(
                [(0.0, 100, "soft", False, False), (0.2, 100, "soft", False, False)]
            ),
            ("verify", 0),
        )
        self.assertEqual(
            trusted_override_outcome(
                [(0.0, 100, "soft", False, False), (0.2, 101, "soft", False, False)]
            ),
            ("hard_hold", 2),
        )
        self.assertEqual(
            trusted_override_outcome([(0.0, 100, "catastrophic", False, False)]),
            ("hard_hold", 0),
        )

    def test_active_cached_scan_cannot_extend_evidence_deadline(self) -> None:
        self.assertEqual(
            trusted_override_active_evidence_outcome(
                [
                    (0.00, 100, True),
                    (0.40, 100, True),
                    (0.80, 100, True),
                    (1.01, 100, True),
                ]
            ),
            "hard_hold",
        )
        self.assertEqual(
            trusted_override_active_evidence_outcome(
                [
                    (0.00, 100, True),
                    (0.80, 101, True),
                    (1.60, 102, True),
                    (2.40, 103, True),
                ]
            ),
            "active",
        )

    def test_source_authorizes_only_absolute_trusted_quality(self) -> None:
        callback = section("void on_amcl_pose", "void enter_hold")
        quality = section(
            "TrustedOverrideQuality trusted_override_scan_quality_locked",
            "bool advance_trusted_override_confirmation_locked",
        )
        trusted_score = callback.split(
            "trusted_reference_map_odom = last_trusted_map_odom_", 1
        )[1].split("bool accepted = false", 1)[0]
        override = callback.split(
            "if (trusted_override_enabled_ && trusted_override_active_)", 1
        )[1].split(
            "else if (trusted_override_enabled_ && trusted_override_soft_hold_)", 1
        )[0]

        self.assertIn("QualityCovariancePolicy::kIgnored", trusted_score)
        self.assertIn("trusted_reference_map_base = trusted_reference_map_odom * current_odom_base", trusted_score)
        for forbidden in (
            "covariance_converged",
            "best_neighbor_likelihood_gain",
            "candidate_score_not_worse",
        ):
            self.assertNotIn(forbidden, quality)
        self.assertIn("raw_candidate_anomalous", override)
        self.assertIn("trusted_override_retained = true", override)
        self.assertNotIn("cumulative_translation >", override)
        self.assertNotIn("correction_translation >", override)
        self.assertIn(
            'return TrustedOverrideQuality::kSoftFailure;',
            quality.split("trusted_scan_or_same_stamp_tf_not_ready", 1)[1],
        )
        evaluator = section(
            "bool evaluate_bootstrap_quality",
            "bool is_spatial_fault_reason",
        )
        self.assertLess(
            evaluator.index("covariance_policy == QualityCovariancePolicy::kIgnored"),
            evaluator.index("std::vector<tf2::Transform> neighboring_poses"),
        )

    def test_override_precedes_legacy_catchup_and_has_no_fixed_mode_lifetime(self) -> None:
        callback = section("void on_amcl_pose", "void enter_hold")
        monitor = section(
            "void monitor_trusted_override_verification",
            "void set_auto_reanchor_integrity_inhibit_locked",
        )
        self.assertLess(
            callback.index("if (trusted_override_enabled_ && trusted_override_active_)"),
            callback.index("maybe_advance_rolling_anchor_locked"),
        )
        self.assertIn("if (trusted_override_handled)", callback)
        self.assertIn("!trusted_override_active_", monitor)
        self.assertNotIn("trusted_override_active_ = false", monitor)

        evidence_monitor = section(
            "void monitor_trusted_override_active_evidence",
            "void set_auto_reanchor_integrity_inhibit_locked",
        )
        self.assertIn("trusted_override_active_", evidence_monitor)
        self.assertIn("trusted_override_last_good_scan_evidence_steady_", evidence_monitor)
        self.assertIn("trusted_override_active_evidence_timeout_sec_", evidence_monitor)
        self.assertIn("trusted_override_hard_hold_ = true", evidence_monitor)
        self.assertIn(
            "scan_stamp > trusted_override_last_good_scan_stamp_",
            SOURCE,
        )

    def test_launch_exposes_relaxed_quality_and_failure_policy(self) -> None:
        for token in (
            "'trusted_override_active_consecutive_samples': 2",
            "'trusted_override_active_min_duration_sec': 0.25",
            "'trusted_override_active_evidence_timeout_sec': 1.00",
            "'trusted_override_hold_consecutive_samples': 3",
            "'trusted_override_hold_min_duration_sec': 0.50",
            "'trusted_override_hold_stationary_duration_sec': 0.40",
            "'trusted_override_min_valid_beams': 25",
            "'trusted_override_min_inlier_ratio': 0.60",
            "'trusted_override_min_likelihood': 0.55",
            "'trusted_override_max_trimmed_mean_distance': 0.20",
            "'trusted_override_max_unknown_offmap_ratio': 0.20",
            "'trusted_override_soft_failure_consecutive_samples': 2",
        ):
            self.assertIn(token, LAUNCH_SOURCE)

    def test_hard_integrity_paths_clear_override_state(self) -> None:
        inhibit = section(
            "void set_auto_reanchor_integrity_inhibit_locked",
            "void clear_auto_reanchor_integrity_inhibit_locked",
        )
        hold = section("void enter_hold", "void enter_amcl_timing_hold")
        self.assertIn("clear_trusted_override_locked();", inhibit)
        self.assertIn("clear_trusted_override_locked();", hold)
        self.assertIn("trusted_override_hard_hold_ = true", SOURCE)

    def test_mode_topic_is_latched_reliable_and_heartbeated(self) -> None:
        constructor = section("AmclTfGuard()", "private:")
        timer = section("void on_publish_timer", "bool monitor_publish_timer_health")
        publisher = section("void publish_guard_mode", "std::string global_frame_")
        self.assertIn('"mode_topic", "/amcl_tf_guard/mode"', SOURCE)
        self.assertIn("mode_qos.reliable().transient_local()", constructor)
        self.assertIn("publish_guard_mode();", timer)
        for mode in ("ACTIVE", "VERIFY_TRUSTED", "TRUSTED_OVERRIDE", "HARD_HOLD"):
            self.assertIn(f'message.data = "{mode}"', publisher)
        self.assertIn("'mode_topic': '/amcl_tf_guard/mode'", LAUNCH_SOURCE)


if __name__ == "__main__":
    unittest.main()
