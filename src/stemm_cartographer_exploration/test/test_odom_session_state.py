import math

try:
    from stemm_cartographer_exploration.odom_session_state import (
        OdomStabilityWindow,
        Pose2D,
    )
except ModuleNotFoundError:  # Local patch-staging execution.
    from odom_session_state import OdomStabilityWindow, Pose2D


def make_window():
    return OdomStabilityWindow(
        stable_sec=0.8,
        position_drift_m=0.025,
        yaw_drift_rad=math.radians(3.0),
        linear_speed_mps=0.02,
        angular_speed_rps=0.05,
        position_step_m=0.10,
        yaw_step_rad=math.radians(10.0),
    )


def test_nonzero_stationary_origin_is_accepted():
    window = make_window()
    pose = Pose2D(3.536, 2.709, math.radians(10.9))

    assert not window.observe(
        now=10.0, pose=pose, linear_speed=0.0, angular_speed=0.0).ready
    result = window.observe(
        now=10.81,
        pose=Pose2D(3.540, 2.705, math.radians(11.1)),
        linear_speed=0.0,
        angular_speed=0.0,
    )

    assert result.ready
    assert math.isclose(result.stable_for_sec, 0.81)


def test_motion_restarts_stability_window():
    window = make_window()
    pose = Pose2D(4.0, -2.0, 0.1)
    window.observe(
        now=0.0, pose=pose, linear_speed=0.0, angular_speed=0.0)
    moving = window.observe(
        now=0.7, pose=pose, linear_speed=0.03, angular_speed=0.0)
    early = window.observe(
        now=1.4, pose=pose, linear_speed=0.0, angular_speed=0.0)
    still_early = window.observe(
        now=1.51, pose=pose, linear_speed=0.0, angular_speed=0.0)
    ready = window.observe(
        now=2.21, pose=pose, linear_speed=0.0, angular_speed=0.0)

    assert not moving.ready
    assert 'still moving' in moving.reason
    assert not early.ready
    assert not still_early.ready
    assert ready.ready


def test_pose_drift_and_jump_restart_stability_window():
    window = make_window()
    window.observe(
        now=0.0, pose=Pose2D(1.0, 1.0, 0.0),
        linear_speed=0.0, angular_speed=0.0)
    drift = window.observe(
        now=0.7, pose=Pose2D(1.03, 1.0, 0.0),
        linear_speed=0.0, angular_speed=0.0)
    jump = window.observe(
        now=0.8, pose=Pose2D(1.20, 1.0, 0.0),
        linear_speed=0.0, angular_speed=0.0)

    assert not drift.ready
    assert 'not stable' in drift.reason
    assert not jump.ready
    assert 'discontinuity' in jump.reason
    assert jump.fatal


def test_nonfinite_sample_is_rejected_and_resets_window():
    window = make_window()
    window.observe(
        now=1.0, pose=Pose2D(0.0, 0.0, 0.0),
        linear_speed=0.0, angular_speed=0.0)
    invalid = window.observe(
        now=1.5, pose=Pose2D(float('nan'), 0.0, 0.0),
        linear_speed=0.0, angular_speed=0.0)
    after_reset = window.observe(
        now=2.0, pose=Pose2D(0.0, 0.0, 0.0),
        linear_speed=0.0, angular_speed=0.0)

    assert not invalid.ready
    assert 'non-finite' in invalid.reason
    assert invalid.fatal
    assert not after_reset.ready


def test_stability_reset_preserves_cross_gap_discontinuity_history():
    window = make_window()
    window.observe(
        now=1.0, pose=Pose2D(2.0, 3.0, 0.1),
        linear_speed=0.0, angular_speed=0.0)
    window.reset_stability()

    resumed = window.observe(
        now=2.0, pose=Pose2D(2.2, 3.0, 0.1),
        linear_speed=0.0, angular_speed=0.0)

    assert resumed.fatal
    assert 'discontinuity' in resumed.reason


if __name__ == '__main__':
    test_nonzero_stationary_origin_is_accepted()
    test_motion_restarts_stability_window()
    test_pose_drift_and_jump_restart_stability_window()
    test_nonfinite_sample_is_rejected_and_resets_window()
    test_stability_reset_preserves_cross_gap_discontinuity_history()
