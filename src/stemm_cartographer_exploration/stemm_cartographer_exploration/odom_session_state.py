"""Pure state logic for the Cartographer odometry readiness gate."""

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    """Return an angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StabilityObservation:
    ready: bool
    stable_for_sec: float
    reason: str
    sample_accepted: bool = False
    window_restarted: bool = False
    fatal: bool = False


class OdomStabilityWindow:
    """Require a bounded interval of finite, stationary, continuous odometry.

    Absolute pose is deliberately ignored.  Cartographer compensates the
    existing odom origin with map -> odom_combined; only local continuity and a
    stationary start are required.
    """

    def __init__(
        self,
        *,
        stable_sec: float,
        position_drift_m: float,
        yaw_drift_rad: float,
        linear_speed_mps: float,
        angular_speed_rps: float,
        position_step_m: float,
        yaw_step_rad: float,
    ) -> None:
        positive = {
            'stable_sec': stable_sec,
            'position_drift_m': position_drift_m,
            'yaw_drift_rad': yaw_drift_rad,
            'linear_speed_mps': linear_speed_mps,
            'angular_speed_rps': angular_speed_rps,
            'position_step_m': position_step_m,
            'yaw_step_rad': yaw_step_rad,
        }
        if any(not math.isfinite(value) or value <= 0.0
               for value in positive.values()):
            raise ValueError(f'all limits must be finite and positive: {positive}')

        self.stable_sec = stable_sec
        self.position_drift_m = position_drift_m
        self.yaw_drift_rad = yaw_drift_rad
        self.linear_speed_mps = linear_speed_mps
        self.angular_speed_rps = angular_speed_rps
        self.position_step_m = position_step_m
        self.yaw_step_rad = yaw_step_rad
        self._anchor_pose = None
        self._anchor_time = None
        self._previous_pose = None

    def reset(self) -> None:
        self._anchor_pose = None
        self._anchor_time = None
        self._previous_pose = None

    def reset_stability(self) -> None:
        """Restart readiness timing while preserving session continuity."""
        self._anchor_pose = None
        self._anchor_time = None

    @staticmethod
    def _pose_is_finite(pose: Pose2D) -> bool:
        return all(math.isfinite(value) for value in (
            pose.x, pose.y, pose.yaw))

    @staticmethod
    def _pose_delta(first: Pose2D, second: Pose2D) -> tuple[float, float]:
        position = math.hypot(second.x - first.x, second.y - first.y)
        yaw = abs(normalize_angle(second.yaw - first.yaw))
        return position, yaw

    def _start_window(self, now: float, pose: Pose2D) -> None:
        self._anchor_time = now
        self._anchor_pose = pose

    def observe(
        self,
        *,
        now: float,
        pose: Pose2D,
        linear_speed: float,
        angular_speed: float,
    ) -> StabilityObservation:
        values = (now, linear_speed, angular_speed)
        if not self._pose_is_finite(pose) or not all(
                math.isfinite(value) for value in values):
            self.reset()
            return StabilityObservation(
                False, 0.0, 'non-finite odometry', fatal=True)

        if self._previous_pose is not None:
            step_position, step_yaw = self._pose_delta(
                self._previous_pose, pose)
            if (step_position > self.position_step_m or
                    step_yaw > self.yaw_step_rad):
                self.reset()
                return StabilityObservation(
                    False,
                    0.0,
                    'odometry discontinuity: '
                    f'step={step_position:.3f}m/{math.degrees(step_yaw):.1f}deg',
                    fatal=True,
                )
        self._previous_pose = pose

        if (abs(linear_speed) > self.linear_speed_mps or
                abs(angular_speed) > self.angular_speed_rps):
            self.reset_stability()
            return StabilityObservation(
                False,
                0.0,
                'robot still moving: '
                f'v={linear_speed:.3f}m/s w={angular_speed:.3f}rad/s',
                window_restarted=True,
            )

        if self._anchor_pose is None or self._anchor_time is None:
            self._start_window(now, pose)
            return StabilityObservation(
                False,
                0.0,
                'starting stability window',
                sample_accepted=True,
                window_restarted=True,
            )

        position_drift, yaw_drift = self._pose_delta(self._anchor_pose, pose)
        if (position_drift > self.position_drift_m or
                yaw_drift > self.yaw_drift_rad):
            self._start_window(now, pose)
            return StabilityObservation(
                False,
                0.0,
                'pose not stable: '
                f'drift={position_drift:.3f}m/'
                f'{math.degrees(yaw_drift):.1f}deg',
                sample_accepted=True,
                window_restarted=True,
            )

        stable_for = max(0.0, now - self._anchor_time)
        return StabilityObservation(
            stable_for >= self.stable_sec,
            stable_for,
            'odometry is stationary' if stable_for >= self.stable_sec
            else 'waiting for stability window',
            sample_accepted=True,
        )
