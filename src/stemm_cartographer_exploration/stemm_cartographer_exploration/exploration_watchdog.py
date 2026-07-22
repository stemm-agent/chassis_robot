"""Pure state machine for exploration-goal progress supervision."""

from dataclasses import dataclass
import math


@dataclass
class ExplorationProgressWatchdog:
    """Detect lack of productive progress without treating oscillation as motion."""

    no_progress_timeout_sec: float
    min_progress_m: float
    hard_timeout_sec: float
    started_at: float = 0.0
    last_progress_at: float = 0.0
    best_distance: float | None = None
    recovery_count: int = 0
    active: bool = False

    def reset(self, now):
        self.started_at = float(now)
        self.last_progress_at = float(now)
        self.best_distance = None
        self.recovery_count = 0
        self.active = True

    def clear(self):
        self.started_at = 0.0
        self.last_progress_at = 0.0
        self.best_distance = None
        self.recovery_count = 0
        self.active = False

    @staticmethod
    def _valid_distance(distance):
        return distance is not None and math.isfinite(float(distance))

    def rebase_distance(self, now, distance, recovery_count=None):
        """Switch distance sources or start a recovery grace without resetting age."""
        if self._valid_distance(distance):
            self.best_distance = float(distance)
        self.last_progress_at = float(now)
        if recovery_count is not None:
            self.recovery_count = max(0, int(recovery_count))

    def observe(self, now, distance, recovery_count=0):
        """Return inactive, tracking, progress, recovery_started, stalled, or hard_timeout."""
        if not self.active:
            return 'inactive'

        now = float(now)
        if now - self.started_at >= self.hard_timeout_sec:
            return 'hard_timeout'

        recovery_count = max(0, int(recovery_count))
        if recovery_count > self.recovery_count:
            self.rebase_distance(now, distance, recovery_count)
            return 'recovery_started'

        if self._valid_distance(distance):
            distance = float(distance)
            if self.best_distance is None:
                self.best_distance = distance
                self.last_progress_at = now
                return 'tracking'
            if distance <= self.best_distance - self.min_progress_m:
                self.best_distance = distance
                self.last_progress_at = now
                return 'progress'

        if now - self.last_progress_at >= self.no_progress_timeout_sec:
            return 'stalled'
        return 'tracking'

    def elapsed(self, now):
        return max(0.0, float(now) - self.started_at) if self.active else 0.0

    def stagnant_for(self, now):
        return max(0.0, float(now) - self.last_progress_at) if self.active else 0.0
