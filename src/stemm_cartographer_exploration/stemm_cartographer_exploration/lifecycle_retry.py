"""Small, ROS-independent state tracker for lifecycle request retries."""


class LifecycleRetryState:
    """Track one outstanding lifecycle request and bounded retry backoff."""

    def __init__(
            self,
            names,
            request_timeout_sec,
            retry_delay_sec,
            retry_max_delay_sec,
            warn_interval_sec=5.0):
        self.request_timeout_sec = max(0.1, float(request_timeout_sec))
        self.retry_delay_sec = max(0.0, float(retry_delay_sec))
        self.retry_max_delay_sec = max(
            self.retry_delay_sec, float(retry_max_delay_sec))
        self.warn_interval_sec = max(0.0, float(warn_interval_sec))
        self.started_at = {name: None for name in names}
        self.next_retry_at = {name: 0.0 for name in names}
        self.failure_count = {name: 0 for name in names}
        self.last_warning_at = {name: None for name in names}

    def mark_started(self, name, now):
        self.started_at[name] = now

    def request_expired(self, name, now):
        started_at = self.started_at[name]
        return (
            started_at is not None
            and now - started_at >= self.request_timeout_sec
        )

    def mark_success(self, name, now):
        self.started_at[name] = None
        self.next_retry_at[name] = now
        self.failure_count[name] = 0

    def mark_failure(self, name, now):
        self.started_at[name] = None
        self.failure_count[name] += 1
        exponent = min(self.failure_count[name] - 1, 12)
        delay = min(
            self.retry_max_delay_sec,
            self.retry_delay_sec * (2 ** exponent),
        )
        self.next_retry_at[name] = now + delay
        return delay

    def can_retry(self, name, now):
        return now >= self.next_retry_at[name]

    def should_warn(self, name, now):
        last_warning = self.last_warning_at[name]
        if (last_warning is not None
                and now - last_warning < self.warn_interval_sec):
            return False
        self.last_warning_at[name] = now
        return True
