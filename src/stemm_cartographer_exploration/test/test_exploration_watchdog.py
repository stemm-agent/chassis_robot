from stemm_cartographer_exploration.exploration_watchdog import (
    ExplorationProgressWatchdog,
)


def make_watchdog():
    return ExplorationProgressWatchdog(
        no_progress_timeout_sec=10.0,
        min_progress_m=0.12,
        hard_timeout_sec=50.0,
    )


def test_small_oscillations_do_not_reset_no_progress_window():
    watchdog = make_watchdog()
    watchdog.reset(0.0)

    assert watchdog.observe(0.0, 2.00) == 'tracking'
    assert watchdog.observe(4.0, 1.95) == 'tracking'
    assert watchdog.observe(9.0, 2.03) == 'tracking'
    assert watchdog.observe(10.0, 1.94) == 'stalled'


def test_meaningful_net_distance_reduction_resets_window():
    watchdog = make_watchdog()
    watchdog.reset(0.0)

    assert watchdog.observe(0.0, 2.00) == 'tracking'
    assert watchdog.observe(9.5, 1.87) == 'progress'
    assert watchdog.observe(19.4, 1.84) == 'tracking'
    assert watchdog.observe(19.5, 1.84) == 'stalled'


def test_nav2_recovery_gets_one_bounded_progress_window():
    watchdog = make_watchdog()
    watchdog.reset(0.0)
    watchdog.observe(0.0, 2.00)

    assert watchdog.observe(10.0, 2.01, recovery_count=1) == (
        'recovery_started')
    assert watchdog.observe(19.9, 1.99, recovery_count=1) == 'tracking'
    assert watchdog.observe(20.0, 1.99, recovery_count=1) == 'stalled'


def test_hard_limit_cannot_be_extended_by_small_progress():
    watchdog = make_watchdog()
    watchdog.reset(0.0)
    watchdog.observe(0.0, 3.0)

    for timestamp, distance in (
            (9.0, 2.87), (18.0, 2.74), (27.0, 2.61), (36.0, 2.48),
            (45.0, 2.35)):
        assert watchdog.observe(timestamp, distance) == 'progress'

    assert watchdog.observe(50.0, 2.30) == 'hard_timeout'
