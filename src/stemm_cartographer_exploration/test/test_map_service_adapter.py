from stemm_cartographer_exploration.map_service_adapter import (
    normalize_occupancy_values,
)


def test_rrt_map_normalizes_cartographer_probabilities():
    values = [-1, 0, 1, 29, 49, 50, 65, 99, 100]

    normalized = normalize_occupancy_values(values, 50)

    assert normalized == [-1, 0, 0, 0, 0, 100, 100, 100, 100]


def test_rrt_occupied_threshold_is_clamped():
    assert normalize_occupancy_values([-1, 0, 1], 0) == [-1, 0, 100]
    assert normalize_occupancy_values([99, 100], 101) == [0, 100]
