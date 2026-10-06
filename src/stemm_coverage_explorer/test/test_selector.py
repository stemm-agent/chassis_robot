import time
import unittest

from stemm_coverage_explorer.selector import (
    GridMap,
    Pose2D,
    has_unknown_near,
    is_goal_safe,
    line_is_clear,
    select_coverage_goal,
)


def make_map(width=30, height=30, resolution=0.1):
    return GridMap(width, height, resolution, 0.0, 0.0, [-1] * (width * height))


def set_rect(grid, x0, y0, x1, y1, value):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            grid.data[y * grid.width + x] = value


def set_rect_free(grid, x0, y0, x1, y1):
    set_rect(grid, x0, y0, x1, y1, 0)


def set_rect_occupied(grid, x0, y0, x1, y1):
    set_rect(grid, x0, y0, x1, y1, 100)


class SelectorTest(unittest.TestCase):
    def test_selects_frontier_goal_inside_known_clear_space(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 18, 18)
        grid.data[12 * grid.width + 12] = 100

        goal = select_coverage_goal(grid, Pose2D(0.7, 0.7), min_distance=0.4, max_distance=1.4)

        self.assertIsNotNone(goal)
        mx = int(goal.x / grid.resolution)
        my = int(goal.y / grid.resolution)
        self.assertEqual(grid.data[my * grid.width + mx], 0)
        self.assertTrue(has_unknown_near(grid, mx, my, 0.5))
        self.assertTrue(is_goal_safe(grid, mx, my, obstacle_clearance=0.45, known_clearance=0.2))

    def test_rejects_goals_too_close_to_obstacles(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 18, 18)
        mx, my = 10, 10
        grid.data[my * grid.width + mx] = 100

        self.assertFalse(is_goal_safe(grid, 11, 10, obstacle_clearance=0.25, known_clearance=0.1))

    def test_avoids_recent_bad_places(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 18, 18)
        first = select_coverage_goal(grid, Pose2D(0.7, 0.7), min_distance=0.4, max_distance=1.4)
        self.assertIsNotNone(first)

        second = select_coverage_goal(
            grid,
            Pose2D(0.7, 0.7),
            min_distance=0.4,
            max_distance=1.4,
            bad_places=[(first.x, first.y)],
            bad_radius=10.0,
        )

        self.assertIsNone(second)

    def test_bad_place_can_use_local_radius(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 22, 22)
        first = select_coverage_goal(grid, Pose2D(0.7, 0.7), min_distance=0.4, max_distance=1.7)
        self.assertIsNotNone(first)

        second = select_coverage_goal(
            grid,
            Pose2D(0.7, 0.7),
            min_distance=0.4,
            max_distance=1.7,
            bad_places=[(first.x, first.y, 0.2)],
            bad_radius=10.0,
        )

        self.assertIsNotNone(second)

    def test_allows_unknown_cell_under_robot_when_nearby_free_space_is_reachable(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 18, 18)
        grid.data[7 * grid.width + 7] = -1

        goal = select_coverage_goal(grid, Pose2D(0.75, 0.75), min_distance=0.4, max_distance=1.4)

        self.assertIsNotNone(goal)

    def test_line_check_allows_unknown_but_rejects_occupied_obstacles(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 18, 18)
        for x in range(8, 13):
            grid.data[10 * grid.width + x] = -1

        self.assertTrue(line_is_clear(grid, Pose2D(0.5, 1.0), Pose2D(1.7, 1.0), 0.1))

        grid.data[10 * grid.width + 12] = 100
        self.assertFalse(line_is_clear(grid, Pose2D(0.5, 1.0), Pose2D(1.7, 1.0), 0.1))

    def test_uses_reachable_path_around_l_shaped_obstacle(self):
        grid = make_map(width=50, height=50, resolution=0.1)
        set_rect_free(grid, 5, 5, 12, 36)
        set_rect_free(grid, 5, 29, 40, 36)
        set_rect_occupied(grid, 14, 5, 24, 28)

        goal = select_coverage_goal(
            grid,
            Pose2D(0.75, 0.75),
            min_distance=2.7,
            max_distance=6.0,
            obstacle_clearance=0.2,
            known_clearance=0.1,
            corridor_clearance=0.15,
            frontier_radius=0.4,
            min_frontier_cells=8,
        )

        self.assertIsNotNone(goal)
        self.assertGreater(goal.y, 2.5)

    def test_filters_small_noise_frontier_clusters(self):
        grid = make_map(width=24, height=24, resolution=0.1)
        set_rect_free(grid, 0, 0, 23, 23)
        grid.data[12 * grid.width + 12] = -1

        goal = select_coverage_goal(
            grid,
            Pose2D(0.5, 0.5),
            min_distance=0.2,
            max_distance=3.0,
            obstacle_clearance=0.1,
            known_clearance=0.05,
            frontier_radius=0.2,
            min_frontier_cells=12,
        )

        self.assertIsNone(goal)

    def test_costmap_rejects_goal_candidates(self):
        grid = make_map()
        set_rect_free(grid, 4, 4, 22, 22)
        costmap = GridMap(grid.width, grid.height, grid.resolution, 0.0, 0.0, [0] * (grid.width * grid.height))
        set_rect_occupied(costmap, 0, 0, costmap.width - 1, costmap.height - 1)

        goal = select_coverage_goal(
            grid,
            Pose2D(0.7, 0.7),
            global_costmap=costmap,
            min_distance=0.4,
            max_distance=1.8,
            obstacle_clearance=0.2,
            known_clearance=0.1,
        )

        self.assertIsNone(goal)


    def test_selects_frontier_through_standard_doorway(self):
        grid = make_map(width=120, height=72, resolution=0.05)
        set_rect_occupied(grid, 0, 0, 119, 71)
        set_rect_free(grid, 6, 12, 36, 58)
        set_rect_free(grid, 37, 26, 76, 41)
        set_rect(grid, 77, 12, 119, 58, -1)

        goal = select_coverage_goal(
            grid,
            Pose2D(0.8, 1.6),
            min_distance=0.4,
            max_distance=3.8,
            obstacle_clearance=0.38,
            known_clearance=0.2,
            corridor_clearance=0.28,
            frontier_radius=0.5,
            min_frontier_cells=8,
        )

        self.assertIsNotNone(goal)
        self.assertGreater(goal.x, 2.0)

    def test_bad_place_radius_does_not_block_entire_frontier_area(self):
        grid = make_map(width=50, height=30, resolution=0.1)
        set_rect_free(grid, 4, 4, 42, 24)
        first = select_coverage_goal(
            grid,
            Pose2D(0.8, 0.8),
            min_distance=0.4,
            max_distance=3.8,
            obstacle_clearance=0.2,
            known_clearance=0.1,
        )
        self.assertIsNotNone(first)

        second = select_coverage_goal(
            grid,
            Pose2D(0.8, 0.8),
            min_distance=0.4,
            max_distance=3.8,
            obstacle_clearance=0.2,
            known_clearance=0.1,
            bad_places=[(first.x, first.y, 0.55)],
            bad_radius=0.55,
        )

        self.assertIsNotNone(second)
        self.assertGreaterEqual(((second.x - first.x) ** 2 + (second.y - first.y) ** 2) ** 0.5, 0.55)

    def test_large_map_selection_stays_bounded(self):
        grid = make_map(width=140, height=120, resolution=0.05)
        set_rect_free(grid, 10, 10, 120, 95)
        set_rect_occupied(grid, 55, 10, 60, 70)

        started = time.perf_counter()
        goal = select_coverage_goal(
            grid,
            Pose2D(1.0, 1.0),
            min_distance=0.4,
            max_distance=4.0,
            obstacle_clearance=0.38,
            known_clearance=0.2,
            corridor_clearance=0.28,
            frontier_radius=0.7,
            min_frontier_cells=8,
        )
        elapsed = time.perf_counter() - started

        self.assertIsNotNone(goal)
        self.assertLess(elapsed, 3.0)



if __name__ == '__main__':
    unittest.main()
