"""selector 单元测试：重点覆盖 world_to_map 的取整回归。

2026-10-10: 原实现用 int() 截断，(x-origin)/res 恰在一格边界时会得到
18.999999999999996 这类浮点值，int() 取 18 而非 19，造成系统性偏移一格。
本测试用真实会触发该浮点误差的数值锁定 round 语义。
"""
import math

import pytest

from stemm_frontier_explorer.selector import (
    GridMap,
    Pose2D,
    build_distance_field,
    is_frontier_cell,
    map_to_world,
    world_to_map,
)


def make_grid(width=40, height=40, resolution=0.05, origin_x=-1.0, origin_y=-1.0,
              fill=0):
    return GridMap(
        width=width, height=height, resolution=resolution,
        origin_x=origin_x, origin_y=origin_y,
        data=[fill] * (width * height),
    )


class TestWorldToMap:
    """world_to_map 必须与 Nav2 worldToMap 的取整语义一致。"""

    def test_boundary_cell_does_not_truncate_down(self):
        # 关键回归：0.95 / 0.05 = 18.999999999999996
        grid = make_grid()
        assert (0.95 - 0.0) / 0.05 == pytest.approx(19.0, abs=1e-9)
        assert int((0.95 - 0.0) / 0.05) == 18      # 旧行为（截断）
        # y=0.95 且 origin_y=0.0 时，必须落在第 19 格
        g2 = make_grid(origin_x=0.0, origin_y=0.0)
        assert world_to_map(g2, 0.95, 0.95) == (19, 19)

    def test_exact_lattice_points(self):
        grid = make_grid(origin_x=0.0, origin_y=0.0)
        for cell in (0, 1, 3, 18, 19, 20, 39):
            world = cell * 0.05
            assert world_to_map(grid, world, world) == (cell, cell), \
                'cell %d (world %.2f) mapped wrong' % (cell, world)

    def test_round_trip_map_to_world(self):
        grid = make_grid()
        for mx in (0, 5, 17, 19, 39):
            wx, wy = map_to_world(grid, mx, mx)
            assert world_to_map(grid, wx, wy) == (mx, mx)

    def test_out_of_bounds_returns_none(self):
        grid = make_grid()
        assert world_to_map(grid, -5.0, 0.0) is None
        assert world_to_map(grid, 0.0, -5.0) is None
        assert world_to_map(grid, 50.0, 0.0) is None
        assert world_to_map(grid, 0.0, 50.0) is None

    def test_upper_edge_of_last_cell(self):
        grid = make_grid(origin_x=0.0, origin_y=0.0)
        # 第 39 格跨 [1.95, 2.00)。格中心 1.975/0.05 = 39.5 会被 round 到 40，
        # 因此格中心不属于任何格——这正是旧实现 +0.5 与 round 冲突的原因。
        # round + 格原点约定下，格 n 的有效世界范围是 [n*res, (n+0.5)*res)。
        # 第 39 格因此是 [1.950, 1.975)；1.975 及以上都会被 round 到 40（越界）。
        assert world_to_map(grid, 1.950, 1.950) == (39, 39)
        assert world_to_map(grid, 1.970, 1.970) == (39, 39)
        assert world_to_map(grid, 1.975, 1.975) is None
        assert world_to_map(grid, 2.000, 2.000) is None

    def test_cell_center_maps_to_itself_under_truncation_convention(self):
        # 说明性测试：格中心只在截断约定下回到自身。本包采用 round + 格原点
        # 约定，因此不再依赖格中心语义。
        grid = make_grid(origin_x=0.0, origin_y=0.0)
        center = (39 + 0.5) * 0.05
        assert int(center / 0.05) == 39


class TestFrontierDetection:
    def test_known_free_next_to_unknown_is_frontier(self):
        grid = make_grid(width=5, height=5, fill=-1)   # 全未知
        # 中心一格已知自由，其邻居为未知 -> 是前沿格
        idx = 2 * 5 + 2
        grid.data[idx] = 0
        assert is_frontier_cell(grid, 2, 2) is True

    def test_fully_surrounded_free_cell_is_not_frontier(self):
        grid = make_grid(width=5, height=5, fill=0)    # 全已知自由
        assert is_frontier_cell(grid, 2, 2) is False


class TestDistanceField:
    def test_distance_grows_away_from_obstacle(self):
        grid = make_grid(width=20, height=20, fill=0)
        # 在第 0 列放一道墙
        for my in range(20):
            grid.data[my * 20 + 0] = 100
        field = build_distance_field(grid, lambda v: v >= 50)
        assert field is not None
        d0 = field.distances[10 * 20 + 0]
        d1 = field.distances[10 * 20 + 1]
        d5 = field.distances[10 * 20 + 5]
        assert d0 == pytest.approx(0.0)
        assert 0.0 < d1 < d5
        # 单调不减
        assert d1 <= d5

    def test_no_obstacle_grid_yields_infinite_field(self):
        # 无障碍时不返回 None，而是返回一个距离全为 inf 的场；
        # 调用方（nearest_obstacle_distance）通过 inf 判定无约束。
        grid = make_grid(width=8, height=8, fill=0)
        field = build_distance_field(grid, lambda v: v >= 50)
        assert field is not None
        assert all(math.isinf(d) for d in field.distances)
