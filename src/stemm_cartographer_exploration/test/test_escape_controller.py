"""Unit tests for escape_controller's Nav2-equivalent footprint check.

These tests do not need ROS topics or a live system - they pin the two
behaviours the node's exit decision depends on:

  1. footprintCost takes the MAXIMUM raw cost along the rasterised footprint
     edges (not the interior, not an average) - matching
     nav2_costmap_2d::FootprintCollisionChecker::lineCost / footprintCost.
  2. A point off the costmap grid counts as LETHAL - matching the
     worldToMap-failure path in footprintCost().

Both matter because the node's whole contract is "release exactly when Nav2
would call this heading collision free".
"""

import math

try:
    from stemm_cartographer_exploration.escape_controller import (
        LETHAL_OBSTACLE,
        EscapeController,
        bresenham,
        _yaw_from_quaternion,
    )
except ModuleNotFoundError:  # Local patch-staging execution.
    from escape_controller import (
        LETHAL_OBSTACLE,
        EscapeController,
        bresenham,
        _yaw_from_quaternion,
    )


class _Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Pose:
    """nav2_msgs/Costmap metadata.origin is a geometry_msgs/Pose."""

    def __init__(self, x, y):
        self.position = _Point(x, y)


class _Meta:
    def __init__(self, size_x, size_y, resolution, ox, oy):
        self.size_x = size_x
        self.size_y = size_y
        self.resolution = resolution
        self.origin = _Pose(ox, oy)


class _Costmap:
    def __init__(self, size_x, size_y, resolution, ox, oy, data):
        self.metadata = _Meta(size_x, size_y, resolution, ox, oy)
        self.data = data


def _blank(size_x=20, size_y=20, res=0.05, ox=-0.5, oy=-0.5, fill=0):
    return _Costmap(size_x, size_y, res, ox, oy,
                    bytearray([fill] * (size_x * size_y)))


def _make_node_like(costmap, pose):
    """Bind the pure methods of EscapeController without running rclpy."""
    node = EscapeController.__new__(EscapeController)
    node.costmap = costmap
    node.pose = pose
    return node


def _cell_of(cm, wx, wy):
    """World point -> cell index, using the same conversion as the node."""
    res = cm.metadata.resolution
    return (int((wx - cm.metadata.origin.position.x) / res),
            int((wy - cm.metadata.origin.position.y) / res))


SQUARE = [(0.2, 0.2), (0.2, -0.2), (-0.2, -0.2), (-0.2, 0.2)]


def test_bresenham_hits_endpoints_and_is_connected():
    cells = list(bresenham(0, 0, 5, 3))
    assert cells[0] == (0, 0)
    assert cells[-1] == (5, 3)
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        assert max(abs(x1 - x0), abs(y1 - y0)) == 1


def test_yaw_roundtrip():
    for deg in (0.0, 37.0, -120.0, 179.0):
        yaw = math.radians(deg)
        q = type('Q', (), {})()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        assert abs(_yaw_from_quaternion(q) - yaw) < 1e-9


def test_free_costmap_reports_zero():
    node = _make_node_like(_blank(), (0.0, 0.0, 0.0))
    assert node.footprint_cost(0.0, 0.0, 0.0, SQUARE) == 0.0


def test_off_grid_counts_as_lethal():
    """A robot far outside the local costmap must read as LETHAL, not free."""
    node = _make_node_like(_blank(), (0.0, 0.0, 0.0))
    assert node.footprint_cost(99.0, 99.0, 0.0, SQUARE) == float(LETHAL_OBSTACLE)


def test_max_not_average_along_edge():
    """One lethal cell on an edge must dominate the whole footprint cost."""
    cm = _blank(size_x=40, size_y=40, ox=-1.0, oy=-1.0)
    # a lethal cell sitting exactly on the +x edge of a 0.2 m half-size square
    mx, my = _cell_of(cm, 0.2, 0.0)
    cm.data[my * cm.metadata.size_x + mx] = LETHAL_OBSTACLE

    node = _make_node_like(cm, (0.0, 0.0, 0.0))
    assert node.footprint_cost(0.0, 0.0, 0.0, SQUARE) == float(LETHAL_OBSTACLE)


def test_cost_below_lethal_at_edge_still_reports_the_maximum():
    """A non-lethal cost on an edge must surface as the footprint cost."""
    cm = _blank(size_x=40, size_y=40, ox=-1.0, oy=-1.0)
    mx, my = _cell_of(cm, 0.2, 0.0)
    cm.data[my * cm.metadata.size_x + mx] = 137

    node = _make_node_like(cm, (0.0, 0.0, 0.0))
    cost = node.footprint_cost(0.0, 0.0, 0.0, SQUARE)
    assert cost == 137.0
    assert cost < float(LETHAL_OBSTACLE)


def test_inflated_below_lethal_is_not_a_collision():
    """253 (INSCRIBED_INFLATED_OBSTACLE) must NOT be treated as a collision.

    This is the whole point: Nav2 compares >= LETHAL_OBSTACLE (254), so the node
    must accept 253 and below as traversable, otherwise it would refuse to
    release the robot on headings Nav2 considers fine.
    """
    cm = _blank(size_x=40, size_y=40, ox=-1.0, oy=-1.0, fill=253)
    node = _make_node_like(cm, (0.0, 0.0, 0.0))
    cost = node.footprint_cost(0.0, 0.0, 0.0, SQUARE)
    assert cost == 253.0
    assert cost < float(LETHAL_OBSTACLE)


def test_rotation_changes_the_verdict():
    """Rotating the same footprint must be able to clear a collision.

    This is the exact premise of the node: an obstruction the footprint edge
    sweeps at one yaw is not swept at every yaw.  A rectangular footprint is
    used deliberately - a square is invariant under 90 degree rotation, which
    would hide the effect entirely.

    The obstruction position was found by brute force over the 72 headings
    rather than hand-derived: cells (2, 2) relative to the robot, i.e.
    (0.10, 0.10) m, is swept at 28 of the 72 headings and missed at the other
    44, so it exercises both outcomes.
    """
    cm = _blank(size_x=80, size_y=80, ox=-2.0, oy=-2.0)
    res = cm.metadata.resolution
    # Convert the robot-relative cell into an absolute cell before writing, so
    # the obstruction really sits 0.10 m away rather than 2 m away.
    want_mx, want_my = 2, 2
    mx = want_mx + int(round((0.0 - cm.metadata.origin.position.x) / res))
    my = want_my + int(round((0.0 - cm.metadata.origin.position.y) / res))
    assert (mx, my) == (42, 42)
    cm.data[my * cm.metadata.size_x + mx] = LETHAL_OBSTACLE

    world_x = cm.metadata.origin.position.x + mx * res
    world_y = cm.metadata.origin.position.y + my * res
    assert abs(math.hypot(world_x, world_y) - math.hypot(0.10, 0.10)) < 0.04

    # 0.60 m long x 0.24 m wide: clearly not square
    rect = [(0.30, 0.12), (0.30, -0.12), (-0.30, -0.12), (-0.30, 0.12)]

    node = _make_node_like(cm, (0.0, 0.0, 0.0))
    verdicts = []
    for i in range(72):
        yaw = math.radians(i * 5.0)
        cost = node.footprint_cost(0.0, 0.0, yaw, rect)
        assert cost is not None
        verdicts.append(cost >= float(LETHAL_OBSTACLE))

    assert any(verdicts), 'the obstruction must be swept at some yaw'
    assert not all(verdicts), (
        'rotating must be able to clear it, otherwise the node could never '
        'release the robot')


def test_missing_inputs_yield_none():
    node = _make_node_like(None, (0.0, 0.0, 0.0))
    assert node.footprint_cost(0.0, 0.0, 0.0, SQUARE) is None
    node2 = _make_node_like(_blank(), (0.0, 0.0, 0.0))
    assert node2.footprint_cost(0.0, 0.0, 0.0, []) is None
