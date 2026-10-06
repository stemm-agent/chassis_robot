import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]


@dataclass
class Pose2D:
    x: float
    y: float


@dataclass
class GoalCandidate:
    x: float
    y: float
    score: float


@dataclass
class FrontierCluster:
    cells: List[int]
    centroid_x: float
    centroid_y: float
    information_gain: float


@dataclass
class DistanceField:
    grid: GridMap
    distances: Sequence[float]


def world_to_map(grid: GridMap, x: float, y: float) -> Optional[Tuple[int, int]]:
    mx = int((x - grid.origin_x) / grid.resolution)
    my = int((y - grid.origin_y) / grid.resolution)
    if mx < 0 or my < 0 or mx >= grid.width or my >= grid.height:
        return None
    return mx, my


def map_to_world(grid: GridMap, mx: int, my: int) -> Tuple[float, float]:
    return (
        grid.origin_x + (mx + 0.5) * grid.resolution,
        grid.origin_y + (my + 0.5) * grid.resolution,
    )


def cell_value(grid: GridMap, mx: int, my: int) -> int:
    return grid.data[my * grid.width + mx]


def cell_index(grid: GridMap, mx: int, my: int) -> int:
    return my * grid.width + mx


def index_to_cell(grid: GridMap, index: int) -> Tuple[int, int]:
    return index % grid.width, index // grid.width


def is_known_free_value(value: int) -> bool:
    return 0 <= value < 30


def is_occupied_value(value: int) -> bool:
    return value >= 50


def is_costmap_blocked_value(value: int) -> bool:
    return value >= 90


def iter_neighbors4(grid: GridMap, mx: int, my: int):
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx = mx + dx
        ny = my + dy
        if 0 <= nx < grid.width and 0 <= ny < grid.height:
            yield nx, ny


def iter_neighbors8(grid: GridMap, mx: int, my: int):
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx = mx + dx
            ny = my + dy
            if 0 <= nx < grid.width and 0 <= ny < grid.height:
                yield nx, ny


def iter_radius_cells(grid: GridMap, mx: int, my: int, radius: float):
    cells = max(1, int(math.ceil(radius / grid.resolution)))
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            nx = mx + dx
            ny = my + dy
            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                continue
            if math.hypot(dx * grid.resolution, dy * grid.resolution) <= radius:
                yield nx, ny


def build_distance_field(grid: GridMap, blocked_value) -> DistanceField:
    total = grid.width * grid.height
    distances = [math.inf] * total
    for index, value in enumerate(grid.data):
        if blocked_value(value):
            distances[index] = 0.0

    cardinal = grid.resolution
    diagonal = grid.resolution * math.sqrt(2.0)
    forward_neighbors = ((-1, 0, cardinal), (0, -1, cardinal), (-1, -1, diagonal), (1, -1, diagonal))
    backward_neighbors = ((1, 0, cardinal), (0, 1, cardinal), (1, 1, diagonal), (-1, 1, diagonal))

    for y in range(grid.height):
        for x in range(grid.width):
            index = cell_index(grid, x, y)
            best = distances[index]
            for dx, dy, cost in forward_neighbors:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < grid.width and 0 <= ny < grid.height:
                    best = min(best, distances[cell_index(grid, nx, ny)] + cost)
            distances[index] = best

    for y in range(grid.height - 1, -1, -1):
        for x in range(grid.width - 1, -1, -1):
            index = cell_index(grid, x, y)
            best = distances[index]
            for dx, dy, cost in backward_neighbors:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < grid.width and 0 <= ny < grid.height:
                    best = min(best, distances[cell_index(grid, nx, ny)] + cost)
            distances[index] = best

    return DistanceField(grid, distances)


def distance_at(field: Optional[DistanceField], mx: int, my: int, default: float) -> float:
    if field is None:
        return default
    return field.distances[cell_index(field.grid, mx, my)]


def has_unknown_near(grid: GridMap, mx: int, my: int, radius: float) -> bool:
    return any(cell_value(grid, nx, ny) < 0 for nx, ny in iter_radius_cells(grid, mx, my, radius))


def count_unknown_near(grid: GridMap, mx: int, my: int, radius: float) -> int:
    return sum(1 for nx, ny in iter_radius_cells(grid, mx, my, radius) if cell_value(grid, nx, ny) < 0)


def has_occupied_near(grid: GridMap, mx: int, my: int, radius: float) -> bool:
    return any(is_occupied_value(cell_value(grid, nx, ny)) for nx, ny in iter_radius_cells(grid, mx, my, radius))


def is_goal_safe(
    grid: GridMap,
    mx: int,
    my: int,
    obstacle_clearance: float = 0.5,
    known_clearance: float = 0.25,
    obstacle_field: Optional[DistanceField] = None,
) -> bool:
    value = cell_value(grid, mx, my)
    if not is_known_free_value(value):
        return False

    if obstacle_field is not None:
        if distance_at(obstacle_field, mx, my, obstacle_clearance) < obstacle_clearance:
            return False
    elif has_occupied_near(grid, mx, my, obstacle_clearance):
        return False

    for nx, ny in iter_radius_cells(grid, mx, my, known_clearance):
        value = cell_value(grid, nx, ny)
        if value < 0 or is_occupied_value(value):
            return False

    return True


def line_is_clear(
    grid: GridMap,
    start: Pose2D,
    end: Pose2D,
    corridor_clearance: float = 0.35,
    start_skip_distance: float = 0.25,
) -> bool:
    distance = math.hypot(end.x - start.x, end.y - start.y)
    steps = max(1, int(math.ceil(distance / max(grid.resolution, 0.05))))
    for index in range(steps + 1):
        ratio = index / steps
        if distance * ratio < start_skip_distance:
            continue
        x = start.x + (end.x - start.x) * ratio
        y = start.y + (end.y - start.y) * ratio
        cell = world_to_map(grid, x, y)
        if cell is None:
            return False
        for nx, ny in iter_radius_cells(grid, cell[0], cell[1], corridor_clearance):
            if is_occupied_value(cell_value(grid, nx, ny)):
                return False
    return True


def near_bad_place(x: float, y: float, bad_places: Iterable[Tuple[float, ...]], bad_radius: float) -> bool:
    for place in bad_places:
        bx = place[0]
        by = place[1]
        radius = place[2] if len(place) >= 3 else bad_radius
        if math.hypot(x - bx, y - by) <= radius:
            return True
    return False


def make_traversable_checker(grid: GridMap, clearance: float, obstacle_field: Optional[DistanceField] = None):
    cache: Dict[int, bool] = {}

    def traversable(mx: int, my: int) -> bool:
        index = cell_index(grid, mx, my)
        if index in cache:
            return cache[index]
        value = cell_value(grid, mx, my)
        obstacle_distance = distance_at(obstacle_field, mx, my, clearance) if obstacle_field else clearance
        ok = is_known_free_value(value) and obstacle_distance >= clearance
        cache[index] = ok
        return ok

    return traversable


def nearest_reachable_seed(grid: GridMap, pose: Pose2D, traversable) -> Optional[Tuple[int, int]]:
    pose_cell = world_to_map(grid, pose.x, pose.y)
    if pose_cell is None:
        return None
    if traversable(*pose_cell):
        return pose_cell

    max_cells = max(2, int(math.ceil(0.7 / grid.resolution)))
    px, py = pose_cell
    best = None
    best_distance = None
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            mx = px + dx
            my = py + dy
            if mx < 0 or my < 0 or mx >= grid.width or my >= grid.height:
                continue
            if not traversable(mx, my):
                continue
            distance = math.hypot(dx, dy)
            if best_distance is None or distance < best_distance:
                best = (mx, my)
                best_distance = distance
    return best


def reachable_free_space(
    grid: GridMap,
    pose: Pose2D,
    corridor_clearance: float,
    obstacle_field: Optional[DistanceField] = None,
) -> Dict[int, float]:
    traversable = make_traversable_checker(grid, corridor_clearance, obstacle_field)
    seed = nearest_reachable_seed(grid, pose, traversable)
    if seed is None:
        return {}

    seed_index = cell_index(grid, seed[0], seed[1])
    distances: Dict[int, float] = {seed_index: 0.0}
    queue = deque([seed])
    while queue:
        mx, my = queue.popleft()
        base_distance = distances[cell_index(grid, mx, my)]
        for nx, ny in iter_neighbors4(grid, mx, my):
            index = cell_index(grid, nx, ny)
            if index in distances or not traversable(nx, ny):
                continue
            distances[index] = base_distance + grid.resolution
            queue.append((nx, ny))
    return distances


def is_frontier_cell(grid: GridMap, mx: int, my: int) -> bool:
    if not is_known_free_value(cell_value(grid, mx, my)):
        return False
    return any(cell_value(grid, nx, ny) < 0 for nx, ny in iter_neighbors8(grid, mx, my))


def cluster_frontiers(grid: GridMap, frontier_indices: Set[int]) -> List[List[int]]:
    clusters = []
    visited: Set[int] = set()
    for start in frontier_indices:
        if start in visited:
            continue
        visited.add(start)
        cluster = []
        queue = deque([start])
        while queue:
            index = queue.popleft()
            cluster.append(index)
            mx, my = index_to_cell(grid, index)
            for nx, ny in iter_neighbors8(grid, mx, my):
                neighbor = cell_index(grid, nx, ny)
                if neighbor in frontier_indices and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        clusters.append(cluster)
    return clusters


def cluster_information_gain(grid: GridMap, cells: Sequence[int], radius: float) -> int:
    unknown_cells = set()
    sample_step = max(1, len(cells) // 120)
    for index in cells[::sample_step]:
        mx, my = index_to_cell(grid, index)
        for nx, ny in iter_radius_cells(grid, mx, my, radius):
            if cell_value(grid, nx, ny) < 0:
                unknown_cells.add(cell_index(grid, nx, ny))
    return len(unknown_cells)


def build_frontier_clusters(
    grid: GridMap,
    reachable: Dict[int, float],
    frontier_radius: float,
    min_frontier_cells: int,
) -> List[FrontierCluster]:
    frontier_indices = set()
    checked = set()
    for index in reachable:
        mx, my = index_to_cell(grid, index)
        cells = [(mx, my), *iter_neighbors8(grid, mx, my)]
        for nx, ny in cells:
            neighbor = cell_index(grid, nx, ny)
            if neighbor in checked:
                continue
            checked.add(neighbor)
            if is_frontier_cell(grid, nx, ny):
                frontier_indices.add(neighbor)
    clusters = []
    for cells in cluster_frontiers(grid, frontier_indices):
        if len(cells) < min_frontier_cells:
            continue
        xs = []
        ys = []
        for index in cells:
            x, y = map_to_world(grid, *index_to_cell(grid, index))
            xs.append(x)
            ys.append(y)
        information_gain = len(cells) + cluster_information_gain(grid, cells, frontier_radius)
        clusters.append(FrontierCluster(cells, sum(xs) / len(xs), sum(ys) / len(ys), information_gain))
    return clusters


def costmap_allows_goal(
    costmap: Optional[GridMap],
    x: float,
    y: float,
    obstacle_clearance: float,
    known_clearance: float,
    obstacle_field: Optional[DistanceField] = None,
) -> bool:
    del obstacle_clearance
    if costmap is None:
        return True
    cell = world_to_map(costmap, x, y)
    if cell is None:
        return True
    mx, my = cell
    value = cell_value(costmap, mx, my)
    if value < 0 or is_costmap_blocked_value(value):
        return False

    if obstacle_field is not None:
        return distance_at(obstacle_field, mx, my, known_clearance) >= known_clearance

    for nx, ny in iter_radius_cells(costmap, mx, my, known_clearance):
        if is_costmap_blocked_value(cell_value(costmap, nx, ny)):
            return False
    return True


def nearest_obstacle_distance(
    grid: GridMap,
    mx: int,
    my: int,
    search_radius: float,
    obstacle_field: Optional[DistanceField] = None,
) -> float:
    if obstacle_field is not None:
        return min(search_radius, distance_at(obstacle_field, mx, my, search_radius))
    best = search_radius
    for nx, ny in iter_radius_cells(grid, mx, my, search_radius):
        if is_occupied_value(cell_value(grid, nx, ny)):
            distance = math.hypot((nx - mx) * grid.resolution, (ny - my) * grid.resolution)
            best = min(best, distance)
    return best


def nearest_costmap_obstacle_distance(
    grid: GridMap,
    mx: int,
    my: int,
    search_radius: float,
    obstacle_field: Optional[DistanceField] = None,
) -> float:
    if obstacle_field is not None:
        return min(search_radius, distance_at(obstacle_field, mx, my, search_radius))
    best = search_radius
    for nx, ny in iter_radius_cells(grid, mx, my, search_radius):
        if is_costmap_blocked_value(cell_value(grid, nx, ny)):
            distance = math.hypot((nx - mx) * grid.resolution, (ny - my) * grid.resolution)
            best = min(best, distance)
    return best


def costmap_clearance(
    costmap: Optional[GridMap],
    x: float,
    y: float,
    search_radius: float,
    obstacle_field: Optional[DistanceField] = None,
) -> float:
    if costmap is None:
        return search_radius
    cell = world_to_map(costmap, x, y)
    if cell is None:
        return search_radius
    return nearest_costmap_obstacle_distance(costmap, cell[0], cell[1], search_radius, obstacle_field)


def cluster_candidate_indices(
    grid: GridMap,
    cluster: FrontierCluster,
    reachable: Dict[int, float],
    frontier_radius: float,
) -> Set[int]:
    candidates = set()
    sample_step = max(1, len(cluster.cells) // 100)
    for index in cluster.cells[::sample_step]:
        mx, my = index_to_cell(grid, index)
        for nx, ny in iter_radius_cells(grid, mx, my, frontier_radius):
            neighbor = cell_index(grid, nx, ny)
            if neighbor in reachable:
                candidates.add(neighbor)
    return candidates


def nearest_cluster_distance(grid: GridMap, candidate: int, cells: Sequence[int]) -> float:
    mx, my = index_to_cell(grid, candidate)
    best = None
    sample_step = max(1, len(cells) // 80)
    for index in cells[::sample_step]:
        fx, fy = index_to_cell(grid, index)
        distance = math.hypot((fx - mx) * grid.resolution, (fy - my) * grid.resolution)
        if best is None or distance < best:
            best = distance
    return best if best is not None else 0.0


def lookahead_gain(x: float, y: float, cluster: FrontierCluster, clusters: Sequence[FrontierCluster]) -> float:
    best = 0.0
    for other in clusters:
        if other is cluster:
            continue
        distance = math.hypot(other.centroid_x - x, other.centroid_y - y)
        best = max(best, other.information_gain / (1.0 + distance))
    return best


def select_coverage_goal(
    grid: GridMap,
    pose: Pose2D,
    bad_places: Optional[Iterable[Tuple[float, ...]]] = None,
    min_distance: float = 0.6,
    max_distance: float = 1.5,
    obstacle_clearance: float = 0.5,
    known_clearance: float = 0.25,
    corridor_clearance: float = 0.35,
    frontier_radius: float = 0.7,
    bad_radius: float = 1.2,
    stride: int = 2,
    global_costmap: Optional[GridMap] = None,
    local_costmap: Optional[GridMap] = None,
    min_frontier_cells: int = 8,
    gain_scale: float = 1.0,
    potential_scale: float = 1.2,
    clearance_scale: float = 0.8,
    lookahead_scale: float = 0.35,
) -> Optional[GoalCandidate]:
    del stride
    bad_places = list(bad_places or [])
    obstacle_field = build_distance_field(grid, is_occupied_value)
    global_costmap_field = build_distance_field(global_costmap, is_costmap_blocked_value) if global_costmap else None
    local_costmap_field = build_distance_field(local_costmap, is_costmap_blocked_value) if local_costmap else None
    reachable = reachable_free_space(grid, pose, corridor_clearance, obstacle_field)
    if not reachable:
        return None

    clusters = build_frontier_clusters(grid, reachable, frontier_radius, max(1, min_frontier_cells))
    if not clusters:
        return None

    best = None
    best_score = None
    clearance_search_radius = max(obstacle_clearance + 0.25, obstacle_clearance * 2.0)
    for cluster in clusters:
        for candidate in cluster_candidate_indices(grid, cluster, reachable, frontier_radius):
            mx, my = index_to_cell(grid, candidate)
            x, y = map_to_world(grid, mx, my)
            path_distance = reachable[candidate]
            if path_distance < min_distance or path_distance > max_distance:
                continue
            if near_bad_place(x, y, bad_places, bad_radius):
                continue
            if not is_goal_safe(grid, mx, my, obstacle_clearance, known_clearance, obstacle_field):
                continue
            if not costmap_allows_goal(global_costmap, x, y, obstacle_clearance, known_clearance, global_costmap_field):
                continue
            if not costmap_allows_goal(local_costmap, x, y, obstacle_clearance, known_clearance, local_costmap_field):
                continue

            map_clearance = nearest_obstacle_distance(grid, mx, my, clearance_search_radius, obstacle_field)
            global_clearance = costmap_clearance(global_costmap, x, y, clearance_search_radius, global_costmap_field)
            local_clearance = costmap_clearance(local_costmap, x, y, clearance_search_radius, local_costmap_field)
            clearance = min(map_clearance, global_clearance, local_clearance)
            frontier_distance = nearest_cluster_distance(grid, candidate, cluster.cells)
            score = (
                gain_scale * cluster.information_gain
                - potential_scale * path_distance
                + clearance_scale * clearance
                + lookahead_scale * lookahead_gain(x, y, cluster, clusters)
                - 0.25 * frontier_distance
            )
            if best_score is None or score > best_score:
                best_score = score
                best = GoalCandidate(x, y, score)

    return best
