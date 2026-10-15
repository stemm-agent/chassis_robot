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
    # 2026-10-10: 原实现用 int() 截断。(x-origin)/res 恰在一格边界时会得到
    # 18.999999999999996 这类值，int() 取 18 而非 19，造成系统性偏移一格。
    # 与 Nav2 的 worldToMap 及 stemm_cartographer 的 map_service_adapter
    # 保持一致，改用 round 取最近格。
    mx = int(round((x - grid.origin_x) / grid.resolution))
    my = int(round((y - grid.origin_y) / grid.resolution))
    if mx < 0 or my < 0 or mx >= grid.width or my >= grid.height:
        return None
    return mx, my


def map_to_world(grid: GridMap, mx: int, my: int) -> Tuple[float, float]:
    # 2026-10-10: 原实现返回格中心（+0.5），而 world_to_map 用 round 取最近格，
    # 二者相差一格——实测 mx=0..39 经 map_to_world/world_to_map 往返后全部 +1。
    # 与 Nav2 的 mapToWorld 保持一致：返回格的左下角。这样 round 与 trunc
    # 等价，不再有"恰在两格之间"的取舍歧义。
    return (
        grid.origin_x + mx * grid.resolution,
        grid.origin_y + my * grid.resolution,
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


@dataclass
class SelectionStats:
    """2026-10-10 诊断：候选被拒的原因分类计数（纯观测，不参与决策）。"""

    reason: str = ''
    clusters: int = 0
    frontier_cells: int = 0          # 窗口内被判为前沿格的总数
    frontier_in_reachable: int = 0   # 其中落在可达域内的数量
    largest_cluster: int = 0         # 最大簇的格数（对比 min_frontier_cells）
    dropped_small_clusters: int = 0  # 因 < min_frontier_cells 被丢掉的簇数
    edge_frontier_cells: int = 0     # 前沿格中有几个是因窗口边界(100)才成前沿
    reachable_cells: int = 0         # 可达域格数（含 corridor_clearance 约束）
    known_free_cells: int = 0        # 窗口内已知自由格数
    frontier_known_free_only: int = 0  # 前沿格中"仅按 known-free 可通行"的数量
    frontier_adjacent_reachable: int = 0  # 前沿格 4 邻域里有可达格的个数
    candidates: int = 0
    reject_distance: int = 0       # path_distance 不在 [min,max]
    reject_bad_place: int = 0      # 命中失败点记忆（注意 bad_places 目前恒空）
    reject_goal_unsafe: int = 0    # is_goal_safe 失败
    reject_global_costmap: int = 0 # 全局代价不允许
    reject_local_costmap: int = 0  # 局部代价不允许

    def summary(self) -> str:
        return ('reason=%s frontier_cells=%d(in_reach=%d adj_reach=%d '
                'kf_only=%d edge=%d) reach=%d known_free=%d '
                'clusters=%d(kept) largest=%d dropped_small=%d cand=%d '
                'rej[distance=%d bad_place=%d goal_unsafe=%d global_cm=%d '
                'local_cm=%d]'
                % (self.reason or '-', self.frontier_cells,
                   self.frontier_in_reachable,
                   self.frontier_adjacent_reachable,
                   self.frontier_known_free_only,
                   self.edge_frontier_cells,
                   self.reachable_cells, self.known_free_cells,
                   self.clusters, self.largest_cluster,
                   self.dropped_small_clusters, self.candidates,
                   self.reject_distance, self.reject_bad_place,
                   self.reject_goal_unsafe, self.reject_global_costmap,
                   self.reject_local_costmap))


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
    stats: Optional[SelectionStats] = None,
) -> Optional[GoalCandidate]:
    del stride
    bad_places = list(bad_places or [])
    obstacle_field = build_distance_field(grid, is_occupied_value)
    global_costmap_field = build_distance_field(global_costmap, is_costmap_blocked_value) if global_costmap else None
    local_costmap_field = build_distance_field(local_costmap, is_costmap_blocked_value) if local_costmap else None
    reachable = reachable_free_space(grid, pose, corridor_clearance, obstacle_field)
    if not reachable:
        if stats is not None:
            stats.reason = 'no_reachable'
        return None

    # ---- 2026-10-10 诊断：前沿格普查（纯读，不改任何决策量） ----
    _min_cells = max(1, min_frontier_cells)
    if stats is not None:
        _frontier_idx = []
        for _my in range(grid.height):
            for _mx in range(grid.width):
                if is_frontier_cell(grid, _mx, _my):
                    _frontier_idx.append(cell_index(grid, _mx, _my))
        stats.frontier_cells = len(_frontier_idx)
        stats.frontier_in_reachable = sum(
            1 for _i in _frontier_idx if _i in reachable)
        stats.reachable_cells = len(reachable)
        _known = 0
        for _my in range(grid.height):
            _row = _my * grid.width
            for _mx in range(grid.width):
                if is_known_free_value(grid.data[_row + _mx]):
                    _known += 1
        stats.known_free_cells = _known
        # 前沿格若"只看 known-free"是否本来就在可达域里？用于区分
        # "corridor_clearance 把它排除了" vs "BFS 根本到不了"。
        _known_free_set = set()
        for _my in range(grid.height):
            _row = _my * grid.width
            for _mx in range(grid.width):
                if is_known_free_value(grid.data[_row + _mx]):
                    _known_free_set.add(_row + _mx)
        _kf_reach = 0
        _adj_reach = 0
        for _i in _frontier_idx:
            _mx, _my = index_to_cell(grid, _i)
            _nb = [cell_index(grid, _nx, _ny)
                   for _nx, _ny in iter_neighbors4(grid, _mx, _my)]
            if any(_n in _known_free_set for _n in _nb):
                _kf_reach += 1
            if any(_n in reachable for _n in _nb):
                _adj_reach += 1
        stats.frontier_known_free_only = _kf_reach
        stats.frontier_adjacent_reachable = _adj_reach
        if _frontier_idx:
            _raw = cluster_frontiers(grid, set(_frontier_idx))
            stats.largest_cluster = max(len(_c) for _c in _raw)
            stats.dropped_small_clusters = sum(
                1 for _c in _raw if len(_c) < _min_cells)
            # 边缘前沿格：8 邻域里没有"未知(<0)"、只有被窗口标成占用的 100
            for _i in _frontier_idx:
                _mx, _my = index_to_cell(grid, _i)
                _unk = _art = 0
                for _nx, _ny in iter_neighbors8(grid, _mx, _my):
                    _v = cell_value(grid, _nx, _ny)
                    if _v < 0:
                        _unk += 1
                    elif _v == 100:
                        _art += 1
                if _unk == 0 and _art > 0:
                    stats.edge_frontier_cells += 1

    clusters = build_frontier_clusters(grid, reachable, frontier_radius, _min_cells)
    if stats is not None:
        stats.clusters = len(clusters)
    if not clusters:
        if stats is not None:
            # 2026-10-10 诊断：把"没有簇"拆成两种根因完全不同的情况。
            if stats.frontier_in_reachable == 0:
                stats.reason = ('no_frontier_cell'
                                if stats.frontier_cells == 0
                                else 'frontier_not_reachable')
            else:
                stats.reason = 'clusters_too_small'
        return None

    best = None
    best_score = None
    clearance_search_radius = max(obstacle_clearance + 0.25, obstacle_clearance * 2.0)
    for cluster in clusters:
        for candidate in cluster_candidate_indices(grid, cluster, reachable, frontier_radius):
            mx, my = index_to_cell(grid, candidate)
            x, y = map_to_world(grid, mx, my)
            path_distance = reachable[candidate]
            if stats is not None:
                stats.candidates += 1
            if path_distance < min_distance or path_distance > max_distance:
                if stats is not None:
                    stats.reject_distance += 1
                continue
            if near_bad_place(x, y, bad_places, bad_radius):
                if stats is not None:
                    stats.reject_bad_place += 1
                continue
            if not is_goal_safe(grid, mx, my, obstacle_clearance, known_clearance, obstacle_field):
                if stats is not None:
                    stats.reject_goal_unsafe += 1
                continue
            if not costmap_allows_goal(global_costmap, x, y, obstacle_clearance, known_clearance, global_costmap_field):
                if stats is not None:
                    stats.reject_global_costmap += 1
                continue
            if not costmap_allows_goal(local_costmap, x, y, obstacle_clearance, known_clearance, local_costmap_field):
                if stats is not None:
                    stats.reject_local_costmap += 1
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
