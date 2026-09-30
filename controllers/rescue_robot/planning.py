"""팀 점유지도와 경로 규격을 사용하는 순수 Python Planning 모듈입니다.

A*는 기본 4연결이며, 8연결은 명시적으로 선택해야 합니다. 원시 LiDAR에 대한
마지막 안전 검사는 main의 SafetyMonitor가 담당합니다.
"""

import heapq
import math
from collections import deque

import config
from interfaces import FREE, OCCUPIED, UNKNOWN


NEIGHBORS_4 = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0))
NEIGHBORS = NEIGHBORS_4 + (
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)


def inside(grid, row, col):
    """셀 좌표가 지도 범위 안인지 확인합니다."""
    return bool(grid) and 0 <= row < len(grid) and 0 <= col < len(grid[row])


def manhattan(a, b):
    """4연결 경로의 장애물 없는 최소 이동 거리를 반환합니다."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def heuristic(a, b):
    """8연결 경로에 사용할 유클리드 거리 하한을 반환합니다."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def find_frontiers(grid):
    """UNKNOWN과 직교 방향으로 인접한 FREE 셀을 행/열 순서로 반환합니다."""
    frontiers = []
    for row, cells in enumerate(grid):
        for col, value in enumerate(cells):
            if value != FREE:
                continue
            if any(inside(grid, row + dr, col + dc)
                   and grid[row + dr][col + dc] == UNKNOWN
                   for dr, dc, _ in NEIGHBORS_4):
                frontiers.append((row, col))
    return frontiers


def cluster_frontiers(frontiers, min_size=1):
    """8연결 frontier를 묶고 큰 클러스터부터 결정적인 순서로 반환합니다."""
    remaining = set(frontiers)
    clusters = []
    for point in sorted(remaining):
        if point not in remaining:
            continue
        remaining.remove(point)
        queue = deque([point])
        cluster = []
        while queue:
            row, col = queue.popleft()
            cluster.append((row, col))
            for dr, dc, _ in NEIGHBORS:
                neighbor = (row + dr, col + dc)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
        if len(cluster) >= min_size:
            clusters.append(sorted(cluster))
    clusters.sort(key=lambda cluster: (-len(cluster), cluster))
    return clusters


def cluster_centroid(cluster):
    """클러스터의 평균 (row, col)을 반환합니다. 빈 클러스터는 None입니다."""
    if not cluster:
        return None
    return (sum(row for row, _ in cluster) / len(cluster),
            sum(col for _, col in cluster) / len(cluster))


def information_gain(grid, frontier, radius=4):
    """frontier 주변에서 아직 관측하지 않은 셀의 수를 반환합니다."""
    row, col = frontier
    return sum(inside(grid, row + dr, col + dc)
               and grid[row + dr][col + dc] == UNKNOWN
               for dr in range(-radius, radius + 1)
               for dc in range(-radius, radius + 1))


def _resolution(resolution):
    value = config.GRID_RESOLUTION if resolution is None else float(resolution)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("지도 해상도는 유한한 양수여야 합니다")
    return value


def _cell_center(cell, resolution, origin):
    row, col = cell
    return (origin[0] + (col + 0.5) * resolution,
            origin[1] + (row + 0.5) * resolution)


def frontier_score(frontier, pose, grid, resolution=None, origin=(0.0, 0.0)):
    """지도 origin과 셀 중심을 반영해 정보량/거리/회전 비용을 평가합니다.

    centered_on 지도의 호출자는 OccupancyGrid.origin을 origin으로 전달합니다.
    """
    resolution = _resolution(resolution)
    fx, fy = _cell_center(frontier, resolution, origin)
    x, y, theta = pose
    dx, dy = fx - x, fy - y
    heading = math.atan2(dy, dx)
    angle_error = abs(math.atan2(math.sin(heading - theta), math.cos(heading - theta)))
    return (3.0 * information_gain(grid, frontier)
            - 2.0 * math.hypot(dx, dy) - 1.5 * angle_error)


def select_frontier(grid, pose, resolution=None, origin=(0.0, 0.0)):
    """동일 world 좌표계의 pose와 지도 origin을 사용해 frontier를 선택합니다."""
    candidates = find_frontiers(grid)
    if not candidates:
        return None
    return max(candidates, key=lambda frontier:
               frontier_score(frontier, pose, grid, resolution, origin))


def clear_footprint(inflated, grid, center, radius_cells):
    """로봇이 서 있는 원 안의 UNKNOWN 칸과 출발 칸을 FREE로 푼 복사본을 반환합니다.

    LiDAR 최소 거리(0.12 m)와 로봇 몸체 때문에 로봇 주변은 지도에서 UNKNOWN으로
    남습니다. 로봇이 실제로 차지한 공간은 비어 있으므로 계획 출발점으로 풀어 줍니다.
    팽창으로 막힌 칸과 원본 OCCUPIED 칸은 그대로 둡니다(출발 칸만 예외).
    """
    result = [list(row) for row in inflated]
    radius = math.ceil(float(radius_cells))
    row, col = center
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            nr, nc = row + dr, col + dc
            if (dr * dr + dc * dc <= radius * radius and inside(grid, nr, nc)
                    and grid[nr][nc] != OCCUPIED
                    and (result[nr][nc] == UNKNOWN or (dr, dc) == (0, 0))):
                result[nr][nc] = FREE
    return result


def reachable_cells(grid, start):
    """start에서 FREE 칸만 밟아 갈 수 있는 칸 집합을 반환합니다(4연결 BFS).

    대각선 양옆을 모두 검사하는 8연결 A*의 도달 범위는 4연결 도달 범위와 같습니다.
    """
    if not inside(grid, *start) or grid[start[0]][start[1]] != FREE:
        return set()
    seen = {start}
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for dr, dc, _ in NEIGHBORS_4:
            cell = (row + dr, col + dc)
            if cell not in seen and inside(grid, *cell) and grid[cell[0]][cell[1]] == FREE:
                seen.add(cell)
                queue.append(cell)
    return seen


def plan_to_frontier(grid, inflated, start, pose, resolution=None, origin=(0.0, 0.0),
                     max_tries=5, footprint_cells=0):
    """도달 가능한 frontier와 경로를 (frontier, path)로 반환합니다. 없으면 (None, []).

    출발점 주변 footprint_cells 반경은 clear_footprint로 풀고, 팽창 지도에서 known FREE
    칸으로 갈 수 있는 frontier만 점수 순으로 최대 max_tries개 A*를 시도합니다.
    (점수가 높은 먼 frontier는 끊긴 LiDAR 줄기 끝인 경우가 많아 먼저 걸러 냅니다.)
    """
    if not inside(inflated, *start):
        return None, []
    inflated = clear_footprint(inflated, grid, start, footprint_cells)
    reachable = reachable_cells(inflated, start)
    candidates = [cell for cell in find_frontiers(grid) if cell in reachable]
    candidates.sort(key=lambda frontier:
                    -frontier_score(frontier, pose, grid, resolution, origin))
    for frontier in candidates[:max_tries]:
        path = astar(inflated, start, frontier, allow_unknown=False, connectivity=8)
        if path:
            return frontier, path
    return None, []

def inflate_obstacles(grid, radius_cells=2):
    """원본을 변경하지 않고 장애물을 팽창한 {-1, 0, 1} 지도를 반환합니다.

    셀 단위 실수 반경은 올림한 원형 마스크를 사용합니다. 팽창 범위 안의
    UNKNOWN도 OCCUPIED로 막으며, 범위 밖의 UNKNOWN과 FREE는 보존합니다.
    비용지도는 이 함수의 반환 규격이 아니며 A*의 costmap 인자로 따로 전달합니다.
    """
    radius_value = float(radius_cells)
    if not math.isfinite(radius_value) or radius_value < 0:
        raise ValueError("팽창 반경은 유한한 0 이상의 값이어야 합니다")
    radius = math.ceil(radius_value)
    result = [list(row) for row in grid]
    offsets = [(dr, dc) for dr in range(-radius, radius + 1)
               for dc in range(-radius, radius + 1)
               if dr * dr + dc * dc <= radius * radius]
    for row, cells in enumerate(grid):
        for col, value in enumerate(cells):
            if value != OCCUPIED:
                continue
            for dr, dc in offsets:
                nr, nc = row + dr, col + dc
                if inside(grid, nr, nc):
                    result[nr][nc] = OCCUPIED
    return result


def astar(grid, start, goal, costmap=None, allow_unknown=True, connectivity=4):
    """안전한 셀 경로를 반환합니다. 경로가 없거나 끝점이 잘못되면 []입니다.

    기존 기본값은 UNKNOWN 통과 허용입니다. known-only 계획은
    allow_unknown=False를 명시합니다. costmap은 점유지도와 크기가 같아야 하며
    254 이상을 통과 금지로 취급합니다. 8연결은 대각선 양옆 셀도 검사합니다.
    """
    if connectivity not in (4, 8):
        raise ValueError("connectivity는 4 또는 8이어야 합니다")
    if not grid or not grid[0]:
        return []
    width = len(grid[0])
    if any(len(row) != width for row in grid):
        raise ValueError("점유지도는 직사각형이어야 합니다")
    if costmap is not None and (len(costmap) != len(grid)
                               or any(len(row) != width for row in costmap)):
        raise ValueError("비용지도와 점유지도의 크기가 일치해야 합니다")

    def traversable(cell):
        if len(cell) != 2 or not all(isinstance(value, int) for value in cell):
            return False
        row, col = cell
        if not inside(grid, row, col):
            return False
        value = grid[row][col]
        if value == OCCUPIED or (value == UNKNOWN and not allow_unknown):
            return False
        if value not in (FREE, UNKNOWN):
            return False
        if costmap is not None:
            cost = costmap[row][col]
            if not math.isfinite(cost) or cost < 0 or cost >= 254:
                return False
        return True

    start, goal = tuple(start), tuple(goal)
    if not traversable(start) or not traversable(goal):
        return []
    if start == goal:
        return [start]
    neighbors = NEIGHBORS_4 if connectivity == 4 else NEIGHBORS
    estimate = manhattan if connectivity == 4 else heuristic
    queue = [(estimate(start, goal), 0.0, start)]
    parent = {start: None}
    costs = {start: 0.0}
    while queue:
        _, queued_cost, current = heapq.heappop(queue)
        if queued_cost != costs[current]:
            continue
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return list(reversed(path))
        row, col = current
        for dr, dc, move_cost in neighbors:
            nxt = (row + dr, col + dc)
            if not traversable(nxt):
                continue
            if dr and dc and (not traversable((row + dr, col))
                              or not traversable((row, col + dc))):
                continue
            extra = costmap[nxt[0]][nxt[1]] / 100.0 if costmap is not None else 0.0
            new_cost = queued_cost + move_cost + extra
            if new_cost < costs.get(nxt, math.inf):
                costs[nxt] = new_cost
                parent[nxt] = current
                heapq.heappush(queue, (new_cost + estimate(nxt, goal), new_cost, nxt))
    return []


def _segment_cost(costmap, pose, target, resolution, origin):
    """1D 전방 비용 또는 동일 지도 좌표계의 2D 이동 구간 비용을 확인합니다."""
    if costmap is None or len(costmap) == 0:
        return 0.0
    if not isinstance(costmap[0], (list, tuple)):
        values = list(costmap)
    else:
        distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
        steps = max(1, math.ceil(distance / (resolution / 2.0)))
        values = []
        for index in range(steps + 1):
            fraction = index / steps
            x = pose[0] + fraction * (target[0] - pose[0])
            y = pose[1] + fraction * (target[1] - pose[1])
            row = math.floor((y - origin[1]) / resolution)
            col = math.floor((x - origin[0]) / resolution)
            if not inside(costmap, row, col):
                return 254.0
            values.append(costmap[row][col])
    if any(not math.isfinite(value) or value < 0 for value in values):
        return 254.0
    return max(values, default=0.0)


def local_planner(path, pose, local_costmap=None, resolution=None, origin=(0.0, 0.0)):
    """셀 경로에서 제한된 (이동 속도, 회전 속도)를 계산하는 보조 함수입니다.

    origin은 OccupancyGrid.origin과 같아야 합니다. 2D costmap도 이 지도와 같은
    좌표계를 사용합니다. 이 함수만으로 LiDAR 회피 안전을 보장하지 않으며,
    main의 마지막 SafetyMonitor 검사를 반드시 적용해야 합니다. 실제 main의
    waypoint 추종은 navigation_control이 담당합니다.
    """
    if not path:
        return 0.0, 0.0
    resolution = _resolution(resolution)
    points = [_cell_center(cell, resolution, origin) for cell in path]
    goal_distance = math.hypot(points[-1][0] - pose[0], points[-1][1] - pose[1])
    if goal_distance <= config.NAV_GOAL_TOLERANCE:
        return 0.0, 0.0
    nearest = min(range(len(points)), key=lambda index:
                  math.hypot(points[index][0] - pose[0], points[index][1] - pose[1]))
    target_index = nearest
    along_path = 0.0
    while target_index < len(points) - 1 and along_path < config.NAV_LOOKAHEAD:
        first, second = points[target_index], points[target_index + 1]
        along_path += math.hypot(second[0] - first[0], second[1] - first[1])
        target_index += 1
    target = points[target_index]
    angle = math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2]
    error = math.atan2(math.sin(angle), math.cos(angle))
    angular = max(-config.MAX_ANGULAR_SPEED,
                  min(config.MAX_ANGULAR_SPEED, config.NAV_HEADING_KP * error))
    danger = _segment_cost(local_costmap, pose, target, resolution, origin)
    if danger >= 254:
        return 0.0, 0.0
    if abs(error) > config.NAV_ROTATE_THRESHOLD:
        return 0.0, angular
    speed = config.MAX_LINEAR_SPEED * min(1.0, goal_distance / config.NAV_APPROACH_DISTANCE)
    speed *= max(0.0, 1.0 - danger / 254.0)
    return speed, angular
