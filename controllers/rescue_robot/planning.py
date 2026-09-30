"""Global planning baseline on the occupancy grid (Webots-independent).

Implemented:
    * astar(grid, start, goal) -> [(row, col), ...]  4-neighbour, unit cost
    * inflate_obstacles(grid, radius_cells) for safety clearance
    * find_frontiers(grid) and cluster_frontiers(frontiers)

NOT implemented (TODO, feat/planning):
    * frontier selection strategy (distance / information gain)
    * 8-neighbour moves, path smoothing, cost maps
    * replanning policy when the map changes
"""

import heapq
from collections import deque

from interfaces import FREE, OCCUPIED, UNKNOWN

NEIGHBORS_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))
NEIGHBORS_8 = NEIGHBORS_4 + ((-1, -1), (-1, 1), (1, -1), (1, 1))


def _in_bounds(grid, row, col):
    return 0 <= row < len(grid) and 0 <= col < len(grid[0])


def manhattan(a, b):
    """A* heuristic.

    With 4-neighbour moves and a cost of 1 per move, the true shortest path
    can never be shorter than |d_row| + |d_col|, so Manhattan distance is
    admissible (never overestimates) and consistent -> A* returns an optimal
    path.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal, allow_unknown=True):
    """Shortest 4-connected path from start to goal.

    grid[row][col] uses UNKNOWN/FREE/OCCUPIED. OCCUPIED cells and cells outside
    the grid are never entered. UNKNOWN cells are traversable unless
    allow_unknown=False (optimistic planning while exploring).

    Returns [start, ..., goal], [start] if start == goal, [] if no path or if
    start/goal is out of bounds or blocked.
    """
    start, goal = tuple(start), tuple(goal)
    if not grid or not grid[0]:
        return []

    def passable(cell):
        if not _in_bounds(grid, *cell):
            return False
        value = grid[cell[0]][cell[1]]
        return value != OCCUPIED and (allow_unknown or value != UNKNOWN)

    if not passable(start) or not passable(goal):
        return []
    if start == goal:
        return [start]

    counter = 0  # tie-breaker so heapq never compares cells
    open_heap = [(manhattan(start, goal), counter, start)]
    g_cost = {start: 0}
    parent = {start: None}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]
        closed.add(current)
        for dr, dc in NEIGHBORS_4:
            nxt = (current[0] + dr, current[1] + dc)
            if nxt in closed or not passable(nxt):
                continue
            new_g = g_cost[current] + 1
            if new_g < g_cost.get(nxt, float("inf")):
                g_cost[nxt] = new_g
                parent[nxt] = current
                counter += 1
                heapq.heappush(open_heap, (new_g + manhattan(nxt, goal), counter, nxt))
    return []


def inflate_obstacles(grid, radius_cells):
    """Copy of grid where every cell within radius_cells (Euclidean) of an
    OCCUPIED cell is also OCCUPIED. Use radius = (ROBOT_RADIUS + SAFETY_MARGIN) / resolution.
    """
    rows, cols = len(grid), len(grid[0])
    out = [list(r) for r in grid]
    rad = int(radius_cells)
    offsets = [(dr, dc) for dr in range(-rad, rad + 1) for dc in range(-rad, rad + 1)
               if dr * dr + dc * dc <= radius_cells * radius_cells]
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == OCCUPIED:
                for dr, dc in offsets:
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < rows and 0 <= cc < cols:
                        out[rr][cc] = OCCUPIED
    return out


def find_frontiers(grid):
    """Frontier cell = FREE cell with at least one 4-neighbour that is UNKNOWN.

    Returns a list of (row, col) in row-major order ([] if none).
    """
    frontiers = []
    for r, row in enumerate(grid):
        for c, value in enumerate(row):
            if value != FREE:
                continue
            for dr, dc in NEIGHBORS_4:
                rr, cc = r + dr, c + dc
                if _in_bounds(grid, rr, cc) and grid[rr][cc] == UNKNOWN:
                    frontiers.append((r, c))
                    break
    return frontiers


def cluster_frontiers(frontiers, min_size=1):
    """Group frontier cells into 8-connected clusters.

    Returns a list of clusters (each a list of (row, col)), largest first.
    Clusters smaller than min_size are dropped.
    """
    remaining = set(frontiers)
    clusters = []
    while remaining:
        seed = remaining.pop()
        cluster, queue = [seed], deque([seed])
        while queue:
            r, c = queue.popleft()
            for dr, dc in NEIGHBORS_8:
                n = (r + dr, c + dc)
                if n in remaining:
                    remaining.remove(n)
                    cluster.append(n)
                    queue.append(n)
        if len(cluster) >= min_size:
            clusters.append(sorted(cluster))
    clusters.sort(key=len, reverse=True)
    return clusters


def cluster_centroid(cluster):
    """Mean (row, col) of a cluster as floats."""
    n = len(cluster)
    return (sum(c[0] for c in cluster) / n, sum(c[1] for c in cluster) / n)
