"""Log-odds occupancy grid mapping using the supplied pose estimate.

Implemented:
    * grid creation (UNKNOWN / FREE / OCCUPIED), grid[row][col]
    * world_to_grid() / grid_to_world() / in_bounds()
    * LiDAR polar -> robot-local Cartesian -> world helpers (LiDAR mount offset)
    * log-odds scan insertion with clamping (docs/research/03 §A):
      rays start at the LiDAR origin, only finite hits inside
      [min_range, max_range) are used, cells inside min_range are not cleared,
      each cell is updated once per scan and a hit wins over a miss.
    * hysteresis thresholds export log-odds as UNKNOWN / FREE / OCCUPIED
    * repeated valid free rays can clear old OCCUPIED cells; occluded cells
      and cells with only invalid/no-return measurements are not cleared
    * reset_region() resets log-odds to zero, observation flags to False,
      and grid cells to UNKNOWN

Log-odds and observation flags use NumPy arrays, updated in batches per scan.
The exported grid remains a Python list of lists for existing planner callers.
Pose estimation is external; this module does not correct odometry or
distinguish moving people from static obstacles.

NOT implemented (TODO, feat/mapping):
    * scan matching / SLAM (pose correction from the map)
    * dynamic obstacle (moving people) filtering
"""

import math

import numpy as np

import config
from interfaces import FREE, OCCUPIED, UNKNOWN


class OccupancyGrid:
    """grid[row][col] with row along +y and col along +x (see interfaces.py)."""

    LOG_ODDS_HIT = 0.85
    LOG_ODDS_MISS = -0.40
    LOG_ODDS_MIN = -2.0
    LOG_ODDS_MAX = 3.5
    LOG_ODDS_OCCUPIED = 0.4
    LOG_ODDS_FREE = -0.2

    def __init__(self, width, height, resolution, origin):
        self.width = int(width)        # columns
        self.height = int(height)      # rows
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))  # world (x, y) of cell (0,0) lower-left corner
        self.grid = [[UNKNOWN] * self.width for _ in range(self.height)]
        self.logodds = np.zeros((self.height, self.width), dtype=np.float64)
        self.observed = np.zeros((self.height, self.width), dtype=np.bool_)

    @classmethod
    def centered_on(cls, x, y, width, height, resolution):
        origin = (x - width * resolution / 2.0, y - height * resolution / 2.0)
        return cls(width, height, resolution, origin)

    # ------------------------------------------------------- conversions
    def world_to_grid(self, x, y):
        """World (x, y) [m] -> (row, col). May be out of bounds; check in_bounds()."""
        col = math.floor((x - self.origin[0]) / self.resolution)
        row = math.floor((y - self.origin[1]) / self.resolution)
        return (row, col)

    def grid_to_world(self, row, col):
        """(row, col) -> world (x, y) of the cell CENTRE."""
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return (x, y)

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def get(self, row, col):
        return self.grid[row][col]

    def set(self, row, col, value):
        if self.in_bounds(row, col):
            self.grid[row][col] = value

    def _update_cell(self, row, col, delta):
        self._update_cells(((row, col),), delta)

    def _update_cells(self, cells, delta):
        """Batch unique, in-bounds cells; preserve the exported list object.

        insert_scan supplies disjoint hit/miss sets. float64 preserves the
        previous Python-float threshold behaviour (including hysteresis).
        """
        valid = sorted({(r, c) for r, c in cells if self.in_bounds(r, c)})
        if not valid:
            return
        indices = np.asarray(valid, dtype=np.intp)
        rows, cols = indices[:, 0], indices[:, 1]
        values = np.clip(self.logodds[rows, cols] + delta,
                         self.LOG_ODDS_MIN, self.LOG_ODDS_MAX)
        self.logodds[rows, cols] = values
        self.observed[rows, cols] = True
        # Only export changed cells; converting the whole map each scan would
        # copy unobserved cells and invalidate references held by other modules.
        for row, col in indices[values > self.LOG_ODDS_OCCUPIED].tolist():
            self.grid[row][col] = OCCUPIED
        for row, col in indices[values < self.LOG_ODDS_FREE].tolist():
            self.grid[row][col] = FREE

    def _export_cell(self, row, col):
        value = self.logodds[row][col]
        if not self.observed[row][col]:
            self.grid[row][col] = UNKNOWN
        elif value > self.LOG_ODDS_OCCUPIED:
            self.grid[row][col] = OCCUPIED
        elif value < self.LOG_ODDS_FREE:
            self.grid[row][col] = FREE

    def reset_region(self, center, radius):
        """Reset cells within ``radius`` metres of (x, y) to UNKNOWN."""
        cx, cy = center
        row_min, col_min = self.world_to_grid(cx - radius, cy - radius)
        row_max, col_max = self.world_to_grid(cx + radius, cy + radius)
        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                if not self.in_bounds(row, col):
                    continue
                x, y = self.grid_to_world(row, col)
                if math.hypot(x - cx, y - cy) <= radius:
                    self.logodds[row][col] = 0.0
                    self.observed[row][col] = False
                    self.grid[row][col] = UNKNOWN

    # ------------------------------------------------------- scan insertion
    def insert_scan(self, pose, ranges, lidar_fov, max_range, mark_free=True, min_range=0.0):
        """Mark one LiDAR scan into the grid using ``pose`` (x, y, theta).

        * Rays start at the LiDAR origin (config.LIDAR_MOUNT_OFFSET).
        * inf / NaN / out-of-range rays are skipped: "no return" cannot be told
          apart from an object inside the minRange blind zone, so it never
          creates FREE space (research 09 §4, 10 §11.1).
        * Cells closer than ``min_range`` to the LiDAR are not cleared.
        * Each cell is updated once per scan; a hit in the same scan wins.
        """
        origin = local_to_world(pose, *config.LIDAR_MOUNT_OFFSET)
        start = self.world_to_grid(*origin)
        n = len(ranges)
        hits, misses = set(), set()
        for i, r in enumerate(ranges):
            if not is_valid_hit(r, min_range, max_range):
                continue
            angle = lidar_angle(i, n, lidar_fov)
            end = self.world_to_grid(*local_to_world(pose, *_sensor_point(r, angle)))
            hits.add(end)
            if mark_free:
                ray = bresenham(start, end)[:-1]
                if min_range > 0.0:
                    # Cells in the minRange blind zone stay as they are.
                    ray = [c for c in ray if self._distance_to(c, origin) >= min_range]
                misses.update(ray)
        self._update_cells(misses - hits, self.LOG_ODDS_MISS)
        self._update_cells(hits, self.LOG_ODDS_HIT)

    def _distance_to(self, cell, point):
        x, y = self.grid_to_world(*cell)
        return math.hypot(x - point[0], y - point[1])

    def count(self, value):
        return sum(row.count(value) for row in self.grid)

    def clearance_grid(self, radius):
        """Copy occupancy with a hard circular clearance band around obstacles.

        Include the occupied cell's half diagonal, so discretisation never
        understates the supplied physical clearance. Unknown stays unknown
        unless it lies inside that band. The stored map is never changed.
        """
        if not math.isfinite(radius) or radius < 0:
            raise ValueError("clearance radius must be finite and nonnegative")
        cells = np.asarray(self.grid)
        occupied = cells == OCCUPIED
        blocked = occupied.copy()
        cell_radius = radius / self.resolution + math.sqrt(2) / 2
        extent = math.ceil(cell_radius)
        for dr in range(-min(extent, self.height - 1), min(extent, self.height - 1) + 1):
            for dc in range(-min(extent, self.width - 1), min(extent, self.width - 1) + 1):
                if dr * dr + dc * dc > cell_radius * cell_radius:
                    continue
                r0, r1 = max(0, dr), min(self.height, self.height + dr)
                c0, c1 = max(0, dc), min(self.width, self.width + dc)
                blocked[r0:r1, c0:c1] |= occupied[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
        return np.where(blocked, OCCUPIED, cells).tolist()

    def save_pgm(self, path):
        """Debug dump: black = OCCUPIED, white = FREE, grey = UNKNOWN. Top row = max y."""
        shade = {UNKNOWN: 128, FREE: 255, OCCUPIED: 0}
        with open(path, "w", encoding="ascii") as f:
            f.write(f"P2\n{self.width} {self.height}\n255\n")
            for row in reversed(self.grid):
                f.write(" ".join(str(shade[v]) for v in row) + "\n")


# ---------------------------------------------------------------------------
# LiDAR helpers
# ---------------------------------------------------------------------------
def lidar_angle(index, n_rays, fov):
    """Angle of ray ``index`` in the robot frame (0 = forward, + = left/CCW).

    Convention from config (LIDAR_FIRST_ANGLE, LIDAR_ANGLE_DIRECTION).
    """
    return config.LIDAR_FIRST_ANGLE + config.LIDAR_ANGLE_DIRECTION * index * fov / n_rays


def polar_to_local(r, angle):
    """Polar (range, angle) in the robot frame -> local Cartesian (x fwd, y left)."""
    return (r * math.cos(angle), r * math.sin(angle))


def local_to_world(pose, lx, ly):
    """Robot-local point -> world point using pose (x, y, theta)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return (x + c * lx - s * ly, y + s * lx + c * ly)


def is_valid_hit(r, min_range, max_range):
    """True for a finite LiDAR return inside [min_range, max_range)."""
    return math.isfinite(r) and min_range <= r < max_range


def _sensor_point(r, angle):
    """Point at range r along a ray, in the robot frame (LiDAR mount applied)."""
    lx, ly = polar_to_local(r, angle)
    return (lx + config.LIDAR_MOUNT_OFFSET[0], ly + config.LIDAR_MOUNT_OFFSET[1])


def scan_to_world_points(pose, ranges, fov, max_range):
    """Return [((wx, wy), hit), ...]; hit=False when the ray saw nothing (clipped to max_range)."""
    n = len(ranges)
    ox, oy = config.LIDAR_MOUNT_OFFSET
    out = []
    for i, r in enumerate(ranges):
        hit = math.isfinite(r) and r < max_range
        rr = r if hit else max_range
        lx, ly = polar_to_local(rr, lidar_angle(i, n, fov))
        out.append((local_to_world(pose, lx + ox, ly + oy), hit))
    return out


def bresenham(start, end):
    """Integer cells on the line from start (row, col) to end, both inclusive."""
    (r0, c0), (r1, c1) = start, end
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dc - dr
    cells = []
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if (r, c) == (r1, c1):
            return cells
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
