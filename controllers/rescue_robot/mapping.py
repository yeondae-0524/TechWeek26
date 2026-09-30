"""Occupancy grid baseline.

Implemented:
    * grid creation (UNKNOWN / FREE / OCCUPIED), grid[row][col]
    * world_to_grid() / grid_to_world() / in_bounds()
    * LiDAR polar -> robot-local Cartesian -> world helpers
    * minimal scan insertion: cells along each ray -> FREE, hit cell -> OCCUPIED
      (pure pose-based, trusts the current pose estimate)

NOT implemented (TODO, feat/mapping):
    * probabilistic update (log-odds) - currently the last observation wins
    * scan matching / SLAM (pose correction from the map)
    * dynamic obstacle (moving people) filtering
"""

import math

import config
from interfaces import FREE, OCCUPIED, UNKNOWN


class OccupancyGrid:
    """grid[row][col] with row along +y and col along +x (see docs/INTERFACES.md)."""

    def __init__(self, width, height, resolution, origin):
        self.width = int(width)        # columns
        self.height = int(height)      # rows
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))  # world (x, y) of cell (0,0) lower-left corner
        self.grid = [[UNKNOWN] * self.width for _ in range(self.height)]

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

    # ------------------------------------------------------- scan insertion
    def insert_scan(self, pose, ranges, lidar_fov, max_range, mark_free=True):
        """Mark one LiDAR scan into the grid using ``pose`` (x, y, theta).

        Rays that return inf/over max_range only clear free space up to max_range.
        TODO(feat/mapping): replace "last write wins" with log-odds.
        """
        start = self.world_to_grid(pose[0], pose[1])
        for (wx, wy), hit in scan_to_world_points(pose, ranges, lidar_fov, max_range):
            end = self.world_to_grid(wx, wy)
            ray = bresenham(start, end)
            if mark_free:
                for cell in (ray[:-1] if hit else ray):
                    self._mark_free(*cell)
            if hit:
                self.set(end[0], end[1], OCCUPIED)

    def _mark_free(self, row, col):
        # An observed obstacle is never erased by a later free ray (TODO: log-odds).
        if self.in_bounds(row, col) and self.grid[row][col] != OCCUPIED:
            self.grid[row][col] = FREE

    def count(self, value):
        return sum(row.count(value) for row in self.grid)

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
