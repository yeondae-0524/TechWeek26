"""Historical 2-D camera visibility, separate from LiDAR occupancy.

This records possible image coverage, not successful object detection. Unknown
cells and walls occlude the camera; a missing image must never mark coverage.
"""
import math

import numpy as np

from interfaces import FREE, OCCUPIED
from mapping import local_to_world


def _visible_endpoints(clear, x, y, end_rows, end_cols):
    """Batched supercover traversal; touching a blocked corner occludes a ray."""
    cols = np.full(len(end_rows), math.floor(x), dtype=int)
    rows = np.full(len(end_rows), math.floor(y), dtype=int)
    dx, dy = end_cols + 0.5 - x, end_rows + 0.5 - y
    sx, sy = np.sign(dx).astype(int), np.sign(dy).astype(int)
    step_x, step_y = np.full(len(dx), np.inf), np.full(len(dy), np.inf)
    np.divide(1, np.abs(dx), out=step_x, where=dx != 0)
    np.divide(1, np.abs(dy), out=step_y, where=dy != 0)
    tx, ty = np.full(len(dx), np.inf), np.full(len(dy), np.inf)
    np.divide(np.where(dx > 0, cols + 1 - x, x - cols), np.abs(dx), out=tx, where=dx != 0)
    np.divide(np.where(dy > 0, rows + 1 - y, y - rows), np.abs(dy), out=ty, where=dy != 0)
    visible = clear[rows, cols].copy()
    while True:
        active = np.flatnonzero(visible & ((rows != end_rows) | (cols != end_cols)))
        if not len(active):
            return visible
        corner = active[np.isclose(tx[active], ty[active], rtol=0, atol=1e-12)]
        visible[corner] &= (clear[rows[corner], cols[corner] + sx[corner]]
                            & clear[rows[corner] + sy[corner], cols[corner]])
        move_x = active[tx[active] < ty[active]]
        move_y = active[ty[active] < tx[active]]
        # Corner rays step along both axes, including near-equal roundoff cases.
        move_x = np.union1d(move_x, corner)
        move_y = np.union1d(move_y, corner)
        cols[move_x] += sx[move_x]
        rows[move_y] += sy[move_y]
        tx[move_x] += step_x[move_x]
        ty[move_y] += step_y[move_y]
        visible[active] &= clear[rows[active], cols[active]]


class CameraCoverageGrid:
    def __init__(self, grid, *, hfov, max_range, camera_offset, footprint_radius):
        if not (0 < hfov <= 2 * math.pi and math.isfinite(max_range) and max_range > 0):
            raise ValueError("invalid camera field of view or range")
        if not math.isfinite(footprint_radius) or footprint_radius < 0:
            raise ValueError("invalid footprint radius")
        if len(camera_offset) != 2 or not all(math.isfinite(v) for v in camera_offset):
            raise ValueError("invalid camera offset")
        self.grid = grid
        self.hfov, self.max_range = hfov, max_range
        self.camera_offset, self.footprint_radius = camera_offset, footprint_radius
        self.seen = np.zeros((grid.height, grid.width), dtype=bool)

    def update(self, pose, *, frame_valid):
        """Mark known free cell centres visible in a successfully acquired frame."""
        if not frame_valid or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            return 0
        grid = self.grid
        x, y = local_to_world(pose, *self.camera_offset)
        start_row, start_col = grid.world_to_grid(x, y)
        if not grid.in_bounds(start_row, start_col):
            return 0
        radius = math.ceil(self.max_range / grid.resolution)
        r0, r1 = max(0, start_row - radius), min(grid.height, start_row + radius + 1)
        c0, c1 = max(0, start_col - radius), min(grid.width, start_col + radius + 1)
        rows, cols = np.mgrid[r0:r1, c0:c1]
        dx = grid.origin[0] + (cols + 0.5) * grid.resolution - x
        dy = grid.origin[1] + (rows + 0.5) * grid.resolution - y
        angle = np.arctan2(np.sin(np.arctan2(dy, dx) - pose[2]),
                           np.cos(np.arctan2(dy, dx) - pose[2]))
        cells = np.asarray([row[c0:c1] for row in grid.grid[r0:r1]])
        candidates = ((cells == FREE) & ~self.seen[r0:r1, c0:c1]
                      & (dx * dx + dy * dy <= self.max_range ** 2)
                      & (np.abs(angle) <= self.hfov / 2))
        gx = (x - grid.origin[0]) / grid.resolution - c0
        gy = (y - grid.origin[1]) / grid.resolution - r0
        # LiDAR cannot see under the robot. Only unknown footprint cells may
        # be crossed; occupied cells always occlude, including under the robot.
        under_robot = ((dx + x - pose[0]) ** 2 + (dy + y - pose[1]) ** 2
                       <= self.footprint_radius ** 2)
        clear = (cells == FREE) | ((cells != OCCUPIED) & under_robot)
        target_rows, target_cols = rows[candidates], cols[candidates]
        visible = _visible_endpoints(clear, gx, gy, target_rows - r0, target_cols - c0)
        self.seen[target_rows[visible], target_cols[visible]] = True
        return int(np.count_nonzero(visible))

    def free_fraction(self):
        free = np.asarray(self.grid.grid) == FREE
        total = int(np.count_nonzero(free))
        return float(np.count_nonzero(self.seen & free) / total) if total else 0.0
