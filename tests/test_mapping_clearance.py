import math
import unittest

import _path  # noqa: F401
import numpy as np
from mapping import OccupancyGrid
from interfaces import FREE, OCCUPIED, UNKNOWN


class MappingClearanceTests(unittest.TestCase):
    def test_physical_radius_with_cell_uncertainty_not_square_extent(self):
        grid = OccupancyGrid(21, 21, 0.05, (0, 0))
        grid.grid = [[FREE] * 21 for _ in range(21)]
        grid.grid[10][10] = OCCUPIED
        grid.grid[10][11] = UNKNOWN
        before = np.asarray(grid.grid).copy()
        safe = grid.clearance_grid(0.161)
        for row in range(21):
            for col in range(21):
                expected = math.hypot(row - 10, col - 10) * 0.05 <= 0.161 + 0.05 / math.sqrt(2)
                self.assertEqual(safe[row][col] == OCCUPIED, expected)
        np.testing.assert_array_equal(grid.grid, before)
        self.assertEqual(safe[10][14], FREE)

    def test_boundary_obstacle_does_not_wrap_and_unknown_is_preserved(self):
        grid = OccupancyGrid(10, 10, 0.05, (0, 0))
        grid.grid[0][0] = OCCUPIED
        safe = grid.clearance_grid(0.05)
        self.assertEqual(safe[1][1], OCCUPIED)
        self.assertEqual(safe[9][9], UNKNOWN)
        self.assertEqual(safe[0][9], UNKNOWN)

    def test_zero_and_invalid_radius(self):
        grid = OccupancyGrid(3, 3, 0.05, (0, 0))
        grid.grid[1][1] = OCCUPIED
        self.assertEqual(grid.clearance_grid(0), grid.grid)
        for radius in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                grid.clearance_grid(radius)
