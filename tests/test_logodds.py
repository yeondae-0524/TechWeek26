import math
import unittest

import _path  # noqa: F401
import mapping
from interfaces import FREE, OCCUPIED, UNKNOWN


FOV = 2 * math.pi
FRONT = 180
INF = float("inf")


def single_ray(r):
    ranges = [INF] * 360
    ranges[FRONT] = r
    return ranges


class TestLogOddsMapping(unittest.TestCase):
    def setUp(self):
        self.grid = mapping.OccupancyGrid.centered_on(0.0, 0.0, 80, 80, 0.05)
        self.hit_cell = self.grid.world_to_grid(0.97, 0.0)

    def test_one_scan_updates_each_cell_once(self):
        self.grid.insert_scan((0.0, 0.0, 0.0), single_ray(1.0), FOV, 3.5)
        row, col = self.hit_cell
        self.assertTrue(self.grid.observed[row][col])
        self.assertEqual(self.grid.grid[row][col], OCCUPIED)
        self.assertAlmostEqual(self.grid.logodds[row][col], 0.85)

    def test_saturated_obstacle_clears_after_ten_misses(self):
        ranges = single_ray(1.0)
        for _ in range(5):
            self.grid.insert_scan((0.0, 0.0, 0.0), ranges, FOV, 3.5)
        row, col = self.hit_cell
        self.assertEqual(self.grid.logodds[row][col], 3.5)

        for _ in range(9):
            self.grid.insert_scan((0.0, 0.0, 0.0), single_ray(1.5), FOV, 3.5)
        self.assertEqual(self.grid.grid[row][col], OCCUPIED)
        self.grid.insert_scan((0.0, 0.0, 0.0), single_ray(1.5), FOV, 3.5)
        self.assertEqual(self.grid.grid[row][col], FREE)

    def test_unobserved_cells_stay_unknown(self):
        self.grid.insert_scan((0.0, 0.0, 0.0), [INF] * 360, FOV, 3.5)
        self.assertEqual(self.grid.count(UNKNOWN), 80 * 80)

    def test_reset_region_restores_unknown(self):
        self.grid.insert_scan((0.0, 0.0, 0.0), single_ray(1.0), FOV, 3.5)
        self.grid.reset_region((0.97, 0.0), 0.05)
        row, col = self.hit_cell
        self.assertFalse(self.grid.observed[row][col])
        self.assertEqual(self.grid.logodds[row][col], 0.0)
        self.assertEqual(self.grid.grid[row][col], UNKNOWN)


if __name__ == "__main__":
    unittest.main()
