import math
import unittest

import _path  # noqa: F401
import config
import mapping
from interfaces import FREE, OCCUPIED, UNKNOWN


class TestGridConversion(unittest.TestCase):
    def setUp(self):
        # 10 cols x 8 rows, 0.05 m cells, cell (0,0) lower-left corner at (-0.25, -0.2)
        self.g = mapping.OccupancyGrid(10, 8, 0.05, (-0.25, -0.2))

    def test_initial_unknown(self):
        self.assertEqual(len(self.g.grid), 8)       # rows
        self.assertEqual(len(self.g.grid[0]), 10)   # cols
        self.assertEqual(self.g.count(UNKNOWN), 80)

    def test_world_to_grid(self):
        self.assertEqual(self.g.world_to_grid(-0.25, -0.2), (0, 0))       # origin corner
        self.assertEqual(self.g.world_to_grid(-0.2001, -0.1501), (0, 0))  # still in cell 0,0
        self.assertEqual(self.g.world_to_grid(-0.19, -0.2), (0, 1))       # +x -> col+1
        self.assertEqual(self.g.world_to_grid(-0.25, -0.14), (1, 0))      # +y -> row+1
        self.assertEqual(self.g.world_to_grid(0.01, 0.01), (4, 5))

    def test_grid_to_world_is_cell_center(self):
        x, y = self.g.grid_to_world(0, 0)
        self.assertAlmostEqual(x, -0.225)
        self.assertAlmostEqual(y, -0.175)
        x, y = self.g.grid_to_world(4, 5)
        self.assertAlmostEqual(x, 0.025)
        self.assertAlmostEqual(y, 0.025)

    def test_round_trip(self):
        for r in range(8):
            for c in range(10):
                self.assertEqual(self.g.world_to_grid(*self.g.grid_to_world(r, c)), (r, c))

    def test_bounds(self):
        self.assertTrue(self.g.in_bounds(0, 0))
        self.assertTrue(self.g.in_bounds(7, 9))
        self.assertFalse(self.g.in_bounds(8, 0))
        self.assertFalse(self.g.in_bounds(0, 10))
        self.assertFalse(self.g.in_bounds(-1, 0))
        self.assertFalse(self.g.in_bounds(*self.g.world_to_grid(-0.26, 0.0)))  # left of origin
        self.g.set(100, 100, OCCUPIED)  # out-of-bounds write is ignored
        self.assertEqual(self.g.count(OCCUPIED), 0)

    def test_centered_on(self):
        g = mapping.OccupancyGrid.centered_on(1.0, 2.0, 20, 20, 0.05)
        self.assertEqual(g.world_to_grid(1.01, 2.01), (10, 10))


class TestLidarHelpers(unittest.TestCase):
    def test_polar_to_local(self):
        x, y = mapping.polar_to_local(1.0, 0.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        x, y = mapping.polar_to_local(2.0, math.pi / 2)  # left of robot
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 2.0)

    def test_local_to_world(self):
        # robot at (1, 1) facing +y: a point 1 m in front is at (1, 2)
        wx, wy = mapping.local_to_world((1.0, 1.0, math.pi / 2), 1.0, 0.0)
        self.assertAlmostEqual(wx, 1.0)
        self.assertAlmostEqual(wy, 2.0)

    def test_lidar_angle_convention(self):
        # 360 rays over 2*pi: index N/2 points forward, N/4 left, 3N/4 right
        n, fov = 360, 2 * math.pi

        def wrap(a):
            return math.atan2(math.sin(a), math.cos(a))
        self.assertAlmostEqual(wrap(mapping.lidar_angle(180, n, fov)), 0.0)
        self.assertAlmostEqual(wrap(mapping.lidar_angle(90, n, fov)), math.pi / 2)
        self.assertAlmostEqual(wrap(mapping.lidar_angle(270, n, fov)), -math.pi / 2)

    def test_insert_scan_marks_free_and_occupied(self):
        g = mapping.OccupancyGrid.centered_on(0.0, 0.0, 40, 40, 0.05)
        ranges = [0.52] * 360  # circular wall at 0.52 m -> nothing observed beyond it
        g.insert_scan((0.0, 0.0, 0.0), ranges, 2 * math.pi, 1.0)
        # rays start at the LiDAR origin (TurtleBot3: 0.03 m behind the axle)
        self.assertEqual(g.get(*g.world_to_grid(0.52 + config.LIDAR_MOUNT_OFFSET[0],
                                                config.LIDAR_MOUNT_OFFSET[1])), OCCUPIED)
        self.assertEqual(g.get(*g.world_to_grid(0.25, 0.0)), FREE)
        self.assertEqual(g.get(*g.world_to_grid(0.0, 0.0)), FREE)
        self.assertEqual(g.get(*g.world_to_grid(0.7, 0.0)), UNKNOWN)  # behind the hit

    def test_bresenham_endpoints(self):
        cells = mapping.bresenham((0, 0), (3, 5))
        self.assertEqual(cells[0], (0, 0))
        self.assertEqual(cells[-1], (3, 5))
        self.assertEqual(len(cells), 6)


if __name__ == "__main__":
    unittest.main()
