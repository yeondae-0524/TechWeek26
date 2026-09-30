"""Scan insertion rules for LDS-01 (docs/research/03 §A, 09 §4, 10 §11.1)."""

import math
import unittest

import _path  # noqa: F401
import config
import mapping
from interfaces import FREE, OCCUPIED, UNKNOWN

INF = float("inf")
FOV = 2 * math.pi
FRONT = 180  # index of the forward ray (360 rays)


def single_ray(r, index=FRONT, n=360):
    ranges = [INF] * n
    ranges[index] = r
    return ranges


class TestScanInsertion(unittest.TestCase):
    def setUp(self):
        self.g = mapping.OccupancyGrid.centered_on(0.0, 0.0, 80, 80, 0.05)
        self.ox = config.LIDAR_MOUNT_OFFSET[0]

    def cell(self, x, y=0.0):
        return self.g.get(*self.g.world_to_grid(x, y))

    def test_inf_rays_do_not_create_free_space(self):
        self.g.insert_scan((0.0, 0.0, 0.0), [INF] * 360, FOV, 3.5, min_range=0.12)
        self.assertEqual(self.g.count(FREE), 0)
        self.assertEqual(self.g.count(OCCUPIED), 0)

    def test_nan_and_out_of_range_rays_skipped(self):
        ranges = single_ray(float("nan"))
        ranges[90] = 5.0  # beyond maxRange
        ranges[270] = 0.05  # below minRange (Webots would report inf; still ignored)
        self.g.insert_scan((0.0, 0.0, 0.0), ranges, FOV, 3.5, min_range=0.12)
        self.assertEqual(self.g.count(FREE) + self.g.count(OCCUPIED), 0)

    def test_hit_is_measured_from_lidar_origin(self):
        self.g.insert_scan((0.0, 0.0, 0.0), single_ray(1.0), FOV, 3.5)
        self.assertEqual(self.cell(1.0 + self.ox), OCCUPIED)
        self.assertEqual(self.cell(0.5), FREE)

    def test_min_range_cells_not_cleared(self):
        self.g.insert_scan((0.0, 0.0, 0.0), single_ray(1.0), FOV, 3.5, min_range=0.12)
        self.assertEqual(self.cell(self.ox + 0.02), UNKNOWN)   # inside the blind zone
        self.assertEqual(self.cell(self.ox + 0.30), FREE)      # beyond minRange
        self.assertEqual(self.cell(1.0 + self.ox), OCCUPIED)

    def test_old_obstacle_cleared_by_later_free_ray(self):
        # A person stood at 0.5 m, then left: repeated free rays clear the ghost.
        self.g.insert_scan((0.0, 0.0, 0.0), single_ray(0.5), FOV, 3.5)
        self.assertEqual(self.cell(0.5 + self.ox), OCCUPIED)
        for _ in range(3):
            self.g.insert_scan((0.0, 0.0, 0.0), single_ray(1.5), FOV, 3.5)
        self.assertEqual(self.cell(0.5 + self.ox), FREE)
        self.assertEqual(self.cell(1.5 + self.ox), OCCUPIED)

    def test_hit_wins_over_miss_in_same_scan(self):
        # Two neighbouring rays: one ends in a cell the other one passes through.
        n = 360
        ranges = [INF] * n
        ranges[FRONT] = 0.5
        ranges[FRONT + 1] = 1.0  # 1 deg to the right, passes through the 0.5 m cell
        pose = (0.0, 0.025, 0.0)  # both rays stay in the same grid row
        end = self.g.world_to_grid(0.5 + self.ox, 0.025)
        self.assertIn(end, mapping.bresenham(self.g.world_to_grid(self.ox, 0.025),
                                             self.g.world_to_grid(1.0 + self.ox, 0.025)))
        self.g.insert_scan(pose, ranges, FOV, 3.5)
        self.assertEqual(self.g.get(*end), OCCUPIED)

    def test_inf_keeps_previous_knowledge(self):
        self.g.insert_scan((0.0, 0.0, 0.0), single_ray(0.5), FOV, 3.5)
        self.g.insert_scan((0.0, 0.0, 0.0), [INF] * 360, FOV, 3.5, min_range=0.12)
        self.assertEqual(self.cell(0.5 + self.ox), OCCUPIED)


if __name__ == "__main__":
    unittest.main()
