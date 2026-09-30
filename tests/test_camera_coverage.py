import math
import unittest

import _path  # noqa: F401
import numpy as np
from camera_coverage import CameraCoverageGrid, _visible_endpoints
from mapping import OccupancyGrid
from interfaces import FREE, OCCUPIED, UNKNOWN


class CameraCoverageTests(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid(15, 15, 1.0, (-7.5, -7.5))
        self.grid.grid = [[FREE] * 15 for _ in range(15)]
        self.coverage = CameraCoverageGrid(self.grid, hfov=math.pi / 3, max_range=5,
                                           camera_offset=(0, 0), footprint_radius=0)

    def seen(self, x, y):
        return self.coverage.seen[self.grid.world_to_grid(x, y)]

    def test_fov_range_and_heading(self):
        self.coverage.update((0, 0, 0), frame_valid=True)
        self.assertTrue(self.seen(4, 1))
        self.assertFalse(self.seen(-2, 0))
        self.assertFalse(self.seen(1, 2))
        self.assertFalse(self.seen(6, 0))
        self.coverage.update((0, 0, math.pi / 2), frame_valid=True)
        self.assertTrue(self.seen(0, 4))

    def test_wall_and_unknown_occlude(self):
        for blocker in (OCCUPIED, UNKNOWN):
            with self.subTest(blocker=blocker):
                self.coverage.seen[:] = False
                self.grid.grid[7][9] = blocker
                self.coverage.update((0, 0, 0), frame_valid=True)
                self.assertTrue(self.seen(1, 0))
                self.assertFalse(self.seen(3, 0))
                self.assertFalse(self.seen(2, 0))

    def test_no_corner_leak(self):
        self.grid.grid[7][8] = OCCUPIED
        self.coverage.update((0, 0, math.pi / 4), frame_valid=True)
        self.assertFalse(self.seen(2, 2))

    def test_no_frame_invalid_pose_and_outside_map_do_not_mark(self):
        self.assertEqual(self.coverage.update((0, 0, 0), frame_valid=False), 0)
        self.assertEqual(self.coverage.update((math.nan, 0, 0), frame_valid=True), 0)
        self.assertEqual(self.coverage.update((100, 0, 0), frame_valid=True), 0)
        self.assertFalse(self.coverage.seen.any())

    def test_does_not_modify_occupancy_and_repeated_view_is_idempotent(self):
        before = np.asarray(self.grid.grid).copy()
        first = self.coverage.update((0, 0, 0), frame_valid=True)
        self.assertGreater(first, 0)
        self.assertEqual(self.coverage.update((0, 0, 0), frame_valid=True), 0)
        np.testing.assert_array_equal(before, self.grid.grid)
        self.assertAlmostEqual(self.coverage.free_fraction(), first / 225)

    def test_camera_offset_and_nonzero_origin(self):
        self.coverage.camera_offset = (2, 0)
        self.coverage.update((0, 0, math.pi / 2), frame_valid=True)
        self.assertTrue(self.seen(0, 6))
        self.assertFalse(self.seen(0, 1))

    def test_only_unknown_inside_robot_footprint_can_be_crossed(self):
        self.coverage.footprint_radius = 0.6
        self.grid.grid[7][7] = UNKNOWN
        self.coverage.update((0, 0, 0), frame_valid=True)
        self.assertTrue(self.seen(2, 0))
        self.coverage.seen[:] = False
        self.grid.grid[7][7] = OCCUPIED
        self.coverage.update((0, 0, 0), frame_valid=True)
        self.assertFalse(self.seen(2, 0))

    def test_empty_free_map_fraction(self):
        self.grid.grid = [[UNKNOWN] * 15 for _ in range(15)]
        self.assertEqual(self.coverage.free_fraction(), 0)

    def test_batched_visibility_matches_segment_rectangle_intersections(self):
        # Independent geometric oracle: intersect each sight line with every
        # blocked cell rectangle, rather than duplicating grid traversal.
        rng = np.random.default_rng(7)
        rows, cols = np.indices((11, 12))
        er, ec = rows.ravel(), cols.ravel()
        for x, y in ((5.2, 4.3), (0.2, 0.7), (10.8, 9.4)):
            clear = rng.random((11, 12)) > 0.12
            clear[math.floor(y), math.floor(x)] = True
            expected = []
            blockers = np.argwhere(~clear)
            for row, col in zip(er, ec):
                visible = True
                for rr, cc in blockers:
                    low, high = 0.0, 1.0
                    for origin, delta, edge in ((x, col + 0.5 - x, cc),
                                                (y, row + 0.5 - y, rr)):
                        a, b = sorted(((edge - origin) / delta,
                                       (edge + 1 - origin) / delta))
                        low, high = max(low, a), min(high, b)
                    # A tangent is blocked, including rounding at a cell corner.
                    if low <= high + 1e-12:
                        visible = False
                        break
                expected.append(visible)
            np.testing.assert_array_equal(_visible_endpoints(clear, x, y, er, ec), expected)
