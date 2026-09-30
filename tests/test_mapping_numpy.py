"""NumPy mapping regression checks without Webots or ground-truth inputs."""
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import _path  # noqa: F401
import config
import mapping
import planning
from interfaces import FREE, OCCUPIED, UNKNOWN


class ScalarReference(mapping.OccupancyGrid):
    """Pre-migration scalar update, used to compare complete scan sequences."""

    def __init__(self, *args):
        super().__init__(*args)
        self.logodds = self.logodds.tolist()
        self.observed = self.observed.tolist()

    def _update_cells(self, cells, delta):
        for row, col in set(cells):
            if not self.in_bounds(row, col):
                continue
            self.logodds[row][col] = min(
                self.LOG_ODDS_MAX,
                max(self.LOG_ODDS_MIN, self.logodds[row][col] + delta))
            self.observed[row][col] = True
            self._export_cell(row, col)


class TestNumpyMapping(unittest.TestCase):
    def make_grid(self):
        return mapping.OccupancyGrid(80, 80, 0.05, (-2.0, -2.0))

    def test_scan_sequence_matches_scalar_reference(self):
        actual = self.make_grid()
        reference = ScalarReference(80, 80, 0.05, (-2.0, -2.0))
        rng = np.random.default_rng(7)
        for step in range(24):
            ranges = rng.uniform(0.12, 3.49, 360)
            ranges[::17] = np.inf
            ranges[::29] = np.nan
            ranges[::31] = 0.05
            ranges[::37] = 3.5
            pose = (0.04 * step, -0.3, step * 0.13)
            for grid in (actual, reference):
                grid.insert_scan(pose, ranges, 2 * math.pi, 3.5,
                                 min_range=0.12, mark_free=step % 4 != 0)
                if step == 12:
                    grid.reset_region((0.7, -0.2), 0.4)
            np.testing.assert_array_equal(actual.logodds, reference.logodds)
            np.testing.assert_array_equal(actual.observed, reference.observed)
            self.assertEqual(actual.grid, reference.grid)

    def test_batch_clamp_hysteresis_and_bounds(self):
        grid = self.make_grid()
        cells = [(20, 20), (20, 20), (-1, 0), (80, 0)]
        for _ in range(5):
            grid._update_cells(cells, grid.LOG_ODDS_HIT)
        self.assertEqual(grid.logodds[20, 20], 3.5)
        for _ in range(9):
            grid._update_cells(cells, grid.LOG_ODDS_MISS)
        self.assertEqual(grid.get(20, 20), OCCUPIED)
        grid._update_cells(cells, grid.LOG_ODDS_MISS)
        self.assertEqual(grid.get(20, 20), FREE)
        for _ in range(10):
            grid._update_cells(cells, grid.LOG_ODDS_MISS)
        self.assertEqual(grid.logodds[20, 20], -2.0)
        self.assertEqual(int(np.count_nonzero(grid.observed)), 1)

    def test_export_stays_list_and_planner_can_use_it(self):
        grid = self.make_grid()
        exported, row = grid.grid, grid.grid[40]
        scan = [float('inf')] * 360
        scan[180] = 1.0
        grid.insert_scan((0, 0.025, 0), scan, 2 * math.pi, 3.5, min_range=0.12)
        self.assertIs(grid.grid, exported)
        self.assertIs(grid.grid[40], row)
        self.assertIsInstance(grid.logodds, np.ndarray)
        self.assertIsInstance(grid.observed, np.ndarray)
        start, goal = grid.world_to_grid(0.2, 0.025), grid.world_to_grid(0.6, 0.025)
        path = planning.astar(grid.grid, start, goal, allow_unknown=False)
        self.assertTrue(path)
        self.assertEqual((path[0], path[-1]), (start, goal))
        self.assertTrue(all(type(grid.get(*cell)) is int for cell in path))

    def test_known_poses_project_same_landmark_to_same_cell(self):
        # Analytic pose input checks transforms, NOT real odometry accuracy.
        grid = self.make_grid()
        target = (1.025, 0.025)
        cell = grid.world_to_grid(*target)
        for pose in ((0.03, 0.025, 0), (0.23, 0.025, 0),
                     (1.025, -0.97, math.pi / 2)):
            origin = mapping.local_to_world(pose, *config.LIDAR_MOUNT_OFFSET)
            scan = [float('inf')] * 360
            scan[180] = math.hypot(target[0] - origin[0], target[1] - origin[1])
            grid.insert_scan(pose, scan, 2 * math.pi, 3.5, min_range=0.12)
            self.assertEqual(grid.get(*cell), OCCUPIED)
        self.assertEqual(grid.count(OCCUPIED), 1)
        self.assertAlmostEqual(grid.logodds[cell], 3 * grid.LOG_ODDS_HIT)

    def test_pgm_orientation_and_reset_export(self):
        grid = mapping.OccupancyGrid(2, 2, 0.05, (0, 0))
        grid._update_cells([(1, 0)], grid.LOG_ODDS_HIT)
        grid._update_cells([(0, 1)], grid.LOG_ODDS_MISS)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'map.pgm'
            grid.save_pgm(path)
            self.assertEqual(path.read_text().splitlines(),
                             ['P2', '2 2', '255', '0 128', '128 255'])
        grid.reset_region(grid.grid_to_world(1, 0), 0.01)
        self.assertEqual(grid.get(1, 0), UNKNOWN)
        self.assertEqual(grid.logodds[1, 0], 0.0)
        self.assertFalse(grid.observed[1, 0])
        self.assertEqual(grid.get(0, 1), FREE)


if __name__ == '__main__':
    unittest.main()
