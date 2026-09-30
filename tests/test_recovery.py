import unittest

import _path  # noqa: F401
from recovery import RecoveryLadder, reachable_cells


class RecoveryTests(unittest.TestCase):
    def make(self):
        return RecoveryLadder(replan_delay=1, blacklist_duration=10, blacklist_radius=0.15)

    def test_bounded_replans_then_neighbourhood_blacklist(self):
        recovery = self.make()
        recovery.begin((2, 3))
        recovery.fail(0)
        self.assertEqual((recovery.stage, recovery.ready_at), ("WAIT_REPLAN", 1))
        recovery.planned()
        recovery.fail(2)
        self.assertEqual(recovery.attempts, 2)
        recovery.planned()
        recovery.fail(4)
        self.assertIsNone(recovery.goal)
        self.assertEqual(recovery.stage, "CHOOSE_GOAL")
        self.assertTrue(recovery.excluded((2.1, 3), 5))
        self.assertFalse(recovery.excluded((2.2, 3), 5))
        self.assertFalse(recovery.excluded((2, 3), 14))

    def test_reset_cancels_goal_but_preserves_blacklist(self):
        recovery = self.make()
        recovery.exclude((2, 3), 0)
        recovery.begin((4, 5))
        recovery.fail(0)
        recovery.reset()
        self.assertIsNone(recovery.goal)
        self.assertEqual(recovery.attempts, 0)
        self.assertTrue(recovery.excluded((2, 3), 1))

    def test_new_goal_has_own_retry_budget(self):
        recovery = self.make()
        recovery.begin((2, 3))
        recovery.fail(0)
        recovery.begin((4, 5))
        self.assertEqual(recovery.attempts, 0)

    def test_invalid_settings(self):
        with self.assertRaises(ValueError):
            RecoveryLadder(replan_delay=0, blacklist_duration=10, blacklist_radius=0.1)

    def test_reachable_cells_reject_unknown_obstacles_and_diagonal_gaps(self):
        grid = [[0, 1, 0], [-1, 0, 0], [0, 0, 0]]
        self.assertEqual(reachable_cells(grid, (0, 0)), {(0, 0)})
        self.assertEqual(reachable_cells(grid, (0, 1)), set())
        self.assertEqual(reachable_cells(grid, (-1, 0)), set())
        self.assertEqual(reachable_cells([], (0, 0)), set())
        self.assertEqual(len(reachable_cells(grid, (2, 2))), 6)
