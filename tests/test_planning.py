import unittest

import _path  # noqa: F401
from interfaces import FREE, OCCUPIED, UNKNOWN
from planning import astar, inflate_obstacles, manhattan


def empty(rows, cols, value=FREE):
    return [[value] * cols for _ in range(rows)]


class TestAStar(unittest.TestCase):
    def assert_valid_path(self, grid, path, start, goal):
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            self.assertEqual(abs(r0 - r1) + abs(c0 - c1), 1)  # 4-neighbour steps
        for r, c in path:
            self.assertTrue(0 <= r < len(grid) and 0 <= c < len(grid[0]))
            self.assertNotEqual(grid[r][c], OCCUPIED)

    def test_empty_map_optimal(self):
        g = empty(5, 7)
        path = astar(g, (0, 0), (4, 6))
        self.assert_valid_path(g, path, (0, 0), (4, 6))
        self.assertEqual(len(path), manhattan((0, 0), (4, 6)) + 1)

    def test_detour_around_wall(self):
        g = empty(5, 5)
        for r in range(4):
            g[r][2] = OCCUPIED  # wall with a gap in the last row
        path = astar(g, (0, 0), (0, 4))
        self.assert_valid_path(g, path, (0, 0), (0, 4))
        self.assertIn((4, 2), path)
        self.assertEqual(len(path), 13)  # optimal detour

    def test_no_path(self):
        g = empty(5, 5)
        for r in range(5):
            g[r][2] = OCCUPIED
        self.assertEqual(astar(g, (0, 0), (0, 4)), [])

    def test_start_equals_goal(self):
        self.assertEqual(astar(empty(3, 3), (1, 1), (1, 1)), [(1, 1)])

    def test_blocked_or_outside_endpoints(self):
        g = empty(3, 3)
        g[2][2] = OCCUPIED
        self.assertEqual(astar(g, (0, 0), (2, 2)), [])   # goal occupied
        self.assertEqual(astar(g, (0, 0), (5, 5)), [])   # goal outside
        self.assertEqual(astar(g, (-1, 0), (1, 1)), [])  # start outside

    def test_unknown_policy(self):
        g = empty(3, 3, UNKNOWN)
        g[0][0] = FREE
        g[0][2] = FREE
        self.assertEqual(len(astar(g, (0, 0), (0, 2))), 3)
        self.assertEqual(astar(g, (0, 0), (0, 2), allow_unknown=False), [])

    def test_inflate(self):
        g = empty(7, 7)
        g[3][3] = OCCUPIED
        out = inflate_obstacles(g, 1)
        self.assertEqual(g[3][4], FREE)  # input untouched
        for r, c in ((2, 3), (4, 3), (3, 2), (3, 4)):
            self.assertEqual(out[r][c], OCCUPIED)
        self.assertEqual(out[2][2], FREE)  # diagonal distance sqrt(2) > 1


if __name__ == "__main__":
    unittest.main()
