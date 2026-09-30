import unittest

import _path  # noqa: F401
from interfaces import FREE, OCCUPIED, UNKNOWN
from planning import cluster_centroid, cluster_frontiers, find_frontiers


class TestFrontier(unittest.TestCase):
    def test_free_unknown_boundary(self):
        # left part FREE, right part UNKNOWN -> column 2 is the frontier
        g = [[FREE, FREE, FREE, UNKNOWN, UNKNOWN] for _ in range(4)]
        self.assertEqual(find_frontiers(g), [(r, 2) for r in range(4)])

    def test_occupied_is_never_frontier(self):
        self.assertEqual(find_frontiers([[FREE, OCCUPIED, UNKNOWN]]), [])

    def test_no_frontier_all_free(self):
        self.assertEqual(find_frontiers([[FREE] * 4 for _ in range(4)]), [])

    def test_no_frontier_all_unknown(self):
        self.assertEqual(find_frontiers([[UNKNOWN] * 4 for _ in range(4)]), [])

    def test_diagonal_unknown_is_not_frontier(self):
        g = [[FREE, OCCUPIED], [OCCUPIED, UNKNOWN]]
        self.assertEqual(find_frontiers(g), [])  # only 4-neighbours count

    def test_clusters(self):
        frontiers = [(0, 0), (1, 1), (2, 2), (0, 5), (0, 6)]
        clusters = cluster_frontiers(frontiers)
        self.assertEqual(len(clusters), 2)
        self.assertEqual(clusters[0], [(0, 0), (1, 1), (2, 2)])  # largest first, 8-connected
        self.assertEqual(clusters[1], [(0, 5), (0, 6)])
        self.assertEqual(cluster_frontiers(frontiers, min_size=3), [clusters[0]])
        self.assertEqual(cluster_centroid(clusters[1]), (0.0, 5.5))
        self.assertEqual(cluster_frontiers([]), [])


if __name__ == "__main__":
    unittest.main()
