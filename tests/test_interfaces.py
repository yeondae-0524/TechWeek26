import math
import unittest

import _path  # noqa: F401
import config
import detection
import interfaces
import mapping
from interfaces import FREE, OCCUPIED, UNKNOWN


class TestInterfaces(unittest.TestCase):
    def test_grid_values(self):
        self.assertEqual((UNKNOWN, FREE, OCCUPIED), (-1, 0, 1))
        self.assertEqual(config.GRID_RESOLUTION, 0.05)

    def test_grid_indexing_is_row_col(self):
        g = mapping.OccupancyGrid(5, 3, 0.05, (0.0, 0.0))  # 5 cols, 3 rows
        self.assertEqual((len(g.grid), len(g.grid[0])), (3, 5))
        g.set(2, 4, OCCUPIED)
        self.assertEqual(g.grid[2][4], OCCUPIED)

    def test_detection_default(self):
        t = detection.detect_target(None)
        self.assertEqual(t, {"found": False, "cx": None, "direction": None, "area": 0.0})
        self.assertIsInstance(t["area"], float)
        self.assertTrue(interfaces.is_valid_target(t))
        t["found"] = True  # every call must return a fresh dict
        self.assertFalse(detection.detect_target(None)["found"])

    def test_target_validator(self):
        ok = {"found": True, "cx": 25, "direction": "CENTER", "area": 30.0}
        self.assertTrue(interfaces.is_valid_target(ok))
        self.assertFalse(interfaces.is_valid_target({"found": True, "cx": 25, "direction": "UP", "area": 1.0}))
        self.assertFalse(interfaces.is_valid_target({"found": False}))

    def test_pose_format(self):
        p = interfaces.make_pose(1, 2, 3 * math.pi)
        self.assertTrue(interfaces.is_valid_pose(p))
        self.assertAlmostEqual(p[2], math.pi)
        self.assertAlmostEqual(interfaces.normalize_angle(-math.pi), math.pi)
        self.assertAlmostEqual(interfaces.normalize_angle(2 * math.pi + 0.1), 0.1)
        self.assertFalse(interfaces.is_valid_pose((0.0, 0.0)))
        self.assertTrue(interfaces.is_valid_pose(interfaces.make_pose(*config.START_POSE)))

    def test_path_and_waypoint_format(self):
        self.assertTrue(interfaces.is_valid_path([(0, 0), (0, 1)]))
        self.assertTrue(interfaces.is_valid_path([]))
        self.assertFalse(interfaces.is_valid_path([[0, 0]]))
        self.assertTrue(interfaces.is_valid_waypoint((0.5, -1.0)))
        self.assertFalse(interfaces.is_valid_waypoint((0.5,)))

    def test_required_device_names_configured(self):
        for key in config.REQUIRED_DEVICES:
            self.assertTrue(config.DEVICE_NAMES.get(key))


if __name__ == "__main__":
    unittest.main()
