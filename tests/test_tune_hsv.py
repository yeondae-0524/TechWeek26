"""scripts/tune_hsv.py helpers (the slider window itself needs a display)."""

import os
import sys
import unittest

import _path  # noqa: F401
import config

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import tune_hsv  # noqa: E402


class TestSliders(unittest.TestCase):
    def test_single_range(self):
        ranges = tune_hsv.ranges_from_sliders(35, 85, 80, 255, 40, 255)
        self.assertEqual(ranges, [((35, 80, 40), (85, 255, 255))])
        self.assertEqual(tune_hsv.sliders_from_ranges(ranges), (35, 85, 80, 255, 40, 255))

    def test_red_wraps_into_two_ranges(self):
        ranges = tune_hsv.ranges_from_sliders(170, 10, 120, 255, 70, 255)
        self.assertEqual(ranges, [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))])
        self.assertEqual(tune_hsv.sliders_from_ranges(ranges), (170, 10, 120, 255, 70, 255))

    def test_config_round_trip(self):
        values = tune_hsv.sliders_from_ranges(config.TARGET_HSV_RANGES)
        self.assertEqual(tune_hsv.ranges_from_sliders(*values), [tuple(map(tuple, r)) for r in config.TARGET_HSV_RANGES])

    def test_config_line_is_python(self):
        line = tune_hsv.config_line([((0, 120, 70), (10, 255, 255))])
        scope = {}
        exec(line, scope)
        self.assertEqual(scope["TARGET_HSV_RANGES"], [((0, 120, 70), (10, 255, 255))])


@unittest.skipIf(tune_hsv.cv2 is None, "OpenCV/NumPy not installed")
class TestPixel(unittest.TestCase):
    def test_hsv_at(self):
        import numpy as np
        image = np.zeros((20, 20, 3), np.uint8)
        image[:, :] = (0, 0, 255)   # pure red BGR
        self.assertEqual(tune_hsv.hsv_at(image, 10, 10), (0, 255, 255))


if __name__ == "__main__":
    unittest.main()
