"""detection.detect_target on synthetic 640x480 frames (no Webots)."""

import unittest

import _path  # noqa: F401
import detection
from interfaces import is_valid_target

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

W, H = 640, 480
GREEN_BGR = (0, 102, 0)      # Webots green ball colour 0 0.4 0 (RGB) -> BGR
RED_BGR = (0, 0, 255)
RED_RANGES = [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))]


def frame_with_disc(cx, cy, radius, bgr, background=(128, 128, 128)):
    frame = np.full((H, W, 3), background, np.uint8)
    cv2.circle(frame, (cx, cy), radius, bgr, -1)
    return frame


@unittest.skipIf(cv2 is None, "OpenCV/NumPy not installed")
class TestDetectTarget(unittest.TestCase):
    def test_none_frame(self):
        self.assertEqual(detection.detect_target(None), {"found": False, "cx": None, "direction": None, "area": 0.0})

    def test_empty_scene_not_found(self):
        target = detection.detect_target(np.full((H, W, 3), 128, np.uint8))
        self.assertFalse(target["found"])
        self.assertTrue(is_valid_target(target))

    def test_center_left_right(self):
        for cx, expected in ((320, "CENTER"), (80, "LEFT"), (560, "RIGHT")):
            with self.subTest(cx=cx):
                target = detection.detect_target(frame_with_disc(cx, 300, 30, GREEN_BGR))
                self.assertTrue(target["found"])
                self.assertEqual(target["direction"], expected)
                self.assertAlmostEqual(target["cx"], cx, delta=2)
                self.assertAlmostEqual(target["area"], 3.1416 * 30 ** 2, delta=150)
                self.assertTrue(is_valid_target(target))

    def test_direction_borders(self):
        self.assertEqual(detection.direction_of(213, W), "LEFT")    # 213 < 640/3
        self.assertEqual(detection.direction_of(214, W), "CENTER")
        self.assertEqual(detection.direction_of(426, W), "CENTER")  # 426 < 2*640/3
        self.assertEqual(detection.direction_of(427, W), "RIGHT")

    def test_small_blob_ignored(self):
        self.assertFalse(detection.detect_target(frame_with_disc(320, 300, 3, GREEN_BGR))["found"])

    def test_largest_blob_wins(self):
        frame = frame_with_disc(100, 300, 15, GREEN_BGR)
        cv2.circle(frame, (500, 300), 40, GREEN_BGR, -1)
        self.assertEqual(detection.detect_target(frame)["direction"], "RIGHT")

    def test_other_colour_ignored_and_red_ranges(self):
        frame = frame_with_disc(320, 300, 30, RED_BGR)
        self.assertFalse(detection.detect_target(frame)["found"])            # default = green
        self.assertTrue(detection.detect_target(frame, hsv_ranges=RED_RANGES)["found"])

    def test_webots_style_bgra_view(self):
        # devices.read_camera_frame(): np.frombuffer(...).reshape((h, w, 4))[:, :, :3]
        bgra = np.full((H, W, 4), 255, np.uint8)
        bgra[:, :, :3] = frame_with_disc(320, 300, 30, GREEN_BGR)
        view = np.frombuffer(bgra.tobytes(), np.uint8).reshape((H, W, 4))[:, :, :3]
        self.assertFalse(view.flags["C_CONTIGUOUS"])
        self.assertTrue(detection.detect_target(view)["found"])

    def test_frame_dump(self):
        import os
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            old, detection.FRAME_DUMP_DIR = detection.FRAME_DUMP_DIR, folder
            try:
                detection.detect_target(frame_with_disc(320, 300, 30, GREEN_BGR))
            finally:
                detection.FRAME_DUMP_DIR = old
            self.assertTrue(os.path.isfile(os.path.join(folder, "frame.png")))
            self.assertTrue(os.path.isfile(os.path.join(folder, "mask.png")))

    def test_returns_new_dict(self):
        frame = frame_with_disc(320, 300, 30, GREEN_BGR)
        self.assertIsNot(detection.detect_target(frame), detection.detect_target(frame))


if __name__ == "__main__":
    unittest.main()
