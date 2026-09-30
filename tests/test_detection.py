"""detection.py on synthetic 640x480 frames drawn with the real camera geometry (no Webots)."""

import math
import os
import tempfile
import unittest

import _path  # noqa: F401
import config
import detection
from interfaces import is_valid_target

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

W, H = 640, 480
F = (W / 2) / math.tan(config.CAMERA_HFOV / 2)   # ~554.3 px
RED = (0, 0, 230)            # BGR
DARK_RED = (0, 0, 150)       # shaded side
GREEN = (0, 102, 0)
GREEN_RANGES = [((35, 80, 40), (85, 255, 255))]


def blank():
    return np.full((H, W, 3), (128, 128, 128), np.uint8)


def draw_apple(frame, depth, lateral=0.0, height=0.05, color=RED, shaded=True):
    """Apple (diameter TARGET_SIZE) at depth [m] ahead, lateral [m] left, centre height [m]."""
    cx = int(round(W / 2 - F * lateral / depth))
    cy = int(round(H / 2 + F * (config.CAMERA_HEIGHT - height) / depth))
    r = int(round(F * (config.TARGET_SIZE / 2) / depth))
    cv2.circle(frame, (cx, cy), r, color, -1)
    if shaded:  # darker lower-right part, like a lit sphere
        cv2.ellipse(frame, (cx, cy), (r, r), 0, 0, 90, DARK_RED, -1)
    return frame


@unittest.skipIf(cv2 is None, "OpenCV/NumPy not installed")
class TestSingleFrame(unittest.TestCase):
    def test_none_and_empty(self):
        self.assertEqual(detection.detect_target(None),
                         {"found": False, "cx": None, "direction": None, "area": 0.0})
        self.assertFalse(detection.detect_target(blank())["found"])

    def test_apple_straight_ahead(self):
        target, blobs = detection.detect(draw_apple(blank(), 1.0))
        self.assertTrue(target["found"])
        self.assertEqual(target["direction"], "CENTER")
        self.assertTrue(is_valid_target(target))
        self.assertAlmostEqual(blobs[0]["range"], 1.0, delta=0.1)
        self.assertAlmostEqual(blobs[0]["bearing"], 0.0, delta=math.radians(2))

    def test_left_right_and_distance(self):
        for lateral, direction in ((0.6, "LEFT"), (-0.6, "RIGHT")):
            with self.subTest(lateral=lateral):
                target, blobs = detection.detect(draw_apple(blank(), 2.0, lateral))
                self.assertEqual(target["direction"], direction)
                self.assertAlmostEqual(blobs[0]["depth"], 2.0, delta=0.2)
                self.assertAlmostEqual(blobs[0]["lateral"], lateral, delta=0.1)

    def test_no_floor_assumption_by_default(self):
        # apple on a 0.8 m table, 3 m ahead: must still be a candidate
        self.assertIsNone(config.TARGET_HEIGHT_RANGE)
        target, blobs = detection.detect(draw_apple(blank(), 3.0, height=0.83))
        self.assertTrue(target["found"])
        self.assertAlmostEqual(blobs[0]["center_height"], 0.83, delta=0.15)

    def test_optional_height_window(self):
        old, config.TARGET_HEIGHT_RANGE = config.TARGET_HEIGHT_RANGE, (0.0, 0.2)
        try:
            self.assertFalse(detection.detect_target(draw_apple(blank(), 3.0, height=0.83))["found"])
            self.assertTrue(detection.detect_target(draw_apple(blank(), 1.5))["found"])
        finally:
            config.TARGET_HEIGHT_RANGE = old

    def test_red_book_rejected(self):
        frame = blank()
        cv2.rectangle(frame, (250, 150), (290, 330), RED, -1)   # tall red rectangle
        valid, rejected, _ = detection.find_blobs(frame)
        self.assertEqual(valid, [])
        self.assertIn(rejected[0][1], ("aspect", "not round"))

    def test_cut_off_at_border_rejected(self):
        frame = blank()
        cv2.circle(frame, (10, 300), 40, RED, -1)      # apple cut by the left edge
        valid, rejected, _ = detection.find_blobs(frame)
        self.assertEqual(valid, [])
        self.assertEqual(rejected[0][1], "border")
        frame = blank()
        cv2.circle(frame, (60, 300), 40, RED, -1)      # fully visible near the edge
        self.assertTrue(detection.detect_target(frame)["found"])

    def test_red_panel_rejected_next_to_apple(self):
        # breakroom_teleop_yolo: red cabinet panel (wide rectangle) behind the apple
        frame = draw_apple(blank(), 0.5, 0.1)
        cv2.rectangle(frame, (330, 150), (470, 190), (0, 0, 200), -1)
        valid, rejected, _ = detection.find_blobs(frame)
        self.assertEqual(len(valid), 1)
        self.assertTrue(any(r in ("aspect", "not round") for _, r in rejected))

    def test_square_red_door_rejected_by_corners(self):
        # breakroom_teleop_yolo: a square red cabinet door passes fill (0.64) and aspect (1.0)
        for pts in ([[300, 200], [380, 200], [380, 280], [300, 280]],        # square
                    [[300, 200], [380, 190], [385, 280], [298, 270]],        # door in perspective
                    [[320, 180], [380, 240], [320, 300], [260, 240]]):       # rotated square
            frame = blank()
            cv2.fillPoly(frame, [np.array(pts)], RED)
            valid, rejected, _ = detection.find_blobs(frame)
            self.assertEqual(valid, [])
            self.assertEqual(rejected[0][1], "corners")

    def test_apple_with_stem_is_not_rejected_by_corners(self):
        for depth in (0.4, 1.0, 2.5):
            with self.subTest(depth=depth):
                frame = draw_apple(blank(), depth)
                r = int(round(F * config.TARGET_SIZE / 2 / depth))
                cy = int(round(H / 2 + F * (config.CAMERA_HEIGHT - 0.05) / depth))
                cv2.line(frame, (320, cy - r), (322, cy - r - max(2, r // 3)), RED, max(1, r // 12))
                target, blobs = detection.detect(frame)
                self.assertTrue(target["found"])
                self.assertGreater(blobs[0]["vertices"], config.TARGET_MAX_CORNERS)

    def test_too_far_rejected(self):
        valid, rejected, _ = detection.find_blobs(draw_apple(blank(), 6.0, shaded=False))
        self.assertEqual(valid, [])

    def test_two_apples(self):
        frame = draw_apple(draw_apple(blank(), 1.0, 0.3), 2.0, -0.5)
        target, blobs = detection.detect(frame)
        self.assertEqual(len(blobs), 2)
        self.assertEqual(target["direction"], "LEFT")   # largest (closest) first

    def test_colour_selection(self):
        green = draw_apple(blank(), 1.0, color=GREEN, shaded=False)
        self.assertFalse(detection.detect_target(green)["found"])                      # default = red
        self.assertTrue(detection.detect_target(green, hsv_ranges=GREEN_RANGES)["found"])
        # hue wrap-around: two ranges on both ends of 0..179 (independent of tuned config values)
        wrap = draw_apple(blank(), 1.0, color=(40, 0, 230), shaded=False)              # hue ~175
        wrap_ranges = [((0, 100, 60), (10, 255, 255)), ((170, 100, 60), (179, 255, 255))]
        self.assertTrue(detection.detect_target(wrap, hsv_ranges=wrap_ranges)["found"])

    def test_direction_borders(self):
        self.assertEqual(detection.direction_of(213, W), "LEFT")
        self.assertEqual(detection.direction_of(214, W), "CENTER")
        self.assertEqual(detection.direction_of(426, W), "CENTER")
        self.assertEqual(detection.direction_of(427, W), "RIGHT")

    def test_webots_style_bgra_view(self):
        bgra = np.full((H, W, 4), 255, np.uint8)
        bgra[:, :, :3] = draw_apple(blank(), 1.0)
        view = np.frombuffer(bgra.tobytes(), np.uint8).reshape((H, W, 4))[:, :, :3]
        self.assertFalse(view.flags["C_CONTIGUOUS"])
        self.assertTrue(detection.detect_target(view)["found"])

    def test_returns_new_dict(self):
        frame = draw_apple(blank(), 1.0)
        self.assertIsNot(detection.detect_target(frame), detection.detect_target(frame))

    def test_frame_dump(self):
        with tempfile.TemporaryDirectory() as folder:
            old, detection.FRAME_DUMP_DIR = detection.FRAME_DUMP_DIR, folder
            try:
                detection.detect(draw_apple(blank(), 1.0))
            finally:
                detection.FRAME_DUMP_DIR = old
            for name in ("frame.png", "mask.png", "debug.png"):
                self.assertTrue(os.path.isfile(os.path.join(folder, name)), name)


def blob(depth, lateral=0.0):
    return {"depth": depth, "lateral": lateral, "range": math.hypot(depth, lateral)}


class TestTargetTracker(unittest.TestCase):
    def tracker(self):
        return detection.TargetTracker(window=5, min_hits=3, dedup_radius=0.3,
                                       range_error=0.15, max_spread=0.1, required=2)

    def test_blob_to_world(self):
        x, y = detection.blob_to_world(blob(1.0), (1.0, 2.0, math.pi / 2))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0 + config.CAMERA_OFFSET[0] + 1.0)

    def test_three_of_five_confirms(self):
        t = self.tracker()
        seen = [True, False, True, False, True]
        confirmed = []
        for k, hit in enumerate(seen):
            confirmed += t.update(k * 0.128, (0, 0, 0), [blob(1.0)] if hit else [])
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(len(t.confirmed()), 1)

    def test_two_of_five_not_confirmed_and_dropped(self):
        t = self.tracker()
        for k, hit in enumerate([True, True, False, False, False, False, False]):
            t.update(k * 0.128, (0, 0, 0), [blob(1.0)] if hit else [])
        self.assertEqual(t.confirmed(), [])
        self.assertEqual(len(t.tracks), 1)   # kept while briefly unseen (occlusion)
        t.update(0.128 + config.TRACK_FORGET_S + 0.1, (0, 0, 0), [])
        self.assertEqual(t.tracks, [])       # forgotten after TRACK_FORGET_S

    def test_candidate_survives_short_occlusion(self):
        t = self.tracker()
        t.update(0.0, (0, 0, 0), [blob(1.0)])
        t.update(0.128, (0, 0, 0), [blob(1.0)])
        for k in range(2, 12):                       # ~1.3 s hidden (person walks by)
            t.update(k * 0.128, (0, 0, 0), [])
        t.update(12 * 0.128, (0, 0, 0), [blob(1.0)])
        self.assertEqual(len(t.tracks), 1)
        t.update(13 * 0.128, (0, 0, 0), [blob(1.0)])
        t.update(14 * 0.128, (0, 0, 0), [blob(1.0)])
        self.assertEqual(len(t.confirmed()), 1)

    def test_relative_to_and_arrival(self):
        bearing, dist = detection.relative_to((0.0, 0.0, math.pi / 2), (-1.0, 1.0))
        self.assertAlmostEqual(bearing, math.pi / 4)     # target is 45 deg to the left
        self.assertAlmostEqual(dist, math.sqrt(2))
        self.assertTrue(detection.is_arrived((0.0, 0.0, 0.0), (0.2, 0.0), distance=0.3))
        self.assertFalse(detection.is_arrived((0.0, 0.0, 0.0), (0.5, 0.0), distance=0.3))
        t = self.tracker()
        for k in range(3):
            t.update(k * 0.128, (0, 0, 0), [blob(1.0)])
        track = t.confirmed()[0]
        self.assertIs(t.get(track["id"]), track)
        self.assertIsNone(t.get(999))

    def test_same_apple_from_moving_robot_is_one_target(self):
        t = self.tracker()
        # apple at world (2.02, 0): robot drives forward 0.5 m while looking at it
        for k, x in enumerate([0.0, 0.1, 0.2, 0.3, 0.4, 0.5]):
            t.update(k * 0.128, (x, 0.0, 0.0), [blob(2.0 - x)])
        self.assertEqual(len(t.tracks), 1)
        self.assertAlmostEqual(t.tracks[0]["xy"][0], 2.02, delta=0.01)

    def test_two_apples_two_targets_then_done(self):
        t = self.tracker()
        for k in range(5):
            t.update(k * 0.128, (0, 0, 0), [blob(1.0, 0.5), blob(2.0, -0.5)])
        self.assertEqual(len(t.confirmed()), 2)
        self.assertFalse(t.all_visited())
        first = t.nearest_unvisited((0, 0, 0))
        t.mark_visited(first["id"])
        self.assertEqual(t.visited_count(), 1)
        self.assertNotEqual(t.nearest_unvisited((0, 0, 0))["id"], first["id"])
        t.mark_visited(t.nearest_unvisited((0, 0, 0))["id"])
        self.assertTrue(t.all_visited())
        self.assertIsNone(t.nearest_unvisited((0, 0, 0)))

    def test_visited_target_seen_again_is_not_new(self):
        t = self.tracker()
        for k in range(3):
            t.update(k * 0.128, (0, 0, 0), [blob(1.0)])
        t.mark_visited(t.confirmed()[0]["id"])
        for k in range(3, 8):
            t.update(k * 0.128, (0, 0, 0), [blob(1.05)])
        self.assertEqual(len(t.confirmed()), 1)
        self.assertEqual(t.unvisited(), [])

    def test_scattered_positions_not_confirmed(self):
        # range scaling off: tolerance = max_spread 0.1 m. Detections jump by 0.28 m
        # (inside the 0.3 m association radius) -> position std ~0.14 m -> not confirmed
        t = detection.TargetTracker(window=5, min_hits=3, dedup_radius=0.3,
                                    range_error=0.0, max_spread=0.1, required=2)
        for k, lat in enumerate([0.0, 0.28, 0.0, 0.28, 0.0]):
            t.update(k * 0.128, (0, 0, 0), [blob(1.0, lat)])
        self.assertEqual(t.confirmed(), [])


if __name__ == "__main__":
    unittest.main()
