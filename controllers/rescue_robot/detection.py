"""Target detection: one camera frame -> target dict (single-frame, OpenCV HSV).

Input:
    frame : NumPy array (H, W, 3), BGR, uint8 - from devices.read_camera_frame()
            or None if no camera image is available.

Output (see interfaces.empty_target / AGENTS.md "팀 규격"):
    {
        "found": bool,
        "cx": int | None,          # target centre column in pixels
        "direction": "LEFT" | "CENTER" | "RIGHT" | None,
        "area": float,             # pixel area, 0.0 if not found
    }
    direction rule: split the image width in three equal parts.

Pipeline (same family as the official tb3_segmentation example and
docs/research/07_TARGET_SEARCH.md §2):
    BGR -> blur -> HSV -> inRange (config.TARGET_HSV_RANGES) -> opening
        -> largest external contour -> area >= TARGET_MIN_AREA -> centroid cx

NOT implemented (TODO, feat/detection):
    * multi-frame confirmation (M-of-N), bearing / world position, dedup
      (TargetTracker, research 07 §4-§6)
    * final colour / shape rules - the target appearance is announced on the day
"""

import os

import config
from interfaces import empty_target

try:  # OpenCV is optional at import time so the controller never crashes without it
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - depends on the local Python install
    cv2 = None
    np = None

_warned = False

# Debug: RESCUE_FRAME_DUMP=<folder> saves the latest frame.png and mask.png at
# every detection call (overwritten), to tune TARGET_HSV_RANGES on real frames.
FRAME_DUMP_DIR = os.environ.get("RESCUE_FRAME_DUMP")


def direction_of(cx, width):
    """LEFT / CENTER / RIGHT by splitting the image width in three equal parts."""
    if cx < width / 3.0:
        return "LEFT"
    if cx < 2.0 * width / 3.0:
        return "CENTER"
    return "RIGHT"


def target_mask(frame, hsv_ranges=None):
    """Binary mask (uint8, 0/255) of pixels whose HSV value is inside any range."""
    hsv_ranges = config.TARGET_HSV_RANGES if hsv_ranges is None else hsv_ranges
    image = np.ascontiguousarray(frame)  # Webots frames are a BGRA view -> make contiguous
    k = config.DETECTION_BLUR_KERNEL
    if k:
        image = cv2.GaussianBlur(image, (k, k), 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lower, upper in hsv_ranges:
        mask |= cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    k = config.DETECTION_MORPH_KERNEL
    if k:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
    return mask


def detect_target(frame, hsv_ranges=None, min_area=None):
    """Largest blob of the target colour in one frame. Always returns a NEW dict."""
    global _warned
    if frame is None:
        return empty_target()
    if cv2 is None:
        if not _warned:
            print("[detection] WARNING: OpenCV/NumPy not installed -> detection disabled")
            _warned = True
        return empty_target()
    min_area = config.TARGET_MIN_AREA if min_area is None else min_area

    mask = target_mask(frame, hsv_ranges)
    if FRAME_DUMP_DIR:
        os.makedirs(FRAME_DUMP_DIR, exist_ok=True)
        cv2.imwrite(os.path.join(FRAME_DUMP_DIR, "frame.png"), np.ascontiguousarray(frame))
        cv2.imwrite(os.path.join(FRAME_DUMP_DIR, "mask.png"), mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return empty_target()
    largest = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(largest))
    moments = cv2.moments(largest)
    if area < min_area or moments["m00"] == 0:
        return empty_target()

    cx = int(round(moments["m10"] / moments["m00"]))
    return {"found": True, "cx": cx, "direction": direction_of(cx, frame.shape[1]), "area": area}
