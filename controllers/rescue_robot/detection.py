"""Target detection: red apples (mission: find 2, then return home). Location unknown.

Single frame  -> ``detect(frame)`` returns (target dict, list of Blob)
    BGR -> blur -> HSV -> inRange (config.TARGET_HSV_RANGES) -> opening
        -> external contours -> per-blob geometry -> filters -> valid blobs
    Filters (docs/research/07_TARGET_SEARCH.md §4-§5):
        * area >= TARGET_MIN_AREA
        * round: area / enclosing-circle area >= TARGET_MIN_FILL, aspect in range,
          no corners (approxPolyDP vertices > TARGET_MAX_CORNERS: rejects squares)
        * size-based distance (pinhole, TARGET_SIZE) <= TARGET_MAX_RANGE
        * not touching the image border (TARGET_BORDER_MARGIN): cut-off objects
          have an unreliable shape
        * optional height window TARGET_HEIGHT_RANGE (OFF by default: where the
          apples lie is unknown; only enable it if the organizers confirm it)
    ``detect_target(frame)`` returns only the shared target dict:
        {"found": bool, "cx": int | None,
         "direction": "LEFT" | "CENTER" | "RIGHT" | None, "area": float}
    (largest valid blob; direction = image width split in three).

Multiple frames -> ``TargetTracker``
    Converts valid blobs to world (x, y) with the robot pose, associates them to
    tracks (de-duplication), confirms a track when it was seen in >= M of the
    last N detection cycles with a small position spread, and keeps a visited
    flag so a target is counted once.

Input frame: NumPy (H, W, 3) BGR uint8 from devices.read_camera_frame(), or None.

Helpers for Control / integration: relative_to(pose, xy) -> (bearing, distance),
is_arrived(pose, xy), TargetTracker.get(id) / mark_visited(id).

NOT implemented (TODO): LiDAR range fusion (an apple lower than the 0.173 m LiDAR
plane is not seen by the LiDAR, so size-based range is used), occlusion memory /
re-search.
"""

import math
import os
from collections import deque

import config
from interfaces import empty_target

try:  # OpenCV is optional at import time so the controller never crashes without it
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - depends on the local Python install
    cv2 = None
    np = None

_warned = False

# Debug: RESCUE_FRAME_DUMP=<folder> writes frame.png (camera), mask.png (colour
# threshold) and debug.png (blobs: green = accepted, red = rejected + reason) at
# every detection call (overwritten). Use with scripts/tune_hsv.py.
FRAME_DUMP_DIR = os.environ.get("RESCUE_FRAME_DUMP")


# ---------------------------------------------------------------------------
# Single frame
# ---------------------------------------------------------------------------
def direction_of(cx, width):
    """LEFT / CENTER / RIGHT by splitting the image width in three equal parts."""
    if cx < width / 3.0:
        return "LEFT"
    if cx < 2.0 * width / 3.0:
        return "CENTER"
    return "RIGHT"


def focal_length_px(width, hfov=None):
    """Pinhole focal length [px]: (W/2) / tan(HFOV/2). 640 px, 60 deg -> ~554.3."""
    hfov = config.CAMERA_HFOV if hfov is None else hfov
    return (width / 2.0) / math.tan(hfov / 2.0)


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


def blob_geometry(contour, width, height):
    """Measurements of one contour + camera-geometry estimates (no filtering)."""
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    x, y, w, h = cv2.boundingRect(contour)
    (_, _), radius = cv2.minEnclosingCircle(contour)
    if moments["m00"] > 0:
        cx, cy = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
    else:
        cx, cy = x + w / 2.0, y + h / 2.0
    perimeter = cv2.arcLength(contour, True)
    vertices = len(cv2.approxPolyDP(contour, 0.015 * perimeter, True)) if perimeter > 0 else 0
    f = focal_length_px(width)
    diameter = max(2.0 * radius, 1.0)
    depth = f * config.TARGET_SIZE / diameter                # along the optical axis [m]
    lateral = (width / 2.0 - cx) * depth / f                 # + = left [m]
    center_height = config.CAMERA_HEIGHT - (cy - height / 2.0) * depth / f
    return {
        "cx": int(round(cx)), "cy": int(round(cy)), "area": area,
        "box": (x, y, w, h), "radius": float(radius),
        "fill": area / (math.pi * radius * radius) if radius > 0 else 0.0,
        "aspect": w / float(h) if h else 0.0,
        "vertices": vertices,
        "depth": depth, "lateral": lateral,
        "range": math.hypot(depth, lateral),
        "bearing": math.atan2(lateral, depth),               # + = left (CCW)
        "center_height": center_height,
    }


def reject_reason(blob, width=None, height=None):
    """None if the blob looks like a target, else a short reason."""
    if blob["area"] < config.TARGET_MIN_AREA:
        return "small"
    if width is not None and height is not None:
        x, y, w, h = blob["box"]
        m = config.TARGET_BORDER_MARGIN
        if x <= m or y <= m or x + w >= width - m or y + h >= height - m:
            return "border"
    if blob["fill"] < config.TARGET_MIN_FILL:
        return "not round"
    lo, hi = config.TARGET_ASPECT_RANGE
    if not lo <= blob["aspect"] <= hi:
        return "aspect"
    if blob["vertices"] <= config.TARGET_MAX_CORNERS:
        return "corners"
    if blob["range"] > config.TARGET_MAX_RANGE:
        return "too far"
    if config.TARGET_HEIGHT_RANGE is not None:
        lo, hi = config.TARGET_HEIGHT_RANGE
        if not lo <= blob["center_height"] <= hi:
            return "height"
    return None


def find_blobs(frame, hsv_ranges=None):
    """(valid blobs largest first, rejected [(blob, reason)], mask)."""
    height, width = frame.shape[:2]
    mask = target_mask(frame, hsv_ranges)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid, rejected = [], []
    for contour in contours:
        blob = blob_geometry(contour, width, height)
        reason = reject_reason(blob, width, height)
        if reason is None:
            valid.append(blob)
        elif blob["area"] >= 4:  # ignore single-pixel noise in the debug output
            rejected.append((blob, reason))
    valid.sort(key=lambda b: b["area"], reverse=True)
    return valid, rejected, mask


def detect(frame, hsv_ranges=None):
    """(target dict, valid blobs). The dict follows the shared interface."""
    global _warned
    if frame is None:
        return empty_target(), []
    if cv2 is None:
        if not _warned:
            print("[detection] WARNING: OpenCV/NumPy not installed -> detection disabled")
            _warned = True
        return empty_target(), []
    valid, rejected, mask = find_blobs(frame, hsv_ranges)
    if FRAME_DUMP_DIR:
        dump_debug(FRAME_DUMP_DIR, frame, mask, valid, rejected)
    if not valid:
        return empty_target(), []
    best = valid[0]
    target = {"found": True, "cx": best["cx"], "direction": direction_of(best["cx"], frame.shape[1]),
              "area": best["area"]}
    return target, valid


def detect_target(frame, hsv_ranges=None):
    """Shared interface: largest valid target in one frame. Always a NEW dict."""
    return detect(frame, hsv_ranges)[0]


def dump_debug(folder, frame, mask, valid, rejected):
    os.makedirs(folder, exist_ok=True)
    image = np.ascontiguousarray(frame).copy()
    cv2.imwrite(os.path.join(folder, "frame.png"), image)
    cv2.imwrite(os.path.join(folder, "mask.png"), mask)
    labels = ([(b, (0, 255, 0), f"OK {b['range']:.2f}m h{b['center_height']:.2f}") for b in valid]
              + [(b, (0, 0, 255), reason) for b, reason in rejected])
    for blob, color, text in labels:
        x, y, w, h = blob["box"]
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        cv2.putText(image, text, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    h = image.shape[0]
    cv2.line(image, (0, h // 2), (image.shape[1], h // 2), (255, 255, 0), 1)  # horizon
    cv2.imwrite(os.path.join(folder, "debug.png"), image)


# ---------------------------------------------------------------------------
# Multiple frames
# ---------------------------------------------------------------------------
def blob_to_world(blob, pose):
    """World (x, y) of a blob seen from ``pose`` (camera offset applied)."""
    x, y, th = pose
    px = config.CAMERA_OFFSET[0] + blob["depth"]
    py = config.CAMERA_OFFSET[1] + blob["lateral"]
    c, s = math.cos(th), math.sin(th)
    return (x + c * px - s * py, y + s * px + c * py)


class TargetTracker:
    """M-of-N confirmation + de-duplication + visited bookkeeping.

    Call ``update(now, pose, blobs)`` once per detection cycle (also with an
    empty list, so misses are counted). Tracks are dicts:
        {"id", "xy", "confirmed", "visited", "hits", "points", "ranges", "last_seen"}
    """

    def __init__(self, window=None, min_hits=None, dedup_radius=None,
                 range_error=None, max_spread=None, required=None, forget_s=None):
        self.window = config.TRACK_WINDOW if window is None else window
        self.min_hits = config.TRACK_MIN_HITS if min_hits is None else min_hits
        self.dedup_radius = config.TRACK_DEDUP_RADIUS if dedup_radius is None else dedup_radius
        self.range_error = config.TRACK_RANGE_ERROR if range_error is None else range_error
        self.max_spread = config.TRACK_MAX_SPREAD if max_spread is None else max_spread
        self.required = config.REQUIRED_TARGETS if required is None else required
        self.forget_s = config.TRACK_FORGET_S if forget_s is None else forget_s
        self.tracks = []
        self._next_id = 1

    # --------------------------------------------------------- update
    def update(self, now, pose, blobs):
        """Returns the tracks that became confirmed in this cycle."""
        observations = [(blob_to_world(b, pose), b["range"]) for b in blobs]
        matched = set()
        newly_confirmed = []
        for xy, rng in observations:
            track = self._associate(xy, rng, matched)
            if track is None:
                track = {"id": self._next_id, "xy": xy, "confirmed": False, "visited": False,
                         "hits": deque(maxlen=self.window), "points": deque(maxlen=self.window),
                         "ranges": deque(maxlen=self.window), "last_seen": now}
                self._next_id += 1
                self.tracks.append(track)
            if track["id"] in matched:
                continue  # two blobs on the same target in one frame: count once
            matched.add(track["id"])
            track["hits"].append(True)
            track["points"].append(xy)
            track["ranges"].append(rng)
            track["last_seen"] = now
            track["xy"] = _mean(track["points"])
            if not track["confirmed"] and self._confirmable(track):
                track["confirmed"] = True
                newly_confirmed.append(track)
        for track in self.tracks:
            if track["id"] not in matched:
                track["hits"].append(False)
        # tentative tracks survive short occlusions (TRACK_FORGET_S), then are dropped
        self.tracks = [t for t in self.tracks
                       if t["confirmed"] or now - t["last_seen"] <= self.forget_s]
        return newly_confirmed

    def _tolerance(self, base, rng):
        return max(base, self.range_error * rng)

    def _associate(self, xy, rng, matched):
        best, best_d = None, None
        for track in self.tracks:
            d = math.hypot(xy[0] - track["xy"][0], xy[1] - track["xy"][1])
            if d <= self._tolerance(self.dedup_radius, rng) and (best_d is None or d < best_d):
                best, best_d = track, d
        return best

    def _confirmable(self, track):
        if sum(track["hits"]) < self.min_hits:
            return False
        mx, my = track["xy"]
        spread = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in track["points"])
                           / len(track["points"]))
        mean_range = sum(track["ranges"]) / len(track["ranges"])
        return spread <= self._tolerance(self.max_spread, mean_range)

    # --------------------------------------------------------- queries
    def confirmed(self):
        return [t for t in self.tracks if t["confirmed"]]

    def unvisited(self):
        return [t for t in self.confirmed() if not t["visited"]]

    def nearest_unvisited(self, pose):
        candidates = self.unvisited()
        if not candidates:
            return None
        return min(candidates, key=lambda t: math.hypot(t["xy"][0] - pose[0], t["xy"][1] - pose[1]))

    def get(self, track_id):
        return next((t for t in self.tracks if t["id"] == track_id), None)

    def mark_visited(self, track_id):
        for track in self.tracks:
            if track["id"] == track_id:
                track["visited"] = True

    def visited_count(self):
        return sum(1 for t in self.tracks if t["visited"])

    def all_visited(self):
        return self.required is not None and self.visited_count() >= self.required


def relative_to(pose, xy):
    """(bearing [rad, + = left], distance [m]) of world point xy seen from the robot pose.

    For Control / integration: turn by ``bearing`` and drive ``distance`` towards a
    confirmed target (tracker.get(id)["xy"]).
    """
    dx, dy = xy[0] - pose[0], xy[1] - pose[1]
    bearing = math.atan2(dy, dx) - pose[2]
    return math.atan2(math.sin(bearing), math.cos(bearing)), math.hypot(dx, dy)


def is_arrived(pose, xy, distance=None):
    """True when the robot centre is within TARGET_ARRIVAL_DISTANCE of the target."""
    distance = config.TARGET_ARRIVAL_DISTANCE if distance is None else distance
    return relative_to(pose, xy)[1] <= distance


def _mean(points):
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)
