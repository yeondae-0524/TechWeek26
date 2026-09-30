"""Tune config.TARGET_HSV_RANGES on a real camera frame (OpenCV window with sliders).

1. Save frames from Webots: set RESCUE_FRAME_DUMP=<folder> before starting Webots
   -> <folder>/frame.png, mask.png, debug.png are rewritten at every detection.
2. Copy an interesting frame.png (target visible) somewhere, then:

    python scripts/tune_hsv.py D:/Dev/projects/frames/frame.png            # sliders
    python scripts/tune_hsv.py frame.png --pixel 320 300 330 310           # HSV of pixels
    python scripts/tune_hsv.py frame.png --check                           # run the detector

Window keys: click = print HSV at that pixel, p = print the config line,
s = save mask_tuned.png next to the image, q / Esc = quit.
Hue wraps for red: set "H low" > "H high" (e.g. 170 and 10) to get two ranges.
OpenCV HSV: H 0-179, S 0-255, V 0-255.
"""

import argparse
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "rescue_robot"))

import config  # noqa: E402

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
    np = None

WINDOW = "tune_hsv (click: HSV, p: print, s: save, q: quit)"
SLIDERS = (("H low", 179), ("H high", 179), ("S low", 255), ("S high", 255), ("V low", 255), ("V high", 255))


def ranges_from_sliders(h_lo, h_hi, s_lo, s_hi, v_lo, v_hi):
    """Slider values -> list of (lower, upper). h_lo > h_hi wraps around 0 (red)."""
    if h_lo <= h_hi:
        return [((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))]
    return [((0, s_lo, v_lo), (h_hi, s_hi, v_hi)), ((h_lo, s_lo, v_lo), (179, s_hi, v_hi))]


def sliders_from_ranges(ranges):
    """Inverse of ranges_from_sliders for the config format (1 range, or 2 wrapping at 0/179)."""
    if len(ranges) == 2:
        (lo1, hi1), (lo2, hi2) = sorted(ranges)
        if lo1[0] == 0 and hi2[0] == 179:
            return (lo2[0], hi1[0], max(lo1[1], lo2[1]), min(hi1[1], hi2[1]),
                    max(lo1[2], lo2[2]), min(hi1[2], hi2[2]))
    (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi) = ranges[0]
    return (h_lo, h_hi, s_lo, s_hi, v_lo, v_hi)


def config_line(ranges):
    return "TARGET_HSV_RANGES = [" + ", ".join(f"({tuple(lo)}, {tuple(hi)})" for lo, hi in ranges) + "]"


def hsv_at(image, x, y, k=5):
    """Median HSV of a k x k patch around (x, y) of a BGR image."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    x0, x1 = max(0, x - k // 2), min(w, x + k // 2 + 1)
    y0, y1 = max(0, y - k // 2), min(h, y + k // 2 + 1)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
    return tuple(int(v) for v in np.median(patch, axis=0))


def mask_for(image, ranges):
    import detection
    return detection.target_mask(image, ranges)


def run_check(image):
    import detection
    valid, rejected, _ = detection.find_blobs(image)
    print(f"config ranges: {config_line(config.TARGET_HSV_RANGES)}")
    for b in valid:
        print(f"  OK       cx={b['cx']:3d} cy={b['cy']:3d} area={b['area']:7.0f} fill={b['fill']:.2f} "
              f"range={b['range']:.2f} m bearing={np.degrees(b['bearing']):+.1f} deg height={b['center_height']:.2f} m")
    for b, reason in rejected:
        print(f"  REJECTED ({reason:12s}) cx={b['cx']:3d} cy={b['cy']:3d} area={b['area']:7.0f} "
              f"fill={b['fill']:.2f} aspect={b['aspect']:.2f} range={b['range']:.2f} m")
    if not valid and not rejected:
        print("  nothing matches the colour ranges")


def run_gui(image, path):
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    for (name, maximum), value in zip(SLIDERS, sliders_from_ranges(config.TARGET_HSV_RANGES)):
        cv2.createTrackbar(name, WINDOW, int(value), maximum, lambda _v: None)

    def on_click(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x = x % image.shape[1]  # left half = image, right half = mask
            print(f"pixel ({x}, {y}) HSV = {hsv_at(image, x, y)}")
    cv2.setMouseCallback(WINDOW, on_click)

    while True:
        values = [cv2.getTrackbarPos(name, WINDOW) for name, _ in SLIDERS]
        ranges = ranges_from_sliders(*values)
        mask = mask_for(image, ranges)
        overlay = image.copy()
        overlay[mask > 0] = (0, 255, 0)
        cv2.imshow(WINDOW, np.hstack([overlay, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
        key = cv2.waitKey(50) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("p"):
            print(config_line(ranges))
        if key == ord("s"):
            out = os.path.join(os.path.dirname(os.path.abspath(path)), "mask_tuned.png")
            cv2.imwrite(out, mask)
            print(f"saved {out}")
    print("final: " + config_line(ranges_from_sliders(*values)))
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", help="BGR image, e.g. frame.png from RESCUE_FRAME_DUMP")
    parser.add_argument("--pixel", nargs="+", type=int, metavar="X Y", help="print HSV at pixel pairs")
    parser.add_argument("--check", action="store_true", help="run the detector with config values")
    args = parser.parse_args()
    if cv2 is None:
        raise SystemExit("OpenCV/NumPy not installed: py -3.10 -m pip install numpy==1.23.5 opencv-python==4.8.0.74")
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"cannot read {args.image}")
    if args.pixel:
        if len(args.pixel) % 2:
            raise SystemExit("--pixel needs X Y pairs")
        for x, y in zip(args.pixel[::2], args.pixel[1::2]):
            print(f"pixel ({x}, {y}) HSV = {hsv_at(image, x, y)}")
    elif args.check:
        run_check(image)
    else:
        run_gui(image, args.image)
    return 0


if __name__ == "__main__":
    sys.exit(main())
