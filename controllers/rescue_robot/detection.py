"""Target detection interface (STUB).

The real detector will be developed on the ``feat/detection`` branch once the
target's visual features are announced on hackathon day. Only this module
should need to change: main.py calls ``detect_target(frame)`` and relies on
the return format below.

Input:
    frame : NumPy array (H, W, 3), BGR, uint8 - from devices.read_camera_frame()
            or None if no camera image is available.

Output (see interfaces.empty_target / docs/INTERFACES.md):
    {
        "found": bool,
        "cx": int | None,          # target centre column in pixels
        "direction": "LEFT" | "CENTER" | "RIGHT" | None,
        "area": float,             # pixel area, 0.0 if not found
    }
    direction rule: split the image width in three equal parts.

Reference: archive/practice_project/controllers/detection_controller/ contains the practice HSV
red-box detector (kept as-is, not copied here on purpose).
"""

from interfaces import empty_target


def detect_target(frame):
    # TODO(feat/detection): implement once target features are known.
    return empty_target()
