"""Shared data formats used by every module (see docs/INTERFACES.md).

Keep this file dependency-free: it is imported by tests without Webots.

Coordinate conventions (verified in Webots R2025a, WorldInfo ENU default):
    * World frame: x = +East/"forward" of the arena, y = +North (left of x),
      z = up. Units: metres.
    * theta = 0  -> robot faces +x.  theta grows counter-clockwise (+z axis),
      i.e. turning LEFT increases theta. Range: (-pi, pi].
    * grid[row][col]: row follows +y, col follows +x.
      Cell (row, col) covers x in [ox + col*res, ox + (col+1)*res),
                            y in [oy + row*res, oy + (row+1)*res)
      where (ox, oy) = grid origin = world coordinate of the lower-left corner
      of cell (0, 0).
"""

import math

# ---------------------------------------------------------------------------
# Occupancy grid values
# ---------------------------------------------------------------------------
UNKNOWN = -1
FREE = 0
OCCUPIED = 1
CELL_VALUES = (UNKNOWN, FREE, OCCUPIED)

# Default resolution (metre / cell). The runtime value comes from config.py.
DEFAULT_RESOLUTION = 0.05

# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------
DIRECTIONS = ("LEFT", "CENTER", "RIGHT")


def empty_target():
    """Detection result when nothing is found. Always return a NEW dict."""
    return {
        "found": False,
        "cx": None,         # int pixel column of the target centre, or None
        "direction": None,  # "LEFT" | "CENTER" | "RIGHT" | None
        "area": 0.0,        # float pixel area, 0.0 when not found
    }


def is_valid_target(target):
    """True if ``target`` follows the detection interface."""
    if not isinstance(target, dict) or not {"found", "cx", "direction", "area"} <= set(target):
        return False
    if not isinstance(target["found"], bool) or not isinstance(target["area"], (int, float)):
        return False
    if target["found"]:
        return isinstance(target["cx"], int) and target["direction"] in DIRECTIONS
    return target["cx"] is None and target["direction"] is None


# ---------------------------------------------------------------------------
# Pose:  pose = (x, y, theta)   metres, metres, radians
# ---------------------------------------------------------------------------
def normalize_angle(theta):
    """Wrap an angle to (-pi, pi]."""
    theta = math.fmod(theta, 2.0 * math.pi)
    if theta <= -math.pi:
        theta += 2.0 * math.pi
    elif theta > math.pi:
        theta -= 2.0 * math.pi
    return theta


def make_pose(x, y, theta):
    """Build a pose tuple with a normalised heading."""
    return (float(x), float(y), normalize_angle(float(theta)))


def is_valid_pose(pose):
    return (
        isinstance(pose, tuple)
        and len(pose) == 3
        and all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose)
        and -math.pi < pose[2] <= math.pi
    )


# ---------------------------------------------------------------------------
# Planning:  path = [(row, col), ...]   start first, goal last, [] if none
# Control:   waypoint = (x, y)          world metres
# ---------------------------------------------------------------------------
def is_valid_path(path):
    return isinstance(path, list) and all(
        isinstance(c, tuple) and len(c) == 2 and all(isinstance(v, int) for v in c) for c in path
    )


def is_valid_waypoint(waypoint):
    return (
        isinstance(waypoint, tuple)
        and len(waypoint) == 2
        and all(isinstance(v, (int, float)) and math.isfinite(v) for v in waypoint)
    )
