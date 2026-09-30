"""Central configuration for the rescue_robot controller.

Every value that may change on hackathon day (robot model, device names,
wheel geometry, map size, safety distances) lives here so that other modules
never hard-code robot-specific numbers.

Values marked ``PRACTICE DEFAULT`` were measured on the practice e-puck in
``worlds/rescue_baseline.wbt`` (Webots R2025a E-puck.proto, version "1").
They MUST be re-checked against the official robot on hackathon day.

Environment overrides (for testing only):
    RESCUE_MODE       -> overrides BASELINE_MODE ("STOP" or "CONTROL_TEST")
    RESCUE_MAP_DUMP   -> if set, path of a .pgm file where the map is dumped
"""

import math
import os

# ---------------------------------------------------------------------------
# Baseline behaviour
# ---------------------------------------------------------------------------
# "STOP"         : mission state machine runs, motors are held at zero.
#                  Safe default while exploration / tracking are TODO.
# "CONTROL_TEST" : short scripted forward / left / right / stop sequence,
#                  used to verify the drive train. Emergency stop stays active.
BASELINE_MODE = os.environ.get("RESCUE_MODE", "STOP").upper()

# Seconds per step of the CONTROL_TEST sequence.
CONTROL_TEST_STEP_DURATION = 1.5

# Mission time budget. When exceeded the state machine switches to RETURN_HOME.
MISSION_TIME_LIMIT = 8 * 60.0  # s (TODO: set to the official limit on the day)

# RETURN_HOME is complete when the pose is within this distance of home_pose.
HOME_TOLERANCE = 0.10  # m

# Status line period in the console.
STATUS_PRINT_PERIOD = 2.0  # s

# ---------------------------------------------------------------------------
# Start pose (home_pose)
# ---------------------------------------------------------------------------
# The hackathon provides the start position & orientation. Put it here.
# Format: (x [m], y [m], theta [rad]) in the Webots world frame (see
# docs/INTERFACES.md). PRACTICE DEFAULT = E-puck pose in rescue_baseline.wbt.
START_POSE = (-0.5, -0.8, 0.0)  # PRACTICE DEFAULT

# ---------------------------------------------------------------------------
# Occupancy grid
# ---------------------------------------------------------------------------
GRID_RESOLUTION = 0.05  # m / cell
GRID_WIDTH = 160        # number of columns (x direction) -> 8.0 m
GRID_HEIGHT = 160       # number of rows    (y direction) -> 8.0 m

# World coordinate of the lower-left corner of cell (0, 0).
# None -> the grid is centred on START_POSE (map size is unknown in advance).
GRID_ORIGIN = None

# Recompute the map every N control steps (LiDAR ray marking is pure Python).
MAP_UPDATE_PERIOD_STEPS = 5

# ---------------------------------------------------------------------------
# Robot geometry & limits
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.020        # m    PRACTICE DEFAULT (E-puck.proto wheel radius)
AXLE_LENGTH = 0.052         # m    PRACTICE DEFAULT (wheels at y = +/-0.026)
MAX_WHEEL_SPEED = 6.28      # rad/s PRACTICE DEFAULT (e-puck v1 motor maxVelocity)
ROBOT_RADIUS = 0.037        # m    PRACTICE DEFAULT (e-puck body radius)

SAFETY_MARGIN = 0.05        # m  extra clearance used when inflating obstacles
MAX_LINEAR_SPEED = 0.08     # m/s (must be <= WHEEL_RADIUS * MAX_WHEEL_SPEED)
MAX_ANGULAR_SPEED = 1.5     # rad/s

# Emergency collision prevention: stop forward motion if an obstacle is
# closer than this (measured from the LiDAR centre) inside the front sector.
EMERGENCY_STOP_DISTANCE = ROBOT_RADIUS + 0.04  # m
EMERGENCY_FRONT_HALF_ANGLE = math.radians(30)  # rad

# ---------------------------------------------------------------------------
# Wheel encoders
# ---------------------------------------------------------------------------
# Webots PositionSensor on a RotationalMotor reports radians.
# If the official robot reports ticks, set ENCODER_UNITS = "ticks" and
# ENCODER_TICKS_PER_REV to the documented value (do NOT guess it).
ENCODER_UNITS = "rad"          # PRACTICE DEFAULT
ENCODER_TICKS_PER_REV = None   # only used when ENCODER_UNITS == "ticks"


def encoder_to_rad():
    """Factor converting one encoder unit to wheel radians."""
    if ENCODER_UNITS == "rad":
        return 1.0
    if ENCODER_UNITS == "ticks":
        if not ENCODER_TICKS_PER_REV:
            raise ValueError("ENCODER_UNITS='ticks' requires ENCODER_TICKS_PER_REV in config.py")
        return 2.0 * math.pi / ENCODER_TICKS_PER_REV
    raise ValueError(f"Unknown ENCODER_UNITS: {ENCODER_UNITS!r}")


# ---------------------------------------------------------------------------
# Inertial sensors (no IMU node; individual sensors only)
# ---------------------------------------------------------------------------
# The e-puck Gyro lookupTable maps +/-13.315805 rad/s to +/-100000, so raw
# values must be scaled. Re-check the lookupTable of the official robot.
GYRO_RAW_TO_RAD_S = 13.315805 / 100000.0  # PRACTICE DEFAULT
GYRO_YAW_AXIS = 2                           # index of the z (yaw) axis in getValues()

# ---------------------------------------------------------------------------
# LiDAR
# ---------------------------------------------------------------------------
# Angle of range index i, in the robot frame (0 = forward, + = CCW/left):
#     angle_i = LIDAR_FIRST_ANGLE + LIDAR_ANGLE_DIRECTION * i * fov / N
# Verified in Webots R2025a for the 360 deg Lidar in rescue_baseline.wbt:
# index 0 points backwards and indices sweep clockwise (see docs/INTERFACES.md).
LIDAR_FIRST_ANGLE = math.pi    # rad, PRACTICE DEFAULT (= +fov/2 for a 360 deg lidar)
LIDAR_ANGLE_DIRECTION = -1     # -1: clockwise sweep, +1: counter-clockwise
LIDAR_MOUNT_OFFSET = (0.0, 0.0)  # (x, y) of the lidar in the robot frame [m]

# ---------------------------------------------------------------------------
# Device names (change these first when the robot model changes)
# ---------------------------------------------------------------------------
DEVICE_NAMES = {
    # required
    "left_motor": "left wheel motor",     # PRACTICE DEFAULT
    "right_motor": "right wheel motor",   # PRACTICE DEFAULT
    # optional (None or missing device -> feature disabled with a warning)
    "left_encoder": "left wheel sensor",  # PRACTICE DEFAULT
    "right_encoder": "right wheel sensor",  # PRACTICE DEFAULT
    "camera": "camera",                   # PRACTICE DEFAULT
    "lidar": "lidar",                     # PRACTICE DEFAULT (added in rescue_baseline.wbt)
    "gyro": "gyro",                       # PRACTICE DEFAULT
    "accelerometer": "accelerometer",     # PRACTICE DEFAULT
    "compass": None,                      # e-puck has none
    # Ground-truth GPS: DEBUG ONLY. Must never feed localization, because the
    # hackathon requires sensor-based pose estimation.
    "gps_debug": "gps",                   # PRACTICE DEFAULT (turretSlot GPS)
}
REQUIRED_DEVICES = ("left_motor", "right_motor")
