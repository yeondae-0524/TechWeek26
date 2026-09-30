"""Central configuration for the rescue_robot controller.

Every value that may change on hackathon day (robot model, device names,
wheel geometry, map size, safety distances) lives here so that other modules
never hard-code robot-specific numbers.

Robot profile: official TECH WEEK robot = Webots R2025a ``TurtleBot3Burger``
with the default ``RobotisLds01`` (LDS-01) LiDAR and a 640x480 / 60 deg camera.
Source of every number: docs/research/09_WEBOTS_REFERENCES.md and
docs/TURTLEBOT3_MIGRATION.md. Value tags (same as docs/research):

    [OFFICIAL]        official TECH WEEK repo / notebook / Webots R2025a PROTO
    [DERIVED]         computed from official values
    [ORGANIZER]       confirmed by the organizers
    [INITIAL TUNING]  starting value, NOT validated -> tune in Webots
    [DAY-OF]          must be re-checked with the final competition world

Sensor rules [ORGANIZER]: wheel encoders and 2D LiDAR are mandatory, IMU is
optional (team choice), Compass and GPS are NOT used. Supervisor ground-truth
pose is never a controller input.

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
#                  used to verify the drive train. Safety monitor stays active.
BASELINE_MODE = os.environ.get("RESCUE_MODE", "STOP").upper()

# Seconds per step of the CONTROL_TEST sequence.
CONTROL_TEST_STEP_DURATION = 1.5

# Mission time budget. When exceeded the state machine switches to RETURN_HOME.
# TODO(feat/integration): replace by the ETA-based time budget (research 08 §2.2).
MISSION_TIME_LIMIT = 8 * 60.0  # s [DAY-OF] official limit and clock (sim/real) unknown

# RETURN_HOME is complete when the pose is within this distance of home_pose.
HOME_TOLERANCE = 0.10  # m [INITIAL TUNING] arrival rule is [DAY-OF]

# Status line period in the console.
STATUS_PRINT_PERIOD = 2.0  # s

# ---------------------------------------------------------------------------
# Start pose (home_pose)
# ---------------------------------------------------------------------------
# The hackathon provides the start position & orientation [OFFICIAL plan],
# format [DAY-OF]. Format here: (x [m], y [m], theta [rad]) in the Webots world
# frame (see docs/INTERFACES.md). Default = TurtleBot3 pose in rescue_baseline.wbt.
START_POSE = (-0.5, -0.8, 0.0)

# If True, a JSON ``{"start_pose": [x, y, theta]}`` in the robot's customData
# field overrides START_POSE (the test world writes it there). Only the known
# start pose is read; nothing else from the world is used for navigation.
START_POSE_FROM_CUSTOM_DATA = True

# ---------------------------------------------------------------------------
# Scheduling (seconds, never step counts: the official driving worlds use a
# 64 ms basicTimeStep, some test worlds 32 ms [OFFICIAL]; research 10 §2.2)
# ---------------------------------------------------------------------------
MAP_UPDATE_PERIOD = 0.128   # s [INITIAL TUNING] one new scan into the grid
DETECTION_PERIOD = 0.128    # s [INITIAL TUNING] camera read + detect_target()

# ---------------------------------------------------------------------------
# Occupancy grid
# ---------------------------------------------------------------------------
GRID_RESOLUTION = 0.05  # m / cell
# Official example worlds are ~13 m wide [DERIVED] and the grid is centred on
# the start pose, so >= 20 m is recommended (research 03 §A.4, 09 §11).
GRID_WIDTH = 400        # number of columns (x direction) -> 20.0 m  [DAY-OF]
GRID_HEIGHT = 400       # number of rows    (y direction) -> 20.0 m  [DAY-OF]

# World coordinate of the lower-left corner of cell (0, 0).
# None -> the grid is centred on START_POSE (map size is unknown in advance).
GRID_ORIGIN = None

# ---------------------------------------------------------------------------
# Robot geometry & limits (TurtleBot3Burger R2025a)
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.033        # m     [OFFICIAL] notebook + PROTO
AXLE_LENGTH = 0.160         # m     [OFFICIAL] wheel separation, anchors y = +/-0.08
MAX_WHEEL_SPEED = 6.67      # rad/s [OFFICIAL] PROTO motor maxVelocity
# Notebook model radius. Kept for reference only - NOT used for safety.
ROBOT_RADIUS_NOTEBOOK = 0.105  # m  [OFFICIAL] notebook ROBOT_RADIUS
# Conservative safety circle around the wheel axle centre: PROTO circumscribed
# radius ~0.1103 m rounded up [DERIVED]. The real footprint is asymmetric
# (front x ~ +0.036 m, back x ~ -0.100 m).
ROBOT_RADIUS = 0.111        # m     [DERIVED]

SAFETY_MARGIN = 0.05        # m  [INITIAL TUNING] inflation = 0.111 + 0.05 = 0.161 m (3.22 cells)
# Operating limits. Hardware limits are 0.22 m/s and 2.75 rad/s [DERIVED];
# v and w are additionally limited together by the wheel speed limit.
MAX_LINEAR_SPEED = 0.15     # m/s   [INITIAL TUNING]
MAX_ANGULAR_SPEED = 1.5     # rad/s [INITIAL TUNING]

# ---------------------------------------------------------------------------
# Safety monitor (raw LiDAR, every step, applied last; research 05 §3)
# ---------------------------------------------------------------------------
# STOP zone: obstacle points (robot frame, wheel-axle centre) closer than
# ROBOT_RADIUS + SAFETY_STOP_MARGIN + v * SAFETY_REACTION_TIME in the direction
# of travel. The whole STOP disc boundary lies outside the LDS-01 minRange
# blind zone (LiDAR is 0.03 m behind the axle, minRange 0.12 m).
SAFETY_STOP_MARGIN = 0.05       # m [INITIAL TUNING]
SAFETY_REACTION_TIME = 0.2      # s [INITIAL TUNING] sensor age + command latency; braking UNCONFIRMED
SAFETY_FRONT_HALF_ANGLE = math.radians(90)  # rad: points ahead of the axle
SAFETY_MIN_POINTS = 2           # [INITIAL TUNING] points needed to trigger STOP
# A single point this close always triggers STOP (thin obstacles, research 05 §3).
SAFETY_SINGLE_POINT_DISTANCE = ROBOT_RADIUS + 0.03  # m [INITIAL TUNING]
# A ray whose last finite hit was closer than this and that now reads inf is
# treated as "object inside the minRange blind zone" -> STOP, never as free.
SAFETY_BLIND_ARM_DISTANCE = ROBOT_RADIUS + SAFETY_STOP_MARGIN + 0.10  # m [INITIAL TUNING]
# In-place rotation sweeps the asymmetric body (~0.110 m); require this clearance.
SAFETY_SPIN_CLEARANCE = 0.13    # m [INITIAL TUNING]
# The rear is mostly inside the LiDAR blind zone -> no reversing (M0 rule).
SAFETY_ALLOW_REVERSE = False
# No valid scan for this long -> stop all motion (fail closed).
SENSOR_STALE_TIMEOUT = 0.5      # s [INITIAL TUNING]

# ---------------------------------------------------------------------------
# Wheel encoders (mandatory [ORGANIZER])
# ---------------------------------------------------------------------------
# TurtleBot3 "left/right wheel sensor" are PositionSensors in radians,
# resolution 0.00628 rad [OFFICIAL].
ENCODER_UNITS = "rad"
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
# Inertial sensors (optional, team choice [ORGANIZER])
# ---------------------------------------------------------------------------
# TurtleBot3 Gyro has no lookupTable: Webots returns rad/s directly [OFFICIAL].
GYRO_RAW_TO_RAD_S = 1.0
GYRO_YAW_AXIS = 2                           # index of the z (yaw) axis in getValues()

# ---------------------------------------------------------------------------
# LiDAR (LDS-01, mandatory [ORGANIZER])
# ---------------------------------------------------------------------------
# Angle of range index i, in the robot frame (0 = forward, + = CCW/left):
#     angle_i = LIDAR_FIRST_ANGLE + LIDAR_ANGLE_DIRECTION * i * fov / N
# Matches the official labels ranges[180]=front, [90]=left, [0]=back,
# [270]=right [OFFICIAL]. Sub-degree beam-centre alignment is UNCONFIRMED
# (check the boot log "[lidar] ranges:" against the world).
LIDAR_FIRST_ANGLE = math.pi
LIDAR_ANGLE_DIRECTION = -1     # -1: clockwise sweep, +1: counter-clockwise
# LDS-01 origin in the robot frame: 0.03 m behind the axle, 0.173 m high [DERIVED].
LIDAR_MOUNT_OFFSET = (-0.03, 0.0)  # (x, y) [m]

# ---------------------------------------------------------------------------
# Device names (change these first when the robot model changes)
# ---------------------------------------------------------------------------
# Compass and GPS exist on the official robot/examples but are NOT used by the
# competition controller [ORGANIZER] -> intentionally absent from this table.
DEVICE_NAMES = {
    "left_motor": "left wheel motor",       # [OFFICIAL]
    "right_motor": "right wheel motor",     # [OFFICIAL]
    "left_encoder": "left wheel sensor",    # [OFFICIAL]
    "right_encoder": "right wheel sensor",  # [OFFICIAL]
    "lidar": "LDS-01",                      # [OFFICIAL] RobotisLds01 name
    "camera": "camera",                     # [OFFICIAL] extensionSlot Camera
    "gyro": "gyro",                         # [OFFICIAL] optional
    "accelerometer": "accelerometer",       # [OFFICIAL] optional
}
# Missing / invalid -> DeviceError and the robot never moves (fail closed).
REQUIRED_DEVICES = ("left_motor", "right_motor", "left_encoder", "right_encoder", "lidar")
# Devices that must never be enabled by the competition controller.
FORBIDDEN_DEVICE_NAMES = ("compass", "gps")
