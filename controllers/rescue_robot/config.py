"""rescue_robot 공통 설정. 로봇 사양과 주행 조정값을 이 파일에서 관리합니다.

공식 TurtleBot3 Burger와 LDS-01 기준입니다. 운용 속도, PID 계수, 거리와
시간 임계값은 초기 조정값이며 Webots에서 검증해야 합니다.
기본 STOP, 명시적인 NAV_TEST/CONTROL_TEST/MISSION에서만 동작을 허용합니다.
"""
import math
import os
from types import SimpleNamespace

# 기본은 정지입니다. MISSION은 미완성 탐색/검출 모듈의 통합 개발용입니다.
BASELINE_MODE = os.environ.get("RESCUE_MODE", "STOP").upper()
CONTROL_TEST_STEP_DURATION = 1.5
# 실제 미션 제한 시간은 운영진 확인 후 변경해야 합니다.
MISSION_TIME_LIMIT = 8 * 60.0
HOME_TOLERANCE = 0.10
STATUS_PRINT_PERIOD = 2.0

# 기존 Localization과 Mapping은 제공된 시작 pose와 같은 좌표계를 사용합니다.
# 선택한 breakroom 시험 world의 제공된 시작 pose입니다. 다른 world에서는 변경합니다.
START_POSE = (-1.265, 1.811, math.radians(-24.3))
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
# The grid is centred on the start pose. Official apartment.wbt spans ~12.4 x
# 13.1 m and its robot starts near an edge (-0.3, -7.5) [OFFICIAL], so the
# grid must be ~2x the world size: 26 m covers any ~13 m world from any start.
GRID_WIDTH = 520        # number of columns (x direction) -> 26.0 m  [DAY-OF]
GRID_HEIGHT = 520       # number of rows    (y direction) -> 26.0 m  [DAY-OF]

# World coordinate of the lower-left corner of cell (0, 0).
# None -> the grid is centred on START_POSE (map size is unknown in advance).
GRID_ORIGIN = None

# 공식 사양: 바퀴 반경, 바퀴 간격, 모터 최대 각속도입니다.
WHEEL_RADIUS = 0.033
AXLE_LENGTH = 0.160
MAX_WHEEL_SPEED = 6.67
ROBOT_RADIUS_NOTEBOOK = 0.105
# PROTO 외접 반경을 올림한 안전 계산용 반경입니다.
ROBOT_RADIUS = 0.111
SAFETY_MARGIN = 0.05
MAX_LINEAR_SPEED = 0.15
MAX_ANGULAR_SPEED = 1.5

# 기존 저수준 안전 모니터 설정을 유지합니다. 마지막에 추가로 적용됩니다.
SAFETY_STOP_MARGIN = 0.05
SAFETY_REACTION_TIME = 0.2
SAFETY_FRONT_HALF_ANGLE = math.radians(90)
SAFETY_MIN_POINTS = 2
SAFETY_SINGLE_POINT_DISTANCE = ROBOT_RADIUS + 0.03
SAFETY_BLIND_ARM_DISTANCE = ROBOT_RADIUS + SAFETY_STOP_MARGIN + 0.10
SAFETY_SPIN_CLEARANCE = 0.13
SAFETY_ALLOW_REVERSE = False
SENSOR_STALE_TIMEOUT = 0.5

# encoder는 라디안 단위입니다. 기존 tick 입력 지원도 유지합니다.
ENCODER_UNITS = "rad"
ENCODER_TICKS_PER_REV = None

def encoder_to_rad():
    """encoder 단위를 바퀴 회전각 라디안으로 바꾸는 계수입니다."""
    if ENCODER_UNITS == "rad":
        return 1.0
    if ENCODER_UNITS == "ticks":
        if not ENCODER_TICKS_PER_REV:
            raise ValueError("ticks 단위에는 ENCODER_TICKS_PER_REV 설정이 필요합니다")
        return 2.0 * math.pi / ENCODER_TICKS_PER_REV
    raise ValueError(f"알 수 없는 encoder 단위: {ENCODER_UNITS!r}")

# IMU는 선택입니다. 기존 Localizer에서는 아직 gyro를 융합하지 않습니다.
GYRO_RAW_TO_RAD_S = 1.0
GYRO_YAW_AXIS = 2
# 대표 인덱스는 180=전방, 90=좌측, 0=후방, 270=우측입니다.
# 정확한 광선 중심 각도는 Webots 벽 배치 테스트로 확인해야 합니다.
LIDAR_FIRST_ANGLE = math.pi
LIDAR_ANGLE_DIRECTION = -1
LIDAR_MOUNT_OFFSET = (-0.03, 0.0)
LIDAR_EXPECTED_RESOLUTION = 360
LIDAR_EXPECTED_FOV = 2 * math.pi
LIDAR_MIN_RANGE = 0.12
LIDAR_MAX_RANGE = 3.5
DEVICE_NAMES = {
    "left_motor": "left wheel motor",
    "right_motor": "right wheel motor",
    "left_encoder": "left wheel sensor",
    "right_encoder": "right wheel sensor",
    "lidar": "LDS-01",
    "camera": "camera",
    "gyro": "gyro",
    "accelerometer": "accelerometer",
}
REQUIRED_DEVICES = ("left_motor", "right_motor", "left_encoder", "right_encoder", "lidar")
FORBIDDEN_DEVICE_NAMES = ("compass", "gps")

# 경로 추종과 안전 예측의 초기 조정값입니다. 신규 의존성은 사용하지 않습니다.
NAV_ROTATE_SPEED = 1.0
NAV_ROTATE_THRESHOLD = 0.6
NAV_HEADING_KP = 2.0
NAV_HEADING_KI = 0.0  # I 항은 실제 주행 조정 전까지 비활성화합니다.
NAV_HEADING_KD = 0.05
NAV_INTEGRAL_LIMIT = 0.4
NAV_LOOKAHEAD = 0.20
NAV_WAYPOINT_TOLERANCE = 0.08
NAV_GOAL_TOLERANCE = 0.10
NAV_APPROACH_DISTANCE = 0.25
NAV_CURVATURE_RADIUS = 0.30
NAV_SLOW_MARGIN = 0.25
NAV_SLOW_RATIO = 0.5
NAV_PREDICTION_TIME = 1.0
NAV_PREDICTION_STEP = 0.05
NAV_SCAN_TIMEOUT = 0.20
NAV_OBSERVATION_HALF_ANGLE = math.pi / 6
NAV_WAIT_TIMEOUT = 4.0
NAV_PROGRESS_TIMEOUT = 10.0
NAV_PROGRESS_DISTANCE = 0.15
NAV_REPLAN_PERIOD = 1.0

def navigation_config():
    """공통 설정을 순수 Python 주행 모듈에 전달합니다. 숫자를 중복 저장하지 않습니다."""
    return SimpleNamespace(
        wheel_radius=WHEEL_RADIUS, axle_length=AXLE_LENGTH,
        max_wheel_speed=MAX_WHEEL_SPEED, robot_radius=ROBOT_RADIUS,
        safety_margin=SAFETY_MARGIN, lidar_offset_x=LIDAR_MOUNT_OFFSET[0],
        lidar_offset_y=LIDAR_MOUNT_OFFSET[1], lidar_min=LIDAR_MIN_RANGE,
        lidar_max=LIDAR_MAX_RANGE, lidar_count=LIDAR_EXPECTED_RESOLUTION,
        lidar_first_angle=LIDAR_FIRST_ANGLE, lidar_direction=LIDAR_ANGLE_DIRECTION,
        lidar_fov=LIDAR_EXPECTED_FOV, max_linear=MAX_LINEAR_SPEED,
        max_angular=MAX_ANGULAR_SPEED, rotate_speed=NAV_ROTATE_SPEED,
        rotate_threshold=NAV_ROTATE_THRESHOLD, heading_kp=NAV_HEADING_KP,
        heading_ki=NAV_HEADING_KI, heading_kd=NAV_HEADING_KD,
        integral_limit=NAV_INTEGRAL_LIMIT, lookahead=NAV_LOOKAHEAD,
        waypoint_tolerance=NAV_WAYPOINT_TOLERANCE, goal_tolerance=NAV_GOAL_TOLERANCE,
        approach_distance=NAV_APPROACH_DISTANCE, curvature_radius=NAV_CURVATURE_RADIUS,
        slow_margin=NAV_SLOW_MARGIN, slow_ratio=NAV_SLOW_RATIO,
        min_points=SAFETY_MIN_POINTS, prediction_time=NAV_PREDICTION_TIME,
        prediction_step=NAV_PREDICTION_STEP, scan_timeout=NAV_SCAN_TIMEOUT,
        observation_half_angle=NAV_OBSERVATION_HALF_ANGLE, wait_timeout=NAV_WAIT_TIMEOUT,
        progress_timeout=NAV_PROGRESS_TIMEOUT, progress_distance=NAV_PROGRESS_DISTANCE,
    )
