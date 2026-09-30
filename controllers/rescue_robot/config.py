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
# customData가 없는 경우의 기본값은 팀 검증 world rescue_baseline의 제공된 시작점입니다.
# 다른 world는 제공된 customData.start_pose를 우선 사용합니다.
START_POSE = (-0.5, -0.8, 0.0)
START_POSE_FROM_CUSTOM_DATA = True

# 주기는 step 개수가 아닌 초 단위입니다. 공식 주행 world는 64ms, 일부 시험 world는 32ms입니다.
MAP_UPDATE_PERIOD = 0.128
DETECTION_PERIOD = 0.128

# ---------------------------------------------------------------------------
# Detection: mission targets = 2 red apples (team decision, 2026-09-30)
# ---------------------------------------------------------------------------
# Target LOCATIONS are never configured here: they are unknown by rule.
REQUIRED_TARGETS = 2          # distinct targets to visit before RETURN_HOME

# OpenCV HSV ranges (H 0-179, S 0-255, V 0-255), list of (lower, upper). A pixel
# matching ANY range counts. Red needs two ranges because its hue wraps at 0/180.
# Tune on real Webots frames: RESCUE_FRAME_DUMP + scripts/tune_hsv.py [INITIAL TUNING].
# Tuned on breakroom_teleop_yolo.wbt frames (2026-09-30). The red cabinet panel there
# is also red; it is rejected by the shape filters below, not by colour.
TARGET_HSV_RANGES = [((0, 111, 60), (7, 255, 255)), ((179, 111, 60), (179, 255, 255))]
# Other practice colours (examples, tune first):
#   green ball [((35, 80, 40), (85, 255, 255))]   orange [((10, 120, 70), (25, 255, 255))]
TARGET_MIN_AREA = 40.0        # px, smaller blobs are ignored [INITIAL TUNING]
DETECTION_BLUR_KERNEL = 5     # odd, GaussianBlur kernel size (0 = off)
DETECTION_MORPH_KERNEL = 3    # odd, opening kernel to remove speckles (0 = off)

# Target size: official RedApple = bounding sphere diameter 0.1 m [OFFICIAL protos].
TARGET_SIZE = 0.10            # m, diameter used for size-based distance (no height assumption)
# Blob filters (reject furniture, books, paintings ...) - none of them assumes where
# the apple lies. Practice worlds put apples on the floor, but the competition
# placement is unknown, so the height filter is OFF by default.
TARGET_MIN_FILL = 0.5         # blob area / enclosing circle area (apple ~0.8) [INITIAL TUNING]
TARGET_ASPECT_RANGE = (0.5, 2.0)  # bounding box width / height [INITIAL TUNING]
# Corner test: the outline simplified with cv2.approxPolyDP (epsilon = 1.5 % of the
# perimeter) has 4 vertices for squares/trapezoids (red cabinet doors, panels) and
# >= 8 for circles and apples (with stem) at every size. Fill alone cannot separate
# them: a square fills 0.64 of its enclosing circle, an apple with stem ~0.65-0.7.
TARGET_MAX_CORNERS = 6        # blobs with this many vertices or fewer are rejected
TARGET_MAX_RANGE = 4.0        # m, farther estimates are ignored [INITIAL TUNING]
# Blobs closer than this to the image border are ignored: a cut-off object has an
# unreliable shape/size (e.g. a red cabinet at the left edge). It is detected once
# it is fully in view.
TARGET_BORDER_MARGIN = 2      # px
# (min, max) height of the apple centre above the floor [m], or None = any height.
# Only set this if the organizers confirm where targets can be, e.g. (0.0, 0.2) = floor.
TARGET_HEIGHT_RANGE = None

# Camera (official extensionSlot Camera): 640x480, horizontal FOV 1.0472 rad,
# robot frame (0.02, 0, 0.073), looking forward, no tilt [OFFICIAL/DERIVED].
CAMERA_HFOV = 1.0472          # rad
CAMERA_OFFSET = (0.02, 0.0)   # (x, y) in the robot frame [m]
CAMERA_HEIGHT = 0.073         # m above the floor

# Multi-frame confirmation and de-duplication (research 07 §4, §6) [INITIAL TUNING]
TRACK_WINDOW = 5              # last N detection cycles
TRACK_MIN_HITS = 3            # seen in >= M of them -> confirmed
TRACK_DEDUP_RADIUS = 0.30     # m, observations closer than this = same target
TRACK_RANGE_ERROR = 0.15      # relative size-distance error -> association radius grows with range
TRACK_MAX_SPREAD = 0.10       # m (grows with range the same way) max position std to confirm
TRACK_FORGET_S = 3.0          # s, a not yet confirmed target survives short occlusions
# Arrival: robot centre within this distance of a confirmed target counts as a visit.
# [DAY-OF] replace with the official arrival rule when the organizers answer.
TARGET_ARRIVAL_DISTANCE = 0.30  # m [INITIAL TUNING]
APPROACH_TIMEOUT = 40.0        # s per approach attempt, then turn to the target and replan [INITIAL TUNING]
APPROACH_MAX_ATTEMPTS = 2      # attempts before the target is skipped [INITIAL TUNING]
APPROACH_FACING_TOLERANCE = 0.15  # rad, heading error to stop the timeout re-orientation [INITIAL TUNING]

# 0.05m 셀, 총 26×26m 지도입니다. 시작점 중심으로 배치해 가장자리 시작점도 포함합니다.
GRID_RESOLUTION = 0.05
GRID_WIDTH = 520
GRID_HEIGHT = 520
# None이면 시작점 중심, 지정하면 [0][0] 셀 왼쪽 아래의 미터 좌표입니다.
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
# 탈출: 팽창 영역 안에 갇혀 계획이 실패하면 가장 트인 방향으로 돌아 천천히 전진합니다.
ESCAPE_SPEED = 0.05             # m/s [INITIAL TUNING]
ESCAPE_DISTANCE = 0.15          # m, 이만큼 전진하면 다시 계획합니다 [INITIAL TUNING]
ESCAPE_TIMEOUT = 12.0           # s, 회전(~3 s)+SLOW 감속 전진(~6 s) 전체 제한 [INITIAL TUNING]
ESCAPE_HALF_ANGLE = math.radians(20)   # 트인 방향을 볼 부채꼴 반각 [INITIAL TUNING]
ESCAPE_HEADING_TOLERANCE = 0.15        # rad [INITIAL TUNING]

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
        observation_half_angle=NAV_OBSERVATION_HALF_ANGLE, escape_half_angle=ESCAPE_HALF_ANGLE,
        spin_clearance=SAFETY_SPIN_CLEARANCE, wait_timeout=NAV_WAIT_TIMEOUT,
        progress_timeout=NAV_PROGRESS_TIMEOUT, progress_distance=NAV_PROGRESS_DISTANCE,
    )
