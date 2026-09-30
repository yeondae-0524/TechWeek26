"""이전 독립 시험 설정입니다. 공식 TB3 기하를 사용하며 주행 임계값은 초기 조정값입니다."""
from dataclasses import dataclass
from math import pi

@dataclass(frozen=True)
class Config:
    wheel_radius: float = 0.033
    axle_length: float = 0.160
    max_wheel_speed: float = 6.67
    robot_radius: float = 0.111
    safety_margin: float = 0.05
    lidar_offset_x: float = -0.03
    lidar_offset_y: float = 0.0
    lidar_min: float = 0.12
    lidar_max: float = 3.5
    lidar_count: int = 360
    lidar_first_angle: float = pi  # 대표 인덱스 기준이며 Webots에서 확인해야 합니다.
    lidar_direction: float = -1.0
    lidar_fov: float = 2*pi
    left_motor: str = 'left wheel motor'
    right_motor: str = 'right wheel motor'
    left_encoder: str = 'left wheel sensor'
    right_encoder: str = 'right wheel sensor'
    lidar: str = 'LDS-01'
    max_linear: float = 0.15
    max_angular: float = 1.5
    rotate_speed: float = 1.0
    rotate_threshold: float = 0.6
    heading_kp: float = 2.0
    heading_ki: float = 0.0  # I/D 계수는 실제 주행에서 조정해야 합니다.
    heading_kd: float = 0.05
    integral_limit: float = 0.4
    lookahead: float = 0.20
    waypoint_tolerance: float = 0.08
    goal_tolerance: float = 0.10
    approach_distance: float = 0.25
    curvature_radius: float = 0.30
    slow_margin: float = 0.25
    slow_ratio: float = 0.5
    min_points: int = 3
    prediction_time: float = 1.0
    prediction_step: float = 0.05
    scan_timeout: float = 0.20
    observation_half_angle: float = pi/6
    wait_timeout: float = 4.0
    progress_timeout: float = 10.0
    progress_distance: float = 0.15

DEFAULT = Config()
