"""Webots 없이 실행하는 경로 추종, LiDAR 안전 검사, 대기 및 재계획 요청."""
from dataclasses import dataclass
from math import atan2, cos, sin, hypot, isfinite, isnan, pi, ceil
from numbers import Integral
try:
    from . import config
    from .interfaces import normalize_angle
except ImportError:
    import config
    from interfaces import normalize_angle

DEFAULT = config.navigation_config()

def wrap_angle(angle):
    return normalize_angle(angle)

def checked_pose(pose):
    x, y, theta = map(float, pose)
    if not all(map(isfinite, (x, y, theta))):
        raise ValueError('pose에는 유한한 x, y, theta가 필요합니다')
    return x, y, wrap_angle(theta)

@dataclass(frozen=True)
class Scan:
    ranges: tuple
    timestamp: float

    def points(self, config=DEFAULT):
        points = []
        for i, distance in enumerate(self.ranges):
            if isfinite(distance) and config.lidar_min <= distance <= config.lidar_max:
                angle = config.lidar_first_angle + config.lidar_direction*i*config.lidar_fov/config.lidar_count
                points.append((config.lidar_offset_x+distance*cos(angle),
                               config.lidar_offset_y+distance*sin(angle)))
        return points

def wheel_speeds(v, w, config=DEFAULT):
    if not all(map(isfinite, (v, w))):
        raise ValueError('invalid velocity')
    left = (v-w*config.axle_length/2)/config.wheel_radius
    right = (v+w*config.axle_length/2)/config.wheel_radius
    scale = max(1.0, abs(left)/config.max_wheel_speed,
                abs(right)/config.max_wheel_speed)
    return left/scale, right/scale

def limit_twist(v, w, config=DEFAULT):
    left, right = wheel_speeds(v, w, config)
    return (left+right)*config.wheel_radius/2, (right-left)*config.wheel_radius/config.axle_length

def grid_path_to_waypoints(path, origin, resolution=0.05):
    """지도 원점은 [0][0] 셀의 왼쪽 아래 모서리이며, 셀 중심 좌표를 반환합니다."""
    ox, oy = map(float, origin)
    if not all(map(isfinite, (ox, oy, resolution))) or resolution <= 0:
        raise ValueError('invalid grid geometry')
    result = []
    for row, col in path:
        if not isinstance(row, Integral) or not isinstance(col, Integral) or row < 0 or col < 0:
            raise ValueError('path requires nonnegative integer row, col')
        result.append((ox+(col+0.5)*resolution, oy+(row+0.5)*resolution))
    return result

class HeadingPID:
    def __init__(self, config=DEFAULT):
        self.config = config
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None

    def compute(self, error, dt):
        if not isfinite(error) or not isfinite(dt) or dt <= 0:
            raise ValueError('invalid PID input')
        c = self.config
        error = wrap_angle(error)
        derivative = 0.0 if self.previous is None else wrap_angle(error-self.previous)/dt
        candidate = max(-c.integral_limit, min(c.integral_limit, self.integral+error*dt))
        raw = c.heading_kp*error+c.heading_ki*candidate+c.heading_kd*derivative
        bound = c.rotate_speed
        if abs(raw) <= bound or raw*error < 0:
            self.integral = candidate
        self.previous = error
        return max(-bound, min(bound, raw))

class PathFollower:
    def __init__(self, config=DEFAULT):
        self.config = config
        self.pid = HeadingPID(config)
        self.path = []
        self.index = 0

    def set_path(self, waypoints):
        path = [tuple(map(float, point)) for point in waypoints]
        if any(len(p) != 2 or not all(map(isfinite, p)) for p in path):
            raise ValueError('waypoints must be finite (x, y)')
        self.path, self.index = path, 0
        self.pid.reset()

    def compute(self, pose, scan_points=(), dt=0.064):
        x, y, theta = checked_pose(pose)
        c = self.config
        if not self.path:
            return 0.0, 0.0, 'NO_PATH'
        # waypoint를 순서대로 처리해 순환 경로나 코너를 건너뛰지 않습니다.
        while self.index < len(self.path)-1 and hypot(self.path[self.index][0]-x, self.path[self.index][1]-y) <= c.waypoint_tolerance:
            self.index += 1
        gx, gy = self.path[self.index]
        distance = hypot(gx-x, gy-y)
        if self.index == len(self.path)-1 and distance <= c.goal_tolerance:
            self.pid.reset()
            return 0.0, 0.0, 'REACHED'
        # 다음 waypoint까지만 추종점을 잡아 급한 코너의 안쪽을 가로지르지 않습니다.
        ratio = min(1.0, c.lookahead/max(distance, 1e-9))
        dx, dy = (gx-x)*ratio, (gy-y)*ratio
        rx = cos(theta)*dx+sin(theta)*dy
        ry = -sin(theta)*dx+cos(theta)*dy
        error = atan2(ry, rx)
        if abs(error) > c.rotate_threshold:
            return 0.0, self.pid.compute(error, dt), 'RUNNING'
        self.pid.reset()
        curvature = 2*ry/max(rx*rx+ry*ry, 1e-9)
        scale = min(1.0, 1/max(abs(curvature)*c.curvature_radius, 1.0),
                    distance/c.approach_distance)
        if scan_points:
            clearance = min(hypot(px, py) for px, py in scan_points)-c.robot_radius
            scale = min(scale, max(0.0, clearance/c.slow_margin))
        v = c.max_linear*scale
        w = max(-c.max_angular, min(c.max_angular, v*curvature))
        v, w = limit_twist(v, w, c)
        return v, w, 'RUNNING'

class SafetyMonitor:
    def __init__(self, config=DEFAULT):
        self.config = config

    def filter(self, v, w, scan, now):
        c = self.config
        if not all(map(isfinite, (v, w, now))):
            return 0.0, 0.0, 'INVALID_COMMAND'
        if scan is None or not isfinite(scan.timestamp) or not 0 <= now-scan.timestamp <= c.scan_timeout or len(scan.ranges) != c.lidar_count:
            return 0.0, 0.0, 'INVALID_SCAN'
        if any(isnan(d) or d < c.lidar_min or (isfinite(d) and d > c.lidar_max) for d in scan.ranges):
            return 0.0, 0.0, 'INVALID_SCAN'
        points = scan.points(c)
        radius = c.robot_radius+c.safety_margin
        if any(hypot(px, py) <= radius for px, py in points):
            return 0.0, 0.0, 'STOP'
        if v == 0 and w == 0:
            return v, w, 'CLEAR'
        v, w = limit_twist(v, w, c)
        spin = abs(v) < 1e-9
        # 진행 방향의 미관측 구역은 이동을 차단합니다. inf를 빈 공간으로 바꾸지 않습니다.
        bearing = 0.0 if v >= 0 else pi
        for i, d in enumerate(scan.ranges):
            angle = c.lidar_first_angle+c.lidar_direction*i*c.lidar_fov/c.lidar_count
            # 예측 구간에서 달라지는 진행 방향까지 관측 범위에 포함합니다.
            relevant = spin or abs(wrap_angle(angle-bearing)) <= c.observation_half_angle+abs(w)*c.prediction_time
            if relevant and (not isfinite(d) or not c.lidar_min <= d <= c.lidar_max):
                return 0.0, 0.0, 'UNKNOWN_SPACE'
        # 현재 스캔의 장애물 점으로 이동 궤적을 검사하고 샘플 간격만큼 여유를 둡니다.
        padding = abs(v)*c.prediction_step/2
        count = ceil(c.prediction_time/c.prediction_step)
        for step in range(1, count+1):
            t = step*c.prediction_time/count
            if abs(w) < 1e-9:
                cx, cy = v*t, 0.0
            else:
                cx, cy = v*sin(w*t)/w, v*(1-cos(w*t))/w
            if any(hypot(px-cx, py-cy) <= radius+padding for px, py in points):
                return 0.0, 0.0, 'PREDICTED_COLLISION'
        near = sum(hypot(px, py) < c.robot_radius+c.slow_margin and
                   (spin or (px >= 0 if v > 0 else px <= 0)) for px, py in points)
        if near >= c.min_points:
            return v*c.slow_ratio, w*c.slow_ratio, 'SLOW'
        return v, w, 'CLEAR'

class ProgressMonitor:
    def __init__(self, config=DEFAULT):
        self.config = config
        self.anchor = None
        self.started = None

    def reset(self):
        self.anchor = self.started = None

    def update(self, pose, now, translating):
        if not translating:
            self.reset()  # 대기와 제자리 회전은 전진 실패 시간에 포함하지 않습니다.
            return False
        x, y, _ = checked_pose(pose)
        if self.anchor is None or hypot(x-self.anchor[0], y-self.anchor[1]) >= self.config.progress_distance:
            self.anchor, self.started = (x, y), now
        return now-self.started >= self.config.progress_timeout

class Controller:
    """통합 주행 진입점. 재계획 요청은 새 경로를 받을 때까지 유지합니다."""
    def __init__(self, config=DEFAULT):
        self.config = config
        self.follower = PathFollower(config)
        self.safety = SafetyMonitor(config)
        self.progress = ProgressMonitor(config)
        self.blocked_since = None
        self.replan_required = False

    def set_path(self, waypoints):
        self.follower.set_path(waypoints)
        self.progress.reset()
        self.blocked_since = None
        self.replan_required = False

    def set_grid_path(self, path, origin, resolution=0.05):
        self.set_path(grid_path_to_waypoints(path, origin, resolution))

    def compute(self, pose, scan, now, dt=0.064, defer_safety=False):
        if not isfinite(dt) or dt <= 0 or not isfinite(now):
            return 0.0, 0.0, 'INVALID_TIME'
        if self.replan_required:
            return 0.0, 0.0, 'REPLAN_REQUIRED'
        try:
            points = scan.points(self.config) if scan is not None else ()
            v, w, status = self.follower.compute(pose, points, dt)
        except (ValueError, TypeError):
            return 0.0, 0.0, 'INVALID_INPUT'
        if status in ('NO_PATH', 'REACHED'):
            self.progress.reset()
            return 0.0, 0.0, status
        requested_v, requested_w = v, w
        v, w, event = self.safety.filter(v, w, scan, now)
        if event not in ('CLEAR', 'SLOW'):
            self.progress.reset()
            if self.blocked_since is None:
                self.blocked_since = now
            if now-self.blocked_since >= self.config.wait_timeout:
                self.replan_required = True
                return 0.0, 0.0, 'REPLAN_REQUIRED'
            return 0.0, 0.0, event
        self.blocked_since = None
        if self.progress.update(pose, now, v > 0):
            self.replan_required = True
            return 0.0, 0.0, 'REPLAN_REQUIRED'
        # main이 마지막에 안전 필터를 적용할 때는 감속을 두 번 적용하지 않습니다.
        if defer_safety:
            v, w = requested_v, requested_w
        return v, w, status if event == 'CLEAR' else event
