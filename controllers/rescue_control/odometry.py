"""이전 독립 시험에서 시작점 기준 pose를 추정하는 encoder odometry입니다."""
from math import cos, sin, pi, isfinite
from .config import DEFAULT

def wrap_angle(angle):
    value = (angle + pi) % (2*pi) - pi
    return pi if value == -pi else value

def checked_pose(pose):
    x, y, theta = map(float, pose)
    if not all(map(isfinite, (x, y, theta))):
        raise ValueError('pose must contain finite x, y, theta')
    return x, y, wrap_angle(theta)

class Odometry:
    def __init__(self, pose=(0.0, 0.0, 0.0), config=DEFAULT):
        self.config = config
        self.pose = checked_pose(pose)
        self.previous = None

    def correct_pose(self, pose):
        # encoder 기준값을 유지하여 다음 갱신에는 새 이동량만 반영합니다.
        self.pose = checked_pose(pose)

    def update(self, left, right):
        left, right = float(left), float(right)
        if not all(map(isfinite, (left, right))):
            raise ValueError('invalid encoder sample')
        if self.previous is None:
            self.previous = left, right
            return self.pose
        dl = (left-self.previous[0])*self.config.wheel_radius
        dr = (right-self.previous[1])*self.config.wheel_radius
        self.previous = left, right
        ds = (dr+dl)/2
        da = (dr-dl)/self.config.axle_length
        x, y, theta = self.pose
        self.pose = (x+ds*cos(theta+da/2), y+ds*sin(theta+da/2),
                     wrap_angle(theta+da))
        return self.pose
