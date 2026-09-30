"""이전 독립 시험의 Webots 장치 연결입니다. encoder와 LiDAR는 필수입니다."""
from math import isclose
from .config import DEFAULT
from .control import Scan, wheel_speeds

class Devices:
    def __init__(self, robot, config=DEFAULT):
        self.robot, self.config = robot, config
        self.step_ms = int(robot.getBasicTimeStep())
        self.left_motor = robot.getDevice(config.left_motor)
        self.right_motor = robot.getDevice(config.right_motor)
        if self.left_motor is None or self.right_motor is None:
            raise RuntimeError('required motors missing')
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        self.left_encoder = robot.getDevice(config.left_encoder)
        self.right_encoder = robot.getDevice(config.right_encoder)
        self.lidar = robot.getDevice(config.lidar)
        if any(device is None for device in (self.left_encoder, self.right_encoder, self.lidar)):
            raise RuntimeError('required encoders/LiDAR missing; motors stopped')
        for sensor in (self.left_encoder, self.right_encoder, self.lidar):
            sensor.enable(self.step_ms)
        if (self.lidar.getHorizontalResolution() != config.lidar_count or
            not isclose(self.lidar.getFov(), config.lidar_fov, abs_tol=0.01) or
            not isclose(self.lidar.getMinRange(), config.lidar_min, abs_tol=0.01) or
            not isclose(self.lidar.getMaxRange(), config.lidar_max, abs_tol=0.01)):
            raise RuntimeError('LiDAR profile mismatch; motors stopped')
        print('TB3 control: step=%d ms, encoder-only, LiDAR=%d %.2f..%.2f m; synchronization unqueried (no Supervisor)' %
              (self.step_ms, self.lidar.getHorizontalResolution(), self.lidar.getMinRange(), self.lidar.getMaxRange()))

    def read(self):
        now = self.robot.getTime()
        return (self.left_encoder.getValue(), self.right_encoder.getValue(),
                Scan(tuple(self.lidar.getRangeImage()), now), now)

    def command(self, v, w):
        left, right = wheel_speeds(v, w, self.config)
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def stop(self):
        self.command(0.0, 0.0)
