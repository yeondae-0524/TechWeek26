"""Pose estimation baseline: pose = (x, y, theta).

Implemented:
    * pose interface + reset to the provided start pose (home_pose)
    * differential-drive wheel odometry, parameterised by wheel radius,
      axle length and encoder unit conversion (all injected from config)
    * optional GyroHeading: stationary bias calibration, gyro heading with
      encoder translation, and encoder-only fallback on unusable gyro data

NOT implemented (TODO, feat/localization):
    * main.py opt-in wiring of GyroHeading and explicit stationary confirmation
    * accelerometer use / slip detection
    * scan matching against the occupancy grid (pose correction)
    * uncertainty / covariance
"""

import math

from interfaces import make_pose


class DiffDriveOdometry:
    """Integrates wheel encoder deltas into a world-frame pose.

    Parameters are required on purpose: the official robot's geometry must be
    written in config.py, never guessed inside this module.
    """

    def __init__(self, wheel_radius, axle_length, encoder_to_rad, initial_pose):
        if wheel_radius <= 0 or axle_length <= 0 or encoder_to_rad <= 0:
            raise ValueError("wheel_radius, axle_length and encoder_to_rad must be > 0")
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.encoder_to_rad = encoder_to_rad
        self._prev = None
        self.pose = make_pose(*initial_pose)

    def reset(self, pose):
        self.pose = make_pose(*pose)
        self._prev = None

    def update(self, left_enc, right_enc, heading_delta=None):
        """Update from absolute encoder readings; returns the new pose.

        The first call only stores the reference readings.
        """
        if self._prev is None:
            self._prev = (left_enc, right_enc)
            return self.pose
        d_left = (left_enc - self._prev[0]) * self.encoder_to_rad * self.wheel_radius
        d_right = (right_enc - self._prev[1]) * self.encoder_to_rad * self.wheel_radius
        self._prev = (left_enc, right_enc)
        self.pose = integrate_diff_drive(self.pose, d_left, d_right, self.axle_length,
                                        heading_delta=heading_delta)
        return self.pose


def integrate_diff_drive(pose, d_left, d_right, axle_length, heading_delta=None):
    """Midpoint integration of wheel travel distances [m] (+theta = CCW)."""
    x, y, th = pose
    d_center = (d_left + d_right) / 2.0
    d_theta = ((d_right - d_left) / axle_length
               if heading_delta is None else heading_delta)
    mid = th + d_theta / 2.0
    return make_pose(x + d_center * math.cos(mid), y + d_center * math.sin(mid), th + d_theta)


def _finite(value):
    try:
        return value is not None and math.isfinite(value)
    except (TypeError, ValueError):
        return False


class GyroHeading:
    """Opt-in heading correction; all tuning limits are supplied by the caller.

    calibration_duration/max_dt: seconds; stationary_wheel_speed: m/s;
    max_stationary_rate/max_rate: rad/s; weight: gyro share in [0, 1].
    Bias requires consecutive valid intervals with explicit stationary=True,
    low encoder speed AND low gyro rate. Encoder standstill alone is not proof
    of a stationary body. Once calibrated, integrate corrected rate over dt.
    This does not correct translation slip or detect a frozen, finite gyro.
    """

    def __init__(self, *, calibration_duration, stationary_wheel_speed,
                 max_stationary_rate, max_rate, max_dt, weight=1.0):
        limits = (calibration_duration, stationary_wheel_speed,
                  max_stationary_rate, max_rate, max_dt)
        if any(not _finite(v) or v <= 0 for v in limits):
            raise ValueError("gyro limits must be finite and positive")
        if max_stationary_rate > max_rate:
            raise ValueError("stationary rate limit must not exceed max_rate")
        if not _finite(weight) or not 0 <= weight <= 1:
            raise ValueError("gyro weight must be in [0, 1]")
        self.calibration_duration = calibration_duration
        self.stationary_wheel_speed = stationary_wheel_speed
        self.max_stationary_rate = max_stationary_rate
        self.max_rate = max_rate
        self.max_dt = max_dt
        self.weight = weight
        self.reset()

    def reset(self):
        self.bias = None
        self.status = "UNCALIBRATED"
        self.clear_pending()

    def clear_pending(self):
        self._duration = 0.0
        self._integral = 0.0

    def update(self, d_left, d_right, encoder_delta, rate, dt, stationary=False):
        if not all(_finite(v) for v in (d_left, d_right, encoder_delta, rate, dt)) or not 0 < dt <= self.max_dt:
            self.clear_pending()
            self.status = "INVALID_SAMPLE"
            return None
        if abs(rate) > self.max_rate:
            self.clear_pending()
            self.status = "RATE_OUT_OF_RANGE"
            return None
        if self.bias is None:
            still = (stationary is True
                     and max(abs(d_left), abs(d_right)) / dt <= self.stationary_wheel_speed
                     and abs(rate) <= self.max_stationary_rate)
            if not still:
                self.clear_pending()
                self.status = "WAITING_FOR_STATIONARY"
                return None
            self._duration += dt
            self._integral += rate * dt
            self.status = "CALIBRATING"
            if self._duration >= self.calibration_duration:
                self.bias = self._integral / self._duration
                self.status = "READY"
            # Calibration never rotates or retroactively changes the pose.
            return None
        corrected = rate - self.bias
        if abs(corrected) > self.max_rate:
            self.status = "RATE_OUT_OF_RANGE"
            return None
        self.status = "FUSED"
        return (1 - self.weight) * encoder_delta + self.weight * corrected * dt


class Localizer:
    """Facade used by main.py. Swap internals here without touching main.py."""

    def __init__(self, home_pose, wheel_radius, axle_length, encoder_to_rad,
                 *, gyro_heading=None):
        self.odometry = DiffDriveOdometry(wheel_radius, axle_length, encoder_to_rad, home_pose)
        self.gyro_heading = gyro_heading
        self.heading_source = "ENCODER"

    @property
    def pose(self):
        return self.odometry.pose

    def reset(self, pose):
        """Reset pose, encoder reference and gyro calibration together."""
        self.odometry.reset(pose)
        if self.gyro_heading is not None:
            self.gyro_heading.reset()
        self.heading_source = "ENCODER"

    def update(self, encoders, gyro_yaw_rate=None, dt=None, *, stationary=False):
        """Fuse only when explicitly configured and calibrated; never replace encoders.

        stationary must be supplied by a caller holding the robot stopped.
        A missing encoder interval is dropped; the next valid sample rebases,
        avoiding integration of a multi-step encoder delta with one gyro dt.
        """
        odo = self.odometry
        self.heading_source = "ENCODER"
        if encoders is None or len(encoders) != 2 or not all(_finite(v) for v in encoders):
            odo._prev = None
            if self.gyro_heading is not None:
                self.gyro_heading.clear_pending()
                self.gyro_heading.status = "NO_ENCODERS"
            self.heading_source = "HOLD"
            return self.pose
        heading_delta = None
        if self.gyro_heading is not None:
            if odo._prev is None:
                self.gyro_heading.clear_pending()
            else:
                scale = odo.encoder_to_rad * odo.wheel_radius
                left = (encoders[0] - odo._prev[0]) * scale
                right = (encoders[1] - odo._prev[1]) * scale
                heading_delta = self.gyro_heading.update(
                    left, right, (right - left) / odo.axle_length,
                    gyro_yaw_rate, dt, stationary=stationary)
                if heading_delta is not None:
                    self.heading_source = "GYRO_FUSED"
        odo.update(*encoders, heading_delta=heading_delta)
        return self.pose
