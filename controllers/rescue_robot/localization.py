"""Pose estimation baseline: pose = (x, y, theta).

Implemented:
    * pose interface + reset to the provided start pose (home_pose)
    * differential-drive wheel odometry, parameterised by wheel radius,
      axle length and encoder unit conversion (all injected from config)

NOT implemented (TODO, feat/localization):
    * gyro fusion for heading (gyro yaw rate is read by devices.py but unused)
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

    def update(self, left_enc, right_enc):
        """Update from absolute encoder readings; returns the new pose.

        The first call only stores the reference readings.
        """
        if self._prev is None:
            self._prev = (left_enc, right_enc)
            return self.pose
        d_left = (left_enc - self._prev[0]) * self.encoder_to_rad * self.wheel_radius
        d_right = (right_enc - self._prev[1]) * self.encoder_to_rad * self.wheel_radius
        self._prev = (left_enc, right_enc)
        self.pose = integrate_diff_drive(self.pose, d_left, d_right, self.axle_length)
        return self.pose


def integrate_diff_drive(pose, d_left, d_right, axle_length):
    """Midpoint integration of wheel travel distances [m] (+theta = CCW)."""
    x, y, th = pose
    d_center = (d_left + d_right) / 2.0
    d_theta = (d_right - d_left) / axle_length
    mid = th + d_theta / 2.0
    return make_pose(x + d_center * math.cos(mid), y + d_center * math.sin(mid), th + d_theta)


class Localizer:
    """Facade used by main.py. Swap internals here without touching main.py."""

    def __init__(self, home_pose, wheel_radius, axle_length, encoder_to_rad):
        self.odometry = DiffDriveOdometry(wheel_radius, axle_length, encoder_to_rad, home_pose)

    @property
    def pose(self):
        return self.odometry.pose

    def update(self, encoders, gyro_yaw_rate=None, dt=None):
        """encoders: (left, right) or None. gyro_yaw_rate is accepted but unused (TODO)."""
        if encoders is not None:
            self.odometry.update(*encoders)
        # TODO(feat/localization): fuse gyro_yaw_rate * dt into heading.
        # TODO(feat/localization): scan-matching correction using the map.
        return self.pose
