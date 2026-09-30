"""Differential-drive control baseline.

Implemented:
    * set_wheel_speeds / stop / drive_forward / rotate_left / rotate_right
    * set_velocity(v, w): unicycle -> wheel speeds using config geometry
    * emergency collision prevention hook (front sector of the LiDAR)
    * set_target_waypoint(): stores the waypoint for the future tracker

NOT implemented (TODO, feat/control):
    * waypoint / path tracking (follow_waypoint currently holds position)
    * local planner (DWA etc.), dynamic obstacle avoidance
    * recovery behaviour (stuck detection, back-off)

Motors are passed in (duck-typed: setVelocity), so this module is testable
without Webots.
"""

import math

from interfaces import is_valid_waypoint


class DiffDriveController:
    def __init__(self, left_motor, right_motor, wheel_radius, axle_length,
                 max_wheel_speed, max_linear_speed, max_angular_speed):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.max_wheel_speed = max_wheel_speed
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.waypoint = None
        self.command = (0.0, 0.0)      # last requested (v, w)
        self.wheel_speeds = (0.0, 0.0)  # last applied (left, right) rad/s
        self.emergency_stop_active = False

    # --------------------------------------------------------- low level
    def set_wheel_speeds(self, left, right):
        """Wheel angular velocities [rad/s], clamped to +/- max_wheel_speed."""
        lim = self.max_wheel_speed
        left = max(-lim, min(lim, left))
        right = max(-lim, min(lim, right))
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
        self.wheel_speeds = (left, right)

    def set_velocity(self, v, w):
        """Body velocity: v [m/s] forward, w [rad/s] CCW (+ = turn left)."""
        v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
        w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
        self.command = (v, w)
        half = self.axle_length / 2.0
        self.set_wheel_speeds((v - w * half) / self.wheel_radius, (v + w * half) / self.wheel_radius)

    # --------------------------------------------------------- primitives
    def stop(self):
        self.set_velocity(0.0, 0.0)

    def drive_forward(self, ratio=0.5):
        self.set_velocity(ratio * self.max_linear_speed, 0.0)

    def rotate_left(self, ratio=0.5):
        self.set_velocity(0.0, ratio * self.max_angular_speed)

    def rotate_right(self, ratio=0.5):
        self.set_velocity(0.0, -ratio * self.max_angular_speed)

    # --------------------------------------------------------- safety hook
    def apply_emergency_stop(self, min_front_distance, stop_distance):
        """Call every step AFTER commanding motion.

        If an obstacle is closer than stop_distance in front and the robot is
        commanded forward, forward motion is cancelled (rotation in place is
        still allowed so the robot can turn away). Returns True if triggered.
        min_front_distance=None (no sensor) disables the hook.
        """
        v, w = self.command
        triggered = (min_front_distance is not None and min_front_distance < stop_distance and v > 0.0)
        if triggered:
            self.set_velocity(0.0, w)
        self.emergency_stop_active = triggered
        return triggered

    # --------------------------------------------------------- waypoint API
    def set_target_waypoint(self, waypoint):
        """waypoint = (x, y) in world metres, or None to clear."""
        if waypoint is not None and not is_valid_waypoint(waypoint):
            raise ValueError(f"invalid waypoint: {waypoint!r}")
        self.waypoint = waypoint

    def follow_waypoint(self, pose):
        """TODO(feat/control): track self.waypoint from pose (x, y, theta).

        Not implemented yet: holds position so an unfinished tracker can never
        drive the robot. Returns False (waypoint not reached).
        """
        self.stop()
        return False


def min_front_distance(ranges, angles, half_angle):
    """Smallest finite range whose ray angle is within +/- half_angle of forward."""
    best = None
    for r, a in zip(ranges, angles):
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(a) <= half_angle and math.isfinite(r) and (best is None or r < best):
            best = r
    return best
