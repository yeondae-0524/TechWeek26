"""Differential-drive control baseline.

Implemented:
    * set_wheel_speeds / stop / drive_forward / rotate_left / rotate_right
    * set_velocity(v, w): unicycle -> wheel speeds using config geometry;
      v and w are limited together by the wheel speed limit (curvature kept)
    * SafetyMonitor: raw-LiDAR STOP filter applied last every step
      (TurtleBot3 footprint, LDS-01 minRange blind zone, min_points,
      spin clearance, no reversing, stale scan -> stop). docs/research/05 §3
    * legacy emergency stop hook (front sector of the LiDAR)
    * set_target_waypoint(): stores the waypoint for the future tracker

NOT implemented (TODO, feat/control):
    * waypoint / path tracking (follow_waypoint currently holds position)
    * RPP-lite path follower, SLOWDOWN zone, 1 s forward projection
    * ProgressMonitor / recovery behaviour (stuck detection, spin, back-off)

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
        """Body velocity: v [m/s] forward, w [rad/s] CCW (+ = turn left).

        v and w are clamped to their operating limits, then scaled down
        together if a wheel would exceed max_wheel_speed, so the commanded
        curvature is preserved: |(v +/- w*L/2) / r| <= max_wheel_speed.
        """
        v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
        w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
        half = self.axle_length / 2.0
        peak = (abs(v) + abs(w) * half) / self.wheel_radius
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            v, w = v * scale, w * scale
        self.command = (v, w)
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


# ---------------------------------------------------------------------------
# Safety monitor (Nav2 Collision Monitor idea: raw scan, bypasses the map)
# ---------------------------------------------------------------------------
class SafetyMonitor:
    """Filters (v, w) with the latest raw LiDAR scan. Call every step, last.

    Obstacle points are expressed in the robot frame centred on the wheel
    axle (LiDAR mount offset applied), so the safety circle ``robot_radius``
    is the conservative TurtleBot3 footprint.

    Rules (all distances from config, [INITIAL TUNING]):
      * forward (v > 0): STOP if >= min_points points ahead of the axle are
        inside robot_radius + stop_margin + v * reaction_time, or a single
        point is inside single_point_distance.
      * blind zone: a ray whose last finite return was ahead of the axle and
        inside blind_arm_distance and that now reads inf/NaN is an object
        inside the LDS-01 minRange -> STOP (inf is never treated as free).
      * rotation in place: blocked if a point is inside spin_clearance
        (behind the axle this is mostly inside the LiDAR blind zone, so a
        clear check there is not proof of free space - research 12 common).
      * reverse (v < 0): blocked unless allow_reverse (rear is mostly blind).
      * no valid scan for stale_timeout seconds -> all motion blocked.

    filter() returns (v, w, event) with event None when nothing was changed.
    """

    def __init__(self, robot_radius, stop_margin, reaction_time, front_half_angle, min_points,
                 single_point_distance, blind_arm_distance, spin_clearance, allow_reverse,
                 stale_timeout, mount_offset):
        self.robot_radius = robot_radius
        self.stop_margin = stop_margin
        self.reaction_time = reaction_time
        self.front_half_angle = front_half_angle
        self.min_points = max(1, int(min_points))
        self.single_point_distance = single_point_distance
        self.blind_arm_distance = blind_arm_distance
        self.spin_clearance = spin_clearance
        self.allow_reverse = allow_reverse
        self.stale_timeout = stale_timeout
        self.mount_offset = mount_offset
        self._armed = None          # per-ray flag: last finite return was close, ahead
        self._blind = []            # armed rays that now read inf/NaN
        self._last_scan_time = None
        self.points = []            # [(x, y, distance, bearing, index)] of the last scan

    @classmethod
    def from_config(cls, cfg):
        return cls(cfg.ROBOT_RADIUS, cfg.SAFETY_STOP_MARGIN, cfg.SAFETY_REACTION_TIME,
                   cfg.SAFETY_FRONT_HALF_ANGLE, cfg.SAFETY_MIN_POINTS,
                   cfg.SAFETY_SINGLE_POINT_DISTANCE, cfg.SAFETY_BLIND_ARM_DISTANCE,
                   cfg.SAFETY_SPIN_CLEARANCE, cfg.SAFETY_ALLOW_REVERSE,
                   cfg.SENSOR_STALE_TIMEOUT, cfg.LIDAR_MOUNT_OFFSET)

    def update_scan(self, ranges, angles, now):
        """Store a new scan (ranges may be None when the LiDAR data is invalid)."""
        if ranges is None or angles is None or len(ranges) != len(angles):
            return
        self._last_scan_time = now
        if self._armed is None or len(self._armed) != len(ranges):
            self._armed = [False] * len(ranges)
        ox, oy = self.mount_offset
        self.points = []
        self._blind = []
        for i, (r, a) in enumerate(zip(ranges, angles)):
            if math.isfinite(r):
                x, y = ox + r * math.cos(a), oy + r * math.sin(a)
                d = math.hypot(x, y)
                bearing = math.atan2(y, x)
                self.points.append((x, y, d, bearing, i))
                self._armed[i] = d < self.blind_arm_distance and x > 0.0
            elif self._armed[i]:
                self._blind.append(i)  # object went into the blind zone: keep the ray armed

    def stop_distance(self, v):
        return self.robot_radius + self.stop_margin + max(0.0, v) * self.reaction_time

    def filter(self, v, w, now):
        if self._last_scan_time is None or now - self._last_scan_time > self.stale_timeout:
            return (0.0, 0.0, "STALE_SCAN") if (v, w) != (0.0, 0.0) else (v, w, None)
        event = None
        if v < 0.0 and not self.allow_reverse:
            v, event = 0.0, "REVERSE_BLOCKED"
        if v > 0.0:
            reason = self._forward_blocked(v)
            if reason:
                v, event = 0.0, reason
        if v == 0.0 and w != 0.0 and self._spin_blocked():
            w, event = 0.0, "SPIN_BLOCKED"
        return v, w, event

    def _forward_blocked(self, v):
        if self._blind:
            return "BLIND_ZONE"
        limit = self.stop_distance(v)
        ahead = [p for p in self.points if abs(p[3]) <= self.front_half_angle and p[0] > 0.0]
        close = [p for p in ahead if p[2] < limit]
        if len(close) >= self.min_points:
            return "STOP_ZONE"
        if any(p[2] < self.single_point_distance for p in ahead):
            return "STOP_ZONE"
        return None

    def _spin_blocked(self):
        return any(p[2] < self.spin_clearance for p in self.points)
