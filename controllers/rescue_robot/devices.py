"""All Webots device access for the rescue robot.

Only this file (and main.py) import the Webots ``controller`` module, so the
rest of the code base can be unit-tested without Webots.

When the robot model changes, edit ``config.DEVICE_NAMES`` first; this file
should only need changes if a sensor type changes (e.g. ticks vs radians).

Sensor rules [ORGANIZER]: encoders + 2D LiDAR mandatory (missing -> DeviceError,
the robot never moves), IMU optional, Compass / GPS never enabled.
"""

import json
import math

import config
from interfaces import make_pose
from scheduling import sensor_period_ms


class DeviceError(RuntimeError):
    """A required Webots device is missing or a forbidden device is configured."""


def parse_start_pose(custom_data):
    """Parse ``{"start_pose": [x, y, theta]}`` from a robot customData string.

    Returns a pose tuple, or None if the text is empty / has no valid start_pose.
    Never raises: a malformed customData must not crash the controller.
    """
    text = (custom_data or "").strip()
    if not text:
        return None
    try:
        values = json.loads(text)["start_pose"]
        pose = make_pose(*values)
    except (ValueError, TypeError, KeyError):
        print(f"[devices] WARNING: ignoring customData without a valid start_pose: {text!r}")
        return None
    if not all(math.isfinite(v) for v in pose):
        print(f"[devices] WARNING: ignoring non-finite start_pose in customData: {text!r}")
        return None
    return pose


class Devices:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        self.available = self._list_device_names()
        print("[devices] available:", ", ".join(sorted(self.available)) or "(none)")

        forbidden = [f"{key} -> {name!r}" for key, name in config.DEVICE_NAMES.items()
                     if name and any(bad in name.lower() for bad in config.FORBIDDEN_DEVICE_NAMES)]
        if forbidden:
            raise DeviceError("Compass/GPS are not allowed as controller inputs (organizer rule): "
                              + "; ".join(forbidden) + ". Remove them from config.DEVICE_NAMES.")

        missing_required = []
        for key in config.REQUIRED_DEVICES:
            name = config.DEVICE_NAMES.get(key)
            if not name or name not in self.available:
                missing_required.append(f"{key} -> {name!r}")
        if missing_required:
            raise DeviceError(
                "Required device(s) not found: " + "; ".join(missing_required)
                + ". Fix config.DEVICE_NAMES. Available devices: "
                + ", ".join(sorted(self.available))
            )

        self.left_motor = self._get("left_motor")
        self.right_motor = self._get("right_motor")
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float("inf"))  # velocity control mode
            motor.setVelocity(0.0)

        # Encoders, LiDAR (safety monitor) and gyro are read every step.
        self.left_encoder = self._get_enabled("left_encoder")
        self.right_encoder = self._get_enabled("right_encoder")
        self.lidar = self._get_enabled("lidar")
        self.gyro = self._get_enabled("gyro")
        self.accelerometer = self._get_enabled("accelerometer")
        # The camera is only read at DETECTION_PERIOD (640x480 frames are costly).
        self.camera_period_ms = sensor_period_ms(config.DETECTION_PERIOD, timestep)
        self.camera = self._get_enabled("camera", self.camera_period_ms)

        self.lidar_fov = self.lidar.getFov()
        self.lidar_resolution = self.lidar.getHorizontalResolution()
        self.lidar_min_range = self.lidar.getMinRange()
        self.lidar_max_range = self.lidar.getMaxRange()
        self._print_startup_log()

    # ------------------------------------------------------------------ setup
    def _list_device_names(self):
        names = set()
        for i in range(self.robot.getNumberOfDevices()):
            names.add(self.robot.getDeviceByIndex(i).getName())
        return names

    def _get(self, key):
        name = config.DEVICE_NAMES.get(key)
        if not name:
            return None
        if name not in self.available:
            print(f"[devices] WARNING: optional device '{key}' ({name!r}) not found -> disabled")
            return None
        return self.robot.getDevice(name)

    def _get_enabled(self, key, period_ms=None):
        device = self._get(key)
        if device is not None:
            device.enable(period_ms or self.timestep)
        return device

    def _print_startup_log(self):
        sync = self.robot.getSynchronization() if hasattr(self.robot, "getSynchronization") else "?"
        print(f"[devices] basicTimeStep={self.timestep} ms, synchronization={sync}")
        print(f"[devices] lidar {config.DEVICE_NAMES['lidar']!r}: {self.lidar_resolution} rays, "
              f"fov={self.lidar_fov:.3f} rad, minRange={self.lidar_min_range} m, "
              f"maxRange={self.lidar_max_range} m, mount={config.LIDAR_MOUNT_OFFSET}")
        if self.camera is not None:
            print(f"[devices] camera: {self.camera.getWidth()} x {self.camera.getHeight()}, "
                  f"fov={self.camera.getFov():.4f} rad, period={self.camera_period_ms} ms")
        else:
            print("[devices] WARNING: no camera -> detection disabled")
        print("[devices] gyro: " + ("enabled (team choice, not fused yet)" if self.gyro is not None
                                    else "absent -> encoder-only odometry"))
        print("[devices] compass/GPS: not used (organizer rule)")

    # ------------------------------------------------------------------ reads
    def read_encoders(self):
        """(left, right) wheel encoder values in config.ENCODER_UNITS, or None if invalid."""
        left, right = self.left_encoder.getValue(), self.right_encoder.getValue()
        if not (math.isfinite(left) and math.isfinite(right)):  # NaN on the first step after enable()
            return None
        return left, right

    def read_lidar(self):
        """List of ranges [m] (inf = no return / inside minRange), or None if invalid."""
        ranges = self.lidar.getRangeImage()
        if not ranges or len(ranges) != self.lidar_resolution:
            return None
        return list(ranges)

    def read_gyro_yaw_rate(self):
        """Yaw rate [rad/s] (+ = CCW), or None (absent or invalid -> encoder-only)."""
        if self.gyro is None:
            return None
        value = self.gyro.getValues()[config.GYRO_YAW_AXIS] * config.GYRO_RAW_TO_RAD_S
        return value if math.isfinite(value) else None

    def read_accelerometer(self):
        if self.accelerometer is None:
            return None
        return tuple(self.accelerometer.getValues())

    def read_start_pose(self):
        """Start pose from the robot's customData (see config.START_POSE_FROM_CUSTOM_DATA), or None."""
        if not config.START_POSE_FROM_CUSTOM_DATA:
            return None
        return parse_start_pose(self.robot.getCustomData())

    def read_camera_frame(self):
        """Camera image as a NumPy BGR array (H, W, 3), or None.

        NumPy is imported lazily so the rest of the baseline has no hard
        dependency on it.
        """
        if self.camera is None:
            return None
        image = self.camera.getImage()
        if image is None:
            return None
        import numpy as np
        h, w = self.camera.getHeight(), self.camera.getWidth()
        return np.frombuffer(image, np.uint8).reshape((h, w, 4))[:, :, :3]  # BGRA -> BGR
