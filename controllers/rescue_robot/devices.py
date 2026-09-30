"""All Webots device access for the rescue robot.

Only this file (and main.py) import the Webots ``controller`` module, so the
rest of the code base can be unit-tested without Webots.

When the robot model changes, edit ``config.DEVICE_NAMES`` first; this file
should only need changes if a sensor type changes (e.g. ticks vs radians).
"""

import math

import config


class DeviceError(RuntimeError):
    """A required Webots device is missing."""


class Devices:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        self.available = self._list_device_names()
        print("[devices] available:", ", ".join(sorted(self.available)) or "(none)")

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

        self.left_encoder = self._get_enabled("left_encoder")
        self.right_encoder = self._get_enabled("right_encoder")
        self.camera = self._get_enabled("camera")
        self.lidar = self._get_enabled("lidar")
        self.gyro = self._get_enabled("gyro")
        self.accelerometer = self._get_enabled("accelerometer")
        self.compass = self._get_enabled("compass")
        self.gps_debug = self._get_enabled("gps_debug")

        if self.lidar is not None:
            self.lidar_fov = self.lidar.getFov()
            self.lidar_resolution = self.lidar.getHorizontalResolution()
            self.lidar_max_range = self.lidar.getMaxRange()
            print(f"[devices] lidar: {self.lidar_resolution} rays, fov={self.lidar_fov:.3f} rad, "
                  f"maxRange={self.lidar_max_range} m")
        if self.camera is not None:
            print(f"[devices] camera: {self.camera.getWidth()} x {self.camera.getHeight()}")

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

    def _get_enabled(self, key):
        device = self._get(key)
        if device is not None:
            device.enable(self.timestep)
        return device

    # ------------------------------------------------------------------ reads
    def read_encoders(self):
        """(left, right) wheel encoder values in config.ENCODER_UNITS, or None."""
        if self.left_encoder is None or self.right_encoder is None:
            return None
        left, right = self.left_encoder.getValue(), self.right_encoder.getValue()
        if math.isnan(left) or math.isnan(right):  # first step after enable()
            return None
        return left, right

    def read_lidar(self):
        """List of ranges [m] (inf = no hit), or None if no lidar."""
        if self.lidar is None:
            return None
        return list(self.lidar.getRangeImage())

    def read_gyro_yaw_rate(self):
        """Yaw rate [rad/s] (+ = CCW), or None."""
        if self.gyro is None:
            return None
        return self.gyro.getValues()[config.GYRO_YAW_AXIS] * config.GYRO_RAW_TO_RAD_S

    def read_accelerometer(self):
        if self.accelerometer is None:
            return None
        return tuple(self.accelerometer.getValues())

    def read_gps_debug(self):
        """Ground-truth (x, y) for DEBUG/VERIFICATION ONLY. Never use for localization."""
        if self.gps_debug is None:
            return None
        x, y, _ = self.gps_debug.getValues()
        if math.isnan(x):
            return None
        return (x, y)

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
