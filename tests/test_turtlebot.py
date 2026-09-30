"""TurtleBot3 Burger profile: official values from docs/research/09 §3-§6."""

import math
import re
import unittest
from pathlib import Path

import _path  # noqa: F401
import config
import mapping
from control import DiffDriveController
from devices import Devices, parse_start_pose
from localization import DiffDriveOdometry

ROOT = Path(__file__).resolve().parents[1]


class Motor:
    def setVelocity(self, value):
        self.velocity = value


def make_controller():
    return DiffDriveController(Motor(), Motor(), config.WHEEL_RADIUS, config.AXLE_LENGTH,
                               config.MAX_WHEEL_SPEED, config.MAX_LINEAR_SPEED, config.MAX_ANGULAR_SPEED)


class TestTurtleBotProfile(unittest.TestCase):
    def test_official_geometry(self):
        self.assertEqual(config.WHEEL_RADIUS, 0.033)
        self.assertEqual(config.AXLE_LENGTH, 0.160)
        self.assertEqual(config.MAX_WHEEL_SPEED, 6.67)
        self.assertEqual(config.ROBOT_RADIUS_NOTEBOOK, 0.105)
        # safety circle = PROTO circumscribed radius rounded up, inflation 0.161 m
        self.assertEqual(config.ROBOT_RADIUS, 0.111)
        self.assertAlmostEqual(config.ROBOT_RADIUS + config.SAFETY_MARGIN, 0.161)
        self.assertGreaterEqual(config.ROBOT_RADIUS, 0.1103)

    def test_operating_limits_below_hardware(self):
        self.assertLessEqual(config.MAX_LINEAR_SPEED, config.WHEEL_RADIUS * config.MAX_WHEEL_SPEED)
        self.assertLessEqual(config.MAX_ANGULAR_SPEED,
                             2 * config.WHEEL_RADIUS * config.MAX_WHEEL_SPEED / config.AXLE_LENGTH)

    def test_sensor_rules(self):
        names = {v.lower() for v in config.DEVICE_NAMES.values() if v}
        self.assertFalse(any("gps" in n or "compass" in n for n in names))
        self.assertEqual(config.DEVICE_NAMES["lidar"], "LDS-01")
        for key in ("left_encoder", "right_encoder", "lidar"):
            self.assertIn(key, config.REQUIRED_DEVICES)
        self.assertNotIn("gyro", config.REQUIRED_DEVICES)  # IMU is optional
        self.assertEqual(config.GYRO_RAW_TO_RAD_S, 1.0)
        self.assertEqual(config.LIDAR_MOUNT_OFFSET, (-0.03, 0.0))

    def test_periods_are_seconds(self):
        self.assertFalse(hasattr(config, "MAP_UPDATE_PERIOD_STEPS"))
        self.assertAlmostEqual(config.MAP_UPDATE_PERIOD, 0.128)
        self.assertAlmostEqual(config.DETECTION_PERIOD, 0.128)

    def test_grid_covers_official_world_size(self):
        # apartment.wbt: x in [-12.4, 0], y in [-13.12, 0], start (-0.3, -7.5) [OFFICIAL]
        half_w = config.GRID_WIDTH * config.GRID_RESOLUTION / 2
        half_h = config.GRID_HEIGHT * config.GRID_RESOLUTION / 2
        self.assertGreaterEqual(half_w, max(abs(-12.4 + 0.3), abs(0.0 + 0.3)))
        self.assertGreaterEqual(half_h, max(abs(-13.12 + 7.5), abs(0.0 + 7.5)))


class TestTurtleBotKinematics(unittest.TestCase):
    def test_one_wheel_revolution_moves_correct_distance(self):
        odom = DiffDriveOdometry(config.WHEEL_RADIUS, config.AXLE_LENGTH, 1.0, (0, 0, 0))
        odom.update(0, 0)
        pose = odom.update(2 * math.pi, 2 * math.pi)
        self.assertAlmostEqual(pose[0], 2 * math.pi * 0.033)
        self.assertAlmostEqual(pose[1], 0)

    def test_turning_speed_uses_burger_axle(self):
        c = make_controller()
        c.set_velocity(0, 1)
        self.assertAlmostEqual(c.wheel_speeds[0], -0.08 / 0.033)
        self.assertAlmostEqual(c.wheel_speeds[1], 0.08 / 0.033)

    def test_v_and_w_limited_together(self):
        # max v and max w together exceed 6.67 rad/s on the outer wheel:
        # both are scaled down, curvature w/v is preserved.
        c = DiffDriveController(Motor(), Motor(), 0.033, 0.160, 6.67, 0.22, 2.75)
        c.set_velocity(0.22, 2.75)
        v, w = c.command
        self.assertAlmostEqual(w / v, 2.75 / 0.22)
        self.assertLessEqual(max(abs(s) for s in c.wheel_speeds), 6.67 + 1e-9)
        self.assertAlmostEqual(max(abs(s) for s in c.wheel_speeds), 6.67)

    def test_lidar_mount_transforms_with_heading(self):
        points = mapping.scan_to_world_points((1, 2, math.pi / 2), [float("inf")] * 360, 2 * math.pi, 3.5)
        self.assertAlmostEqual(points[180][0][0], 1)
        self.assertAlmostEqual(points[180][0][1], 2 - 0.03 + 3.5)

    def test_official_lidar_index_labels(self):
        # ranges[180]=front, [90]=left, [0]=back, [270]=right [OFFICIAL]
        def wrap(a):
            return math.atan2(math.sin(a), math.cos(a))
        n, fov = 360, 2 * math.pi
        self.assertAlmostEqual(wrap(mapping.lidar_angle(180, n, fov)), 0.0)
        self.assertAlmostEqual(wrap(mapping.lidar_angle(90, n, fov)), math.pi / 2)
        self.assertAlmostEqual(abs(wrap(mapping.lidar_angle(0, n, fov))), math.pi)
        self.assertAlmostEqual(wrap(mapping.lidar_angle(270, n, fov)), -math.pi / 2)


class TestDevicesWithoutWebots(unittest.TestCase):
    def test_gyro_values_are_already_radians(self):
        class Gyro:
            def getValues(self):
                return (0, 0, 1.25)
        d = object.__new__(Devices)
        d.gyro = Gyro()
        self.assertAlmostEqual(d.read_gyro_yaw_rate(), 1.25)

    def test_invalid_gyro_falls_back(self):
        class Gyro:
            def getValues(self):
                return (0, 0, float("nan"))
        d = object.__new__(Devices)
        d.gyro = Gyro()
        self.assertIsNone(d.read_gyro_yaw_rate())
        d.gyro = None
        self.assertIsNone(d.read_gyro_yaw_rate())

    def test_parse_start_pose(self):
        self.assertEqual(parse_start_pose('{"start_pose": [-0.5, -0.8, 0.0]}'), (-0.5, -0.8, 0.0))
        pose = parse_start_pose('{"start_pose": [1, 2, 4.0]}')
        self.assertAlmostEqual(pose[2], 4.0 - 2 * math.pi)  # normalised
        for bad in ("", "   ", "not json", '{"other": 1}', '{"start_pose": [1, 2]}',
                    '{"start_pose": [1, "x", 0]}', None):
            self.assertIsNone(parse_start_pose(bad))


class TestBaselineWorld(unittest.TestCase):
    def setUp(self):
        self.world = (ROOT / "worlds" / "rescue_baseline.wbt").read_text(encoding="utf-8")

    def test_world_controller_and_initial_pose_match(self):
        self.assertIn("TurtleBot3Burger {", self.world)
        self.assertNotIn("E-puck {", self.world)
        self.assertIn('controller "rescue_robot"', self.world)
        self.assertIn("translation -0.5 -0.8 0", self.world)
        self.assertEqual(config.START_POSE, (-0.5, -0.8, 0))
        custom = re.search(r'customData "(.*)"', self.world).group(1).replace('\\"', '"')
        self.assertEqual(parse_start_pose(custom), config.START_POSE)

    def test_world_has_no_gps_or_compass_nodes(self):
        self.assertIsNone(re.search(r"^\s*(GPS|Compass)\s*\{", self.world, re.M))

    def test_world_uses_official_driving_timestep(self):
        self.assertIn("basicTimeStep 64", self.world)

    def test_world_camera_matches_official_mount(self):
        self.assertIn("RobotisLds01 {", self.world)
        self.assertRegex(self.world, r"Camera \{\s*translation 0\.05 0 -0\.08\s*fieldOfView 1\.0472\s*"
                                     r"width 640\s*height 480")


if __name__ == "__main__":
    unittest.main()
