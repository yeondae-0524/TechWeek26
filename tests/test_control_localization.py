import math
import unittest

import _path  # noqa: F401
from control import DiffDriveController, min_front_distance
from localization import DiffDriveOdometry, integrate_diff_drive


class FakeMotor:
    def __init__(self):
        self.velocity = None

    def setVelocity(self, v):
        self.velocity = v


def make_controller():
    return DiffDriveController(FakeMotor(), FakeMotor(), wheel_radius=0.02, axle_length=0.052,
                               max_wheel_speed=6.28, max_linear_speed=0.08, max_angular_speed=1.5)


class TestControl(unittest.TestCase):
    def test_primitives(self):
        c = make_controller()
        c.drive_forward(1.0)
        self.assertAlmostEqual(c.left_motor.velocity, 4.0)
        self.assertAlmostEqual(c.right_motor.velocity, 4.0)
        c.rotate_left()
        self.assertLess(c.left_motor.velocity, 0)
        self.assertGreater(c.right_motor.velocity, 0)
        c.rotate_right()
        self.assertGreater(c.left_motor.velocity, 0)
        self.assertLess(c.right_motor.velocity, 0)
        c.stop()
        self.assertEqual((c.left_motor.velocity, c.right_motor.velocity), (0.0, 0.0))

    def test_wheel_speed_clamp(self):
        c = make_controller()
        c.set_wheel_speeds(100, -100)
        self.assertEqual(c.wheel_speeds, (6.28, -6.28))

    def test_emergency_stop(self):
        c = make_controller()
        c.drive_forward()
        self.assertTrue(c.apply_emergency_stop(0.05, 0.08))
        self.assertEqual(c.wheel_speeds, (0.0, 0.0))
        c.rotate_left()  # turning in place is still allowed
        self.assertFalse(c.apply_emergency_stop(0.05, 0.08))
        c.drive_forward()
        self.assertFalse(c.apply_emergency_stop(None, 0.08))  # no sensor -> no-op

    def test_waypoint_stub_holds_position(self):
        c = make_controller()
        c.set_target_waypoint((1.0, 2.0))
        self.assertEqual(c.waypoint, (1.0, 2.0))
        c.drive_forward()
        self.assertFalse(c.follow_waypoint((0.0, 0.0, 0.0)))
        self.assertEqual(c.wheel_speeds, (0.0, 0.0))
        with self.assertRaises(ValueError):
            c.set_target_waypoint((1.0,))

    def test_min_front_distance(self):
        angles = [0.0, math.radians(20), math.radians(90), -math.radians(25)]
        ranges = [1.0, 0.5, 0.1, float("inf")]
        self.assertEqual(min_front_distance(ranges, angles, math.radians(30)), 0.5)


class TestOdometry(unittest.TestCase):
    def test_straight(self):
        odo = DiffDriveOdometry(0.02, 0.052, 1.0, (0.0, 0.0, 0.0))
        odo.update(0.0, 0.0)
        x, y, th = odo.update(10.0, 10.0)  # 10 rad * 0.02 m = 0.2 m
        self.assertAlmostEqual(x, 0.2)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(th, 0.0)

    def test_rotate_in_place_ccw(self):
        # right wheel forward, left backward -> theta increases (turn left)
        axle = 0.052
        wheel_dist = math.pi / 2 * axle / 2
        x, y, th = integrate_diff_drive((0.0, 0.0, 0.0), -wheel_dist, wheel_dist, axle)
        self.assertAlmostEqual(th, math.pi / 2)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)

    def test_invalid_params(self):
        with self.assertRaises(ValueError):
            DiffDriveOdometry(0.0, 0.052, 1.0, (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
