import math
import unittest

import _path  # noqa: F401
from localization import GyroHeading, Localizer


def fusion(**overrides):
    options = dict(calibration_duration=0.2, stationary_wheel_speed=0.001,
                   max_stationary_rate=0.1, max_rate=3.0, max_dt=0.2)
    options.update(overrides)
    return GyroHeading(**options)


class TestGyroLocalizer(unittest.TestCase):
    def make(self, weight=1.0):
        return Localizer((0, 0, 0), 0.033, 0.160, 1.0,
                         gyro_heading=fusion(weight=weight))

    def calibrate(self, loc):
        for _ in range(3):
            loc.update((0, 0), 0.02, 0.1, stationary=True)
        self.assertAlmostEqual(loc.gyro_heading.bias, 0.02)

    def test_default_keeps_encoder_heading(self):
        loc = Localizer((0, 0, 0), 0.033, 0.160, 1)
        loc.update((0, 0), 1.0, 0.1)
        self.assertEqual(loc.update((1, 1), 1.0, 0.1), (0.033, 0.0, 0.0))

    def test_requires_explicit_stationary_and_consecutive_samples(self):
        loc = self.make()
        for _ in range(5):
            loc.update((0, 0), 0.02, 0.1)
        self.assertIsNone(loc.gyro_heading.bias)
        loc.update((0, 0), 0.02, 0.1, stationary=True)
        loc.update((1, 1), 0.02, 0.1, stationary=True)
        loc.update((1, 1), 0.02, 0.1, stationary=True)
        self.assertIsNone(loc.gyro_heading.bias)
        loc.update((1, 1), 0.02, 0.1, stationary=True)
        self.assertAlmostEqual(loc.gyro_heading.bias, 0.02)

    def test_rotation_with_stopped_encoders_rejects_calibration(self):
        loc = self.make()
        for _ in range(5):
            loc.update((0, 0), 0.5, 0.1, stationary=True)
        self.assertIsNone(loc.gyro_heading.bias)

    def test_bias_is_time_weighted(self):
        g = fusion(calibration_duration=0.15)
        g.update(0, 0, 0, 0.01, 0.05, True)
        g.update(0, 0, 0, 0.04, 0.10, True)
        self.assertAlmostEqual(g.bias, 0.03)

    def test_heading_corrects_rotation_slip(self):
        loc = self.make()
        self.calibrate(loc)
        # Encoder turn is 0.2 rad; analytic gyro truth is 0.1 rad.
        wheel_angle = 0.2 * 0.160 / (2 * 0.033)
        pose = loc.update((-wheel_angle, wheel_angle), 1.02, 0.1)
        self.assertAlmostEqual(pose[2], 0.1)
        self.assertEqual(pose[:2], (0.0, 0.0))
        self.assertEqual(loc.heading_source, 'GYRO_FUSED')

    def test_translation_uses_corrected_midpoint(self):
        loc = self.make()
        self.calibrate(loc)
        x, y, theta = loc.update((1, 1), 1.02, 0.1)
        self.assertAlmostEqual(x, 0.033 * math.cos(0.05))
        self.assertAlmostEqual(y, 0.033 * math.sin(0.05))
        self.assertAlmostEqual(theta, 0.1)

    def test_invalid_gyro_or_dt_falls_back_without_losing_encoder_motion(self):
        for rate, dt in ((None, .1), (math.nan, .1), (math.inf, .1),
                         (100, .1), (.02, 0), (.02, math.nan), (.02, 1)):
            with self.subTest(rate=rate, dt=dt):
                loc = self.make()
                self.calibrate(loc)
                result = loc.update((0, 1), rate, dt)
                self.assertAlmostEqual(result[2], .033 / .160)
                self.assertEqual(loc.heading_source, 'ENCODER')

    def test_missing_encoder_holds_then_rebases_without_gyro_jump(self):
        loc = self.make()
        self.calibrate(loc)
        self.assertEqual(loc.update(None, 1.02, .1), (0, 0, 0))
        self.assertEqual(loc.update((5, 5), 1.02, .1), (0, 0, 0))
        self.assertAlmostEqual(loc.update((6, 6), .02, .1)[0], .033)

    def test_reset_clears_bias_and_reference(self):
        loc = self.make()
        self.calibrate(loc)
        loc.reset((1, 2, math.pi))
        self.assertIsNone(loc.gyro_heading.bias)
        self.assertEqual(loc.update((10, 10), .02, .1), (1, 2, math.pi))

    def test_weight_and_angle_wrap(self):
        loc = self.make(weight=0.5)
        self.calibrate(loc)
        loc.odometry.pose = (0, 0, math.pi - .01)
        pose = loc.update((0, 0), 1.02, .1)
        self.assertAlmostEqual(pose[2], -math.pi + .04)
        for options in ({'weight': 2}, {'max_dt': 0}, {'max_rate': math.nan}):
            with self.assertRaises(ValueError):
                fusion(**options)


if __name__ == '__main__':
    unittest.main()
