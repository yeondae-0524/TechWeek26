import unittest

import _path  # noqa: F401
from scheduling import Periodic, StepTimer, period_to_steps, sensor_period_ms


class TestScheduling(unittest.TestCase):
    def test_period_to_steps(self):
        self.assertEqual(period_to_steps(0.128, 64), 2)
        self.assertEqual(period_to_steps(0.128, 32), 4)
        self.assertEqual(period_to_steps(0.01, 64), 1)  # never 0
        with self.assertRaises(ValueError):
            period_to_steps(0.1, 0)

    def test_sensor_period_is_timestep_multiple(self):
        self.assertEqual(sensor_period_ms(0.128, 64), 128)
        self.assertEqual(sensor_period_ms(0.1, 64), 128)  # 100 ms is not a 64 ms multiple
        self.assertEqual(sensor_period_ms(0.128, 32), 128)

    def test_periodic_on_64ms_steps(self):
        p = Periodic(0.128)
        runs = [p.due(k * 0.064) for k in range(8)]
        self.assertEqual(runs, [True, False, True, False, True, False, True, False])
        self.assertEqual(p.missed, 0)

    def test_periodic_counts_missed_runs(self):
        p = Periodic(0.128)
        self.assertTrue(p.due(0.0))
        self.assertTrue(p.due(0.5))  # deadlines 0.128, 0.256, 0.384 passed: one run
        self.assertEqual(p.missed, 2)
        self.assertFalse(p.due(0.51))  # next deadline 0.512 stays on the 0.128 grid
        self.assertTrue(p.due(0.512))

    def test_step_timer(self):
        t = StepTimer()
        self.assertIsNone(t.summary())
        for ms in range(1, 101):
            t.add(ms / 1000.0)
        med, p95, worst, n = t.summary()
        self.assertEqual(n, 100)
        self.assertAlmostEqual(med, 0.051)
        self.assertAlmostEqual(p95, 0.095)
        self.assertAlmostEqual(worst, 0.100)
        self.assertIsNone(t.summary())  # cleared


if __name__ == "__main__":
    unittest.main()
