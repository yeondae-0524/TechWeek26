"""SafetyMonitor on the TurtleBot3 footprint (docs/research/05 §3, §6)."""

import math
import unittest

import _path  # noqa: F401
import config
import mapping
from control import SafetyMonitor

N = 360
FOV = 2 * math.pi
ANGLES = [mapping.lidar_angle(i, N, FOV) for i in range(N)]
INF = float("inf")
FRONT, LEFT, BACK, RIGHT = 180, 90, 0, 270


def monitor(**overrides):
    m = SafetyMonitor.from_config(config)
    for key, value in overrides.items():
        setattr(m, key, value)
    return m


def scan(**by_index):
    ranges = [INF] * N
    for index, r in by_index.items():
        ranges[int(index.lstrip("i"))] = r
    return ranges


def lidar_range_for(center_distance):
    """Range reported by the forward ray for an object at center_distance ahead of the axle."""
    return center_distance - config.LIDAR_MOUNT_OFFSET[0]


class TestSafetyMonitor(unittest.TestCase):
    def test_clear_scan_passes(self):
        m = monitor()
        m.update_scan(scan(i180=2.0, i179=2.0), ANGLES, 0.0)
        self.assertEqual(m.filter(0.1, 0.0, 0.0), (0.1, 0.0, None))

    def test_stop_zone_blocks_forward(self):
        m = monitor()
        r = lidar_range_for(config.ROBOT_RADIUS + 0.03)
        m.update_scan(scan(i179=r, i180=r, i181=r), ANGLES, 0.0)
        v, w, event = m.filter(0.1, 0.2, 0.0)
        self.assertEqual((v, event), (0.0, "STOP_ZONE"))
        self.assertEqual(w, 0.2)

    def test_stop_zone_grows_with_speed(self):
        m = monitor()
        self.assertGreater(m.stop_distance(0.15), m.stop_distance(0.0))
        self.assertAlmostEqual(m.stop_distance(0.0), config.ROBOT_RADIUS + config.SAFETY_STOP_MARGIN)

    def test_obstacle_behind_does_not_block_forward(self):
        m = monitor()
        m.update_scan(scan(i0=0.15, i1=0.15, i359=0.15), ANGLES, 0.0)
        self.assertEqual(m.filter(0.1, 0.0, 0.0)[2], None)

    def test_min_points_filters_single_far_point(self):
        m = monitor()
        # one point inside the STOP zone but outside the single-point veto distance
        r = lidar_range_for(config.ROBOT_RADIUS + config.SAFETY_STOP_MARGIN - 0.005)
        m.update_scan(scan(i180=r), ANGLES, 0.0)
        self.assertEqual(m.filter(0.05, 0.0, 0.0)[2], None)

    def test_single_close_point_always_stops(self):
        m = monitor()
        m.update_scan(scan(i180=lidar_range_for(config.ROBOT_RADIUS + 0.01)), ANGLES, 0.0)
        self.assertEqual(m.filter(0.05, 0.0, 0.0)[2], "STOP_ZONE")

    def test_inf_after_close_hit_is_blind_zone(self):
        # object approaches until it disappears inside the LDS-01 minRange
        m = monitor()
        m.update_scan(scan(i180=0.2), ANGLES, 0.0)
        m.update_scan(scan(), ANGLES, 0.064)
        self.assertEqual(m.filter(0.05, 0.0, 0.064)[2], "BLIND_ZONE")
        m.update_scan(scan(), ANGLES, 0.128)  # still inf -> still blocked
        self.assertEqual(m.filter(0.05, 0.0, 0.128)[2], "BLIND_ZONE")
        m.update_scan(scan(i180=1.0), ANGLES, 0.192)  # seen far again -> released
        self.assertEqual(m.filter(0.05, 0.0, 0.192)[2], None)

    def test_inf_after_far_hit_is_not_blind_zone(self):
        m = monitor()
        m.update_scan(scan(i180=1.0), ANGLES, 0.0)
        m.update_scan(scan(), ANGLES, 0.064)
        self.assertEqual(m.filter(0.05, 0.0, 0.064)[2], None)

    def test_reverse_blocked(self):
        m = monitor()
        m.update_scan(scan(), ANGLES, 0.0)
        self.assertEqual(m.filter(-0.05, 0.0, 0.0), (0.0, 0.0, "REVERSE_BLOCKED"))

    def test_spin_needs_clearance(self):
        m = monitor()
        m.update_scan(scan(i270=0.2), ANGLES, 0.0)  # side point ~0.2 m from the axle
        self.assertEqual(m.filter(0.0, 1.0, 0.0), (0.0, 1.0, None))
        m.update_scan(scan(i180=0.15), ANGLES, 0.0)  # 0.12 m ahead of the axle
        self.assertEqual(m.filter(0.0, 1.0, 0.0), (0.0, 0.0, "SPIN_BLOCKED"))

    def test_stale_or_missing_scan_stops(self):
        m = monitor()
        self.assertEqual(m.filter(0.1, 0.5, 0.0), (0.0, 0.0, "STALE_SCAN"))
        m.update_scan(scan(i180=2.0), ANGLES, 0.0)
        self.assertEqual(m.filter(0.1, 0.0, config.SENSOR_STALE_TIMEOUT + 0.1)[2], "STALE_SCAN")
        m.update_scan(None, ANGLES, 1.0)  # invalid scan does not refresh
        self.assertEqual(m.filter(0.1, 0.0, 1.0)[2], "STALE_SCAN")
        self.assertEqual(m.filter(0.0, 0.0, 1.0), (0.0, 0.0, None))  # standing still is fine

    def test_stop_disc_is_outside_lidar_blind_zone(self):
        # Every boundary point of the STOP half-disc ahead of the axle must be
        # farther than minRange (0.12 m) from the LiDAR origin, or it is invisible.
        ox, oy = config.LIDAR_MOUNT_OFFSET
        r = config.ROBOT_RADIUS + config.SAFETY_STOP_MARGIN
        for deg in range(-90, 91):
            a = math.radians(deg)
            d = math.hypot(r * math.cos(a) - ox, r * math.sin(a) - oy)
            self.assertGreater(d, 0.12)


if __name__ == "__main__":
    unittest.main()
