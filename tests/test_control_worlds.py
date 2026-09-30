"""휴게실 시험 복사본이 기본 설정과 무관하게 제공된 시작 pose를 전달하는지 확인합니다."""
import math
import re
import unittest
from pathlib import Path

import _path  # noqa: F401
from devices import parse_start_pose

ROOT = Path(__file__).resolve().parents[1]


class ControlWorldStartPoseTests(unittest.TestCase):
    def test_breakroom_copy_has_its_own_provided_start_pose(self):
        world = (ROOT / "worlds" / "breakroom_control_test.wbt").read_text(encoding="utf-8")
        custom = re.search(r'customData "(.*)"', world).group(1).replace('\\"', '"')
        pose = parse_start_pose(custom)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0], -1.265)
        self.assertAlmostEqual(pose[1], 1.811)
        self.assertAlmostEqual(pose[2], math.radians(-24.3))
        self.assertIn('controller "rescue_robot"', world)
        self.assertNotIn('supervisor TRUE', world)


if __name__ == "__main__":
    unittest.main()
