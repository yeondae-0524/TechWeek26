"""World copies that run rescue_robot (official originals stay unmodified)."""

import math
import os
import re
import unittest

import _path  # noqa: F401
from devices import parse_start_pose

WORLDS = os.path.join(os.path.dirname(__file__), "..", "worlds")
# copy -> (official original, expected start pose from the original robot node)
RESCUE_WORLDS = {
    "apartment_rescue": ("apartment", (-0.3, -7.5, math.pi)),
    "breakroom_teleop_rescue": ("breakroom_teleop", (-1.2652, 1.8113, -0.4244)),
}


def read(name):
    with open(os.path.join(WORLDS, name + ".wbt"), encoding="utf-8") as f:
        return f.read()


def robot_node(text):
    start = text.index("\nTurtleBot3Burger {")
    return text[start:text.index("\n}\n", start)]


class TestRescueWorlds(unittest.TestCase):
    def test_robot_runs_rescue_robot_with_start_pose(self):
        for name, (_, (x, y, th)) in RESCUE_WORLDS.items():
            with self.subTest(world=name):
                robot = robot_node(read(name))
                self.assertIn('controller "rescue_robot"', robot)
                self.assertNotIn("supervisor", robot)
                custom = re.search(r'customData "(.*)"', robot).group(1).replace('\\"', '"')
                pose = parse_start_pose(custom)
                self.assertAlmostEqual(pose[0], x, places=3)
                self.assertAlmostEqual(pose[1], y, places=3)
                self.assertAlmostEqual(math.cos(pose[2] - th), 1.0, places=5)

    def test_only_robot_controller_lines_differ_from_original(self):
        for name, (original, _) in RESCUE_WORLDS.items():
            with self.subTest(world=name):
                def strip(text):
                    return [l for l in text.splitlines()
                            if not re.match(r"\s*(controller|customData|supervisor)\s", l)]
                self.assertEqual(strip(read(name)), strip(read(original)))


if __name__ == "__main__":
    unittest.main()
