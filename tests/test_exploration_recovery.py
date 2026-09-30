"""Real mission wiring with fake devices; no Webots API or ground truth."""
import subprocess
import sys
import unittest

from test_rescue_robot_main import FOLDER, SETUP


class ExplorationRecoveryTests(unittest.TestCase):
    def run_case(self, code):
        script = "FOLDER = " + repr(str(FOLDER)) + "\n" + SETUP + "\n" + code
        result = subprocess.run([sys.executable, "-X", "utf8", "-B", "-c", script],
                                text=True, capture_output=True, encoding="utf-8", timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_blocked_goal_retries_are_bounded_and_excluded(self):
        self.run_case('''robot, mission = make("MISSION")
mission.explore_goal = (20, 25)
xy = mission.grid.grid_to_world(20,25)
mission.recovery.begin(xy)
mission.set_navigation_path([xy])
with patch.object(mission.navigation, "compute", return_value=(0,0,"REPLAN_REQUIRED")):
    mission.do_explore(1, (0,0,0))
assert mission.recovery.stage == "WAIT_REPLAN"
assert not mission.navigation.follower.path
with patch.object(main.planning, "astar") as astar:
    mission.do_explore(1.5, (0,0,0))
    astar.assert_not_called()
mission.do_explore(2, (0,0,0))
mission.do_explore(3, (0,0,0))
assert mission.explore_goal is None
assert mission.recovery.excluded(xy, 3)
assert mission.controller.command == (0,0)
''')

    def test_arrival_clears_path_scans_and_selects_again(self):
        self.run_case('''robot, mission = make("MISSION")
mission.explore_goal = (20,20)
mission.recovery.begin(mission.grid.grid_to_world(20,20))
mission.set_navigation_path([(0,0)])
mission.do_explore(1, (0,0,0))
assert not mission.navigation.follower.path
assert mission.view_scan_remaining is not None
mission.last_camera_time = 1.1
mission.do_explore(1.1, (0,0,0))
assert mission.controller.command == (0,config.NAV_ROTATE_SPEED)
# A stalled turn cannot run forever.
mission.last_camera_time = 12
mission.do_explore(12, (0,0,0))
assert mission.view_scan_remaining is None
assert mission.explore_goal is None
assert mission.controller.command == (0,0)
''')

    def test_missing_camera_does_not_turn_or_claim_coverage(self):
        self.run_case('''robot, mission = make("MISSION")
mission.set_navigation_path([(0,0)])
mission.do_explore(1, (0,0,0))
mission.do_explore(1.1, (0,0,0))
assert mission.controller.command == (0,0)
assert mission.view_scan_remaining is None
assert not mission.camera_coverage.seen.any()
''')

    def test_observation_turn_completes_across_yaw_wrap(self):
        self.run_case('''robot, mission = make("MISSION")
mission.set_navigation_path([(0,0)])
mission.do_explore(1, (0,0,0))
for now,yaw in ((2,math.pi/2), (3,math.pi), (4,-math.pi/2), (5,0)):
    mission.last_camera_time = now
    mission.do_explore(now, (0,0,yaw))
assert mission.view_scan_remaining is None
assert mission.controller.command == (0,0)
''')

    def test_safe_stop_without_candidates_throttles_replanning(self):
        self.run_case('''robot, mission = make("MISSION")
mission.do_explore(1, (0,0,0))
assert mission.recovery.stage == "SAFE_STOP"
assert mission.controller.command == (0,0)
with patch.object(main.planning, "find_frontiers") as find:
    mission.do_explore(1.1, (0,0,0))
    find.assert_not_called()
''')

    def test_new_state_cancels_recovery_and_old_path(self):
        self.run_case('''robot, mission = make("MISSION")
mission.explore_goal = (20,25)
mission.recovery.begin((0.3,0))
mission.set_navigation_path([(0.3,0)])
mission.transition(main.APPROACH_TARGET, 1)
assert mission.explore_goal is None and mission.recovery.goal is None
mission.set_navigation_path([(0.3,0)])
mission.transition(main.EXPLORE, 2)
assert not mission.navigation.follower.path
assert mission.controller.command == (0,0)
''')

    def test_coverage_requires_frame_and_does_not_change_grid(self):
        self.run_case('''import numpy as np
robot, mission = make("MISSION")
mission.grid.grid = [[0]*40 for _ in range(40)]
mission.detection_timer = main.Periodic(config.DETECTION_PERIOD)
robot.now = 1
with patch.object(mission, "do_explore"):
    mission.step()
assert not mission.camera_coverage.seen.any()
mission.devices.read_camera_frame = lambda: np.zeros((48,64,3), dtype=np.uint8)
robot.now = 2
with patch.object(mission, "do_explore"):
    mission.step()
assert mission.camera_coverage.seen.any()
assert all(value == 0 for row in mission.grid.grid for value in row)
''')

    def test_safe_path_uses_known_free_and_keeps_footprint_exception_local(self):
        self.run_case('''robot, mission = make("MISSION")
for r in range(15,26):
    for c in range(15,30): mission.grid.grid[r][c] = 0
mission.grid.grid[20][20] = -1
mission.explore_goal = (20,27)
mission.recovery.begin(mission.grid.grid_to_world(20,27))
mission.do_explore(1, (0,0,0))
assert mission.navigation.follower.path
assert mission.grid.grid[20][20] == -1
assert all(mission.grid.grid[r][c] == 0 or (r,c) == (20,20)
           for r,c in [mission.grid.world_to_grid(*xy) for xy in mission.navigation.follower.path])
''')

    def test_unseen_frontier_preference_and_blacklist(self):
        self.run_case('''robot, mission = make("MISSION")
mission.grid.grid = [[0]*40 for _ in range(40)]
a, b = (20,25), (20,30)
mission.camera_coverage.seen[a] = True
with patch.object(main.planning, "find_frontiers", return_value=[a,b]):
    mission.do_explore(1, (0,0,0))
    assert mission.explore_goal == b
    mission.recovery.exclude(mission.grid.grid_to_world(*b), 1)
    mission.recovery.reset()
    mission.explore_goal = None
    mission.do_explore(2, (0,0,0))
    assert mission.explore_goal == a
''')

    def test_unreachable_unseen_frontier_is_not_selected(self):
        self.run_case('''robot, mission = make("MISSION")
for r in range(15,26):
    for c in range(15,30): mission.grid.grid[r][c] = 0
reachable, unreachable = (20,27), (20,35)
mission.grid.grid[20][35] = 0
mission.camera_coverage.seen[reachable] = True
with patch.object(main.planning, "find_frontiers", return_value=[unreachable,reachable]):
    mission.do_explore(1, (0,0,0))
assert mission.explore_goal == reachable
''')

    def test_obstacle_safety_vetoes_camera_turn(self):
        self.run_case('''robot, mission = make("MISSION")
mission.set_navigation_path([(0,0)])
mission.do_explore(1, (0,0,0))
mission.last_camera_time = 1.1
mission.do_explore(1.1, (0,0,0))
ranges = [0.13]*360
mission.scan = main.navigation_control.Scan(tuple(ranges), 1.1)
mission.apply_safety(1.1, (0,0), ranges)
assert mission.controller.command == (0,0)
''')
