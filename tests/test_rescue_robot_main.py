"""기존 main 실행 흐름을 모의 장치로 검증합니다. Webots 주행 검증과는 별개입니다."""
import subprocess
import sys
import unittest
from pathlib import Path

FOLDER = Path(__file__).resolve().parents[1] / "controllers" / "rescue_robot"
SETUP = r''' 
import math, os, sys
from unittest.mock import patch
sys.path.insert(0, FOLDER)
import main
import config
class Motor:
    def __init__(self): self.velocity = 0.0
    def setVelocity(self, value): self.velocity = value
class Robot:
    def __init__(self): self.now = 0.0
    def getBasicTimeStep(self): return 64
    def getTime(self): return self.now
    def step(self, step): return 0
class Devices:
    pose = (0.0,0.0,0.0)
    def __init__(self, robot, step):
        self.robot = robot
        self.left_motor, self.right_motor = Motor(), Motor()
        self.lidar_resolution = 360
        self.lidar_fov = 2*math.pi
        self.lidar_min_range, self.lidar_max_range = 0.12, 3.5
        self.encoders = (0.0,0.0)
        self.ranges = [3.5]*360
    def read_start_pose(self): return self.pose
    def read_encoders(self): return self.encoders
    def read_lidar(self): return self.ranges
    def read_gyro_yaw_rate(self): return None
    def read_camera_frame(self): return None
class Timer:
    missed = 0
    def due(self, now): return False
main.Devices = Devices
config.GRID_WIDTH = config.GRID_HEIGHT = 40
config.STATUS_PRINT_PERIOD = 1e9
os.environ["RESCUE_WAYPOINTS"] = "[[1,0]]"
def make(mode="NAV_TEST", pose=(0.0,0.0,0.0)):
    config.BASELINE_MODE = mode
    Devices.pose = pose
    robot = Robot()
    mission = main.RescueMission(robot)
    mission.map_timer = mission.detection_timer = Timer()
    mission.step()
    return robot, mission
'''

class MainIntegrationTests(unittest.TestCase):
    def run_case(self, code):
        # flat import를 사용하는 기존 팀 모듈이 다른 테스트의 import를 오염시키지 않게 격리합니다.
        script = "FOLDER = " + repr(str(FOLDER)) + "\n" + SETUP + "\n" + code
        result = subprocess.run([sys.executable, "-X", "utf8", "-B", "-c", script],
                                text=True, capture_output=True, encoding="utf-8", timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_nav_test_transforms_start_relative_coordinates(self):
        self.run_case('''robot, mission = make(pose=(2,3,math.pi/2))
assert mission.state == main.NAV_TEST
x,y = mission.navigation.follower.path[0]
assert abs(x-2)<1e-9 and abs(y-4)<1e-9
robot.now = 0.064
mission.step()
assert mission.controller.command[0] > 0
''')

    def test_stop_mode_never_moves_even_with_path(self):
        self.run_case('''robot, mission = make("STOP")
mission.set_navigation_path([(1,0)])
mission.controller.set_velocity(0.15,0)
robot.now = 0.064
mission.step()
assert mission.controller.command == (0,0)
assert mission.devices.left_motor.velocity == mission.devices.right_motor.velocity == 0
''')

    def test_missing_sensor_and_nan_scan_stop_immediately(self):
        self.run_case('''robot, mission = make()
robot.now = 0.064
mission.step()
assert mission.controller.command[0] > 0
mission.devices.ranges = None
robot.now = 0.128
mission.step()
assert mission.controller.command == (0,0)
assert mission.safety_event == "REQUIRED_SENSOR_INVALID"
mission.devices.ranges = [3.5]*360
mission.devices.ranges[0] = float("nan")
robot.now = 0.192
mission.step()
assert mission.controller.command == (0,0)
assert mission.safety_event == "INVALID_SCAN"
''')

    def test_obstacle_wait_resume_and_latched_replan(self):
        self.run_case('''robot, mission = make()
mission.devices.ranges[180] = 0.18
robot.now = 0.064
mission.step()
assert mission.controller.command == (0,0)
mission.devices.ranges[180] = 3.5
robot.now = 1
mission.step()
assert mission.controller.command[0] > 0
mission.devices.ranges[180] = 0.18
for now in (2,3,4,5,6.1):
    robot.now = now
    mission.step()
assert mission.replan_requested
mission.devices.ranges[180] = 3.5
robot.now = 6.2
mission.step()
assert mission.controller.command == (0,0)
mission.set_navigation_path([(1,0)])
robot.now = 6.3
mission.step()
assert mission.controller.command[0] > 0
assert not mission.replan_requested
''')

    def test_slowdown_is_applied_once(self):
        self.run_case('''robot, mission = make()
for i in (179,180,181): mission.devices.ranges[i] = 0.38
robot.now = 0.064
mission.step()
expected = 0.15 * ((0.38-0.03-0.111)/0.25) * 0.5
assert abs(mission.controller.command[0]-expected)<1e-6
assert mission.navigation_status == "SLOW"
''')

    def test_grid_path_uses_mapping_origin_and_centre(self):
        self.run_case('''robot, mission = make("MISSION", pose=(2,3,0))
mission.set_navigation_grid_path([(20,20),(20,25)])
expected = [mission.grid.grid_to_world(20,20), mission.grid.grid_to_world(20,25)]
assert mission.navigation.follower.path == expected
mission.set_navigation_grid_path([])
assert mission.navigation.follower.path == []
try: mission.set_navigation_grid_path([(999,999)])
except ValueError: pass
else: raise AssertionError("범위 밖 경로를 받아들였습니다")
''')

    def test_return_home_planner_runs_after_stop_then_hands_path_to_follower(self):
        self.run_case('''robot, mission = make("MISSION")
pose = (0.5,0,math.pi)
mission.controller.set_velocity(0.15,0)
mission.transition(main.RETURN_HOME, 0)
mission.do_return_home(0, pose)
assert mission.home_plan_pending
assert mission.controller.command == (0,0)
calls = []
def astar(grid,start,goal,allow_unknown):
    assert mission.controller.command == (0,0)
    calls.append(allow_unknown)
    return [start,goal]
with patch.object(main.planning,"astar",astar):
    mission.scan = main.navigation_control.Scan(tuple([3.5]*360),0.064)
    mission.do_return_home(0.064,pose)
assert calls == [False]
assert mission.home_path
assert mission.navigation.follower.path == [mission.grid.grid_to_world(*c) for c in mission.home_path]
assert mission.controller.command != (0,0)
''')

    def test_return_home_replan_waits_before_recomputing(self):
        self.run_case('''robot, mission = make("MISSION")
mission.transition(main.RETURN_HOME,0)
mission.replan_requested = True
mission.do_return_home(2,(0.5,0,0))
assert mission.home_path is None and not mission.home_plan_pending
assert mission.controller.command == (0,0)
assert mission.next_home_plan_time == 2 + config.NAV_REPLAN_PERIOD
''')

    def test_empty_nav_test_path_stops(self):
        self.run_case('''os.environ["RESCUE_WAYPOINTS"] = "[]"
robot, mission = make()
robot.now = 0.064
mission.step()
assert mission.navigation_status == "NO_PATH"
assert mission.controller.command == (0,0)
''')

    def test_shutdown_on_exception_stops_motors(self):
        self.run_case('''robot, mission = make()
mission.controller.set_velocity(0.15,0)
def broken_step(): raise RuntimeError("모의 오류")
mission.step = broken_step
try: mission.run()
except RuntimeError: pass
else: raise AssertionError("예외가 누락되었습니다")
assert mission.controller.command == (0,0)
''')

    def test_closed_loop_runs_existing_localizer_and_main_to_done(self):
        self.run_case('''os.environ["RESCUE_WAYPOINTS"] = "[[0.4,0],[0.4,0.4],[0,0.4]]"
robot, mission = make()
left = right = 0.0
for i in range(1,2000):
    left += mission.devices.left_motor.velocity * mission.dt
    right += mission.devices.right_motor.velocity * mission.dt
    mission.devices.encoders = (left,right)
    robot.now = i*mission.dt
    mission.step()
    if mission.state == main.DONE: break
assert mission.state == main.DONE
assert math.hypot(mission.localizer.pose[0], mission.localizer.pose[1]-0.4) <= config.NAV_GOAL_TOLERANCE
assert mission.controller.command == (0,0)
''')

if __name__ == "__main__":
    unittest.main()
