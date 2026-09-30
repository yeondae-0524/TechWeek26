"""팀 Planner와 Control 연결에서 발생한 병합 오류를 재현하는 회귀 테스트입니다."""
import copy
import math
import subprocess
import sys
import unittest
from pathlib import Path

import _path  # noqa: F401
import planning
from interfaces import FREE, OCCUPIED, UNKNOWN


class PlanningContractTests(unittest.TestCase):
    def test_fractional_inflation_uses_conservative_cell_radius(self):
        grid = [[FREE] * 13 for _ in range(13)]
        grid[6][6] = OCCUPIED
        grid[0][0] = UNKNOWN
        original = copy.deepcopy(grid)

        inflated = planning.inflate_obstacles(grid, 3.22)

        # 0.161m 안전 반경을 5cm 셀로 바꾸면 3.22셀이며 올림해서 4셀을 막습니다.
        for row, col in ((6, 10), (10, 6), (9, 8)):
            self.assertEqual(inflated[row][col], OCCUPIED)
        self.assertEqual(inflated[6][11], FREE)
        self.assertEqual(inflated[10][7], FREE)
        self.assertEqual(inflated[0][0], UNKNOWN)
        self.assertEqual(grid, original)
        self.assertIsNot(inflated, grid)
        self.assertIsNot(inflated[0], grid[0])

    def test_unknown_cells_inside_inflation_are_blocked(self):
        grid = [[UNKNOWN] * 7 for _ in range(7)]
        grid[3][3] = OCCUPIED
        inflated = planning.inflate_obstacles(grid, 1)
        self.assertEqual(inflated[3][4], OCCUPIED)
        self.assertEqual(inflated[0][0], UNKNOWN)
        self.assertEqual(grid[3][4], UNKNOWN)

    def test_astar_detours_around_inflated_footprint(self):
        grid = [[FREE] * 11 for _ in range(9)]
        grid[4][5] = OCCUPIED
        inflated = planning.inflate_obstacles(grid, 1.01)
        path = planning.astar(inflated, (4, 0), (4, 10), allow_unknown=False)
        self.assertTrue(path)
        self.assertEqual((path[0], path[-1]), ((4, 0), (4, 10)))
        self.assertTrue(all(inflated[row][col] == FREE for row, col in path))
        self.assertTrue(all(math.hypot(row - 4, col - 5) > 2 for row, col in path))
        self.assertGreater(len(path), 11)

    def test_unknown_wall_respects_explicit_policy(self):
        grid = [[FREE, UNKNOWN, FREE] for _ in range(3)]
        self.assertEqual(planning.astar(grid, (1, 0), (1, 2), allow_unknown=False), [])
        path = planning.astar(grid, (1, 0), (1, 2), allow_unknown=True)
        self.assertTrue(path)
        self.assertIn((1, 1), path)

    def test_same_endpoint_must_still_be_valid(self):
        grid = [[FREE, OCCUPIED], [FREE, UNKNOWN]]
        for cell in ((-1, 0), (2, 2), (0, 1)):
            with self.subTest(cell=cell):
                self.assertEqual(planning.astar(grid, cell, cell), [])
        self.assertEqual(planning.astar(grid, (1, 1), (1, 1), allow_unknown=False), [])

    def test_diagonal_cannot_cross_occupied_or_unknown_corner(self):
        for side_value, allow_unknown in ((OCCUPIED, True), (UNKNOWN, False)):
            with self.subTest(side_value=side_value):
                grid = [[FREE, side_value], [side_value, FREE]]
                path = planning.astar(grid, (0, 0), (1, 1),
                                      allow_unknown=allow_unknown, connectivity=8)
                self.assertEqual(path, [])

    def test_diagonal_checks_each_side_independently(self):
        grid = [[FREE, OCCUPIED], [FREE, FREE]]
        path = planning.astar(grid, (0, 0), (1, 1), connectivity=8)
        self.assertEqual(path, [(0, 0), (1, 0), (1, 1)])

    def test_diagonal_cannot_cross_lethal_costmap_corner(self):
        grid = [[FREE] * 2 for _ in range(2)]
        costmap = [[0, 254], [254, 0]]
        self.assertEqual(planning.astar(grid, (0, 0), (1, 1),
                                       costmap=costmap, connectivity=8), [])

    def test_2d_costmap_prefers_lower_cost_route(self):
        grid = [[FREE] * 5 for _ in range(3)]
        costmap = [[0] * 5 for _ in range(3)]
        costmap[1][2] = 253
        original = copy.deepcopy(costmap)
        path = planning.astar(grid, (1, 0), (1, 4), costmap=costmap)
        self.assertTrue(path)
        self.assertNotIn((1, 2), path)
        self.assertEqual(costmap, original)

    def test_occupied_and_lethal_endpoints_are_rejected(self):
        grid = [[FREE] * 3 for _ in range(3)]
        grid[0][0] = OCCUPIED
        self.assertEqual(planning.astar(grid, (0, 0), (2, 2)), [])
        self.assertEqual(planning.astar(grid, (2, 2), (0, 0)), [])
        costmap = [[0] * 3 for _ in range(3)]
        costmap[2][2] = 254
        grid[0][0] = FREE
        self.assertEqual(planning.astar(grid, (0, 0), (2, 2), costmap=costmap), [])
        self.assertEqual(planning.astar(grid, (2, 2), (2, 2), costmap=costmap), [])


class FrontierPlanningTests(unittest.TestCase):
    def test_skips_frontier_blocked_by_inflation(self):
        # 오른쪽 위 frontier는 벽 옆이라 팽창 지도에서 막힙니다. 막힌 칸 대신 도달 가능한 칸을 고릅니다.
        grid = [[FREE] * 8 for _ in range(8)]
        for row in range(8):
            grid[row][7] = UNKNOWN
        grid[0][6] = OCCUPIED
        inflated = planning.inflate_obstacles(grid, 1)
        frontier, path = planning.plan_to_frontier(grid, inflated, (6, 1), (0.075, 0.325, 0.0),
                                                   resolution=0.05)
        self.assertIsNotNone(frontier)
        self.assertEqual(inflated[frontier[0]][frontier[1]], FREE)
        self.assertEqual((path[0], path[-1]), ((6, 1), frontier))

    def test_start_inside_inflation_is_released(self):
        grid = [[FREE] * 6 for _ in range(6)]
        for row in range(6):
            grid[row][5] = UNKNOWN
        grid[2][0] = OCCUPIED
        inflated = planning.inflate_obstacles(grid, 1)
        self.assertEqual(inflated[2][1], OCCUPIED)
        frontier, path = planning.plan_to_frontier(grid, inflated, (2, 1), (0.075, 0.125, 0.0),
                                                   resolution=0.05)
        self.assertTrue(path)
        self.assertEqual(path[0], (2, 1))
        self.assertEqual(inflated[2][1], OCCUPIED)  # 입력은 바꾸지 않습니다

    def test_unknown_ring_around_robot_is_cleared(self):
        # LiDAR 최소 거리 때문에 로봇 주변이 UNKNOWN이어도 출발할 수 있어야 합니다.
        grid = [[FREE] * 12 for _ in range(12)]
        for row in range(12):
            grid[row][11] = UNKNOWN
        for row in range(3, 8):
            for col in range(3, 8):
                grid[row][col] = UNKNOWN
        grid[5][3] = OCCUPIED
        inflated = planning.inflate_obstacles(grid, 0)
        self.assertEqual(planning.plan_to_frontier(grid, inflated, (5, 5), (0.275, 0.275, 0.0),
                                                   resolution=0.05), (None, []))
        frontier, path = planning.plan_to_frontier(grid, inflated, (5, 5), (0.275, 0.275, 0.0),
                                                   resolution=0.05, footprint_cells=2.2)
        self.assertTrue(path)
        cleared = planning.clear_footprint(inflated, grid, (5, 5), 2.2)
        self.assertEqual(cleared[5][3], OCCUPIED)  # 실제 장애물은 풀지 않습니다
        self.assertEqual(cleared[5][7], FREE)
        self.assertEqual(inflated[5][7], UNKNOWN)  # 입력은 바꾸지 않습니다

    def test_prefers_reachable_frontier_over_higher_scored_island(self):
        # 오른쪽 FREE 섬은 UNKNOWN에 둘러싸여 점수가 높지만 갈 수 없습니다.
        grid = [[UNKNOWN] * 20 for _ in range(10)]
        for row in range(10):
            for col in range(4):
                grid[row][col] = FREE
        grid[5][15] = FREE
        inflated = planning.inflate_obstacles(grid, 0)
        self.assertEqual(planning.select_frontier(grid, (0.175, 0.275, 0.0), 0.05), (5, 15))
        frontier, path = planning.plan_to_frontier(grid, inflated, (5, 1), (0.075, 0.275, 0.0),
                                                   resolution=0.05, max_tries=1)
        self.assertEqual(frontier[1], 3)
        self.assertEqual(path[-1], frontier)
        self.assertIn(frontier, planning.reachable_cells(inflated, (5, 1)))

    def test_no_reachable_frontier(self):
        grid = [[FREE, OCCUPIED, FREE, UNKNOWN]]
        inflated = planning.inflate_obstacles(grid, 0)
        self.assertEqual(planning.plan_to_frontier(grid, inflated, (0, 0), (0.0, 0.0, 0.0),
                                                   resolution=0.05), (None, []))


class LocalPlannerContractTests(unittest.TestCase):
    def test_2d_costmap_uses_same_origin_and_cell_centres(self):
        origin, resolution = (-2.0, 4.0), 0.1
        path = [(1, col) for col in range(9)]
        pose = (-1.95, 4.15, 0.0)
        costmap = [[0] * 9 for _ in range(3)]
        v, w = planning.local_planner(path, pose, local_costmap=costmap,
                                      origin=origin, resolution=resolution)
        self.assertGreater(v, 0.0)
        self.assertAlmostEqual(w, 0.0)

    def test_2d_costmap_blocks_lethal_cell_before_lookahead(self):
        origin, resolution = (-2.0, 4.0), 0.1
        path = [(1, col) for col in range(9)]
        pose = (-1.95, 4.15, 0.0)
        costmap = [[0] * 9 for _ in range(3)]
        # 목표 셀만 보지 않고 현재 위치부터 추종점 사이의 위험 셀도 검사합니다.
        costmap[1][1] = 254
        command = planning.local_planner(path, pose, local_costmap=costmap,
                                         origin=origin, resolution=resolution)
        self.assertEqual(command, (0.0, 0.0))

    def test_frontier_score_is_invariant_under_world_translation(self):
        grid = [[FREE, FREE, FREE, UNKNOWN, UNKNOWN] for _ in range(5)]
        frontier = (2, 2)
        pose = (0.2, 0.3, 0.4)
        offset = (-12.0, 9.0)
        reference = planning.frontier_score(frontier, pose, grid,
                                           resolution=0.2, origin=(0.0, 0.0))
        translated_pose = (pose[0] + offset[0], pose[1] + offset[1], pose[2])
        translated = planning.frontier_score(frontier, translated_pose, grid,
                                            resolution=0.2, origin=offset)
        self.assertAlmostEqual(reference, translated)

    def test_local_planner_obeys_shared_speed_limits(self):
        from unittest.mock import patch
        path = [(1, col) for col in range(9)]
        # 팀 공용 설정이 바뀌어도 별도 고정 속도로 주행하지 않습니다.
        with patch.object(planning.config, "MAX_LINEAR_SPEED", 0.07), \
             patch.object(planning.config, "MAX_ANGULAR_SPEED", 0.4):
            v, w = planning.local_planner(path, (0.05, 0.15, 0.0), resolution=0.1)
            self.assertGreater(v, 0.0)
            self.assertLessEqual(v, 0.07)
            self.assertLessEqual(abs(w), 0.4)
            v, w = planning.local_planner(path, (0.05, 0.15, -math.pi / 2),
                                          resolution=0.1)
            self.assertEqual(v, 0.0)
            self.assertGreater(w, 0.0)
            self.assertLessEqual(abs(w), 0.4)

class ReturnHomePlanningIntegrationTests(unittest.TestCase):
    def test_real_planner_hands_safe_grid_path_to_control(self):
        # main의 전역 설정과 평면 import를 다른 테스트에서 격리합니다.
        folder = Path(__file__).resolve().parents[1] / "controllers" / "rescue_robot"
        script = "FOLDER = " + repr(str(folder)) + "\n" + r'''
import math
import sys
from unittest.mock import patch
sys.path.insert(0, FOLDER)
import main
import config

class Motor:
    def __init__(self):
        self.velocity = 0.0
    def setVelocity(self, value):
        self.velocity = value

class Robot:
    def getBasicTimeStep(self):
        return 64
    def getTime(self):
        return 0.0

class Devices:
    def __init__(self, robot, timestep):
        self.left_motor, self.right_motor = Motor(), Motor()
        self.lidar_resolution, self.lidar_fov = 360, 2 * math.pi
        self.lidar_min_range, self.lidar_max_range = 0.12, 3.5
    def read_start_pose(self):
        return (0.0, 0.0, 0.0)
    def read_encoders(self):
        return (0.0, 0.0)
    def read_lidar(self):
        return [3.5] * 360
    def read_gyro_yaw_rate(self):
        return None
    def read_camera_frame(self):
        return None

with patch.object(main, "Devices", Devices), \
     patch.object(config, "BASELINE_MODE", "MISSION"), \
     patch.object(config, "GRID_WIDTH", 40), \
     patch.object(config, "GRID_HEIGHT", 40), \
     patch.object(config, "GRID_ORIGIN", None):
    mission = main.RescueMission(Robot())
    # 첫 상태 출력까지 실행해서 Planner의 frontier 인터페이스도 확인합니다.
    mission.step()
    mission.grid.grid = [[main.mapping.FREE] * 40 for _ in range(40)]
    mission.grid.grid[20][25] = main.mapping.OCCUPIED
    pose = (0.5, 0.0, math.pi)
    mission.transition(main.RETURN_HOME, 0.0)
    mission.do_return_home(0.0, pose)
    assert mission.home_plan_pending
    assert mission.controller.command == (0.0, 0.0)
    mission.scan = main.navigation_control.Scan(tuple([3.5] * 360), 0.064)
    mission.do_return_home(0.064, pose)
    assert mission.home_path, "실제 Planner가 복귀 경로를 만들지 못했습니다"
    radius = (config.ROBOT_RADIUS + config.SAFETY_MARGIN) / mission.grid.resolution
    assert abs(radius - 3.22) < 1e-9
    inflated = main.planning.inflate_obstacles(mission.grid.grid, radius)
    assert all(inflated[row][col] == main.mapping.FREE for row, col in mission.home_path)
    assert (20, 25) not in mission.home_path
    expected = [mission.grid.grid_to_world(row, col) for row, col in mission.home_path]
    assert mission.navigation.follower.path == expected
    assert mission.home_path[0] == mission.grid.world_to_grid(pose[0], pose[1])
    assert mission.home_path[-1] == mission.grid.world_to_grid(0.0, 0.0)
'''
        result = subprocess.run([sys.executable, "-X", "utf8", "-B", "-c", script],
                                capture_output=True, text=True, encoding="utf-8", timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
