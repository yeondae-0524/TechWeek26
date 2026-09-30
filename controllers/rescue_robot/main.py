"""TurtleBot3 구조 미션의 실행 진입점입니다.

매 step: 센서 → 기존 Localization → Mapping/Detection → 경로 추종 → 안전 검사.
NAV_TEST에서는 명시적으로 받은 waypoint를 추종합니다. MISSION에서는 팀 Planner가
전달한 경로와 복귀 A* 경로를 추종합니다. 탐색 목표 선택과 대상 접근은 아직 TODO입니다.
기본 STOP에서는 모터를 항상 정지시킵니다. 금지 센서는 사용하지 않습니다.
"""
import json
import math
import os
import sys
import time

import config
import control
import detection
import mapping
import planning
import navigation_control
from devices import DeviceError, Devices
from interfaces import empty_target, make_pose
from localization import Localizer
from scheduling import Periodic, StepTimer

INITIALIZE = "INITIALIZE"
EXPLORE = "EXPLORE"
APPROACH_TARGET = "APPROACH_TARGET"
RETURN_HOME = "RETURN_HOME"
DONE = "DONE"
CONTROL_TEST = "CONTROL_TEST"
NAV_TEST = "NAV_TEST"

# 기존 구동계 점검 순서를 유지합니다. 각 동작의 시간은 config에 있습니다.
CONTROL_TEST_SEQUENCE = (
    ("drive_forward", "FORWARD"), ("stop", "STOP"),
    ("rotate_left", "LEFT"), ("stop", "STOP"),
    ("rotate_right", "RIGHT"), ("stop", "STOP"),
)

def fmt_pose(pose):
    return f"({pose[0]:+.3f}, {pose[1]:+.3f}, {math.degrees(pose[2]):+.1f}deg)"

class RescueMission:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0
        self.devices = Devices(robot, self.timestep)
        d = self.devices
        # 잘못된 모드에서는 초기 모터 정지 이후 실행을 거부합니다.
        if config.BASELINE_MODE not in ("STOP", "CONTROL_TEST", "NAV_TEST", "MISSION"):
            raise ValueError("RESCUE_MODE는 STOP/CONTROL_TEST/NAV_TEST/MISSION 중 하나여야 합니다")
        custom_pose = d.read_start_pose()
        self.start_pose = custom_pose if custom_pose is not None else make_pose(*config.START_POSE)
        self.start_pose_source = "customData" if custom_pose is not None else "config.START_POSE"
        self.home_pose = None
        # 기존 encoder 위치 추정 모듈을 재사용합니다. 별도 odometry를 중복 실행하지 않습니다.
        self.localizer = Localizer(self.start_pose, config.WHEEL_RADIUS,
                                   config.AXLE_LENGTH, config.encoder_to_rad())
        self.controller = control.DiffDriveController(
            d.left_motor, d.right_motor, config.WHEEL_RADIUS, config.AXLE_LENGTH,
            config.MAX_WHEEL_SPEED, config.MAX_LINEAR_SPEED, config.MAX_ANGULAR_SPEED)
        self.safety = control.SafetyMonitor.from_config(config)
        self.navigation = navigation_control.Controller(config.navigation_config())
        self.scan = None
        self.navigation_status = "NO_PATH"
        self.replan_requested = False
        self.last_navigation_status = None
        self.grid = None
        self.lidar_angles = [mapping.lidar_angle(i, d.lidar_resolution, d.lidar_fov)
                             for i in range(d.lidar_resolution)]
        # 예상과 다른 센서 사양으로 잘못된 각도를 사용하지 않도록 정지합니다.
        self.lidar_profile_valid = (
            d.lidar_resolution == config.LIDAR_EXPECTED_RESOLUTION
            and math.isclose(d.lidar_fov, config.LIDAR_EXPECTED_FOV, abs_tol=0.01)
            and math.isclose(d.lidar_min_range, config.LIDAR_MIN_RANGE, abs_tol=0.01)
            and math.isclose(d.lidar_max_range, config.LIDAR_MAX_RANGE, abs_tol=0.01))
        self.map_timer = Periodic(config.MAP_UPDATE_PERIOD)
        self.detection_timer = Periodic(config.DETECTION_PERIOD)
        self.step_timer = StepTimer()
        self.state = INITIALIZE
        self.state_start_time = 0.0
        self.step_count = 0
        self.last_status_time = -1e9
        self.target = empty_target()
        self.tracker = detection.TargetTracker()  # confirmed targets with world (x, y)
        self.detection_timer_stats = StepTimer()  # camera read + detect() time
        self.approach_target_id = None
        self.home_path = None
        self.home_plan_pending = False
        self.next_home_plan_time = 0.0
        self.test_index = -1
        self.safety_event = None
        self.sensor_fault = None
        self.sensor_status_reported = False

    def run(self):
        """정상 종료와 예외 발생 모두에서 마지막 모터 명령을 정지로 바꿉니다."""
        try:
            while self.robot.step(self.timestep) != -1:
                self.step()
        finally:
            self.controller.stop()

    def step(self):
        started = time.perf_counter()
        now = self.robot.getTime()
        self.step_count += 1
        encoders = self.devices.read_encoders()
        ranges = self.devices.read_lidar()
        yaw_rate = self.devices.read_gyro_yaw_rate()
        # getRangeImage에는 독립적인 취득 timestamp가 없어 현재 simulation 시간을 사용합니다.
        self.scan = navigation_control.Scan(tuple(ranges), now) if ranges is not None else None
        self.check_required_sensors(encoders, ranges)
        pose = self.localizer.update(encoders, yaw_rate, self.dt)
        if self.grid is not None and ranges and self.map_timer.due(now):
            self.grid.insert_scan(pose, ranges, self.devices.lidar_fov,
                                  self.devices.lidar_max_range, min_range=self.devices.lidar_min_range)
        if self.detection_timer.due(now):
            started_detection = time.perf_counter()
            self.target, blobs = detection.detect(self.devices.read_camera_frame())
            self.detection_timer_stats.add(time.perf_counter() - started_detection)
            for track in self.tracker.update(now, pose, blobs):
                print(f"[detection] CONFIRMED target #{track['id']} at "
                      f"({track['xy'][0]:+.2f}, {track['xy'][1]:+.2f})")
        handler = {
            INITIALIZE: self.do_initialize, EXPLORE: self.do_explore,
            APPROACH_TARGET: self.do_approach_target, RETURN_HOME: self.do_return_home,
            DONE: self.do_done, CONTROL_TEST: self.do_control_test, NAV_TEST: self.do_nav_test,
        }[self.state]
        # 필수 센서 이상 시 상태별 제어와 계획도 보류합니다.
        if encoders is None or ranges is None or not self.lidar_profile_valid:
            self.controller.stop()
        else:
            handler(now, pose)
        self.apply_safety(now, encoders, ranges)
        self.step_timer.add(time.perf_counter() - started)
        self.print_status(now, pose, ranges)

    def check_required_sensors(self, encoders, ranges):
        """필수 센서 오류를 숨기지 않고 정지 이유를 기록합니다."""
        fault = None
        if not self.lidar_profile_valid:
            fault = "LiDAR 사양 불일치"
        elif encoders is None:
            fault = "encoder 값 오류"
        elif ranges is None:
            fault = "LiDAR 값 오류"
        if not self.sensor_status_reported or fault != self.sensor_fault:
            print(f"[safety] {fault} -> STOP" if fault else "[safety] 필수 센서 정상")
            self.sensor_fault = fault
            self.sensor_status_reported = True

    def apply_safety(self, now, encoders, ranges):
        """경로 추종과 구동계 점검 모두에 원시 LiDAR 안전 검사를 마지막에 적용합니다."""
        self.safety.update_scan(ranges, self.lidar_angles, now)
        v, w = self.controller.command
        if config.BASELINE_MODE == "STOP":
            v2, w2, event = 0.0, 0.0, None
        elif encoders is None or ranges is None or not self.lidar_profile_valid:
            v2, w2, event = 0.0, 0.0, "REQUIRED_SENSOR_INVALID"
        else:
            # 새 예측 검사와 기존 사각 감시를 모두 통과한 명령만 적용합니다.
            v2, w2, event = self.navigation.safety.filter(v, w, self.scan, now)
            v2, w2, old_event = self.safety.filter(v2, w2, now)
            if old_event is not None:
                event = old_event
            elif event == "CLEAR":
                event = None
        self.controller.set_velocity(v2, w2)
        if event != self.safety_event:
            if event is not None:
                print(f"[safety] {event}: ({v:+.3f}, {w:+.2f}) -> ({v2:+.3f}, {w2:+.2f})")
            self.safety_event = event

    def transition(self, new_state, now, reason=""):
        print(f"[state] {self.state} -> {new_state}" + (f" ({reason})" if reason else ""))
        # 목적이 바뀌면 이전 목적지의 경로를 계속 실행하지 않습니다.
        if new_state in (RETURN_HOME, APPROACH_TARGET, DONE):
            self.navigation.set_path([])
            self.replan_requested = False
            self.navigation_status = "NO_PATH"
            self.controller.stop()
        if new_state == RETURN_HOME:
            self.home_path = None
            self.home_plan_pending = False
            self.next_home_plan_time = now
        self.state = new_state
        self.state_start_time = now

    def set_navigation_path(self, waypoints):
        """Planner가 전달한 동일 좌표계의 미터 waypoint 경로를 등록합니다."""
        self.navigation.set_path(waypoints)
        self.replan_requested = False
        self.navigation_status = "RUNNING" if self.navigation.follower.path else "NO_PATH"
        self.controller.stop()

    def set_navigation_grid_path(self, path):
        """팀의 (row,col) 경로를 현재 지도 셀 중심 좌표로 변환합니다."""
        if self.grid is None:
            raise ValueError("지도 초기화 후 경로를 전달해야 합니다")
        cells = list(path)
        if any(not self.grid.in_bounds(row, col) for row, col in cells):
            raise ValueError("경로에 지도 범위 밖의 셀이 있습니다")
        self.set_navigation_path([self.grid.grid_to_world(row, col) for row, col in cells])

    def follow_navigation(self, now, pose):
        """실제 pose와 스캔으로 속도를 계산하고 저수준 모터 제어에 전달합니다."""
        v, w, status = self.navigation.compute(pose, self.scan, now, self.dt, defer_safety=True)
        self.controller.set_velocity(v, w)
        self.navigation_status = status
        self.replan_requested = status == "REPLAN_REQUIRED"
        if status != self.last_navigation_status:
            print(f"[navigation] {status}: pose={fmt_pose(pose)}")
            self.last_navigation_status = status
        return status

    def do_initialize(self, now, pose):
        self.controller.stop()
        self.home_pose = self.start_pose
        self.localizer.odometry.reset(self.home_pose)
        self.grid = mapping.OccupancyGrid.centered_on(
            self.home_pose[0], self.home_pose[1], config.GRID_WIDTH, config.GRID_HEIGHT,
            config.GRID_RESOLUTION) if config.GRID_ORIGIN is None else mapping.OccupancyGrid(
            config.GRID_WIDTH, config.GRID_HEIGHT, config.GRID_RESOLUTION, config.GRID_ORIGIN)
        print(f"[main] home_pose={fmt_pose(self.home_pose)} ({self.start_pose_source}), mode={config.BASELINE_MODE}")
        self.print_lidar_check()
        if config.BASELINE_MODE == "CONTROL_TEST":
            self.transition(CONTROL_TEST, now, "구동계 점검")
        elif config.BASELINE_MODE == "NAV_TEST":
            # 시험 경로는 시작점 기준 좌표입니다. 기존 지도/pose 좌표계로 변환합니다.
            path = json.loads(os.environ.get("RESCUE_WAYPOINTS", "[]"))
            checker = navigation_control.PathFollower(config.navigation_config())
            checker.set_path(path)
            self.set_navigation_path([mapping.local_to_world(self.home_pose, *point)
                                      for point in checker.path])
            self.transition(NAV_TEST, now, "명시적으로 받은 waypoint 주행 시험")
        else:
            self.transition(EXPLORE, now)

    def do_nav_test(self, now, pose):
        status = self.follow_navigation(now, pose)
        if status == "REACHED":
            self.transition(DONE, now, "waypoint 경로 도착")
        # 재계획 요청은 새 경로를 받을 때까지 정지 상태로 유지합니다.

    def do_explore(self, now, pose):
        if config.BASELINE_MODE == "STOP":
            self.controller.stop()
            return
        if self.tracker.all_visited():
            self.transition(RETURN_HOME, now, f"{self.tracker.visited_count()} targets visited")
            return
        track = self.tracker.nearest_unvisited(pose)
        if track is not None:  # a single-frame hit is not enough: wait for confirmation
            self.approach_target_id = track["id"]
            self.transition(APPROACH_TARGET, now,
                            f"target #{track['id']} at ({track['xy'][0]:+.2f}, {track['xy'][1]:+.2f})")
            return
        if now > config.MISSION_TIME_LIMIT:
            self.transition(RETURN_HOME, now, "미션 제한 시간")
            return
        # TODO: 탐색 목표 선택은 Planning 팀 담당입니다. 경로가 없으면 정지합니다.
        # Planner는 set_navigation_grid_path(path) 또는 set_navigation_path(waypoints)를 호출합니다.
        self.follow_navigation(now, pose)

    def do_approach_target(self, now, pose):
        """확정된 target 앞 standoff 지점까지 계획·추종하고 도착하면 방문 처리합니다."""
        if config.BASELINE_MODE == "STOP":
            self.controller.stop()
            return
        if now > config.MISSION_TIME_LIMIT:
            self.transition(RETURN_HOME, now, "미션 제한 시간")
            return
        track = self.tracker.get(self.approach_target_id)
        if track is None or track["visited"] or track.get("skipped"):
            self.controller.stop()
            self.transition(EXPLORE, now, "approach target unavailable")
            return
        if getattr(self, "_approach_id", None) != track["id"]:
            self.reset_approach(track["id"], now)
        if detection.is_arrived(pose, track["xy"]):
            self.controller.stop()
            self.tracker.mark_visited(track["id"])
            self.approach_log(f"[approach] visited #{track['id']}")
            self.transition(EXPLORE, now, f"target #{track['id']} visited")
            return
        if self.approach_rotating:
            self.rotate_towards_target(now, pose, track)
            return
        if now - self.approach_attempt_start > config.APPROACH_TIMEOUT:
            self.approach_attempts += 1
            if self.approach_attempts > config.APPROACH_MAX_ATTEMPTS:
                self.controller.stop()
                self.tracker.skip(track["id"])
                self.approach_log(f"[approach] skipped #{track['id']}")
                self.transition(EXPLORE, now, f"target #{track['id']} skipped")
                return
            self.approach_log(f"[approach] timeout #{track['id']}: turn to target "
                              f"(attempt {self.approach_attempts})")
            self.approach_rotating = True
            self.approach_path_ready = False
            self.navigation.set_path([])
            self.rotate_towards_target(now, pose, track)
            return
        self.follow_approach_plan(now, pose, track)

    # ---------------------------------------------------------- approach helpers
    def reset_approach(self, track_id, now):
        self._approach_id = track_id
        self.approach_attempts = 1
        self.approach_attempt_start = now
        self.approach_rotating = False
        self.approach_path_ready = False
        self.approach_plan_pending = False
        self.next_approach_plan_time = now
        self.approach_goal = None
        self.last_approach_log = None
        self.approach_log(f"[approach] start #{track_id}")

    def approach_log(self, message):
        if message != getattr(self, "last_approach_log", None):
            print(message)
            self.last_approach_log = message

    def rotate_towards_target(self, now, pose, track):
        """사과 방향을 볼 때까지 제자리 회전한 뒤 새 시도로 재계획합니다."""
        bearing, _ = detection.relative_to(pose, track["xy"])
        if abs(bearing) <= config.APPROACH_FACING_TOLERANCE:
            self.controller.stop()
            self.approach_rotating = False
            self.approach_attempt_start = now
            self.next_approach_plan_time = now
            self.approach_log(f"[approach] facing #{track['id']}: replan")
            return
        self.controller.set_velocity(0.0, math.copysign(config.NAV_ROTATE_SPEED, bearing))

    def follow_approach_plan(self, now, pose, track):
        """do_return_home과 같은 순서: 정지 1 step -> inflate -> A* -> 경로 추종."""
        if self.replan_requested:
            self.approach_path_ready = False
            self.approach_plan_pending = False
            self.next_approach_plan_time = now + config.NAV_REPLAN_PERIOD
            self.replan_requested = False
        if not self.approach_path_ready:
            self.controller.stop()
            if self.grid is None or now < self.next_approach_plan_time:
                return
            if not self.approach_plan_pending:
                self.approach_plan_pending = True
                return
            self.approach_plan_pending = False
            # 사과는 LiDAR 평면 아래라 지도에 없습니다. 사과 앞 standoff 지점을 목표로 합니다.
            goal_xy = detection.standoff_point(
                pose, track["xy"], config.TARGET_ARRIVAL_DISTANCE * 0.8)
            radius = (config.ROBOT_RADIUS + config.SAFETY_MARGIN) / self.grid.resolution
            costmap = planning.inflate_obstacles(self.grid.grid, int(math.ceil(radius)))
            start = self.grid.world_to_grid(pose[0], pose[1])
            goal = self.grid.world_to_grid(goal_xy[0], goal_xy[1])
            path = planning.astar(self.grid.grid, start, goal, costmap)
            if not path:
                self.navigation.set_path([])
                self.navigation_status = "NO_PATH"
                self.next_approach_plan_time = now + config.NAV_REPLAN_PERIOD
                self.approach_log(f"[approach] no path to #{track['id']}: retry")
                return
            self.approach_goal = goal_xy
            self.approach_path_ready = True
            self.set_navigation_grid_path(path)
            self.approach_log(f"[approach] path to #{track['id']}: {len(path)} cells")
        status = self.follow_navigation(now, pose)
        if status == "REACHED" and self.approach_goal is not None:
            # 셀 중심 도착 후 실제 standoff 지점으로 마무리합니다.
            self.set_navigation_path([self.approach_goal])
            self.approach_goal = None

    def do_return_home(self, now, pose):
        if config.BASELINE_MODE == "STOP":
            self.controller.stop()
            return
        distance = math.hypot(pose[0]-self.home_pose[0], pose[1]-self.home_pose[1])
        if distance < config.HOME_TOLERANCE:
            self.controller.stop()
            self.transition(DONE, now, f"시작점 복귀 ({distance:.2f} m)")
            return
        if self.replan_requested:
            self.home_path = None
            self.home_plan_pending = False
            self.next_home_plan_time = now + config.NAV_REPLAN_PERIOD
            self.replan_requested = False
        if self.home_path is None:
            self.controller.stop()
            if self.grid is None or now < self.next_home_plan_time:
                return
            if not self.home_plan_pending:
                # A* 계산 전에 정지 명령을 한 번 robot.step으로 전달합니다.
                self.home_plan_pending = True
                return
            radius = (config.ROBOT_RADIUS+config.SAFETY_MARGIN)/self.grid.resolution
            # planning API (PR #15): inflate_obstacles -> cost map (0-254), whole-cell radius;
            # astar(grid, start, goal, costmap) blocks OCCUPIED cells, UNKNOWN is passable.
            costmap = planning.inflate_obstacles(self.grid.grid, int(math.ceil(radius)))
            start = self.grid.world_to_grid(pose[0], pose[1])
            goal = self.grid.world_to_grid(self.home_pose[0], self.home_pose[1])
            path = planning.astar(self.grid.grid, start, goal, costmap)
            mode = "costmap"
            self.home_plan_pending = False
            print(f"[plan] 복귀 경로 ({mode}): {len(path)} cells")
            if not path:
                self.navigation.set_path([])
                self.navigation_status = "NO_PATH"
                self.next_home_plan_time = now + config.NAV_REPLAN_PERIOD
                return
            self.home_path = path
            self.set_navigation_grid_path(path)
        status = self.follow_navigation(now, pose)
        if status == "REACHED":
            # 셀 중심 도착과 실제 시작 pose 도착을 혼동하지 않습니다.
            self.set_navigation_path([(self.home_pose[0], self.home_pose[1])])

    def do_done(self, now, pose):
        self.controller.stop()

    def do_control_test(self, now, pose):
        index = int((now-self.state_start_time)//config.CONTROL_TEST_STEP_DURATION)
        if index >= len(CONTROL_TEST_SEQUENCE):
            self.controller.stop()
            self.transition(DONE, now, "구동계 점검 종료")
            return
        primitive, label = CONTROL_TEST_SEQUENCE[index]
        getattr(self.controller, primitive)()
        if index != self.test_index:
            self.test_index = index
            front = self.lidar_range_at(0.0)
            text = f" lidar_front={front:.3f}" if front is not None else ""
            print(f"[control-test] {label:<7} odom={fmt_pose(pose)}{text}")

    def lidar_range_at(self, target_angle, ranges=None):
        """로봇 기준 목표 각도와 가장 가까운 LiDAR 광선의 거리를 반환합니다."""
        ranges = ranges if ranges is not None else self.devices.read_lidar()
        if not ranges:
            return None
        index = min(range(len(ranges)), key=lambda k: abs(math.atan2(
            math.sin(self.lidar_angles[k]-target_angle), math.cos(self.lidar_angles[k]-target_angle))))
        return ranges[index]

    def print_lidar_check(self):
        """전후좌우 대표 광선으로 시작 시 센서 방향을 점검합니다."""
        ranges = self.devices.read_lidar()
        if not ranges:
            return
        values = [f"{name}={self.lidar_range_at(angle, ranges):.3f}"
                  for name,angle in (("front",0.0),("left",math.pi/2),("back",math.pi),("right",-math.pi/2))]
        print("[lidar] ranges (센서 원점 기준): " + " ".join(values))

    def print_status(self, now, pose, ranges):
        if now-self.last_status_time < config.STATUS_PRINT_PERIOD:
            return
        self.last_status_time = now
        parts = [f"t={now:6.1f}s", f"state={self.state}", f"pose={fmt_pose(pose)}",
                 f"navigation={self.navigation_status}"]
        front = self.lidar_range_at(0.0, ranges) if ranges else None
        if front is not None:
            parts.append(f"lidar_front={front:.3f}")
        if ranges:
            finite = sum(math.isfinite(r) and self.devices.lidar_min_range <= r <= self.devices.lidar_max_range
                         for r in ranges)
            no_return = sum(math.isinf(r) and r > 0 for r in ranges)
            parts.append(f"lidar_finite={finite}/{len(ranges)} inf={no_return}")
        timing = self.step_timer.summary()
        if timing:
            median,p95,worst,_ = timing
            parts.append(f"step_ms med={median*1e3:.1f} p95={p95*1e3:.1f} max={worst*1e3:.1f}")
        if self.map_timer.missed:
            parts.append(f"map_missed={self.map_timer.missed}")
        if self.grid is not None:
            frontiers = planning.find_frontiers(self.grid.grid)
            clusters = [c for c in planning.cluster_frontiers(frontiers) if len(c) >= 3]
            parts.append(f"map free={self.grid.count(0)} occ={self.grid.count(1)} frontiers={len(frontiers)} clusters={len(clusters)}")
            dump = os.environ.get("RESCUE_MAP_DUMP")
            if dump:
                self.grid.save_pgm(dump)
        parts.append(f"target_found={self.target['found']}")
        det = self.detection_timer_stats.summary()
        if det:
            parts.append(f"det_ms med={det[0] * 1e3:.1f} max={det[2] * 1e3:.1f}")
        parts.append(f"targets confirmed={len(self.tracker.confirmed())} "
                     f"visited={self.tracker.visited_count()}/{config.REQUIRED_TARGETS}")
        if self.target["found"]:
            parts.append(f"target cx={self.target['cx']} dir={self.target['direction']} "
                         f"area={self.target['area']:.0f}")
        if self.safety_event or self.sensor_fault:
            parts.append(f"safety={self.sensor_fault or self.safety_event}")
        print("[status] " + " | ".join(parts))

class _Tee:
    """콘솔과 RESCUE_LOG 파일에 같은 로그를 기록합니다."""
    def __init__(self, *streams):
        self.streams = streams
    def write(self, text):
        for stream in self.streams:
            stream.write(text)
            stream.flush()
    def flush(self):
        for stream in self.streams:
            stream.flush()

def main():
    # Webots 의존성은 실제 실행 시에만 불러와 main을 오프라인으로 테스트할 수 있습니다.
    from controller import Robot
    log_path = os.environ.get("RESCUE_LOG")
    if log_path:
        log_file = open(log_path, "w", encoding="utf-8")
        sys.stdout = _Tee(sys.stdout, log_file)
        sys.stderr = _Tee(sys.stderr, log_file)
    robot = Robot()
    try:
        mission = RescueMission(robot)
    except DeviceError as error:
        print(f"[main] FATAL: {error}")
        return
    mission.run()

if __name__ == "__main__":
    main()
