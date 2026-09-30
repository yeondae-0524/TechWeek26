"""rescue_robot - Autonomous Search and Rescue baseline controller.

Mission workflow (state machine):

    INITIALIZE -> EXPLORE --(target found)--> APPROACH_TARGET -> RETURN_HOME -> DONE
                     |                                              ^
                     +------------(mission time limit)--------------+

Every control step runs the same pipeline (periods in seconds, config.py):

    sensors -> localization -> mapping (MAP_UPDATE_PERIOD)
            -> detection (DETECTION_PERIOD) -> planning -> control -> safety (last)

Robot: TurtleBot3Burger + LDS-01 (docs/TURTLEBOT3_MIGRATION.md). Encoders and
LiDAR are mandatory (fail closed), Compass / GPS / Supervisor pose are never used.

BASELINE STATUS: exploration target selection, waypoint tracking and target
approach are TODO. Those states hold the robot still instead of faking
behaviour. Set RESCUE_MODE=CONTROL_TEST to run a short drive-train check.
"""

import math
import os
import sys
import time

from controller import Robot

import config
import control
import detection
import mapping
import planning
from devices import DeviceError, Devices
from interfaces import empty_target, make_pose
from localization import Localizer
from scheduling import Periodic, StepTimer

INITIALIZE = "INITIALIZE"
EXPLORE = "EXPLORE"
APPROACH_TARGET = "APPROACH_TARGET"
RETURN_HOME = "RETURN_HOME"
DONE = "DONE"
CONTROL_TEST = "CONTROL_TEST"  # verification only (BASELINE_MODE == "CONTROL_TEST")

# (primitive, description) - each step lasts config.CONTROL_TEST_STEP_DURATION
CONTROL_TEST_SEQUENCE = (
    ("drive_forward", "FORWARD"),
    ("stop", "STOP"),
    ("rotate_left", "LEFT"),
    ("stop", "STOP"),
    ("rotate_right", "RIGHT"),
    ("stop", "STOP"),
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

        custom_pose = d.read_start_pose()
        self.start_pose = custom_pose if custom_pose is not None else make_pose(*config.START_POSE)
        self.start_pose_source = "customData" if custom_pose is not None else "config.START_POSE"
        self.home_pose = None  # set in INITIALIZE, used by RETURN_HOME
        self.localizer = Localizer(self.start_pose, config.WHEEL_RADIUS,
                                   config.AXLE_LENGTH, config.encoder_to_rad())
        self.controller = control.DiffDriveController(
            d.left_motor, d.right_motor, config.WHEEL_RADIUS, config.AXLE_LENGTH,
            config.MAX_WHEEL_SPEED, config.MAX_LINEAR_SPEED, config.MAX_ANGULAR_SPEED)
        self.safety = control.SafetyMonitor.from_config(config)
        self.grid = None
        n = d.lidar_resolution
        self.lidar_angles = [mapping.lidar_angle(i, n, d.lidar_fov) for i in range(n)]

        self.map_timer = Periodic(config.MAP_UPDATE_PERIOD)
        self.detection_timer = Periodic(config.DETECTION_PERIOD)
        self.step_timer = StepTimer()

        self.state = INITIALIZE
        self.state_start_time = 0.0
        self.step_count = 0
        self.last_status_time = -1e9
        self.target = empty_target()
        self.found_targets = []  # TODO(feat/integration): target world positions
        self.home_path = None
        self.home_plan_pending = False
        self.test_index = -1
        self.safety_event = None
        self.sensor_fault = None

    # ================================================================ loop
    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.step()

    def step(self):
        started = time.perf_counter()
        now = self.robot.getTime()
        self.step_count += 1

        # 1. sensors (encoders + LiDAR every step: odometry and safety need them)
        encoders = self.devices.read_encoders()
        ranges = self.devices.read_lidar()
        yaw_rate = self.devices.read_gyro_yaw_rate()
        self.check_required_sensors(encoders, ranges)

        # 2. localization (encoder odometry; gyro is read but not fused yet)
        pose = self.localizer.update(encoders, yaw_rate, self.dt)

        # 3. mapping (one new scan per MAP_UPDATE_PERIOD)
        if self.grid is not None and ranges and self.map_timer.due(now):
            self.grid.insert_scan(pose, ranges, self.devices.lidar_fov, self.devices.lidar_max_range,
                                  min_range=self.devices.lidar_min_range)

        # 4. detection (camera frame only read when due)
        if self.detection_timer.due(now):
            self.target = detection.detect_target(self.devices.read_camera_frame())

        # 5 + 6. planning & control (per state)
        handler = {
            INITIALIZE: self.do_initialize,
            EXPLORE: self.do_explore,
            APPROACH_TARGET: self.do_approach_target,
            RETURN_HOME: self.do_return_home,
            DONE: self.do_done,
            CONTROL_TEST: self.do_control_test,
        }[self.state]
        handler(now, pose)

        # safety monitor, always last (raw scan, independent of the map)
        self.apply_safety(now, encoders, ranges)

        self.step_timer.add(time.perf_counter() - started)
        self.print_status(now, pose, ranges)

    def check_required_sensors(self, encoders, ranges):
        """Encoders and LiDAR are mandatory: log when their data becomes invalid."""
        fault = None
        if encoders is None and self.step_count > 1:  # NaN on the first step is normal
            fault = "encoder data invalid"
        elif ranges is None:
            fault = "lidar data invalid"
        if fault != self.sensor_fault:
            print(f"[safety] {fault} -> STOP" if fault else "[safety] required sensors OK")
            self.sensor_fault = fault

    def apply_safety(self, now, encoders, ranges):
        self.safety.update_scan(ranges, self.lidar_angles, now)
        if encoders is None:  # no odometry -> never move (fail closed)
            self.controller.stop()
        v, w = self.controller.command
        v2, w2, event = self.safety.filter(v, w, now)
        if event is not None:
            self.controller.set_velocity(v2, w2)
        if event != self.safety_event:
            if event is not None:
                print(f"[safety] {event}: command ({v:+.3f} m/s, {w:+.2f} rad/s) -> "
                      f"({v2:+.3f}, {w2:+.2f})")
            self.safety_event = event

    def transition(self, new_state, now, reason=""):
        print(f"[state] {self.state} -> {new_state}" + (f"  ({reason})" if reason else ""))
        self.state = new_state
        self.state_start_time = now

    # ================================================================ states
    def do_initialize(self, now, pose):
        self.controller.stop()
        self.home_pose = self.start_pose
        self.localizer.odometry.reset(self.home_pose)
        self.grid = mapping.OccupancyGrid.centered_on(
            self.home_pose[0], self.home_pose[1], config.GRID_WIDTH, config.GRID_HEIGHT,
            config.GRID_RESOLUTION) if config.GRID_ORIGIN is None else mapping.OccupancyGrid(
            config.GRID_WIDTH, config.GRID_HEIGHT, config.GRID_RESOLUTION, config.GRID_ORIGIN)
        print(f"[main] home_pose = {fmt_pose(self.home_pose)} (from {self.start_pose_source})  "
              f"mode = {config.BASELINE_MODE}")
        self.print_lidar_check()
        if config.BASELINE_MODE == "CONTROL_TEST":
            self.transition(CONTROL_TEST, now, "RESCUE_MODE=CONTROL_TEST")
        else:
            self.transition(EXPLORE, now)

    def do_explore(self, now, pose):
        if self.target["found"]:
            self.found_targets.append({"time": now, "target": dict(self.target), "pose": pose})
            self.transition(APPROACH_TARGET, now, f"target {self.target}")
            return
        if now > config.MISSION_TIME_LIMIT:
            self.transition(RETURN_HOME, now, "mission time limit")
            return
        # TODO(feat/planning): choose a frontier cluster, plan with astar(),
        # hand the next waypoint to the controller. Frontiers are computed in
        # print_status() for monitoring only.
        self.controller.follow_waypoint(pose)  # holds position until implemented

    def do_approach_target(self, now, pose):
        # TODO(feat/integration): estimate target world position from camera
        # direction + lidar range, plan to it, stop at a safe distance, mark it
        # as rescued, then go back to EXPLORE or RETURN_HOME.
        self.controller.stop()
        if now > config.MISSION_TIME_LIMIT:
            self.transition(RETURN_HOME, now, "mission time limit")

    def do_return_home(self, now, pose):
        dist = math.hypot(pose[0] - self.home_pose[0], pose[1] - self.home_pose[1])
        if dist < config.HOME_TOLERANCE:
            self.controller.stop()
            self.transition(DONE, now, f"home reached ({dist:.2f} m)")
            return
        if self.home_path is None and self.grid is not None:
            if not self.home_plan_pending:
                # Blocking A* below: deliver a stop command through robot.step() first.
                self.controller.stop()
                self.home_plan_pending = True
                return
            radius = (config.ROBOT_RADIUS + config.SAFETY_MARGIN) / config.GRID_RESOLUTION
            inflated = planning.inflate_obstacles(self.grid.grid, radius)
            start = self.grid.world_to_grid(pose[0], pose[1])
            goal = self.grid.world_to_grid(self.home_pose[0], self.home_pose[1])
            # Known-only first; optimistic (unknown allowed) only as a fallback (research 08 §2.3).
            self.home_path = planning.astar(inflated, start, goal, allow_unknown=False)
            mode = "known-only"
            if not self.home_path:
                self.home_path = planning.astar(inflated, start, goal, allow_unknown=True)
                mode = "unknown allowed"
            print(f"[plan] home path ({mode}): {len(self.home_path)} cells")
        # TODO(feat/control): track self.home_path waypoint by waypoint.
        self.controller.follow_waypoint(pose)

    def do_done(self, now, pose):
        self.controller.stop()

    def do_control_test(self, now, pose):
        index = int((now - self.state_start_time) // config.CONTROL_TEST_STEP_DURATION)
        if index >= len(CONTROL_TEST_SEQUENCE):
            self.controller.stop()
            self.transition(DONE, now, "control test finished")
            return
        primitive, label = CONTROL_TEST_SEQUENCE[index]
        getattr(self.controller, primitive)()
        if index != self.test_index:
            self.test_index = index
            # Commanded motion vs. LiDAR range change (no GPS: organizer rule).
            front = self.lidar_range_at(0.0)
            front_txt = f"  lidar_front={front:.3f}" if front is not None else ""
            print(f"[control-test] {label:<7} odom={fmt_pose(pose)}{front_txt}")

    # ================================================================ logging
    def lidar_range_at(self, target_angle, ranges=None):
        """Range of the ray closest to target_angle (robot frame, from the LiDAR), or None."""
        ranges = ranges if ranges is not None else self.devices.read_lidar()
        if not ranges:
            return None
        i = min(range(len(ranges)), key=lambda k: abs(math.atan2(
            math.sin(self.lidar_angles[k] - target_angle), math.cos(self.lidar_angles[k] - target_angle))))
        return ranges[i]

    def print_lidar_check(self):
        """One-line range check in the 4 robot directions (axis verification)."""
        ranges = self.devices.read_lidar()
        if not ranges:
            return
        out = [f"{name}={self.lidar_range_at(a, ranges):.3f}"
               for name, a in (("front", 0.0), ("left", math.pi / 2), ("back", math.pi), ("right", -math.pi / 2))]
        print("[lidar] ranges (from LiDAR origin): " + " ".join(out))

    def print_status(self, now, pose, ranges):
        if now - self.last_status_time < config.STATUS_PRINT_PERIOD:
            return
        self.last_status_time = now
        parts = [f"t={now:6.1f}s", f"state={self.state}", f"pose={fmt_pose(pose)}"]
        front = self.lidar_range_at(0.0, ranges) if ranges else None
        if front is not None:
            parts.append(f"lidar_front={front:.3f}")
        timing = self.step_timer.summary()
        if timing:
            med, p95, worst, _ = timing
            parts.append(f"step_ms med={med * 1e3:.1f} p95={p95 * 1e3:.1f} max={worst * 1e3:.1f}")
        if self.map_timer.missed:
            parts.append(f"map_missed={self.map_timer.missed}")
        if self.grid is not None:
            frontiers = planning.find_frontiers(self.grid.grid)
            clusters = planning.cluster_frontiers(frontiers, min_size=3)
            parts.append(f"map free={self.grid.count(0)} occ={self.grid.count(1)} "
                         f"frontiers={len(frontiers)} clusters={len(clusters)}")
            dump = os.environ.get("RESCUE_MAP_DUMP")
            if dump:
                self.grid.save_pgm(dump)
        parts.append(f"target_found={self.target['found']}")
        if self.target["found"]:
            parts.append(f"target cx={self.target['cx']} dir={self.target['direction']} "
                         f"area={self.target['area']:.0f}")
        if self.safety_event or self.sensor_fault:
            parts.append(f"safety={self.sensor_fault or self.safety_event}")
        print("[status] " + " | ".join(parts))


class _Tee:
    """Duplicate console output into a file (RESCUE_LOG env var, used by
    scripts/verify_baseline.py --webots). Tracebacks go to the file as well."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for s in self.streams:
            s.write(text)
            s.flush()

    def flush(self):
        for s in self.streams:
            s.flush()


def main():
    log_path = os.environ.get("RESCUE_LOG")
    if log_path:
        log_file = open(log_path, "w", encoding="utf-8")
        sys.stdout = _Tee(sys.stdout, log_file)
        sys.stderr = _Tee(sys.stderr, log_file)
    robot = Robot()
    try:
        mission = RescueMission(robot)
    except DeviceError as e:
        print(f"[main] FATAL: {e}")
        return
    mission.run()


if __name__ == "__main__":
    main()
