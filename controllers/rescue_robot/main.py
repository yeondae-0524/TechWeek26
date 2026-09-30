"""rescue_robot - Autonomous Search and Rescue baseline controller.

Mission workflow (state machine):

    INITIALIZE -> EXPLORE --(target found)--> APPROACH_TARGET -> RETURN_HOME -> DONE
                     |                                              ^
                     +------------(mission time limit)--------------+

Every control step runs the same pipeline:

    sensors -> localization -> mapping -> detection -> planning -> control

BASELINE STATUS: exploration target selection, waypoint tracking and target
approach are TODO. Those states hold the robot still instead of faking
behaviour. Set RESCUE_MODE=CONTROL_TEST to run a short drive-train check.
"""

import math
import os
import sys

from controller import Robot

import config
import control
import detection
import mapping
import planning
from devices import DeviceError, Devices
from interfaces import make_pose
from localization import Localizer

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

        self.home_pose = None  # set in INITIALIZE, used by RETURN_HOME
        self.localizer = Localizer(config.START_POSE, config.WHEEL_RADIUS,
                                   config.AXLE_LENGTH, config.encoder_to_rad())
        self.controller = control.DiffDriveController(
            d.left_motor, d.right_motor, config.WHEEL_RADIUS, config.AXLE_LENGTH,
            config.MAX_WHEEL_SPEED, config.MAX_LINEAR_SPEED, config.MAX_ANGULAR_SPEED)
        self.grid = None
        self.lidar_angles = None
        if d.lidar is not None:
            n = d.lidar_resolution
            self.lidar_angles = [mapping.lidar_angle(i, n, d.lidar_fov) for i in range(n)]
        else:
            print("[main] WARNING: no lidar -> mapping and emergency stop disabled")

        self.state = INITIALIZE
        self.state_start_time = 0.0
        self.step_count = 0
        self.last_status_time = -1e9
        self.target = None
        self.found_targets = []  # TODO(feat/integration): target world positions
        self.home_path = None
        self.test_index = -1

    # ================================================================ loop
    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.step()

    def step(self):
        now = self.robot.getTime()
        self.step_count += 1

        # 1. sensors
        encoders = self.devices.read_encoders()
        ranges = self.devices.read_lidar()
        frame = self.devices.read_camera_frame()
        yaw_rate = self.devices.read_gyro_yaw_rate()

        # 2. localization
        pose = self.localizer.update(encoders, yaw_rate, self.dt)

        # 3. mapping
        if self.grid is not None and ranges and self.step_count % config.MAP_UPDATE_PERIOD_STEPS == 0:
            self.grid.insert_scan(pose, ranges, self.devices.lidar_fov, self.devices.lidar_max_range)

        # 4. detection
        self.target = detection.detect_target(frame)

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

        # safety hook, always last
        front = None
        if ranges and self.lidar_angles:
            front = control.min_front_distance(ranges, self.lidar_angles, config.EMERGENCY_FRONT_HALF_ANGLE)
        was_active = self.controller.emergency_stop_active
        if self.controller.apply_emergency_stop(front, config.EMERGENCY_STOP_DISTANCE) and not was_active:
            print(f"[safety] EMERGENCY STOP: obstacle at {front:.3f} m in front")

        self.print_status(now, pose, ranges)

    def transition(self, new_state, now, reason=""):
        print(f"[state] {self.state} -> {new_state}" + (f"  ({reason})" if reason else ""))
        self.state = new_state
        self.state_start_time = now

    # ================================================================ states
    def do_initialize(self, now, pose):
        self.controller.stop()
        self.home_pose = make_pose(*config.START_POSE)
        self.localizer.odometry.reset(self.home_pose)
        if self.devices.lidar is not None:
            self.grid = mapping.OccupancyGrid.centered_on(
                self.home_pose[0], self.home_pose[1], config.GRID_WIDTH, config.GRID_HEIGHT,
                config.GRID_RESOLUTION) if config.GRID_ORIGIN is None else mapping.OccupancyGrid(
                config.GRID_WIDTH, config.GRID_HEIGHT, config.GRID_RESOLUTION, config.GRID_ORIGIN)
        print(f"[main] home_pose = {fmt_pose(self.home_pose)}  mode = {config.BASELINE_MODE}")
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
            radius = (config.ROBOT_RADIUS + config.SAFETY_MARGIN) / config.GRID_RESOLUTION
            inflated = planning.inflate_obstacles(self.grid.grid, radius)
            self.home_path = planning.astar(inflated, self.grid.world_to_grid(pose[0], pose[1]),
                                            self.grid.world_to_grid(self.home_pose[0], self.home_pose[1]))
            print(f"[plan] home path: {len(self.home_path)} cells")
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
            gps = self.devices.read_gps_debug()
            gps_txt = f"  gps_debug=({gps[0]:+.3f}, {gps[1]:+.3f})" if gps else ""
            print(f"[control-test] {label:<7} odom={fmt_pose(pose)}{gps_txt}")

    # ================================================================ logging
    def print_lidar_check(self):
        """One-line range check in the 4 robot directions (axis verification)."""
        ranges = self.devices.read_lidar()
        if not ranges or not self.lidar_angles:
            return
        out = []
        for name, target_angle in (("front", 0.0), ("left", math.pi / 2), ("back", math.pi), ("right", -math.pi / 2)):
            i = min(range(len(ranges)), key=lambda k: abs(math.atan2(
                math.sin(self.lidar_angles[k] - target_angle), math.cos(self.lidar_angles[k] - target_angle))))
            out.append(f"{name}={ranges[i]:.3f}")
        print("[lidar] ranges: " + " ".join(out))

    def print_status(self, now, pose, ranges):
        if now - self.last_status_time < config.STATUS_PRINT_PERIOD:
            return
        self.last_status_time = now
        parts = [f"t={now:6.1f}s", f"state={self.state}", f"pose={fmt_pose(pose)}"]
        gps = self.devices.read_gps_debug()
        if gps:
            parts.append(f"gps_debug=({gps[0]:+.3f}, {gps[1]:+.3f})")
        if self.grid is not None:
            frontiers = planning.find_frontiers(self.grid.grid)
            clusters = planning.cluster_frontiers(frontiers, min_size=3)
            parts.append(f"map free={self.grid.count(0)} occ={self.grid.count(1)} "
                         f"frontiers={len(frontiers)} clusters={len(clusters)}")
            dump = os.environ.get("RESCUE_MAP_DUMP")
            if dump:
                self.grid.save_pgm(dump)
        parts.append(f"target_found={self.target['found']}")
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
