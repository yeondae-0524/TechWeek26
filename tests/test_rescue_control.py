import unittest
from dataclasses import replace
from math import pi, sin, cos, hypot, inf, nan
from controllers.rescue_control.config import DEFAULT
from controllers.rescue_control.odometry import Odometry, wrap_angle
from controllers.rescue_control.control import (Scan, PathFollower, Controller,
    SafetyMonitor, ProgressMonitor, HeadingPID, wheel_speeds, grid_path_to_waypoints)


def scan(now=0.0, hits=None):
    ranges = [3.5]*360
    for i, distance in (hits or {}).items():
        ranges[i] = distance
    return Scan(tuple(ranges), now)


class OdometryTests(unittest.TestCase):
    def test_initial_encoder_offset_does_not_move_robot(self):
        o = Odometry()
        self.assertEqual(o.update(20, 20), (0, 0, 0))
        x, y, a = o.update(20+1/0.033, 20+1/0.033)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(a, 0)

    def test_rotation_and_reverse(self):
        o = Odometry()
        o.update(0, 0)
        rotation = pi/2*0.160/(2*0.033)
        self.assertAlmostEqual(o.update(-rotation, rotation)[2], pi/2)
        self.assertAlmostEqual(o.update(-rotation-1, rotation-1)[1], -0.033)

    def test_midpoint_curve(self):
        o = Odometry()
        o.update(0, 0)
        x, y, a = o.update(0, 1)
        self.assertAlmostEqual(a, 0.033/0.160)
        self.assertAlmostEqual(x, 0.0165*cos(a/2))
        self.assertAlmostEqual(y, 0.0165*sin(a/2))

    def test_corrected_pose_keeps_encoder_baseline(self):
        o = Odometry()
        o.update(0, 0)
        o.update(1, 1)
        o.correct_pose((2, 3, 0))
        self.assertAlmostEqual(o.update(2, 2)[0], 2.033)
        with self.assertRaises(ValueError):
            o.update(nan, 1)
        self.assertEqual(wrap_angle(-pi), pi)


class TrackingTests(unittest.TestCase):
    def test_empty_path_is_stopped(self):
        self.assertEqual(PathFollower().compute((0,0,0)), (0,0,'NO_PATH'))

    def test_straight_and_approach(self):
        f = PathFollower()
        f.set_path([(1,0)])
        v, w, status = f.compute((0,0,0))
        self.assertGreater(v, 0)
        self.assertEqual(w, 0)
        self.assertEqual(status, 'RUNNING')
        self.assertLess(f.compute((0.85,0,0))[0], v)
        self.assertEqual(f.compute((0.95,0,0)), (0,0,'REACHED'))

    def test_corner_rotates_left(self):
        f = PathFollower()
        f.set_path([(1,0),(1,1)])
        v, w, _ = f.compute((1,0,0))
        self.assertEqual(v, 0)
        self.assertGreater(w, 0)

    def test_path_does_not_skip_loop(self):
        f = PathFollower()
        f.set_path([(1,0),(1,1),(0,0)])
        self.assertEqual(f.compute((0,0,0))[2], 'RUNNING')
        self.assertEqual(f.index, 0)

    def test_grid_coordinates_and_empty_path(self):
        self.assertEqual(grid_path_to_waypoints([(2,3)], (-1,-2)), [(-0.825,-1.875)])
        self.assertEqual(grid_path_to_waypoints([], (0,0)), [])
        with self.assertRaises(ValueError):
            grid_path_to_waypoints([(1,-1)], (0,0))
        with self.assertRaises(ValueError):
            PathFollower().set_path([(inf,0)])

    def test_wheel_limit_preserves_curvature(self):
        left, right = wheel_speeds(0.22, 2.75)
        self.assertLessEqual(max(abs(left),abs(right)), 6.67)
        v = (left+right)*0.033/2
        w = (right-left)*0.033/0.160
        self.assertAlmostEqual(w/v, 2.75/0.22)

    def test_pid_wrap_reset_and_invalid_dt(self):
        pid = HeadingPID(replace(DEFAULT, heading_kp=0, heading_kd=1))
        pid.compute(pi-0.01, 0.1)
        self.assertAlmostEqual(pid.compute(-pi+0.01, 0.1), 0.2)
        pid.reset()
        self.assertEqual(pid.compute(0.1,0.1), 0)
        with self.assertRaises(ValueError):
            pid.compute(1,0)

    def test_closed_loop_multi_waypoint(self):
        control = Controller()
        control.set_path([(1,0),(1,1),(0,1)])
        x = y = theta = 0.0
        dt = 0.064
        for step in range(3000):
            now = step*dt
            v, w, status = control.compute((x,y,theta), scan(now), now, dt)
            if status == 'REACHED':
                break
            self.assertNotEqual(status, 'REPLAN_REQUIRED')
            x += v*dt*cos(theta+w*dt/2)
            y += v*dt*sin(theta+w*dt/2)
            theta = wrap_angle(theta+w*dt)
        self.assertEqual(status, 'REACHED')
        self.assertLessEqual(hypot(x, y-1), DEFAULT.goal_tolerance)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.monitor = SafetyMonitor()

    def test_lidar_cardinals_and_mount(self):
        points = Scan((1.0,)*360,0).points()
        for i, expected in [(0,(-1.03,0)),(90,(-0.03,1)),(180,(0.97,0)),(270,(-0.03,-1))]:
            self.assertAlmostEqual(points[i][0], expected[0])
            self.assertAlmostEqual(points[i][1], expected[1])

    def test_front_thin_obstacle_veto(self):
        self.assertEqual(self.monitor.filter(0.15,0,scan(hits={180:0.18}),0)[2], 'STOP')

    def test_rear_obstacle_and_spin_veto(self):
        self.assertEqual(self.monitor.filter(-0.1,0,scan(hits={0:0.125}),0)[2], 'STOP')
        self.assertEqual(self.monitor.filter(0,1,scan(hits={90:0.125}),0)[2], 'STOP')

    def test_forward_prediction(self):
        self.assertEqual(self.monitor.filter(0.15,0,scan(hits={180:0.30}),0)[2], 'PREDICTED_COLLISION')

    def test_slow_zone_min_points(self):
        single = scan(hits={180:0.38})
        group = scan(hits={179:0.38,180:0.38,181:0.38})
        self.assertEqual(self.monitor.filter(0.15,0,single,0)[2], 'CLEAR')
        self.assertEqual(self.monitor.filter(0.15,0,group,0)[0], 0.075)

    def test_unknown_stale_nan_and_incorrect_size(self):
        self.assertEqual(self.monitor.filter(0.1,0,scan(hits={180:inf}),0)[2], 'UNKNOWN_SPACE')
        self.assertEqual(self.monitor.filter(0,1,scan(hits={0:inf}),0)[2], 'UNKNOWN_SPACE')
        self.assertEqual(self.monitor.filter(-0.1,0,scan(hits={0:inf}),0)[2], 'UNKNOWN_SPACE')
        self.assertEqual(self.monitor.filter(0.1,0,scan(),1)[2], 'INVALID_SCAN')
        self.assertEqual(self.monitor.filter(0.1,0,scan(hits={0:nan}),0)[2], 'INVALID_SCAN')
        self.assertEqual(self.monitor.filter(0.1,0,Scan((),0),0)[2], 'INVALID_SCAN')


class IntegrationTests(unittest.TestCase):
    def test_temporary_obstacle_resumes_same_path(self):
        c = Controller()
        c.set_path([(1,0)])
        self.assertEqual(c.compute((0,0,0),scan(hits={180:0.18}),0)[2], 'STOP')
        self.assertGreater(c.compute((0,0,0),scan(1),1)[0], 0)
        self.assertEqual(c.follower.path, [(1,0)])

    def test_persistent_obstacle_latches_replan(self):
        c = Controller()
        c.set_path([(1,0)])
        for now in (0,1,2,3,4):
            result = c.compute((0,0,0),scan(now,{180:0.18}),now)
        self.assertEqual(result, (0,0,'REPLAN_REQUIRED'))
        self.assertEqual(c.compute((0,0,0),scan(5),5), result)
        c.set_grid_path([(0,10)], (0,0))
        self.assertGreater(c.compute((0,0,0),scan(6),6)[0], 0)

    def test_stationary_translation_replans(self):
        c = Controller()
        c.set_path([(1,0)])
        for now in range(11):
            result = c.compute((0,0,0),scan(now),now)
        self.assertEqual(result, (0,0,'REPLAN_REQUIRED'))

    def test_rotation_does_not_trigger_translation_timeout(self):
        p = ProgressMonitor()
        self.assertFalse(p.update((0,0,0),0,False))
        self.assertFalse(p.update((0,0,1),20,False))
        self.assertFalse(p.update((0,0,1),21,True))

    def test_no_path_and_invalid_pose_fail_closed(self):
        c = Controller()
        self.assertEqual(c.compute((0,0,0),None,0), (0,0,'NO_PATH'))
        c.set_path([(1,0)])
        self.assertEqual(c.compute((nan,0,0),scan(),0), (0,0,'INVALID_INPUT'))
        self.assertEqual(c.compute((0,0,0),scan(),0,0), (0,0,'INVALID_TIME'))

class DeviceTests(unittest.TestCase):
    class Device:
        def __init__(self):
            self.velocity = None
            self.enabled = None
        def setPosition(self, position):
            self.position = position
        def setVelocity(self, velocity):
            self.velocity = velocity
        def enable(self, step):
            self.enabled = step
        def getHorizontalResolution(self):
            return 360
        def getFov(self):
            return 2*pi
        def getMinRange(self):
            return 0.12
        def getMaxRange(self):
            return 3.5
        def getValue(self):
            return 0.0
        def getRangeImage(self):
            return [3.5]*360

    class Robot:
        def __init__(self, devices):
            self.devices = devices
            self.requested = []
        def getDevice(self, name):
            self.requested.append(name)
            return self.devices.get(name)
        def getBasicTimeStep(self):
            return 64
        def getTime(self):
            return 1.0

    def make_robot(self):
        names = (DEFAULT.left_motor, DEFAULT.right_motor, DEFAULT.left_encoder,
                 DEFAULT.right_encoder, DEFAULT.lidar)
        return self.Robot({name:self.Device() for name in names})

    def test_wiring_enables_required_sensors_and_starts_stopped(self):
        from controllers.rescue_control.devices import Devices
        robot = self.make_robot()
        devices = Devices(robot)
        self.assertEqual(len(robot.requested), 5)
        self.assertEqual(robot.devices[DEFAULT.left_motor].velocity, 0)
        self.assertEqual(robot.devices[DEFAULT.left_encoder].enabled, 64)
        self.assertEqual(devices.read()[2].timestamp, 1)
        devices.command(0.22, 2.75)
        self.assertLessEqual(abs(robot.devices[DEFAULT.right_motor].velocity), 6.67)
        devices.stop()
        self.assertEqual(robot.devices[DEFAULT.right_motor].velocity, 0)

    def test_missing_required_sensor_keeps_motors_stopped(self):
        from controllers.rescue_control.devices import Devices
        robot = self.make_robot()
        del robot.devices[DEFAULT.lidar]
        with self.assertRaises(RuntimeError):
            Devices(robot)
        self.assertEqual(robot.devices[DEFAULT.left_motor].velocity, 0)
        self.assertEqual(robot.devices[DEFAULT.right_motor].velocity, 0)

    def test_incompatible_lidar_profile_is_rejected(self):
        from controllers.rescue_control.devices import Devices
        robot = self.make_robot()
        robot.devices[DEFAULT.lidar].getHorizontalResolution = lambda: 180
        with self.assertRaises(RuntimeError):
            Devices(robot)
        self.assertEqual(robot.devices[DEFAULT.left_motor].velocity, 0)

if __name__ == '__main__':
    unittest.main()
