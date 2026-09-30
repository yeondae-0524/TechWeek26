"""이전 독립 시험 진입점입니다. 기본 STOP이며 NAV_TEST에는 명시적인 경로가 필요합니다."""
import json
import os
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from controllers.rescue_control.control import Controller
from controllers.rescue_control.devices import Devices
from controllers.rescue_control.odometry import Odometry

def main():
    from controller import Robot
    robot = Robot()
    devices = Devices(robot)
    odometry = Odometry()
    control = Controller()
    mode = os.environ.get('RESCUE_MODE', 'STOP')
    try:
        if mode not in ('STOP', 'NAV_TEST'):
            raise ValueError('supported modes: STOP, NAV_TEST')
        if mode == 'NAV_TEST':
            control.set_path(json.loads(os.environ.get('RESCUE_WAYPOINTS', '[]')))
        print('mode=%s; local home=(0,0,0); no mission/global planner connected' % mode)
        previous_status = None
        while robot.step(devices.step_ms) != -1:
            left, right, scan, now = devices.read()
            pose = odometry.update(left, right)
            if mode == 'NAV_TEST':
                v, w, status = control.compute(pose, scan, now, devices.step_ms/1000)
            else:
                v, w, status = 0.0, 0.0, 'STOP'
            devices.command(v, w)
            if status != previous_status:
                print('control: %s, pose=%s' % (status, pose))
                previous_status = status
    except Exception:
        devices.stop()
        raise
    finally:
        devices.stop()

if __name__ == '__main__':
    main()
