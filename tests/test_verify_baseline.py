"""현재 한글 controller 로그를 검증 스크립트가 올바르게 판정하는지 확인합니다."""
import importlib.util
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "verify_baseline.py"
spec = importlib.util.spec_from_file_location("verify_baseline", SCRIPT)
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)

HEALTHY_STOP = """[safety] 필수 센서 정상
[main] home_pose=(-0.500, -0.800, +0.0deg), mode=STOP
[state] INITIALIZE -> EXPLORE
[status] t=0.0s | pose=(-0.500, -0.800, +0.0deg)
[status] t=2.0s | pose=(-0.500, -0.800, +0.0deg)
[status] t=4.0s | pose=(-0.500, -0.800, +0.0deg)
[status] t=6.0s | pose=(-0.500, -0.800, +0.0deg)
[status] t=8.0s | pose=(-0.500, -0.800, +0.0deg)
"""


class VerificationLogTests(unittest.TestCase):
    def checks(self, log, mode="STOP"):
        records = {}
        def record(name, ok, detail=""):
            records[name] = bool(ok)
        with patch.object(verify, "report", record):
            verify.check_webots_log(log, mode)
        return records

    def test_healthy_stop_log_passes(self):
        self.assertTrue(all(self.checks(HEALTHY_STOP).values()))

    def test_missing_sensor_confirmation_does_not_pass(self):
        checks = self.checks(HEALTHY_STOP.replace("[safety] 필수 센서 정상", ""))
        self.assertFalse(checks["Webots: required sensors OK (no fail-closed stop)"])

    def test_korean_sensor_fault_is_reported_even_after_recovery(self):
        log = HEALTHY_STOP + "[safety] LiDAR 값 오류 -> STOP\n[safety] 필수 센서 정상\n"
        self.assertFalse(self.checks(log)["Webots: required sensors OK (no fail-closed stop)"])

    def test_stop_motion_is_detected(self):
        log = HEALTHY_STOP.replace("t=2.0s | pose=(-0.500", "t=2.0s | pose=(-0.450")
        self.assertFalse(self.checks(log)["Webots: robot holds position (STOP mode, odometry)"])

    def test_mid_run_motion_then_return_still_fails_stop_check(self):
        log = HEALTHY_STOP.replace("t=4.0s | pose=(-0.500", "t=4.0s | pose=(+0.000")
        self.assertFalse(self.checks(log)["Webots: robot holds position (STOP mode, odometry)"])

    def test_early_stop_log_cannot_pass(self):
        log = "\n".join(HEALTHY_STOP.splitlines()[:5])
        self.assertFalse(self.checks(log)["Webots: robot holds position (STOP mode, odometry)"])

    def test_korean_control_completion_is_recognized(self):
        self.assertTrue(verify.control_test_finished("[state] CONTROL_TEST -> DONE (구동계 점검 종료)"))
        self.assertFalse(verify.control_test_finished("[state] NAV_TEST -> DONE (waypoint 경로 도착)"))

    def test_missing_control_stages_cannot_pass_motion_check(self):
        checks = self.checks(HEALTHY_STOP + "[state] CONTROL_TEST -> DONE (구동계 점검 종료)", "CONTROL_TEST")
        self.assertTrue(checks["Webots: control test finished"])
        self.assertFalse(checks["Webots: forward along heading, left turn = +theta (odometry)"])


if __name__ == "__main__":
    unittest.main()
