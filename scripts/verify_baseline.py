"""Python 3.10으로 팀 controller의 문법·구조·전체 단위 테스트를 검사합니다.

    python scripts/verify_baseline.py
    python scripts/verify_baseline.py --webots

--webots는 별도 숨겨진 Webots 프로세스로 선택한 world를 실행합니다.
기본 STOP에서는 encoder pose가 유지되는지, CONTROL_TEST에서는 구동계
점검이 완료되는지 확인합니다. 검증 로그는 임시 폴더에 보관합니다.
"""

import argparse
import math
import os
import py_compile
import re
import socket
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONTROLLER_DIR = os.path.join(ROOT, "controllers", "rescue_robot")
TESTS_DIR = os.path.join(ROOT, "tests")
WORLD = os.path.join(ROOT, "worlds", "rescue_baseline.wbt")
IS_WINDOWS = os.name == "nt"
WEBOTS_CANDIDATES = (
    (os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots\msys64\mingw64\bin\webots.exe"),
     r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe")
    if IS_WINDOWS else ("/usr/local/bin/webots", "/usr/bin/webots", "/snap/bin/webots"))
WEBOTS_EXE = os.environ.get(
    "WEBOTS_EXE", next((p for p in WEBOTS_CANDIDATES if os.path.exists(p)), WEBOTS_CANDIDATES[0]))

# Only these modules may import the Webots API (keeps the rest testable).
WEBOTS_API_ALLOWED = {"devices.py", "main.py", "rescue_robot.py"}

TEST_GROUPS = [
    ("Grid", "test_grid"),
    ("A*", "test_planning"),
    ("Frontier", "test_frontier"),
    ("Interface", "test_interfaces"),
    ("Control/Odometry", "test_control_localization"),
    ("TurtleBot3 profile", "test_turtlebot"),
    ("Scan insertion", "test_scan_insertion"),
    ("Log-odds mapping", "test_logodds"),
    ("Safety monitor", "test_safety"),
    ("Scheduling", "test_scheduling"),
    ("Rescue worlds", "test_rescue_worlds"),
    ("Detection", "test_detection"),
    ("HSV tuning tool", "test_tune_hsv"),
]

# Ground-truth style inputs the competition controller must never use
# (organizer rule: no Compass / GPS; Supervisor pose is demo/test only).
FORBIDDEN_CONTROLLER_PATTERNS = (
    (r"import[^\n]*\bSupervisor\b|\bSupervisor\s*\(", "Supervisor"),
    (r"\bgetSelf\s*\(|\bgetFromDef\s*\(", "Supervisor node access"),
    (r"\bGPS\s*\(|\bCompass\s*\(", "GPS/Compass device class"),
    (r"getDevice\(\s*[\"'](gps|compass)", "GPS/Compass device name"),
)

results = []


def report(name, ok, detail=""):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))


def check_python_version():
    ok = sys.version_info[:2] == (3, 10)
    report("Python 3.10", ok, sys.version.split()[0] + ("" if ok else " (expected 3.10.x)"))


def check_syntax():
    errors = []
    for folder in (CONTROLLER_DIR, TESTS_DIR, os.path.dirname(__file__)):
        for name in sorted(os.listdir(folder)):
            if name.endswith(".py"):
                path = os.path.join(folder, name)
                # 동시에 검증해도 다른 실행의 임시 bytecode와 충돌하지 않습니다.
                fd, compiled_path = tempfile.mkstemp(prefix="rescue_syntax_", suffix=".pyc")
                os.close(fd)
                try:
                    py_compile.compile(path, cfile=compiled_path, doraise=True)
                except (py_compile.PyCompileError, OSError) as e:
                    errors.append(str(e))
                finally:
                    os.remove(compiled_path)
    report("Syntax", not errors, "; ".join(errors))


def check_architecture():
    offenders = []
    for name in os.listdir(CONTROLLER_DIR):
        if name.endswith(".py") and name not in WEBOTS_API_ALLOWED:
            with open(os.path.join(CONTROLLER_DIR, name), encoding="utf-8") as f:
                if re.search(r"^\s*(from|import)\s+controller\b", f.read(), re.M):
                    offenders.append(name)
    report("Webots API isolated in devices.py/main.py", not offenders, ", ".join(offenders))


def check_forbidden_inputs():
    offenders = []
    for name in sorted(os.listdir(CONTROLLER_DIR)):
        if name.endswith(".py"):
            with open(os.path.join(CONTROLLER_DIR, name), encoding="utf-8") as f:
                code = f.read()
            offenders += [f"{name}: {label}" for pattern, label in FORBIDDEN_CONTROLLER_PATTERNS
                          if re.search(pattern, code)]
    sys.path.insert(0, CONTROLLER_DIR)
    import config
    offenders += [f"config.DEVICE_NAMES[{k!r}] = {v!r}" for k, v in config.DEVICE_NAMES.items()
                  if v and any(bad in v.lower() for bad in ("gps", "compass"))]
    report("No GPS/Compass/Supervisor pose in controller", not offenders, "; ".join(offenders))


def run_unit_tests():
    """새 테스트 파일도 빠짐없이 실행하고 모듈별 결과를 출력합니다."""
    sys.path.insert(0, ROOT)
    sys.path.insert(0, TESTS_DIR)
    labels = dict((module, label) for label, module in TEST_GROUPS)
    loader = unittest.TestLoader()
    modules = sorted(name[:-3] for name in os.listdir(TESTS_DIR)
                     if name.startswith("test_") and name.endswith(".py"))
    for module in modules:
        suite = loader.loadTestsFromName(module)
        with open(os.devnull, "w") as stream:
            res = unittest.TextTestRunner(verbosity=0, stream=stream).run(suite)
        detail = f"{res.testsRun} tests"
        if not res.wasSuccessful():
            detail += "".join(f"\n    {t.id()}: {tb.strip().splitlines()[-1]}"
                              for t, tb in res.failures + res.errors)
        report(f"Tests: {labels.get(module, module)}", res.wasSuccessful() and res.testsRun > 0, detail)
    if not modules:
        report("Unit tests", False, "테스트 파일이 없습니다")


def control_test_finished(log):
    """현재 한글 상태 전이와 이전 영어 완료 로그를 모두 인식합니다."""
    return ("[state] CONTROL_TEST -> DONE" in log
            or "control test finished" in log)


def check_webots_log(log, mode):
    """실제 controller 로그를 검사합니다. 정상 센서 기록이 없으면 통과시키지 않습니다."""
    report("Webots: controller started", "[main] home_pose" in log)
    report("Webots: no traceback / fatal error", bool(log) and "Traceback" not in log and "FATAL" not in log)
    healthy = "[safety] 필수 센서 정상" in log or "[safety] required sensors OK" in log
    faults = re.search(r"\[safety\] (?:.* -> STOP|\w+ data invalid|REQUIRED_SENSOR_INVALID|INVALID_SCAN)", log)
    report("Webots: required sensors OK (no fail-closed stop)", healthy and not faults)
    if mode == "STOP":
        report("Webots: state machine reached EXPLORE", "-> EXPLORE" in log)
        poses = re.findall(r"\[status\].*?pose=\(([-+\d.]+), ([-+\d.]+), ([-+\d.]+)deg\)", log)
        if len(poses) >= 5:
            samples = [tuple(map(float, p)) for p in poses]
            x0, y0, t0 = samples[0]
            # 출발점으로 돌아온 경우에도 중간의 이동이나 회전을 놓치지 않습니다.
            moved = max(math.hypot(x - x0, y - y0) for x, y, _ in samples)
            turned = max(abs((theta - t0 + 180.0) % 360.0 - 180.0) for _, _, theta in samples)
            report("Webots: robot holds position (STOP mode, odometry)", moved < 0.01 and turned < 1.0,
                   f"max drift {moved:.4f} m, {turned:.1f} deg")
        else:
            report("Webots: robot holds position (STOP mode, odometry)", False, "상태 로그 부족")
    else:
        report("Webots: control test finished", control_test_finished(log))
        steps = re.findall(r"\[control-test\] (\w+)\s+odom=\(([-+\d.]+), ([-+\d.]+), ([-+\d.]+)deg\)", log)
        if len(steps) >= 6:
            (x0, y0, t0), (x1, y1, _) = [tuple(map(float, s[1:])) for s in (steps[0], steps[1])]
            fwd = (x1 - x0) * math.cos(math.radians(t0)) + (y1 - y0) * math.sin(math.radians(t0))
            left = (float(steps[3][3]) - float(steps[2][3]) + 180.0) % 360.0 - 180.0
            blocked = bool(re.search(r"\[safety\] (STOP_ZONE|BLIND_ZONE)", log))
            report("Webots: forward along heading, left turn = +theta (odometry)",
                   (fwd > 0.02 or blocked) and left > 10.0,
                   f"forward={fwd:+.3f} m, dtheta={left:+.1f} deg" + (" (safety stop seen)" if blocked else ""))
        else:
            report("Webots: forward along heading, left turn = +theta (odometry)", False, "구동계 단계 로그 부족")


def run_webots(mode, seconds, world=WORLD):
    if not os.path.exists(WEBOTS_EXE):
        report("Webots launch", False, f"Webots not found at {WEBOTS_EXE} (set WEBOTS_EXE)")
        return
    if not os.path.isfile(world):
        report("Webots launch", False, f"world not found: {world}")
        return
    fd, log_path = tempfile.mkstemp(prefix="rescue_baseline_", suffix=".log")
    os.close(fd)
    output_path = log_path + ".webots.log"
    env = dict(os.environ, RESCUE_LOG=log_path, RESCUE_MODE=mode, PYTHONUNBUFFERED="1")
    # 실행 중인 다른 Webots와 독립적인 통신 포트를 사용합니다.
    with socket.socket() as port_socket:
        port_socket.bind(("127.0.0.1", 0))
        port = port_socket.getsockname()[1]
    args = [WEBOTS_EXE, "--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr",
            f"--port={port}", world]
    launch_options = {}
    if IS_WINDOWS:
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = subprocess.SW_HIDE
        launch_options = {"startupinfo": startup, "creationflags": subprocess.CREATE_NO_WINDOW}
    print(f"[....] Webots ({mode}), 최대 {seconds:.0f}초, world: {world}")
    print(f"[....] controller 로그: {log_path}")

    def read_log():
        with open(log_path, encoding="utf-8") as stream:
            return stream.read()

    with open(output_path, "w", encoding="utf-8") as output:
        proc = subprocess.Popen(args, env=env, stdout=output, stderr=output, **launch_options)
        completed = False
        natural_exit_code = None
        try:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                log = read_log()
                complete = log.count("[status]") >= 5 if mode == "STOP" else control_test_finished(log)
                if complete:
                    completed = True
                    break
                if "Traceback" in log or "FATAL" in log or proc.poll() is not None:
                    break
                time.sleep(0.5)
        finally:
            # 검증을 위해 종료한 코드와 그 전에 발생한 비정상 종료를 구분합니다.
            natural_exit_code = proc.poll()
            # 이번 검증에서 만든 프로세스만 종료합니다. world를 저장하지 않습니다.
            if natural_exit_code is None:
                if IS_WINDOWS:
                    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
                else:
                    proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=10)
    log = read_log()
    print("----- controller log -----\n" + log.rstrip() + "\n--------------------------")
    if not log:
        print(f"[....] Webots 시작 오류 확인: {output_path}")
    report("Webots: smoke test completed", completed, "" if completed else "시간 초과 또는 조기 종료")
    report("Webots: no unexpected process failure", natural_exit_code in (None, 0),
           "" if natural_exit_code in (None, 0) else f"exit code {natural_exit_code}")
    check_webots_log(log, mode)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--webots", action="store_true", help="also run the Webots smoke test")
    parser.add_argument("--mode", default="STOP", choices=["STOP", "CONTROL_TEST"])
    parser.add_argument("--seconds", type=float, default=90.0, help="max wait for the log")
    parser.add_argument("--world", default=WORLD, help="world file for --webots (default: worlds/rescue_baseline.wbt)")
    args = parser.parse_args()

    check_python_version()
    check_syntax()
    check_architecture()
    check_forbidden_inputs()
    run_unit_tests()
    if args.webots:
        run_webots(args.mode, args.seconds, os.path.abspath(args.world))

    failed = [name for name, ok in results if not ok]
    print("\n" + ("ALL CHECKS PASSED" if not failed else f"{len(failed)} CHECK(S) FAILED: " + ", ".join(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
