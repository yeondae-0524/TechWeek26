"""One-command verification of the rescue baseline.

Usage (from the project root, with Python 3.10):

    python scripts/verify_baseline.py            # syntax + architecture + unit tests
    python scripts/verify_baseline.py --webots   # ... + launch Webots smoke test

The Webots smoke test opens worlds/rescue_baseline.wbt, lets the controller
run for a few seconds, then closes Webots. It checks that the controller
started, reached EXPLORE, printed no traceback and that the robot did not move
(STOP mode). Use --mode CONTROL_TEST to run the drive-train check instead.

Exit code 0 = everything passed.
"""

import argparse
import os
import py_compile
import re
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONTROLLER_DIR = os.path.join(ROOT, "controllers", "rescue_robot")
TESTS_DIR = os.path.join(ROOT, "tests")
WORLD = os.path.join(ROOT, "worlds", "rescue_baseline.wbt")
WEBOTS_EXE = os.environ.get(
    "WEBOTS_EXE",
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots\msys64\mingw64\bin\webots.exe"))

# Only these modules may import the Webots API (keeps the rest testable).
WEBOTS_API_ALLOWED = {"devices.py", "main.py", "rescue_robot.py"}

TEST_GROUPS = [
    ("Grid", "test_grid"),
    ("A*", "test_planning"),
    ("Frontier", "test_frontier"),
    ("Interface", "test_interfaces"),
    ("Control/Odometry", "test_control_localization"),
]

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
                try:
                    py_compile.compile(path, cfile=os.path.join(tempfile.gettempdir(), "vb_" + name + "c"),
                                       doraise=True)
                except py_compile.PyCompileError as e:
                    errors.append(str(e))
    report("Syntax", not errors, "; ".join(errors))


def check_architecture():
    offenders = []
    for name in os.listdir(CONTROLLER_DIR):
        if name.endswith(".py") and name not in WEBOTS_API_ALLOWED:
            with open(os.path.join(CONTROLLER_DIR, name), encoding="utf-8") as f:
                if re.search(r"^\s*(from|import)\s+controller\b", f.read(), re.M):
                    offenders.append(name)
    report("Webots API isolated in devices.py/main.py", not offenders, ", ".join(offenders))


def run_unit_tests():
    sys.path.insert(0, TESTS_DIR)
    loader = unittest.TestLoader()
    for label, module in TEST_GROUPS:
        suite = loader.loadTestsFromName(module)
        res = unittest.TextTestRunner(verbosity=0, stream=open(os.devnull, "w")).run(suite)
        detail = f"{res.testsRun} tests"
        if not res.wasSuccessful():
            detail += "".join(f"\n    {t.id()}: {tb.strip().splitlines()[-1]}"
                              for t, tb in res.failures + res.errors)
        report(f"Tests: {label}", res.wasSuccessful() and res.testsRun > 0, detail)


def run_webots(mode, seconds):
    if not os.path.exists(WEBOTS_EXE):
        report("Webots launch", False, f"webots.exe not found at {WEBOTS_EXE} (set WEBOTS_EXE)")
        return
    log_path = os.path.join(tempfile.gettempdir(), "rescue_baseline_webots.log")
    if os.path.exists(log_path):
        os.remove(log_path)
    env = dict(os.environ, RESCUE_LOG=log_path, RESCUE_MODE=mode, PYTHONUNBUFFERED="1")
    print(f"[....] launching Webots ({mode}), waiting up to {seconds:.0f}s for the controller log ...")
    proc = subprocess.Popen([WEBOTS_EXE, "--mode=realtime", "--stdout", "--stderr", WORLD], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def log_is_complete():
        # Webots needs 10-30 s to load the world, so poll the log instead of a fixed sleep.
        if not os.path.exists(log_path):
            return False
        text = open(log_path, encoding="utf-8").read()
        if "Traceback" in text or "FATAL" in text:
            return True
        if mode == "STOP":
            return text.count("[status]") >= 5
        return "control test finished" in text

    try:
        deadline = time.time() + seconds
        while time.time() < deadline and not log_is_complete():
            time.sleep(1.0)
        time.sleep(1.0)  # let the last lines flush
    finally:
        # webots.exe is a launcher; kill the whole tree without saving the world.
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["taskkill", "/F", "/IM", "webots-bin.exe"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log = open(log_path, encoding="utf-8").read() if os.path.exists(log_path) else ""
    print("----- controller log -----\n" + log.rstrip() + "\n--------------------------")

    report("Webots: controller started", "[main] home_pose" in log)
    report("Webots: no traceback / fatal error", bool(log) and "Traceback" not in log and "FATAL" not in log)
    if mode == "STOP":
        report("Webots: state machine reached EXPLORE", "-> EXPLORE" in log)
        gps = re.findall(r"gps_debug=\(([-+\d.]+), ([-+\d.]+)\)", log)
        if len(gps) >= 2:
            (x0, y0), (x1, y1) = [tuple(map(float, g)) for g in (gps[0], gps[-1])]
            moved = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            report("Webots: robot holds position (STOP mode)", moved < 0.01, f"max drift {moved:.4f} m")
        else:
            report("Webots: robot holds position (STOP mode)", False, "not enough gps_debug status lines")
    else:
        report("Webots: control test finished", "control test finished" in log)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--webots", action="store_true", help="also run the Webots smoke test")
    parser.add_argument("--mode", default="STOP", choices=["STOP", "CONTROL_TEST"])
    parser.add_argument("--seconds", type=float, default=90.0, help="max wait for the log")
    args = parser.parse_args()

    check_python_version()
    check_syntax()
    check_architecture()
    run_unit_tests()
    if args.webots:
        run_webots(args.mode, args.seconds)

    failed = [name for name, ok in results if not ok]
    print("\n" + ("ALL CHECKS PASSED" if not failed else f"{len(failed)} CHECK(S) FAILED: " + ", ".join(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
