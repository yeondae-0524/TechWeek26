# TECH WEEK Autonomous Search and Rescue — Baseline

부산대학교 TECH WEEK **Autonomous Search and Rescue 해커톤**용 Webots baseline.
목적은 완성품이 아니라, 기능별 branch(Mapping / Localization / Detection / Planning / Control)에서
병렬 개발할 수 있는 **깨끗하고 실행 가능한 기초 구조 + 최소 구현**이다.

미션: 사전 지도 없음 · 시작 pose만 제공 · 목표물 위치 모름 → 탐색 → 구조 대상 식별 → 접근 → 시작점 복귀.
정적 장애물·움직이는 사람과 충돌 금지.

**로봇: 공식 TurtleBot3 Burger + LDS-01 (Webots R2025a).** 전환 내용과 연구 기반 수정 내역은
[docs/TURTLEBOT3_MIGRATION.md](docs/TURTLEBOT3_MIGRATION.md). 센서 규칙 [운영진 확인]: 엔코더·2D LiDAR 필수,
IMU 선택, **Compass·GPS 미사용**, Supervisor pose는 controller 입력이 아니다.

## 환경

| 항목 | 값 |
|---|---|
| Webots | R2025a (`%LOCALAPPDATA%\Programs\Webots`) |
| Python | 3.10 — python.org 기본 설치 경로 `%LOCALAPPDATA%\Programs\Python\Python310\python.exe` |
| Python 연결 | `controllers/rescue_robot/runtime.ini` 가 `$(LOCALAPPDATA)\Programs\Python\Python310\python.exe` 를 지정 (Webots가 변수 치환, PATH·Webots 전역 설정 변경 없음) |
| 외부 package | 없음 (camera frame 변환에만 NumPy 사용 — 이미 설치됨). GPU 불필요 |
| 공식 기준 OS | Ubuntu 22.04 + Python 3.10 — Ubuntu에서는 `runtime.ini`의 `COMMAND`를 로컬에서 `python3.10` 등으로 수정 (commit 금지) |

## 처음 clone한 팀원 설정

1. Webots R2025a, Python 3.10(python.org 설치 프로그램, "per-user" 기본 경로) 설치 후 `python -m pip install numpy`
2. Python 3.10을 **다른 경로**에 설치했다면 `controllers/rescue_robot/runtime.ini`의 `COMMAND`만 로컬에서 수정 (개인 경로는 commit하지 않기)
3. `python scripts/verify_baseline.py --webots` → **ALL CHECKS PASSED** 확인
   (Webots가 기본 경로가 아니면 환경변수 `WEBOTS_EXE=<...\msys64\mingw64\bin\webots.exe>` 또는 Linux `webots` 경로 지정)

## 실행 방법

1. Webots에서 `worlds/rescue_baseline.wbt` 열기 → 실행(▶)
2. 기본 모드 `STOP`: state machine·mapping·safety monitor가 돌지만 **로봇은 정지** 상태를 유지한다.
   콘솔에 2초마다 `[status] ... state=EXPLORE | pose=... | lidar_front=.. | step_ms med/p95/max | map free=.. occ=.. frontiers=..` 출력
3. 구동계 확인: `config.BASELINE_MODE = "CONTROL_TEST"` 또는 환경변수 `RESCUE_MODE=CONTROL_TEST`
   → 전진 / 정지 / 좌회전 / 정지 / 우회전 / 정지 후 DONE (각 단계 `odom=`과 `lidar_front=` 출력)

## 검증 (코드 수정 후 항상 실행)

```bash
python scripts/verify_baseline.py                           # syntax + 구조 검사(GPS/Compass/Supervisor 금지 포함) + unit tests
python scripts/verify_baseline.py --webots                  # + Webots 실행(STOP): 시작/traceback/정지 유지(odometry)
python scripts/verify_baseline.py --webots --mode CONTROL_TEST
```

(`python`은 Python 3.10이어야 한다. 아니면 위 전체 경로로 실행.)
unit test만: `python -m unittest discover -s tests -p "test_*.py"`

## 폴더 구조

```text
webots_test/
├─ worlds/
│   └─ rescue_baseline.wbt     # 우리가 만든 2 m 검증 world: TurtleBot3 Burger + LDS-01 + camera, 64 ms
├─ controllers/
│   └─ rescue_robot/
│       ├─ rescue_robot.py     # Webots entry point → main.main()
│       ├─ main.py             # state machine + pipeline
│       ├─ config.py           # 바뀔 수 있는 값 전부 ([OFFICIAL]/[INITIAL TUNING]/[DAY-OF] 태그)
│       ├─ devices.py          # Webots device 접근
│       ├─ interfaces.py       # 공통 규격
│       ├─ mapping.py / localization.py / detection.py / planning.py / control.py
│       ├─ scheduling.py       # 초 단위 주기(Periodic), step 시간 통계
│       └─ runtime.ini         # Python 3.10 지정
├─ tests/                      # Webots 없이 도는 unittest (test_grid, test_planning, test_frontier,
│                              #   test_interfaces, test_control_localization, test_turtlebot,
│                              #   test_scan_insertion, test_safety, test_scheduling)
├─ docs/  INTERFACES.md  ARCHITECTURE.md  DEVELOPMENT.md  TURTLEBOT3_MIGRATION.md
│   └─ research/               # reference 조사 · 목표 architecture · 구현 로드맵 (00~13)
├─ scripts/verify_baseline.py
├─ README.md  AGENTS.md
```

개발 흐름과 branch별 작업 가이드는 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

## 공통 interface (자세히: docs/INTERFACES.md)

```python
UNKNOWN, FREE, OCCUPIED = -1, 0, 1
grid[row][col]            # row = +y, col = +x, resolution 0.05 m, origin = cell(0,0) 왼쪽 아래 모서리
pose = (x, y, theta)      # ENU world, theta=0 → +x, 반시계 +
target = {"found": False, "cx": None, "direction": None, "area": 0.0}
path = [(row, col), ...]  # 없으면 []
waypoint = (x, y)
```

## 현재 구현 상태

```text
[x] Project structure
[x] Common interface
[x] Coordinate conversion
[x] A*
[x] Basic frontier detection
[x] Differential drive basic control
[x] TurtleBot3 Burger 공식 환경 이행 (S0: config·device·64 ms 주기·계측 로그)
[x] Raw-LiDAR SafetyMonitor 기본판 (STOP 영역, minRange 사각, 회전 여유, 후진 금지, stale 정지)

[ ] Full mapping              (binary ray marking: inf skip·minRange·영구 ghost 금지. log-odds/SLAM 없음)
[ ] Robust localization       (wheel odometry만. gyro는 읽기만, fusion / scan matching 없음)
[ ] Detection implementation  (detect_target는 stub, 항상 found=False)
[ ] Frontier selection strategy
[ ] Waypoint tracking         (follow_waypoint는 정지 유지 stub)
[ ] Dynamic obstacle avoidance
[ ] Recovery behavior
[ ] Full mission integration
```

구현되어 있는 것 (세부):
- `devices.py`: 필수 device(모터·**엔코더·LDS-01**) 없으면 `DeviceError`(로봇 정지), Compass/GPS 설정 시 `DeviceError`,
  선택 device(camera·gyro·accelerometer) 없으면 경고 후 비활성, 시작 로그(timestep·synchronization·센서 스펙), customData 시작 pose
- `mapping.py`: grid 생성, world↔grid 변환, bounds 검사, LiDAR polar→local→world(mount offset), Bresenham ray로 FREE/OCCUPIED 표시
  (LiDAR 원점에서 시작, inf skip, minRange 안 셀 보존, hit 우선), PGM 덤프
- `localization.py`: wheel radius / axle length / encoder 단위를 주입받는 diff-drive odometry
- `planning.py`: 4-neighbour A* (Manhattan heuristic), obstacle inflation, frontier 검출 + 8-연결 clustering
- `control.py`: `set_wheel_speeds`, `set_velocity(v, w)`(바퀴 한계로 v·ω 동시 제한), `stop/drive_forward/rotate_left/rotate_right`,
  `set_target_waypoint`, `SafetyMonitor`(매 step 마지막 적용)
- `main.py`: INITIALIZE → EXPLORE → APPROACH_TARGET → RETURN_HOME → DONE, `home_pose` 저장, 초 단위 mapping/detection 주기,
  필수 센서 invalid → 정지, RETURN_HOME에서 known-only → unknown 허용 순서로 A* 계산

## Research & 구현 로드맵

검증된 시스템(Nav2, m-explore, Hector, GBPlanner 등)의 소스를 조사해 정리한 결과는 [`docs/research/`](docs/research/00_RESEARCH_INDEX.md)에 있다.

| 문서 | 내용 |
|---|---|
| [00_RESEARCH_INDEX](docs/research/00_RESEARCH_INDEX.md) | 목차, 핵심 발견, 당일 확인 목록 |
| [10_FINAL_ARCHITECTURE](docs/research/10_FINAL_ARCHITECTURE.md) | **목표 architecture와 최종 결정** (Q1–Q15) |
| [11_IMPLEMENTATION_ROADMAP](docs/research/11_IMPLEMENTATION_ROADMAP.md) | M0 → MVP(S0–S9) → Stable → Competitive → Stretch, 수용 기준, 작업 prompt |
| [12_FAILURE_SCENARIOS](docs/research/12_FAILURE_SCENARIOS.md) | 실패 상황별 대응 |

요약: ROS 없이 Webots + Python classical stack · Encoder+Gyro localization · log-odds mapping ·
거리장 기반 frontier 선택 · A* + RPP-lite path follower + raw LiDAR SafetyMonitor · 단계적 recovery ·
NumPy HSV detection + multi-frame 확인 · 시간 예산 기반 복귀. 먼저 **M0 Walking Skeleton**으로 끝까지 연결한다.

팀 결정 대기: `navigation.py` 신규 여부, OpenCV 허용 여부, Scan Matching 우선순위 (10 §6·11 §3.1 참고).

## 참고

- 로봇 값은 공식 TurtleBot3 Burger 기준([OFFICIAL]). 안전거리·속도·주기는 [INITIAL TUNING]이므로 Webots에서 튜닝하고,
  대회 world가 공개되면 [DAY-OF] 항목(timestep, 시작 pose 형식, 아레나 크기, 미션 시간)을 다시 확인한다.
- GPS·Compass는 대회 규정상 사용하지 않는다(verify가 정적 검사). 검증은 엔코더 odometry와 LiDAR 거리 변화로 한다.
- 공식 TECH WEEK repo의 controller·world·notebook은 read-only 참고만 하고 이 저장소로 복사하지 않는다.
- `.gitignore`에 `__pycache__/`, 디버그 출력(`*.pgm`)이 포함되어 있다.
