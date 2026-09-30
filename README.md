# TECH WEEK Autonomous Search and Rescue

부산대학교 TECH WEEK **Autonomous Mobile Robot의 Search & Rescue Mission** 해커톤 팀 저장소.

미션: 사전 지도 없음 · 시작 pose만 제공 · 목표물 위치 모름 → 탐색 → 구조 대상 식별 → 접근 → 시작점 복귀.
정적 장애물·움직이는 사람과 충돌 금지.

**현재 상태: Control 통합 개발 중.** `controllers/rescue_robot/main.py`에 encoder 위치 추정, waypoint 추종, LiDAR 안전 검사와 A* 복귀 경로 추종을 연결했다.
탐색 목표 선택과 구조 대상 접근은 아직 미구현이다. 소형 시험 world에서 Webots 직선/코너 주행은 통과했으며 휴게실/동적 장애물 시나리오는 미검증이다. 기본 모드는 STOP이다.
**현재 상태:** 운영진 공식 baseline(world·예제 controller) 옆에 우리 controller `controllers/rescue_robot/`이 있다.
센서 읽기·odometry·LiDAR 지도·A*·frontier 검출·SafetyMonitor까지 있고, 로봇은 기본적으로 **정지** 상태다.
경로 추종, target 검출, 탐색 전략, 복귀는 각 담당이 구현한다(아래 "역할별 파일").

## 환경

| 항목 | 값 |
|---|---|
| 시뮬레이터 | Webots **R2025a** |
| Python | **3.10** (공식 기준 Ubuntu 22.04, Windows 가능) |
| 로봇 | 공식 **TurtleBot3 Burger** + LDS-01 LiDAR + 640×480 카메라 |
| 라이브러리 | 공식 교육 자료 기준 numpy 1.23.5, opencv-python 4.8.0.74 (추가 dependency는 팀 합의 후) |
| GPU | 필요 없음 |

## 대회 규칙 (운영진 확인)

- **필수 센서**: wheel encoder, 2D LiDAR. **선택**: IMU.
- **사용 금지**: **Compass, GPS**. Supervisor 정답 pose(`tb3_ground_truth`)도 대회 controller 입력으로 쓰지 않는다.

## 폴더 구조

공식 baseline(`kyu-rae-kim/PNU-TECHWEEK-260930` @ `383de18`)의 원본 구조를 유지하고 팀 controller와 테스트를 추가했다.

```text
TechWeek26/
├─ controllers/
│   ├─ rescue_robot/          # 팀 실행 진입점, 경로 추종 및 안전 제어
│   ├─ rescue_control/        # 이전 독립 시험 진입점 및 호환 모듈
│   ├─ rescue_robot/          # 🆕 우리 controller (역할별 파일은 아래 표)
│   ├─ tb3_teleop/            # 키보드 조종 (W/A/S/D)
│   ├─ tb3_teleop_sensors/    # 조종 + 센서 값 Display (compass도 읽지만 대회에선 사용 금지)
│   ├─ tb3_teleop_cam/        # 조종 + OpenCV 색 검출 창
│   ├─ tb3_teleop_yolo/       # 조종 + YOLO (ultralytics 필요, 가중치는 models/YOLO/)
│   ├─ tb3_lidar/             # LiDAR 전/후/좌/우 값 출력
│   ├─ tb3_cam/               # 카메라 영상 창 (OpenCV)
│   ├─ tb3_segmentation/      # OpenCV 색 검출 예제
│   └─ tb3_ground_truth/      # Supervisor 정답 pose 데모 (대회 입력 금지)
├─ worlds/
│   ├─ breakroom_control_test.wbt  # 공식 센서 시험 world의 복사본, rescue_robot 실행
│   ├─ apartment.wbt          # 약 13 m 아파트, 움직이는 사람, 색 사과 (64 ms)
│   ├─ breakroom_teleop.wbt   # 휴게실 + 센서 Display (64 ms)
│   ├─ breakroom_teleop_yolo.wbt, breakroom_ground_truth.wbt   (64 ms)
│   ├─ breakroom_ball.wbt, breakroom_sensor_test.wbt           (32 ms)
│   ├─ empty.wbt
│   ├─ apartment_rescue.wbt, breakroom_teleop_rescue.wbt       # 🆕 복사본, controller = rescue_robot
│   └─ rescue_baseline.wbt    # 🆕 우리가 만든 2 m 검증 world
├─ tests/                     # 🆕 Webots 없이 도는 unit test
├─ scripts/verify_baseline.py # 🆕 한 번에 검증
├─ protos/                    # 공식 world의 사과 PROTO
├─ models/YOLO/               # YOLO 가중치 자리 (commit하지 않음)
├─ docs/research/             # 알고리즘·공식 환경 조사, 목표 architecture, 구현 로드맵
├─ tests/                     # 순수 Python 단위 및 main 통합 테스트
├─ AGENTS.md                  # 작업 규칙 (사람·AI 공통, 먼저 읽기)
└─ README.md
```

공식 world·controller·PROTO는 운영진 허락을 받아 **원본 그대로** 넣었다. 수정하지 않는다.
공식 notebook·pdf는 재배포 금지라 넣지 않았다.

## 처음 받은 팀원

```bash
git clone https://github.com/yeondae-0524/TechWeek26.git
cd TechWeek26
```

1. Webots R2025a, Python 3.10 설치 → `python -m pip install numpy==1.23.5 opencv-python==4.8.0.74`
2. Webots에서 `worlds/breakroom_teleop.wbt` 열기 → 실행(▶) → 3D 화면 클릭 후 W/A/S/D로 로봇이 움직이는지 확인
3. `worlds/breakroom_sensor_test.wbt` 실행 → 콘솔에 LiDAR 전/후/좌/우 값이 나오는지 확인

## 우리 controller 실행

| world | 설명 |
|---|---|
| `worlds/apartment_rescue.wbt` | 공식 apartment 복사본 (약 13 m, 움직이는 사람), 시작 (-0.3, -7.5, 180°) |
| `worlds/breakroom_teleop_rescue.wbt` | 공식 breakroom 복사본, 시작 (-1.265, 1.811, -24.3°) |
| `worlds/rescue_baseline.wbt` | 우리가 만든 2 m 검증 world |

세 world 모두 로봇 controller가 `rescue_robot`이다. 공식 원본 world는 수정하지 않았다(controller·시작 pose 줄만 다름).

1. `controllers/rescue_robot/runtime.ini`의 `COMMAND`가 본인 Python 3.10 경로인지 확인 (다르면 로컬에서만 수정, commit 금지)
2. Webots에서 위 world 중 하나를 열고 실행 → 기본 모드 `STOP`: 로봇은 정지, 콘솔에 2초마다 `[status] ... pose / lidar_front / map` 출력
3. 구동계 확인: 환경변수 `RESCUE_MODE=CONTROL_TEST` 또는 `config.BASELINE_MODE = "CONTROL_TEST"` → 전진/정지/좌회전/정지/우회전/정지

```bash
py -3.10 scripts/verify_baseline.py                                  # 문법·구조·unit test
py -3.10 scripts/verify_baseline.py --webots --world worlds/apartment_rescue.wbt
py -3.10 scripts/verify_baseline.py --webots --mode CONTROL_TEST --world worlds/apartment_rescue.wbt
```

## 역할별 파일 (`controllers/rescue_robot/`)

| 담당 | 파일 | 지금 있는 것 | 할 일 |
|---|---|---|---|
| Mapping + Localization | `localization.py`, `mapping.py` | encoder odometry, LiDAR 좌표 변환, log-odds Occupancy Grid, 영역 초기화 | gyro 보정, 이동 중 지도 품질 검증, (가능하면) Scan Matching, 동적 장애물 필터링 |
| Detection | `detection.py` | `detect_target(frame)` **stub** (항상 found=False) | 목표 검출 → `found/cx/direction/area` |
| Planning | `planning.py` | A*, 장애물 inflation, frontier 검출·clustering | frontier 선택, 목표 접근 경로, 복귀 경로 |
| Control + Local Planning | `control.py` | 바퀴 명령, SafetyMonitor(정지 영역·LiDAR 사각·후진 금지) | `follow_waypoint` 경로 추종, 장애물 회피, recovery |
| 통합 | `main.py`, `devices.py`, `config.py` | state machine, 센서 읽기, 설정 | 모듈 연결, 모드 추가 |

각 모듈은 Webots 없이 테스트한다: `tests/test_<모듈>.py`. 데이터 형식은 `AGENTS.md`의 "팀 규격"을 따른다.

### 매핑 연결 안내

기본 지도 갱신은 구현되어 있어 Planning/Detection 담당이 연결 작업을 시작할 수 있다.
이동 중 지도 정확도와 전체 미션 완료까지 검증된 상태는 아니다.

- `main.py`가 odometry pose와 LiDAR scan으로 `OccupancyGrid.insert_scan()`을 호출한다.
- Planning 입력은 `OccupancyGrid.grid`이며 Python 리스트다. `grid[row][col]`에서 row는 +y,
  col은 +x, 값은 `UNKNOWN=-1`, `FREE=0`, `OCCUPIED=1`이다. 미관측 칸을 빈 공간으로 취급하지 않는다.
- 좌표 변환은 해당 지도 객체의 `world_to_grid(x, y)`와 `grid_to_world(row, col)`을 쓴다.
  전자는 범위 밖 좌표도 반환하므로 `in_bounds()`로 확인한다. 후자는 셀 중심의 미터 좌표다.
- Detection은 기존 `detect_target(frame)`과 `found/cx/direction/area` 규격으로 작업한다.
  검출 결과를 지도상의 목표 위치로 변환하는 기능은 아직 구현되지 않았다.
- log-odds는 관측 증거를 누적하고 상하한을 제한한다. 기존 장애물은 유효한 free ray의 반복 관측으로
  해제되지만, 가려진 장애물이나 `inf`만 반환되는 곳은 자동으로 지워지지 않는다.
- `reset_region((x, y), radius)`는 미터 단위 영역을 UNKNOWN으로 초기화하는 함수다.
  자동 recovery 연결과 동적 장애물 구분은 아직 구현되지 않았다.
- 위치 추정은 encoder odometry이며 gyro 융합과 Scan Matching은 TODO다.
  기본 모드는 계속 `STOP`이고, 경로 추종 및 자율 탐색은 별도 구현이 필요하다.

## Control 주행 시험

[한글 실행 안내](controllers/rescue_robot/README.md)에 모드, 좌표계, 팀 연결 API와 검증 범위를 정리했다.
주 실행 파일은 `controllers/rescue_robot/main.py`이고, 순수 주행 로직은 `navigation_control.py`에 있다.

Webots를 실행하는 PowerShell에서 다음을 설정하고 `worlds/control_arena_test.wbt`를 연다.

```powershell
$env:RESCUE_MODE = 'NAV_TEST'
$env:RESCUE_WAYPOINTS = '[[1,0],[1,1],[0,1]]'
```

시험 waypoint는 시작점 기준 미터 좌표이며, 주행 가능한 시험 공간에서 사용한다.
장애물이 잠깐 막으면 정지 후 경로를 재개하고, 계속 막히면 Planner에 재계획을 요청한다.
전체 구조 미션을 자동 수행하는 완성 controller는 아직 아니다.

## 작업 방식

- `main`에서 branch를 만들어 작업하고(`feat/<module>`) PR로 merge한다. `main`에 직접 push하지 않는다.
- commit 메시지: `<type>: <내용>` (영어 소문자, 예: `feat: add lidar occupancy grid`)
- 자세한 규칙: [AGENTS.md](AGENTS.md)

## 참고 자료

[`docs/research/`](docs/research/00_RESEARCH_INDEX.md)에 검증된 시스템(Nav2, m-explore, Hector 등)과 공식 환경을 조사한 결과가 있다.

| 문서 | 내용 |
|---|---|
| [00_RESEARCH_INDEX](docs/research/00_RESEARCH_INDEX.md) | 목차, 핵심 발견 |
| [09_WEBOTS_REFERENCES](docs/research/09_WEBOTS_REFERENCES.md) | **공식 로봇·센서·world 사실** (치수, LiDAR 순서, 카메라, timestep) |
| [10_FINAL_ARCHITECTURE](docs/research/10_FINAL_ARCHITECTURE.md) | 추천 architecture와 알고리즘 선택 |
| [11_IMPLEMENTATION_ROADMAP](docs/research/11_IMPLEMENTATION_ROADMAP.md) | 단계별 구현 순서(S0–S11), 수용 기준 |
| [12_FAILURE_SCENARIOS](docs/research/12_FAILURE_SCENARIOS.md) | 실패 상황별 대응 |

research 문서의 파일 이름(`config.py`, `mapping.py`, `control.py` 등)은 지금 `controllers/rescue_robot/`의 파일과 같다.
