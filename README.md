# TECH WEEK Autonomous Search and Rescue

부산대학교 TECH WEEK **Autonomous Mobile Robot의 Search & Rescue Mission** 해커톤 팀 저장소.

미션: 사전 지도 없음 · 시작 pose만 제공 · 목표물 위치 모름 → 탐색 → 구조 대상 식별 → 접근 → 시작점 복귀.
정적 장애물·움직이는 사람과 충돌 금지.

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

공식 baseline(`kyu-rae-kim/PNU-TECHWEEK-260930` @ `383de18`)과 같은 구조다.

```text
TechWeek26/
├─ controllers/
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
| Mapping + Localization | `localization.py`, `mapping.py` | encoder odometry, LiDAR → 좌표 변환 → 이진 Occupancy Grid | gyro 보정, log-odds 지도, (가능하면) Scan Matching |
| Detection | `detection.py` | `detect_target(frame)` **stub** (항상 found=False) | 목표 검출 → `found/cx/direction/area` |
| Planning | `planning.py` | A*, 장애물 inflation, frontier 검출·clustering | frontier 선택, 목표 접근 경로, 복귀 경로 |
| Control + Local Planning | `control.py` | 바퀴 명령, SafetyMonitor(정지 영역·LiDAR 사각·후진 금지) | `follow_waypoint` 경로 추종, 장애물 회피, recovery |
| 통합 | `main.py`, `devices.py`, `config.py` | state machine, 센서 읽기, 설정 | 모듈 연결, 모드 추가 |

각 모듈은 Webots 없이 테스트한다: `tests/test_<모듈>.py`. 데이터 형식은 `AGENTS.md`의 "팀 규격"을 따른다.

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
