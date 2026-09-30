# TECH WEEK Autonomous Search and Rescue

부산대학교 TECH WEEK **Autonomous Mobile Robot의 Search & Rescue Mission** 해커톤 팀 저장소.

미션: 사전 지도 없음 · 시작 pose만 제공 · 목표물 위치 모름 → 탐색 → 구조 대상 식별 → 접근 → 시작점 복귀.
정적 장애물·움직이는 사람과 충돌 금지.

**현재 상태: Control 통합 개발 중.** `controllers/rescue_robot/main.py`에 encoder 위치 추정, waypoint 추종, LiDAR 안전 검사와 A* 복귀 경로 추종을 연결했다.
탐색 목표 선택과 구조 대상 접근은 아직 미구현이다. 소형 시험 world에서 Webots 직선/코너 주행은 통과했으며 휴게실/동적 장애물 시나리오는 미검증이다. 기본 모드는 STOP이다.

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
│   └─ empty.wbt
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

## 우리 controller 만들기

1. `controllers/<이름>/<이름>.py` 생성 (Webots는 폴더 이름과 같은 `.py`를 실행한다)
2. 공식 world를 복사해 새 이름으로 저장 (예: Webots에서 `apartment.wbt` 열기 → File → Save World As → `apartment_rescue.wbt`)
3. 복사본에서 `TurtleBot3Burger`의 `controller` 필드를 `<이름>`으로 변경 (원본 world는 수정하지 않는다)
4. 로봇 시작 pose: apartment `(-0.3, -7.5, 180°)`, breakroom 계열 `(-1.265, 1.811, -24.3°)`

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

research 문서는 이전 baseline의 파일 이름(`config.py`, `mapping.py` 등)을 예로 들고 있다. 새 코드의 구조는 팀이 정하고,
알고리즘·수치 근거로만 참고한다.
