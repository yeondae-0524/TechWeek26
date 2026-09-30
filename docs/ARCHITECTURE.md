# Architecture

> 이 문서는 **현재 baseline 코드**의 구조다. 앞으로 구현할 **목표 architecture**(Navigator, SafetyMonitor, log-odds 등)와
> 구현 순서는 [research/10_FINAL_ARCHITECTURE.md](research/10_FINAL_ARCHITECTURE.md),
> [research/11_IMPLEMENTATION_ROADMAP.md](research/11_IMPLEMENTATION_ROADMAP.md)를 기준으로 한다.
> 기능이 코드에 반영되면 이 문서의 표를 갱신한다.

## 모듈 구성 (`controllers/rescue_robot/`)

| 파일 | 역할 | Webots 의존 | 상태 |
|---|---|---|---|
| `rescue_robot.py` | Webots entry point (`main.main()` 호출만) | O | 완료 |
| `main.py` | state machine + 매 step pipeline, 초 단위 주기, 필수 센서 fail-closed | O | 뼈대 (EXPLORE/APPROACH 동작은 TODO) |
| `config.py` | 로봇/맵/안전 파라미터, device 이름 (TurtleBot3 Burger, 태그 표기) | X | 완료 (S0) |
| `devices.py` | Webots device 접근 전부. 필수(모터·엔코더·LDS-01) 없거나 Compass/GPS 설정 시 `DeviceError`, 시작 로그 | O | 완료 |
| `interfaces.py` | 공통 데이터 규격 + validator | X | 완료 |
| `mapping.py` | Occupancy grid, 좌표 변환, LiDAR → grid (mount offset, inf skip, minRange, hit 우선, 영구 ghost 금지) | X | binary 최소 구현 (log-odds TODO) |
| `localization.py` | diff-drive wheel odometry | X | odometry만 (gyro fusion TODO) |
| `detection.py` | `detect_target(frame)` interface | X | **stub** |
| `planning.py` | A*, obstacle inflation, frontier 검출/cluster | X | A*/frontier 완료, 선택 전략 TODO |
| `control.py` | wheel/velocity 명령(바퀴 한계 동시 제한), primitive, `SafetyMonitor` | X | 기본 동작 + safety 기본판, waypoint tracking TODO |
| `scheduling.py` | 초 단위 `Periodic`, step 시간 통계 `StepTimer` | X | 완료 |

Webots `controller` 모듈은 `devices.py`, `main.py`, `rescue_robot.py`에서만 import한다
(`verify_baseline.py`가 검사). 나머지는 Webots 없이 unit test 가능하다.
Compass·GPS·Supervisor pose는 대회 controller 입력으로 쓰지 않는다(운영진 확인, `verify_baseline.py`가 정적 검사).

## 매 step pipeline (`RescueMission.step`)

```text
1. sensors       devices.read_encoders / read_lidar / read_gyro_yaw_rate (매 step)
                 필수 센서(엔코더·LiDAR) invalid -> [safety] 로그 + 정지
2. localization  Localizer.update(encoders)            -> pose (x, y, theta)
3. mapping       OccupancyGrid.insert_scan(pose, ranges, min_range) (MAP_UPDATE_PERIOD = 0.128 s마다)
4. detection     detect_target(read_camera_frame())    -> target dict (DETECTION_PERIOD = 0.128 s마다)
5. planning      state별 handler (frontier / A*)
6. control       DiffDriveController 명령
   safety        SafetyMonitor.filter(v, w)  ← 항상 마지막, raw scan 기준
                 (STOP 반원, minRange 사각, 회전 여유, 후진 금지, stale scan)
```

주기는 step 수가 아니라 **초**로 정의한다(`scheduling.Periodic`). 공식 주행 world는 64 ms, 일부 테스트 world는 32 ms다.

## Mission state machine

```text
INITIALIZE ──► EXPLORE ──(target found)──► APPROACH_TARGET ──► RETURN_HOME ──► DONE
                  │                               │                  ▲
                  └────────(MISSION_TIME_LIMIT)───┴──────────────────┘

(검증용) INITIALIZE ──(RESCUE_MODE=CONTROL_TEST)──► CONTROL_TEST ──► DONE
```

| State | 현재 구현 | TODO |
|---|---|---|
| INITIALIZE | 정지, `home_pose = config.START_POSE` 저장, odometry reset, grid 생성 | - |
| EXPLORE | target 발견/시간초과 전이. 로봇은 **정지 유지** (`follow_waypoint` stub) | frontier 선택 → A* → waypoint tracking |
| APPROACH_TARGET | 정지 유지 | target world 위치 추정, 안전거리 접근, 다수 target 처리 |
| RETURN_HOME | home까지 inflated grid에서 A* 계산(실제 구현), 도착 판정 | path tracking |
| DONE | 정지 | - |

미완성 기능은 로봇을 움직이지 않는다. 즉 baseline 기본 동작(`BASELINE_MODE = "STOP"`)은 **정지**다.

## 대회 world가 공개되면

로봇은 공식 TurtleBot3 Burger로 이행되어 있다([TURTLEBOT3_MIGRATION.md](TURTLEBOT3_MIGRATION.md)).

1. 부팅 로그 `[devices] available / basicTimeStep / synchronization / lidar / camera / gyro`가 config와 같은지 확인
2. `[lidar] ranges (from LiDAR origin): front/left/back/right`가 실제 배치와 맞는지 확인
3. [DAY-OF] 항목 갱신: `START_POSE`(형식), `GRID_WIDTH/HEIGHT`(아레나 크기), `MISSION_TIME_LIMIT`
4. `python scripts/verify_baseline.py --webots --mode CONTROL_TEST` 로 전진=+x, 좌회전=theta 증가 확인
