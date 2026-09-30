# Architecture

> 이 문서는 **현재 baseline 코드**의 구조다. 앞으로 구현할 **목표 architecture**(Navigator, SafetyMonitor, log-odds 등)와
> 구현 순서는 [research/10_FINAL_ARCHITECTURE.md](research/10_FINAL_ARCHITECTURE.md),
> [research/11_IMPLEMENTATION_ROADMAP.md](research/11_IMPLEMENTATION_ROADMAP.md)를 기준으로 한다.
> 기능이 코드에 반영되면 이 문서의 표를 갱신한다.

## 모듈 구성 (`controllers/rescue_robot/`)

| 파일 | 역할 | Webots 의존 | 상태 |
|---|---|---|---|
| `rescue_robot.py` | Webots entry point (`main.main()` 호출만) | O | 완료 |
| `main.py` | state machine + 매 step pipeline | O | 뼈대 (EXPLORE/APPROACH 동작은 TODO) |
| `config.py` | 로봇/맵/안전 파라미터, device 이름 | X | 완료 (PRACTICE DEFAULT 값 포함) |
| `devices.py` | Webots device 접근 전부. 없는 device는 명확한 메시지 | O | 완료 |
| `interfaces.py` | 공통 데이터 규격 + validator | X | 완료 |
| `mapping.py` | Occupancy grid, 좌표 변환, LiDAR → grid 최소 구현 | X | 최소 구현 |
| `localization.py` | diff-drive wheel odometry | X | odometry만 |
| `detection.py` | `detect_target(frame)` interface | X | **stub** |
| `planning.py` | A*, obstacle inflation, frontier 검출/cluster | X | A*/frontier 완료, 선택 전략 TODO |
| `control.py` | wheel/velocity 명령, primitive, emergency stop hook | X | 기본 동작 완료, waypoint tracking TODO |

Webots `controller` 모듈은 `devices.py`, `main.py`, `rescue_robot.py`에서만 import한다
(`verify_baseline.py`가 검사). 나머지는 Webots 없이 unit test 가능하다.

## 매 step pipeline (`RescueMission.step`)

```text
1. sensors       devices.read_encoders / read_lidar / read_camera_frame / read_gyro_yaw_rate
2. localization  Localizer.update(encoders)            -> pose (x, y, theta)
3. mapping       OccupancyGrid.insert_scan(pose, ranges) (MAP_UPDATE_PERIOD_STEPS마다)
4. detection     detection.detect_target(frame)         -> target dict
5. planning      state별 handler (frontier / A*)
6. control       DiffDriveController 명령
   safety        apply_emergency_stop(front LiDAR 최소거리)  ← 항상 마지막
```

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

## 당일 로봇이 바뀌면

1. `config.DEVICE_NAMES` 수정 (부팅 로그 `[devices] available: ...` 참고)
2. `WHEEL_RADIUS`, `AXLE_LENGTH`, `MAX_WHEEL_SPEED`, `ROBOT_RADIUS`, `ENCODER_UNITS`,
   `GYRO_RAW_TO_RAD_S`, `LIDAR_*`, `START_POSE` 확인 (제공 문서/proto 값으로, 추측 금지)
3. `python scripts/verify_baseline.py --webots --mode CONTROL_TEST` 로 전진=+x, 좌회전=theta 증가 확인
