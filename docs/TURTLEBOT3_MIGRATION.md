# TurtleBot3 Burger 전환 (S0: 공식 환경 이행)

대상: 공식 TECH WEEK 로봇 = Webots R2025a **TurtleBot3Burger** + 기본 **RobotisLds01 (LDS-01)** + 640×480 카메라.
근거 문서: [research/09](research/09_WEBOTS_REFERENCES.md)(공식 환경 사실), [research/10](research/10_FINAL_ARCHITECTURE.md) §0·§2.2·§11,
[research/11](research/11_IMPLEMENTATION_ROADMAP.md) S0 / Prompt 0, [research/05](research/05_LOCAL_PLANNING_SAFETY.md) §3.

이 전환은 팀원이 만든 TurtleBot3 전환본(zip)의 **코드 구조를 그대로 채택**하고,
2026-09-30 공식 repo 재검증 연구(research 00–13)와 **충돌하는 부분만 수정**한 것이다.
S1(gyro fusion) 이후 알고리즘은 구현하지 않았다 — 아래 "아직 구현되지 않은 기능" 참고.

## 1. 실행

1. Webots R2025a에서 `worlds/rescue_baseline.wbt`(우리가 만든 2 m × 2 m 검증 월드)를 열고 실행한다.
2. `controllers/rescue_robot/runtime.ini`의 `COMMAND`가 본인 Python 3.10(+NumPy) 경로인지 확인한다.
   기본값은 Windows `$(LOCALAPPDATA)` 경로다. 공식 기준 OS는 Ubuntu 22.04이므로 Ubuntu에서는
   `COMMAND = python3.10` 등으로 **로컬에서만** 수정한다(개인 경로 commit 금지).
3. 기본 `STOP`: 지도·위치 추정·safety만 돌고 로봇은 정지. 부팅 로그에서 timestep / synchronization /
   LDS-01 스펙 / camera / gyro 유무 / `[lidar] ranges (from LiDAR origin): front=.. left=.. back=.. right=..`를 확인한다.
4. 구동계 확인: `RESCUE_MODE=CONTROL_TEST` → 전진 / 정지 / 좌회전 / 정지 / 우회전 / 정지 후 DONE.
   각 단계에서 `odom=`와 `lidar_front=`가 함께 찍힌다(명령 이동 vs LiDAR 거리 변화, GPS 없음).
5. `python scripts/verify_baseline.py` 및 `--webots`, `--webots --mode CONTROL_TEST`로 검증한다.

## 2. 최종 사양 (config.py)

| 항목 | 값 | 태그 |
|---|---|---|
| 바퀴 반지름 / 바퀴 간격 | 0.033 m / 0.160 m | [OFFICIAL] |
| 모터 최대 속도 | 6.67 rad/s (→ 0.22 m/s, 제자리 2.75 rad/s) | [OFFICIAL]/[DERIVED] |
| notebook 로봇 반경 | 0.105 m (`ROBOT_RADIUS_NOTEBOOK`, 참고용) | [OFFICIAL] |
| 안전 원 `ROBOT_RADIUS` | **0.111 m** (PROTO 외접 ≈0.1103 m 올림) | [DERIVED] |
| inflation | 0.111 + 0.05 = **0.161 m** (3.22셀) | [INITIAL TUNING] |
| 운용 속도 | v ≤ 0.15 m/s, ω ≤ 1.5 rad/s, **v·ω는 바퀴 한계로 함께 제한** | [INITIAL TUNING] |
| LiDAR | `LDS-01`, 360 rays, minRange 0.12 m, maxRange 3.5 m, 로봇 frame (−0.03, 0), 높이 0.173 m | [OFFICIAL]/[DERIVED] |
| LiDAR 각도 | `angle_i = π − i·FOV/N` → 180 전방, 90 왼쪽, 0 후방, 270 오른쪽 | [OFFICIAL] 라벨과 일치, sub-degree는 UNCONFIRMED |
| Camera | `camera`, 640×480, FOV 1.0472 rad, 로봇 frame (0.02, 0, 0.073) | [OFFICIAL] |
| Gyro 변환 | 1.0 (lookupTable 없음 → rad/s) | [OFFICIAL] |
| 필수 device | 바퀴 모터 2 + **엔코더 2 + LDS-01** (없으면 `DeviceError`, 로봇 정지) | [ORGANIZER] |
| 사용 안 함 | **Compass, GPS** (config에 없음, 설정하면 `DeviceError`), Supervisor pose | [ORGANIZER] |
| 주기 | mapping 0.128 s, detection(camera enable) 0.128 s, odometry/safety 매 step — 모두 **초 단위** | [INITIAL TUNING] |
| Grid | 400 × 400 @ 0.05 m = **20 m × 20 m** (공식 예제 월드 ≈13 m, 시작점 중심) | [INITIAL TUNING]/[DAY-OF] |
| 시작 pose | `config.START_POSE`, 월드 `customData {"start_pose": [x, y, θ]}`가 있으면 그 값 | 형식 [DAY-OF] |

## 3. zip 전환본 대비 수정 내역 (연구와 충돌한 부분)

| # | zip 전환본 | 이번 반영 | 근거 |
|---|---|---|---|
| 1 | `ROBOT_RADIUS = 0.14` (임의 보수값) | **0.111** + margin 0.05 = inflation 0.161, notebook 0.105는 별도 상수 | 10 §11 Q7, 09 §3, 12 #19 |
| 2 | `EMERGENCY_STOP_DISTANCE = 0.21` (LiDAR 기준, 전방 ±30°) | `SafetyMonitor`: 차축 중심 기준 STOP 반원 `0.111 + 0.05 + v·0.2 s`, min_points 2 + 근접 단일점 veto, **minRange 사각**(가까이 있던 ray가 inf가 되면 STOP), 제자리 회전 여유 0.13 m, **후진 금지**, scan 없음/stale → 정지. STOP 반원 경계 전체가 LiDAR minRange 밖에 있음을 테스트로 확인 | 05 §3·§6, 10 §11.1, 11 S2 Prompt A |
| 3 | `set_velocity`가 v, ω를 각각 clamp 후 바퀴별 clamp (곡률 왜곡) | v, ω clamp 후 바퀴 한계 초과 시 **둘을 같은 비율로 축소** (`|(v±ωL/2)/r| ≤ 6.67`) | 05 보충, 10 §11.1 |
| 4 | `compass` 활성화, 월드에 GPS 추가, `gps_debug` 출력/검증 | Compass·GPS **제거**: config에 없음, `FORBIDDEN_DEVICE_NAMES`로 차단, 월드 GPS 노드 삭제, verify는 GPS 대신 **엔코더 odometry**로 정지 유지·전진/좌회전 방향 판정 | [ORGANIZER], 09 §11, 11 §2 "GPS drift 지표 삭제", 12 #24·#25 |
| 5 | 필수 device = 모터만 | 엔코더·LiDAR도 필수(fail closed). 엔코더 값 invalid → 정지, LiDAR invalid/stale → 정지 | 10 §11.1, 11 S0 수용 기준 |
| 6 | 월드 `basicTimeStep 16` | **64 ms** (공식 주행 월드) | 09 §6, 10 §11 Q5 (16 ms 전제 폐기) |
| 7 | `MAP_UPDATE_PERIOD_STEPS = 5` (step 수) | `MAP_UPDATE_PERIOD = 0.128 s`, `DETECTION_PERIOD = 0.128 s`, camera는 timestep 배수 주기로 enable, frame은 detection 주기에만 읽음. `scheduling.py`(Periodic, StepTimer) | 10 §2.2, 11 Prompt 0 |
| 8 | mapping: inf ray를 maxRange까지 FREE로 비움, 관측된 OCCUPIED는 영구 | inf/NaN/범위 밖 ray **skip**, minRange 안 셀은 비우지 않음, 스캔당 셀 1회·hit 우선, 이후 free ray가 OCCUPIED를 비움(영구 ghost 금지, M0 1줄 clearing) | 09 §4-4, 10 §11.1, 03 §A.4, 11 §3.1 |
| 9 | Grid 320 × 320 (16 m) | **400 × 400 (20 m)** | 03 §A.4, 09 §11 |
| 10 | `customData`를 `json.loads`로 바로 읽음 (잘못된 값이면 crash), Webots 호출이 main에 | `devices.parse_start_pose()` — 잘못된 값은 경고 후 `config.START_POSE` 사용, 부팅 로그에 출처 표시 | AGENTS 규칙 5, 12 공통 |
| 11 | RETURN_HOME: `allow_unknown=True` 한 번 | known-only A* 먼저 → 실패 시 unknown 허용. A* 전 정지 명령을 step으로 먼저 전달 | 08 §2.3, 10 §2.2 |
| 12 | 시작 로그: available device, LiDAR fov/maxRange | + timestep, synchronization, LiDAR minRange·mount, camera 해상도·FOV·주기, gyro 유무, compass/GPS 미사용, `[status]`에 step 시간 median/p95/max·map 누락 횟수 | 11 S0 수용 기준 |
| 13 | 공식 repo의 `tb3_*` controller 8개, 공식 월드 7개(+그 파생 `rescue_breakroom.wbt`), `*Apple.proto`, `models/YOLO` 포함 | **포함하지 않음**. 공식 자료는 read-only 참고, 우리 repo로 복사하지 않는다(공식 notebook 무단 복제·재사용 금지 고지) | 09 §0 저작권, 11 §2.1 |
| 14 | `archive/practice_project/` (e-puck 월드) | 포함하지 않음 — 저장소에서 이미 제거된 연습 자료 | yeondae-0524/TechWeek26#2 (practice archive 제거) |
| 15 | docs/research (재검증 이전 버전) | 저장소의 **재검증본(2026-09-30)** 유지 | research 00 |

채택한 zip 구조: 파일 구성(`rescue_robot/*`), LiDAR mount offset을 mapping ray 시작점에 반영,
`GYRO_RAW_TO_RAD_S = 1.0`, `LDS-01` 이름, TurtleBot3 검증 월드(장애물 0.3 m — 0.1 m 장애물은
높이 0.173 m의 LDS-01에 보이지 않음), `customData` 시작 pose, `tests/test_turtlebot.py`,
verify의 Webots 경로 후보·taskkill 정리.

## 4. 팀 결정이 필요한 것 (구현하지 않음)

- **local map frame**: 연구는 home = local (0, 0, 0), 제공 world pose는 정적 변환에만 쓰는 것을 추천(08 §2.1).
  현재 코드는 AGENTS.md 고정 규격대로 **world frame pose**를 유지한다. 바꾸려면 INTERFACES.md·interfaces.py·tests를
  같이 바꾸고 사람 확인이 필요하다.
- D1 `navigation.py`, D2 OpenCV 편입, D3 gyro fusion, D6 scan matching, D8 Supervisor 개발 평가 (research 11 §0).
- 대회 world의 timestep/synchronization, 시작 pose 형식, 아레나 크기, 미션 시간, target 외형 — [DAY-OF].

## 5. 아직 구현되지 않은 기능 (TODO, stub 그대로)

- gyro heading fusion(S1) — gyro는 읽기만 하고 **fusion하지 않는다**
- waypoint/path 추종(RPP-lite, S2) — `follow_waypoint`는 정지 stub
- SLOWDOWN 영역, 1 s 전방 투영, ProgressMonitor, recovery ladder
- log-odds mapping, reset_region(S4), frontier 선택(S7), detection(S8, `detect_target`은 항상 found=False)
- 시간 예산 복귀(S6) — 아직 고정 `MISSION_TIME_LIMIT`
- CONTROL_TEST는 구동계 점검이며 자율주행 미션이 아니다.

LiDAR 평면(≈0.173 m) 아래 물체와 LiDAR 뒤쪽 minRange 사각은 이 센서만으로 감지할 수 없다.
Safety monitor는 전방 사각 진입(가까이 있던 점 → inf)만 막으며, 저위 장애물 충돌 감지(collision map)는 TODO다.

## 6. 사양 근거

- https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto
- https://github.com/cyberbotics/webots/blob/R2025a/projects/devices/robotis/protos/RobotisLds01.proto
- 공식 TECH WEEK repo `kyu-rae-kim/PNU-TECHWEEK-260930` @ `383de18` — 사실·수치만 인용 (research 09 §0)

## 7. 검증 기록

- `python3.10 scripts/verify_baseline.py` (Python 3.10.20, Linux): ALL CHECKS PASSED — 문법, Webots API 분리,
  GPS/Compass/Supervisor 정적 검사, unit test 80개.
- Webots 실행 검증(`--webots`)은 **이 환경에 Webots가 없어 미실행**. 대신 가짜 `controller` 모듈(TB3 기구학 +
  2 m arena ray-cast)로 `main.py`를 돌려 STOP(정지 유지), CONTROL_TEST(odom +0.110 m 전진 = LiDAR 전방 거리 −0.110 m,
  좌회전 +66°, 우회전 후 0°, DONE), LiDAR 없음 → FATAL, RETURN_HOME A*(known-only 실패 → unknown 허용)를 확인했다.
  실제 Webots R2025a에서 `--webots`, `--webots --mode CONTROL_TEST` 재확인이 필요하다.
