# 11. Implementation Roadmap

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> **문서 검토 단계: 아래는 미래 구현 계획이다. 이번 작업에서는 S0–S11을 시작하지 않는다. 팀 confirmation 후 별도 구현한다.**
> 기준 문서: [10_FINAL_ARCHITECTURE.md](10_FINAL_ARCHITECTURE.md). Step 번호(S0–S11)·우선순위(P0–P3)는 10과 동일해야 한다.
> **2026-09-30 공식 TECH WEEK repo(@`383de18`) 기준으로 재정렬.** 공식 환경 사실은 [09](09_WEBOTS_REFERENCES.md).
> 모든 Step 공통: AGENTS.md 준수 — 기존 테스트 유지, `BASELINE_MODE="STOP"` 기본 유지(새 동작은 `RESCUE_MODE` opt-in), Webots API는 devices.py/main.py에만, 수정 후 `python scripts/verify_baseline.py` (+ controller 변경 시 `--webots`), Git 규칙(docs/DEVELOPMENT.md §2.1).
> 태그: [ORGANIZER] [OFFICIAL] [DERIVED] [MEASURED] [REFERENCE] [INITIAL TUNING] [DAY-OF].

---

## 0. 구현 전에 팀이 결정할 것

| # | 결정 | 권장 | 근거 |
|---|---|---|---|
| D1 | `navigation.py` 신규 파일 | 만든다 | 10 §2.1 |
| D2 | Detection 의존성 | **OpenCV classical CV baseline** (NumPy-only 폴백) | 공식 교육·예제가 `opencv-python==4.8.0.74` 사용 [OFFICIAL]; 대회 라이브러리 제한은 최종 확인 대기 [DAY-OF] |
| D3 | IMU | **TEAM CHOICE: Gyro ENABLED**, encoder-only 폴백 | IMU 선택 [ORGANIZER]; 10 §11 Q1 |
| D4 | Local planner | RPP-lite + SafetyMonitor (DWA 아님) | notebook look-ahead와 같은 계열 [OFFICIAL 교육] |
| D5 | Camera coverage / view frontier 시점 | MVP 이후 Competitive (단, 카메라가 유일한 target 센서임을 인지) | 10 §11 Q4 |
| D6 | Scan matching | 조건부(10 §11 Q9 테스트) — 단 '기술 구현' 점수를 위해 P2 도입할지 팀 판단 | 공식 계획안이 활용 가능 기술로 명시 [OFFICIAL] |
| D7 | 검증 환경 | 현재 `verify_baseline --webots`는 **e-puck practice world** 기준 → TB3 config로 바꾸면 깨진다. 선택: (a) `ROBOT_PROFILE` 설정(tb3 기본 / epuck_practice) 또는 (b) 우리가 직접 만든 **최소 TB3 테스트 world** 추가(Webots 공식 TB3 PROTO 사용, world 추가는 사람 확인) | S0 |
| D8 | Supervisor 개발용 평가 | 쓰려면 분리된 테스트 모드/월드에서만, 허용 범위 확인 후 팀 합의 | 대회 controller 입력 아님 |

## 1. Feature Priority

| ID | 기능 | P | 난이도 | 의존 | 실패 위험 | Reference |
|---|---|---|---|---|---|---|
| F0 | **공식 환경 audit + config 이행 + 계측**: device 이름(`LDS-01`), TB3 기하(0.033/0.160/교육 반경 0.105, 안전 반경 올림 0.111), 모터 6.67 rad/s, gyro 단위 1.0, LiDAR offset (−0.03,0), Compass/GPS 비활성, timestep 기반(초 단위) 주기, grid 크기, OS별 runtime.ini, 로그(timestep·synchronization·LiDAR 스펙·실제 갱신 간격·getImage 비용·step 시간) | P0 | LOW~MED | – | e-puck 값 잔존 | 09 [OFFICIAL] |
| F1 | Encoder odometry 검증 + **TEAM CHOICE gyro heading fusion**(정지 bias, 폴백) | P0 | LOW | F0 | gyro 부재 시 크래시 → 폴백 필수 | Borenstein & Feng 1996 📄 |
| F2 | PathFollower (RPP-lite = look-ahead + rotate + 감속 + 충돌 예측) | P0 | MED | F0 | 튜닝(오버슛) | notebook [OFFICIAL 교육], Nav2 RPP ✅ |
| F3 | SafetyMonitor (TB3 footprint, minRange 0.12 m 사각, min_points, 후방, 투영) | P0 | LOW~MED | F0 | 과민 정지 / 근접 inf 오인 | Nav2 Collision Monitor ✅ |
| F4 | planning 유틸 (distance_field, 8-연결 옵션, relax_goal, smooth_path, path_is_valid) + 큰 맵 성능 측정 | P0 | MED | – | 기존 테스트 파손 | Nav2/hector/FAR/SemExp ✅ |
| F5 | log-odds mapping + reset_region + mount offset | P0 | MED | F0 | 벽 침식 | OctoMap/Hector ✅ |
| F6 | Navigator + ProgressMonitor + 기본 recovery (R0/R2/R3/R4, **R4 = collision map + blacklist**) | P0 | MED~HIGH | F2,F3,F4 | 상태 꼬임 | Nav2 ✅, move_base ✅, SemExp ✅ |
| F7 | RETURN_HOME: 시간 예산 + 경로 폴백 MVP(known-only → breadcrumb → unknown 허용; 전체는 08 §2.3) | P0 | MED | F4,F6 | ETA 과소평가 | GBPlanner ✅, m-explore-ros2 ✅ |
| F8 | Exploration: frontier 선택(G1, exp utility, hysteresis) + blacklist + 종료 | P0 | MED | F4,F6 | thrashing | m-explore ✅, rrt_exploration ✅, VLFM ✅ |
| F9 | Target: **OpenCV classical detector** + TargetTracker(M-of-N, bearing, LiDAR/카메라 거리, dedup) + 2단계 접근 | P0 | MED | F6 | 오탐·거리 오차 | 07, 공식 예제 [OFFICIAL] |
| F10 | Mission 통합(FSM) + Initial Active Scan | P0/P1 | MED | F6–F9 | 전이 버그 | notebook FSM [OFFICIAL 교육] |
| F11 | Camera coverage grid + view frontier | P1 | MED | F5,F8 | 계산량 | hector inner exploration ✅ |
| F12 | Full recovery ladder: R1 CLEAR, R5, oscillation | P1 | MED | F5,F6 | 과도한 리셋 | Nav2 ✅, move_base ✅ |
| F13 | danger 비용 + 8-연결 기본화 + LOS smoothing 튜닝 (+ 큰 맵용 0.10 m planning grid 옵션) | P1 | LOW~MED | F4 | 우회 증가 | Hector ✅, Nav2 inflation ✅ |
| F14 | 동적 장애물 WAIT 정책(새 장애물 판정, 대기 상한) | P1 | LOW~MED | F5,F6 | 무한 대기 | Nav2 BT Wait ✅, FAR ✅ |
| F15 | 당일 오도메트리 보정 절차 + 시간 기반 스케줄러 튜닝 | P1 | LOW | F0 | – | webots_ros2 drive_calibrator ✅ |
| F16 | 심사용 덤프(지도+경로+target PPM, target 목록) | P1 | LOW | F5,F9 | – | – |
| F17 | ray-cast IG + 카메라 IG를 utility에 | P2 | MED | F8,F11 | 계산량 | FUEL ✅ |
| F18 | 다가오는 장애물(사람) TTC 감지 | P2 | MED | F3 | 오탐 | Nav2 APPROACH ✅ |
| F19 | 조건부 scan matching (numpy CSM) | P2 | HIGH | F1,F5 | 잘못된 보정 | Olson 📄, Karto ✅ |
| F20 | frontier 도착 시 look-around | P2 | LOW | F10 | 시간 소모 | TARE/FUEL ✅ |
| F21 | arc-sampler local planner | P3 | MED | F3 | 진동 | CMU localPlanner ✅ |
| F22 | 스캔 클러스터 추적 | P3 | HIGH | F3 | 연관 오류 | obstacle_detector ✅ |

## 2. 단계별 계획 (S0–S11) · 수용 기준 · 테스트

GPS drift 지표는 **삭제**. 대체 지표: loop closure 오차(스캔 정합), 명령 이동 대비 오차(LiDAR 벽 거리 변화), 지도 일관성(벽 이중화), (팀 합의 시) 분리된 Supervisor 평가자.

| Step | 기능 | 산출 파일 | 수용 기준 (Accept) | 테스트 |
|---|---|---|---|---|
| S0 | F0 | config.py, devices.py, main.py, runtime.ini, (D7에 따라) scripts/verify·테스트 world | 시작 로그에 timestep·synchronization·LDS-01 스펙(360/0.12/3.5)·camera(640×480, 1.0472)·gyro 존재; 허용된 개발 센서 테스트에서 대표 인덱스 180/0/90/270의 전후좌우 및 beam-center 보정 검증; config에 e-puck 값 없음(practice profile 제외); step 시간 median/p95/max·센서 age 로그; encoder/LiDAR 없음/invalid→STOP; optional gyro 없음→encoder-only | `verify_baseline` + TB3 월드 로그 확인 |
| S1 | F1 | localization.py, config.py, tests/test_localization_gyro.py | 정지 1 s bias 추정; slip 시뮬에서 gyro heading이 encoder-only보다 정확; **gyro 없음 → encoder-only로 동작** | 합성 unit test; Webots 360° 회전 후 벽 방향 재관측 |
| S2 | F2, F3 | control.py, config.py, main.py(`NAV_TEST`), tests/test_path_follower.py, tests/test_safety.py | 고정 사각 경로 충돌 0 완주; 끝점 오차는 **시작점 재관측 스캔 정합** 또는 명령 이동 대비로 평가 (GPS 사용 안 함); 전방 0.12 m 이내 inf를 free로 취급하지 않음 | 직선/코너/전방 장애물/후진 차단/min_points/근접 inf |
| S3 | F4 | planning.py, tests/test_planning_utils.py | 기존 test_planning 통과; 8-연결 ≤ 4-연결; 320×320 이상 맵 계획 시간 기록 | 04 §5 |
| S4 | F5 | mapping.py, config.py, tests/test_logodds.py | 인터페이스 불변(-1/0/1); L_MAX=3.5에서 miss −0.4, FREE<−0.2이면 10 miss; 새 스캔당 1회; insert ≤15 ms는 INITIAL TUNING 목표(기존 binary 벤치와 구분); mount offset 반영 | 03 §C |
| S5 | F6 | navigation.py, control.py(ProgressMonitor), tests/test_navigator.py | 가짜 로봇/맵에서 REACHED; 막힌 경로→ladder→FAILED; 회전 중 진전 판정 보류; 전진 명령과 반복 scan 기반 운동 불일치→충돌 의심 기록; encoder 공회전만으로 성공 판정 금지 | 06 §6 |
| S6 | F7 | planning.py(eta), main.py, tests/test_return_home.py | 경계 시간에서 트리거; 폴백 순서대로 경로 확보; home=(0,0,0) local; 제공 world 시작 pose는 정적 변환 | 08 §3 |
| S7 | F8 | planning.py, tests/test_frontier_selection.py | 02 §6 표 통과; TB3 월드 `EXPLORE` 모드에서 도달가능 관측 영역 확장 및 coverage 기록(동적 occupancy의 FREE 셀 수는 단조 증가를 보장하지 않음) | 02 §6 |
| S8 | F9 | detection.py, tests/test_target_tracker.py | 07 §11 통과; 테스트 target 확인·접근·정지 (target 외형은 당일 교체) | 07 §11 |
| S9 | F10 | main.py | 전체 미션 FSM, 충돌 0 | Webots 시나리오 |
| S10 | F11–F16 | mapping/planning/navigation/main | 사람(Pedestrian) 시나리오·좁은 통로·끼임·저위 장애물에서 완주 | 12 Case별 |
| S11 | F17–F20 | – | 측정으로 효과 확인된 것만 유지 | A/B 로그 |

### 2.1 Webots 시나리오 테스트
- 모드: `RESCUE_MODE=NAV_TEST` / `EXPLORE` / `MISSION` (기본 STOP).
- 월드: 공식 repo 월드는 **read-only 참고**(우리 repo로 복사하지 않음, 공식 notebook 재사용 금지 고지). 우리 테스트 월드가 필요하면 Webots 공식 PROTO로 직접 작성(D7, 사람 확인).
- 지표: 충돌 횟수(가속도 spike / 무이동 감지), loop closure 오차, 명령 이동 대비 오차, 지도 일관성, 탐색률(FREE 셀/시간), target 확인·도착 시간, 복귀 성공, step 시간 max.
- 동적 장애물: 공식 apartment 월드처럼 `Pedestrian`(speed 0.2 m/s 궤적) 사용 예시 참고.

## 3. 버전별 묶음

| 버전 | Step | 목표 |
|---|---|---|
| **M0 Walking Skeleton** | S0 + 각 Step의 최소판 (§3.1) | 전체 루프가 한 번 끝까지 돈다 |
| MVP | S0–S9 | 완주 |
| Stable | + F12, F13, F14, F15 | 실패 상황 완주 |
| Competitive | + F11, F16, Initial Active Scan 튜닝 | target 누락 감소, 설명력 |
| Stretch | S11 (F17–F20) | 차별화 |

### 3.1 M0 Walking Skeleton

각 Step의 가장 단순한 버전으로 end-to-end를 먼저 연결하고 MVP에서 교체한다. **S0(공식 환경 이행)는 M0에서도 생략 불가.**

| 영역 | M0 | MVP에서 교체 | 올리는 기준 |
|---|---|---|---|
| Config | **TB3 공식 값으로 이행 (S0)** | – | 필수 |
| Localization | encoder odom | + gyro heading (team choice) | bias·slip·fallback 검증 후 |
| Mapping | binary grid, `_mark_free`가 OCCUPIED도 비움(1줄), mount offset | log-odds | 사람 시나리오에서 깜빡임/침식 |
| Frontier | 거리장 기준 가장 가까운 도달가능 frontier + 단순 blacklist | G1 utility + hysteresis | thrashing 관측 |
| Planner | 기존 4-연결 A* + relax_goal | 8-연결 + smoothing | 지그재그 |
| Follower | notebook식 look-ahead(v 상수) + 큰 각도면 제자리 회전 | RPP-lite 감속 규칙 | 코너 오버슛 |
| Safety | 전방 STOP(TB3 반경, minRange 사각 고려) + 후진 금지 | SafetyMonitor 전체 | 즉시 |
| Navigator | 계획→추종→타임아웃 시 goal 실패 | ProgressMonitor + R0/R2/R3/R4 | 끼임 1회 |
| Detection | OpenCV 단일 프레임(공식 예제와 같은 계열) → 접근 | M-of-N + world 위치 + dedup | 오탐 접근 |
| Return Home | 고정 시간 여유 + known-only A* | ETA 시간 예산 + 폴백 | 곧바로 |
| Mission | FSM + `REQUIRED_TARGETS` | + Initial Active Scan | – |

팀 리뷰 의견 중 유지한 판단: 영구 ghost 금지(1줄 clearing), MVP에 시간 예산 복귀 유지, M0부터 거리장 기준 frontier.

## 4. 병렬 작업 분배 (feature branch)
- feat/integration: **S0 먼저 (다른 branch의 전제)**, 이후 S5, S6, S9
- feat/localization: S1 (→ F19)
- feat/control: S2, ProgressMonitor
- feat/planning: S3, S7, eta
- feat/mapping: S4, F11
- feat/detection: S8

## 5. 의존성 검토
- 현재 baseline: 표준 라이브러리 + NumPy.
- **OpenCV**: 공식 교육 자료가 `opencv-python==4.8.0.74`(+`numpy==1.23.5`, `scikit-image==0.19.3`)를 설치·사용 [OFFICIAL]. baseline 편입을 권장하되 **"official materials support it; final rule confirmation pending"** — 팀 합의(D2) + 운영진 라이브러리 제한 확인 후. 이전 pass benchmark 기록은 cv2 5.0.0 / numpy 2.2.6로 **버전이 다름** → 공식 고정 버전으로 맞출지 결정 [DAY-OF].
- scipy: 공식 notebook look-ahead 예제가 `cKDTree`를 쓰지만, 우리 follower는 필요 없음(선형 탐색으로 충분).
- ultralytics/PyTorch(YOLO): baseline 아님 (13).
- matplotlib: 불필요 (PPM/PGM 덤프).

## 6. Claude/Codex 작업 prompt 초안

### Prompt 0 — S0 공식 환경 이행 (가장 먼저)
```text
AGENTS.md, docs/research/09_WEBOTS_REFERENCES.md(§3~§6, §11), 10_FINAL_ARCHITECTURE.md를 먼저 읽어라.
config.py/devices.py/main.py를 공식 TurtleBot3Burger 환경에 맞춰라:
- DEVICE_NAMES: lidar="LDS-01", motors/encoders/camera/gyro/accelerometer 이름 확인, compass·gps는 competition에서 비활성(None).
- WHEEL_RADIUS 0.033, AXLE_LENGTH 0.160, notebook radius 0.105를 기록하되 안전 ROBOT_RADIUS는 외접 올림 0.111(또는 검증된 polygon), margin 0.05, MAX_WHEEL_SPEED 6.67, GYRO_RAW_TO_RAD_S 1.0, LIDAR_MOUNT_OFFSET (-0.03, 0.0).
- LIDAR_FIRST_ANGLE π / DIRECTION -1은 대표 index 시작값; beam-center/endpoint 검증 후 확정, 운용 속도·안전거리는 [INITIAL TUNING]으로 주석.
- 주기는 step 수가 아니라 초 단위 설정으로(timestep은 robot.getBasicTimeStep()).
- 시작 로그: timestep, synchronization, LiDAR resolution/minRange/maxRange, camera 해상도/FOV, gyro 유무; encoder/LiDAR required fail-closed, local home=(0,0,0) 좌표 변환.
- 64 ms world 기준 mapping/detection 128 ms 시작; 계획 이벤트 + 1 s 상태 확인; bounded work와 모터 명령 step 전달 보장(10 §2.2).
- 기존 e-puck 값은 practice profile로만 남길지 docs/research/11 D7 결정에 따른다(결정 전이면 사람에게 묻기).
- world 파일 추가/수정은 사람 확인 후. 기본 STOP 유지. verify_baseline 결과를 그대로 보고.
```

### Prompt A — S2 PathFollower + SafetyMonitor
```text
AGENTS.md, docs/research/05_LOCAL_PLANNING_SAFETY.md, 10_FINAL_ARCHITECTURE.md를 먼저 읽어라.
control.py에 Webots 없이 테스트 가능한 PathFollower(RPP-lite)와 SafetyMonitor를 추가하라.
- PathFollower.set_path(waypoints[(x,y)]), compute(pose, scan_points_robot_frame) -> (v, w, status in {RUNNING, REACHED, BLOCKED})
  look-ahead 곡률 κ=2y/(x²+y²), ω=vκ, 큰 각도면 제자리 회전, 곡률/근접/접근 감속, 1 s 전방 투영 충돌 검사.
- SafetyMonitor.filter(v, w, scan_points) -> (v, w, event): TB3 footprint(반경 0.105, 외접 0.111(올림) + margin, inflation 0.161) 기준 STOP/SLOW, min_points,
  LiDAR minRange(0.12 m) 이내는 inf로 나오므로 전방 inf를 free로 취급하지 않기, 후진 시 후방 검사.
- 수치는 config.py에 [INITIAL TUNING]으로. interfaces.py 변경 금지. RESCUE_MODE=NAV_TEST에서만 동작. 기본 STOP 불변.
- tests/test_path_follower.py, tests/test_safety.py 추가 (05 §6). verify_baseline 결과 보고. GPS 사용 금지.
```

### Prompt B — S3+S7 Planning utilities & frontier selection
```text
AGENTS.md, docs/research/02_EXPLORATION.md, 04_GLOBAL_PLANNING.md를 읽어라.
planning.py에 distance_field(flat BFS), astar(connectivity=4 기본 유지, 8 옵션·octile·corner-cut 금지), relax_goal, smooth_path(LOS), path_is_valid,
estimate_information_gain(G1), score_frontier(u = G·exp(−λ·L_eff)), select_frontier(hysteresis, 상위 K), FrontierBlacklist(반경, TTL, second chance)를 추가하라.
인플레이션 반경은 config(ROBOT_RADIUS + SAFETY_MARGIN)에서 가져온다. 320×320, 480×480 grid에서 실행 시간을 측정해 보고하라.
기존 tests는 수정하지 말고 통과. 02 §6, 04 §5 테스트 추가. path/grid 규격 변경 금지.
```

### Prompt C — S4 Log-odds mapping
```text
AGENTS.md와 docs/research/03_MAPPING_LOCALIZATION.md를 읽어라.
mapping.OccupancyGrid 내부를 log-odds(hit +0.85, miss −0.4, clamp [−2, 3.5], export +0.4/−0.2, 스캔당 셀 1회, hit 우선)로 바꾸되
외부 grid[row][col]은 UNKNOWN/FREE/OCCUPIED 유지. reset_region(center, radius) 추가. LIDAR_MOUNT_OFFSET(−0.03, 0) 반영 확인.
LDS-01 0.12–3.5 m: 기본 inf/NaN/범위 밖은 skip(근접 occlusion과 장거리 no-return 구분 불가). 유한 ray만 sensor origin에서 minRange 이후 free 처리, endpoint occupied. 10 miss 검증. timestamp/갱신 cadence로 중복 scan 누적 금지. tests/test_grid.py 통과 + tests/test_logodds.py 추가. insert_scan 시간 보고.
```

### Prompt D — S8 Target pipeline
```text
AGENTS.md, docs/research/07_TARGET_SEARCH.md를 참고하라. (D2가 OpenCV로 결정된 경우)
detection.py에 OpenCV classical 검출기(blur → HSV/LAB inRange → morphology open → 최대 contour → cx, area; 임계는 config)와
TargetTracker(M-of-N 3/5, bearing = atan((W/2 − cx)/f), f = (W/2)/tan(FOV/2) — W/FOV는 camera에서 읽기,
거리 = 센서 mount/시선 대응 검증한 LiDAR 또는 조건이 맞는 카메라 ground-plane/알려진 크기(07 §5); 거리 불명은 tentative. world 위치, dedup, visited, memory)를 추가하라.
target 색·크기는 공식 예제(사과/공)에서 추정하지 말고 config 파라미터로 둔다. interfaces.py 변경 금지. tests/test_target_tracker.py 추가.
```


## 7. S0 이후 공통 gate (계획만)

10 §2.2·§11.1이 수치·주기·프레임 계약의 기준이다. 센서 없이 동작하는 mock 성공과 실제 runtime 성공을 구분한다. 인플레이션 0.161 m를 보수적으로 rasterize하고 wheel v/ω 결합 한계를 검증한다. 03/04의 PC benchmark를 공식 world 성능으로 보고하지 않는다. old 160×160/3회 min 및 대형 grid/합성 CV 원본 provenance는 09 §8에 있다.

Scan matching 도입 gate: loop 시작/종료 스캔 정합, 반복 장소 scan-map residual, wall doubling, home return consistency. 0.10 m/3°는 INITIAL TUNING이고 명령 이동량은 정답이 아니다. 정합 confidence·동적 장애물·대칭 구간을 확인한 반복 A/B에서 개선될 때만 S11 도입; encoder-only/gyro 선택 모두 같은 평가를 적용한다.

팀 결정 D1–D8은 추천안이지 승인 완료가 아니다. Encoder mandatory, 2D LiDAR mandatory, IMU optional, Compass/GPS 미사용은 이미 organizer-confirmed이므로 재질문하지 않는다. 남은 운영진 질문은 target 외형/크기/개수/도착 조건, 미션 시간·시계, 최종 world/sensor 설정, dependency 제한, 분리된 Supervisor 개발 평가 범위이다.
