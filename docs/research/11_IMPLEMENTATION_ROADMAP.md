# 11. Implementation Roadmap

> 기준 문서: [10_FINAL_ARCHITECTURE.md](10_FINAL_ARCHITECTURE.md). Step 번호(S0–S11)·우선순위(P0–P3)는 10과 동일해야 한다.
> 모든 Step 공통: AGENTS.md 준수 — 기존 테스트 유지, `BASELINE_MODE="STOP"` 기본 유지(새 동작은 `RESCUE_MODE` opt-in), Webots API는 devices.py/main.py에만, 새 dependency 금지(OpenCV 포함, 사람 확인 전), 수정 후 `python scripts/verify_baseline.py` (+ controller 변경 시 `--webots`).

---

## 1. Feature Priority

| ID | 기능 | P | 난이도 | 의존 | 실패 위험 | Reference |
|---|---|---|---|---|---|---|
| F0 | 계측: 스텝 시간·synchronization·센서 스펙 로그, 주기적 지도 덤프 | P0 | LOW | – | 낮음 | Webots robot.md ✅ |
| F1 | gyro heading fusion + 정지 bias | P0 | LOW | – | 낮음 | Borenstein & Feng 1996 📄 |
| F2 | PathFollower (RPP-lite) | P0 | MED | F1 | 튜닝(오버슛) | Nav2 RPP ✅, CMU pathFollower ✅ |
| F3 | SafetyMonitor 강화 (zones, min_points, 후방, 1 s 투영) | P0 | LOW~MED | – | 과민 정지 | Nav2 Collision Monitor ✅ |
| F4 | planning 유틸 (distance_field, 8-연결, relax_goal, smooth_path, path_is_valid) | P0 | MED | – | 기존 테스트 파손 | Nav2/hector/FAR/SemExp ✅ |
| F5 | log-odds mapping + reset_region | P0 | MED | – | 벽 침식 | OctoMap/Hector ✅ |
| F6 | Navigator + ProgressMonitor + 기본 recovery (R0/R2/R3/R4) | P0 | MED~HIGH | F2,F3,F4 | 상태 꼬임 | Nav2 BT/behaviors ✅, move_base ✅ |
| F7 | RETURN_HOME: 시간 예산 + 경로 폴백 MVP(known-only → breadcrumb → unknown 허용; 전체는 08 §2.3) | P0 | MED | F4,F6 | ETA 과소평가 | GBPlanner ✅, m-explore-ros2 ✅ |
| F8 | Exploration: frontier 선택(G1, exp utility, hysteresis) + blacklist + 종료 | P0 | MED | F4,F6 | thrashing | m-explore ✅, rrt_exploration ✅, VLFM ✅ |
| F9 | Target: NumPy HSV + TargetTracker(M-of-N, bearing+LiDAR, dedup) + 2단계 접근 | P0 | MED | F6 | 오탐·위치 오차 | 07 |
| F10 | Mission 통합 + Initial Active Scan | P0/P1 | MED | F6–F9 | 전이 버그 | m-explore-ros2 ✅ |
| F11 | Camera coverage grid + view frontier | P1 | MED | F5,F8 | 계산량 | hector inner exploration ✅ |
| F12 | Full recovery ladder: R1 CLEAR, collision_map, R5, oscillation | P1 | MED | F5,F6 | 과도한 리셋 | Nav2 ✅, SemExp ✅, move_base ✅ |
| F13 | danger 비용 + 8-연결 기본화 + LOS smoothing 튜닝 | P1 | LOW | F4 | 우회 증가 | Hector ✅, Nav2 inflation ✅ |
| F14 | 동적 장애물 WAIT 정책(새 장애물 판정) | P1 | LOW~MED | F5,F6 | 무한 대기 | Nav2 BT Wait ✅, FAR ✅ |
| F15 | 당일 오도메트리 보정 절차 + 스텝 예산 스케줄러 | P1 | LOW | F0 | – | webots_ros2 drive_calibrator ✅ |
| F16 | 심사용 덤프(지도+경로+target PPM, target 목록) | P1 | LOW | F5,F9 | – | – |
| F17 | ray-cast IG + 카메라 IG를 utility에 | P2 | MED | F8,F11 | 계산량 | FUEL ✅ |
| F18 | 접근 장애물 TTC 감지 | P2 | MED | F3 | 오탐 | Nav2 APPROACH ✅ |
| F19 | 조건부 numpy CSM | P2 | HIGH | F1,F5 | 잘못된 보정 | Olson 📄, Karto ✅ |
| F20 | frontier 도착 시 look-around | P2 | LOW | F10 | 시간 소모 | TARE/FUEL ✅ |
| F21 | arc-sampler local planner | P3 | MED | F3 | 진동 | CMU localPlanner ✅ |
| F22 | 스캔 클러스터 추적 | P3 | HIGH | F3 | 연관 오류 | obstacle_detector ✅ |

## 2. 단계별 계획 (S0–S11) · 수용 기준 · 테스트

| Step | 기능 | 산출 파일 | 수용 기준 (Accept) | 테스트 |
|---|---|---|---|---|
| S0 | F0 | devices.py, main.py, config.py | 시작 로그에 synchronization·센서 스펙, 10 s마다 스텝 시간 max/avg 출력, 지도 덤프 주기 설정 | verify `--webots` 로그 확인 |
| S1 | F1 | localization.py, config.py, tests/test_localization_gyro.py | 정지 1 s bias 추정; gyro 사용 시 heading이 slip 시뮬에서 odom-only보다 정확 | 합성 데이터 unit test |
| S2 | F2, F3 | control.py, config.py, main.py(`NAV_TEST` opt-in), tests/test_path_follower.py, tests/test_safety.py | 고정 waypoint 사각 경로를 충돌 없이 완주, 끝점 오차 ≤ 0.05 m(GPS debug로 측정) | 직선/코너/전방 장애물/후진 차단/min_points |
| S3 | F4 | planning.py, tests/test_planning_utils.py | 기존 test_planning 통과, 8-연결 경로 ≤ 4-연결, relax/smooth/valid 테스트 통과 | 04 §5 |
| S4 | F5 | mapping.py, config.py, tests/test_logodds.py | 인터페이스 불변(-1/0/1), 9 miss로 ghost 해제, 스캔당 1회 갱신, insert ≤ 15 ms | 03 §C |
| S5 | F6 | navigation.py(신규), control.py(ProgressMonitor), tests/test_navigator.py | 가짜 로봇/맵 시뮬에서 goal REACHED; 막힌 경로→ladder→FAILED 반환; 회전 중 진전 판정 보류 | 06 §6 |
| S6 | F7 | planning.py(eta), main.py, tests/test_return_home.py | 경계 시간에서 트리거, 폴백 순서대로 경로 확보, 도착 판정 | 08 §3 |
| S7 | F8 | planning.py, tests/test_frontier_selection.py | 02 §6 표 전부 통과; Webots `EXPLORE` 모드에서 practice world FREE 셀 수가 단조 증가 | 02 §6 |
| S8 | F9 | detection.py, tests/test_target_tracker.py | 07 §11 통과; practice world 빨간 target을 확인·접근·정지 | 07 §11 |
| S9 | F10 | main.py | 전체 미션: INITIALIZE→(scan)→EXPLORE⇄APPROACH→RETURN_HOME→DONE, 충돌 0 | Webots 시나리오 |
| S10 | F11–F16 | mapping/planning/navigation/main | 사람 장애물 시나리오·좁은 통로·끼임에서 완주 | 12의 Case별 시나리오 |
| S11 | F17–F20 | – | 측정으로 효과 확인된 것만 유지 | A/B 로그 비교 |

### 2.1 Webots 시나리오 테스트 (practice world 사본에서만, 원본 world 대규모 수정 금지)
- 모드: `RESCUE_MODE=NAV_TEST` / `EXPLORE` / `MISSION` (기본 STOP 유지).
- 지표: 충돌 횟수(가속도 spike 또는 테스트용 supervisor), GPS debug drift, 탐색률(FREE 셀 수/시간), target 확인·도착 시간, 복귀 성공, 스텝 시간 max.
- 동적 장애물 시험: 테스트 world 사본에 supervisor로 움직이는 Solid(심사 컨트롤러와 분리).

## 3. 버전별 묶음

| 버전 | Step | 목표 |
|---|---|---|
| **M0 Walking Skeleton** | S0 + 각 Step의 최소판 (아래 §3.1) | 전체 루프가 한 번 끝까지 돈다 |
| MVP | S0–S9 | 완주 |
| Stable | + S10의 F12, F13, F14, F15 | 실패 상황 완주 |
| Competitive | + F11, F16, Initial Active Scan 튜닝 | target 누락 감소, 설명력 |
| Stretch | S11 | 차별화 |

### 3.1 M0 Walking Skeleton (팀 리뷰 반영, 2026-09-30)

리뷰 지적: "S0–S9 전체를 MVP로 두면 사실상 완제품이다." → **수용.** 단, 기능을 빼는 게 아니라 **각 Step의 가장 단순한 버전으로 먼저 end-to-end를 연결**하고 MVP에서 교체한다 (통합 위험을 가장 먼저 제거).

| 영역 | M0 (먼저 끝까지 연결) | MVP에서 교체 | M0에서 MVP로 올리는 기준 |
|---|---|---|---|
| Localization | encoder odom (현재) | + gyro heading | 30분 작업, 즉시 |
| Mapping | binary grid, `_mark_free`가 OCCUPIED도 비움 (Nav2식 raytrace clearing, 1줄) | log-odds | 사람 시나리오에서 깜빡임/벽 침식이 보이면 |
| Frontier | 거리장 기준 **가장 가까운 도달가능** frontier (Euclidean 아님) + 단순 blacklist | G1 utility + hysteresis | goal 왕복(thrashing)이 관측되면 |
| Planner | 기존 4-연결 A* + relax_goal | 8-연결 + smoothing | follower가 지그재그로 흔들리면 |
| Follower | 회전 후 직진 + heading P제어 (CMU pathFollower 수준) | RPP-lite 감속 규칙 | 코너 오버슛 |
| Safety | 기존 전방 e-stop + 후진 금지 | SafetyMonitor 전체 | 즉시 (충돌 금지 조건) |
| Navigator | 계획→추종→**타임아웃 시 goal 실패** | ProgressMonitor + R0/R2/R3/R4 | 끼임 1회라도 발생 시 |
| Detection | 단일 프레임 HSV → 접근 | M-of-N + world 위치 + dedup | 오탐 접근 발생 시 (M-of-N은 수십 줄이라 조기 도입 권장) |
| Return Home | 고정 시간(MISSION_TIME_LIMIT − 여유) + known-only A* | ETA 시간 예산 + 폴백 | 거리장이 이미 있으므로 곧바로 |
| Mission | INITIALIZE→EXPLORE⇄APPROACH→RETURN_HOME→DONE, `REQUIRED_TARGETS` 파라미터 | + Initial Active Scan | – |

리뷰 의견 중 **그대로 수용하지 않은 부분**과 이유:
- "Log-odds는 완주를 막는 기능이 아니다" → M0에선 수용하지만, 대회에 사람이 확정적으로 있으므로 **영구 ghost(현재 코드)는 M0에서도 금지** (위 1줄 clearing). 영구 ghost는 home 경로를 막을 수 있어 완주에 직접 영향.
- "Time-budgeted Return은 차별화(3순위)" → 복귀 실패 = 미완주이므로 **MVP 유지**. M0는 고정 여유로 대신.
- "Frontier는 단순 최근접" → 수용하되 **거리 기준은 M0부터 거리장**(12 ms). Euclidean 최근접은 벽 너머 goal로 끼임 → recovery 없는 M0에서 더 위험.

## 4. 병렬 작업 분배 (feature branch 예시)
- feat/localization: S1 (→ 이후 F19)
- feat/control: S2, ProgressMonitor
- feat/planning: S3, S7, eta
- feat/mapping: S4, F11
- feat/detection: S8
- feat/integration: S0, S5, S6, S9 (S2–S4 병합 후)

## 5. 의존성 검토
- 표준 라이브러리 + NumPy만으로 전 단계 가능 (HSV 변환·연결요소도 NumPy/순수 Python).
- OpenCV를 baseline에 넣으려면 사람 확인 필요 (연습 코드에서는 사용했음).
- matplotlib 불필요 (PPM/PGM 덤프).

## 6. Claude/Codex 작업 prompt 초안

### Prompt A — S2 PathFollower + SafetyMonitor
```text
AGENTS.md와 docs/research/05_LOCAL_PLANNING_SAFETY.md, 10_FINAL_ARCHITECTURE.md를 먼저 읽어라.
controllers/rescue_robot/control.py에 Webots 없이 테스트 가능한 PathFollower(RPP-lite)와 SafetyMonitor를 추가하라.
- PathFollower.set_path(waypoints[(x,y)]), compute(pose, scan_points_robot_frame) -> (v, w, status in {RUNNING, REACHED, BLOCKED})
  rotate-to-heading(임계 config), pure pursuit 곡률, 곡률/근접/접근 감속, 1 s 전방 투영 충돌 검사.
- SafetyMonitor.filter(v, w, scan_points) -> (v, w, event): STOP/SLOW 원형 영역, min_points, 후진 시 후방 검사.
- 모든 수치는 config.py에 추가(기존 값 변경 금지). interfaces.py 변경 금지.
- main.py에는 RESCUE_MODE=NAV_TEST일 때만 고정 사각 waypoint를 따르게 연결; 기본 STOP 동작 불변.
- tests/test_path_follower.py, tests/test_safety.py 추가 (05 §6 항목).
- 완료 후 python scripts/verify_baseline.py 와 --webots 결과를 그대로 보고. stub은 stub이라 보고.
```

### Prompt B — S3+S7 Planning utilities & frontier selection
```text
AGENTS.md, docs/research/02_EXPLORATION.md, 04_GLOBAL_PLANNING.md를 읽어라.
planning.py에 distance_field(flat BFS), astar(connectivity=4 기본 유지, 8 옵션·octile·corner-cut 금지), relax_goal, smooth_path(LOS), path_is_valid,
estimate_information_gain(G1: 반경 내 UNKNOWN 넓이), score_frontier(u = G·exp(−λ·L_eff)), select_frontier(hysteresis, 상위 K 평가), FrontierBlacklist(반경, TTL, second chance)를 추가하라.
기존 tests/test_planning.py·test_frontier.py는 수정하지 말고 통과시켜라. 02 §6, 04 §5의 테스트를 새 파일로 추가.
path 형식 [(row, col)]과 grid 규격은 변경 금지. verify_baseline 결과를 보고.
```

### Prompt C — S4 Log-odds mapping
```text
AGENTS.md와 docs/research/03_MAPPING_LOCALIZATION.md를 읽어라.
mapping.OccupancyGrid 내부를 log-odds(hit +0.85, miss −0.4, clamp [−2, 3.5], 스캔당 셀 1회, hit 우선)로 바꾸되
외부로 보이는 grid[row][col]은 UNKNOWN/FREE/OCCUPIED 그대로 유지하라. reset_region(center, radius) 추가.
기존 tests/test_grid.py 통과 + tests/test_logodds.py 추가(03 §C). insert_scan 시간을 측정해 보고.
```

### Prompt D — S8 Target pipeline
```text
AGENTS.md, docs/research/07_TARGET_SEARCH.md를 참고하라.
detection.py에 NumPy만 쓰는 HSV 검출기(고정 target dict 반환)와 TargetTracker(M-of-N 3/5, bearing=atan2(W/2−cx, f), LiDAR 거리 중앙값, world 위치, dedup 반경, visited, memory)를 추가하라.
OpenCV를 새로 의존하지 마라. interfaces.py 변경 금지. tests/test_target_tracker.py 추가.
```
