# 10. Final Architecture & FINAL RECOMMENDATION

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> **이번 pass는 문서만 수정한다. 아래 모듈·수용 기준·prompt는 미래 구현 계획이며 S0도 실행하지 않는다.**
> **이 문서가 최종 결정의 기준 문서다.** 00(INDEX)과 11(ROADMAP)은 이 문서와 일치해야 한다.
> **2026-09-30 공식 TECH WEEK repo(`kyu-rae-kim/PNU-TECHWEEK-260930` @ `383de18`) 기준으로 재검증함.**
> 공식 환경 사실의 상세 근거는 [09](09_WEBOTS_REFERENCES.md), 알고리즘 근거는 02~08, 실패 대응은 12, GPU는 13.
>
> 숫자 태그: **[ORGANIZER]** 운영진 확인 · **[OFFICIAL]** 공식 repo/notebook/Webots R2025a PROTO·문서 · **[DERIVED]** 공식 값에서 계산 · **[MEASURED]** 우리 PC 측정 · **[REFERENCE]** 외부 프로젝트/논문 · **[INITIAL TUNING]** 검증 전 시작값 · **[DAY-OF]** 당일 확인
> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정

---

## 0. 한눈에: 우리가 쓰는 로봇과 센서

| 항목 | 값 | 태그 |
|---|---|---|
| 로봇 | **TurtleBot3Burger** (Webots R2025a), differential drive | [OFFICIAL] [ORGANIZER] |
| 기하 | 바퀴 반경 **0.033 m**, 바퀴 간격 **0.160 m**, 로봇 반경 **0.105 m** (PROTO 외접 ≈ 0.110 m) | [OFFICIAL] notebook·PROTO / [DERIVED] |
| 속도 한계 | 모터 6.67 rad/s → **0.22 m/s**, 제자리 **2.75 rad/s** | [DERIVED] |
| 시뮬 step | 공식 주행 월드 **64 ms** (일부 테스트 월드는 기본 32 ms) | [OFFICIAL], 대회 월드 [DAY-OF] |
| 동기화 | 공식 월드 `synchronization` TRUE (PROTO 기본) | [OFFICIAL], 대회 설정 [DAY-OF] |

| 센서 구분 | 센서 | 우리 사용 |
|---|---|---|
| **REQUIRED** [ORGANIZER] | Wheel encoders (`left/right wheel sensor`, rad) | localization 기본 입력 |
| **REQUIRED** [ORGANIZER] | 2D LiDAR `LDS-01` (360 samples, 0.12–3.5 m, 높이 ≈0.173 m, 회전중심 뒤 0.03 m) | mapping, safety, target 거리 보조 |
| **MISSION PERCEPTION** | Camera `camera` (640×480, 60° HFOV, 높이 ≈0.073 m) | target 검출·위치·접근, 카메라 coverage |
| **OPTIONAL / TEAM CHOICE** [ORGANIZER: IMU 선택] | Gyro (rad/s, raw), Accelerometer | **TEAM CHOICE: Gyro ENABLED 권장(팀 confirmation 대기)** (heading fusion), 가속도는 충돌 감지 보조. Gyro 없거나 끄면 encoder-only로 자동 폴백 |
| **NOT USED by our competition controller** | Compass, GPS [ORGANIZER: 해커톤 미사용] | 사용 안 함 |
| **NOT USED by our competition controller** | Supervisor ground-truth pose (`tb3_ground_truth` 데모) | competition 입력 아님. 개발용 평가 도구로만, 분리된 테스트 모드에서 팀 합의 시 |
| 사용 안 함 | Webots Camera Recognition | ground truth 성격, 미사용 |

## 1. 설계 원칙

1. **모듈형 classical 스택** — 공식 계획안의 Perception → Localization → Mapping → Planning → Control과 같은 구조 [OFFICIAL]; 실제 환경에서 modular가 end-to-end를 압도 (Gervet 2023 📄), Nav2/CMU/Hector도 동일 ✅.
2. **안전은 지도와 독립** — raw scan 안전 모니터가 매 스텝 최종 필터 (Nav2 Collision Monitor ✅). 단 **LiDAR가 못 보는 것**(0.173 m 아래 물체, 0.12 m 이내)이 있으므로 정지 조건·충돌 감지 보조를 둔다.
3. **무거운 계산은 이벤트/저주기, 주기는 "초" 단위로 정의** — step이 64/32 ms로 바뀔 수 있으므로 스텝 수로 하드코딩하지 않는다.
4. **실패는 정상 경로** — progress 감시 → recovery ladder → blacklist → 다음 goal ✅.
5. **LiDAR 탐색 ≠ 카메라 수색** — 60° 카메라 + **바닥 target이 LiDAR 평면 아래일 가능성** → 카메라 coverage를 별도로 관리.
6. **공식 예제 ≠ 대회 규정** — 예제에 있는 Compass/Supervisor/YOLO를 자동으로 채택하지 않는다.
7. **ROS 없이** — 함수 호출·Python 객체·`(x, y, theta)`.

## 2. [1] 추천 최종 Architecture

```text
 SENSORS (devices.py only)  REQUIRED: encoders · LDS-01 2D LiDAR | MISSION: camera | TEAM CHOICE: gyro(+accel) | NOT USED: compass · GPS · supervisor pose
        │ every step (64 ms official)
        ├──────────────────────────────┬────────────────────────────────┐
        ▼                              ▼                                ▼
 LOCALIZATION (every step)      MAPPING (new scan, start ≈128 ms)     PERCEPTION (start ≈128 ms)
 encoder diff-drive odometry    log-odds grid → {-1,0,1}          OpenCV classical CV → target dict (고정 규격)
 + gyro heading (team choice)   CameraCoverageGrid · reset        → TargetTracker (M-of-N, bearing + range → world xy, dedup)
 + [cond.] scan matching        (LiDAR mount offset −0.03 m)       range = LiDAR (target이 평면 위면) else camera ground-plane/size
        └──────────────────────────────┴────────────────────────────────┘
                                       │ pose · grid · targets (병렬, 서로 기다리지 않음)
                                       ▼
 MISSION MANAGER (main.py, FSM)  INITIALIZE(home, optional gyro bias, active scan) → EXPLORE ⇄ APPROACH_TARGET → RETURN_HOME → DONE
        │   + TimeBudget (≈1 s): time_left vs SF·ETA_home
        ▼
 EXPLORATION MANAGER (planning.py) frontier / view-frontier 선택: distance field · IG · exp(−λL) · hysteresis · blacklist
        │ goal (x, y)
        ▼
 NAVIGATOR (navigation.py 제안, pure Python)
   ├─ GLOBAL PLANNER (planning.py)  inflate(r≈0.16 m) → A* → relax_goal → LOS smooth → waypoints; 이벤트 + 저주기 유효성 검사
   ├─ PATH FOLLOWER (control.py)    RPP-lite = notebook look-ahead(κ=2y/L²) + rotate-to-heading + 감속 규칙 + 충돌 예측
   ├─ PROGRESS MONITOR (control.py) best-so-far 잔여거리 + 변위; STUCK/NO_PROGRESS/OSCILLATION
   └─ RECOVERY LADDER              R0 WAIT → R1 CLEAR+REPLAN → R2 SPIN → R3 BACKUP → R4 MARK&GIVE-UP → R5 SAFE STOP
        │ (v, ω)
        ▼
 SAFETY MONITOR (control.py)      every step, last: raw-scan STOP/SLOW zones (minRange blind zone 고려), min_points, rear check, forward sim
        ▼
 MOTORS
```

### 2.1 모듈 책임

| 모듈 | 책임 | 하지 않는 것 |
|---|---|---|
| devices.py | Webots API 전부, 단위 변환, 시작 로그(timestep, synchronization, 센서 스펙·이름) | 판단 |
| localization.py | encoder odometry(필수), gyro heading fusion(team choice, 폴백 가능), (조건부) scan matching | 지도 소유 |
| mapping.py | log-odds 갱신, 3값 grid export, camera coverage, 영역 리셋, 덤프 | 계획 |
| detection.py | 단일 프레임 검출(고정 target dict), TargetTracker(확인·world 위치·중복·방문) | 이동 결정 |
| planning.py | 거리장, A*, 인플레이션, 스무딩, relax, frontier/view-frontier 선택, blacklist, ETA | 모터 명령 |
| control.py | PathFollower, ProgressMonitor, SafetyMonitor, 저수준 명령 | 경로 탐색 |
| navigation.py (제안) | goal 하나를 계획→추종→감시→복구로 끝까지 수행, REACHED/FAILED | 미션 결정 |
| main.py | FSM, 시간 기반 스케줄러, 시간 예산, 로그/덤프 | 알고리즘 세부 |
| config.py | 모든 튜닝값·로봇 사양·device 이름 | 로직 |
| interfaces.py | 고정 규격 (변경 없음) | – |

> `navigation.py`는 제안이다(팀 결정 대기). 원치 않으면 `control.py` 안 `Navigator` 클래스. 어느 쪽이든 Webots 없이 테스트 가능해야 하며 `docs/ARCHITECTURE.md` 갱신은 사람 확인 후.

### 2.2 실행 주기 (64 ms 기준 재결정)

아래는 **[INITIAL TUNING], 공식 runtime 측정 전** 시작안이다. `robot.getBasicTimeStep()`과 실제 sensor sampling을 읽고 deadline을 초 단위로 관리한다. 64 ms는 15.625 Hz, 32 ms는 31.25 Hz [DERIVED]; 1 s deadline은 다음 step에 실행되어 quantization 오차가 생긴다. 누적 누락 횟수·센서 age를 계측한다.

| 작업 | 시작 주기 | 근거 / 확인할 것 |
|---|---|---|
| Encoder odometry, optional gyro, follower, raw safety | 매 step | 가장 먼저/최종 모터 직전에 수행. 공식 런타임 비용 UNCONFIRMED; 오래된 scan이면 정지 |
| LDS-01 enable / log-odds mapping | 128 ms (64 ms에서 2 step), 새 scan 1회만 | 기존 **binary** insert 6–12 ms [MEASURED]는 참고만. log-odds 성능 S4 검증 후 64 ms 확대 여부 결정 |
| Camera / detection / tracker | 128 ms (2 step) | 합성 영상 CV median 2.47 ms [MEASURED]; getImage·render·실제 contour 비용 미측정. 필요 시 320×240로 낮추되 intrinsics 동시 갱신 |
| Camera coverage | 0.256–0.512 s | ray-cast 비용 별도 계측; 실제 camera frame·pose 시각으로 갱신 |
| Global A* | goal 변경/경로 무효/진전 실패 이벤트 | 정기 1 Hz 전체 계획은 하지 않음. 약 1 s 전체 경로 상태 확인 + 새 scan마다 가까운 경로 검사 |
| Frontier selection | goal 도달/실패 이벤트 + 약 2 s 재평가 | 거리장·candidate cache, 동일 map version 재사용; 매 주기 전체 계산 강제 아님 |
| Time budget | 약 1 s 및 target 선점 직전 | 유효한 home 경로/거리장 재사용, 무효 시 보수적 복귀 결정 |
| Recovery / dump | 이벤트 / 약 2 s·종료 | dump가 safety를 막지 않도록 budget 제한 |

[MEASURED] 09 §8의 이전 PC 합성 grid A* 128/514 ms(320/480 grid)는 64 ms를 초과할 수 있음을 보여준다. **주기를 1 Hz로 낮춰도 한 번의 긴 blocking 계산은 해결되지 않는다.**

- 공식 sample sync TRUE에서는 `step()` 사이 계산 동안 simulation이 기다린다. wall-clock은 소모되며 64 ms 샘플링/구동 지연 자체가 없어지는 것은 아니다. 최종 sync·미션 시계는 [DAY-OF].
- 무거운 계산은 step budget 내로 나누거나 확장 수/시간 상한으로 중단·보류한다. 계산 전 정지할 때는 **모터 setVelocity(0) 후 `robot.step(timestep)`으로 명령을 전달하고 다음 센서 상태 확인 후** 계산한다. 단순 setter 호출만으로 즉시 정지했다고 간주하지 않는다.
- 비동기 설정이면 wall-clock과 simulation 경과시간이 1:1이라는 보장이 없다. 긴 blocking 계획 중 매-step safety 실행을 주장하지 않는다. bounded/incremental 계산이 검증되기 전에는 운행하지 않는다.
- 큰 map은 planning용 0.10 m coarse grid 옵션, 보수적 footprint rasterization, 거리장 재사용을 S3에서 비교한다. 0.05 m grid의 3.22셀 inflation을 정수 3으로 내림하지 않는다.

## 3. [2] 사용할 Algorithm

| 영역 | 선택 | 대안/버림 |
|---|---|---|
| **Localization** | **Base**: encoder differential-drive odometry (필수 센서) + 당일 보정 확인. **Team choice (ENABLED)**: gyro heading fusion(정지 bias 추정, gyro 없으면 폴백). **Conditional**: LiDAR correlative scan matching — §11 Q9 테스트로 필요할 때만 | 버림: pose graph, loop closure, AMCL, ICP 단독, **Compass/GPS/Supervisor pose** |
| **Mapping** | 2D occupancy grid + log-odds + clamp (hit +0.85, miss −0.4, [−2, 3.5] [REFERENCE: OctoMap]), 스캔당 셀 1회·hit 우선, LiDAR mount offset 반영 | 버림: 영구 OCCUPIED(현재), 전체 temporal decay |
| **Exploration** | frontier(FREE 쪽, 도달 가능) → 소진 후 view frontier(카메라 미탐색) → 복귀 | 버림: TSP 투어, RRT frontier, 학습 탐색 |
| **Frontier Selection** | `u = G·exp(−λ·L_eff)`, L = 거리장 경로거리 + 회전 환산, G = unknown-in-radius(+카메라 미탐색 가중), hysteresis(현재 goal ×1.3 + 연속 2~3회 승리), blacklist(0.3 m, TTL 90 s, second chance) — 수치 모두 [INITIAL TUNING] | m-explore 가산식(Euclidean) |
| **Global Planner** | A* (공식 교육 = 4-연결 Manhattan [OFFICIAL]; **우리 개선 옵션 = 8-연결 octile, corner-cut 금지**) + 약한 danger 비용 + relax_goal + LOS smoothing; 다중 목표는 BFS 거리장; known-only 우선 | 버림: D* Lite, LPA*, JPS, Theta*, Hybrid-A* |
| **Local Control** | **RPP-lite** = notebook의 look-ahead pure pursuit(`κ=2y/(x²+y²)`, `ω=vκ`) [OFFICIAL 교육 내용] + rotate-to-heading + 곡률/근접/접근 감속 + 1 s 충돌 예측 [REFERENCE: Nav2 RPP] | 버림: DWA/DWB(P3 이하 — 계획안의 예시일 뿐 필수 아님), TEB, MPC/MPPI |
| **Collision Avoidance** | 인플레이션 전역 경로 + follower 충돌 예측 + Safety monitor(raw scan STOP/SLOW, min_points, 후방 검사, **minRange 사각 고려**) + 저위 장애물용 충돌 감지(진전 0 + 가속도 spike → collision map) | 버림: VFH, APF, VO/ORCA |
| **Dynamic obstacles** | 추적 없음: WAIT(≈4 s, 상한 있음) → 재계획 → ladder; log-odds로 일시 반영 | P3: 클러스터 추적 |
| **Recovery** | R0 WAIT → R1 CLEAR(주변 log-odds 리셋)+REPLAN → R2 SPIN(외접 0.11 m + 여유 확인) → R3 BACKUP(0.10–0.15 m, 후방 확인) → R4 collision_map+blacklist → R5 SAFE STOP | Nav2 BT 프레임워크(개념만) |
| **Detection** | **OpenCV classical CV**: blur → HSV 또는 LAB `inRange` → morphology → contour → 최대 contour 면적·중심(cx)·외접원 [공식 교육/예제와 같은 계열]. NumPy-only는 폴백. 색/형태 임계는 [DAY-OF] | YOLO: 공식 예제 존재하지만 baseline 아님 (13) |
| **Target Tracking** | M-of-N(3/5) + 위치 분산 [INITIAL TUNING], bearing = atan((320−cx)/554.3) [DERIVED], range = LiDAR(target이 LiDAR 평면에 걸릴 때) 또는 카메라 ground-plane/크기 → world xy, dedup, 방문 표시, memory | Kalman 다중 추적 |
| **Target Approach** | A* → standoff(relax_goal, 가시선) → 근거리 visual servo(cx 중심, 거리 기반 v) → 도착 판정 [DAY-OF 규정] | – |
| **Return Home** | home = local map 시작 pose (0,0,0); 제공 world pose는 정적 좌표 변환에만 사용; `time_left < SF·ETA_home + margin`(SF 1.4, margin 15 s [INITIAL TUNING], ETA는 실측 평균 속도로); known-only A* → ghost 리셋 → 인플레이션 축소 → breadcrumb → unknown 허용 | GPS/Compass 없음 |
| **Mission decision** | **FSM 유지** — 공식 notebook이 FSM과 BT를 모두 소개하고 FSM은 "상태 수가 적고 순차적인 경우"에 적합하다고 설명 [OFFICIAL 교육]; 우리 상태는 5개. 복잡한 예외 처리는 Navigator 내부 recovery ladder로 분리 | BT 프레임워크(과함) |

## 4. [3] Reference별로 무엇을 참고했는가

| Reference | 구분 | 참고한 것 |
|---|---|---|
| **PNU TECH WEEK 공식 repo** (notebook, controllers, worlds) ✅ | **공식 환경 사실** | 로봇·센서·timestep·카메라·LiDAR 순서, look-ahead 식, A*/Dijkstra 교육, FSM/BT, OpenCV LAB/HSV 예제 |
| Webots R2025a TurtleBot3Burger / RobotisLds01 PROTO ✅ | 공식 환경 사실 | 바퀴 기하, 모터 한계, 센서 위치, LiDAR 스펙, synchronization 기본값 |
| m-explore / m-explore-ros2 ✅ | 외부 알고리즘 | frontier·클러스터·크기 필터·blacklist·종료·return_to_init |
| hector_exploration_planner ✅ | 외부 알고리즘 | 경로비용 frontier, danger 비용, inner exploration → view frontier |
| rrt_exploration ✅ | 외부 알고리즘 | IG = 반경 내 unknown 넓이, hysteresis gain |
| GBPlanner ✅ | 외부 알고리즘 | exp(−λL), 시간 예산 homing 식 |
| TARE / FUEL ✅ | 외부 알고리즘 | hysteresis 임계, 회전 포함 시간 비용, ray-cast IG (TSP는 버림) |
| VLFM ✅ | 외부 알고리즘 | sticky frontier, acyclic enforcer |
| SemExp / PONI ✅ | 외부 알고리즘 | collision map, 방문 셀 통과, goal 팽창 |
| Nav2 ✅ | 외부 알고리즘 | inflation, progress/goal checker, 재계획 BT, recovery, RPP, Collision Monitor, TB3 burger 파라미터 |
| move_base ✅ | 외부 알고리즘 | patience, oscillation, 단계적 리셋 |
| CMU AEDE ✅ | 외부 알고리즘 | heading 추종 follower, path library(P3) |
| FAR planner ✅ | 외부 알고리즘 | known-first 폴백, goal 재평가, dynamic 분리 |
| Hector SLAM / OctoMap / slam_toolbox ✅ | 외부 알고리즘 | log-odds 규칙, CSM 구조와 odom prior |
| Webots 공식 문서 ✅ | 플랫폼 | Lidar 순서/inf/noise, Camera FOV, synchronization, Gyro lookupTable 기본 |
| Erebus ✅ | 외부 사례 (TECH WEEK 규정 아님) | 멈춤에 상한이 필요하다는 교훈 |
| e-puck / webots_ros2 e-puck | **practice 환경 전용** | 대회 튜닝 근거로 쓰지 않음 |
| Gervet 2023, Macenski 2023 survey 📄 | 설계 근거 | 모듈형 구조 |

## 5. [4] 구현 순서 (요약 — 상세·수용 기준은 [11](11_IMPLEMENTATION_ROADMAP.md))

| Step | 내용 | Priority |
|---|---|---|
| **S0** | **공식 환경 audit + config 이행 + 계측**: device 이름(`LDS-01`), TB3 기하·속도, gyro 단위 1.0, LiDAR offset(−0.03,0), timestep 기반 주기, grid 크기, Compass/GPS 비활성, OS별 runtime.ini, step 시간·LiDAR 갱신 간격·getImage 비용 로그 | P0 |
| S1 | Encoder odometry 검증(명령 이동 대비 오차) + **TEAM CHOICE: gyro heading fusion ENABLED**(bias 추정, 폴백) | P0 |
| S2 | PathFollower(RPP-lite) + SafetyMonitor(TB3 footprint, minRange 사각) + opt-in `NAV_TEST` | P0 |
| S3 | planning 유틸: distance_field, 8-연결 옵션, relax_goal, smooth_path, path_is_valid (+ 큰 맵 성능 측정) | P0 |
| S4 | log-odds mapping (+ reset_region, mount offset) | P0 |
| S5 | Navigator: 계획→추종→ProgressMonitor→기본 recovery(R0/R2/R3/R4 = collision map 포함) → `goto(goal)` | P0 |
| S6 | RETURN_HOME: 시간 예산 + 경로 폴백 MVP(known-only → breadcrumb → unknown 허용; 전체 5단계는 08 §2.3, S10에서 완성) | P0 |
| S7 | Exploration: frontier 선택(G1 + exp utility + hysteresis) + blacklist + 종료 | P0 |
| S8 | Target: OpenCV classical detector + TargetTracker(카메라 기반 거리 포함) + 2단계 접근 | P0 |
| S9 | Mission 통합(FSM) + Initial Active Scan | P0/P1 |
| S10 | P1 강화: camera coverage + view frontier, full ladder, danger 비용, 동적 WAIT 정책, 오도메트리 보정 절차, 심사용 덤프 | P1 |
| S11 | P2: ray-cast IG, 카메라 IG를 utility에, 다가오는 장애물 TTC, 조건부 scan matching, look-around | P2 |

## 6. [5] 버릴 기능 (DO NOT IMPLEMENT)

ROS/ROS2 노드·토픽·액션·TF·BT 프레임워크 · Nav2 설치 · TSP/GTSP 전역 투어 · 3D 탐색 본체(TARE/FUEL/GBPlanner) · RRT frontier · D* Lite/LPA*/JPS/ARA* · Hybrid-A*/State lattice · DWA/DWB(P3 이하) · TEB · MPC/MPPI · VFH/APF 단독 · VO/RVO/ORCA · pose graph/loop closure · AMCL · 새 SLAM 의존성(KISS-ICP 등) · 학습 탐색(SemExp/PONI/VLFM 네트워크) · GPU DNN을 baseline으로 · **대회 controller에서 Compass/GPS/Supervisor pose/Webots Recognition 사용** · 다중 로봇 · 전체 temporal decay 지도.

## 7. [6] 대회 당일 확인/변경 (모두 config.py 또는 운영진 질문)

| 항목 | 현재 근거 | 확인 |
|---|---|---|
| 대회 월드의 basicTimeStep, synchronization | 공식 예제 64 ms(일부 32), sync TRUE | [DAY-OF] 시작 로그 |
| 로봇/센서가 공식 예제와 동일한지 (extensionSlot 구성, 카메라 위치, LiDAR noise) | 공식 월드 기준 | [DAY-OF] 시작 로그 |
| target 외형·크기·개수(`REQUIRED_TARGETS`), 도착 인정 조건 | 공식 사과/공은 CV 예시일 뿐 | [DAY-OF] 운영진 |
| 전체 미션 시간과 기준(sim/real) | – | [DAY-OF] |
| 시작 pose 제공 형식 | 계획안: position & orientation 제공 | [DAY-OF] |
| 아레나 크기 → GRID 크기/해상도 | 공식 예제 월드 13 m급 | [DAY-OF] |
| 대회 PC OS/Python 환경 (Ubuntu 22.04면 runtime.ini `COMMAND`), OpenCV/NumPy 버전 | 공식: Ubuntu 22.04, numpy 1.23.5, opencv 4.8.0.74 | [DAY-OF] |
| 제3자 라이브러리 제한, Supervisor 개발 사용 가능 여부 | 규정 미확인 | [DAY-OF] 운영진 |

## 8. [7] 최소 완주 버전과 [8] 상위 버전

| 버전 | 포함 | 기대 |
|---|---|---|
| **M0 Walking Skeleton** | encoder odom + binary grid(`_mark_free`가 OCCUPIED도 비움) + 거리장 최근접 도달가능 frontier + A* + 단순 look-ahead follower + 기본 e-stop(TB3 값) + 단일 프레임 검출·접근 + 고정 시간 여유 복귀 | 통합 위험 제거 |
| **MVP** | S0–S9: 공식 환경 이행, gyro fusion(team choice), RPP-lite, safety, log-odds, navigator(기본 recovery), frontier(G1)+blacklist, OpenCV CV + M-of-N + 접근, 시간 예산 복귀 | 충돌 없이 탐색·target 접근·복귀 |
| **Stable** | + full recovery ladder, danger 비용, 동적 WAIT 정책, 8-연결+smoothing, 오도메트리 보정, 시간 기반 스케줄러 튜닝 | 끼임/사람/ghost에도 완주 |
| **Competitive** | + camera coverage·view frontier, Initial Active Scan 튜닝, utility·hysteresis 튜닝, 심사용 덤프 | target 누락 감소, 설명력 |
| **Stretch** | + ray-cast IG, 다가오는 장애물 TTC, 조건부 scan matching, look-around, arc sampler | 차별화 |

## 9. [9] Baseline 파일 1:1 매핑

| 파일 | 추가/변경 |
|---|---|
| `config.py` | **S0: TB3 값으로 교체**(device 이름 `LDS-01`, wheel 0.033/separation 0.160/notebook radius 0.105와 safety radius 0.111 구분, 6.67 rad/s, gyro 1.0, LiDAR offset −0.03, compass/gps 비활성), 시간 기반 주기, 모든 새 파라미터, opt-in 모드 |
| `interfaces.py` | **변경 없음** (target world 위치를 공유 규격으로 만들 때만 사람 확인 + INTERFACES.md·tests 동시 수정) |
| `devices.py` | 시작 로그(timestep, synchronization, LiDAR min/max/resolution, camera FOV/해상도), gyro 존재 여부로 폴백 |
| `mapping.py` | log-odds + export, mount offset, `reset_region`, `CameraCoverageGrid`, 덤프 |
| `localization.py` | encoder odometry 검증, gyro fusion(선택), bias, (조건부) `ScanMatcher` |
| `planning.py` | `distance_field`, `obstacle_distance`, `astar(connectivity=)`, `relax_goal`, `smooth_path`, `path_is_valid`, frontier 선택, `FrontierBlacklist`, `select_view_frontier`, `eta_home` |
| `control.py` | `PathFollower`, `SafetyMonitor`, `ProgressMonitor`, spin/backup |
| `navigation.py` (제안) | `Navigator` |
| `detection.py` | OpenCV classical detector(고정 target dict), `TargetTracker` |
| `main.py` | FSM, 시간 기반 스케줄러, TimeBudget, 로그/덤프 |
| `runtime.ini` | OS별 `COMMAND` (현재 Windows `$(LOCALAPPDATA)` 전용) |
| `tests/` | 모듈별 신규 테스트 (11 참고) |

## 10. [10] 첫 구현 작업 prompt 초안 → [11 §6](11_IMPLEMENTATION_ROADMAP.md#6-claudecodex-작업-prompt-초안)

---

## 11. 핵심 질문 답 (공식 repo 재검증 후)

**Q1. IMU를 쓰나?** — **TEAM CHOICE: Gyro ENABLED (권장).** IMU는 선택 [ORGANIZER]이지 필수가 아니다. 근거: (a) 차동구동 heading 오차의 주원인은 제자리 회전·충돌 시 바퀴 미끄러짐이고 gyro가 이를 직접 관측, (b) TB3 Gyro는 lookupTable 없이 rad/s를 바로 반환 [OFFICIAL] → 변환 1줄, (c) encoder translation을 유지하고 yaw 증분만 보조하는 구조라 전체 SLAM보다 작지만 bias·단위·slip·fallback 검증이 필요(구현 시간 UNCONFIRMED, 정지 bias 1 s는 INITIAL TUNING). **폴백**: gyro 장치가 없거나 값이 비정상이면 encoder-only odometry로 자동 전환(pose 인터페이스 동일). 가속도계는 충돌/미끄러짐 감지 보조로만.

**Q2. Detection baseline** — **OpenCV classical CV.** 공식 교육 자료와 예제가 OpenCV(`opencv-python==4.8.0.74`)로 blur → HSV/LAB `inRange` → morphology → contour → centroid를 가르친다 [OFFICIAL]. 이전 PC 합성 640×480에서 median 2.47 ms [MEASURED]였으나 공식 runtime·정확도는 미검증이다. 단 **대회에서 제3자 라이브러리 제한이 없는지는 최종 확인 대기**("official materials support it; final rule confirmation pending"). NumPy-only 임계는 폴백으로 유지. YOLO는 13 참고(baseline 아님).

**Q3. RPP-lite ↔ notebook look-ahead** — 잘 맞는다. notebook의 look-ahead 제어(최근접 waypoint → 경로거리 기준 look-ahead 점 → 로봇 frame → `κ=2y/L²`, `ω=vκ`, v 상수 가능)는 pure pursuit이고, RPP-lite는 여기에 **rotate-to-heading, 곡률/근접/접근 감속, 충돌 예측**만 더한다 [REFERENCE: Nav2 RPP]. notebook은 Nav2 RPP 기능을 요구하지 않으며, 추가 기능은 우리 선택이다.

**Q4. 60° 카메라에서도 CameraCoverage/View Frontier 가치?** — **유지한다.** 60°는 360°의 1/6이고, 공식 바닥 공/사과 같은 낮은 예시는 **LiDAR 평면(약 0.173 m, robot-local) 아래라 관측되지 않을 수 있다** [DERIVED]. 대회 target 형상은 [DAY-OF]이므로 camera를 주 인식 입력으로 두고 LiDAR range 대응은 확인될 때만 쓴다. 640 px에서 기하학적 픽셀 수는 늘지만 검출 성공 거리는 미측정이다. 0.1 m 물체가 3 m에서 약 18 px라는 계산 [DERIVED]을 참고해 coverage 거리 한계는 [INITIAL TUNING] 약 3 m부터 검증한다. FREE 바닥도 camera coverage 대상이며 지도 FREE가 수색 완료를 뜻하지 않는다.

**Q5. 64 ms 기준 scheduler** — §2.2로 재결정. 매 step odometry·follower·safety, mapping/detection은 약 128 ms 시작, coverage 저주기, 계획·frontier 이벤트 기반. 기존 binary/CV 벤치는 참고이며 통합 workload는 S0/S4 계측 전 UNCONFIRMED. 기존 "16 ms step" 전제는 폐기(practice 환경 값).

**Q6. 1 Hz global planning 여전히 합리적?** — **정기 1 Hz 전체 재계획은 하지 않는다.** 공식 예제 월드(13 m급)에 해당하는 16–24 m 맵에서 순수 Python A* 최악 128–514 ms [MEASURED] > 64 ms step. 대신 **이벤트 기반 재계획 + ≈1 s 저비용 경로 유효성 검사**(Nav2 "replan only if path invalid" BT와 같은 방식 [REFERENCE]). 큰 맵 대책(0.10 m planning grid, 확장 상한)은 S3 측정 후 결정.

**Q7. TurtleBot3 기준 시작값** [INITIAL TUNING] — 인플레이션 = 안전 외접 0.111(올림) + 0.05 = **0.161 m (3.22셀)**; notebook 교육 반경 0.105와 구분; STOP 영역 = 로봇 외곽 + 0.05 m (LiDAR 기준 거리로 환산하고 **minRange 0.12 m 사각 고려**); SLOW 영역 = 외곽 + 0.25 m; 운용 최대 v **0.15 m/s**(한계 0.22), ω ≤ **1.5 rad/s**(한계 2.75); look-ahead = 1.5 s × v ≈ **0.2 m**, [0.15, 0.40] clamp; BACKUP **0.10–0.15 m**(뒤쪽 몸체가 축 뒤 0.10 m); SPIN 전 주변 **0.13 m** 비어 있는지 확인; progress **0.15 m / 10 s**(TB3 Nav2 0.1 m/10 s [REFERENCE]). 모두 S2 NAV_TEST에서 튜닝.

**Q8. LiDAR 순서 ↔ mapping convention** — 현재 `LIDAR_FIRST_ANGLE = π`, `LIDAR_ANGLE_DIRECTION = −1`은 공식 예제 라벨(0=Back, 90=Left, 180=Front, 270=Right)과 Webots 문서(왼→오)에 대표 index/회전 방향이 **부합**한다. 정확한 beam-center 각도는 UNCONFIRMED로 두고 S0 벽/point-cloud 대조 후 확정한다. 이행 항목은 `DEVICE_NAMES["lidar"] = "LDS-01"`과 `LIDAR_MOUNT_OFFSET = (−0.03, 0.0)`. S0에서 공식 배치를 참고한 허용된 개발 환경에서 전후좌우 벽 거리 검증 테스트.

**Q9. GPS/Compass 없이 scan matching 필요 판단 테스트** — (1) **Loop closure test**: 명령 경로로 사각/왕복 주행 후 시작 지점 복귀 → 첫 스캔과 마지막 스캔을 오프라인 CSM으로 정합한 오프셋 = drift 추정. (2) **Commanded motion test**: 1 m 직진·360° 회전 후 벽까지 LiDAR 거리 변화 vs odometry. (3) **Map consistency**: 같은 벽이 두 줄로 그려지는 두께/어긋남. (4) **Home return consistency**: 미션 종료 시 home에서 스캔-맵 잔차. 기준 [INITIAL TUNING]: 미션 길이 주행 후 오프셋 ≥ 0.10 m 또는 ≥ 3° 또는 반복 wall doubling이면 scan matching A/B 시험. scan 자체의 노이즈·대칭/동적 물체 오정합을 분리하고 반복 관측으로 확인하며 명령 이동량을 ground truth로 취급하지 않는다. 개선 효과와 비용이 확인될 때만 도입한다. (선택) Supervisor 기반 정답 비교는 **개발용 분리 모드에서만**, 팀 합의 후. 참고: 공식 계획안이 Scan Matching을 활용 가능 기술로 명시 → '기술 구현' 점수 차원의 도입 여부는 **팀 결정 사항**.

**Q10. S0~S11 순서 변경?** — (a) **S0이 "공식 환경 audit + config 이행"으로 확대**되어 모든 단계의 선행 조건이 됨(e-puck 값이 그대로면 모든 튜닝·안전이 틀림). (b) S1 문구가 "gyro 필수"에서 "encoder 검증 + gyro team choice"로 변경. (c) S5 기본 recovery에 **collision map(R4)이 포함됨을 명시** — LiDAR가 저위 장애물을 못 보기 때문. (d) S8 detector가 NumPy → OpenCV classical, 거리 추정에 카메라 ground-plane 추가. 그 외 순서는 유지.

**추가 Q (기존 Q1–Q15 중 여전히 유효한 답)**
- 상위권 요소: 충돌·끼임 0회, 카메라 coverage 기반 수색, target 위치 정확도·중복 처리, 시간 예산으로 항상 복귀, 지도·경로·target 산출물.
- 위험 지점: (1) LiDAR 사각(저위 물체, 0.12 m 이내)에서 충돌 (2) stuck/진동으로 시간 소진 (3) ghost·drift로 home 경로 소실 (4) target 누락/오탐 (5) practice(e-puck) 값이 남은 채 실행.
- 현재 baseline의 위험한 가정: 영구 OCCUPIED, e-puck 튜닝값, `"lidar"` device 이름, LiDAR offset 0, 8 m grid, 16 ms 가정, Windows 전용 runtime.ini, `allow_unknown=True` 기본 복귀, frontier goal relaxation 부재, 고정 `MISSION_TIME_LIMIT`, LiDAR coverage = 수색 완료라는 가정.


### 11.1 구현 계약에서 빠뜨리지 않을 조건

- [ORGANIZER] Encoder·2D LiDAR는 mandatory. 둘 중 없거나 invalid/stale이면 **STOP + 명시적 오류**이며 gyro처럼 선택적 생략하지 않는다. Gyro는 없어도 encoder-only로 동일 pose interface를 유지한다.
- local map +x는 시작 heading, +y는 시작 좌측, theta=0은 시작 방향, CCW 양수. `Δs=r(ΔφR+ΔφL)/2`, `Δθ=r(ΔφR−ΔφL)/L`, midpoint heading으로 x/y 적분. r=0.033 m, L=0.160 m [OFFICIAL]. 제공 world pose와의 변환은 모든 map/target/home에 일관 적용한다. 기존 코드 인터페이스 변경은 미래 구현 때 관련 규칙을 따른다.
- LiDAR 점은 `p_map = pose ⊕ ((−0.03,0) + r_i(cos α_i,sin α_i))`. clearing ray 시작도 sensor origin이며 minRange 내부 blind cells는 비우지 않는다. `α_i≈π−i·FOV/N`은 대표 index 기반 시작식, 정확한 sampling 각도는 S0 검증. 기본 inf/NaN/out-of-range ray는 skip; 모든 inf를 maxRange free로 바꾸지 않는다.
- log-odds: hit +0.85, miss −0.40, clamp [−2,3.5]는 [REFERENCE→INITIAL TUNING]. export hysteresis +0.4/−0.2에서는 포화 occupied→FREE에 **10 miss**가 필요하다(9회면 −0.1). 알려진 FREE도 LiDAR 높이의 관측일 뿐 바닥 충돌 부재를 증명하지 않는다.
- 안전 시작값 0.111 m 외접(올림) + margin 0.05 m; 정지 여유는 `v·sensor_age + v·command_latency + v²/(2a_brake) + uncertainty` 이상으로 실측 검증한다. a_brake는 UNCONFIRMED. 최대 v·ω를 동시에 독립 허용하지 않고 `|(v±ωL/2)/r|≤6.67 rad/s` wheel limit로 함께 제한한다.
- backup/spin은 swept footprint·현재 finite scan·최근 충돌 증거를 확인해야 한다. inf는 안전 증거가 아니다. encoder-only는 바퀴 공회전 중 무이동을 감지 못 할 수 있어 반복 scan 정합/벽 거리 변화로 보완한다. 가속도 spike는 충돌 확정 판정이 아니다.
- target bearing `atan((320−cx)/554.3)`는 camera 기준 [DERIVED]; LiDAR/camera 원점 차이와 robot pose를 적용한다. 바닥 접점/알려진 크기 기반 range는 조건부(07 §5), 근거가 없으면 위치 UNCONFIRMED로 두고 접근을 보류한다.
- 동일한 실패 횟수·WAIT timeout·blacklist 0.30 m/90 s·progress 0.15 m/10 s는 [INITIAL TUNING]. 현재 센서 hit를 강제 FREE로 덮거나 안전 footprint보다 작은 inflation으로 탈출하지 않는다.
