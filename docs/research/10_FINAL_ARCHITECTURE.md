# 10. Final Architecture & FINAL RECOMMENDATION

> 이 문서가 **최종 결정의 기준 문서**다. 00(INDEX)과 11(ROADMAP)은 이 문서와 일치해야 한다.
> 근거 상세는 02~09, 12, 13. ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정/당일 확인.

---

## 1. 설계 원칙 (레퍼런스에서 도출)

1. **모듈형 classical 스택** — 실제 환경에서 modular가 end-to-end를 압도 (Gervet 2023 📄), Nav2/CMU/Hector 모두 동일 구조 ✅.
2. **안전은 지도와 독립** — raw scan 안전 모니터가 매 스텝 최종 필터 (Nav2 Collision Monitor ✅).
3. **무거운 계산은 이벤트/저주파** — 전역 계획 ~80–100 ms(실측) vs 스텝 16 ms; Webots 대회는 비동기 컨트롤러일 수 있음 ✅ robot.md.
4. **실패는 정상 경로** — progress 감시 → recovery ladder → blacklist → 다음 goal (Nav2 BT, move_base, m-explore ✅).
5. **LiDAR 탐색 ≠ 카메라 수색** — 카메라 coverage를 별도로 관리 (hector inner exploration ✅의 S&R 확장).
6. **ROS 없이** — 토픽/액션/TF 대신 함수 호출·파이썬 객체·`(x, y, theta)`.

## 2. [1] 추천 최종 Architecture

```text
 SENSORS (devices.py only)   LiDAR 360° · Camera · Encoders · Gyro · Accel · (Compass if any) · [GPS = debug only]
        │ every step (16 ms)
        ├──────────────────────────────┬───────────────────────────────┐
        ▼                              ▼                               ▼
 LOCALIZATION (every step)      MAPPING (5 Hz)                  PERCEPTION (~15 Hz)
 encoder dist + gyro heading    log-odds grid → {-1,0,1}        HSV (NumPy) → target dict (고정 규격)
 bias · slip flag · [P2 CSM]    CameraCoverageGrid · reset      → TargetTracker (M-of-N, world xy, dedup)
        │ pose ─────────────────────▶ (pose 사용)                 ◀── pose + raw scan (지도에 의존하지 않음)
        └──────────────────────────────┴───────────────────────────────┘
                                       │ pose · grid · targets  (세 모듈은 병렬, 서로의 출력을 기다리지 않음)
                                       ▼
 MISSION MANAGER (main.py)        INITIALIZE(bias, home, active scan) → EXPLORE ⇄ APPROACH_TARGET → RETURN_HOME → DONE
        │   + TimeBudget (1 Hz): time_left vs SF·ETA_home
        ▼
 EXPLORATION MANAGER (planning.py) frontier/view-frontier 선택: distance field · IG · exp(−λL) utility · hysteresis · blacklist
        │ goal (x, y)
        ▼
 NAVIGATOR (navigation.py 신규, pure Python)
   ├─ GLOBAL PLANNER (planning.py)   inflate → A* (8-conn, danger cost) → relax_goal → LOS smooth → waypoints; 1 Hz + events
   ├─ PATH FOLLOWER (control.py)     RPP-lite: rotate-to-heading · pure pursuit · regulated speed · collision-imminent check
   ├─ PROGRESS MONITOR (control.py)  best-so-far remaining + displacement; STUCK/NO_PROGRESS/OSCILLATION
   └─ RECOVERY LADDER               R0 WAIT → R1 CLEAR+REPLAN → R2 SPIN → R3 BACKUP → R4 MARK&GIVE-UP → R5 SAFE STOP
        │ (v, ω)
        ▼
 SAFETY MONITOR (control.py)      every step, last: STOP/SLOW zones on raw scan, min_points, rear check, 1 s forward sim
        ▼
 MOTORS (devices via controller)
```

### 2.1 모듈 책임

| 모듈 | 책임 | 하지 않는 것 |
|---|---|---|
| devices.py | Webots API 전부, 단위 변환, 시작 로그(`getSynchronization`, 센서 스펙) | 판단 |
| localization.py | pose 추정, gyro bias, slip 플래그, (P2) CSM | 지도 소유 |
| mapping.py | log-odds 갱신, 3값 grid export, camera coverage, 영역 리셋, 덤프 | 계획 |
| detection.py | 단일 프레임 검출(고정 target dict), TargetTracker(확인·world 위치·중복·방문 표시) | 이동 결정 |
| planning.py | grid 알고리즘(거리장, A*, 인플레이션, 스무딩, relax), frontier/view-frontier 선택, blacklist, ETA | 모터 명령 |
| control.py | PathFollower, ProgressMonitor, SafetyMonitor, 저수준 명령 | 경로 탐색 |
| navigation.py (신규) | goal 하나를 "계획→추종→감시→복구"로 끝까지 수행, 결과 REACHED/FAILED 반환 | 미션 결정 |
| main.py | 상태기계, 스케줄러(주기), 시간 예산, 로그/덤프 | 알고리즘 세부 |
| config.py | 모든 튜닝값 | 로직 |
| interfaces.py | 고정 규격 (변경 없음) | – |

> `navigation.py` 신규 파일은 제안이다. 팀이 원치 않으면 `control.py` 안의 `Navigator` 클래스로 둬도 된다. 어느 쪽이든 Webots 없이 테스트 가능해야 하며 `docs/ARCHITECTURE.md` 갱신은 사람 확인 후.

### 2.2 실행 주기 (initial tuning suggestion + 근거)

| 작업 | 주기 | 근거 |
|---|---|---|
| Safety monitor, path follower, odometry+gyro | 매 스텝 (16 ms ≈ 62.5 Hz) | 계산 <1 ms. Nav2 controller 20 Hz(기본)/10 Hz(TB3) ✅, CMU pathFollower 100 Hz ✅ |
| Mapping (log-odds insert) | 5 Hz (12 스텝) | insert 5–10 ms 실측; Nav2 local costmap 5 Hz ✅; e-puck 0.08 m/s → 0.2 s에 1.6 cm (< 1셀) |
| Detection + TargetTracker | ~15 Hz (4 스텝) | 52×39 이미지 NumPy <1 ms 예상 ⚠️; 제자리 회전 스캔 중 누락 방지 |
| Camera coverage 갱신 | 5 Hz | ray-cast 비용 |
| Global planning | 1 Hz + 이벤트(경로 무효·goal 변경·진전 실패) | Nav2 BT `RateController hz=1.0` ✅ |
| Frontier 선택 | 1 Hz (계획과 함께) 또는 goal 도달/실패 이벤트 | m-explore 0.33/0.15 Hz ✅, FAR 2.5 Hz ✅ |
| Time budget | 1 Hz | 거리장 재사용 |
| Recovery | 이벤트 | – |
| Map/target 덤프 | 0.5 Hz + 종료 시 | 심사 증거 |

스케줄러는 **한 스텝에 무거운 작업을 두 개 이상 겹치지 않게** 오프셋을 둔다 (예: mapping은 step%12==0, planning은 step%62==6).

## 3. [2] 사용할 Algorithm

| 영역 | 선택 | 대안/버림 |
|---|---|---|
| **Localization** | encoder 거리 + gyro heading(gyrodometry), 정지 bias 추정, 당일 오도메트리 보정 절차 | P2 조건부: numpy correlative scan matching(Olson/Karto 축소판). 버림: pose graph, loop closure, AMCL, ICP 단독 |
| **Mapping** | log-odds + clamp (hit +0.85, miss −0.4, [−2, 3.5]), 스캔당 셀 1회 갱신·hit 우선, 5 Hz | 버림: 영구 OCCUPIED(현재), 전체 temporal decay |
| **Exploration** | frontier(FREE 쪽, 도달 가능) → 소진 후 view frontier(카메라 미탐색) → 복귀 | 버림: TSP 투어(TARE/FUEL/GTSP), RRT frontier 탐지, 학습 탐색 |
| **Frontier Selection** | `u = G·exp(−λ·L_eff)`, L = 거리장 경로거리 + 회전 환산, G = unknown-in-radius(+카메라 미탐색 가중), 현재 goal ×1.3 + 연속 2~3회 승리 교체, blacklist(0.25 m, TTL 90 s, second chance) | m-explore 가산식(Euclidean) |
| **Global Planner** | A* 8-연결 octile(no corner cut) + 약한 danger 비용 + relax_goal + LOS smoothing; 다중 목표는 BFS/Dijkstra 거리장; known-only 우선 | 버림: D* Lite, LPA*, JPS, Theta*(→ smoothing으로 대체), Hybrid-A* |
| **Local Control** | RPP-lite (rotate-to-heading 임계 ~0.6 rad, lookahead 0.10–0.15 m, 곡률/근접/접근 감속, goal 허용 0.05 m) | 버림: DWA/DWB(P3 이하), TEB, MPC/MPPI |
| **Collision Avoidance** | 전역 경로(인플레이션) + follower 1 s 전방 시뮬 + Safety monitor(raw scan STOP/SLOW, min_points, 후방 검사) | 버림: VFH, APF, VO/ORCA |
| **Dynamic obstacles** | 추적 없음: WAIT 4 s → 재계획 → ladder; log-odds로 일시 반영; 접근 판정은 최소거리 감소율 | P3: 클러스터 추적 |
| **Recovery** | R0 WAIT → R1 CLEAR(주변 log-odds 리셋)+REPLAN → R2 SPIN → R3 BACKUP(0.05–0.08 m) → R4 collision_map+blacklist → R5 SAFE STOP | Nav2 BT 프레임워크 자체는 버림(개념만) |
| **Detection** | HSV 임계 + 연결요소 + 면적/종횡비 필터, **NumPy만** (OpenCV 편입은 사람 확인 후) | 버림: DNN/GPU, Webots Recognition(규정 확인 전) |
| **Target Tracking** | M-of-N(3/5) + 위치 분산, bearing(픽셀→각) + LiDAR 거리 → world xy, dedup 0.3 m, 방문 표시, 2.5 s memory | 버림: Kalman 다중 추적(과함) |
| **Target Approach** | A* → standoff(relax_goal, 가시선) → 근거리 IBVS(cx 중심, LiDAR 거리 기반 v) → 정지 1–2 s | – |
| **Return Home** | `time_left < SF(1.4)·ETA_home + 15 s` 트리거; known-only A* → dynamic 리셋 → 인플레이션 축소 → breadcrumb → unknown 허용; 도착 후 heading 정렬(필요 시) | m-explore-ros2식 1회 시도 |

## 4. [3] Reference별로 무엇을 참고했는가

| Reference | 참고한 것 |
|---|---|
| m-explore / m-explore-ros2 ✅ | frontier 정의·클러스터, 크기 필터, blacklist, 종료 조건, return_to_init 구조, resuming grace |
| hector_exploration_planner ✅ | 경로비용 기반 frontier(Exploration Transform), 벽 근접 danger 비용, **inner exploration → view frontier** |
| rrt_exploration ✅ | IG = 반경 내 unknown 넓이, hysteresis gain |
| GBPlanner ✅ | `exp(−λ·L)` 이득 감쇠, **시간 예산 homing 식**(+20 s 여유) |
| TARE / FUEL ✅ | 상태 전이 hysteresis, 회전을 포함한 시간 비용, ray-cast 가시 IG, 큰 frontier 분할 (TSP는 버림) |
| VLFM ✅ | sticky frontier, acyclic enforcer |
| SemExp / PONI ✅ | collision map, 방문 셀 통과 가능, goal 팽창, area+object potential 개념 |
| Nav2 ✅ | inflation 비용식, progress/goal checker, 1 Hz 재계획 BT, recovery RoundRobin, Spin/BackUp/Wait 충돌 검사, RPP, Collision Monitor |
| move_base ✅ | patience, oscillation 감지, 단계적 costmap 리셋 |
| CMU AEDE ✅ | 단순 heading P제어 path follower, path library(P2 arc sampler) |
| FAR planner ✅ | known-first→attemptable 폴백, goal 재평가, 관통 레이 기반 dynamic 분리, path momentum |
| Hector SLAM / OctoMap / slam_toolbox ✅ | log-odds 갱신 규칙·clamp, 스캔당 1회 갱신, CSM 구조와 odom prior |
| obstacle_detector ✅ | (P3) 스캔 클러스터·추적 파라미터 |
| Webots docs/samples, E-puck.proto ✅ | 동기화 위험, Lidar 순서/inf, 카메라 FOV, 센서 노이즈, Display/Recognition 제약 |
| Erebus ✅ | Webots 대회 심판 로직 사례(정지 1 s 식별, 20 s 정지 LoP) |
| webots_ros2 e-puck / TurtleBot3 ✅ | 같은/유사 로봇 튜닝값(반경·속도·허용오차·progress) |
| Gervet 2023, Macenski 2023 survey 📄 | 모듈형 구조와 Nav2 알고리즘 선택 근거 |

## 5. [4] 구현 순서 (요약 — 상세·수용 기준은 [11](11_IMPLEMENTATION_ROADMAP.md))

| Step | 내용 | Priority |
|---|---|---|
| S0 | 계측: 스텝 시간 로그, `getSynchronization`·센서 스펙 로그, 지도 덤프 주기화 | P0 |
| S1 | gyro heading fusion + 정지 bias 추정 | P0 |
| S2 | PathFollower(RPP-lite) + SafetyMonitor 강화 (+ opt-in `NAV_TEST` 모드) | P0 |
| S3 | planning 유틸: distance_field, 8-연결 옵션, relax_goal, smooth_path, path_is_valid | P0 |
| S4 | log-odds mapping (+ reset_region) | P0 |
| S5 | Navigator: 계획→추종→ProgressMonitor→기본 recovery(R0/R2/R3/R4) → `goto(goal)` | P0 |
| S6 | RETURN_HOME: 시간 예산 트리거 + 경로 폴백 MVP(known-only → breadcrumb → unknown 허용; 전체 5단계는 08 §2.3, S10에서 완성) | P0 |
| S7 | Exploration: frontier 선택(G1 + exp utility + hysteresis) + blacklist + 종료 | P0 |
| S8 | Target: NumPy HSV detector + TargetTracker + 2단계 접근 | P0 |
| S9 | Mission 통합 + Initial Active Scan | P0/P1 |
| S10 | P1 강화: camera coverage + view frontier, full ladder(R1 CLEAR, collision map), danger 비용, 동적 WAIT 정책, 오도메트리 보정 절차, 심사용 덤프 | P1 |
| S11 | P2: ray-cast IG, 카메라 IG를 frontier utility에, 접근 장애물 TTC, 조건부 CSM, look-around | P2 |

## 6. [5] 버릴 기능 (DO NOT IMPLEMENT)

ROS/ROS2 노드·토픽·액션·TF·BT 프레임워크 · Nav2 설치 · TSP/GTSP 전역 투어(TARE/FUEL/Kulich) · 3D 탐색(TARE/FUEL/GBPlanner 본체) · RRT 기반 frontier 탐지 · D* Lite/LPA*/JPS/ARA* · Hybrid-A*/State lattice · DWA/DWB(P3 이하) · TEB · MPC/MPPI · VFH/APF 단독 · VO/RVO/ORCA · pose graph/loop closure · AMCL · KISS-ICP 등 새 SLAM 의존성 · 학습 탐색(SemExp/PONI/VLFM 네트워크) · GPU DNN · Webots Recognition/Supervisor/GPS를 대회 컨트롤러에 사용 · 다중 로봇 · 전체 temporal decay 지도.

## 7. [6] 대회 당일 변경 가능성이 높은 부분 (모두 config.py)

| 항목 | 이유 | 확인 방법 |
|---|---|---|
| DEVICE_NAMES, 로봇 기하(바퀴 반경, 축간), 최대 속도 | 공식 로봇 | devices 시작 로그, proto |
| LiDAR 순서/FOV/maxRange/mount offset/noise | 공식 센서 | 360° 스캔을 알려진 벽에 대고 확인 |
| 카메라 FOV·해상도·mount | bearing/검출 거리 | 시작 로그 |
| target HSV/크기, "도착" 기준 거리·정지 시간 | 당일 제공 | 샘플 이미지, 규정 |
| **`REQUIRED_TARGETS`** (찾아야 할 target 수; `None` = 모름) | 공식 계획안은 "지정된 대상/목표 물체"로만 표기 → 개수 미확정 | 규정. `N`이면 N개 방문 즉시 RETURN_HOME, `None`이면 탐색·view frontier 소진 또는 시간 예산까지 계속 |
| 시간 제한, 시간 기준(시뮬/실시간), synchronization | 복귀 트리거 | 규정·로그 |
| START_POSE, 맵 크기(GRID_WIDTH/HEIGHT) | 시작 위치 제공 | 규정 |
| 안전거리·인플레이션 | 로봇 반경 | config |
| 허용 센서(GPS/Compass/Recognition) | 규정 | 규정 |

## 8. [7] 최소 완주 버전 (MVP)과 [8] 상위 버전

| 버전 | 포함 | 기대 |
|---|---|---|
| **M0 Walking Skeleton** (팀 리뷰 반영, 2026-09-30) | 각 단계의 **가장 단순한 버전으로 끝까지 한 번 도는 것**: encoder odom + binary grid(단, `_mark_free`가 OCCUPIED도 비우게 = Nav2식 raytrace clearing 1줄) + 거리장 최근접 도달가능 frontier + A* + 단순 follower + 기존 e-stop + 단일 프레임 검출·접근 + 고정 시간 여유 복귀 | 통합 위험을 가장 먼저 제거 |
| **MVP** | M0 위에 교체/추가 (S0–S9): gyro fusion, RPP-lite, safety, log-odds, navigator(기본 recovery), frontier(G0/G1)+blacklist, HSV+M-of-N+접근, 시간 예산 복귀 | 충돌 없이 탐색·target 접근·복귀 |
| **Stable** | + full recovery ladder, danger 비용, 동적 WAIT 정책, 8-연결+smoothing, 오도메트리 보정 절차, 스텝 시간 예산 | 끼임/사람/ghost에도 완주 |
| **Competitive** | + camera coverage·view frontier, Initial Active Scan, exp utility + hysteresis 튜닝, 심사용 덤프(지도+경로+target) | target 누락 감소, 설명 가능한 기술 |
| **Stretch** | + ray-cast IG, TTC 접근 감지, 조건부 CSM, look-around, arc sampler | 차별화 |

## 9. [9] Baseline 파일 1:1 매핑

| 파일 | 추가/변경 |
|---|---|
| `controllers/rescue_robot/config.py` | 모든 새 파라미터 (log-odds, 주기, follower, safety, progress, recovery, frontier, blacklist, target, 시간 예산), 새 opt-in 모드 이름 |
| `interfaces.py` | **변경 없음** (target world 위치를 공유 규격으로 만들 경우만 사람 확인 후 INTERFACES.md·tests와 동시 수정) |
| `devices.py` | 시작 로그(synchronization, lidar minRange, camera FOV) 추가 정도 |
| `mapping.py` | `OccupancyGrid` 내부 log-odds + export, `reset_region`, `CameraCoverageGrid`, 덤프(색상 PPM) |
| `localization.py` | gyro fusion, bias 추정, slip 플래그, (P2) `ScanMatcher` |
| `planning.py` | `distance_field`, `obstacle_distance`, `astar(connectivity=)`, `relax_goal`, `smooth_path`, `path_is_valid`, `select_frontier`/`estimate_information_gain`/`score_frontier`, `FrontierBlacklist`, `select_view_frontier`, `eta_home` |
| `control.py` | `PathFollower`, `SafetyMonitor`, `ProgressMonitor`, spin/backup primitive |
| `navigation.py` (신규) | `Navigator` (goal 실행 + recovery ladder) |
| `detection.py` | NumPy HSV detector(고정 target dict 반환), `TargetTracker` |
| `main.py` | 상태기계 보강, 스케줄러, TimeBudget, 로그/덤프 |
| `tests/` | 모듈별 신규 테스트 (11 참고) |

## 10. [10] 첫 구현 작업 prompt 초안 → [11 §6](11_IMPLEMENTATION_ROADMAP.md#6-claudecodex-작업-prompt-초안)

---

## 11. 가장 중요한 질문에 대한 답 (Q1–Q15)

**Q1. 가장 먼저 넣을 기능 5개** — ① RPP-lite path follower + SafetyMonitor 강화 ② log-odds mapping (ghost 제거) ③ 탐색 루프: 거리장 frontier 선택 + ProgressMonitor + blacklist ④ target 파이프라인(M-of-N, bearing+LiDAR 위치, 2단계 접근) ⑤ 시간 예산 RETURN_HOME. (+ 30분짜리 gyro heading fusion은 ①과 함께) — 구현 **순서**는 의존성 때문에 §5(S0→S9)를 따른다.

**Q2. 상위권을 가르는 요소** — (1) 충돌·끼임 0회의 주행 안정성(안전 모니터 + recovery ladder), (2) 카메라 coverage를 고려한 수색(LiDAR만 보는 팀은 target을 놓침), (3) target world 위치 정확도와 중복 처리, (4) 시간 예산으로 **항상 복귀 성공**, (5) 지도·경로·target을 보여주는 산출물로 기술 설명력.

**Q3. 가장 위험한 실패 지점 5개** — (1) 사람/벽 근처 회전·접근 중 충돌, (2) stuck/진동 루프로 시간 소진(대회 LoP 위험), (3) ghost 장애물·drift로 home 경로 소실, (4) 카메라 사각으로 target 누락 또는 오탐 접근, (5) 비동기 컨트롤러에서 계산 지연 → 제어 지연.

**Q4. A\* 유지?** — 예. 실측 50 ms(160×160)로 1 Hz 재계획에 충분. 8-연결+octile(−16% 경로), 약한 danger 비용, LOS smoothing, 다중 목표용 거리장만 추가 (04).

**Q5. Frontier 방식으로 충분?** — 지도 작성에는 충분(2D 소규모에서 TSP 계층형 이득 ≤12.5% 📄 Kulich 2019). **수색에는 불충분** → view frontier(카메라 미탐색)와 target 선점 규칙을 추가 (02 §3.4).

**Q6. IG 정의** — MVP: 클러스터 크기(m). Stable: 대표 셀 반경 R 안 UNKNOWN 넓이(rrt_exploration ✅). Competitive: + 카메라 미탐색 셀 가중. Stretch: ray-cast로 보이는 것만(FUEL ✅). 상위 K개 후보만 평가.

**Q7. Scan matching 가치?** — **조건부**. gyro fusion 후 practice world에서 GPS debug로 8분 drift를 측정해 ≥ 1셀(5 cm)이면 numpy CSM(P2), 아니면 넣지 않는다. pose graph/loop closure는 넣지 않는다.

**Q8. Dynamic obstacle 최단·안정 처리** — raw-scan 안전 모니터(항상) + log-odds 지도(자동 소멸) + "새 장애물이면 WAIT 4 s → 재계획 → recovery" 정책. 추적은 하지 않는다.

**Q9. DWA 가치?** — 없음(P3 이하). RPP-lite + 안전 모니터 + 1 Hz 재계획이 더 단순·안정. 동적 회피가 부족하면 P2로 CMU식 소수 arc 샘플러.

**Q10. GPU?** — NOT RECOMMENDED (13).

**Q11. Return Home 설계** — 시간 예산 트리거(GBPlanner 식), known-only 계획 → 폴백 5단계(08 §2.3), navigator의 progress/recovery 재사용, 도착 후 heading 정렬(필요 시), 실패 시에도 1 Hz 재시도·안전 정지.

**Q12. Initial Active Scan 가치?** — 있음. 단 이유는 **카메라**(48°)다. LiDAR는 360°라 지도 이득은 미미. 5–8 s로 시작점 주변 target 확인 + gyro bias 추정을 같이 얻는다 (Case 1).

**Q13. target 접근** — A*로 standoff까지 → 근거리 이미지 기반 visual servo(cx 중심, LiDAR 거리로 속도) → 정지 1–2 s. 가림 시 memory → world 위치 계획 접근 (07 §7–8).

**Q14. 절대 구현하지 말 것** — §6 목록. 특히 ROS 이식, TSP 투어, D* Lite, TEB/MPC, 학습 탐색, GPU, 대회 컨트롤러의 GPS/Supervisor/Recognition 사용.

**Q15. 현재 architecture의 잘못되거나 위험한 가정**
1. `mapping._mark_free`: OCCUPIED 영구 → dynamic ghost 영구 (03).
2. 모든 파이프라인을 매 스텝 같은 흐름으로 돌리는 구조 + 스텝 시간 예산 없음 → 비동기 모드에서 위험 (09).
3. LiDAR coverage = 수색 완료라는 암묵 가정 (02 §3.4).
4. 안전 훅이 전방 ±30° 최소거리 1점, 후진 무검사, 트리거 시 전진만 0 → 회전만 반복하는 교착 가능 (05 §3).
5. `astar(allow_unknown=True)` 기본값 → RETURN_HOME이 unknown을 통과하는 경로를 택할 수 있음 (04 §3.3).
6. frontier 셀을 raw grid에서 뽑지만 인플레이션 후 도달 불가일 수 있음 → goal relaxation 부재 (02 §2.4).
7. 고정 `MISSION_TIME_LIMIT` → 거리 무관 복귀 시점 (08).
8. RECOVERY·재탐색(다중 target 후 EXPLORE 재개)·view 탐색 단계 부재; `found_targets`에 world 위치 추정 경로 없음 (07).
9. `LIDAR_MOUNT_OFFSET (0,0)` vs world 0.0095 m, 카메라 mount 가정 — 공식 로봇 재확인 (09).
10. 시간 기준(시뮬 vs 실시간)·허용 센서를 규정으로 확인하지 않은 상태.
