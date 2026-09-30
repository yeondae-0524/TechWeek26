# 02. Autonomous Exploration & Frontier Selection

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> 범례: ✅ = 이번 조사에서 **소스 코드까지 직접 확인** · 📄 = 논문/문서만 확인(아이디어 참고) · ⚠️ = 확인 못 함/추정
> 소스 확인은 2026-09-29~30, 각 repo의 기본 브랜치 HEAD 기준 (commit은 [01_REFERENCE_MATRIX.md](01_REFERENCE_MATRIX.md) 참고).

---

## 0. 한 줄 결론

**"로봇에서 도달 가능한 frontier만, 실제 경로 거리 × 정보이득으로 점수 매기고, 현재 goal에 hysteresis를 주고,
진전이 없으면 blacklist, 고를 게 없으면 (카메라 미탐색 영역 → 복귀)"** 가 우리 규모에서 가장 비용 대비 효과가 좋다.
TSP 기반 전역 투어(TARE/FUEL/Kulich)는 우리 규모에서 이득이 작고(≤12.5%, 사무실형 맵에선 오히려 -4.5%) 구현 위험이 커서 제외.

---

## 1. 비교한 구현체 (소스 기준)

| 구현 | Frontier 정의 | 클러스터/대표점 | 점수(utility) | 거리 | IG 정의 | Hysteresis | 실패 처리 |
|---|---|---|---|---|---|---|---|
| **m-explore / m-explore-ros2** ✅ | unknown 셀 & 4-이웃에 FREE | 8-연결 BFS, goal=centroid | `cost = ps·d·res − gs·size·res` (작을수록 좋음) | **Euclidean** (가장 가까운 frontier 셀) | frontier 셀 수 | 없음 (1 cm 동일성 비교만) | progress timeout 30 s → blacklist(±5셀 사각형, 영구) |
| **frontier_exploration (paulbovbel)** ✅ | 위와 동일 (m-explore의 원형) | travel_point = closest/middle/centroid 선택 | 가장 가까운 점 선택 | Euclidean | 없음 | 없음 | goal ABORTED 이력 box blacklist |
| **hector_exploration_planner** ✅ | FREE & 인접 UNKNOWN (+min_frontier_size 5) | frontier 셀 전체를 **다중 goal** | **Exploration Transform**: 모든 frontier에서 역방향 wavefront, 비용 = 이동 + `cellDanger`(벽 근접 제곱 페널티) | **실제 경로 비용 (8-연결)** | 없음(도달 비용만) | (각도 페널티 코드는 `if(false)`로 **비활성**) | frontier 없으면 **inner exploration**(지나온 궤적에서 가장 먼 FREE 셀) |
| **rrt_exploration** ✅ (Python) | RRT로 탐지 → mean-shift 클러스터 | centroid | `revenue = m·IG·(h if d<r_h) − d` (클수록 좋음) | Euclidean (`pathCost()` 함수는 있지만 assigner는 `norm` 사용) | **반경 r 안 unknown 넓이 [m²]** | `hysteresis_gain=2.0` (반경 3 m 내 후보 IG ×2) | – |
| **GBPlanner (NTNU, SubT CERBERUS)** ✅ | 3D voxel 기반 | graph vertex | `Σ gain(v)·exp(−λ·dist(v))` × `exp(−λ2·방향이탈)` | 그래프 최단거리 | ray-model로 본 unknown voxel 수 × 60 | 탐사 방향 일관성 페널티 | 시간예산 homing (08 참고) |
| **TARE (CMU, SubT Explorer)** ✅ | 3D coverage surface + frontier | viewpoint 샘플링 | local: coverage 수 기준 greedy+TSP, global: subspace TSP | 그래프/TSP 경로 | 새로 덮는 surface point 수 | cell 상태 전이 임계 (EXPLORING→COVERED 1, COVERED→EXPLORING 10) | 모든 subspace COVERED → 귀환 |
| **FUEL (HKUST)** ✅ | 3D voxel, 증분 갱신 | 큰 frontier 분할, viewpoint | global ATSP + local refine | A* 경로길이/v_max vs yaw변화/ω_max 중 **max** (시간 비용) | viewpoint에서 **ray-cast로 보이는** unknown 수 | – | – |
| **VLFM (BDAI)** ✅ | 2D frontier | midpoint | vision-language value map | – | 학습 모델 값 | **sticky**: 이전 frontier가 0.5 m 이내 남아 있고 값이 거의 안 떨어지면 유지 + `AcyclicEnforcer`(같은 (위치,frontier) 반복 금지) | – |
| **PONI / SemExp** ✅ | map 기반 | 큰 frontier 5개만, 로봇 주변 마스킹 | 학습된 potential(area + object) | FMM 거리 | 학습 예측 | – | SemExp: collision map (06 참고) |

소스 위치(먼저 볼 파일):

- m-explore: [`explore/src/frontier_search.cpp`](https://github.com/hrnr/m-explore/blob/noetic-devel/explore/src/frontier_search.cpp) (`searchFrom`, `buildNewFrontier`, `isNewFrontierCell`, `frontierCost`), [`explore/src/explore.cpp`](https://github.com/hrnr/m-explore/blob/noetic-devel/explore/src/explore.cpp) (`makePlan`, `goalOnBlacklist`, `reachedGoal`)
- m-explore-ros2: [`explore/src/explore.cpp`](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp) (`makePlan`, `returnToInitialPose`, `stop`, `resume`)
- frontier_exploration: [`frontier_exploration/src/frontier_search.cpp`](https://github.com/paulbovbel/frontier_exploration/blob/melodic-devel/frontier_exploration/src/frontier_search.cpp), [`exploration_server/src/exploration_server.cpp`](https://github.com/paulbovbel/frontier_exploration/blob/melodic-devel/exploration_server/src/exploration_server.cpp)
- hector: [`hector_exploration_planner/src/hector_exploration_planner.cpp`](https://github.com/tu-darmstadt-ros-pkg/hector_navigation/blob/melodic-devel/hector_exploration_planner/src/hector_exploration_planner.cpp) (`doExploration`, `buildexploration_trans_array_`, `buildobstacle_trans_array_`, `cellDanger`, `getTrajectory`, `doInnerExploration`, `findInnerFrontier`), 파라미터 [`cfg/ExplorationPlanner.cfg`](https://github.com/tu-darmstadt-ros-pkg/hector_navigation/tree/melodic-devel/hector_exploration_planner/cfg)
- rrt_exploration: [`scripts/assigner.py`](https://github.com/hasauino/rrt_exploration/blob/master/scripts/assigner.py), [`scripts/functions.py`](https://github.com/hasauino/rrt_exploration/blob/master/scripts/functions.py) (`informationGain`, `discount`)
- GBPlanner: [`gbplanner/src/rrg.cpp`](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner3/gbplanner/src/rrg.cpp) (path gain 루프 ~L2436, `homingRequired`, `isRemainingTimeSufficient`)
- TARE: [`sensor_coverage_planner_ground.cpp`](https://github.com/caochao39/tare_planner/blob/melodic-noetic/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp) (`execute`), [`grid_world.cpp`](https://github.com/caochao39/tare_planner/blob/melodic-noetic/src/tare_planner/src/grid_world/grid_world.cpp) (`UpdateCellStatus`), [`local_coverage_planner.cpp`](https://github.com/caochao39/tare_planner/blob/melodic-noetic/src/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp) (`EnqueueViewpointCandidates`, `SelectViewPoint`), [`config/indoor.yaml`](https://github.com/caochao39/tare_planner/blob/melodic-noetic/src/tare_planner/config/indoor.yaml)
- FUEL: [`frontier_finder.cpp`](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/active_perception/src/frontier_finder.cpp) (`findViewpoints`, `splitLargeFrontiers`), [`graph_node.cpp`](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/active_perception/src/graph_node.cpp) (`ViewNode::computeCost`)
- VLFM: [`vlfm/policy/itm_policy.py`](https://github.com/bdaiinstitute/vlfm/blob/main/vlfm/policy/itm_policy.py) (`_get_best_frontier`), [`vlfm/policy/utils/acyclic_enforcer.py`](https://github.com/bdaiinstitute/vlfm/blob/main/vlfm/policy/utils/acyclic_enforcer.py)

### 1.1 문서와 코드가 다른 지점 (코드 기준 판단)

| 구현 | 문서/직관 | 실제 코드 |
|---|---|---|
| m-explore | `orientation_scale` 파라미터 존재 | 읽기만 하고 **어떤 cost에도 사용 안 함** (ROS1/ROS2 모두) |
| m-explore | "information gain" | frontier **경계 셀 개수**일 뿐 |
| m-explore | distance cost | 이미 미터인 거리에 `resolution`을 한 번 더 곱함 → 실효 거리가중치 = `potential_scale × res` (launch값 3.0 × 0.05 = 0.15/m vs gain 1.0/m) → **gain이 압도** |
| m-explore | centroid | 초기 셀이 `size`에는 포함되지만 합계엔 빠짐 → `(size−1)/size` 배 원점 쪽 편향 (코드 추론) |
| hector | goal angle penalty 파라미터(`goal_angle_penalty=50`) | `buildexploration_trans_array_`에서 `if(false)`로 **비활성** |
| rrt_exploration | path cost 기반처럼 보임 (`pathCost()` 존재) | assigner는 **Euclidean norm** 사용; `discount()` 주석에 "셀 면적이 아니라 1을 빼는 버그" 자인 |
| VLFM | "All frontiers are cyclic. Just choosing the **closest** one." | `max(..., key=norm)` → 실제로는 **가장 먼** frontier 선택 (코드 버그) |

→ 교훈: **README/파라미터 이름을 믿지 말고, 우리 구현은 unit test로 의도를 고정**해야 한다.

---

## 2. Frontier Detection — 무엇을 가져올까

### 2.1 정의: "FREE 쪽 frontier" 유지 (우리 baseline 방식)

- m-explore/frontier_exploration은 **UNKNOWN 쪽** 셀을 frontier로 삼고, hector와 우리 baseline은 **FREE 쪽**.
- FREE 쪽 정의는 frontier 셀 자체가 주행 가능 후보라 **goal로 바로 쓸 수 있다**. → 유지.

### 2.2 도달 가능성 필터: m-explore의 "연결 영역 BFS" 아이디어를 거리장으로 확장

- m-explore `searchFrom`은 로봇 근처 FREE에서 BFS하여 **로봇과 연결된 영역의 경계만** frontier로 만든다 → 닫힌 방 너머 frontier는 애초에 생성 안 됨.
- hector Exploration Transform은 한 걸음 더: **frontier 전체를 multi-source로 두고 wavefront**를 퍼뜨려 모든 셀의 "가장 좋은 frontier까지의 비용"을 구한 뒤, 로봇 셀에서 **gradient descent**로 경로를 뽑는다 (`getTrajectory`).
- 우리 권장: **로봇 셀에서 inflated grid 위 BFS/Dijkstra 거리장 1회** → 모든 frontier 셀의 실제 경로 거리 + 도달 불가(∞) 판정 + parent로 경로 복원까지 한 번에.
  - 실측 (이 PC, Python 3.10, 160×160, 2026-09-30): BFS 거리장 dict 구현 24 ms, **flat list 구현 12 ms**; A* 코너→코너 49 ms. (벤치 스크립트: 세션 scratchpad `bench.py`)
  - 즉 **frontier마다 A*를 돌리는 것(수십×49 ms)보다 거리장 1회가 압도적으로 싸다.**

### 2.3 작은 frontier 제거 (노이즈)

| 구현 | 기준 |
|---|---|
| m-explore | `size × res ≥ min_frontier_size` (코드 기본 0.5 m, launch 0.75 m) |
| hector | `min_frontier_size = 5` 셀 |
| PONI | 가장 큰 5개만 유지, 로봇 주변 1 m 마스킹 |
| FUEL | `cluster_min=100` voxel, 큰 frontier는 `cluster_size_xy=2.0 m`로 분할 |

→ 우리(TB3 반경 0.105 m [OFFICIAL], 외접 약 0.110 m [DERIVED], 0.05 m/셀 [INITIAL TUNING]): **`FRONTIER_MIN_CELLS = 4~6` (0.2~0.3 m)** 로 시작(initial tuning suggestion).
클러스터 길이는 통로 폭이 아니다. 0.16 m 인플레이션 후 도달성을 별도 판정한다. 너무 크면 좁은 틈 뒤 공간을 놓친다. 추가로 **inflated grid에서 도달 가능한 셀이 0개인 클러스터는 제거**.

### 2.4 대표점 (goal cell)

| 선택지 | 출처 | 장단점 |
|---|---|---|
| centroid | m-explore(goal), frontier_exploration 옵션 | 호/L자 frontier에서 **벽·unknown 위에 떨어질 수 있음** → ABORTED → blacklist 악순환 |
| closest cell | frontier_exploration `closest`, m-explore `middle`(미사용) | 항상 가까움, 그러나 큰 frontier 가장자리만 핥는 경향 |
| midpoint of contour | PONI | 안정적이지만 도달성 보장 없음 |
| multi-goal (전체 셀) | hector | 가장 견고: "가장 싸게 닿는 frontier 셀"이 자동 선택 |

→ 우리: **클러스터 셀 중 (a) inflated grid에서 도달 가능하고 (b) centroid에 가장 가까운 셀**. 없으면 클러스터 제외.
(대안: 거리장 최소 셀 = hector식. 두 방식 모두 unit test로 비교 가능)

---

## 3. Frontier Scoring — "어느 frontier가 제일 좋은가?"

### 3.1 문헌/구현의 utility 형태

| 형태 | 식 | 출처 | 특성 |
|---|---|---|---|
| 가산형 (cost) | `c = a·d − b·G` | m-explore ✅, Burgard 2005 📄 (`U − β·cost`) | 단위 스케일에 민감, 가중치 튜닝 필요 |
| 가산형 (revenue) | `r = m·G·h − d` | rrt_exploration ✅ | 동일 |
| 지수 감쇠형 | `u = G·exp(−λ·L)` | González-Baños & Latombe 2002 📄, NBVP(Bircher 2016) 📄, GBPlanner ✅ | **거리 감쇠 길이를 해석하기 쉬움**: λ = "몇 m 가면 가치가 반으로" (`ln2/λ`) 로 직관적 튜닝 |
| 시간 비용형 | `t = max(L/v, Δψ/ω)` | FUEL ✅ | 회전 비용을 시간으로 통합 (드론은 이동 중 yaw 가능 → max, **차동구동 제자리 회전은 합(+)**) |
| 순수 거리 | nearest | Yamauchi 1997 📄, frontier_exploration ✅ | 가장 단순. Holz 2010 📄 평가에서 room-aware 등 개선 여지 보고 |
| 다기준(MCDM) | Choquet integral 등 | Basilico & Amigoni 2011 📄 (S&R 대상) | 이론적, 해커톤엔 과함 |
| 전역 투어 | TSP/GTSP over goals | TARE ✅, FUEL ✅, Kulich 2019 📄 | Kulich 2019: 최대 +12.5% (저장애물), 사무실형에선 −4.5% |

### 3.2 m-explore 식을 우리 식으로 바꿀 때의 평가

사용자가 제안한 `score = α·path_cost − β·information_gain` 은 **방향은 맞다**. 다만:

1. **path_cost는 Euclidean이 아니라 거리장(= A*와 동일한 최단거리)** 을 써야 "벽 반대편 frontier" 문제가 해결된다. 추가 계산비용은 거리장 1회(≈12 ms)뿐.
2. 가산형은 G와 L의 단위(m² vs m) 스케일이 맵 크기에 따라 흔들린다 → **지수형 `u = G·exp(−λL)`** 을 권장 (GBPlanner/NBVP와 동일 계열). 튜닝: `λ = ln 2 / L_half`, 예) L_half = 1.0 m (TB3 운용 시작값 0.15 m/s → 약 6.7 s 거리) — initial tuning suggestion.
3. **회전 비용**(m-explore엔 없음): 차동구동은 목표 방향으로 제자리 회전 후 출발하므로 `L_eff = L + (v/ω)·|Δψ|` ([INITIAL TUNING] `0.15/1.5 = 0.10 m/rad`) 로 거리 환산. 180°가 약 0.31 m — **작지만 동률 frontier 사이의 진동을 줄이는 tie-breaker** 로 유용.
4. **hysteresis (필수)**: m-explore에는 없음(→ goal thrashing). 가져올 곳:
   - rrt_exploration: 현재 위치 3 m 이내 후보 IG × 2.0
   - VLFM: 직전 frontier가 0.5 m 내 여전히 존재하고 가치가 크게 안 떨어지면 유지 + 같은 (위치, frontier) 재선택 금지(`AcyclicEnforcer`)
   - FAR: `path_momentum_thred = 3` (경로 전환에 연속 3회 확인 필요)
   - TARE: 셀 상태 전이 임계 비대칭(1 vs 10)
   → 우리: **현재 goal의 utility에 ×1.3~1.5 보너스 + 새 후보가 이기려면 연속 2~3회 승리** (initial suggestion).

### 3.3 Information Gain — 현실적 정의 (Q6)

| 레벨 | 정의 | 비용 | 출처 | 권장 |
|---|---|---|---|---|
| G0 | 클러스터 셀 수 × res (경계 길이) | 0 | m-explore | 비교용 단순안 |
| G1 | 대표 셀 반경 R 안 UNKNOWN 셀 수 × res² | O(R²) per 후보 | rrt_exploration `informationGain` | **MVP 기본값** (R = LiDAR 유효거리 or 0.5~1.0 m) |
| G2 | 대표 셀에서 N개 ray-cast로 **보이는** UNKNOWN 수 (벽에서 ray 중단) | O(N·R) | FUEL `findViewpoints`, hector `getYawToUnknown` | Competitive |
| G3 | **카메라 미탐색(unsearched) 셀** 중 카메라 FOV·거리 안에 보이는 수 | O(N·R) | (우리 S&R 확장; TARE coverage 아이디어) | **S&R 차별화 핵심** |

- 계산량 제어: 거리장으로 **상위 K개(5~8) 후보만** G1/G2 평가 (PONI도 상위 5개만 유지).
- G1은 벽 너머 unknown도 세므로 과대평가 → G2로 개선. 해커톤에서는 G1 → 시간 남으면 G2.

### 3.4 우리 S&R 특유: "LiDAR로 탐색됨 ≠ 카메라로 수색됨"

- [OFFICIAL] LDS-01은 360 samples, 약 360°, 0.12–3.5 m, 공식 camera는 **640×480, 약 60° HFOV** ([09](09_WEBOTS_REFERENCES.md)). 이전 [E-puck.proto R2025a](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto)는 practice/reference only. 전방 약 1/6만 보므로 camera coverage는 여전히 필요하다.
- 따라서 LiDAR frontier가 다 사라져도 **카메라가 한 번도 보지 않은 벽/구석**이 남는다 → target 누락.
- hector의 **inner exploration**(frontier 소진 시 "지나온 궤적에서 가장 먼 도달 가능 셀"로 이동)이 바로 이 문제의 고전적 해법.
- 우리 권장: `camera_seen[row][col]` 보조 grid(OCCUPIED/FREE 셀을 카메라 frustum·거리 한계·가림 ray-cast로 표시)를 유지하고,
  1. EXPLORE 1단계: LiDAR frontier (G1/G2 + G3 가중)
  2. EXPLORE 2단계(frontier 소진 후): **view frontier** = "카메라 미탐색 FREE 바닥 또는 OCCUPIED 표면을 볼 수 있는 도달 가능한 FREE 셀" 중 utility 최대
  3. 그래도 없으면 RETURN_HOME

---

## 4. 진전 감시 · Blacklist · 종료 (상세는 06, 08)

| 항목 | m-explore | Nav2 | 우리 권장 |
|---|---|---|---|
| 진전 판정 | frontier까지 Euclidean 최소거리 감소 여부, 30 s | 로봇이 `required_movement_radius`(0.5 m) 이상 이동했는지, `movement_time_allowance` 10 s ([SimpleProgressChecker](https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp)) | **이중**: (a) 경로 잔여거리 best-so-far가 δ(0.05 m) 이상 감소 (b) 변위 반경 (우리 0.15 m/10 s [INITIAL TUNING], 외부 TB3 Nav2 0.1 m/10 s [REFERENCE]) |
| blacklist | centroid, ±5셀 사각형, 영구 | – | goal 셀 기준 반경 0.30 m, **TTL 90 s** [INITIAL TUNING], 전부 막히면 1회 초기화(second chance) |
| 종료 | frontier 없음 / 전부 blacklist | – | + target 모두 발견 + **시간 예산** + view frontier 소진 |

---

## 5. 추천 알고리즘 (우리 버전)

```text
select_exploration_goal(grid, pose, state, now):
    infl  = inflate(grid, r_robot + margin)            # 이미 있음
    dist, parent = distance_field(infl, robot_cell)    # BFS 4-연결 (or 8-연결 Dijkstra), flat list
    clusters = cluster_frontiers(find_frontiers(grid), min_size=FRONTIER_MIN_CELLS)
    cands = []
    for cl in clusters:
        reach = [c for c in cl if dist[c] < INF]
        if not reach: continue
        goal = argmin_{c in reach} |c - centroid(cl)|
        if blacklist.contains(goal, now): continue
        L = dist[goal]*res + (v/ω)*|heading_change(pose, goal)|
        cands.append((goal, cl, L))
    top = sorted(cands, key=L)[:K]                      # 상위 K개만 IG 평가
    for c in top:
        G = IG_lidar(grid, c.goal) + w_cam * IG_camera(camera_seen, grid, c.goal)
        u = G * exp(-λ * L)
        if near(c.goal, state.current_goal): u *= HYST_BONUS
    best = argmax u  (단, 현재 goal과 다르면 N회 연속 승리해야 교체)
    return best or None      # None → view-frontier 단계 → RETURN_HOME
```

## 6. 이 문서의 결론을 검증할 테스트 (tests/, Webots 불필요)

| 테스트 | 기대 |
|---|---|
| 벽 반대편이 Euclidean으로 더 가깝지만 경로가 긴 frontier | 경로가 짧은 다른 frontier 선택 |
| 닫힌 방 안 frontier | 후보에서 제외 (dist = ∞) |
| 크기 < MIN인 클러스터 | 제외 |
| 선택 goal 셀 | inflated grid에서 FREE, 원 클러스터 소속 |
| hysteresis | 점수 차 < 보너스면 현재 goal 유지, N회 연속 승리 시 교체 |
| blacklist | 반경 내 후보 skip, TTL 후 복귀, 전부 blacklist → second chance 1회 |
| IG G1 | 알려진 반경 내 unknown 셀 수와 일치 |
| camera coverage | frustum 안·거리 안·가림 없는 셀만 seen 처리 |
| frontier 없음 | None |

## 7. 참고 문헌 (탐색)

- Yamauchi, "A frontier-based approach for autonomous exploration," CIRA 1997. [doi:10.1109/CIRA.1997.613851](https://doi.org/10.1109/CIRA.1997.613851) 📄
- González-Baños & Latombe, "Navigation strategies for exploring indoor environments," IJRR 2002. [doi:10.1177/0278364902021010834](https://doi.org/10.1177/0278364902021010834) 📄
- Burgard et al., "Coordinated multi-robot exploration," T-RO 2005. [doi:10.1109/TRO.2004.839232](https://doi.org/10.1109/TRO.2004.839232) 📄
- Wirth & Pellenz, "Exploration Transform: A stable exploring algorithm for robots in rescue environments," SSRR 2007. [doi:10.1109/SSRR.2007.4381274](https://doi.org/10.1109/SSRR.2007.4381274) 📄 (구현 ✅ = hector_exploration_planner)
- Holz, Basilico, Amigoni, Behnke, "Evaluating the efficiency of frontier-based exploration strategies," ISR/ROBOTIK 2010. [PDF](https://www.ais.uni-bonn.de/papers/ISR_Robotik_2010_Holz_Exploration.pdf) 📄
- Basilico & Amigoni, "Exploration strategies based on multi-criteria decision making for searching environments in rescue operations," Auton. Robots 2011. [doi:10.1007/s10514-011-9249-9](https://doi.org/10.1007/s10514-011-9249-9) 📄 (초록만 확인; 세부 기준은 ⚠️ 미확인)
- Juliá, Gil, Reinoso, "A comparison of path planning strategies for autonomous exploration and mapping of unknown environments," Auton. Robots 2012. [doi:10.1007/s10514-012-9298-8](https://doi.org/10.1007/s10514-012-9298-8) 📄
- Keidar & Kaminka, "Efficient frontier detection for robot exploration," IJRR 2014. [doi:10.1177/0278364913494911](https://doi.org/10.1177/0278364913494911) 📄 (WFD/FFD — old practice-PC 160×160에서 약 6 ms; 큰 grid 비용은 04에서 별도 검토)
- Bircher et al., "Receding Horizon Next-Best-View Planner for 3D Exploration," ICRA 2016. [doi:10.1109/ICRA.2016.7487281](https://doi.org/10.1109/ICRA.2016.7487281) 📄
- Umari & Mukhopadhyay, "Autonomous robotic exploration based on multiple rapidly-exploring randomized trees," IROS 2017. [doi:10.1109/IROS.2017.8202319](https://doi.org/10.1109/IROS.2017.8202319) (구현 ✅)
- Kulich, Kubalík, Přeučil, "An Integrated Approach to Goal Selection in Mobile Robot Exploration," Sensors 2019. [doi:10.3390/s19061400](https://doi.org/10.3390/s19061400) · [arXiv:2007.10085](https://arxiv.org/abs/2007.10085) 📄
- Dang et al., "Graph-based subterranean exploration path planning using aerial and legged robots," JFR 2020. [doi:10.1002/rob.21993](https://doi.org/10.1002/rob.21993) (구현 ✅)
- Zhou et al., "FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning," RA-L 2021. [doi:10.1109/LRA.2021.3051563](https://doi.org/10.1109/LRA.2021.3051563) (구현 ✅)
- Cao et al., "TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments," RSS 2021; "Representation granularity enables time-efficient autonomous exploration in large, complex worlds," Science Robotics 2023. [doi:10.1126/scirobotics.adf0970](https://doi.org/10.1126/scirobotics.adf0970) (구현 ✅)
- Yokoyama et al., "VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation," ICRA 2024. [arXiv:2312.03275](https://arxiv.org/abs/2312.03275) (구현 ✅, 학습 부분은 DO NOT USE)
