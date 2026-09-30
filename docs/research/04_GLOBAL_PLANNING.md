# 04. Global Planning

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 태그: [OFFICIAL] [DERIVED] [MEASURED] [REFERENCE] [INITIAL TUNING] [DAY-OF].
> **2026-09-30 공식 TECH WEEK repo 기준 재검증.** 모든 실측은 **이 개발 PC**(Windows 11, Intel64 Family 6 Model 186, Python 3.10.11, 순수 Python, 우리 baseline 함수)이며 **공식 로봇/월드 실행 성능이 아니다.** 스크립트: 세션 scratchpad `bench.py`·`bench2.py`(old, 160×160), `bench3.py`(320/480, 연산별 5/7/15회 median/소표본 p95).

---

## 0. 결론 (Q4 / Q6)

**A\*를 유지한다.** 공식 notebook도 Global Planner로 **Dijkstra와 A\***를 가르치며 예제는 **4-연결·Manhattan** A*다 [OFFICIAL 교육]. 우리 baseline(4-연결 A*)은 이와 같고, 아래는 **우리 개선안**이다(공식 요구 아님).

1. **8-연결 + octile heuristic 옵션** (corner-cutting 금지): 경로 길이 −16% (14.50 m → 12.13 m, old 160×160 벤치), 시간 거의 동일.
2. **BFS/Dijkstra 거리장**: 다중 목표 질의(frontier 순위, home ETA)에 1회로 대응.
3. **벽 근접 비용**(Hector `cellDanger` / Nav2 inflation cost 개념): 약하게 시작.
4. **Line-of-sight smoothing**: 셀 수백 개 → waypoint 10개 내외, follower 안정화.
5. **재계획 정책 변경**: 정기 1 Hz 전체 재계획이 아니라 **이벤트 기반 + ≈1 s 저비용 경로 유효성 검사** (공식 예제 월드 크기에서 A*가 step을 넘을 수 있으므로, §1).

D* Lite / LPA* / Theta* / JPS / Anytime 계열은 우리 규모에서 이득이 구현 위험보다 작다 (§2).

---

## 1. 실측 [MEASURED] 과 공식 timestep 대비 해석

### 1.1 old practice-PC measurement (160×160 = 8 m @0.05, e-puck 인플레이션 1.74셀)

| 연산 | 시간 | 비고 |
|---|---|---|
| `planning.inflate_obstacles` (r = 1.74 셀) | 2.7 ms | e-puck 반경 기준 |
| `planning.astar` 4-연결 Manhattan | 49.2 ms | 291 셀 = 14.50 m |
| 8-연결 A* (octile, corner-cut 금지) 프로토타입 | 49.8 ms | 210 셀 = **12.13 m** |
| 8-연결 A* + danger(`α·(5−d)²`, α=2, 5셀 이내) | 57.4 ms | 15.12 m — **α가 과해 우회 증가** |
| 장애물 거리 변환 (BFS, cutoff 5셀) | 9.5 ms | danger 비용용 |
| BFS 거리장 (dict) / (flat list) | 24.4 / **12.3 ms** | |
| LOS smoothing (Bresenham 검사) | 12~17 ms | waypoint 7~10개 |
| `find_frontiers` + `cluster_frontiers` | 6.1 + 0.3 ms | |

(bench.py 연산은 3회 min; bench2.py의 8-connected/danger/LOS/거리변환은 1회 측정. 벽 2 + random 300(seed=0), start=(5,5), goal=(150,150). median/p95 실험 아님. 원본 경로는 09 §8.)

### 1.2 이전 pass PC measurement — 합성 대형 grid (TB3 인플레이션 r = (0.105+0.05)/0.05 = 3.1셀)

공식 예제 월드는 apartment ≈13×13 m, breakroom ≈13×8 m [DERIVED] → 시작점 중심 grid면 16~24 m가 필요할 수 있다. stress 조건(교대 gap 벽+random 장애물; 이론적 최악 시간 보장 아님), median / 소표본 p95:

| 연산 | 320×320 (16 m) | 480×480 (24 m) |
|---|---|---|
| inflate (3.1셀) | 10 / 12 ms | 20 / 21 ms |
| A* 4-연결 (최장 경로) | **128 / 131 ms** | **514 / 525 ms** |
| BFS 거리장 (flat) | 33 / 35 ms | 101 / 116 ms |
| find_frontiers (중앙 면적 1/4 관측) | 24 / 26 ms | 58 / 59 ms |
| 기존 binary insert_scan (360 rays, ≤3.5 m) | 6–11 ms (해당 grid 범위에서) | 동일 |

### 1.3 해석 (공식 step 64 ms 기준)
- 공식 주행 월드 `basicTimeStep` = **64 ms** (일부 테스트 월드는 기본 32 ms) [OFFICIAL, 09 §6]. 구 문서의 "16 ms step" 전제는 **e-puck practice 월드 값이라 폐기**.
- 큰 맵에서 A* 한 번이 **2~8 step 분량**. 거리장·frontier 검출도 step에 근접. → **매 step 실행 불가, 정기 1 Hz 전체 재계획도 비권장.**
- **동기 모드**(공식 월드: TB3 `synchronization` 기본 TRUE, 월드 미변경 [OFFICIAL]): step 사이 계산 동안 simulation이 기다리므로 추가 simulation-time 진행은 없다. 샘플링/명령 전달 지연과 wall-clock 비용은 남는다. 제한시간이 real-time 기준이면 손실 [DAY-OF].
- **비동기일 가능성**(Webots 문서: "asynchronous mode is currently used only for the robot competitions" — **일반 설명이지 TECH WEEK 설정 확인 아님**): 계산 중 직전 명령 유지. 0.15 m/s로 simulation 0.5 s가 진행되면 7.5 cm [DERIVED]; wall-clock과 simulation 시간 비율은 보장되지 않는다. 정지 명령을 step으로 전달한 후 계산하고, work budget/확장 상한 또는 증분 실행으로 안전 루프를 유지한다(10 §2.2). 저주기만으로 blocking 문제는 해결되지 않는다.
- 대책 옵션 [INITIAL TUNING, S3에서 측정 후 결정]: (a) planning 전용 **0.10 m 다운샘플 grid**(셀 수 1/4, TB3 반경 0.105 m ≈ 1셀이라 인플레이션 1.6셀), (b) A* 확장/시간 상한 + 유효한 cached 거리장만 폴백(새 거리장도 budget 필요), (c) 거리장 재사용(ETA·frontier 공용), (d) 경로 유효성 검사로 불필요한 재계획 제거.

## 2. 알고리즘 비교 (우리 기준: 구현 난이도 / 속도 / 맵 갱신 대응 / 동적 환경 / 안정성)

| 알고리즘 | 구현 | 속도(우리 맵) | 맵 갱신 대응 | 동적 환경 | 안정성 | 판정 | 출처 |
|---|---|---|---|---|---|---|---|
| **A\*** (현재, 공식 교육과 동일 계열) | 완료 | 50 ms (8 m) / 128–514 ms (16–24 m 최악) [MEASURED] | 이벤트 기반 전체 재계획 | 재계획으로 대응 | 높음, 결정적 | **유지** | Hart 1968 [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136) |
| **BFS/Dijkstra 거리장** | LOW | 12 ms (단위비용 BFS) | 전체 재계산 | – | 높음 | **추가 (다중 목표용)** | Nav2 NavFn 기본 `use_astar: false`(= Dijkstra 파동) ✅ nav2_params.yaml; hector Exploration Transform ✅ |
| Weighted A* (ε>1) | LOW (1줄) | 더 빠름 | – | – | 준최적 | 필요 시 옵션 | Pohl 1970 📄 |
| **8-연결 + octile** | LOW | 50 ms | – | – | 코너 컷 금지 필수 | **추가** | Nav2 Smac 2D(비용 인지 A*) ✅ 문서 |
| Theta* (any-angle) | MED | A*보다 느림(LOS 검사 다수) | – | – | 경로가 벽에 붙기 쉬움 | **대신 LOS smoothing 후처리** | Daniel et al. 2010 [doi:10.1613/jair.2994](https://doi.org/10.1613/jair.2994); Nav2 Theta* ✅; PythonRobotics [`theta_star.py`](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ThetaStar/theta_star.py) |
| Jump Point Search | MED | 균일 비용 grid에서 빠름 | – | – | 비용 있는 grid(danger)와 궁합 나쁨 | DO NOT | Harabor & Grastien 2011 📄 |
| D* Lite | HIGH (PythonRobotics 405줄) | 증분 재계획 빠름 | **증분** | 좋음 | 인플레이션·로봇 이동과 얽혀 버그 위험 | **DO NOT** (증분 구조 복잡도; 우선 bounded A* 계측) | Koenig & Likhachev 2002 (AAAI) 📄; [`d_star_lite.py`](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DStarLite/d_star_lite.py) |
| LPA* | HIGH | 증분 | 증분 | – | 동일 | DO NOT | Koenig et al. 2004 [doi:10.1016/j.artint.2003.12.001](https://doi.org/10.1016/j.artint.2003.12.001) |
| ARA*/Anytime | HIGH | – | – | – | – | DO NOT (우선 bounded A*·budget 관리, 시간 제약은 존재) | Likhachev 2003 📄 |
| Hybrid-A*/State Lattice | HIGH | 느림 | – | – | 차동구동 원형 로봇엔 불필요 | DO NOT | Nav2 문서: 원형 footprint 로봇은 holonomic planner(NavFn, Smac 2D, Theta*)로 충분 |
| FAR (visibility graph) | VERY HIGH | – | 동적 가시성 그래프 | 좋음 | – | 아이디어만 (아래 3.3) | Yang 2022 [doi:10.1109/IROS47612.2022.9981574](https://doi.org/10.1109/IROS47612.2022.9981574) ✅ |

## 3. 레퍼런스에서 가져올 설계 요소

### 3.1 비용 인지(cost-aware) 계획
- Nav2 InflationLayer ✅ [`inflation_layer.hpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp) `computeCost`: 장애물 = 254, 내접반경 안 = 253, 그 밖은 `252·exp(−k·(d − r_inscribed))` (기본 `cost_scaling_factor 3.0`, `inflation_radius 0.70`) [REFERENCE]. **TurtleBot3 burger Nav2 예제**: `robot_radius 0.1`, `inflation_radius 0.5`, `cost_scaling_factor 5.0` ✅ [burger.yaml](https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_navigation2/param/burger.yaml) [REFERENCE]. (webots_ros2 e-puck 예제 0.035/0.05는 practice 참고만.)
- Hector ✅ `cellDanger`: 장애물 거리 < `min_obstacle_dist`(기본 10셀)이면 `α·(d_min − d)²` (α = `security_constant` 0.5, 단위 = 셀당 100).
- 우리 [INITIAL TUNING]: 하드 인플레이션 = 안전 footprint 0.111(외접 약 0.1103을 올림) + SAFETY_MARGIN 0.05 = **0.161 m (3.22셀)**; notebook의 0.105는 교육 모델로 별도 보존, 그 바깥 0.15 m 정도에 약한 danger 비용. `obstacle_distance`로 `step_cost = 1 + w·max(0, d_safe − d)²`. **w는 우회 길이가 +10% 넘지 않게** 작게 시작 (실측에서 α=2는 +25% 우회).

### 3.2 도달 불가 goal 처리 (goal relaxation)
- Nav2 NavFn `tolerance: 0.5` ✅ (goal이 막혀 있으면 반경 안 대체 goal).
- FAR `ReEvaluateGoalPosition` ✅ [`graph_planner.cpp`](https://github.com/MichaelFYang/far_planner/blob/melodic-noetic/src/far_planner/src/graph_planner.cpp): goal에서 BFS로 **가장 가까운 FREE 셀**로 이동.
- SemExp ✅ [`sem_exp.py`](https://github.com/devendrachaplot/Object-Goal-Navigation/blob/master/agents/sem_exp.py) `_get_stg`: goal을 disk(10)로 팽창해 **영역 goal**로 계획.
- 우리: `relax_goal(infl, goal, max_r)` — goal에서 BFS, inflated FREE인 첫 셀. target 접근(07)과 home(08)에서 필수.

### 3.3 알려진 공간 우선, 실패 시 낙관적 계획
- FAR ✅: `is_free_nav`(알려진 free만) 실패 시 **attemptable**(unknown 통과 허용) 자동 전환.
- Nav2 NavFn `allow_unknown: true` 가 기본.
- 우리 `planning.astar(..., allow_unknown=True)` 가 현재 기본값 → **RETURN_HOME에서는 먼저 `allow_unknown=False`**, 실패 시 True로. 탐색 중에는 goal이 frontier(FREE)이므로 known-only로 충분.

### 3.4 "방문한 셀은 통과 가능" (SemExp 트릭 ✅)
- `traversible[visited == 1] = 1`, 로봇 주변 3×3 강제 통과 가능.
- 이유: 인플레이션 때문에 **로봇이 지금 서 있는 셀/지나온 좁은 통로가 막혀 보이는** 경우(시작점이 벽 옆, 좁은 복도)에 경로가 사라지는 것을 방지. 우리: reference의 강제 FREE를 그대로 복사하지 않는다. 현재 유한 hit/충돌 기록을 덮지 않고 footprint 이산화 오차만 보정한다. breadcrumb도 현재 scan과 footprint로 재검증한다. 안전 반경 미만으로 축소하지 않는다.

### 3.5 재계획 트리거 (Nav2 BT ✅)
- `navigate_to_pose_w_replanning_and_recovery.xml`: 1 Hz 주기 + `ValidatePath`(경로가 유효하지 않을 때) + 목표 변경 시.
- `navigate_w_replanning_only_if_path_becomes_invalid.xml`: 경로가 무효일 때만.
- 우리: **(a) 남은 경로 앞쪽 N m 안의 셀이 inflated OCCUPIED가 됨 (b) goal 변경 (c) progress 실패** 중 하나면 재계획 (Nav2 `navigate_w_replanning_only_if_path_becomes_invalid.xml`과 같은 방식 [REFERENCE]). (a)는 새 스캔마다 남은 경로 셀만 확인(검사 범위 제한; 비용은 runtime 계측 전 UNCONFIRMED). **정기 1 Hz 전체 재계획은 하지 않음** — 공식 월드 규모에서 A*가 64 ms step을 여러 번 넘기 때문(§1.2). 대신 ≈1 s마다 (a)의 유효성 검사만.

## 4. 추천 함수 (planning.py, 인터페이스 `path = [(row, col), ...]` 유지)

| 함수 | 역할 |
|---|---|
| `astar(grid, start, goal, allow_unknown=True, connectivity=4)` | 기존 유지, `connectivity=8` 옵션 추가 (기존 테스트 보호) |
| `distance_field(grid, start)` → `(dist, parent)` | 다중 목표 질의, ETA |
| `obstacle_distance(grid, cutoff)` | danger 비용·RPP 감속용 |
| `relax_goal(grid, goal, max_radius)` | 막힌 goal 대체 |
| `smooth_path(grid, path)` | LOS 단축 → waypoint 리스트 |
| `path_is_valid(grid, path, from_index, horizon)` | 재계획 트리거 |

## 5. 테스트
- 8-연결: 대각 코너 컷 금지, 경로 길이 ≤ 4-연결.
- 기존 `tests/test_planning.py`는 4-연결 기본값으로 그대로 통과해야 함.
- `relax_goal`: OCCUPIED goal → 가장 가까운 FREE 셀.
- `smooth_path`: 결과 선분이 모두 inflated FREE 위.
- `path_is_valid`: 경로 위에 장애물 추가 시 False.
- `distance_field`: BFS 결과 == A* 경로 길이(4-연결, 단위 비용).


## 기존 reference 링크 보존

아래는 이전 연구의 참고 링크를 보존한 것이다. e-puck은 practice/concept only, Erebus는 REFERENCE이며 TECH WEEK 규정이 아니다. 일반 API 예제는 공식 로봇의 장치 배치·competition 입력 허용을 증명하지 않는다.

- [robot](https://cyberbotics.com/doc/reference/robot) — platform/algorithm reference.
- [nav2_params.yaml](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/resource/nav2_params.yaml) — practice/reference only.
- [navigate_to_pose_w_replanning_and_recovery.xml](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml) — platform/algorithm reference.
