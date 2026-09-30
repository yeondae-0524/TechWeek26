# 04. Global Planning

> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 실측값은 이 개발 PC(Python 3.10, 순수 Python, 160×160 grid, 벽 2개 + 무작위 장애물 300개, 코너→코너) 기준 2026-09-30 측정. 세션 scratchpad `bench.py`, `bench2.py` (프로젝트 파일은 수정하지 않고 import만 함).

---

## 0. 결론 (Q4)

**A\*를 유지한다.** 다만 다음 네 가지를 붙인다.

1. **4-연결 → 8-연결 + octile heuristic** (corner-cutting 금지): 경로 길이 −16% (14.50 m → 12.13 m), 시간은 거의 같음 (49 ms → 50 ms).
2. **BFS/Dijkstra 거리장**을 "다중 목표 질의"용으로 추가 (frontier 순위, home ETA): 1회 12 ms (flat list 구현).
3. **벽 근접 비용**(Hector `cellDanger` / Nav2 inflation cost 개념): 좁은 곳에서 벽에 붙지 않게. 가중치가 과하면 경로가 크게 우회하므로 약하게 시작.
4. **Line-of-sight 단축(smoothing)**: 경로 셀 210개 → waypoint 10개 (12~17 ms). 경로 추종(05)이 훨씬 안정적이 된다.

D* Lite / LPA* / Theta* / JPS / Anytime 계열은 **우리 규모에서 이득이 구현 위험보다 작다** (아래 표).

---

## 1. 실측 (순수 Python, 160×160)

| 연산 | 시간 | 비고 |
|---|---|---|
| `planning.inflate_obstacles` (r = 1.74 셀) | 2.7 ms | 현 baseline |
| `planning.astar` 4-연결 Manhattan | 49.2 ms | 경로 291 셀 = 14.50 m |
| 8-연결 A* (octile, corner-cut 금지) 프로토타입 | 49.8 ms | 210 셀 = **12.13 m** |
| 8-연결 A* + danger(`α·(5−d)²`, α=2, 5셀 이내) | 57.4 ms | 15.12 m — **α가 과해 우회 증가** |
| 장애물 거리 변환 (BFS, cutoff 5셀) | 9.5 ms | danger 비용 계산용 |
| BFS 거리장 (dict) / (flat list) | 24.4 / **12.3 ms** | 로봇에서 모든 셀까지 |
| LOS smoothing (Bresenham 검사) | 12~17 ms | waypoint 7~10개 |
| `find_frontiers` + `cluster_frontiers` | 6.1 + 0.3 ms | 반쯤 알려진 맵 |

해석:
- 한 번의 전역 계획(인플레이션 + 거리장 + A* + smoothing)은 **약 80~100 ms**. `basicTimeStep = 16 ms`이므로 **매 스텝 실행하면 안 된다**. 1 Hz 또는 이벤트 기반으로 실행 (Nav2 기본 BT도 `RateController hz="1.0"` 로 재계획 ✅ [`navigate_to_pose_w_replanning_and_recovery.xml`](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)).
- ⚠️ **Webots 비동기 컨트롤러 위험**: Webots 문서는 "asynchronous mode is currently used only for the robot competitions"라고 명시한다 ([robot.md](https://cyberbotics.com/doc/reference/robot) `synchronization` 필드). 대회가 `synchronization FALSE`라면 100 ms 계산 동안 로봇은 **직전 명령으로 계속 움직인다** (0.08 m/s × 0.1 s = 8 mm). 그래서 (a) 무거운 계산은 스텝당 예산을 두고 나누거나 1 Hz 이하로, (b) 계획 직전 속도를 줄이거나, (c) 안전 모니터(05)가 계산과 무관하게 매 스텝 돌도록 한다. 대회 당일 `robot.getSynchronization()` 값을 로그로 확인.

## 2. 알고리즘 비교 (우리 기준: 구현 난이도 / 속도 / 맵 갱신 대응 / 동적 환경 / 안정성)

| 알고리즘 | 구현 | 속도(우리 맵) | 맵 갱신 대응 | 동적 환경 | 안정성 | 판정 | 출처 |
|---|---|---|---|---|---|---|---|
| **A\*** (현재) | 완료 | 50 ms | 전체 재계획 (1 Hz면 충분) | 재계획으로 대응 | 높음, 결정적 | **유지** | Hart 1968 [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136) |
| **BFS/Dijkstra 거리장** | LOW | 12 ms (단위비용 BFS) | 전체 재계산 | – | 높음 | **추가 (다중 목표용)** | Nav2 NavFn 기본 `use_astar: false`(= Dijkstra 파동) ✅ nav2_params.yaml; hector Exploration Transform ✅ |
| Weighted A* (ε>1) | LOW (1줄) | 더 빠름 | – | – | 준최적 | 필요 시 옵션 | Pohl 1970 📄 |
| **8-연결 + octile** | LOW | 50 ms | – | – | 코너 컷 금지 필수 | **추가** | Nav2 Smac 2D(비용 인지 A*) ✅ 문서 |
| Theta* (any-angle) | MED | A*보다 느림(LOS 검사 다수) | – | – | 경로가 벽에 붙기 쉬움 | **대신 LOS smoothing 후처리** | Daniel et al. 2010 [doi:10.1613/jair.2994](https://doi.org/10.1613/jair.2994); Nav2 Theta* ✅; PythonRobotics [`theta_star.py`](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ThetaStar/theta_star.py) |
| Jump Point Search | MED | 균일 비용 grid에서 빠름 | – | – | 비용 있는 grid(danger)와 궁합 나쁨 | DO NOT | Harabor & Grastien 2011 📄 |
| D* Lite | HIGH (PythonRobotics 405줄) | 증분 재계획 빠름 | **증분** | 좋음 | 인플레이션·로봇 이동과 얽혀 버그 위험 | **DO NOT** (재계획이 이미 싸다) | Koenig & Likhachev 2002 (AAAI) 📄; [`d_star_lite.py`](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DStarLite/d_star_lite.py) |
| LPA* | HIGH | 증분 | 증분 | – | 동일 | DO NOT | Koenig et al. 2004 [doi:10.1016/j.artint.2003.12.001](https://doi.org/10.1016/j.artint.2003.12.001) |
| ARA*/Anytime | HIGH | – | – | – | – | DO NOT (시간 제약이 거의 없음) | Likhachev 2003 📄 |
| Hybrid-A*/State Lattice | HIGH | 느림 | – | – | 차동구동 원형 로봇엔 불필요 | DO NOT | Nav2 문서: 원형 footprint 로봇은 holonomic planner(NavFn, Smac 2D, Theta*)로 충분 |
| FAR (visibility graph) | VERY HIGH | – | 동적 가시성 그래프 | 좋음 | – | 아이디어만 (아래 3.3) | Yang 2022 [doi:10.1109/IROS47612.2022.9981574](https://doi.org/10.1109/IROS47612.2022.9981574) ✅ |

## 3. 레퍼런스에서 가져올 설계 요소

### 3.1 비용 인지(cost-aware) 계획
- Nav2 InflationLayer ✅ [`inflation_layer.hpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp) `computeCost`: 장애물 = 254, 내접반경 안 = 253, 그 밖은 `252·exp(−k·(d − r_inscribed))` (기본 `cost_scaling_factor 3.0`, `inflation_radius 0.70`, e-puck 예제는 `robot_radius 0.035`, local `inflation_radius 0.05`, global 0.55 ✅ [webots_ros2 e-puck nav2_params.yaml](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/resource/nav2_params.yaml)).
- Hector ✅ `cellDanger`: 장애물 거리 < `min_obstacle_dist`(기본 10셀)이면 `α·(d_min − d)²` (α = `security_constant` 0.5, 단위 = 셀당 100).
- 우리: `obstacle_distance`(9.5 ms)로 `step_cost = 1 + w·max(0, d_safe − d)²`. **w는 우회 길이가 +10% 넘지 않게** 작게 시작 (실측에서 α=2는 +25% 우회).

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
- 이유: 인플레이션 때문에 **로봇이 지금 서 있는 셀/지나온 좁은 통로가 막혀 보이는** 경우(시작점이 벽 옆, 좁은 복도)에 경로가 사라지는 것을 방지. 우리: 로봇 현재 셀과 반경 1셀은 계획 시 강제 FREE, breadcrumb(지나온 셀)은 인플레이션 완화.

### 3.5 재계획 트리거 (Nav2 BT ✅)
- `navigate_to_pose_w_replanning_and_recovery.xml`: 1 Hz 주기 + `ValidatePath`(경로가 유효하지 않을 때) + 목표 변경 시.
- `navigate_w_replanning_only_if_path_becomes_invalid.xml`: 경로가 무효일 때만.
- 우리: **(a) 1 Hz 주기 (b) 남은 경로 앞쪽 N m 안의 셀이 inflated OCCUPIED가 됨 (c) goal 변경 (d) progress 실패** 중 하나면 재계획. (b)가 핵심 — 매 맵 갱신마다 남은 경로 셀만 확인(수십 셀, <1 ms).

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
