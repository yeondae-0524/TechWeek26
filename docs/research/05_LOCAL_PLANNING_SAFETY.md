# 05. Local Planning, Collision Avoidance, Safety & Dynamic Obstacles

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 태그: [ORGANIZER] [OFFICIAL] [DERIVED] [MEASURED] [REFERENCE] [INITIAL TUNING] [DAY-OF].
> **2026-09-30 공식 TECH WEEK repo 기준 재검증.** 우리 로봇 = **TurtleBot3Burger**: 반경 0.105 m(외접 ≈0.110), 바퀴 간격 0.160 m, 한계 0.22 m/s · 2.75 rad/s, 앞 끝 x≈+0.036 / 뒤 끝 x≈−0.100 (회전중심 기준, 비대칭) [OFFICIAL/DERIVED, 09 §3]. LiDAR는 회전중심 뒤 0.03 m, 높이 ≈0.173 m, **minRange 0.12 m** [OFFICIAL/DERIVED]. (구판의 e-puck 수치는 practice 기록으로 강등.)

### 공식 notebook의 Path 제어와 비교 [OFFICIAL 교육 내용 요약]
- 공식 notebook "Action" 절: waypoint 경로 → 최근접 waypoint(Euclidean, `cKDTree`) → **경로거리 기준 look-ahead 점** → 로봇 frame 변환 → 곡률 `κ = 2y/(x²+y²)` → `ω = vκ` (v는 기본/최대 속도 상수 가능) → `v_r = v + ωL/2`, `v_l = v − ωL/2`.
- 이는 **pure pursuit** 이며 우리 RPP-lite의 핵심과 같다. RPP-lite가 추가하는 것: 큰 각도 제자리 회전, 곡률·근접·접근 감속, 1 s 충돌 예측 [REFERENCE: Nav2 RPP]. 이 추가 기능은 **우리 선택**이며 공식 요구가 아니다.
- 공식 계획안은 Local planner 예시로 "DWA 등 critic 기반"을 들지만 **예시 목록**이다 [OFFICIAL]. 아래 이유로 DWA를 채택하지 않는 결정은 유지.

---

## 0. 결론 (Q9)

**DWA는 P0/P1에서 구현하지 않는다.** 대신 3층 구조:

1. **Path follower = "RPP-lite"** (Nav2 Regulated Pure Pursuit의 핵심 3가지만): 큰 각도면 제자리 회전 → pure pursuit 곡률 → 곡률/장애물 근접/goal 근접 감속.
2. **Safety monitor** (Nav2 Collision Monitor 원리): 지도·계획과 무관하게 **raw LiDAR로 매 스텝** STOP/SLOWDOWN 영역 + 짧은 전방 시뮬레이션 충돌 검사.
3. **동적 장애물은 추적하지 않고** "기다림 → 재계획 → recovery" 정책 + log-odds 지도(03)로 처리.

DWA가 지는 이유: (a) Python에서 (v,ω) 샘플 수 × 예측 스텝 × 장애물 점 비용이 크다 (PythonRobotics 기본 `v_resolution 0.01`, `yaw_rate_resolution 0.1°`, `predict_time 3.0 s` ✅ [`dynamic_window_approach.py`](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py)), (b) local minimum — PythonRobotics조차 `robot_stuck_flag_cons`로 억지 회전하는 hack을 둠, (c) 가중치 3개 이상 튜닝, (d) 전역 경로가 이미 인플레이션된 안전 경로라 local planner가 할 일이 적다.

---

## 1. 비교표

| 방법 | 원리 | 구현(Python) | 튜닝 | 동적 장애물 | 좁은 통로 | 우리 판정 | 출처 |
|---|---|---|---|---|---|---|---|
| **Pure Pursuit** | lookahead 점으로 곡률 κ=2y/L² | LOW | lookahead 1개 | ✗ (safety에 위임) | 경로 품질 의존 | 기반 | Coulter 1992 (CMU-RI-TR-92-01) 📄 |
| **RPP (Regulated PP)** | PP + 곡률·비용·접근 감속 + rotate-to-heading + 전방 충돌 검사 | LOW~MED | 적음 | 전방 충돌 시 정지 | 감속으로 강함 | **채택 (lite)** | Macenski 2023 [doi:10.1007/s10514-023-10097-6](https://doi.org/10.1007/s10514-023-10097-6) · ✅ 코드 |
| CMU pathFollower | heading 오차 P제어 + 오차 < `dirDiffThre`(0.1 rad)일 때만 가속 + 끝점 감속 | LOW | 적음 | ✗ | – | RPP-lite와 동등, **단순 참고** | ✅ [`pathFollower.cpp`](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/local_planner/src/pathFollower.cpp) |
| CMU localPlanner (path library) | 미리 만든 경로군을 스캔 점과 충돌 검사, 방향 점수 최대 그룹 선택, 없으면 scale 축소 | MED | 중 | 좋음 (raw 점 기반) | 좋음 | P3 옵션 ("arc sampler") | ✅ [`localPlanner.cpp`](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/local_planner/src/localPlanner.cpp) |
| DWA / DWB | (v,ω) 샘플 → 궤적 점수(critics) | MED~HIGH | 많음 (TB3 DWB critics 7개) | 좋음 | 진동 위험 | P3 이하 | Fox 1997 [doi:10.1109/100.580977](https://doi.org/10.1109/100.580977); TB3 [`burger.yaml`](https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_navigation2/param/burger.yaml) ✅ |
| VFH / VFH+ | 극좌표 히스토그램에서 빈 골짜기 선택 | MED | 임계값 여러 개 | 반응적 | 좁은 통로 진동 | DO NOT (전역 경로와 이중 결정) | Borenstein & Koren 1991 [doi:10.1109/70.88137](https://doi.org/10.1109/70.88137); Ulrich & Borenstein 1998 [doi:10.1109/ROBOT.1998.677362](https://doi.org/10.1109/ROBOT.1998.677362) |
| Artificial Potential Field | 인력+척력 | LOW | 쉬움 | 반응적 | **local minimum·진동** 이론적으로 증명된 한계 | DO NOT (단, 척력 개념은 감속에만) | Khatib 1986 [doi:10.1177/027836498600500106](https://doi.org/10.1177/027836498600500106); Koren & Borenstein 1991 [doi:10.1109/ROBOT.1991.131810](https://doi.org/10.1109/ROBOT.1991.131810) |
| TEB | 시간 탄성 밴드 최적화 | HIGH | 많음 | 좋음 | – | DO NOT | Rösmann 2017 [doi:10.1016/j.robot.2016.11.007](https://doi.org/10.1016/j.robot.2016.11.007) |
| MPC / MPPI | 예측 최적화 / 샘플 최적화 | HIGH | 많음 | 좋음 | – | DO NOT (Python 실시간 부담) | Nav2 MPPI ✅ (nav2_params 기본 컨트롤러) |
| Velocity Obstacles / RVO / ORCA | 상대 속도 공간 회피 | HIGH | – | 이동체 속도 추정 필요 | – | DO NOT (사람은 협조 에이전트가 아님, 속도 추정이 불안정) | Fiorini & Shiller 1998 [doi:10.1177/027836499801700706](https://doi.org/10.1177/027836499801700706); van den Berg 2011 [doi:10.1007/978-3-642-19457-3_1](https://doi.org/10.1007/978-3-642-19457-3_1) |

## 2. RPP 코드에서 확인한 핵심 (✅ [`regulated_pure_pursuit_controller.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp), [`regulation_functions.hpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp), [`collision_checker.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/src/collision_checker.cpp), [`parameter_handler.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp))

`computeVelocityCommands` 흐름:
1. lookahead 거리 → carrot 점, 곡률 계산.
2. goal XY 도달 → **goal heading으로 제자리 회전**; carrot 방향 오차 > `rotate_to_heading_min_angle`(0.785 rad) → **제자리 회전** (각가속 제한 + `sqrt(2·a·|Δθ|)`로 감속 정지).
3. 그 외: `applyConstraints` = min(곡률 감속, 비용 감속) + goal 접근 감속 → `ω = v·κ`.
4. `isCollisionImminent`: (v, ω)로 **최대 1.0 s(`max_allowed_time_to_collision_up_to_carrot`) 전방 시뮬레이션**, 각 스텝 footprint 충돌 시 예외(정지).

규제 함수:
- 곡률: 회전 반경 `R < regulated_linear_scaling_min_radius`(0.9 m)면 `v·(1 − |R−R_min|/R_min)`.
- 비용: 추정 장애물 거리 < `cost_scaling_dist`(0.6 m)면 `v·gain·d/cost_scaling_dist`.
- 접근: 남은 거리 < `approach_velocity_scaling_dist`(0.6 m)면 비례 감속, 최소 `min_approach_linear_velocity`(0.05).

기본값: `lookahead_dist 0.6`, `min/max 0.3/0.9`, `lookahead_time 1.5`, `rotate_to_heading_angular_vel 1.8`.

### 2.1 TurtleBot3 환산 (모두 [INITIAL TUNING], S2 `NAV_TEST`에서 튜닝)

Nav2 기본값은 0.5 m/s급 로봇 기준 [REFERENCE]. TB3 burger는 한계 0.22 m/s [DERIVED]이므로 **운용 최대 v = 0.15 m/s** (한계의 ≈70%, 안전 여유)로 시작하고 거리 파라미터를 0.15/0.5 ≈ 0.3배로 축소. 참고: TurtleBot3 공식 Nav2 burger 설정 `max_vel_x 0.3`(DWB), `controller_frequency 10`, `xy_goal_tolerance 0.25` ✅ [REFERENCE].

| 파라미터 | Nav2 기본 [REFERENCE] | 우리 시작값 [INITIAL TUNING] | 근거 |
|---|---|---|---|
| 운용 최대 v / ω | 0.5 m/s / 2.5 rad/s | **0.15 m/s / 1.5 rad/s** | 한계 0.22 m/s, 2.75 rad/s [DERIVED]; 공식 teleop 바퀴 3.0 rad/s ≈ 0.099 m/s [OFFICIAL] |
| lookahead | 0.6 m (min 0.3, max 0.9) | **≈0.20 m, clamp [0.15, 0.40]** | 1.5 s × 0.15 m/s ≈ 0.22 m, 최소 3셀 |
| rotate-in-place 임계 | 0.785 rad | 0.6 rad | CMU는 0.1 rad 넘으면 가속 안 함 (더 보수적) |
| 제자리 회전 속도 | 1.8 rad/s | ≤ 1.0 rad/s | 카메라 블러·오버슛 방지 |
| 곡률 감속 반경 | 0.9 m | 0.3 m | 축소 |
| 장애물 근접 감속 거리 | 0.6 m | 로봇 외곽 + 0.25 m | 인플레이션 0.16 m 바깥 |
| 접근 감속 거리 | 0.6 m | 0.25 m | – |
| 전방 충돌 시뮬 시간 | 1.0 s | 1.0 s (= 0.15 m) | 그대로 |
| goal XY 허용오차 | 0.25 m | waypoint 0.08 m, 최종 goal 0.10 m; target 도착 기준은 [DAY-OF] | – |

(practice e-puck 환산값 0.10~0.15 m lookahead 등은 폐기)

## 3. Safety Monitor (Nav2 Collision Monitor 원리, ✅ [`polygon.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/src/polygon.cpp), [`collision_monitor_node.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/src/collision_monitor_node.cpp), [문서](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html))

- 동작 유형: **STOP / SLOWDOWN(`slowdown_ratio` 0.5) / LIMIT / APPROACH**.
- APPROACH: 현재 속도로 `time_before_collision`(기본 2.0 s)까지 `simulation_time_step`(0.1 s) 간격으로 로봇을 투영, 충돌 시각에 맞춰 감속 → "항상 충돌까지 T초 이상 유지".
- `min_points`(기본 4): 영역 안 점이 4개 이상이어야 트리거 → **단일 점 노이즈 무시**.
- 핵심 원칙: **costmap·planner를 우회해 센서 데이터에 직접 작동**.

현재 baseline(`control.apply_emergency_stop`)과 비교:

| 항목 | 현재 | 권장 |
|---|---|---|
| 영역 | 전방 ±30°, `ROBOT_RADIUS + 0.04` (e-puck 값) | **TB3 footprint 기준** STOP: 로봇 외곽 + 0.05 m, SLOWDOWN: 외곽 + 0.25 m, 진행 방향 부채꼴 [INITIAL TUNING]. 외곽은 비대칭(앞 0.036, 뒤 0.100, 옆 0.089 m)이므로 원보다 **사각형/다각형 footprint**가 정확 |
| **LiDAR 사각** | 고려 없음 | LiDAR가 회전중심 뒤 0.03 m + **minRange 0.12 m** → 앞 끝에서 **≈0.054 m 이내 물체는 inf** [DERIVED]. 전방 inf는 "free"가 아니라 "정보 없음": 직전 스캔에서 가까워지던 점이 inf로 바뀌면 STOP. 정지 거리를 이 사각 밖에서 확보하도록 STOP 영역을 설정 |
| **저위 물체** | 고려 없음 | LiDAR 평면 ≈0.173 m 아래(바닥 공·사과 등)는 안 보임 → 06 collision map(전진 명령 중 무이동/가속도 spike) + (P2) 카메라 바닥 장애물 |
| 판정 | 최소 거리 1점 | `min_points` 2~3 [INITIAL TUNING]; 얇거나 먼 물체는 1점일 수 있어 emergency 최소거리 veto를 별도로 둠 |
| 후진 | 검사 없음 | 후진 명령 시 후방 부채꼴 검사 |
| 전방 시뮬 | 없음 | (v, ω) 1.0 s 투영, 각 스텝 원형 footprint vs 스캔 점 |
| 트리거 후 | 전진만 0 | navigator에 이벤트 전달 → 06 recovery ladder |

TB3는 **비대칭 footprint**(몸체가 바퀴축 뒤로 치우침)라 제자리 회전 시 뒤쪽 모서리가 반경 ≈0.110 m를 쓸고 지나간다 [DERIVED] → SPIN 전 **반경 0.13 m 안이 비었는지** 확인 [INITIAL TUNING]. (구판의 'e-puck은 원형이라 회전이 항상 안전' 서술은 폐기.)

## 4. 동적 장애물 (H)

### 4.1 레퍼런스 처리 방식

| 구현 | 방법 | 출처 |
|---|---|---|
| Nav2 | 추적 없음. costmap raytrace clearing + controller 전방 충돌 검사 + Collision Monitor + BT `Wait 5 s` | ✅ |
| FAR | 기존 장애물을 관통하는 스캔 레이 → dynamic 분류, 10 s decay | ✅ |
| obstacle_detector (tysik) | 스캔 점 그룹화(`max_group_distance 0.1 + range·0.00628`) → 선분/원 → Kalman 추적(`tracking_duration 2 s`) | ✅ [`obstacle_extractor.cpp`](https://github.com/tysik/obstacle_detector/blob/master/src/obstacle_extractor.cpp), [`obstacle_tracker.cpp`](https://github.com/tysik/obstacle_detector/blob/master/src/obstacle_tracker.cpp) |
| 사람 검출 | 다리 패턴 AdaBoost | Arras 2007 [doi:10.1109/ROBOT.2007.363998](https://doi.org/10.1109/ROBOT.2007.363998) 📄 |
| DATMO | SLAM + 이동체 추적 동시 | Wang 2007 [doi:10.1177/0278364907081229](https://doi.org/10.1177/0278364907081229) 📄 |
| 사람 추적 | JPDAF | Schulz 2003 [doi:10.1177/0278364903022002002](https://doi.org/10.1177/0278364903022002002) 📄 |

### 4.2 질문별 답

| 질문 | 답 |
|---|---|
| LiDAR만으로 접근 중인 장애물 판단? | **진행 방향 부채꼴의 최소 거리 d(t)의 감소율**이 우리 속도로 설명되는 것보다 크면 "접근 중" (`ḋ < −v_robot·cosφ − ε`). 두세 스캔의 이동평균으로 노이즈 억제. 전방 투영(APPROACH)과 결합. |
| 단순 temporal difference로 충분? | **안전 판단엔 충분.** "이전 스캔에서 free였던 레이 방향에 새 점"(= FAR의 관통 레이 판정 반대 방향)으로 new/dynamic 후보 표시 가능. 속도 벡터까지 필요한 경우(추월·회피 경로 예측)가 아니면 추적 불필요. |
| object tracking 필요? | **P3.** 사람 1~2명 수준이면 "멈춰서 기다림 + 재계획"이 더 안정적. 추적은 오탐/연관 오류가 새 실패 모드를 만든다. |
| global map 반영? | log-odds로 **일시 반영**(03). 현재 제안 임계에서 10회 유한 free 관측이 필요; 가림/inf면 해제 시간 보장 없음. dynamic 의심 셀은 복귀 계획 실패 시 우선 초기화 대상. |
| 기다릴까 우회할까? | 경로를 막은 장애물이 **최근 새로 나타남**(해당 셀이 이전엔 확신 free) → **WAIT** (T_wait 3~5 s; Nav2 BT `Wait wait_duration="5.0"`). 여전히 막힘 → 재계획(우회로 있으면 우회). 우회로 없음 → recovery ladder. 정적 장애물(원래 지도에 있던 것)이 경로를 막으면 기다리지 말고 즉시 재계획. |
| 언제 global replanning? | 04 §3.5: 경로 무효·goal 변경·진전 실패 시 이벤트 재계획; 약 1 Hz는 유효성 확인만. 대기 중엔 1 Hz로 "경로가 다시 유효해졌나"만 확인. |

## 5. 추천 구현 (control.py)

```text
class PathFollower:                       # RPP-lite
    set_path(waypoints)                    # smooth_path 결과 (world 좌표)
    compute(pose, scan_points) -> (v, w, status)
        carrot = lookahead point (L = clamp(k_t * v, L_min, L_max))
        err = angle to carrot (robot frame)
        if |err| > ROTATE_THRESH: return (0, sign(err)*ω_rot_limited, RUNNING)
        κ = 2*y_c / L²
        v = v_max * min(curv_scale(κ), prox_scale(d_obs), approach_scale(d_goal))
        ω = v*κ
        if collision_imminent(v, ω, scan_points, T=1.0): return (0, 0, BLOCKED)
        if at goal: return (0, 0, REACHED)

class SafetyMonitor:                      # 매 스텝, 마지막에 적용
    filter(v, ω, scan_points) -> (v', ω', event)   # STOP/SLOW/후진검사/min_points
```

- 모두 **duck-typed, Webots 없이 테스트 가능** (현재 control.py 스타일 유지).
- Webots API는 계속 devices.py/main.py에만.

## 6. 테스트
- 직선 경로: 오차 없이 도달, 끝에서 감속.
- 90° 코너: 임계각 초과 시 v=0 회전 후 진행.
- robot frame에서 footprint 앞 5 cm에 합성 점군: `collision_imminent` True → BLOCKED.
- 뒤에서 점 접근 + 후진 명령: 후진 차단.
- `min_points`: 멀리 떨어진 단일 노이즈는 필터하되 임박한 단일 유효 hit/얇은 장애물은 정지 검사.
- 접근 판정: 연속 스캔에서 d(t)가 자기 속도보다 빠르게 감소 → APPROACHING 이벤트.


### 안전·구동 한계 보충 (10 §11.1과 동일)

[INITIAL TUNING] 보수적 safety circle은 PROTO 외접 약 0.1103 m를 올린 **0.111 m**, inflation은 +0.05 m = **0.161 m**부터 검증한다. STOP margin 0.05 m는 고정 안전 보장이 아니다. `v·sensor_age + v·command_latency + v²/(2a_brake) + uncertainty`가 더 크면 확대하거나 속도를 줄인다. a_brake와 실제 sensor age는 UNCONFIRMED, S0/S2에서 측정한다. 전방 body 끝 기반 0.05 m 여유만 쓰면 LDS blind zone에 걸릴 수 있으므로 sensor origin 변환까지 검사한다.

`v=0.15 m/s`, `ω≤1.5 rad/s`도 동시에 허용되는 사각 한계가 아니다. `|(v±ω·0.160/2)/0.033|≤6.67 rad/s`를 만족하도록 함께 제한한다 [OFFICIAL 기하에서 DERIVED]. inf/NaN/stale scan 및 SPIN/BACKUP의 관측되지 않은 공간은 안전 증거가 아니므로 정지/동작 보류한다. 충돌 경험 지도는 사후 복구이며 첫 충돌을 예방했다는 보장이 아니다.
