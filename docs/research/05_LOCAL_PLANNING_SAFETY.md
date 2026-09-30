# 05. Local Planning, Collision Avoidance, Safety & Dynamic Obstacles

> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 숫자는 레퍼런스 원값과 우리 로봇(e-puck: 반경 0.037 m, `MAX_LINEAR_SPEED 0.08 m/s`, `MAX_ANGULAR_SPEED 1.5 rad/s`) 환산값을 구분 표기.

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
| CMU localPlanner (path library) | 미리 만든 경로군을 스캔 점과 충돌 검사, 방향 점수 최대 그룹 선택, 없으면 scale 축소 | MED | 중 | 좋음 (raw 점 기반) | 좋음 | P2 옵션 ("arc sampler") | ✅ [`localPlanner.cpp`](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/local_planner/src/localPlanner.cpp) |
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

### 2.1 우리 로봇 환산 (initial tuning suggestion)

기본값은 0.5 m/s급 로봇 기준이다. **거리 파라미터는 속도에 비례해 축소**한다 (0.08/0.5 ≈ 0.16).

| 파라미터 | Nav2 기본 | 우리 시작값 | 근거 |
|---|---|---|---|
| lookahead | 0.6 m (min 0.3) | **0.10~0.15 m** | `lookahead_time 1.5 s × 0.08 m/s = 0.12 m`, 최소 2~3셀 |
| rotate-in-place 임계 | 0.785 rad | 0.5~0.785 rad | CMU는 0.1 rad 넘으면 가속 안 함 (더 보수적) |
| 제자리 회전 속도 | 1.8 rad/s | ≤ 1.0 rad/s | config 최대 1.5, 카메라 블러·오버슛 방지 |
| 곡률 감속 반경 | 0.9 m | 0.15 m | 축소 |
| 장애물 근접 감속 거리 | 0.6 m | 0.10~0.15 m | 인플레이션 0.087 m 바깥 |
| 접근 감속 거리 | 0.6 m | 0.10 m | – |
| 전방 충돌 시뮬 시간 | 1.0 s | 1.0 s (= 8 cm) | 그대로 |
| goal XY 허용오차 | 0.25 m (Nav2), **0.05 m (webots_ros2 e-puck)** | 0.05 m (waypoint는 0.03~0.05) | e-puck 예제 ✅ |

## 3. Safety Monitor (Nav2 Collision Monitor 원리, ✅ [`polygon.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/src/polygon.cpp), [`collision_monitor_node.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/src/collision_monitor_node.cpp), [문서](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html))

- 동작 유형: **STOP / SLOWDOWN(`slowdown_ratio` 0.5) / LIMIT / APPROACH**.
- APPROACH: 현재 속도로 `time_before_collision`(기본 2.0 s)까지 `simulation_time_step`(0.1 s) 간격으로 로봇을 투영, 충돌 시각에 맞춰 감속 → "항상 충돌까지 T초 이상 유지".
- `min_points`(기본 4): 영역 안 점이 4개 이상이어야 트리거 → **단일 점 노이즈 무시**.
- 핵심 원칙: **costmap·planner를 우회해 센서 데이터에 직접 작동**.

현재 baseline(`control.apply_emergency_stop`)과 비교:

| 항목 | 현재 | 권장 |
|---|---|---|
| 영역 | 전방 ±30°, 0.077 m | STOP 원(반경 r_robot + 0.02), SLOWDOWN 원(r_robot + 0.08), 진행 방향 부채꼴 |
| 판정 | 최소 거리 1점 | `min_points` 2~3 (360 rays에서 3~4 cm 물체도 여러 점이 잡힘) |
| 후진 | 검사 없음 | 후진 명령 시 후방 부채꼴 검사 |
| 전방 시뮬 | 없음 | (v, ω) 1.0 s 투영, 각 스텝 원형 footprint vs 스캔 점 |
| 트리거 후 | 전진만 0 | navigator에 이벤트 전달 → 06 recovery ladder |

e-puck은 원형이라 **제자리 회전은 항상 충돌 안전**(외접원 = 몸체) — 단, 근접 물체가 몸체 반경 안이면 이미 접촉 상태.

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
| global map 반영? | log-odds로 **일시 반영**(03). 계획은 사람을 피해 가고 떠나면 ~2 s 후 복원. dynamic 의심 셀은 복귀 계획 실패 시 우선 초기화 대상. |
| 기다릴까 우회할까? | 경로를 막은 장애물이 **최근 새로 나타남**(해당 셀이 이전엔 확신 free) → **WAIT** (T_wait 3~5 s; Nav2 BT `Wait wait_duration="5.0"`). 여전히 막힘 → 재계획(우회로 있으면 우회). 우회로 없음 → recovery ladder. 정적 장애물(원래 지도에 있던 것)이 경로를 막으면 기다리지 말고 즉시 재계획. |
| 언제 global replanning? | 04 §3.5: 1 Hz + 남은 경로가 무효 + goal 변경 + 진전 실패. 대기 중엔 1 Hz로 "경로가 다시 유효해졌나"만 확인. |

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
- 전방 5 cm에 점군: `collision_imminent` True → BLOCKED.
- 뒤에서 점 접근 + 후진 명령: 후진 차단.
- `min_points`: 단일 노이즈 점은 무시.
- 접근 판정: 연속 스캔에서 d(t)가 자기 속도보다 빠르게 감소 → APPROACHING 이벤트.
