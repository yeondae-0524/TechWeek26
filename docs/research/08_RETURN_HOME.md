# 08. Return Home (시간 예산 기반 복귀)

> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 외부 숫자는 [REFERENCE], 우리 ETA·거리·주기는 [INITIAL TUNING]. **GPS/Compass 미사용 [organizer-confirmed]**, Supervisor pose도 competition 입력에서 제외한다.

---

## 1. 레퍼런스 구현

| 구현 | 복귀 트리거 | 경로 | 방향 복원 | 실패 처리 | 출처 |
|---|---|---|---|---|---|
| **m-explore-ros2** `return_to_init` | 탐색 종료(frontier 없음/전부 blacklist)로 `stop(true)`일 때만. 수동 정지는 복귀 안 함 | Nav2 `NavigateToPose` 1회 | ✅ quaternion 포함 | **없음** (SUCCEEDED일 때만 상태 발행) | ✅ [`explore.cpp`](https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp) 생성자(초기 pose TF 조회, 실패 시 기능 비활성), `returnToInitialPose`, `stop` |
| **TARE** | 모든 subspace가 탐색 완료(`IsReturningHome()` && local coverage 완료) → `exploration_finished_` | 전역 TSP 경로에 home 포함 | – | `kRushHomeDist 5 m`, `kAtHomeDistThreshold 0.5 m` | ✅ [`sensor_coverage_planner_ground.cpp`](https://github.com/caochao39/tare_planner/blob/melodic-noetic/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp) `execute`, `GetRobotToHomeDistance`; [`indoor.yaml`](https://github.com/caochao39/tare_planner/blob/melodic-noetic/src/tare_planner/config/indoor.yaml) |
| **GBPlanner (CERBERUS)** | **`time_to_home = homing_path_len / v_homing_max`**, `time_remaining = min(time_budget_limit − elapsed, battery_remaining)`, **`time_to_home > time_remaining − 20 s`**이면 homing | 전역 그래프 최단경로 + 안전 개선(`improveFreePath`) + 보간 | – | `homing_backward` 옵션(온 길 역주행), 예산 0이면 정지 | ✅ [`rrg.cpp`](https://github.com/ntnu-arl/gbplanner_ros/blob/gbplanner3/gbplanner/src/rrg.cpp) `homingRequired`, `isRemainingTimeSufficient`(`kTimeDelta = 20 // magic number, extra safety`) |
| **Hector** | – | 로봇 **궤적(trajectory) 서비스**를 exploration에 활용 | – | – | ✅ `findInnerFrontier`가 `GetRobotTrajectory` 사용 |
| DARPA SubT 팀들 | 통신/배터리 예산 기반 귀환 | – | – | – | Tranzatto 2022 [doi:10.1126/scirobotics.abp9742](https://doi.org/10.1126/scirobotics.abp9742), Scherer 2022 [doi:10.55417/fr.2022023](https://doi.org/10.55417/fr.2022023) 📄 |

## 2. 우리 설계

### 2.1 home_pose
- 추천: 시작 자세를 local map frame 원점으로 잡아 `home_pose=(0,0,0)`으로 저장한다. local +x는 시작 heading, +y는 시작 좌측, CCW 양수. 제공 world 시작 pose는 local↔world 정적 변환에만 사용한다. 현재 코드의 world `START_POSE` 방식과 구분하며 이후 구현에서 모든 pose/grid/target에 같은 변환을 적용한다. 복귀는 localization estimate + occupancy map 기반이다.

### 2.2 트리거 (Q11, GBPlanner 식을 우리 규모로)

```text
ETA_home   = L_home / v_avg + N_turns * (π/2)/ω + T_margin_fixed
L_home     = distance_field(from robot)[home_cell] * res     # 04의 거리장, known-only
v_avg      = 실측 평균 주행속도 (로그에서 갱신, 초기값 0.6 * MAX_LINEAR_SPEED)
trigger if  time_left < SAFETY_FACTOR * ETA_home + T_margin
            SAFETY_FACTOR = 1.4, T_margin = 15 s [INITIAL TUNING] (GBPlanner +20 s는 REFERENCE)
```
- 약 1 s마다 예산을 확인하되 전체 계획은 이벤트 기반이다. 유효한 거리장/경로 길이를 재사용하고 무효해지면 재계산한다. 거리장 자체는 비용이 크다(04). `T_margin_fixed=0`으로 시작해 margin 이중 계산을 피한다. TB3 v=0.15 m/s라면 v_avg 초기 0.09 m/s [INITIAL TUNING]; 실제 회전·대기 포함 주행 로그로 교체한다.
- target 접근 결정에도 사용: `time_left < SF * (ETA_target + ETA_target→home) + margin` 이면 접근 포기(위치만 기록).
- 현재 baseline의 고정 `MISSION_TIME_LIMIT` 비교보다 **맵이 클수록/멀리 있을수록 일찍 복귀**하므로 안전.
- ⚠️ 시간 기준(시뮬레이션 시간 vs 실시간)은 당일 규정으로 확인. 동기/비동기 어느 모드에서도 두 시간은 다를 수 있음 (09 참고).

### 2.3 경로 전략 (우선순위)
1. **known-only A\*** (`allow_unknown=False`) on inflated 최종 지도 + LOS smoothing.
2. 실패 → **dynamic 의심 셀(최근 생긴 점유) 리셋** 후 재계획 (03 §A.5).
3. 실패 → **인플레이션 반경 축소**(r_robot + 작은 margin) 재계획 (속도 제한 걸고).
4. 실패 → **breadcrumb 경로**: 지나온 궤적(셀 리스트)을 역순으로 — 로봇이 실제로 지나간 곳이라 정적으로는 통과 가능 (GBPlanner `homing_backward`, SemExp "방문 셀은 통과 가능" 개념). 루프 제거(같은 셀 재방문 구간 삭제) 후 smoothing.
5. 실패 → `allow_unknown=True` 낙관적 계획 (FAR attemptable 모드와 같은 취지).
6. 전부 실패 → 제한된 recovery ladder(06), 안전 정지 중 약 1 Hz 상태 확인; 새 정보가 생길 때만 재계획하고 미션 종료 시 실패 기록.

### 2.4 도착 및 방향
- 위치: `HOME_TOLERANCE`(현재 0.10 m) — 규정 기준에 맞춤.
- 방향: 규정이 요구하면 도착 후 제자리 회전으로 `home_pose.theta` 정렬 (RPP의 rotate-to-goal-heading ✅과 같은 동작). 허용오차 0.1~0.25 rad (Nav2 `yaw_goal_tolerance 0.25`).
- 도착 후 DONE: 모터 정지 + 최종 지도/target 목록 덤프(심사용 증거).

### 2.5 m-explore-ros2 대비 개선점 요약

| m-explore-ros2 한계 | 우리 |
|---|---|
| 탐색이 "끝나야만" 복귀 | 시간 예산 트리거가 항상 감시 |
| 실패 시 아무것도 안 함 | 6단계 경로 전략 + recovery |
| TF 실패 시 기능 꺼짐 | 시작 pose는 local 원점, 제공값은 world 변환에만 사용 |
| 목표물 개념 없음 | target 접근 결정에도 ETA 사용 |

## 3. 테스트
- ETA: 알려진 맵에서 거리장 기반 L_home 정확.
- 트리거: time_left를 줄여가며 경계값에서 전이.
- 경로 폴백: 1단계 실패 맵(좁은 통로) → 인플레이션 축소 또는 breadcrumb로 성공.
- breadcrumb 루프 제거.
- 도착 후 heading 정렬 (요구 시).
- target 접근 포기 조건.


### 안전 불변 조건

Inflation 축소는 여유분만 줄이고 검증된 물리 footprint(보수적 원 0.111 m 시작)보다 작게 하지 않는다. 최근 hit·충돌 증거는 ghost clear/방문 셀 보정에서 제외한다. Breadcrumb은 과거 통과 기록이지 현재 장애물 부재 보장이 아니다. unknown 허용 경로는 탐색 후보이며 현재 센서로 다음 이동 구간이 검증되지 않으면 전진하지 않는다. 10 §2.2의 bounded planning 및 sensor-age 정지 조건을 그대로 따른다.
