# 08. Return Home (시간 예산 기반 복귀)

> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정

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
- 현재 `do_initialize`에서 `config.START_POSE`로 설정 (대회가 시작 pose를 제공하므로 TF 조회 실패 같은 문제 없음). 유지.

### 2.2 트리거 (Q11, GBPlanner 식을 우리 규모로)

```text
ETA_home   = L_home / v_avg + N_turns * (π/2)/ω + T_margin_fixed
L_home     = distance_field(from robot)[home_cell] * res     # 04의 거리장, known-only
v_avg      = 실측 평균 주행속도 (로그에서 갱신, 초기값 0.6 * MAX_LINEAR_SPEED)
trigger if  time_left < SAFETY_FACTOR * ETA_home + T_margin
            SAFETY_FACTOR 1.3~1.5, T_margin 10~20 s   (GBPlanner는 +20 s 고정)
```
- 매 1 Hz 계획 주기에 계산 (거리장을 이미 계산하므로 추가비용 ≈ 0).
- target 접근 결정에도 사용: `time_left < SF * (ETA_target + ETA_target→home) + margin` 이면 접근 포기(위치만 기록).
- 현재 baseline의 고정 `MISSION_TIME_LIMIT` 비교보다 **맵이 클수록/멀리 있을수록 일찍 복귀**하므로 안전.
- ⚠️ 시간 기준(시뮬레이션 시간 vs 실시간)은 당일 규정으로 확인. 비동기 모드라면 둘이 다를 수 있음 (09 참고).

### 2.3 경로 전략 (우선순위)
1. **known-only A\*** (`allow_unknown=False`) on inflated 최종 지도 + LOS smoothing.
2. 실패 → **dynamic 의심 셀(최근 생긴 점유) 리셋** 후 재계획 (03 §A.5).
3. 실패 → **인플레이션 반경 축소**(r_robot + 작은 margin) 재계획 (속도 제한 걸고).
4. 실패 → **breadcrumb 경로**: 지나온 궤적(셀 리스트)을 역순으로 — 로봇이 실제로 지나간 곳이라 정적으로는 통과 가능 (GBPlanner `homing_backward`, SemExp "방문 셀은 통과 가능" 개념). 루프 제거(같은 셀 재방문 구간 삭제) 후 smoothing.
5. 실패 → `allow_unknown=True` 낙관적 계획 (FAR attemptable 모드와 같은 취지).
6. 전부 실패 → recovery ladder(06) 반복, 안전 정지 유지하며 1 Hz 재시도.

### 2.4 도착 및 방향
- 위치: `HOME_TOLERANCE`(현재 0.10 m) — 규정 기준에 맞춤.
- 방향: 규정이 요구하면 도착 후 제자리 회전으로 `home_pose.theta` 정렬 (RPP의 rotate-to-goal-heading ✅과 같은 동작). 허용오차 0.1~0.25 rad (Nav2 `yaw_goal_tolerance 0.25`).
- 도착 후 DONE: 모터 정지 + 최종 지도/target 목록 덤프(심사용 증거).

### 2.5 m-explore-ros2 대비 개선점 요약

| m-explore-ros2 한계 | 우리 |
|---|---|
| 탐색이 "끝나야만" 복귀 | 시간 예산 트리거가 항상 감시 |
| 실패 시 아무것도 안 함 | 6단계 경로 전략 + recovery |
| TF 실패 시 기능 꺼짐 | 시작 pose는 config 제공값 |
| 목표물 개념 없음 | target 접근 결정에도 ETA 사용 |

## 3. 테스트
- ETA: 알려진 맵에서 거리장 기반 L_home 정확.
- 트리거: time_left를 줄여가며 경계값에서 전이.
- 경로 폴백: 1단계 실패 맵(좁은 통로) → 인플레이션 축소 또는 breadcrumb로 성공.
- breadcrumb 루프 제거.
- 도착 후 heading 정렬 (요구 시).
- target 접근 포기 조건.
