# 06. Recovery · Progress Monitoring · Blacklist

> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정

---

## 1. 실제 스택들이 쓰는 방식 (코드 기준)

### 1.1 Nav2 (ROS 2) ✅
- **Progress checker** [`simple_progress_checker.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp): 기준 pose에서 `required_movement_radius`(0.5 m) 이상 벗어나면 기준 갱신, `movement_time_allowance`(10 s) 안에 못 벗어나면 실패. **목표까지 거리와 무관한 "변위" 기반.** [`pose_progress_checker.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/pose_progress_checker.cpp)는 회전 0.5 rad도 진전으로 인정.
  - TurtleBot3는 `required_movement_radius 0.1` ✅ (느린 로봇에 맞춰 축소). webots_ros2 e-puck 예제는 0.5 m/10 s 그대로 — e-puck `max_vel_x 0.05`에선 딱 한계값이라 부적절한 복사로 보임.
- **Goal checker** [`simple_goal_checker.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/simple_goal_checker.cpp): xy 0.25, yaw 0.25, `stateful`.
- **BT** [`navigate_to_pose_w_replanning_and_recovery.xml`](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml):
  - 전체 `RecoveryNode number_of_retries="6"`.
  - 계획 실패 → (도움이 되는 오류면) global costmap 비우기 후 1회 재시도. 추종 실패 → local costmap 비우기 후 1회 재시도.
  - 그래도 실패 → `RoundRobin`: **① 양쪽 costmap 비우기 → ② Spin 1.57 rad → ③ Wait 5 s → ④ BackUp 0.30 m @0.15 m/s**, 목표가 바뀌면 중단.
- **Behaviors** [`nav2_behaviors/plugins`](https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors/plugins): Spin/BackUp/DriveOnHeading 모두 `simulate_ahead_time`(2.0 s)으로 **충돌 검사 후 실행**, 충돌 예상 시 `COLLISION_AHEAD`로 실패.

### 1.2 ROS 1 move_base ✅ [`move_base.cpp`](https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp)
- `planner_patience 5 s`, `controller_patience 15 s`, `max_planning_retries −1`.
- **Oscillation**: `oscillation_distance 0.5 m` 이상 움직이지 못한 채 `oscillation_timeout`(기본 0 = 비활성)이 지나면 recovery.
- 기본 recovery **단계적 강화**: `conservative_reset`(3 m 밖 장애물 지우기) → rotate → `aggressive_reset`(외접반경×4 밖 지우기) → rotate.

### 1.3 m-explore ✅ (02 참고)
- Recovery 자체는 없음. **frontier를 blacklist**하고 다른 frontier로.
- progress timeout 30 s, blacklist는 centroid ±5셀 사각형, 영구.
- ROS 2: `resuming_` 플래그로 pause/resume 직후 오판 방지.

### 1.4 SemExp (ObjectNav) ✅ [`sem_exp.py`](https://github.com/devendrachaplot/Object-Goal-Navigation/blob/master/agents/sem_exp.py) `_plan`
- 전진 명령 후 이동 < `collision_threshold`(0.20, 코드 단위 기준) → 로봇 **앞쪽 셀들을 `collision_map`에 장애물로 기록**, 반복 실패 시 폭 확장(1→3→5).
- 센서에 안 보이는 장애물(낮은 턱, 센서 평면 아래 물체)을 **"부딪힌 경험"으로 지도에 새김**. 2D LiDAR 평면 아래 장애물에 매우 유용.

### 1.5 RoboCupJunior Rescue Simulation (Erebus, Webots) ✅ [`MainSupervisor.py`](https://github.com/robocup-junior/erebus/blob/master/game/controllers/MainSupervisor/MainSupervisor.py), [`Robot.py`](https://github.com/robocup-junior/erebus/blob/master/game/controllers/MainSupervisor/Robot.py)
- 심판 supervisor가 로봇이 **20 s 이상 정지하면 자동 Lack-of-Progress** → 마지막 체크포인트로 재배치(감점). 정지 판정은 속도 성분 모두 < 0.001.
- 즉 대회 환경에선 **"멈춰서 오래 생각하는 것" 자체가 벌점**일 수 있다 → 우리 recovery의 STOP/WAIT에 상한 시간이 필요 (규정 확인 필요 ⚠️).

## 2. 비교: m-explore blacklist vs Nav2 recovery vs 우리 제안

| 상황 | m-explore | Nav2 | 우리 제안 |
|---|---|---|---|
| 진전 없음 | 30 s 후 frontier blacklist | 10 s 내 0.5 m 못 움직이면 controller 실패 → recovery | **8~10 s 내 경로잔여 −0.05 m 또는 변위 0.1 m 없으면** NO_PROGRESS → ladder |
| 진동 | 없음 | DWB Oscillation critic / move_base oscillation | 최근 N초 순변위 < 0.05 m & 명령은 계속 있음 → OSCILLATION → ladder |
| 경로 막힘 | Nav2가 처리 | 계획 재시도 + costmap clear | 05 WAIT(3~5 s) → 재계획 → ladder |
| 도달 불가 goal | ABORTED → blacklist | 계획 실패 → 재시도 6회 | 거리장 ∞ → 즉시 후보 제외, relax_goal 실패 → blacklist |
| 끼임(stuck) | – | BackUp 0.30 m | 후방 검사 후 BackUp 0.05~0.08 m + Spin 90° |
| 보이지 않는 장애물 | – | – | SemExp식 collision_map 기록 |
| localization 이상 | – | (AMCL 수준) | gyro-odom 불일치/매칭 점수 하락 → 감속 + 제자리 회전 재관측 |
| 모든 방향 위험 | – | behavior가 COLLISION_AHEAD로 실패 | **STOP 후 주기적 재평가**, 절대 강행 금지, 단 LoP 규정 고려해 최대 대기 시간 설정 |

## 3. 우리 Recovery Ladder (navigator 내부 하위 상태, mission state 아님)

| 단계 | 동작 | 진입 조건 | 종료/다음 |
|---|---|---|---|
| R0 WAIT | 정지 2~3 s, 스캔 갱신 | BLOCKED(새 장애물), 첫 NO_PROGRESS | 경로 유효해지면 복귀, 아니면 R1 |
| R1 CLEAR + REPLAN | 로봇 반경 0.3 m 안 log-odds를 prior로 리셋(ghost 제거) → 재계획 | R0 실패 | 경로 있으면 복귀, 아니면 R2 |
| R2 SPIN | 가장 열린 방향으로 90°~180° 회전(360° LiDAR라 회전 자체는 안전, 카메라 재관측 효과) | R1 실패, OSCILLATION | 재계획 → 실패 시 R3 |
| R3 BACKUP | 후방 부채꼴 점검 후 0.05~0.08 m 후진 (Nav2 0.30 m를 e-puck 규모로 축소) | R2 실패, 전방 끼임 | 재계획 → 실패 시 R4 |
| R4 MARK & GIVE UP GOAL | 앞 셀을 collision_map에 기록(SemExp), 현재 goal blacklist(TTL) | R3 실패 | 상위(mission)에 GOAL_FAILED → 다른 frontier/target |
| R5 SAFE STOP | 모든 방향 위험 → 정지, 2 s마다 재평가 | 안전한 동작 없음 | 공간 생기면 R0부터 |

- 각 단계는 **안전 모니터 통과가 전제**. 모든 동작은 Nav2 behavior처럼 실행 전 짧은 전방/후방 시뮬레이션.
- ladder 카운터는 goal 변경 또는 성공적 진전(0.1 m 이동) 시 리셋 (move_base: oscillation reset 시 recovery index 리셋과 같은 방식).
- 같은 goal에서 ladder를 2회 완주하면 무조건 R4.

## 4. Progress Monitor 설계 (control.py `ProgressMonitor`)

```text
reset(goal, now, pose)
update(remaining_path_len, pose, commanded_v, now) -> OK | NO_PROGRESS | STUCK | OSCILLATION
  - best = min(best, remaining)                    # 최선값(진동 강함)
  - if remaining < best_prev - DELTA(0.05): t_prog = now
  - if |pose - anchor| > RADIUS(0.10): anchor = pose; t_move = now     # Nav2식 변위
  - if now - max(t_prog, t_move) > TIMEOUT(8~10 s):
        return STUCK if 변위≈0 and commanded_v > 0 else NO_PROGRESS
  - 회전 중(recovery SPIN / 제자리 회전)에는 판정 보류 (PoseProgressChecker처럼 회전도 진전 인정)
  - APPROACH_TARGET → EXPLORE 복귀 직후 grace period (m-explore-ros2 resuming_ 개념)
```

## 5. Blacklist 설계 (planning.py `FrontierBlacklist`)
- 항목: `(x, y, t_added, reason)`; 반경 0.25 m(유클리드); TTL 60~90 s; 모두 blacklist면 1회 전체 초기화(second chance).
- target goal도 같은 구조 재사용(도달 실패 target은 "나중에 다시").

## 6. 테스트
- NO_PROGRESS: 잔여거리 불변 10 s → 이벤트.
- 회전 중에는 이벤트 없음.
- OSCILLATION: 좌우 왕복 궤적(순변위 0) → 이벤트.
- ladder: 각 단계 실패 시 다음 단계, 성공 시 리셋, 2회 완주 → R4.
- 후방 장애물 있으면 BACKUP 스킵.
- blacklist TTL, second chance.
