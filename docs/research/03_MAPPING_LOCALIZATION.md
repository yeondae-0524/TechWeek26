# 03. Mapping (dynamic obstacle 대응) & Localization

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> ✅ 소스 확인 · 📄 논문/문서만 · ⚠️ 미확인/추정. 수치 중 "initial tuning suggestion"은 레퍼런스 값을 우리 로봇 규모로 옮긴 **시작값**이지 검증값이 아니다.
> **2026-09-30 공식 TECH WEEK repo 기준 재검증.** 태그: [ORGANIZER] [OFFICIAL] [DERIVED] [MEASURED] [REFERENCE] [INITIAL TUNING] [DAY-OF]. 공식 환경 사실 전체는 [09](09_WEBOTS_REFERENCES.md).

### 공식 환경 요약 (이 문서에 영향을 주는 것)
| 항목 | 값 | 태그 |
|---|---|---|
| 로봇 | TurtleBot3Burger, 바퀴 반경 0.033 m, 간격 0.160 m, 반경 0.105 m | [OFFICIAL] |
| 필수 센서 | Wheel encoder, 2D LiDAR | [ORGANIZER] |
| IMU | 선택 (우리: TEAM CHOICE gyro ENABLED, 폴백 있음) | [ORGANIZER] / 팀 선택 |
| 미사용 | Compass, GPS | [ORGANIZER] |
| Supervisor 정답 pose | 공식 `tb3_ground_truth` 데모 전용 — competition 입력 아님 | [OFFICIAL] 예제 |
| LiDAR | `LDS-01`, 360 samples, minRange 0.12, maxRange 3.5 m, noise 0.0043(σ≈0.015 m), 로봇 frame (−0.03, 0, 0.173) | [OFFICIAL]/[DERIVED] |
| LiDAR 순서 | index 0=Back, 90=Left, 180=Front, 270=Right → `angle_i ≈ π − i·FOV/N` (우리 config의 방향·대표 index와 부합; sub-degree 정렬 S0 검증) | [OFFICIAL] 예제 + [DERIVED] |

---

## A. Mapping

### A.1 현재 baseline의 문제 (코드 기준)

`controllers/rescue_robot/mapping.py`:

```python
def _mark_free(self, row, col):
    # An observed obstacle is never erased by a later free ray (TODO: log-odds).
    if self.in_bounds(row, col) and self.grid[row][col] != OCCUPIED:
        self.grid[row][col] = FREE
```

- 한 번 OCCUPIED가 된 셀은 **영원히** OCCUPIED → 지나간 사람이 **영구 벽(ghost)** 으로 남는다.
- 결과: (1) frontier가 막혀 탐색 조기 종료, (2) **home path가 막혀 복귀 실패**, (3) 좁은 통로 영구 봉쇄.
- 반대로 "last write wins"로 바꾸면 LiDAR 노이즈/grazing ray에 벽이 깎인다. → **확률적 누적(log-odds) + clamping**이 표준 해법.

### A.2 레퍼런스별 처리 방식

| 구현 | 표현 | 갱신 규칙 | 동적 장애물 해제 | 출처 |
|---|---|---|---|---|
| **OctoMap** | log-odds | hit +0.85 (p=0.7), miss −0.4 (p=0.4), **clamp [−2.0, +3.5]** (p 0.12~0.97), 점유 임계 0 | clamp 덕분에 최대 확신 상태에서도 **약 9번의 miss**면 비점유로 전환 | ✅ [`AbstractOccupancyOcTree.cpp`](https://github.com/OctoMap/octomap/blob/devel/octomap/src/AbstractOccupancyOcTree.cpp) L42-47 |
| **Hector mapping** | log-odds | free `logit(0.4)`, occupied `logit(0.9)`(ROS 노드 기본); 셀당 **스캔 1회만 갱신**(updateIndex), 같은 스캔에서 hit면 free 갱신 취소(`updateUnsetFree`); 점유 상한 50, **free 하한 없음** | 레이 통과 시 자연 감소 (하한이 없어 오래 free였던 셀은 재점유가 늦음 → 우리에겐 하한 필요) | ✅ [`GridMapLogOdds.h`](https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/noetic-devel/hector_mapping/include/hector_slam_lib/map/GridMapLogOdds.h), [`OccGridMapBase.h`](https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/noetic-devel/hector_mapping/include/hector_slam_lib/map/OccGridMapBase.h), [`HectorMappingRos.cpp`](https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/noetic-devel/hector_mapping/src/HectorMappingRos.cpp) |
| **Nav2 ObstacleLayer** | 이진 costmap | `marking`/`clearing` observation source, **raytrace로 비움** (`raytrace_max_range 3.0`, `obstacle_max_range 2.5` 기본) | 다음 스캔 레이가 지나가면 즉시 삭제 | ✅ [`obstacle_layer.cpp`](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/obstacle_layer.cpp), [`nav2_params.yaml`](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml) |
| **STVL** | voxel + 시간 | voxel마다 만료 시각, `voxel_decay`(linear/exp/persistent) | 시간 경과로 소멸, frustum 가속 (README: 레이저는 `decay_acceleration` 0) | ✅ [`spatio_temporal_voxel_grid.cpp`](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/ros2/spatio_temporal_voxel_layer/src/spatio_temporal_voxel_grid.cpp), README |
| **FAR planner** | 점군 + grid | 현재 스캔 레이가 **관통하는 기존 장애물 점 = dynamic**으로 분류 → grid에서 제거, `dynamic_obs_dacay_time 10 s`, `dyosb_update_thred 4`점 이상일 때만 | ray-casting 기반 dynamic 분리 | ✅ [`far_planner.cpp`](https://github.com/MichaelFYang/far_planner/blob/melodic-noetic/src/far_planner/src/far_planner.cpp) `ExtractDynamicObsFromScan`, [`config/default.yaml`](https://github.com/MichaelFYang/far_planner/blob/melodic-noetic/src/far_planner/config/default.yaml) |
| webots_ros2 e-puck `simple_mapper.py` | 1 cm 이진 | 점유만 표시 (free 없음) | 없음 | ✅ [`simple_mapper.py`](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/simple_mapper.py) — 참고 가치 낮음 |

이론: Moravec & Elfes 1985 [doi:10.1109/ROBOT.1985.1087316](https://doi.org/10.1109/ROBOT.1985.1087316); Thrun·Burgard·Fox, *Probabilistic Robotics* (2005) Ch.9; clamping 정책: Yguel et al., "Update Policy of Dense Maps," [doi:10.1007/978-3-540-75404-6_3](https://doi.org/10.1007/978-3-540-75404-6_3) 📄; OctoMap 논문 Hornung et al. 2013 [doi:10.1007/s10514-012-9321-0](https://doi.org/10.1007/s10514-012-9321-0).

### A.3 비교: binary vs log-odds vs temporal decay

| 방식 | 구현 난이도 | ghost 해제 | 노이즈 강건성 | 벽 침식 위험 | 가려진 ghost | 우리 평가 |
|---|---|---|---|---|---|---|
| 현재 binary (occupied 영구) | – | ✗ 영구 | 중 | 없음 | ✗ | **위험** |
| binary + raytrace clearing (Nav2식) | LOW | 즉시 | 낮음 (1회 노이즈로 깜빡임) | 중 | ✗ | 가능하나 깜빡임 |
| **log-odds + clamp (OctoMap/Hector식)** | LOW~MED | ~1–2 s | **높음** | 낮음(스캔당 1회 갱신 + hit 우선) | ✗ (보이는 레이 필요) | **추천 (P0/S4)** |
| + dynamic layer (FAR식 분리 + TTL) | MED | 즉시 분리, TTL 소멸 | 높음 | 낮음 | TTL로 해제 | P2 |
| temporal decay 전체 적용 (STVL식) | MED | 시간 경과 | 중 | **정적 벽도 사라짐** (재관측 필요) | ✓ | 2D 360° LiDAR엔 불필요 |

### A.4 추천 설계 (mapping.py 내부 변경, 외부 인터페이스 `grid[row][col] ∈ {-1,0,1}` 유지)

```text
내부 상태: logodds[row][col] (float, 0.0 = prior), observed[row][col] (bool)
insert_scan(pose, ranges, ...):
    hits, misses = set(), set()
    for each ray:
        if invalid_or_inf_or_out_of_range(ray): continue  # no-return 원인 불명: free로 만들지 않음
        cells = bresenham(sensor_origin_cell, end_cell)  # mount 반영, minRange 이전 blind cells 제외
        misses.update(cells[:-1] if hit else cells)       # 끝점 직전까지
        if hit: hits.add(end_cell)
    misses -= hits                                        # Hector: 같은 스캔의 hit 우선
    for c in misses: L[c] = max(L_MIN, L[c] + L_MISS); observed[c]=True
    for c in hits:   L[c] = min(L_MAX, L[c] + L_HIT);  observed[c]=True
    export: grid[c] = UNKNOWN if not observed[c] else (OCCUPIED if L[c] > L_OCC else FREE if L[c] < L_FREE else 이전값 유지)
```

시작값 (OctoMap 기본값 이식, initial tuning suggestion). `reset_region`은 logodds=0과 observed=False를 함께 갱신해 UNKNOWN으로 내보내며 FREE를 조작하지 않는다:

| 파라미터 | 값 [REFERENCE: OctoMap → INITIAL TUNING] | 의미 (스캔 간격 ≈ 64–128 ms 가정) |
|---|---|---|
| `L_HIT` | +0.85 | unknown → 1 hit이면 점유 |
| `L_MISS` | −0.40 | unknown → 1 miss면 free |
| `L_MAX` | +3.5 | 최대 확신 장애물도 **10 miss ≈ 0.64–1.28 s** 관측이면 해제 |
| `L_MIN` | −2.0 | 오래 free였던 셀도 **3 hit ≈ 0.19–0.38 s**면 점유 (raw scan monitor가 별도로 반응하지만 blind zone 안전을 보장하지는 않음) |
| `L_OCC`, `L_FREE` | +0.4 / −0.2 | 약한 hysteresis로 깜빡임 억제 |

주의점:
- **grazing ray 벽 침식**: 끝점 셀과 그 8-이웃은 miss 갱신에서 제외하는 옵션을 두고 unit test로 비교.
- **가려진 ghost**(사람이 서 있던 자리 뒤쪽에서만 보이는 경우)는 log-odds로 안 지워진다 → RECOVERY에서 "현재 유효 hit·충돌 증거를 제외한 주변 ghost 의심 셀을 prior(0), observed=False로 리셋"하는 **costmap clearing 대응 동작**을 둔다 (Nav2 BT의 `ClearEntireCostmap`, move_base의 `conservative_reset`/`aggressive_reset`과 같은 역할; 06 참고).
- 성능: **기존 binary** `insert_scan`은 360 rays, 최대 3.5 m에서 **6–12 ms** [MEASURED, 09 §8]. 새 log-odds/set 처리는 미측정이다. 시작안은 128 ms마다 새 scan 삽입 [INITIAL TUNING], S0/S4 계측 후 조절 (LiDAR를 `enable(timestep)` 또는 2×timestep으로; 공식 예제의 `enable(100)`은 실제 간격 UNCONFIRMED). (구 문서의 "5 Hz = 16 ms step의 12 step" 권고는 e-puck practice 전제라 폐기.)
- **LiDAR mount offset**: 공식 TB3에서 LiDAR는 회전중심 뒤 0.03 m, 높이 ≈0.173 m [DERIVED] → 스캔 투영에 `(−0.03, 0)` 적용. 평탄한 자세에서 LiDAR 평면 아래 물체는 지도에 **나타나지 않을 수 있다** (저위 장애물·바닥 target) → 06 collision map, 07 카메라 거리 추정이 이를 보완.
- **해상도**: 0.05 m 유지 [INITIAL TUNING]. 로봇 반경 0.105 m ≈ 2.1셀, 인플레이션 0.16 m ≈ 3.2셀. 공식 예제 월드(13 m급)를 담으려면 grid 20 m(400셀) 이상을 시작 후보로 검토하되 시작점 편향과 실제 경계에 따라 더 커야 할 수 있음 [INITIAL TUNING] → planning 비용은 04 참조.

### A.5 동적 장애물의 지도 반영 원칙 (자세한 행동 정책은 05)

1. **안전 판단은 지도가 아니라 raw scan** (Nav2 Collision Monitor 원칙: costmap/planner를 우회해 센서 데이터로 직접 판단).
2. 지도(log-odds)는 사람을 **일시적으로** 반영 → 유한 free ray로 재관측될 때만 해제된다. 제안 임계에서는 10 miss, 128 ms 시작안이면 약 1.28 s이며 가림/inf가 지속되면 해제시간 보장 없음.
3. 복귀 경로 계획 전에는 "오래된(dynamic 의심) 점유 셀"을 한 번 더 의심: 경로가 없으면 **ghost 의심 셀만 prior로 되돌리고 재계획** (FAR의 dynamic 분리 개념의 단순판).

---

## B. Localization

### B.1 공식 로봇 센서 사실관계 (TurtleBot3Burger R2025a, ✅ PROTO + 공식 controller)

| 센서 | 값 | 사용 | 함의 |
|---|---|---|---|
| Wheel encoder | `left/right wheel sensor`, resolution **0.00628 rad** → 바퀴 0.033 m에서 0.21 mm/tick [OFFICIAL/DERIVED]; 공식 예제는 `motor.getPositionSensor()` | **필수** [ORGANIZER] | 양자화 오차 무시 가능. 오차원은 **미끄러짐**(제자리 회전·충돌) |
| 차동구동 식 | `Δs = r(Δφ_r+Δφ_l)/2`, `Δθ = r(Δφ_r−Δφ_l)/L`, r=0.033, L=0.160; 양의 바퀴 회전 = 전진 [OFFICIAL/DERIVED] | 필수 | 우리 `integrate_diff_drive`(midpoint)와 동일 식 — config 값만 교체 |
| Gyro | `gyro`, **lookupTable 없음 → raw rad/s**, noise 필드 없음 [OFFICIAL] | **TEAM CHOICE: ENABLED** (IMU 선택 [ORGANIZER]) | yaw rate(z축) 직접 관측 → 회전 미끄러짐 보정. `GYRO_RAW_TO_RAD_S = 1.0` |
| Accelerometer | `accelerometer`, raw m/s² [OFFICIAL] | 선택 (충돌 감지 보조) | 적분 금지 |
| Compass | PROTO에 존재, 공식 예제가 읽음 | **사용 안 함** [ORGANIZER] | competition controller에서 비활성 |
| GPS | 공식 TB3 PROTO에 없음 | **사용 안 함** [ORGANIZER] | 기존 "GPS debug로 drift 측정" 계획 **폐기** |
| Supervisor pose | `tb3_ground_truth` 데모(supervisor TRUE) | competition 입력 아님 | 개발용 평가 도구로만(분리 모드, 팀 합의) |
| LiDAR | `LDS-01` 360 samples, 0.12–3.5 m, σ≈0.015 m, offset (−0.03, 0), 높이 ≈0.173 m | **필수** [ORGANIZER] | mapping·(조건부) scan matching |

좌표 convention: 로봇 +x 전방, +y 좌측 [DERIVED]; 현재 코드의 world ENU convention은 그대로 기록한다. **추천 local map**은 시작 heading을 +x, 좌측을 +y, theta=0을 시작 방향으로 두고 CCW 양수, row=+y/col=+x, home=(0,0,0)이다. 제공 world 시작 pose는 정적 변환에만 사용하며 형식은 [DAY-OF]. 이는 팀 내부 convention이며 공식 절대 north 요구가 아니다.

(practice e-puck 기록: 엔코더 동일 resolution, gyro lookupTable ±13.315805 ↔ ±100000, 연습 world LiDAR offset 0.0095 m — **대회 튜닝에 사용하지 않음**.)

### B.2 선택지 비교

| 방법 | 원리 | 우리 구현 난이도 | CPU(Python) | 강점 | 약점 | 출처 | 판정 |
|---|---|---|---|---|---|---|---|
| Wheel odometry (현재) | 적분 | 완료 | 무시 | 단순 | 회전 시 미끄러짐으로 heading drift | – | **필수 base** (encoder 필수 [ORGANIZER]) |
| UMBmark 보정 | 사각 경로 왕복으로 r/L 보정 | LOW | – | 체계 오차 제거 | 대회 당일 시간 필요 | Borenstein & Feng 1996 [doi:10.1109/70.544770](https://doi.org/10.1109/70.544770) 📄, webots_ros2 [`drive_calibrator.py`](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/drive_calibrator.py) ✅ (e-puck용 절차 참고만) | **P1 (당일 절차)** |
| **Gyrodometry** | Δθ는 gyro, 거리는 encoder (불일치 클 때 gyro 채택) | **LOW** | 무시 | heading drift 대폭 감소 | gyro bias → 정지 중 평균으로 추정 | Borenstein & Feng, "Gyrodometry," ICRA 1996 [doi:10.1109/ROBOT.1996.503813](https://doi.org/10.1109/ROBOT.1996.503813) 📄 | **TEAM CHOICE: ENABLED** (IMU 선택 [ORGANIZER], 폴백 필수) |
| Slip/충돌 감지 | \|ω_gyro−ω_odom\|, 가속도 spike | LOW | 무시 | recovery 트리거 | 임계 튜닝 | 동일 | P1 |
| **Correlative Scan Matching (scan→map)** | (x,y,θ) 창을 brute-force로 훑어 smeared map 위 점수 최대, **odom prior 페널티** | MED | 추정 5~20 ms (numpy, 창 ±0.1 m/±6°, 90점) ⚠️ | 전역 최적(창 내), 구현 단순, 실패 판정 쉬움(점수) | 창 밖 오차 불가, 맵 오염 시 동반 오염 | Olson 2009 [doi:10.1109/ROBOT.2009.5152375](https://doi.org/10.1109/ROBOT.2009.5152375) 📄, Karto(slam_toolbox) ✅ | **P2 (측정 후 결정)** |
| Hector식 Gauss-Newton scan→map | 맵 bilinear 보간 기울기로 최적화, 다해상도 | MED~HIGH | 중 | odom 불필요 | 초기값 민감, 회전 큰 경우 발산 | Kohlbrecher 2011 [doi:10.1109/SSRR.2011.6106777](https://doi.org/10.1109/SSRR.2011.6106777) ✅ | 대안 |
| ICP / PL-ICP (scan→scan) | 점 대응 반복 | MED | 중 | 정밀 | 복도/대칭 환경에서 미끄러짐, 대응 오류 | Censi 2008 [doi:10.1109/ROBOT.2008.4543181](https://doi.org/10.1109/ROBOT.2008.4543181) 📄, PythonRobotics [`icp_matching.py`](https://github.com/AtsushiSakai/PythonRobotics/blob/master/SLAM/ICPMatching/icp_matching.py) ✅ | 비추천 (CSM이 더 견고) |
| KISS-ICP | 적응 임계 point-to-point ICP, 3D 중심 | – | – | – | pip 의존성(C++ 코어) = 새 dependency | Vizzo 2023 [doi:10.1109/LRA.2023.3236571](https://doi.org/10.1109/LRA.2023.3236571) ✅ repo 확인 | DO NOT (의존성) |
| Pose graph + loop closure | 최적화 | HIGH | 높음 | 대규모 맵 일관성 | 해커톤 규모 과잉 | slam_toolbox ✅, Cartographer (Hess 2016 [doi:10.1109/ICRA.2016.7487258](https://doi.org/10.1109/ICRA.2016.7487258)) 📄 | **DO NOT** |
| Particle filter (AMCL) | 사전 지도 필요 | – | – | – | 사전 지도 없음 | – | **DO NOT** |

### B.3 Karto CSM 핵심 (slam_toolbox 코드로 확인 ✅)

- 파일: [`lib/karto_sdk/src/Mapper.cpp`](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/lib/karto_sdk/src/Mapper.cpp) — `ScanMatcher::MatchScan`, `CorrelateScan`, `GetResponse`, `ComputePositionalCovariance`
- `GetResponse`: 각 스캔 점이 떨어지는 correlation grid(가우시안 smear된 점유 맵) 값을 합산 → `/ (N × occupied값)`로 0~1 정규화.
- odom prior: `response *= (1 − k·d²/σ_d²) × (1 − k·Δθ²/σ_θ²)` (각각 최소값 clamp) — **오도메트리에서 먼 해를 깎는다**.
- 기본값 ([`mapper_params_online_async.yaml`](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)): 탐색창 0.5 m, 분해능 0.01 m, smear 0.1 m, coarse 각도 ±0.349 rad @0.0349, fine @0.00349, `minimum_travel_distance 0.5`, `minimum_travel_heading 0.5`.
- 우리 축소판(initial suggestion): 창 ±0.10 m @0.02 m, ±6° @1°, 스캔 90점 다운샘플, smear σ=1셀, **이동 0.05 m 또는 0.1 rad마다 실행**, 점수 < τ 또는 보정량 > 게이트면 기각(odom 유지).

### B.4 권장 조합 (우리 수준에서 현실적인 것)

1. **Base (필수)**: encoder differential-drive odometry — 공식 TB3 값(r 0.033, L 0.160)으로 config 이행 후 검증.
2. **Team choice (ENABLED 권장)**: gyro heading fusion — 정지 1 s gyro bias 평균(INITIALIZE), Δθ를 gyro로 대체(또는 \|ω_gyro−ω_odom\| 클 때만 채택). gyro 장치 부재/비정상 → **encoder-only 폴백** (pose 인터페이스 동일). IMU 사용은 대회 요건이 아니라 **팀 설계 선택**이다.
3. **P1**: 당일 오도메트리 보정 절차 — GPS 없이: (a) 1 m 명령 직진 전후 전방 벽까지 LiDAR 거리 변화 vs odometry, (b) 제자리 360° 명령 회전 전후 스캔 정합 각도 오차 + slip 감지 플래그.
4. **Conditional (scan matching)**: GPS/Compass 없이 다음 테스트로 판단 [INITIAL TUNING 기준]:
   - **Loop closure test**: 사각/왕복 경로로 시작점 복귀 → 첫 스캔과 마지막 스캔을 오프라인 CSM으로 정합한 오프셋 ≥ 0.10 m 또는 ≥ 3°
   - **Map consistency**: 같은 벽이 두 줄로(1셀 이상 어긋나게) 그려짐 / 모서리 번짐
   - **Repeated-place residual**: 같은 장소 재방문 시 스캔-맵 잔차 증가
   - **Home return consistency**: 미션 끝 home에서 초기 스캔과의 정합 오프셋
   - 반복 시험에서 확인되면 numpy CSM A/B 평가(P2); 동적 장애물·대칭 오정합을 제거하고 drift/비용 개선이 입증될 때만 도입한다. 명령 이동량은 ground truth가 아니다. (선택) Supervisor 정답 비교는 개발용 분리 모드에서만, 팀 합의 시.
   - 참고: 공식 계획안이 **Scan Matching을 활용 가능 기술로 명시** [OFFICIAL] → '기술 구현' 점수 차원의 도입은 팀 결정.
5. **DO NOT**: loop closure 최적화, pose graph, particle filter, ICP scan-to-scan 단독, **Compass/GPS/Supervisor pose 입력**.

### B.5 Localization 실패 감지 (P2)

- CSM을 넣었다면 매칭 점수 이동평균이 급락 → "localization degraded" → 속도 제한 + 제자리 회전으로 재관측.
- CSM이 없다면: gyro-odom 불일치 누적, 가속도 spike(충돌) 발생 시 플래그 → 해당 구간 맵 갱신 일시 중지(Hector가 `map_update_distance_thresh 0.4 m`/`angle 0.9 rad`로 갱신 빈도를 제한하는 것과 같은 취지: 나쁜 포즈로 맵을 오염시키지 않기).

## C. 테스트 계획 (Webots 없이)

| 테스트 | 기대 |
|---|---|
| 같은 셀에 hit 1회 | OCCUPIED |
| OCCUPIED(L_MAX) 셀을 레이가 10회 관통 | FREE로 전환 |
| 한 스캔에서 같은 셀이 hit과 miss 모두 | hit만 반영 |
| 한 스캔에서 여러 레이가 같은 셀 통과 | miss 1회만 반영 |
| 관측 안 된 셀 | UNKNOWN 유지 (인터페이스 불변) |
| gyro fusion: 바퀴 미끄러짐 시뮬(좌우 엔코더 오차) + 정확한 gyro | heading 오차가 odom-only보다 작음 |
| gyro bias 추정 | 정지 샘플 평균이 bias와 일치 |
| (P2) CSM: 알려진 맵 + 이동된 스캔 | 창 안의 참 오프셋 복원, 창 밖이면 기각 |

| gyro 없음/NaN | encoder-only로 폴백, 예외 없음 |
| LiDAR 각도식 | 대표 index 180/90/0/270의 전/좌/후/우 방향을 확인하고 실제 광선 중심은 S0 대조 |
| mount offset | 스캔 점이 로봇 중심 기준 x −0.03 m 이동해 투영 |

Webots 테스트 (GPS 없이): `RESCUE_MODE` opt-in 모드에서 loop closure test, 명령 이동 대비 LiDAR 벽 거리 변화, 지도 일관성(벽 이중화) 로그. Supervisor 정답 비교는 팀 합의 시 개발용 분리 모드에서만.


## 기존 reference 링크 보존

아래는 이전 연구의 참고 링크를 보존한 것이다. e-puck은 practice/concept only, Erebus는 REFERENCE이며 TECH WEEK 규정이 아니다. 일반 API 예제는 공식 로봇의 장치 배치·competition 입력 허용을 증명하지 않는다.

- [lidar](https://cyberbotics.com/doc/reference/lidar) — platform/algorithm reference.
