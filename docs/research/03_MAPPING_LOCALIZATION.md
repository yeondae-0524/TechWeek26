# 03. Mapping (dynamic obstacle 대응) & Localization

> ✅ 소스 확인 · 📄 논문/문서만 · ⚠️ 미확인/추정. 수치 중 "initial tuning suggestion"은 레퍼런스 값을 우리 로봇 규모로 옮긴 **시작값**이지 검증값이 아니다.

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
| **log-odds + clamp (OctoMap/Hector식)** | LOW~MED | ~1–2 s | **높음** | 낮음(스캔당 1회 갱신 + hit 우선) | ✗ (보이는 레이 필요) | **추천 (P1)** |
| + dynamic layer (FAR식 분리 + TTL) | MED | 즉시 분리, TTL 소멸 | 높음 | 낮음 | TTL로 해제 | P2 |
| temporal decay 전체 적용 (STVL식) | MED | 시간 경과 | 중 | **정적 벽도 사라짐** (재관측 필요) | ✓ | 2D 360° LiDAR엔 불필요 |

### A.4 추천 설계 (mapping.py 내부 변경, 외부 인터페이스 `grid[row][col] ∈ {-1,0,1}` 유지)

```text
내부 상태: logodds[row][col] (float, 0.0 = prior), observed[row][col] (bool)
insert_scan(pose, ranges, ...):
    hits, misses = set(), set()
    for each ray:
        cells = bresenham(robot_cell, end_cell)
        misses.update(cells[:-1] if hit else cells)       # 끝점 직전까지
        if hit: hits.add(end_cell)
    misses -= hits                                        # Hector: 같은 스캔의 hit 우선
    for c in misses: L[c] = max(L_MIN, L[c] + L_MISS); observed[c]=True
    for c in hits:   L[c] = min(L_MAX, L[c] + L_HIT);  observed[c]=True
    export: grid[c] = UNKNOWN if not observed[c] else (OCCUPIED if L[c] > L_OCC else FREE if L[c] < L_FREE else 이전값 유지)
```

시작값 (OctoMap 기본값 이식, initial tuning suggestion):

| 파라미터 | 값 | 의미 (우리 5 Hz 매핑 기준) |
|---|---|---|
| `L_HIT` | +0.85 | unknown → 1 hit이면 점유 |
| `L_MISS` | −0.40 | unknown → 1 miss면 free |
| `L_MAX` | +3.5 | 최대 확신 장애물도 **9 miss ≈ 1.8 s** 관측이면 해제 |
| `L_MIN` | −2.0 | 오래 free였던 셀도 **3 hit ≈ 0.6 s**면 점유 (안전 모니터는 raw scan이라 그 사이도 안전) |
| `L_OCC`, `L_FREE` | +0.4 / −0.2 | 약한 hysteresis로 깜빡임 억제 |

주의점:
- **grazing ray 벽 침식**: 끝점 셀과 그 8-이웃은 miss 갱신에서 제외하는 옵션을 두고 unit test로 비교.
- **가려진 ghost**(사람이 서 있던 자리 뒤쪽에서만 보이는 경우)는 log-odds로 안 지워진다 → RECOVERY에서 "로봇 주변 반경 r 셀을 prior(0)로 리셋"하는 **costmap clearing 대응 동작**을 둔다 (Nav2 BT의 `ClearEntireCostmap`, move_base의 `conservative_reset`/`aggressive_reset`과 같은 역할; 06 참고).
- 성능: 현재 `insert_scan`은 360 rays에서 **4.7~10 ms** (실측). set 기반 dedupe를 넣어도 같은 자릿수 예상 → **5 Hz(12 step마다)** 권장 (현재 5 step마다 = 12.5 Hz는 e-puck 속도 대비 과잉).

### A.5 동적 장애물의 지도 반영 원칙 (자세한 행동 정책은 05)

1. **안전 판단은 지도가 아니라 raw scan** (Nav2 Collision Monitor 원칙: costmap/planner를 우회해 센서 데이터로 직접 판단).
2. 지도(log-odds)는 사람을 **일시적으로** 반영 → 전역 경로는 사람을 피해 계획되지만 사람이 떠나면 ~2 s 후 복원.
3. 복귀 경로 계획 전에는 "오래된(dynamic 의심) 점유 셀"을 한 번 더 의심: 경로가 없으면 **ghost 의심 셀만 prior로 되돌리고 재계획** (FAR의 dynamic 분리 개념의 단순판).

---

## B. Localization

### B.1 우리 센서 사실관계 (practice robot = e-puck R2025a, ✅ E-puck.proto)

| 센서 | 값 | 함의 |
|---|---|---|
| wheel PositionSensor | resolution `0.00628` rad (2π/1000) → 바퀴 반경 0.02 m에서 0.126 mm/tick | 양자화 오차는 무시 가능. 오차원은 **미끄러짐/바퀴 반경·축간 거리 오차** |
| Gyro | lookupTable 노이즈 3번째 열 `0.003` (상대 0.3%) | yaw rate가 매우 정확 → **heading 보정 1순위** |
| Accelerometer | 노이즈 0.003 | 충돌/미끄러짐 **감지용**, 적분 금지 |
| Compass | e-puck 없음 (`config.DEVICE_NAMES["compass"] = None`) | 공식 로봇에 있으면 절대 heading으로 최우선 활용 (규정 확인) |
| GPS | turretSlot GPS — **DEBUG ONLY** | 규정상 위치추정 금지. 대신 **drift 측정 도구**로 사용 |
| LiDAR (baseline world) | 360 rays, maxRange 2 m, noise 0 (practice) | 공식 월드는 noise 가능. Webots noise σ = `noise × maxRange` ([lidar.md](https://cyberbotics.com/doc/reference/lidar)) |

⚠️ 주의: baseline world의 Lidar는 `translation 0.0095 0 0.005` 인데 `config.LIDAR_MOUNT_OFFSET = (0.0, 0.0)` → 9.5 mm 불일치(작지만 CSM을 넣으면 체계 오차가 됨). 공식 로봇에서 반드시 재확인.

### B.2 선택지 비교

| 방법 | 원리 | 우리 구현 난이도 | CPU(Python) | 강점 | 약점 | 출처 | 판정 |
|---|---|---|---|---|---|---|---|
| Wheel odometry (현재) | 적분 | 완료 | 무시 | 단순 | 회전 시 미끄러짐으로 heading drift | – | 유지 |
| UMBmark 보정 | 사각 경로 왕복으로 r/L 보정 | LOW | – | 체계 오차 제거 | 대회 당일 시간 필요 | Borenstein & Feng 1996 [doi:10.1109/70.544770](https://doi.org/10.1109/70.544770) 📄, webots_ros2 [`drive_calibrator.py`](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/drive_calibrator.py) ✅ | **P1 (당일 절차)** |
| **Gyrodometry** | Δθ는 gyro, 거리는 encoder (불일치 클 때 gyro 채택) | **LOW** | 무시 | heading drift 대폭 감소 | gyro bias → 정지 중 평균으로 추정 | Borenstein & Feng, "Gyrodometry," ICRA 1996 [doi:10.1109/ROBOT.1996.503813](https://doi.org/10.1109/ROBOT.1996.503813) 📄 | **P0** |
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

1. **P0**: encoder 거리 + **gyro heading** (정지 1 s 동안 gyro bias 평균 추정, INITIALIZE에서) + 각도 정규화.
2. **P1**: 당일 오도메트리 보정 절차(직진 1 m / 제자리 360°를 GPS debug로 비교) + slip 감지 플래그.
3. **P2 (조건부)**: practice world에서 **GPS debug로 drift를 측정**해서, 8분 미션 후 오차가 `≥ 5 cm`(= 1셀) 수준이면 CSM 추가. 아니면 넣지 않는다 (복잡도 대비 효과가 측정으로 증명될 때만).
4. **DO NOT**: loop closure, pose graph, particle filter, ICP scan-to-scan 단독.

### B.5 Localization 실패 감지 (P2)

- CSM을 넣었다면 매칭 점수 이동평균이 급락 → "localization degraded" → 속도 제한 + 제자리 회전으로 재관측.
- CSM이 없다면: gyro-odom 불일치 누적, 가속도 spike(충돌) 발생 시 플래그 → 해당 구간 맵 갱신 일시 중지(Hector가 `map_update_distance_thresh 0.4 m`/`angle 0.9 rad`로 갱신 빈도를 제한하는 것과 같은 취지: 나쁜 포즈로 맵을 오염시키지 않기).

## C. 테스트 계획 (Webots 없이)

| 테스트 | 기대 |
|---|---|
| 같은 셀에 hit 1회 | OCCUPIED |
| OCCUPIED(L_MAX) 셀을 레이가 9회 관통 | FREE로 전환 |
| 한 스캔에서 같은 셀이 hit과 miss 모두 | hit만 반영 |
| 한 스캔에서 여러 레이가 같은 셀 통과 | miss 1회만 반영 |
| 관측 안 된 셀 | UNKNOWN 유지 (인터페이스 불변) |
| gyro fusion: 바퀴 미끄러짐 시뮬(좌우 엔코더 오차) + 정확한 gyro | heading 오차가 odom-only보다 작음 |
| gyro bias 추정 | 정지 샘플 평균이 bias와 일치 |
| (P2) CSM: 알려진 맵 + 이동된 스캔 | 창 안의 참 오프셋 복원, 창 밖이면 기각 |

Webots 테스트: `RESCUE_MODE` opt-in 모드에서 정해진 경로 주행 후 `read_gps_debug()`와 추정 pose의 오차를 로그 (측정 전용).
