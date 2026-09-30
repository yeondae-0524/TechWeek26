# 01. Reference Matrix

> 분류: **강한 reference** = 관리되는 구현 + 시뮬 예제 있음 · **구현 참고 가능** = 코드 있음 · **아이디어 참고** = 논문만.
> 검증: ✅ 이번 조사에서 소스 직접 확인 (2026-09-29~30, 아래 commit) · 📄 논문/문서만 · DOI는 doi.org/Crossref로 제목 일치 확인.
> 효과 표기: 완=완주, 기=기술 구현, 안=주행 안정성, 창=창의성 (◎ 큼 / ○ 보통 / – 미미)

---

## 0. 공식 환경 source (외부 알고리즘 순위와 별도)

확인 2026-09-30. [organizer-confirmed] 조건이 sample보다 우선한다. 사실 근거표는 [09](09_WEBOTS_REFERENCES.md). 외부 숫자는 [REFERENCE], 우리 이식 시작값은 [INITIAL TUNING].

| Source | 고정 버전 / 먼저 볼 파일 | 사용 범위 |
|---|---|---|
| [PNU TECH WEEK repo](https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930/tree/383de18193b2644a6b662e87c4736f1c314a8e73) | main, `383de18193b2644a6b662e87c4736f1c314a8e73`; README, worlds, controllers, Apple PROTO | 공식 교육 환경; 규정과 구분 |
| [Physical AI notebook](https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930/blob/383de18193b2644a6b662e87c4736f1c314a8e73/TECH-WEEK-26_Physical-AI.ipynb) | cell 7/8(설치), 134/136(계획), 147/148(제어), 153–164(FSM/BT), 187(기하); 0-based | OpenCV, A*, look-ahead, FSM 교육 |
| [TurtleBot3Burger.proto](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto) | Webots R2025a; joints, motors, sensors, extensionSlot, boundingObject | 기하·속도·mount·sync |
| [RobotisLds01.proto](https://github.com/cyberbotics/webots/blob/R2025a/projects/devices/robotis/protos/RobotisLds01.proto) | R2025a; Lidar fields | 360 samples, 0.12–3.5 m; semantics는 R2025a lidar 문서와 대조 |

## 1. 최종 Reference Matrix (외부 알고리즘의 해커톤 기여도 순)

| Priority | Reference | 분야 | 가져올 아이디어 | 난이도 | 우리 효과 | 직접 구현 여부 |
|---|---|---|---|---|---|---|
| ★★★★★ | Nav2 (RPP, Collision Monitor, progress/goal checker, BT recovery, inflation) ✅ | Local/Safety/Recovery | RPP-lite, raw-scan 안전 영역, 변위 기반 진전, WAIT/SPIN/BACKUP ladder | MED | 완◎ 안◎ 기○ | Python 재구현 (개념) |
| ★★★★★ | m-explore / m-explore-ros2 ✅ | Exploration | frontier·클러스터·크기필터·blacklist·종료·return_to_init | LOW | 완◎ 기○ | 재구현 + 개선 |
| ★★★★★ | hector_exploration_planner ✅ | Exploration (S&R) | 경로비용 frontier, danger 비용, inner exploration → view frontier | MED | 완○ 기◎ 창◎ | 개념 재구현 |
| ★★★★☆ | GBPlanner (NTNU) ✅ | Exploration/Homing | 시간 예산 homing 식, exp(−λL) | LOW (식만) | 완◎ 안○ | 식만 이식 |
| ★★★★☆ | OctoMap / Hector mapping ✅ | Mapping | log-odds 값·clamp·스캔당 1회 갱신 | LOW~MED | 안◎ 완○ | 재구현 |
| ★★★★☆ | CMU AEDE (pathFollower/localPlanner) ✅ | Local control | 단순 heading 추종, path library(P3) | LOW | 안○ | 개념 |
| ★★★★☆ | Webots docs/samples ✅; E-puck.proto는 practice only | Platform | API·동기화 의미 | LOW | 완◎ 안○ | TB3 사양은 §0 |
| ★★★★☆ | SemExp ✅ (classical 부분) | Recovery/Target | collision map, 방문 셀 통과, goal 팽창 | LOW | 안○ 창○ | 개념 |
| ★★★★☆ | FAR planner ✅ | Planning/Dynamic | known→attemptable 폴백, goal 재평가, 관통 레이 dynamic, momentum | MED | 안○ | 개념 |
| ★★★★☆ | rrt_exploration ✅ (Python) | Exploration | IG=반경 unknown 넓이, hysteresis gain | LOW | 기○ | 재구현 |
| ★★★☆☆ | VLFM ✅ | Frontier 선택 | sticky frontier, acyclic enforcer | LOW | 안○ | 개념 |
| ★★★☆☆ | slam_toolbox (Karto CSM) ✅ / Olson 2009 📄 | Localization | 조건부 CSM 구조, odom prior 페널티 | HIGH | 안○ 기◎ | 조건부 축소판 |
| ★★★☆☆ | Erebus (RCJ Rescue Sim) ✅ | 대회 사례 | 정지 1 s 식별, 20 s LoP → 멈춤 상한 | – | 완○ | 참고만 |
| ★★★☆☆ | move_base ✅ | Recovery | patience, oscillation, 단계적 리셋 | LOW | 안○ | 개념 |
| ★★★☆☆ | webots_ros2 e-puck / TurtleBot3 params ✅ | concept/reference only | 보정 절차, 외부 TB3 progress | – | 안○ | e-puck robot-specific tuning 금지 |
| ★★★☆☆ | PythonRobotics ✅ | 알고리즘 교재 | A*/Theta*/D* Lite/DWA/ICP 예제 (비교·학습용) | – | 기○ | 읽기만 |
| ★★★☆☆ | TARE ✅ | Exploration | hysteresis 임계, return-home 판단 (TSP는 버림) | – | 기○ | 개념 |
| ★★★☆☆ | FUEL ✅ | Exploration | 회전 포함 시간 비용, ray-cast IG | – | 창○ | 개념 |
| ★★★☆☆ | Gervet 2023 📄 | 근거 | 모듈형 > end-to-end | – | 설계 근거 | – |
| ★★☆☆☆ | PONI ✅ | Target search | area+object potential 분리 개념 | – | 창○ | 개념 |
| ★★☆☆☆ | obstacle_detector ✅ | Dynamic | 스캔 클러스터·추적 파라미터 (P3) | HIGH | – | 보류 |
| ★★☆☆☆ | STVL ✅ | Mapping | temporal decay 개념 (2D엔 불필요) | – | – | 보류 |
| ★★☆☆☆ | Holz 2010 / Juliá 2012 / Kulich 2019 / Basilico 2011 📄 | 평가 연구 | TSP 이득 작음, 단순 전략 경쟁력 | – | 판단 근거 | – |
| ★☆☆☆☆ | KISS-ICP ✅ | Localization | – (새 의존성, 3D 중심) | – | – | DO NOT |
| ★☆☆☆☆ | NBVP (ETH) 📄/repo | Exploration 3D | exp 감쇠 이득(GBPlanner로 대체) | – | – | DO NOT |

## 2. TOP 10 (우리 해커톤에 실제 도움이 되는 순서)

1. **Nav2 소스** — RPP·Collision Monitor·progress checker·BT recovery. *Read first*: `nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`, `include/.../regulation_functions.hpp`, `nav2_collision_monitor/src/polygon.cpp`, `nav2_controller/plugins/simple_progress_checker.cpp`, `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`, `nav2_bringup/params/nav2_params.yaml`
2. **m-explore(-ros2)** — *Read first*: `explore/src/frontier_search.cpp`, `explore/src/explore.cpp`
3. **hector_exploration_planner** — *Read first*: `hector_exploration_planner/src/hector_exploration_planner.cpp` (`buildexploration_trans_array_`, `cellDanger`, `doInnerExploration`, `findInnerFrontier`)
4. **공식 PNU repo + Webots R2025a TB3/LDS PROTO** — *Read first*: [robot.md](https://cyberbotics.com/doc/reference/robot)(synchronization), [lidar.md](https://cyberbotics.com/doc/reference/lidar), [camera.md](https://cyberbotics.com/doc/reference/camera), §0 TB3/LDS 링크 (E-puck.proto는 practice 기록만)
5. **GBPlanner** — *Read first*: `gbplanner/src/rrg.cpp` `homingRequired`, path gain 루프
6. **OctoMap + Hector mapping** — *Read first*: `octomap/src/AbstractOccupancyOcTree.cpp` L42-47, `hector_mapping/include/hector_slam_lib/map/GridMapLogOdds.h`, `OccGridMapBase.h`
7. **CMU AEDE** — *Read first*: `src/local_planner/src/pathFollower.cpp`, `src/local_planner/launch/local_planner.launch`
8. **SemExp (classical 부분)** — *Read first*: `agents/sem_exp.py` `_plan`, `_get_stg`
9. **FAR planner** — *Read first*: `src/far_planner/src/graph_planner.cpp` (`ReEvaluateGoalPosition`, 모드 전환), `far_planner.cpp` `ExtractDynamicObsFromScan`, `config/default.yaml`
10. **rrt_exploration + VLFM** (frontier 점수·hysteresis) — *Read first*: `scripts/assigner.py`, `scripts/functions.py`, `vlfm/policy/itm_policy.py` `_get_best_frontier`

## 3. Reference 카드

형식: 기본 정보 · 관련성 · 가져올 것 · 필요 없는 것 · 난이도 · 해커톤 효과 · 먼저 볼 파일 · 분류

### 3.1 Exploration

**m-explore (explore_lite)** — Jiří Hörner (Charles Univ.), 2015–2016; C++; ROS1; BSD; [repo](https://github.com/hrnr/m-explore) @712bdd4 (2021-01-07) ✅
- 관련성 ★★★★★ · 가져올 것: frontier 정의·8-연결 클러스터·`min_frontier_size`·blacklist·종료 · 불필요: costmap client, move_base action, marker · 난이도 LOW · 효과 완◎ 기○ · 파일: `explore/src/frontier_search.cpp`, `explore/src/explore.cpp` · **구현 참고 가능**
- 주의: Euclidean 거리, gain=경계 셀 수, `orientation_scale` 미사용, centroid 편향 (02 §1.1)

**m-explore-ros2** — robo-friends, C++; ROS2(Nav2); BSD; [repo](https://github.com/robo-friends/m-explore-ros2) @326cf8a (2026-06-01) ✅
- ★★★★★ · 가져올 것: `return_to_init`, pause/resume의 `resuming_` grace, ABORTED 오류코드 구분 · 불필요: status msg, topics · LOW · 완○ · `explore/src/explore.cpp` · **강한 reference**(활발히 관리)

**frontier_exploration** — Paul Bovbel; C++; ROS1; package.xml은 "Proprietary"로 표기, LICENSE는 BSD형 문구 ⚠️; [repo](https://github.com/paulbovbel/frontier_exploration) @fdb6244 ✅
- ★★☆☆☆ · travel_point(closest/middle/centroid) 선택지, ABORTED box blacklist · 파일: `frontier_exploration/src/frontier_search.cpp` · 구현 참고 가능

**hector_exploration_planner** — TU Darmstadt (Team Hector, RoboCup Rescue); C++; ROS1; BSD; [repo](https://github.com/tu-darmstadt-ros-pkg/hector_navigation) @bdc2803 (2020) ✅; 논문 Wirth & Pellenz 2007 [doi:10.1109/SSRR.2007.4381274](https://doi.org/10.1109/SSRR.2007.4381274), Kohlbrecher 2014 [doi:10.1007/978-3-662-44468-9_58](https://doi.org/10.1007/978-3-662-44468-9_58)
- ★★★★★ · 가져올 것: Exploration Transform(다중 frontier wavefront + danger), inner exploration · 불필요: SBPL 연동, 벽 따라가기(exploreWalls) · MED · 기◎ 창◎ · 파일 위 · **구현 참고 가능**
- 주의: goal angle penalty는 코드에서 `if(false)`

**rrt_exploration** — Hassan Umari; Python/C++; ROS1; MIT; [repo](https://github.com/hasauino/rrt_exploration) @3ec97a7 ✅; Umari 2017 [doi:10.1109/IROS.2017.8202319](https://doi.org/10.1109/IROS.2017.8202319)
- ★★★★☆ · 가져올 것: `informationGain`(반경 unknown 넓이), hysteresis_gain 2.0 / radius 3.0 · 불필요: RRT 탐지, 다중 로봇 할당 · LOW · 기○ · `scripts/functions.py`, `scripts/assigner.py` · 구현 참고 가능

**GBPlanner (gbplanner_ros)** — NTNU ARL (CERBERUS, DARPA SubT 우승); C++; ROS1; BSD-3; [repo](https://github.com/ntnu-arl/gbplanner_ros) @f9904bb (gbplanner3, 2026-09) ✅; Dang 2020 [doi:10.1002/rob.21993](https://doi.org/10.1002/rob.21993), Tranzatto 2022 [doi:10.1126/scirobotics.abp9742](https://doi.org/10.1126/scirobotics.abp9742)
- ★★★★☆ · 가져올 것: `homingRequired` 시간 예산 식, `Σgain·exp(−λd)`, 방향 일관성 페널티 · 불필요: 3D 그래프, voxblox · LOW(식) · 완◎ · `gbplanner/src/rrg.cpp` · 아이디어/식만

**TARE** — CMU (Cao, Zhang); C++; ROS1(+humble 브랜치); BSD; [repo](https://github.com/caochao39/tare_planner) @4450059 ✅; RSS 2021, Science Robotics 2023 [doi:10.1126/scirobotics.adf0970](https://doi.org/10.1126/scirobotics.adf0970)
- ★★★☆☆ · 가져올 것: 셀 상태 hysteresis 임계, 귀환 판정 · 불필요: OR-Tools TSP, 3D viewpoint · VERY HIGH(전체) · 기○ · `sensor_coverage_planner_ground.cpp` `execute`, `grid_world.cpp` `UpdateCellStatus` · **강한 reference**(시뮬 환경 포함)지만 우리엔 개념만

**FUEL** — HKUST Aerial Robotics; C++; ROS1; GPL-3.0; [repo](https://github.com/HKUST-Aerial-Robotics/FUEL) @662dd23 ✅; RA-L 2021 [doi:10.1109/LRA.2021.3051563](https://doi.org/10.1109/LRA.2021.3051563)
- ★★★☆☆ · 가져올 것: `computeCost` 시간 비용, `findViewpoints` ray-cast IG · 불필요: ATSP, 궤적 최적화, UAV · 창○ · 아이디어 참고 (GPL이므로 코드 복사 금지)

**NBVP** — ETH ASL; C++; ROS1; [repo](https://github.com/ethz-asl/nbvplanner) (2017) · Bircher 2016 [doi:10.1109/ICRA.2016.7487281](https://doi.org/10.1109/ICRA.2016.7487281) · ★☆☆☆☆ DO NOT (GBPlanner로 대체)

### 3.2 Navigation / Local / Recovery

**Nav2** — Open Navigation/Samsung 등, Macenski et al.; C++; ROS2; Apache-2.0 · LGPL-2.1-or-later 혼합 (파일별 SPDX); [repo](https://github.com/ros-navigation/navigation2) @84a129d (2026-09-28) ✅; Macenski 2023 RPP [doi:10.1007/s10514-023-10097-6](https://doi.org/10.1007/s10514-023-10097-6), 서베이 [doi:10.1016/j.robot.2023.104493](https://doi.org/10.1016/j.robot.2023.104493)
- ★★★★★ · 가져올 것: 05/06 전부 · 불필요: lifecycle, BT.CPP, costmap 레이어 시스템, MPPI · MED · 완◎ 안◎ · 파일: TOP10 #1 · **강한 reference**

**ROS1 navigation (move_base)** — BSD; [repo](https://github.com/ros-planning/navigation) @f44bb1f ✅ · ★★★☆☆ · `move_base/src/move_base.cpp` (patience, oscillation, recovery 순서) · 구현 참고 가능

**CMU Autonomous Exploration Development Environment** — CMU; C++; ROS1(noetic)/ROS2 브랜치; BSD; [repo](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) @bf0cba7, [site](https://www.cmu-exploration.com/) ✅
- ★★★★☆ · 가져올 것: pathFollower(`lookAheadDis 0.5`, `yawRateGain 7.5`, `dirDiffThre 0.1`, `stopDisThre 0.2`, `slowDwnDisThre 0.85`), localPlanner path library 개념 · 불필요: terrain analysis, Unity/Matterport 환경 · LOW · 안○ · **강한 reference**

**FAR planner** — CMU (Fan Yang); C++; ROS1; BSD; [repo](https://github.com/MichaelFYang/far_planner) @2799b69 ✅; IROS 2022 [doi:10.1109/IROS47612.2022.9981574](https://doi.org/10.1109/IROS47612.2022.9981574)
- ★★★★☆ · 가져올 것: 03/04 참고 · 불필요: visibility graph 본체 · VERY HIGH(전체) · 안○ · 구현 참고 가능(개념)

**PythonRobotics** — Atsushi Sakai 외; Python; standalone; MIT; [repo](https://github.com/AtsushiSakai/PythonRobotics) @7298ec2 ✅ · ★★★☆☆ · A*, Theta*, D* Lite, DWA, pure pursuit, ICP 교재용 · 우리 코드엔 복사하지 않고 이해용

**obstacle_detector** — Mateusz Przybyła; C++; ROS1; BSD; [repo](https://github.com/tysik/obstacle_detector) @bd59d40 (2018) ✅ · ★★☆☆☆ · P3

**TurtleBot3** — ROBOTIS; ROS2; Apache-2.0; [repo](https://github.com/ROBOTIS-GIT/turtlebot3) @fc817ce ✅ · ★★★☆☆ · `turtlebot3_navigation2/param/burger.yaml` (controller 10 Hz, progress 0.1 m/10 s, inflation 0.5/scaling 5.0, DWB critics) · 값 참고

**webots_ros2** — Cyberbotics; Python/C++; ROS2; Apache-2.0; [repo](https://github.com/cyberbotics/webots_ros2) @85368b7 ✅ · ★★★☆☆ · e-puck `nav2_params.yaml`, `drive_calibrator.py` · practice/reference only; 보정 절차 개념만, 기하·속도 이식 금지

### 3.3 Mapping / Localization

**Hector SLAM** — TU Darmstadt; C++; ROS1; BSD; [repo](https://github.com/tu-darmstadt-ros-pkg/hector_slam) @a5e77fd ✅; Kohlbrecher 2011 [doi:10.1109/SSRR.2011.6106777](https://doi.org/10.1109/SSRR.2011.6106777) · ★★★★☆ · log-odds 규칙, map update 임계 · 구현 참고 가능

**OctoMap** — Hornung et al.; C++; BSD; [repo](https://github.com/OctoMap/octomap) ✅ (파일 1개 확인); [doi:10.1007/s10514-012-9321-0](https://doi.org/10.1007/s10514-012-9321-0) · ★★★★☆ · 기본 확률·clamp 값

**slam_toolbox** — Steve Macenski; C++; ROS2; LGPL-2.1; [repo](https://github.com/SteveMacenski/slam_toolbox) @33841d0 ✅; JOSS 2021 [doi:10.21105/joss.02783](https://doi.org/10.21105/joss.02783) · ★★★☆☆ · Karto CSM(`lib/karto_sdk/src/Mapper.cpp`) · 조건부 참고 (LGPL 코드 복사 금지, 알고리즘만)

**STVL** — Macenski; C++; ROS2; LGPL-2.1; [repo](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) @16cdf3b ✅ · ★★☆☆☆ · 개념만

**KISS-ICP** — PRBonn; C++/Python; MIT; [repo](https://github.com/PRBonn/kiss-icp) @1ffa7d7 ✅; [doi:10.1109/LRA.2023.3236571](https://doi.org/10.1109/LRA.2023.3236571) · ★☆☆☆☆ · DO NOT (새 의존성)

**논문**: Olson 2009 CSM [doi:10.1109/ROBOT.2009.5152375](https://doi.org/10.1109/ROBOT.2009.5152375) · Censi 2008 PL-ICP [doi:10.1109/ROBOT.2008.4543181](https://doi.org/10.1109/ROBOT.2008.4543181) · Hess 2016 Cartographer [doi:10.1109/ICRA.2016.7487258](https://doi.org/10.1109/ICRA.2016.7487258) · Borenstein & Feng Gyrodometry [doi:10.1109/ROBOT.1996.503813](https://doi.org/10.1109/ROBOT.1996.503813), UMBmark [doi:10.1109/70.544770](https://doi.org/10.1109/70.544770) · Moravec & Elfes [doi:10.1109/ROBOT.1985.1087316](https://doi.org/10.1109/ROBOT.1985.1087316) · Yguel 2007 [doi:10.1007/978-3-540-75404-6_3](https://doi.org/10.1007/978-3-540-75404-6_3) · Placed 2023 Active SLAM survey [doi:10.1109/TRO.2023.3248510](https://doi.org/10.1109/TRO.2023.3248510) (**Active SLAM 전반은 해커톤에 과함** — 불확실성 기반 goal 선택은 DO NOT, "나쁜 pose로 맵을 오염시키지 않기"만 채택)

### 3.4 Target search / ObjectNav

**SemExp** — Chaplot et al. (CMU/FAIR); Python/PyTorch; Habitat; MIT; [repo](https://github.com/devendrachaplot/Object-Goal-Navigation) @5d76902 ✅; [arXiv:2007.00643](https://arxiv.org/abs/2007.00643) · ★★★★☆(classical 부분) · collision map, `_get_stg` · 학습·Habitat 불필요

**PONI** — Ramakrishnan et al.; Python/PyTorch; MIT; [repo](https://github.com/srama2512/PONI) @30682c2 ✅; [arXiv:2201.10029](https://arxiv.org/abs/2201.10029) · ★★☆☆☆ · 개념만

**VLFM** — Boston Dynamics AI Institute; Python; MIT; [repo](https://github.com/bdaiinstitute/vlfm) @584ed56 ✅; [arXiv:2312.03275](https://arxiv.org/abs/2312.03275) · ★★★☆☆ · sticky/acyclic만 · VLM 불필요 (fallback 코드 버그 주의)

**Gervet 2023** 📄 [doi:10.1126/scirobotics.adf6991](https://doi.org/10.1126/scirobotics.adf6991) · 설계 근거

### 3.5 S&R 시스템 / 대회

**RoboCup Rescue** 📄 Sheh, Schwertfeger, Visser, "16 Years of RoboCup Rescue," KI 2016 [doi:10.1007/s13218-016-0444-x](https://doi.org/10.1007/s13218-016-0444-x) — 20분 미션·victim 탐색 경기 구조, Hector 오픈소스의 배경.

**Erebus (RCJ Rescue Simulation)** ✅ Webots + Python; [repo](https://github.com/robocup-junior/erebus) @8e56efe — 가장 우리와 비슷한 **Webots 대회 플랫폼** (심판 로직 참고).

**DARPA SubT** 📄 CMU Explorer: Scherer 2022 [doi:10.55417/fr.2022023](https://doi.org/10.55417/fr.2022023); CERBERUS: Tranzatto 2022 [doi:10.1126/scirobotics.abp9742](https://doi.org/10.1126/scirobotics.abp9742); SLAM 교훈: Ebadi 2024 [doi:10.1109/TRO.2023.3323938](https://doi.org/10.1109/TRO.2023.3323938) — 아키텍처 교훈(모듈화, 복구 우선, 귀환 예산)만.

### 3.6 Local planning 고전 논문
DWA [doi:10.1109/100.580977](https://doi.org/10.1109/100.580977) · VFH [doi:10.1109/70.88137](https://doi.org/10.1109/70.88137) · VFH+ [doi:10.1109/ROBOT.1998.677362](https://doi.org/10.1109/ROBOT.1998.677362) · APF [doi:10.1177/027836498600500106](https://doi.org/10.1177/027836498600500106) · APF 한계 [doi:10.1109/ROBOT.1991.131810](https://doi.org/10.1109/ROBOT.1991.131810) · VO [doi:10.1177/027836499801700706](https://doi.org/10.1177/027836499801700706) · ORCA [doi:10.1007/978-3-642-19457-3_1](https://doi.org/10.1007/978-3-642-19457-3_1) · TEB [doi:10.1016/j.robot.2016.11.007](https://doi.org/10.1016/j.robot.2016.11.007) · A* [doi:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136) · Theta* [doi:10.1613/jair.2994](https://doi.org/10.1613/jair.2994) · LPA* [doi:10.1016/j.artint.2003.12.001](https://doi.org/10.1016/j.artint.2003.12.001) · Visual servo [doi:10.1109/MRA.2006.250573](https://doi.org/10.1109/MRA.2006.250573) · 사람 검출 [doi:10.1109/ROBOT.2007.363998](https://doi.org/10.1109/ROBOT.2007.363998) · DATMO [doi:10.1177/0278364907081229](https://doi.org/10.1177/0278364907081229) · JPDAF 추적 [doi:10.1177/0278364903022002002](https://doi.org/10.1177/0278364903022002002)

## 4. 라이선스 주의
- **GPL-3.0 (FUEL), LGPL (slam_toolbox, STVL)**: 코드 복사 금지, 알고리즘 아이디어만.
- BSD/MIT/Apache: 그래도 AGENTS 규칙상 **복사하지 않고 재구현**.
