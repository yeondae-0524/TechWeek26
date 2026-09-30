# 09. Webots & Official TECH WEEK Environment References

> **2026-09-30 공식 repo 기준으로 재검증함.** 이 문서가 연구 전체에서 **"공식 환경 사실(official-environment facts)"의 기준표**다.
> 알고리즘 근거(Nav2 등 외부 reference)는 02~08에 있고, 여기에는 **우리 로봇·센서·월드에 대한 사실**만 둔다.
>
> 숫자 태그: **[ORGANIZER]** 운영진 직접 확인 · **[OFFICIAL]** 공식 repo/notebook/Webots R2025a 공식 PROTO·문서 · **[DERIVED]** 공식 값으로 계산 · **[MEASURED]** 우리 PC 측정 · **[REFERENCE]** 외부 프로젝트 · **[INITIAL TUNING]** 검증 전 시작값 · **[DAY-OF]** 당일 확인

---

## 0. 확인한 공식 source

| 항목 | 값 |
|---|---|
| 공식 repo | https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930 |
| branch / commit | `main` / `383de18193b2644a6b662e87c4736f1c314a8e73` ("Initial commit", 2026-09-29 03:43 -0400) |
| 확인 일시 | 2026-09-30 재개 검증: 기존 `D:/Dev/projects/PNU-TECHWEEK-260930` read-only. `git ls-remote origin refs/heads/main` 결과가 local HEAD와 동일 |
| 읽은 파일 | `README.md`(+`부산대_TECHWEEK_Physical_AI.png` 계획안 이미지), `TECH-WEEK-26_Physical-AI.ipynb`(191 cells), `controllers/tb3_*/*.py` 8개, `worlds/*.wbt` 7개, `protos/*Apple.proto` 4개 |
| Webots 공식 PROTO | R2025a `projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto`, `projects/devices/robotis/protos/RobotisLds01.proto` (월드의 EXTERNPROTO URL과 동일 태그), `docs/reference/worldinfo.md` |
| 미확인 | `부산대 TECH WEEK Physical AI.pdf` (PDF 렌더러 없음 — 계획안 PNG로 대체 확인) |
| 저작권 | notebook 말미: 무단 복제·수정·재사용 금지 → **우리 문서는 사실·수치만 인용하고 코드/본문은 복사하지 않는다.** |

### 증거 우선순위 (충돌 시)
1. 운영진 직접 확인 [ORGANIZER] → 2. 공식 repo/notebook/world/controller [OFFICIAL] → 3. 행사 공식 계획안 → 4. Webots R2025a 공식 문서/PROTO → 5. 외부 reference/논문 → 6. 우리 initial tuning.
**공식 예제 코드에 있다 ≠ 대회에서 허용된다.** (예: `tb3_teleop_sensors.py`가 Compass를 읽지만 운영진은 Compass 미사용 확인)

### 재개 pass source integrity / locator

공식 local notebook에는 기존 세미콜론 삭제 1건이 있었다. 수정·되돌리지 않았으며 **`git show HEAD:TECH-WEEK-26_Physical-AI.ipynb`**와 대조했다. 이 변경은 아래 facts에 영향 없다. 원격 main SHA 확인과 working-tree 사실 검증을 구분한다.

- [고정 공식 tree](https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930/tree/383de18193b2644a6b662e87c4736f1c314a8e73): README/계획안 PNG, controllers 8개, worlds 7개, colored Apple PROTO 4개.
- [notebook](https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930/blob/383de18193b2644a6b662e87c4736f1c314a8e73/TECH-WEEK-26_Physical-AI.ipynb): 0-based cell 7/8 dependency, 134/136 Dijkstra/A*, 147/148 look-ahead, 153–164 FSM/BT, 187 기하.
- [TurtleBot3Burger R2025a](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto): lines 20/35 sync/slot, 38–160 joints, 275–303 IMU/boundingObject.
- [RobotisLds01 R2025a](https://github.com/cyberbotics/webots/blob/R2025a/projects/devices/robotis/protos/RobotisLds01.proto): lines 11–15 mount/name/noise, 189–195 lidar specs.
- R2025a [lidar](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/lidar.md), [robot](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/robot.md), [worldinfo](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/worldinfo.md), [camera](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/camera.md), [gyro](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/gyro.md) 원문을 대조했다. 아래 일반 웹 문서 링크의 최신판과 구분한다.

## 1. 운영진 확인 조건 [ORGANIZER = organizer-confirmed]

| 항목 | 내용 |
|---|---|
| 환경 | Simulation only, differential-drive mobile robot |
| 필수 센서 | **Wheel encoder 사용 필수**, **2D LiDAR 사용 필수** |
| 선택 센서 | **IMU 선택** (사용 금지 아님, 팀 선택) |
| 사용 안 함 | **Compass 사용 안 함**, **GPS 사용 안 함** |
| Mission | prior map 없음, target 위치 모름, target 시각 정보 당일 제공, target들을 찾아 이동, 시작점 복귀, 정적/동적 장애물 충돌 방지, 안전거리 고려 |
| 평가 | Mission achievement / Technical implementation / Driving stability / Creativity |
| Vision | 지나치게 어려운 인식 문제 의도 아님, rule-based CV 또는 필요 시 DL |
| GPU | 필수 아님, 필요하면 팀당 1대 수준, RTX server 사용 가능 |

## 2. 공식 계획안 (repo README 이미지) [OFFICIAL]
- 파이프라인: Perception → Localization → Mapping → Planning → Control.
- 해커톤 조건: 사전 지도 없음 · 로봇 현재 위치 모름 · **시작 지점(position & orientation) 제공** · 목표 물체 위치 모름 · 목표 물체 종류·시각 특징 제공 · 정적 장애물·이동하는 사람 충돌 금지.
- 활용 가능 기술(예시): NumPy Occupancy Grid Map, **Scan Matching**, 규칙 기반 CV / 학습 DL, Dijkstra·A*, **DWA 등 critic 기반 local planner**.
- 개발 환경: Webots, **Ubuntu 22.04 & Python 3.10 기준**, Windows·macOS 가능, C++ 가능, GPU 선택.
- "활용 가능 기술"은 **예시 목록**이지 필수 구현 요건이 아니다 (DWA를 쓰지 않는 결정의 근거는 05).

## 3. Robot: TurtleBot3Burger [OFFICIAL]

| 항목 | 값 | 출처 |
|---|---|---|
| 모델 | `TurtleBot3Burger` (Webots R2025a) | 모든 로봇 월드의 EXTERNPROTO |
| 바퀴 반경 | **0.033 m** | notebook `WHEEL_RADIUS = 0.033`; PROTO wheel `Cylinder radius 0.033`, joint anchor z 0.033 |
| 바퀴 간격 | **0.160 m** | notebook `WHEEL_SEPARATION = 0.160`; PROTO anchors y = ±0.08 |
| 로봇 반경 | **0.105 m** | notebook `ROBOT_RADIUS = 0.105` |
| 외접 반경 (회전 중심 기준) | ≈ **0.110 m** [DERIVED] | 두 body box의 최대 모서리 거리 약 0.1103 m: (−0.0995, ±0.0475), 다른 box 약 0.1057 m; wheel 포함 확인. 0.105는 notebook 단순 모델이며 안전 원은 0.111 m로 올림 + margin 권장 |
| footprint 비대칭 | 앞쪽 끝 x ≈ +0.036 m, 뒤쪽 끝 x ≈ −0.100 m [DERIVED] | 몸체 중심이 바퀴축보다 3.2 cm 뒤 |
| 모터 | `"left wheel motor"`, `"right wheel motor"`, maxVelocity **6.67 rad/s** | PROTO |
| 최대 선속도 | 6.67 × 0.033 = **0.220 m/s** [DERIVED] | |
| 최대 제자리 각속도 | 2 × 0.220 / 0.160 = **2.75 rad/s** [DERIVED] | |
| 엔코더 | `"left wheel sensor"`, `"right wheel sensor"` (PositionSensor, rad), resolution **0.00628 rad** | PROTO; 공식 controller는 `left_motor.getPositionSensor()`로 획득 |
| 바퀴 회전 부호 | 두 바퀴 모두 joint axis `0 1 0`; **양의 속도 = 전진** [DERIVED] | 축 +y, 접지점 속도 −x ⇒ 로봇 +x. teleop `W` = 양쪽 +SPEED |
| 좌/우 배치 | left wheel y = +0.08, right wheel y = −0.08 | PROTO anchor |
| IMU 부품 | PROTO 기본 포함: `Accelerometer`, `Gyro`, `Compass` (모두 x=−0.032, z=0.078), **lookupTable 없음 → raw 값 반환**(Gyro: rad/s) | PROTO; [gyro.md](https://cyberbotics.com/doc/reference/gyro) "By default the lookup table is empty" |
| 기본 extensionSlot | `RobotisLds01 { }` | PROTO |
| Robot `synchronization` | PROTO 필드 기본 **TRUE**, 공식 월드 모두 미지정(=TRUE) | PROTO, worlds |
| 공식 teleop 속도 | 바퀴 `SPEED = 3.0 rad/s` (≈ 0.099 m/s) | `tb3_teleop*.py` |

좌표 convention:
- Webots 로봇 local frame: **+x 전방, +y 좌측, +z 위** (FLU) [DERIVED: 바퀴 배치·teleop 동작].
- 월드 frame, `theta = 0 → +x`, 반시계 + : **우리 내부 convention** (INTERFACES.md). 공식 자료는 이를 규정하지 않음. 시작 pose는 운영진 제공 [OFFICIAL 계획안] — 제공 형식(월드 좌표인지, 원점 기준인지)은 [DAY-OF].
- 차동구동 식 (notebook과 동일한 표준식): `v = (v_r+v_l)/2`, `ω = (v_r−v_l)/L`, `v_r = v + ωL/2`, `v_l = v − ωL/2`, 바퀴 각속도 = 선속도 / r. 우리 `control.set_velocity`, `localization.integrate_diff_drive`와 같은 식.

## 4. 2D LiDAR: RobotisLds01 [OFFICIAL]

| 항목 | 값 | 출처 |
|---|---|---|
| device 이름 | **`"LDS-01"`** | PROTO `name`, 공식 controller `robot.getDevice("LDS-01")` |
| 형식 | Webots `Lidar`, `numberOfLayers 1` → **2D** | PROTO |
| 샘플 수 | `horizontalResolution` **360** (1°) | PROTO |
| FOV | `fieldOfView 6.28318` (360°) | PROTO |
| near / minRange / maxRange | **0.07 / 0.12 / 3.5 m** | PROTO |
| noise | `0.0043` → Webots 정의상 σ = noise × maxRange ≈ **0.015 m** [DERIVED] | PROTO, [lidar.md](https://cyberbotics.com/doc/reference/lidar) |
| type | 미지정 → 기본 `"fixed"` (PROTO 내부 RotationalMotor는 외형 회전용) | PROTO, lidar.md 기본값 |
| 범위 밖 값 | minRange 미만·maxRange 초과 → **inf** | lidar.md |
| 인덱스 순서 | range image는 **왼쪽→오른쪽** (lidar.md). 공식 예제 라벨: `ranges[180]=Front, [0]=Back, [90]=Left, [270]=Right` | lidar.md, `tb3_lidar.py`, `tb3_teleop_sensors.py` |
| 각도식 | `angle_i ≈ π − i·(FOV/N)` (0=전방, +=좌/CCW) — 공식 대표 라벨과 부합하는 근사식 [DERIVED]. 우리 `config.LIDAR_FIRST_ANGLE=π`, `LIDAR_ANGLE_DIRECTION=−1`과 방향·대표 index가 **같다**. 픽셀 중심/endpoint의 sub-degree 차이는 UNCONFIRMED, S0 known-wall/point-cloud 대조 필요 | practice world에서 우리 코드로 방향 검증한 결과와도 일치 [MEASURED, practice] |
| 장착 위치 | extensionSlot Pose `(−0.03, 0, 0.153)` + LDS `translation (0,0,0.02)` → 로봇 frame **(−0.03, 0, 0.173)** [DERIVED] | PROTO |
| 공식 예제의 enable | `lidar.enable(100)` (주석으로 `enable(timestep)` 대안 표기), `enablePointCloud()` | `tb3_lidar.py` |
| sampling 주기 | 100 ms가 64 ms step의 배수가 아님 → 실제 갱신 간격 **UNCONFIRMED** (Webots 문서에 반올림 규칙 명시 없음). [DAY-OF: 로그로 측정] | lidar.md |

**설계에 주는 의미 (중요)**
1. **LiDAR 평면 높이 ≈ 0.173 m**(robot-local, 지면 기준은 pose에 따라 변함) → 평면 아래 물체(공식 바닥 공/사과 등)는 **LiDAR에 보이지 않을 수 있다.** 낮은 장애물 충돌 위험 + **target 거리를 LiDAR로 못 잴 수 있음** → 07의 카메라 기반 거리추정 필요.
2. **minRange 0.12 m, LiDAR가 회전중심보다 3 cm 뒤** → 로봇 앞 끝(x≈+0.036)에서 LiDAR까지 0.066 m < 0.12 m. **앞 범퍼에 거의 닿은 물체는 inf로 보인다.** 안전 모니터는 전방 inf를 "free"로 취급하면 안 되고, 정지거리를 minRange 바깥에 둬야 한다 (05).
3. 360 rays × 3.5 m → **기존 binary** `insert_scan` 약 6–12 ms [MEASURED, §8]. 아직 없는 log-odds 처리시간은 아니다.
4. inf는 장거리 no-return과 근접 occlusion을 구분하지 못한다. **기본 mapping은 inf/NaN/invalid ray를 skip**해 UNKNOWN/기존 점유를 보존한다. 정상 유한 hit만 minRange 이후 free + endpoint occupied로 갱신한다. 장거리 무반사라고 독립적으로 판별한 경우에만 제한적 clearing을 별도 검증한다.
5. fixed/cylindrical single-layer range image이므로 물리 LDS의 회전 주파수를 갱신율로 복사하지 않는다. `enable`의 주기는 ms, 첫 측정은 첫 주기 이후; `getSamplingPeriod()`는 요청값이지 관측된 실효 갱신율이 아니다. `robot.step` duration은 basicTimeStep의 배수여야 하며 센서 요청 100 ms의 실효 반올림은 UNCONFIRMED이다. 우리 시작안은 basicTimeStep의 정수 배수(64 ms world에서 128 ms), 동일 scan 중복 누적 금지. point-cloud는 불필요하면 끈다. 정지 환경의 동일 값만으로 새 scan 여부를 판정하지 않는다.

## 5. Camera [OFFICIAL]

| 항목 | 값 |
|---|---|
| device 이름 | `"camera"` |
| 장착 | extensionSlot 안 `translation (0.05, 0, −0.08)` → 로봇 frame **(0.02, 0, 0.073)** [DERIVED], 회전 없음(전방) |
| 해상도 | **640 × 480** |
| 수평 FOV | **1.0472 rad = 60°** |
| 수직 FOV | 2·atan(tan(30°)·480/640) = **46.8°** [DERIVED] |
| 초점거리 | f = 320 / tan(30°) ≈ **554.3 px** (정사각 픽셀) [DERIVED] |
| 이미지 형식 | `getImage()` BGRA 바이트 → 공식 예제는 `np.frombuffer(...).reshape((h,w,4))` 후 `cv2.cvtColor(BGRA2BGR)` |

## 6. Timestep / 월드 [OFFICIAL]

| 월드 | basicTimeStep | 로봇 controller | 비고 |
|---|---|---|---|
| `apartment.wbt` | **64** | tb3_teleop | 약 13×13 m, **Pedestrian 1명** (`--speed=0.2`, 궤적 지정), 색 사과 7개, 기타 사물 |
| `breakroom_ground_truth.wbt` | **64** | tb3_ground_truth, **`supervisor TRUE`**, Display | Ground-truth 데모 |
| `breakroom_teleop.wbt` | **64** | tb3_teleop_sensors, Display | 센서 데모 |
| `breakroom_teleop_yolo.wbt` | **64** | tb3_teleop_yolo | YOLO 데모 |
| `breakroom_ball.wbt` | 미지정 → Webots 기본 **32** | tb3_segmentation | 공 1개 |
| `breakroom_sensor_test.wbt` | 미지정 → **32** | tb3_lidar | |
| `empty.wbt` | 미지정 → **32** | 로봇 없음 | |

- Webots 기본값 32 ms: [worldinfo.md](https://cyberbotics.com/doc/reference/worldinfo) `basicTimeStep 32`.
- **결론: 공식 teleop/주행 월드는 64 ms, 일부 테스트 월드는 32 ms. 대회 월드 값은 [DAY-OF].** 우리 코드는 `robot.getBasicTimeStep()`을 읽어 쓰고, 주기는 **스텝 수가 아니라 시간(초)** 으로 정의해야 한다.
- breakroom 계열 약 13×8 m, apartment 약 13×13 m [DERIVED: 벽/바닥 좌표] → 현재 baseline `GRID_WIDTH/HEIGHT 160 @0.05 m = 8×8 m`는 **공식 예제 월드 크기보다 작다** (§8).

## 7. 비동기 컨트롤러 주장 재검증

| 구분 | 내용 | 상태 |
|---|---|---|
| Webots 문서 | "asynchronous mode is currently used only for the robot competitions" ([robot.md](https://cyberbotics.com/doc/reference/robot) synchronization) — **Webots 일반 대회 운영 방식에 대한 설명** | [OFFICIAL, Webots docs] |
| 공식 TECH WEEK 월드 | TurtleBot3Burger `synchronization` 기본 TRUE, 월드에서 변경 없음 → **동기 모드** | [OFFICIAL] |
| 의미 | 동기 모드의 step 사이 계산 동안 simulator가 기다리므로 **추가 simulation-time 진행은 없으나 샘플링·명령 전달 지연과 wall-clock 비용은 남는다**. 단 미션 제한시간이 **벽시계(real time)** 기준이면 계산 시간이 곧 손실 시간이다. | [DERIVED] |
| 대회 설정 | 대회 월드의 synchronization, 제한시간 기준(sim/real) | **[DAY-OF]** |

→ 기존 "대회는 비동기일 것" 서술은 **가능성**으로 강등. 설계 원칙(무거운 계산은 저주기, 안전 모니터는 매 스텝)은 두 경우 모두에서 유효하므로 유지.

## 8. 측정값 provenance [MEASURED]

기존 문서는 Windows 11 개발 PC 측정으로 기록했다. **bench3 결과에서 Intel64 Family 6 Model 186 Stepping 2 / Python 3.10.11 확인**. 순수 Python planning/binary mapping과 합성 영상 CV 실험을 구분한다. OS는 이전 pass 기록이며 이번에 재계측하지 않았다. **공식 로봇/월드 실행 성능이 아니다.** 재개 pass에서는 아래 원본 기록만 확인했고 새 benchmark를 실행하지 않았다.

| 측정 | 조건 | 결과 (median / p95) |
|---|---|---|
| (old, 2026-09-30 오전) A* 4-연결 | 160×160, 벽 2 + 무작위 300, 코너→코너, 3회 min | 49 ms (e-puck practice 기준 벤치) |
| A* 4-연결 | 320×320 (16 m @0.05), snake 합성 stress 조건, 7회 | **128 / 131 ms** |
| A* 4-연결 | 480×480 (24 m), 동일 | **514 / 525 ms** |
| BFS 거리장 (flat list) | 320 / 480 | **33 / 35 ms**, **101 / 116 ms** |
| inflate (r = (0.105+0.05)/0.05 = 3.1셀) | 320 / 480 | 10 / 12 ms, 20 / 21 ms |
| find_frontiers | 320 / 480 (중앙 면적 1/4 관측) | 24 / 26 ms, 58 / 59 ms |
| 기존 binary insert_scan 360 rays | 최대 3.5 m, 해당 grid 범위 | 6~8 ms (1–3 m), 9–12 ms (전부 max) |
| OpenCV blur+LAB+inRange+open+contours | 640×480 / 320×240 (cv2 5.0.0, numpy 2.2.6 — **공식 고정 버전과 다름**) | 2.5 / 2.8 ms, 1.1 / 1.5 ms |
| NumPy RGB 임계+중심 | 640×480 / 320×240 | 1.8 / 2.0 ms, 0.4 / 0.5 ms |

원본 위치: `C:/Users/shiny/AppData/Local/Temp/claude/D--Dev-projects-webots-test/ce3735b7-220a-4696-ba6d-e3216372ec93/scratchpad/`의 `bench.py`, `bench2.py`, `bench3.py`, `bench3_result.txt`, `bench_cv.py`, `bench_cv_result.txt`.

재현 조건: bench3 seed=0, 4-connected unit-cost Manhattan, 0.05 m/cell, 교대 gap 벽 3개 + N×2 random 장애물, start=(5,5), goal=거리장에서 가장 먼 도달 가능 셀(스크립트 출력의 corner-corner라는 라벨은 부정확). inflate 5회, A*/BFS/frontier 7회, binary insert 15회; 정렬 후 floor(0.95×(n−1)) 순서통계량을 p95로 표기했으므로 소표본 tail 추정 한계가 있다. frontier 입력의 관측 영역은 중앙 가로·세로 절반(면적 1/4)이다. CV는 합성 random BGRA + 직사각형, seed=0, 40회, 별도 warm-up 없음. CV 실행 Python/CPU는 결과 파일에 미기록(UNCONFIRMED); cv2/NumPy 버전만 확인. 서로 다른 detector의 정확도 비교 실험이 아니다. binary insert 결과로 log-odds set/dedup 비용을 검증했다고 주장하지 않는다. `camera.getImage()` 전송 비용과 Webots 렌더링 비용은 **미측정**.

## 9. 공식 controller 예제 요약 [OFFICIAL] (사용 목적 구분)

| controller | 하는 일 | 우리 사용 판단 |
|---|---|---|
| `tb3_teleop` | 키보드 W/A/S/D, 바퀴 3.0 rad/s | 구동 방향 확인 |
| `tb3_teleop_sensors` | LDS-01, 엔코더(getPositionSensor), accelerometer, gyro, **compass**, camera 읽기 + Display | 센서 이름·단위 확인. **Compass는 대회에서 사용 안 함 [ORGANIZER]** |
| `tb3_lidar` | `enable(100)`, 전후좌우 인덱스 출력 | LiDAR 순서 검증 기준 |
| `tb3_cam` | 카메라 enable (표시 코드는 주석) | – |
| `tb3_segmentation`, `tb3_teleop_cam` | **cv2**: GaussianBlur(11) → **LAB** → inRange → findContours → 최대 contour → minEnclosingCircle + moments 중심 | classical CV baseline 후보의 공식 교육 예시 (07) |
| `tb3_teleop_yolo` | `ultralytics` YOLO `yolo11n.pt`, `model.to("cpu")` (가중치 파일은 repo에 없음, `models/YOLO/.gitkeep`) | 참고만 (13) |
| `tb3_ground_truth` | **Supervisor** `getSelf().getPosition()/getOrientation()`로 정답 pose를 Display에 표시 | **데모/테스트 전용.** 우리 대회 controller의 localization 입력으로 쓰지 않는다 (허용 여부는 운영진 미확인 → 규정 위반이라 단정하지 않음) |

target 예제(`RedApple/GreenApple/PurpleApple/OrangeApple.proto`: scale=1 bounding sphere 지름 0.1 m의 사과(시각 mesh 줄기 포함 높이는 별도), Webots `Apple` 기반, 월드엔 공·고양이·소 등): **CV 튜토리얼 예시일 뿐, 대회 target 외형은 [DAY-OF]** (운영진: 당일 제공). 참고로 이 PROTO들은 `recognitionColors`를 가지므로 Webots Recognition으로 인식 가능한 형태이지만, 우리는 Recognition을 쓰지 않는다.

## 10. 공식 notebook 교육 내용 [OFFICIAL] (요약만; 본문 복제 금지)

| 절 | 내용 | 우리 설계와의 관계 |
|---|---|---|
| 1 문제 정의 | Perception → Planning → Action; "모든 목표 대상 구출 후 귀환" 예시 | target이 **복수**일 수 있음을 시사 → `REQUIRED_TARGETS` 파라미터 유지, 개수는 [DAY-OF] |
| 2 CV | 설치 `numpy==1.23.5 opencv-python==4.8.0.74 scikit-image==0.19.3 (+matplotlib 3.7.5)`; HSV(빨강 두 구간 OR), blur, morphology, contour, minEnclosingCircle, centroid; YOLO(PyTorch 2.8 CPU/CUDA, ultralytics) | 07, 13 |
| 3 Planning | Global/Local planner 개념, **Dijkstra vs A\*** (4-연결, Manhattan, 10×8 예제 grid) | 04: 공식 교육 = 4-연결 A*; 8-연결은 **우리 개선안** |
| 4 Action | **Look-ahead**: 최근접 waypoint(scipy `cKDTree`) → 경로거리 기반 look-ahead 점 → 로봇 frame 변환 → `κ = 2y/(x²+y²)`, `ω = vκ`, v는 상수 가능 → 바퀴속도 | 05: pure pursuit 계열 = 우리 RPP-lite의 핵심과 동일 |
| 4 Decision | **FSM**(탐험·접근·귀환 예시) vs **Behavior Tree**(Selector/Sequence/Parallel 등, Python 예시) | 10: 상태 수가 적어 FSM 유지 |
| 5 Webots | Ubuntu `.deb` R2025a 설치, 핵심 파라미터 WHEEL_RADIUS/WHEEL_SEPARATION/ROBOT_RADIUS | §3 |

## 11. 우리 baseline과 공식 환경의 불일치 (코드는 이번 작업에서 수정하지 않음 — 구현 단계 S0에서 처리)

| 항목 | 현재 baseline (practice e-puck) | 공식 | 처리 |
|---|---|---|---|
| `DEVICE_NAMES["lidar"]` | `"lidar"` | **`"LDS-01"`** | S0 |
| `DEVICE_NAMES["compass"]`, `["gps_debug"]` | None / `"gps"` (debug) | 대회 미사용 [ORGANIZER] | S0: competition 모드에서 비활성, GPS는 practice 전용 표기 |
| `WHEEL_RADIUS`, `AXLE_LENGTH` | 0.020, 0.052 | **0.033, 0.160** | S0 |
| `MAX_WHEEL_SPEED` | 6.28 | **6.67** | S0 |
| `ROBOT_RADIUS` | 0.037 | **0.105** notebook (외접 약 0.1103) | S0: 안전값은 0.111 올림 또는 검증된 polygon |
| `MAX_LINEAR_SPEED`, `MAX_ANGULAR_SPEED` | 0.08, 1.5 | 한계 0.22 m/s, 2.75 rad/s → 운용값은 [INITIAL TUNING] | S0/S2 |
| `GYRO_RAW_TO_RAD_S` | 13.315805/100000 (e-puck lookupTable) | **1.0** (TB3 Gyro lookupTable 없음) | S0/S1 |
| `LIDAR_MOUNT_OFFSET` | (0, 0) | **(−0.03, 0)** | S0 |
| `LIDAR_FIRST_ANGLE`, `DIRECTION` | π, −1 | 공식 라벨과 일치 | 유지 + S0 재검증 |
| `EMERGENCY_STOP_DISTANCE` | ROBOT_RADIUS + 0.04 = 0.077 (LiDAR 중심 기준) | LiDAR minRange 0.12보다 작아 **측정 불가 구간** | S0/S2 재설계 (05) |
| `GRID_WIDTH/HEIGHT` | 160 (8 m) | 공식 예제 월드 13 m급 | [DAY-OF] 월드 크기 반영, 기본 권장 ≥ 400 (20 m) 또는 해상도 재검토 (04) |
| `runtime.ini` | `$(LOCALAPPDATA)\...\python.exe` (Windows 전용) | 공식 기준 **Ubuntu 22.04** | Ubuntu에서는 `COMMAND` 로컬 수정 필요 (`python3.10` 등) — [DAY-OF: 대회 PC OS] |
| 연습 world | `worlds/rescue_baseline.wbt` (e-puck, 16 ms) | TurtleBot3 월드 64/32 ms | **practice 환경으로만 유지.** 공식 월드는 read-only 소스 참고; 후속 runtime 검증은 별도 허용된 개발 환경에서 수행 |

## 12. practice 환경 (e-puck) — 참고용으로 강등

아래는 대회 로봇 튜닝 근거로 **사용하지 않는다.** baseline 개발 초기에 쓴 연습 환경 기록이다.
- `worlds/rescue_baseline.wbt`: E-puck + 360° Lidar(maxRange 2 m), `basicTimeStep 16`.
- E-puck.proto R2025a: 바퀴 0.02 m, 카메라 0.84 rad(48°) 52×39 px, gyro lookupTable ±13.315805 ↔ ±100000.
- webots_ros2 e-puck Nav2 params (robot_radius 0.035, max_vel_x 0.05) — practice 전용 참고.

## 13. 일반 Webots 참고 (여전히 유효)

| 주제 | 사실 | 출처 |
|---|---|---|
| Display | 로봇에 Display 장치가 있어야 사용 (공식 ground_truth/teleop_sensors 월드는 extensionSlot에 Display 추가) | [display.md](https://cyberbotics.com/doc/reference/display) |
| Recognition | `recognitionColors` 있는 Solid를 ground truth로 인식 — 사용하지 않음 | [recognition.md](https://cyberbotics.com/doc/reference/recognition) |
| Supervisor | 노드 조작/정답 pose — 개발용 평가 도구로만 (분리된 테스트 월드/모드) | [supervisor.md](https://cyberbotics.com/doc/reference/supervisor) |
| runtime.ini | `$(VAR)` 치환 지원, `[python] COMMAND` | [controller-programming.md](https://cyberbotics.com/doc/guide/controller-programming) |
| Python 샘플 | lidar.py, display.py, encoders.py, gyro.py | [projects/samples/devices/controllers](https://github.com/cyberbotics/webots/tree/R2025a/projects/samples/devices/controllers) |
| Webots 대회 사례 | RoboCupJunior Rescue Simulation (Erebus): 정지 1 s → victim 판정, 20 s 정지 → LoP | [REFERENCE, NOT TECH WEEK RULE] [erebus](https://github.com/robocup-junior/erebus) |


## 기존 reference 링크 보존

아래는 이전 연구의 참고 링크를 보존한 것이다. e-puck은 practice/concept only, Erebus는 REFERENCE이며 TECH WEEK 규정이 아니다. 일반 API 예제는 공식 로봇의 장치 배치·competition 입력 허용을 증명하지 않는다.

- [camera](https://cyberbotics.com/doc/reference/camera) — platform/algorithm reference.
- [positionsensor](https://cyberbotics.com/doc/reference/positionsensor) — platform/algorithm reference.
- [e-puck_avoid_obstacles.c](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/controllers/e-puck_avoid_obstacles/e-puck_avoid_obstacles.c) — practice/reference only.
- [E-puck.proto](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto) — practice/reference only.
- [camera_recognition.py](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/camera_recognition/camera_recognition.py) — platform/algorithm reference.
- [display.py](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/display/display.py) — platform/algorithm reference.
- [encoders.py](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/encoders/encoders.py) — platform/algorithm reference.
- [gyro.py](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/gyro/gyro.py) — platform/algorithm reference.
- [lidar.py](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/lidar/lidar.py) — platform/algorithm reference.
- [sick_point_cloud.py](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/sick_point_cloud/sick_point_cloud.py) — platform/algorithm reference.
- [turtlebot](https://github.com/cyberbotics/webots/tree/R2025a/projects/robots/robotis/turtlebot) — platform/algorithm reference.
- [supervisor_draw_trail](https://github.com/cyberbotics/webots/tree/R2025a/projects/samples/howto/supervisor_draw_trail) — platform/algorithm reference.
- [nav2_params.yaml](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/resource/nav2_params.yaml) — practice/reference only.
- [drive_calibrator.py](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/drive_calibrator.py) — practice/reference only.
- [simple_mapper.py](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/simple_mapper.py) — practice/reference only.
- [webots_ros2_epuck](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_epuck) — practice/reference only.
- [MainSupervisor.py](https://github.com/robocup-junior/erebus/blob/master/game/controllers/MainSupervisor/MainSupervisor.py) — external competition reference only.
- [Robot.py](https://github.com/robocup-junior/erebus/blob/master/game/controllers/MainSupervisor/Robot.py) — external competition reference only.
- [player_controllers](https://github.com/robocup-junior/erebus/tree/master/player_controllers) — external competition reference only.
