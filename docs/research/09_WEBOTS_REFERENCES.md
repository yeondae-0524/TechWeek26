# 09. Webots References (R2025a, Python)

> ✅ = Webots R2025a 태그 소스/문서에서 직접 확인 (`cyberbotics/webots` @ c6793d8, 2025-02-04). 📄 = 문서 링크만.

---

## 1. 우리 설계에 직접 영향을 주는 사실

| 항목 | 사실 | 우리에게 주는 의미 | 출처 |
|---|---|---|---|
| **Controller 동기화** | `Robot.synchronization` 기본 TRUE(시뮬이 컨트롤러를 기다림). FALSE면 컨트롤러를 기다리지 않음. 문서: *"asynchronous mode is currently used only for the robot competitions"* | 대회가 FALSE면 **계산 시간 = 제어 지연**. 무거운 계산(전역 계획 ~80–100 ms)을 매 스텝 돌리면 위험. `robot.getSynchronization()` 로그로 당일 확인 | ✅ [robot.md](https://cyberbotics.com/doc/reference/robot) |
| Lidar 데이터 순서 | range image는 **왼쪽→오른쪽**, 위층→아래층 | baseline이 확인한 "index 0 = 후방, 시계방향"과 일치(360° 기준). 공식 로봇에서 재검증 | ✅ [lidar.md](https://cyberbotics.com/doc/reference/lidar) |
| Lidar 범위 밖 | `minRange` 미만 / `maxRange` 초과 → **inf** | 현재 `scan_to_world_points`가 inf를 "free up to max_range"로 처리 — 맞음. 단 minRange 미만(너무 가까움)도 inf라 **근접 물체가 free로 보일 수 있음** → 안전 모니터는 inf를 "정보 없음"으로 취급해야 | ✅ lidar.md |
| Lidar noise | `noise` σ = noise × maxRange (가우시안) | 공식 월드에 noise가 있으면 log-odds와 `min_points` 필수 | ✅ lidar.md |
| Lidar 내부 | 내부적으로 depth camera(OpenGL) | 360° fixed lidar는 여러 카메라 렌더 → 시뮬 비용 | ✅ lidar.md |
| Camera FOV | 수평 FOV, 수직 FOV = `2·atan(tan(FOV/2)·H/W)` | bearing 계산식(07) | ✅ [camera.md](https://cyberbotics.com/doc/reference/camera) |
| Recognition | `recognitionColors`가 있는 Solid를 카메라가 **ground truth로 인식**, segmentation도 제공 | 규정상 금지 가능성 높음 → 사용하지 않는 것을 기본으로, 허용 여부 확인 | ✅ [recognition.md](https://cyberbotics.com/doc/reference/recognition) |
| Gyro/PositionSensor | `lookupTable` 3열 = 상대 노이즈, `resolution` −1 = 무한 | e-puck gyro 0.3% 노이즈, 엔코더 2π/1000 | ✅ [gyro.md](https://cyberbotics.com/doc/reference/gyro), [positionsensor.md](https://cyberbotics.com/doc/reference/positionsensor) |
| Display | Robot 자식 노드로 **Display 장치를 world/proto에 추가**해야 사용 가능, `attachCamera`, `setColor`, `drawPixel`, `imageNew` 등 | 공식 로봇에 Display가 없으면 world 수정이 필요 → 금지일 수 있음. **대안: PGM/PPM 파일 덤프**(baseline `save_pgm` 확장)로 충분 | ✅ [display.md](https://cyberbotics.com/doc/reference/display), 샘플 [`display.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/display/display.py) |
| Supervisor | 노드 조작/정답 좌표 획득 가능 | **대회 컨트롤러에서는 사용 금지 가정.** 연습 월드의 **테스트 하네스**(충돌 카운트, drift 측정)로만 | ✅ [supervisor.md](https://cyberbotics.com/doc/reference/supervisor) |

## 2. Practice robot: E-puck (R2025a) 핵심 수치 ✅ [E-puck.proto](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto)

| 항목 | 값 |
|---|---|
| 바퀴 반경 | 0.02 m (config `WHEEL_RADIUS 0.020`) |
| 모터 maxVelocity | 6.28 rad/s (v1) / 7.536 rad/s (e-puck2) → 최대 선속도 0.1256 m/s (v1) |
| 엔코더 resolution | 0.00628 rad |
| 카메라 | FOV 0.84 rad, 52×39 px, noise 0 (기본) |
| Gyro | ±13.315805 rad/s ↔ ±100000 raw, 노이즈 0.003 |
| Accelerometer | ±100 ↔ ±100, 노이즈 0.003 |
| baseline world Lidar | 360 rays, FOV 6.28318, minRange 0.02, maxRange 2, `translation 0.0095 0 0.005` (config offset (0,0)과 9.5 mm 차이 ⚠️) |
| `basicTimeStep` | 16 ms (rescue_baseline.wbt) |

## 3. 바로 참고할 공식 예제 (Python 우선)

| 예제 | 경로 | 볼 내용 |
|---|---|---|
| lidar | [`projects/samples/devices/controllers/lidar/lidar.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/lidar/lidar.py) | `enable`, `enablePointCloud`, 기본 회피 |
| display | [`.../display/display.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/display/display.py) | 카메라 오버레이, 텍스트 |
| encoders | [`.../encoders/encoders.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/encoders/encoders.py) | PositionSensor 사용 |
| gyro / compass / accelerometer | [`.../gyro/gyro.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/gyro/gyro.py) 등 | 값 읽기·단위 |
| sick_point_cloud | [`.../sick_point_cloud/sick_point_cloud.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/sick_point_cloud/sick_point_cloud.py) | 2D 스캐너 점군 |
| camera_recognition | [`.../camera_recognition/camera_recognition.py`](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/devices/controllers/camera_recognition/camera_recognition.py) | Recognition API (사용 금지 가능성 확인용) |
| e-puck obstacle avoidance | [`projects/robots/gctronic/e-puck/controllers/e-puck_avoid_obstacles/e-puck_avoid_obstacles.c`](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/controllers/e-puck_avoid_obstacles/e-puck_avoid_obstacles.c) | 근접센서 Braitenberg (C) |
| TurtleBot3 Burger | [`projects/robots/robotis/turtlebot/`](https://github.com/cyberbotics/webots/tree/R2025a/projects/robots/robotis/turtlebot) (`turtlebot3_burger.wbt`, `turtlebot3_ostacle_avoidance.c`) | 공식 로봇이 TB3류일 경우 대비 |
| supervisor_draw_trail | [`projects/samples/howto/supervisor_draw_trail`](https://github.com/cyberbotics/webots/tree/R2025a/projects/samples/howto/supervisor_draw_trail) | 테스트 월드에서 궤적 시각화 (심사용 아님) |

## 4. Webots 기반 관련 프로젝트

| 프로젝트 | 관련성 | 볼 것 |
|---|---|---|
| **webots_ros2 e-puck** ✅ [repo](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_epuck) | 같은 로봇의 Nav2 튜닝값 | [`resource/nav2_params.yaml`](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/resource/nav2_params.yaml): robot_radius 0.035, max_vel_x 0.05, max_vel_theta 1.0, xy_goal_tolerance 0.05, local costmap res 0.01, controller 20 Hz; [`drive_calibrator.py`](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/drive_calibrator.py): 오도메트리 보정 절차; [`simple_mapper.py`](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_epuck/webots_ros2_epuck/simple_mapper.py): 참고 가치 낮음 |
| webots_ros2 TurtleBot3 ✅ | Nav2+SLAM 예제 (ROS 전용 → 구조만) | README |
| **RoboCupJunior Rescue Simulation (Erebus)** ✅ [repo](https://github.com/robocup-junior/erebus) | **Webots + Python 구조 대회**: 미로 탐색·지도·victim 식별·시작점 복귀 | [`player_controllers/`](https://github.com/robocup-junior/erebus/tree/master/player_controllers) 예제, 심판 [`MainSupervisor.py`](https://github.com/robocup-junior/erebus/blob/master/game/controllers/MainSupervisor/MainSupervisor.py) (정지 1 s → victim 판정, 20 s 정지 → LoP 재배치), [`Robot.py`](https://github.com/robocup-junior/erebus/blob/master/game/controllers/MainSupervisor/Robot.py) `time_stopped` |

## 5. 우리 프로젝트에서 Webots 관련 권장 사항
1. `devices.py`에서 시작 시 로그: `getBasicTimeStep`, `getSynchronization`, lidar `getFov/getHorizontalResolution/getMaxRange/getMinRange`, camera `getFov/getWidth/getHeight`.
2. 무거운 연산은 **스텝 예산**(예: 스텝당 8 ms)으로 스케줄링; 매 스텝 실행 시간 최대값/평균 로그.
3. 시각화는 파일 덤프(PGM/PPM, 1~2 s 간격 + 종료 시) — 의존성 추가 없음.
4. GPS/Supervisor/Recognition은 **디버그·테스트 하네스 전용**.
