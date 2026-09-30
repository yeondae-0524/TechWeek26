# Development Guide

baseline 위에서 기능별 branch로 개발할 때의 작업 방식.
규격은 [INTERFACES.md](INTERFACES.md), 구조는 [ARCHITECTURE.md](ARCHITECTURE.md), AI 작업 규칙은 [../AGENTS.md](../AGENTS.md).

---

## 1. 개발 환경

| 항목 | 값 |
|---|---|
| Python | 3.10 — `%LOCALAPPDATA%\Programs\Python\Python310\python.exe` (runtime.ini가 `$(LOCALAPPDATA)`로 참조) |
| Webots | R2025a |
| 메인 world | `worlds/rescue_baseline.wbt` |
| 메인 controller | `controllers/rescue_robot/` (entry `rescue_robot.py` → `main.py`) |

- 터미널의 `python`이 3.10이 아니면 위 전체 경로로 실행한다.
- Webots 콘솔 출력은 Webots 창의 Console에 나온다. 파일로도 남기려면 환경변수 `RESCUE_LOG=<파일경로>`.

## 2. Branch 전략

```text
main                     항상 verify_baseline.py --webots 통과 상태 유지
 ├─ feat/detection
 ├─ feat/mapping
 ├─ feat/localization
 ├─ feat/planning
 ├─ feat/control
 └─ feat/integration     기능들을 main.py state machine에 연결
```

- branch 하나 = 담당 모듈 하나. **자기 모듈 파일 + 해당 test 파일**만 수정하는 것이 원칙.
- 공통 파일(`interfaces.py`, `config.py`, `main.py`, `docs/INTERFACES.md`)을 바꿔야 하면 팀에 먼저 공유하고 작은 PR로 분리한다.
- merge 전 조건: `python scripts/verify_baseline.py --webots` **ALL CHECKS PASSED**.
- 새 dependency 추가는 팀 합의 후. (현재: 표준 라이브러리 + NumPy)

### 2.1 Git commit 규칙 (팀 합의)

**형식**: `<type>: <작업 내용>` — 짧고 구체적으로, 메시지만 보고 무엇이 바뀌었는지 알 수 있게.

| Type | 사용 상황 | 예시 |
|---|---|---|
| `feat` | 새로운 기능 | `feat: add gyro heading fusion` |
| `fix` | 버그 수정 | `fix: prevent diagonal corner cutting` |
| `test` | 테스트 추가/수정 | `test: add safety monitor tests` |
| `refactor` | 동작 변화 없는 구조 개선 | `refactor: extract frontier scoring helper` |
| `docs` | README·문서 | `docs: update implementation roadmap` |
| `chore` | 설정·환경·관리 | `chore: add rescue mode configuration` |
| `perf` | 성능 개선 | `perf: optimize frontier distance field` |

피할 것: `fix: fix bug`, `feat: update code`, `chore: 수정`, `feat: robot stuff`,
그리고 목적이 섞인 `feat: add mapping and fix control and update docs`.

**Commit 순서**

```text
기능 구현 → 관련 테스트 → 기존 테스트 확인(verify_baseline.py, 필요 시 --webots) → git status / git diff → 필요한 파일만 add → commit
```

```bash
git add controllers/rescue_robot/localization.py controllers/rescue_robot/config.py tests/test_localization_gyro.py
git commit -m "feat: add gyro heading fusion"
```

commit 전 확인: 의도한 파일만 수정됐는가 · 다른 팀원 작업이 섞이지 않았는가 · 임시 파일/로그가 없는가 · `.wbt`가 의도치 않게 바뀌지 않았는가 · 테스트 통과.

**팀 최종 규칙**
1. `main`에 직접 commit/push하지 않는다.
2. 기능별 branch에서 작업한다.
3. 한 commit에는 하나의 목적만 넣는다.
4. 메시지는 `<type>: <구체적인 작업 내용>`.
5. type은 `feat` / `fix` / `test` / `refactor` / `docs` / `chore` / `perf`.
6. 관련 없는 파일 변경을 한 commit에 섞지 않는다.
7. 기존 테스트를 깨뜨린 상태로 commit하지 않는다.
8. 가능하면 commit 전 `verify_baseline.py`를 실행한다.
9. `.wbt`는 충돌이 크므로 불필요하게 수정하지 않는다.
10. 임시 디버깅 코드, 로그, 생성 파일은 commit하지 않는다.

기능별 commit 예시는 [research/11_IMPLEMENTATION_ROADMAP.md](research/11_IMPLEMENTATION_ROADMAP.md)의 Step 구분을 따른다
(예: `feat: add log-odds occupancy mapping` → `test: add log-odds mapping tests`).

### 2.2 구현 순서

- 구현 순서·수용 기준·branch별 담당 Step은 [research/11_IMPLEMENTATION_ROADMAP.md](research/11_IMPLEMENTATION_ROADMAP.md) (§2, §3.1 M0, §4)를 따른다.
  목표 설계는 [research/10_FINAL_ARCHITECTURE.md](research/10_FINAL_ARCHITECTURE.md).
- 새 동작 모드는 `RESCUE_MODE` opt-in으로만 추가하고 기본값 `STOP`은 유지한다.

## 3. 모듈별 작업 가이드

각 모듈의 TODO는 코드에 `TODO(feat/<branch>)`로 표시돼 있다. `grep -rn "TODO(feat/" controllers/` 로 찾는다.

### feat/detection — `detection.py`

- 목표: `detect_target(frame) -> target dict` 구현 (당일 공개되는 target 특징 기준).
- 입력: NumPy `(H, W, 3)` BGR, 또는 `None`. 출력 규격은 INTERFACES.md §4 그대로.
- 참고: `archive/practice_project/controllers/detection_controller/` (HSV 빨간 박스 실습 코드, `archive/practice_project/worlds/practice.wbt`로 실행 가능).
- 완료 기준: `found / cx / direction / area` 규격 유지, `test_interfaces.py` 통과, 저장한 샘플 frame으로 unit test 추가.
- OpenCV를 쓰려면 dependency 추가 합의 필요 (Python 3.10에는 이미 설치돼 있음).

### feat/mapping — `mapping.py`

- 현재: LiDAR ray marking (last write wins), 관측된 OCCUPIED는 지워지지 않음.
- TODO: log-odds 갱신, 움직이는 사람 흔적 제거, (선택) scan matching.
- 완료 기준: `world_to_grid / grid_to_world / in_bounds` 동작과 grid 값 규칙 유지, `test_grid.py` 통과.
- 확인: `RESCUE_MAP_DUMP=map.pgm` 으로 실행하면 2초마다 맵이 PGM으로 저장된다 (위쪽 = +y).

### feat/localization — `localization.py`

- 현재: wheel odometry (`DiffDriveOdometry`).
- TODO: gyro yaw rate fusion (`devices.read_gyro_yaw_rate()` 이미 제공), scan matching 보정.
- 완료 기준: `Localizer.update()` 가 항상 `pose = (x, y, theta)` 반환, theta는 `(-pi, pi]`.
- 확인: 콘솔 `[status]` 의 `pose=` 와 `gps_debug=` 비교. **GPS는 비교용일 뿐, 입력으로 쓰지 않는다.**
  (practice GPS는 로봇 중심보다 0.0095 m 뒤에 있음)

### feat/planning — `planning.py`

- 현재: 4-neighbour A*, obstacle inflation, frontier 검출/cluster.
- TODO: frontier 선택 전략 (거리, cluster 크기 등), 필요 시 8-neighbour / path smoothing.
- 완료 기준: 경로 없음 → `[]`, start == goal → `[start]`, OCCUPIED 통과 금지, `test_planning.py`, `test_frontier.py` 통과.

### feat/control — `control.py`

- 현재: primitive 명령, `set_velocity(v, w)`, emergency stop hook, `follow_waypoint` = 정지 stub.
- TODO: `follow_waypoint(pose)` (예: 목표 방향 회전 → 전진 P 제어), path tracking, recovery.
- 완료 기준: waypoint 도달 시 True 반환, emergency stop이 항상 우선, `test_control_localization.py` 통과.
- 확인: `--mode CONTROL_TEST` 로 구동계 방향 확인.

### feat/integration — `main.py`

- EXPLORE: frontier 선택 → A* → waypoint 추종.
- APPROACH_TARGET: camera 방향 + LiDAR 거리로 target world 위치 추정 → 안전거리까지 접근 → 기록.
- RETURN_HOME: `home_path` 추종 → `HOME_TOLERANCE` 이내면 DONE.
- 미완성 기능이 로봇을 움직이게 하지 않는다 (기본 `BASELINE_MODE = "STOP"`).

## 4. 테스트

```bash
python scripts/verify_baseline.py                               # syntax, 구조, unit test (몇 초)
python scripts/verify_baseline.py --webots                      # + Webots STOP 모드 smoke test
python scripts/verify_baseline.py --webots --mode CONTROL_TEST  # + 구동계 테스트
python -m unittest discover -s tests -p "test_planning.py" -v   # 파일 하나만 (프로젝트 루트에서)
```

- Webots 없이 돌 수 있는 로직은 모두 `tests/test_*.py` 에 unittest로 추가한다 (pytest 불필요).
- 테스트 파일 첫 줄에서 `import _path` 를 하면 `controllers/rescue_robot` 모듈을 import할 수 있다.
- Webots API(`controller` 모듈)는 `devices.py`, `main.py` 에서만 import한다. 다른 모듈에서 import하면 verify가 FAIL.
- `--webots` 는 Webots를 띄웠다가 자동으로 종료한다 (world는 저장하지 않음). 로드에 10~30초 걸린다.

## 5. 코드 규칙

- 로봇 사양·device 이름·속도·안전거리 → `config.py` 에만. 연습용 값은 `PRACTICE DEFAULT` 주석.
- 좌표/grid/pose/target 형식은 INTERFACES.md 규격만 사용.
- 미구현은 `TODO(feat/<branch>)` 로 명시하고, stub을 구현된 것처럼 보고하지 않는다.
- `.wbt` 는 최소 변경. 큰 변경 전 백업.

## 6. 대회 당일 체크리스트

1. 제공된 로봇으로 world를 열고 콘솔 `[devices] available: ...` 확인 → `config.DEVICE_NAMES` 수정
2. `config.py` 의 PRACTICE DEFAULT 값을 대회 로봇 값으로 교체
   - `WHEEL_RADIUS`, `AXLE_LENGTH`, `MAX_WHEEL_SPEED`, `ROBOT_RADIUS`
   - `ENCODER_UNITS` / `ENCODER_TICKS_PER_REV`, `GYRO_RAW_TO_RAD_S`
   - `LIDAR_FIRST_ANGLE`, `LIDAR_ANGLE_DIRECTION`, `LIDAR_MOUNT_OFFSET`
   - `START_POSE` (제공된 시작 position/orientation), `MISSION_TIME_LIMIT`
3. `--mode CONTROL_TEST` 실행 → 전진 = heading 방향, 좌회전 = theta 증가 확인
4. 부팅 로그 `[lidar] ranges: front/left/back/right` 가 실제 배치와 맞는지 확인
5. target 특징 공개 → `feat/detection` 에서 `detect_target` 구현
6. `verify_baseline.py --webots` 통과 후 통합

## 7. 디버깅 도구

| 도구 | 방법 |
|---|---|
| 콘솔 로그 파일 | 환경변수 `RESCUE_LOG=<path>` |
| 맵 이미지 | 환경변수 `RESCUE_MAP_DUMP=<path>.pgm` (2초마다 갱신) |
| 모드 전환 | 환경변수 `RESCUE_MODE=STOP` / `CONTROL_TEST` 또는 `config.BASELINE_MODE` |
| 상태 출력 | `[status]` 줄: state, pose, gps_debug, map 통계, frontier 수, target_found |
| LiDAR 방향 확인 | 부팅 시 `[lidar] ranges:` 한 줄 |
