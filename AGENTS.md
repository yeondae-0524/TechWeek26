# AGENTS.md — AI 작업 규칙 (Claude / Codex 공통)

이 repository에서 작업하는 모든 AI agent와 팀원은 아래 규칙을 따른다.

## 환경

- **Webots R2025a** 시뮬레이션. 로봇은 공식 **TurtleBot3 Burger** (LDS-01 LiDAR + 640×480 카메라). 실제 로봇 코드가 아니다.
- **Python 3.10** (공식 기준: Ubuntu 22.04 + Python 3.10, Windows 가능). PATH, 다른 Python 설치, Webots 전역 설정을 수정하지 않는다.
  controller에서 Python 경로가 필요하면 controller 폴더의 `runtime.ini`를 쓰고, 개인 경로는 commit하지 않는다.
- 폴더 구조는 **공식 TECH WEEK baseline**(`kyu-rae-kim/PNU-TECHWEEK-260930` @ `383de18`)과 같다.

```text
controllers/
  tb3_*/                 # 공식 예제 controller (운영진 제공, 수정하지 않음)
  <우리 controller>/      # 새로 작성: controllers/<name>/<name>.py (Webots 규칙)
worlds/                  # 공식 world 7개 (운영진 허락으로 포함, 원본 그대로)
protos/                  # 공식 world가 쓰는 사과 PROTO
models/YOLO/             # YOLO 가중치 자리 (비어 있음, 가중치는 commit하지 않음)
docs/research/           # 알고리즘·공식 환경 조사 (00_RESEARCH_INDEX.md부터)
```

## 대회 규칙 (운영진 확인)

- **필수 센서**: wheel encoder, 2D LiDAR(`LDS-01`). **선택**: IMU(gyro/accelerometer).
- **사용 금지**: **Compass, GPS**. Supervisor ground-truth pose(`tb3_ground_truth` 방식)와 Webots Camera Recognition은
  대회 controller의 입력으로 쓰지 않는다. `breakroom_ground_truth.wbt`는 데모 전용이다.
- 사전 지도 없음, target 위치 모름(외형은 당일 공개), 시작 pose만 제공, target 방문 후 시작점 복귀,
  정적 장애물·움직이는 사람과 충돌 금지.

## 공식 파일 취급

- `controllers/tb3_*`, `worlds/*.wbt`, `protos/`는 **운영진 원본**이다. 수정하지 않는다.
  우리 controller로 world를 돌릴 때는 원본을 복사해 새 이름(예: `worlds/apartment_rescue.wbt`)으로 저장하고,
  로봇의 `controller` 필드만 바꾼다. `supervisor TRUE`인 world를 복사할 때는 FALSE로 바꾼다.
- 공식 notebook(`TECH-WEEK-26_Physical-AI.ipynb`), pdf, png는 **재배포·수정·재사용 금지 고지**가 있으므로
  이 repository에 넣지 않고, 코드·본문을 복사하지 않는다(사실·수치 인용만).

## 로봇 사실 (공식, 상세: docs/research/09_WEBOTS_REFERENCES.md)

| 항목 | 값 |
|---|---|
| 바퀴 반지름 / 간격 | 0.033 m / 0.160 m, 모터 최대 6.67 rad/s (→ 0.22 m/s) |
| 로봇 반경 | notebook 0.105 m, PROTO 외접 ≈ 0.110 m → 안전 계산은 0.111 m 이상 |
| device 이름 | `left wheel motor`, `right wheel motor`, `left wheel sensor`, `right wheel sensor`(rad), `LDS-01`, `camera`, `gyro`, `accelerometer` |
| LiDAR | 360 rays, 0.12–3.5 m, 로봇 frame (−0.03, 0, 0.173). index 180 = 전방, 90 = 왼쪽, 0 = 후방, 270 = 오른쪽. 범위 밖은 inf |
| Camera | 640×480, 수평 FOV 1.0472 rad (60°), 로봇 frame (0.02, 0, 0.073) |
| Gyro | lookupTable 없음 → rad/s 그대로 |
| basicTimeStep | apartment·breakroom_teleop 계열 64 ms, breakroom_ball·sensor_test·empty 32 ms (기본값) |

## 팀 규격 (새 코드에서 따른다. 바꾸려면 팀 합의)

```python
UNKNOWN, FREE, OCCUPIED = -1, 0, 1
grid[row][col]          # row = +y, col = +x, resolution 0.05 m
pose = (x, y, theta)    # m, m, rad. theta=0 → +x, 반시계 +, 범위 (-pi, pi]
target = {"found": bool, "cx": int|None, "direction": "LEFT"|"CENTER"|"RIGHT"|None, "area": float}
path = [(row, col), ...]   # 경로 없음 = []
waypoint = (x, y)
```

## 규칙

1. **기존 tests를 깨뜨리지 말 것.** 테스트를 지워서 통과시키지 않는다.
2. **unrelated file 수정 금지.** 작업 범위 밖 파일은 건드리지 않는다.
3. **공식 world/controller/PROTO 원본 수정 금지.** world 변경이 필요하면 복사본을 만들고 이유를 보고한다.
4. **새로운 dependency 추가 전 이유를 확인**(사람에게 묻기). 공식 교육 자료 기준 버전: numpy 1.23.5, opencv-python 4.8.0.74.
5. **hard-coded robot-specific values 최소화.** device 이름, wheel geometry, 속도, 안전거리 등은 설정 파일 한 곳에 모은다.
   Webots API 호출도 가능한 한 한 모듈에 모아, 나머지 로직은 Webots 없이 테스트할 수 있게 한다.
6. **stub을 실제 구현처럼 보고하지 말 것.** TODO/stub은 그대로 TODO라고 보고한다. 가짜 동작으로 테스트를 통과시키지 않는다.
7. 확정되지 않은 사양(대회 world, target 외형, 미션 시간 등)은 추측해서 확정하지 않는다.
8. 미완성 알고리즘이 로봇을 움직이게 하지 않는다. 기본 동작은 정지이고, 새 동작은 명시적인 모드에서만 켠다.
9. 새 기능에는 Webots 없이 도는 unit test를 `tests/`에 추가한다.
10. git commit/branch 조작은 사람이 요청할 때만 한다. 요청받았을 때는 아래 **Git 규칙**을 따른다.

## Git 규칙 (팀 합의)

- `main`에 직접 commit/push하지 않는다. 기능별 branch(`feat/<module>`)에서 작업하고 PR로 merge한다.
- **한 commit = 한 목적.** 관련 없는 파일 변경을 섞지 않는다.
- 메시지: `<type>: <구체적인 작업 내용>` (영어, 소문자 시작, 짧게). 예: `feat: add gyro heading fusion`

| Type | 사용 상황 | 예시 |
|---|---|---|
| `feat` | 새로운 기능 | `feat: add gyro heading fusion` |
| `fix` | 버그 수정 | `fix: prevent diagonal corner cutting` |
| `test` | 테스트 추가/수정 | `test: add safety monitor tests` |
| `refactor` | 동작 변화 없는 구조 개선 | `refactor: extract frontier scoring helper` |
| `docs` | README·문서 | `docs: update implementation roadmap` |
| `chore` | 설정·환경·관리 | `chore: add rescue mode configuration` |
| `perf` | 성능 개선 | `perf: optimize frontier distance field` |

- 피할 것: `fix: fix bug`, `feat: update code`, `chore: 수정`, 목적이 섞인 `feat: add mapping and fix control and update docs`.
- commit 전: 관련 테스트 실행 → Webots에서 동작 확인(controller/world를 바꿨다면) → `git status`, `git diff`로 의도한 파일만 바뀌었는지 확인.
- `git add .` 대신 **필요한 파일만** `git add <file>`.
- 테스트가 깨진 상태, 임시 디버깅 코드, 로그, 생성 파일, 개인 경로, 모델 가중치는 commit하지 않는다.

## 코드 수정 후

```bash
python -m unittest discover -s tests -p "test_*.py"   # tests/가 생긴 뒤부터
```

Webots에서 실행해 본 결과(에러 여부, 로봇 동작)와 테스트 결과(PASS/FAIL)를 그대로 보고한다.
