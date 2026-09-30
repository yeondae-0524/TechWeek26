# AGENTS.md — AI 작업 규칙 (Claude / Codex 공통)

이 repository에서 작업하는 모든 AI agent는 아래 규칙을 반드시 지킨다.

## 환경

- **Python 3.10** (`%LOCALAPPDATA%\Programs\Python\Python310\python.exe`; runtime.ini는 `$(LOCALAPPDATA)`로 참조).
  PATH, 다른 Python 설치, Webots 전역 설정을 수정하지 않는다. controller별 `runtime.ini`를 사용한다.
- **Webots R2025a** 시뮬레이션 환경. 실제 로봇 코드가 아니다. GPU 의존 코드 금지.
- 메인 controller: `controllers/rescue_robot/` (entry: `rescue_robot.py` → `main.py`)
- 메인 world: `worlds/rescue_baseline.wbt`

## 고정 규격 (변경 시 docs/INTERFACES.md, interfaces.py, tests 동시 수정 + 사람에게 확인)

```python
UNKNOWN = -1
FREE = 0
OCCUPIED = 1
grid[row][col]          # row = +y, col = +x, resolution 0.05 m
pose = (x, y, theta)    # m, m, rad. ENU world, theta=0 → +x, 반시계 +
target = {"found": bool, "cx": int|None, "direction": "LEFT"|"CENTER"|"RIGHT"|None, "area": float}
path = [(row, col), ...]   # 경로 없음 = []
waypoint = (x, y)
```

## 규칙

1. **기존 tests를 깨뜨리지 말 것.** 테스트를 지워서 통과시키지 않는다.
2. **unrelated file 수정 금지.** 작업 범위 밖 파일(특히 `archive/` 아래 연습 파일)은 건드리지 않는다.
3. **world file(.wbt) 임의 대규모 변경 금지.** 필요하면 백업 후 최소 변경하고 이유를 보고한다.
4. **새로운 dependency 추가 전 이유를 확인**(사람에게 묻기). 현재는 표준 라이브러리 + NumPy(카메라 frame)만 사용.
5. **hard-coded robot-specific values 최소화.** device 이름, wheel geometry, 속도, 안전거리 등은 `config.py`에만 둔다.
   Webots API 접근은 `devices.py`(와 `main.py`)에만 둔다.
6. **stub을 실제 구현처럼 보고하지 말 것.** TODO/stub은 그대로 TODO라고 보고한다. 가짜 동작으로 테스트를 통과시키지 않는다.
7. 확정되지 않은 로봇 사양(wheel radius, encoder 단위 등)은 추측해서 확정하지 않는다.
8. 미완성 알고리즘이 로봇을 움직이게 하지 않는다. 기본 동작은 정지(`BASELINE_MODE = "STOP"`).
9. 새 기능에는 Webots 없이 도는 unit test를 `tests/`에 추가한다.
10. git commit/branch 조작은 사람이 요청할 때만 한다. 요청받았을 때는 아래 **Git 규칙**을 따른다.

## Git 규칙 (팀 합의, 상세: [docs/DEVELOPMENT.md §2.1](docs/DEVELOPMENT.md))

- `main`에 직접 commit/push하지 않는다. 기능별 branch(`feat/<module>`)에서 작업한다.
- **한 commit = 한 목적.** 관련 없는 파일 변경을 섞지 않는다.
- 메시지: `<type>: <구체적인 작업 내용>` (영어, 소문자 시작, 짧게). 예: `feat: add gyro heading fusion`
- type은 `feat` / `fix` / `test` / `refactor` / `docs` / `chore` / `perf`만 사용. `fix: fix bug` 같은 추상적 메시지 금지.
- commit 전: 관련 테스트 → `python scripts/verify_baseline.py` (controller 동작 변경 시 `--webots`) → `git status`, `git diff`로 의도한 파일만 바뀌었는지 확인.
- `git add .` 대신 **필요한 파일만** `git add <file>`.
- 테스트가 깨진 상태, 임시 디버깅 코드, 로그, 생성 파일은 commit하지 않는다. `.wbt`는 불필요하게 수정하지 않는다.

## 코드 수정 후 반드시

```bash
python scripts/verify_baseline.py            # 항상
python scripts/verify_baseline.py --webots   # controller/world를 바꿨다면
```

결과(PASS/FAIL)를 그대로 보고한다.
