# rescue_robot 주행 기능

주 실행 파일은 `main.py`이고 Webots는 `rescue_robot.py`를 통해 실행합니다.
이전에 별도로 만든 주행 로직을 이 controller 안으로 통합했습니다.

## 파일 역할

- `main.py`: 센서 → 기존 Localization → Mapping/Detection → 경로 추종 → 마지막 안전 검사.
- `navigation_control.py`: waypoint 추종, 회전 PID, 장애물 감속/정지, 궤적 충돌 예측, 진전 감시.
- `control.py`: 기존 모터 속도 변환과 사각 감시를 유지합니다.
- `config.py`: 공식 로봇 사양과 주행 조정값을 한 곳에서 관리합니다.
- `localization.py`: 기존 encoder 기반 pose를 사용합니다. 별도 odometry는 중복 실행하지 않습니다.

## Webots 실행

첫 직선/회전 시험은 Webots R2025a에서 `worlds/control_arena_test.wbt`를 엽니다.
3×3m 경기장과 0.5m 벽으로 유한한 LiDAR 관측을 확보하며 시작 pose는 (0,0,0)입니다.
정답 pose나 지도는 controller에 제공하지 않습니다.
휴게실 시나리오에는 `worlds/breakroom_control_test.wbt`를 사용합니다.
공식 `breakroom_sensor_test.wbt`의 복사본이며 controller와 제공된 시작 pose의 customData를 설정했습니다.
기본 모드는 `STOP`입니다. Python은 대회 기준 3.10을 사용해야 합니다.
PATH와 Webots 전역 설정은 이 작업에서 바꾸지 않았습니다.
기본 START_POSE는 팀 검증 world `rescue_baseline.wbt`의 `(-0.5, -0.8, 0도)`입니다.
팀 시험 world는 `customData.start_pose`로 각 world가 제공한 시작 pose를 전달하며 이 값을 우선 사용합니다.
새 world에 customData가 없으면 config.START_POSE를 제공된 시작 pose로 맞춰야 합니다.

Webots를 시작하는 PowerShell에서 다음 환경 변수를 설정합니다.
이미 열린 Webots에는 새 환경 변수가 전달되지 않을 수 있습니다.

```powershell
$env:RESCUE_MODE = 'NAV_TEST'
$env:RESCUE_WAYPOINTS = '[[0.4,0],[0.4,0.4]]'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' "$PWD\worlds\control_arena_test.wbt"
```

이 좌표는 **시작점 기준 미터 좌표**입니다. 시작 방향이 +x, 시작 왼쪽이 +y입니다.
`main.py`가 제공된 시작 pose로 변환하여 기존 Localization/Mapping 좌표계와 맞춥니다.
예시 좌표는 공식 미션 경로나 사전 지도가 아닙니다. 시험 장소의 통행 가능 여부를 먼저 확인합니다.
경로가 없으면 `NO_PATH`로 정지하고, 모든 waypoint에 도달하면 `DONE`으로 정지합니다.

다른 모드:

- `STOP`: 경로나 미션 상태와 관계없이 모터를 정지시키는 기본 모드.
- `CONTROL_TEST`: 기존 전진/회전/정지 구동계 점검. 안전 검사도 적용합니다.
- `MISSION`: 팀 통합 개발용. 탐색 목적지 선택과 대상 접근은 아직 TODO입니다.

## Planner 및 Mapping 연결

```python
mission.set_navigation_path([(x1, y1), (x2, y2)])
mission.set_navigation_grid_path([(row1, col1), (row2, col2)])
```

직접 전달하는 미터 경로는 `mission.localizer.pose`와 **같은 좌표계**를 사용합니다.
`NAV_TEST` 환경 변수의 시작점 상대 좌표와 구분해야 합니다.
셀 경로는 현재 지도 `grid_to_world()`로 셀 중심 좌표로 변환합니다.
지도 원점은 [0][0] 셀의 왼쪽 아래 모서리이고 row는 +y, col은 +x입니다.
빈 경로는 도착 성공이 아니라 정지입니다.

`mission.replan_requested`가 True이면 모터가 정지하고 새 경로를 기다립니다.
Planner가 우회 경로를 계산한 뒤 위 함수를 호출하면 진전 감시와 재계획 요청이 초기화됩니다.
미터 좌표를 임의의 다른 frame으로 보내면 안 됩니다. Localization의 보정 pose는
기존 `Localizer`에 연결해야 하며 gyro 융합/scan matching 자체는 이 작업에서 구현하지 않았습니다.

## 장애물 처리

1. waypoint를 추종하면서 매 step LiDAR를 확인합니다.
2. 장애물 접근 시 감속하고, 가까운 점/예측 충돌/미관측 공간에서는 정지합니다.
3. 기다리는 동안 장애물이 사라지면 기존 경로를 이어 갑니다.
4. 4초 이상 계속 막히거나 전진 명령 대비 위치 진전이 없으면 재계획을 요청합니다.
5. `RETURN_HOME`에서는 로봇 반경과 여유 거리로 팽창한 지도를 A*에 전달해 새 경로를 얻습니다.
   현재 5cm 셀에서는 0.161m 안전 반경을 4셀로 올림해 벽 주변을 통과 금지로 만듭니다.

Control이 임의로 좌우 우회 경로를 만들어 내지는 않습니다. 우회는 Planner가 맡습니다.
막힌 복귀 경로는 새 계획을 하기 전에 정지 명령을 한 번 Webots step으로 전달합니다.
NAV_TEST의 시험 경로가 계속 막히면 Planner가 새 경로를 줄 때까지 정지합니다.

## 검증 범위와 남은 작업

공식 기하값을 사용하고, 운용 속도/거리/PID 계수는 초기 조정값입니다.
PID의 I 항은 기본 비활성화이며 실제 주행에서 조정해야 합니다.
LiDAR의 inf는 미관측 값으로 취급하므로 진행 방향의 inf가 이동을 막을 수 있습니다.
제자리 회전은 전 방향의 유한 관측을 요구합니다. 안전 확인 없이 회전/후진을 강행하지 않습니다.

현재 스캔 점의 1초 예측은 정적 장애물 가정입니다. 움직이는 사람의 속도 예측,
LiDAR 아래 장애물, 실제 제동 거리와 지연, encoder 미끄러짐은 별도 검증이 필요합니다.
스캔에는 현재 simulation 시간을 붙이며 독립적인 센서 취득 시각은 확인하지 못합니다.
Webots 실제 주행 결과와 Python 3.10 실행 결과는 오프라인 테스트와 별개입니다.

저장소 루트에서 전체 단위 테스트를 실행합니다.

```powershell
py -3.10 scripts/verify_baseline.py
py -3.10 scripts/verify_baseline.py --webots
py -3.10 -m unittest discover -s tests -p "test_*.py"
```

## 실제 실행 확인 (2026-09-30)

- Python 3.10.11: 전체 단위 및 main 통합 테스트 147개 PASS. 기존 테스트는 유지하고 회귀 테스트 24개를 추가했습니다.
- Webots R2025a, `control_arena_test.wbt`, 64ms 동기 실행, 별도 숨겨진 검증 프로세스.
- 직선 `[[0.3,0]]`: `REACHED` → `DONE`. encoder 추정 위치는 약 (0.201, 0.000)m.
- 코너 `[[0.4,0],[0.4,0.4]]`: `REACHED` → `DONE`. encoder 추정 pose는 약 (0.405, 0.303, 89.6도).
- 최종 도착 허용 오차 0.10m이므로 정확한 목표 좌표보다 먼저 정지할 수 있습니다.
- 두 시나리오 모두 시작 시 유한한 LiDAR 관측 360/360, inf=0을 확인했습니다.
- 이 결과는 도착 상태와 encoder pose 확인이며 정답 pose 비교/충돌 횟수 계측은 수행하지 않았습니다.
- 휴게실의 전방 inf는 UNKNOWN_SPACE로 정지하는 기존 정책을 유지합니다. 넓은 공간을
  이동하려면 관측 이력/지도와 사각 정보를 결합한 안전 정책을 별도로 설계·검증해야 합니다.

## 병합 후 통합 수정 (2026-09-30)

- Planning API의 `min_size`, `allow_unknown`, `manhattan`, `cluster_centroid` 계약을 맞췄습니다.
- 장애물 팽창은 실수 반경을 지원하고 팀 점유지도 형식을 유지합니다. 원본 지도는 변경하지 않습니다.
- 범위 밖/점유된 끝점, 대각선 모서리 통과, 치명 비용지도 셀은 A*에서 차단합니다.
- 실제 복귀 함수의 팽창 → A* 우회 → 지도 셀 중심 waypoint 인계를 오프라인으로 검증합니다.
- 검증 스크립트가 모든 test 파일을 포함하며 한글 성공/오류 로그와 조기 종료도 검사합니다.
- 제공된 world 시작 pose를 우선 사용하며 휴게실 시험 복사본에도 이 값을 추가했습니다.
- Webots STOP 유지 및 코너 NAV_TEST → DONE을 다시 확인했습니다. 전체 자율 탐색·대상 접근은 TODO입니다.

최종 확인 결과:

- `py -3.10 -m unittest discover -s tests -p "test_*.py"`: 147개 PASS.
- `verify_baseline.py --webots`: 모든 검사 PASS. 기본 2m world에서 STOP의 최대 encoder 편차 0.0000m/0.0도.
- `verify_baseline.py --webots --mode CONTROL_TEST --world worlds/control_arena_test.wbt`: 모든 검사 PASS.
  전진 0.110m, 좌회전 +65.9도, 우회전 후 약 0도, 최종 DONE.
- `verify_baseline.py --webots --world worlds/breakroom_control_test.wbt`: 모든 검사 PASS.
  customData 시작점 (-1.265, 1.811, -24.3도) 확인, STOP의 최대 encoder 편차 0.0000m/0.0도.
- 별도 코너 NAV_TEST `[[0.4,0],[0.4,0.4]]`: REACHED → DONE, encoder pose (0.405, 0.303, 89.6도).
- 장애물 팽창·A* 우회·실제 복귀 함수의 경로 인계는 오프라인 회귀 테스트로 확인했습니다.
  장애물이 있는 복귀 전체 주행과 움직이는 사람 대응은 아직 Webots에서 검증하지 않았습니다.
