# Camera coverage와 탐색 recovery

`camera_coverage.py`, `recovery.py`는 Webots에 의존하지 않는다. 실제 연결은
`main.py`에 있으며, 기본 STOP을 유지한다. 새 탐색·회전 동작은 명시적으로
`RESCUE_MODE=MISSION`으로 실행할 때만 사용한다.

## Camera coverage

- `CameraCoverageGrid.seen`은 occupancy와 같은 크기의 NumPy bool 배열이다.
  라이다로 FREE를 확인한 것과 카메라 방향에 들어온 것을 구분한다.
- 실제 카메라 프레임을 얻었을 때만 갱신한다. pose, 카메라 장착 위치,
  수평 FOV, 관측 거리로 후보 셀을 계산한다.
- NumPy로 여러 광선을 동시에 추적한다. OCCUPIED/UNKNOWN 셀 뒤는 가려진다.
  모서리에 닿는 광선도 차단해 대각선 틈으로 관측 영역이 새지 않게 한다.
  라이다 사각지대인 로봇 몸체 안 UNKNOWN만 통과할 수 있다.
- 기존 occupancy, log-odds, 팀 인터페이스는 바꾸지 않는다.
- 현재 위치에서 안전한 FREE 셀로 연결된 프런티어만 후보로 삼는다.
  그중 카메라가 아직 보지 못한 셀을 우선하고, 같은 조건이면 기존
  planner 점수를 사용한다. 도착하면 경로를 해제하고 한 바퀴 관측한다.
  회전 완료는 명령 시간 대신 odometry 각도 변화로 판단한다.
- 영상이 없거나 회전 제한 시간이 지나면 관측 회전을 종료하고 정지한다.
  LiDAR 안전 검사는 회전에도 마지막에 적용한다.
- 상태 로그의 `camera_seen_free`는 현재 FREE 셀 중 과거에 카메라 시야에
  들어온 셀의 비율이다. **사과 검출 성공률이나 전체 환경 탐색률이 아니다.**
  수직 시야, 조명, 영상 흐림, 검출 신뢰도는 이 2D 모델에 포함하지 않는다.
  모든 장소의 카메라 관측을 보장하는 별도 view planner는 아직 없다.

## 최소 recovery ladder

적용 범위는 EXPLORE이다. target 접근과 복귀의 기존 재시도 로직은 유지한다.

1. 기존 navigation controller가 일시적인 장애물 앞에서 정지·대기한다.
2. `REPLAN_REQUIRED` 또는 경로 생성 실패 시 경로를 지우고 정지한다.
3. 대기 후 같은 목표에 대해 최대 두 번 재계획한다. 경로 생성에 성공했다고
   재시도 횟수를 초기화하지 않아 움직이지 못하는 재계획 반복을 제한한다.
4. 계속 실패하면 해당 목표 주변을 TTL blacklist에 넣고 다른 목표를 선택한다.
5. 가능한 후보가 없으면 `SAFE_STOP`으로 정지하며 주기적으로 다시 확인한다.
   blacklist는 시간이 지나면 만료된다.

프런티어 도착 후 관측을 마친 위치도 잠시 제외한다. 상태 전환 시 이전
목표·경로·관측 회전을 취소한다. 지도 강제 삭제, 후진 탈출, 장애물 안에서
강제로 회전하는 기능은 구현하지 않았다.

탐색 재계획은 `OccupancyGrid.clearance_grid()`의 원형 금지 영역을 사용한다.
반경은 로봇 반경 + 안전 여유 + 셀 반대각선으로, 장애물 셀의 면적도 고려한다.
기존 planner의 사각형 soft cost 영역 전체를 금지해 출발점까지 막는 문제를 피한다.
알려진 FREE 셀과 4방향 경로를 사용하고, UNKNOWN 예외는 현재
로봇 몸체 안으로 제한한다. 예외는 임시 계획용 grid에만 적용한다.
기존 `planning.py`의 공용 API 및 반환 계약 문제 전체를 수정한 것은 아니다.

## 설정과 팀 연결

공용 `config.py`, `interfaces.py`는 수정하지 않았다. 기존 값을 사용한다.

| 동작 | 기존 설정 |
|---|---|
| 카메라 모델 | `CAMERA_HFOV`, `CAMERA_OFFSET` |
| 관측 거리 상한 | `min(TARGET_MAX_RANGE, LIDAR_MAX_RANGE)` |
| 커버리지 갱신 주기 상한 | `min(STATUS_PRINT_PERIOD, CAMERA_HFOV / (2 * MAX_ANGULAR_SPEED))` |
| 재계획 간격 | `NAV_REPLAN_PERIOD` |
| 목표 제외 시간 / 관측 회전 제한 시간 | `NAV_PROGRESS_TIMEOUT` |
| 목표 제외 반경 | `NAV_PROGRESS_DISTANCE` |
| 관측 회전 속도 | `NAV_ROTATE_SPEED` |
| 장애물 여유 공간 | `ROBOT_RADIUS + SAFETY_MARGIN` |

제외 시간과 반경 등은 기존 설정을 재사용하므로 독립적인 조정이 필요하면
config 담당자와 별도 항목을 합의한다. 로봇 물리 사양을 모듈에 복제하지 않는다.

## 검증

```powershell
py -3.10 -m unittest discover -s tests -p test_camera_coverage.py -v
py -3.10 -m unittest discover -s tests -p test_mapping_clearance.py -v
py -3.10 -m unittest discover -s tests -p test_recovery.py -v
py -3.10 -m unittest discover -s tests -p test_exploration_recovery.py -v
py -3.10 scripts/verify_baseline.py
py -3.10 scripts/verify_baseline.py --webots
py -3.10 scripts/verify_baseline.py --webots --world worlds/apartment_rescue.wbt
```

단위 테스트는 가림·좌표·프레임 누락·재시도 상한·목표 제외·정지·안전 차단과
MISSION 연결을 검증한다. 기본 Webots 검사는 STOP 모드 검사다.
대회 환경에서 제한 시간 내 사과 두 개 방문 및 복귀 성공 여부는 별도의
MISSION 주행으로 확인해야 한다.

### 2026-09-30 실행 결과

- 추가 단위·연동 테스트: **28개 PASS** (coverage 9, clearance 3, recovery 5,
  exploration 연결 11).
- `verify_baseline.py` 전체 결과: **FAIL**. 기존 Frontier/A*/planning integration/
  main 복귀 연결의 4개 테스트 그룹 실패가 남아 있다.
- Webots STOP 하위 검사: 기본 검증 월드와 아파트 월드 모두 **PASS**.
  controller 시작, 센서, 예외 없음, odometry 정지 유지 확인.
- 기본 월드 MISSION 약 30초: 경로 생성 → 도착 판정 → 카메라 관측 회전 진입
  확인. 이후 기존 안전 검사에서 STOP/UNKNOWN_SPACE가 발생해 회전이 차단됐고,
  관측 제한 시간 후 `SAFE_STOP`으로 정지했다. 한 바퀴 관측 완료나 미션 성공을
  확인한 결과는 아니다.
- 아파트 MISSION 약 25초: 경로 생성, 두 번 재계획, 목표 임시 제외 및 다른
  목표 선택 확인. 전방 LiDAR `inf`에 대한 기존 `UNKNOWN_SPACE` 정책 때문에
  주행은 진행되지 않았다.

팀원이 확인할 남은 사항:

| 담당 | 확인할 내용 |
|---|---|
| Planning | `cluster_centroid`/`manhattan` 누락, inflation·A*·local planner의 기존 테스트 계약 불일치 |
| 통합/Planning | 복귀 연결 테스트의 A* 모의 함수와 `connectivity` 인자 불일치 |
| Control | `navigation_control.SafetyMonitor`의 `inf`/`UNKNOWN_SPACE` 정책과 관측 회전 시 안전 정지 원인 |

사용자 요청에 따라 위 담당 코드와 공용 config는 수정하지 않았다.
