# 매핑·위치 추정 검증

## 구현 상태

- `Localizer`는 기본 encoder-only를 유지한다. `GyroHeading` 객체를 명시적으로
  전달했을 때만 정지 편향 추정 후 방향 보정을 사용한다.
- 이동 거리는 encoder, 회전량은 보정 gyro와 encoder의 가중합을 사용한다.
  위치 적분에도 보정된 중간 방향을 적용한다.
- gyro 누락/NaN/과도한 회전율/유효하지 않은 dt는 해당 구간을 encoder-only로 처리한다.
- encoder가 없으면 pose를 유지하고 다음 정상 샘플에서 기준을 다시 잡는다.
  누락 구간의 이동량은 복원되지 않으므로 정확도가 유지됐다고 해석하지 않는다.
- 보정 입력은 `stationary=True`와 작은 바퀴 속도, 작은 gyro 회전율을 모두 만족해야 한다.
  움직이거나 샘플이 끊기면 진행 중인 편향 추정은 처음부터 시작한다.
- translation slip, finite 값으로 고장 난 gyro, Scan Matching은 처리하지 않는다.
- **현재 main.py의 자동 연결은 미완료**다. 공용 config와 main은 이 변경에서 수정하지 않는다.
  현재 플래너 인터페이스/통합 테스트 오류도 별도 수정이 필요하다.

## 통합 담당 연결 예시

아래 값은 실험용 초기값이며 최적값이나 안전 보장값이 아니다. 팀 합의 후
운용값은 공용 config에서 전달한다. 기존 pose와 map 데이터 규격은 바뀌지 않는다.

```python
gyro = GyroHeading(
    calibration_duration=1.0,       # 연속 정지 표본 시간, s
    stationary_wheel_speed=0.001,  # 양쪽 바퀴 각각의 이동 속도 상한, m/s
    max_stationary_rate=0.05,       # 정지 보정 중 허용 raw gyro, rad/s
    max_rate=3.0,                   # 유효 gyro 회전율 상한, rad/s
    max_dt=0.2,                    # 적분할 수 있는 단일 표본 간격, s
    weight=1.0,                    # 보정 후 gyro 방향 비중, 0~1
)
localizer = Localizer(home_pose, wheel_radius, axle_length, encoder_to_rad,
                      gyro_heading=gyro)
pose = localizer.update(encoders, gyro_yaw_rate, dt,
                        stationary=confirmed_stationary)
```

`confirmed_stationary`는 통합 담당이 모터를 정지시킨 보정 구간에서만 True로 전달한다.
정지 명령 자체가 실제 정지를 보장하지는 않는다. 외력/충돌이 없는 초기 정지 구간에서
센서 조건까지 확인한다. 정지 보정이 끝나지 않으면 encoder-only가 유지된다.
reset은 `localizer.reset(pose)`를 사용해 encoder 기준과 gyro 보정을 함께 초기화한다.
gyro 유효성 판정은 취득 시각 검증을 대신하지 않으므로 센서 정체 감시도 통합에서 필요하다.

## 오프라인 지도 비교

`scripts/replay_mapping.py`는 기록 파일을 읽는 도구이며 로봇을 움직이거나 센서를 기록하지 않는다.
실제 로그 수집은 통합 담당이 추가해야 한다. 기존 `[status]` 로그만으로 재생할 수 없다.

JSONL 각 줄에 다음 필드를 기록한다:

- `time`: 증가하는 simulation 시각, 초
- `encoders`: 절대 좌/우 encoder 값 `[left, right]`, 누락 시 null
- `gyro_yaw_rate`: rad/s, 누락 시 null
- `ranges`: LiDAR 광선 순서의 360개 거리(m); no-return은 null
- `stationary`: 정지 보정 구간 여부, 생략하면 false

중복 스캔으로 증거를 과도하게 누적하지 않도록 새 센서 표본마다 한 줄만 기록한다.
센서 종류별 주기가 다르면 동기화해서 기록한다. 설정과 로봇 사양은 현재 config를 사용한다.
encoder 누락 및 이후 기준 재설정 표본에서는 지도 삽입을 건너뛴다.

```powershell
$gyroOptions = '{"calibration_duration":1.0,"stationary_wheel_speed":0.001,"max_stationary_rate":0.05,"max_rate":3.0,"max_dt":0.2,"weight":1.0}'
$gyroOptions | Set-Content -Encoding UTF8 "$env:TEMP/techweek-gyro-options.json"
py -3.10 scripts/replay_mapping.py recording.jsonl --output output/mapping/run01 --start-pose 0 0 0 --gyro-options-file "$env:TEMP/techweek-gyro-options.json"
```

시작 pose는 반드시 해당 기록의 제공된 위치/방향으로 교체한다(theta는 rad).
gyro 옵션을 생략하면 두 지도 모두 encoder-only다.
출력 파일이 이미 있으면 덮어쓰지 않으며, 새 run 폴더를 지정해야 한다.

출력: `encoder.pgm`, `gyro.pgm`(검정=장애물, 흰색=빈 공간, 회색=미관측),
`poses.csv`, `summary.json`(지도 origin·해상도·셀 수·편향·방향 차이).
두 추정치의 차이는 실제 오차가 아니다. ground truth 없이 정확도 개선을 단정하지 않는다.
생성 파일과 센서 기록은 commit하지 않는다.

## 실험 순서와 판단 기준

1. 정지: 충분한 초기 보정 구간을 기록하고 bias 준비 여부 및 지도 흔들림을 확인한다.
2. 직진/회전: 같은 벽을 다시 관측했을 때 지도 선이 두 겹으로 갈라지는지 확인한다.
3. 사각형/왕복: 같은 경로를 encoder-only와 gyro 옵션으로 재생하고 지도·위치 차이를 비교한다.
4. 사람 통과: 유효한 free ray 재관측 후 잔상이 해제되는지 확인한다. 가려진 영역의
   잔상이 시간만으로 삭제되는 것은 기대하지 않는다.

현재 합성 테스트는 알려진 입력에 대한 수학적 동작 검증이다. 실제 Webots 주행 정확도,
움직이는 사람 회피, 10분 미션 성공은 별도 통합 실험으로 확인해야 한다.
