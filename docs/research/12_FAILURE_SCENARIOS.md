# 12. Failure Scenarios (detect → decide → act → recover)

> 모듈 이름은 [10_FINAL_ARCHITECTURE.md](10_FINAL_ARCHITECTURE.md) 기준. 수치는 initial tuning suggestion.

| # | 상황 | Detect | Decide | Act | Recover / 검증 |
|---|---|---|---|---|---|
| 1 | **시작점에서 target이 바로 보임** | Initial Active Scan 중 TargetTracker가 M-of-N 확인 | 탐색 생략 가능; 시간 여유 확인 (`ETA_target + ETA_home`) | APPROACH_TARGET 직행 → 도착 후 나머지 스캔 방향 계속 → EXPLORE | 스캔을 중단한 방향은 camera_seen에 미표시로 남아 이후 탐색됨 |
| 2 | **좁은 복도** | 경로 주변 `obstacle_distance` 작음, 인플레이션 후 경로 없음 | 인플레이션만 문제인지 판단 (raw grid로는 경로 있음) | 현재 hit를 보존하고 footprint 이산화 오류만 완화; breadcrumb 재검증 후 저속 통과 | 안전 모니터 STOP 영역은 유지 — 몸체 반경 + 2 cm보다 좁으면 통과 불가로 판정 |
| 3 | **frontier가 벽 반대편** | Euclidean은 가깝지만 거리장 거리 큼 | utility가 경로 거리로 계산되므로 자동으로 낮은 점수 | 다른 frontier 선택 | 테스트로 고정 (02 §6) |
| 4 | **사람이 길을 막음** | 안전 모니터 STOP/SLOWDOWN, 경로 앞 셀 점유(새로 나타남) | 새 장애물 → WAIT | 정지 3~5 s, 1 Hz로 경로 유효성 확인 | 계속 막힘 → 재계획(우회) → 없으면 recovery ladder R1~R4 |
| 5 | **사람이 잠깐 막았다 사라짐** | 스캔에서 장애물 소멸 | 계속 진행 | 10회 유한 free 관측 때 복원; 가림/inf이면 시간 보장 없음 | ghost 잔류 시 #14 |
| 6 | **벽 코너에 끼임** | ProgressMonitor STUCK (명령 v>0인데 변위≈0), 가속도 spike | recovery ladder | R2 SPIN → R3 BACKUP(후방 검사) | R4: 앞 셀 collision_map 기록 + goal blacklist |
| 7 | **Odometry drift 증가** | gyro-odom yaw 불일치, (P2) CSM 점수 하락, 벽이 두 겹으로 그려짐 | 신뢰도 저하 → 속도 제한, 맵 갱신 일시 중지 | 제자리 회전으로 재관측, (P2) CSM 보정 | 당일 오도메트리 보정 절차로 예방 (03 B.4) |
| 8 | **target 발견 후 가려짐** | TargetTracker 미검출 연속 | memory 2~3 s 유지 | 마지막 bearing 방향 회전 → 실패 시 저장된 world 위치로 계획 접근 | 도착 후 look-around, 없으면 tentative 폐기 / confirmed는 TTL blacklist |
| 9 | **target 근처에 장애물** | standoff 후보 셀이 inflated OCCUPIED | relax_goal로 가시선 있는 다른 standoff | A* 접근 후 visual servo, 안전 모니터 우선 | 근접 불가하면 규정상 최소 거리 안인지 확인 후 기록 |
| 10 | **target 여러 개** | 확인된 target 목록 | dedup(0.25~0.3 m) 후 미방문 중 거리장 최근접 | 순차 접근, 방문 target 재검출 무시 | 모든 target 방문 or 총 개수 도달 → RETURN_HOME |
| 11 | **탐색 시간이 거의 끝남** | `time_left < SF·ETA_home + margin` (1 Hz 감시) | 현재 작업 중단 (target 접근 중이어도, 접근 비용이 예산 초과면) | RETURN_HOME | 복귀 중 target 발견 시 예산 여유 있을 때만 접근 |
| 12 | **home 경로가 막힘** | known-only A* 실패 | 08 §2.3 폴백 순서 | dynamic 셀 리셋 → 인플레이션 축소 → breadcrumb → unknown 허용 | 모두 실패 → 유한 ladder, 약 1 Hz 상태 확인·이벤트 재계획, 안전 정지 |
| 13 | **frontier 전부 실패** | 후보 없음 / 전부 blacklist | second chance 1회 → view frontier 단계 | blacklist 초기화 후 재시도, 그다음 카메라 미탐색 영역 | 그래도 없음 → RETURN_HOME (정상 종료) |
| 14 | **dynamic obstacle ghost 잔류** | 경로 불가인데 해당 셀이 최근 생성 + 로봇 시야에서 다시 관측 안 됨 | ghost 의심 | ghost 의심 셀 prior 리셋 (R1 CLEAR) → 재계획 | UNKNOWN으로 reset하면 1 hit부터 점유; 포화 FREE면 3 hit [INITIAL TUNING]. 현재 hit/충돌 기록은 reset에서 제외 |
| (추가) 15 | **계산 지연 (비동기 컨트롤러)** | 스텝 처리시간 > 예산 로그 | 무거운 작업 분산 | 정지 명령을 step으로 전달 후 계획; bounded 계산으로 safety deadline 확보 | 스텝 시간 max를 CI/테스트 로그로 추적 |
| (추가) 16 | **LiDAR 평면 아래 장애물** | 전진 명령 중 변위≈0 + 가속도 spike, 스캔엔 아무것도 없음 | 보이지 않는 장애물 | 후방 확인 시에만 후진 → collision_map 기록, 미확인 시 STOP | SemExp collision map ✅ |
| (추가) 17 | **오탐 target** | M-of-N 미달, 위치 분산 큼, 접근 시 사라짐 | tentative 유지 | 접근 전 확인 강화 | 도착 후 미검출이면 폐기 |

## 공식 환경 전환 audit (2026-09-30)

공식 source는 [09](09_WEBOTS_REFERENCES.md), 설계 계약은 [10 §11.1](10_FINAL_ARCHITECTURE.md). 다음은 후속 구현에서 검증할 시나리오이며 이번 pass에서 실행한 테스트가 아니다.

| # | Failure | Detection | Mitigation | Fallback |
|---|---|---|---|---|
| 18 | e-puck 튜닝 잔존 | 시작 로그의 robot profile·wheel r/L·device 이름을 공식값과 대조 | TB3 profile 분리, practice 값 이식 금지 | 불일치 시 STOP |
| 19 | robot radius mismatch 충돌 | notebook 0.105와 PROTO 외접 약 0.110 m 차이, corner clearance 검사 | 안전 외접 0.111(올림)+margin 0.05로 0.161 m inflation 시작 [INITIAL TUNING], 보수적 rasterization | 좁은 경로 거절·다른 goal |
| 20 | 640×480 CV가 control 차단 | getImage/검출 median·p95·max, 센서 age 기록 | 약 128 ms 시작, frame 비용 상한, 320×240와 intrinsics 동시 조정 | NumPy-only/검출 저주기, stale safety이면 정지 |
| 21 | 계산이 64 ms 초과 | 동기 여부·wall/sim 시간·planner 최대 지연 기록 | 이벤트 재계획, bounded work; 정지 setter 후 step으로 전달 | 계산 보류·coarse grid·안전 정지 |
| 22 | LiDAR angular indexing 반대 | 전/후/좌/우 벽과 180/0/90/270 대표 index 비교 | 방향·mount·beam-center 보정 검증 | 지도 갱신/주행 중단 |
| 23 | IMU 부재인데 gyro 필수 의존 | gyro 없는 mock/device 경로, NaN 검사 | optional fusion, 동일 pose interface | encoder-only; encoder/LiDAR도 없으면 STOP |
| 24 | Compass 우발 사용 | device 활성화·pose input 경로 정적 audit | competition adapter에서 미사용 [organizer-confirmed] | 입력 제거·encoder(+optional gyro) |
| 25 | GPS 우발 사용 | gps_debug·drift metric 입력 추적 | competition 경로 및 평가 gate에서 제거 | scan 재관측·지도 일관성 평가 |
| 26 | Supervisor pose가 competition에 혼입 | getSelf/getPosition/getOrientation 의존 추적 | demo/test 평가자를 별도 분리; 허용 범위 확인 | evaluator 없이 센서 기반 지표 사용 |
| 27 | OpenCV/YOLO 과도한 compute | module별 비용·인식 정확도 비교 | CPU classical baseline, DL은 필요 입증 후 | NumPy-only, inference 미완료 시 접근 보류 |
| 28 | tutorial target을 실제 target으로 착각 | 사과/공 색·크기 상수가 detector 규격에 고정됐는지 검토 | target appearance/size/count [DAY-OF CHECK] | 특징 미제공이면 탐색만, 방문 완료 판정 보류 |
| 29 | map coverage를 camera coverage로 착각 | LiDAR frontier 소진 뒤 unseen FREE 바닥/표면 확인 | 60° frustum·거리·가림·실제 frame 기반 camera_seen | 안전한 active scan/view frontier, 시간 부족 시 home |

공통: finite hit가 없는 inf는 근접 occlusion일 수도 있으므로 UNKNOWN으로 남긴다. swept footprint와 근접/저위 사각이 확인되지 않으면 spin/backup을 강행하지 않는다. encoder 공회전과 가속도 spike만으로 충돌 여부를 확정하지 않는다.
