# 12. Failure Scenarios (detect → decide → act → recover)

> 모듈 이름은 [10_FINAL_ARCHITECTURE.md](10_FINAL_ARCHITECTURE.md) 기준. 수치는 initial tuning suggestion.

| # | 상황 | Detect | Decide | Act | Recover / 검증 |
|---|---|---|---|---|---|
| 1 | **시작점에서 target이 바로 보임** | Initial Active Scan 중 TargetTracker가 M-of-N 확인 | 탐색 생략 가능; 시간 여유 확인 (`ETA_target + ETA_home`) | APPROACH_TARGET 직행 → 도착 후 나머지 스캔 방향 계속 → EXPLORE | 스캔을 중단한 방향은 camera_seen에 미표시로 남아 이후 탐색됨 |
| 2 | **좁은 복도** | 경로 주변 `obstacle_distance` 작음, 인플레이션 후 경로 없음 | 인플레이션만 문제인지 판단 (raw grid로는 경로 있음) | 로봇 셀·breadcrumb 강제 통과(SemExp 트릭), RPP 근접 감속으로 저속 통과 | 안전 모니터 STOP 영역은 유지 — 몸체 반경 + 2 cm보다 좁으면 통과 불가로 판정 |
| 3 | **frontier가 벽 반대편** | Euclidean은 가깝지만 거리장 거리 큼 | utility가 경로 거리로 계산되므로 자동으로 낮은 점수 | 다른 frontier 선택 | 테스트로 고정 (02 §6) |
| 4 | **사람이 길을 막음** | 안전 모니터 STOP/SLOWDOWN, 경로 앞 셀 점유(새로 나타남) | 새 장애물 → WAIT | 정지 3~5 s, 1 Hz로 경로 유효성 확인 | 계속 막힘 → 재계획(우회) → 없으면 recovery ladder R1~R4 |
| 5 | **사람이 잠깐 막았다 사라짐** | 스캔에서 장애물 소멸 | 계속 진행 | log-odds miss로 ~2 s 내 지도 복원 | ghost 잔류 시 #14 |
| 6 | **벽 코너에 끼임** | ProgressMonitor STUCK (명령 v>0인데 변위≈0), 가속도 spike | recovery ladder | R2 SPIN → R3 BACKUP(후방 검사) | R4: 앞 셀 collision_map 기록 + goal blacklist |
| 7 | **Odometry drift 증가** | gyro-odom yaw 불일치, (P2) CSM 점수 하락, 벽이 두 겹으로 그려짐 | 신뢰도 저하 → 속도 제한, 맵 갱신 일시 중지 | 제자리 회전으로 재관측, (P2) CSM 보정 | 당일 오도메트리 보정 절차로 예방 (03 B.4) |
| 8 | **target 발견 후 가려짐** | TargetTracker 미검출 연속 | memory 2~3 s 유지 | 마지막 bearing 방향 회전 → 실패 시 저장된 world 위치로 계획 접근 | 도착 후 look-around, 없으면 tentative 폐기 / confirmed는 TTL blacklist |
| 9 | **target 근처에 장애물** | standoff 후보 셀이 inflated OCCUPIED | relax_goal로 가시선 있는 다른 standoff | A* 접근 후 visual servo, 안전 모니터 우선 | 근접 불가하면 규정상 최소 거리 안인지 확인 후 기록 |
| 10 | **target 여러 개** | 확인된 target 목록 | dedup(0.25~0.3 m) 후 미방문 중 거리장 최근접 | 순차 접근, 방문 target 재검출 무시 | 모든 target 방문 or 총 개수 도달 → RETURN_HOME |
| 11 | **탐색 시간이 거의 끝남** | `time_left < SF·ETA_home + margin` (1 Hz 감시) | 현재 작업 중단 (target 접근 중이어도, 접근 비용이 예산 초과면) | RETURN_HOME | 복귀 중 target 발견 시 예산 여유 있을 때만 접근 |
| 12 | **home 경로가 막힘** | known-only A* 실패 | 08 §2.3 폴백 순서 | dynamic 셀 리셋 → 인플레이션 축소 → breadcrumb → unknown 허용 | 모두 실패 → ladder, 1 Hz 재시도, 안전 정지 |
| 13 | **frontier 전부 실패** | 후보 없음 / 전부 blacklist | second chance 1회 → view frontier 단계 | blacklist 초기화 후 재시도, 그다음 카메라 미탐색 영역 | 그래도 없음 → RETURN_HOME (정상 종료) |
| 14 | **dynamic obstacle ghost 잔류** | 경로 불가인데 해당 셀이 최근 생성 + 로봇 시야에서 다시 관측 안 됨 | ghost 의심 | ghost 의심 셀 prior 리셋 (R1 CLEAR) → 재계획 | 실제 장애물이면 다음 스캔에 다시 hit → 정상 반영 (3 hit ≈ 0.6 s) |
| (추가) 15 | **계산 지연 (비동기 컨트롤러)** | 스텝 처리시간 > 예산 로그 | 무거운 작업 분산 | 전역 계획 중 감속, 안전 모니터는 매 스텝 | 스텝 시간 max를 CI/테스트 로그로 추적 |
| (추가) 16 | **LiDAR 평면 아래 장애물** | 전진 명령 중 변위≈0 + 가속도 spike, 스캔엔 아무것도 없음 | 보이지 않는 장애물 | 후진 → collision_map 기록 | SemExp collision map ✅ |
| (추가) 17 | **오탐 target** | M-of-N 미달, 위치 분산 큼, 접근 시 사라짐 | tentative 유지 | 접근 전 확인 강화 | 도착 후 미검출이면 폐기 |
