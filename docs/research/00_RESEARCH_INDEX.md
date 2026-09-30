# 00. Deep Reference Research — Index

**목적**: 검증된 autonomous exploration / navigation / S&R 시스템에서 **우리 Webots R2025a + Python 3.10 Search & Rescue 로봇**에 가져올 설계와 알고리즘을 추출한다. 설치·이식이 아니라 "무엇을, 왜, 어느 파일에, 어떤 테스트로".

**작성**: 2026-09-29 ~ 2026-09-30. 프로젝트 코드(controllers/, worlds/, tests/, scripts/)와 기존 문서는 **수정하지 않았다**. 레퍼런스 repo는 세션 scratchpad에 읽기 전용으로 shallow clone했고, 설치한 dependency는 없다.

---

## 1. 읽는 순서

| 목적 | 문서 |
|---|---|
| 결론만 | **[10_FINAL_ARCHITECTURE.md](10_FINAL_ARCHITECTURE.md)** (FINAL RECOMMENDATION, Q1–Q15) |
| 바로 구현 시작 | **[11_IMPLEMENTATION_ROADMAP.md](11_IMPLEMENTATION_ROADMAP.md)** (P0–P3, S0–S11, 수용 기준, prompt 초안) |
| 어떤 자료를 읽을까 | [01_REFERENCE_MATRIX.md](01_REFERENCE_MATRIX.md) (Matrix, TOP10, 카드, "먼저 볼 파일") |
| 분야별 근거 | [02 탐색](02_EXPLORATION.md) · [03 지도/위치](03_MAPPING_LOCALIZATION.md) · [04 전역계획](04_GLOBAL_PLANNING.md) · [05 지역계획/안전/동적장애물](05_LOCAL_PLANNING_SAFETY.md) · [06 복구](06_RECOVERY.md) · [07 target 수색](07_TARGET_SEARCH.md) · [08 복귀](08_RETURN_HOME.md) · [09 Webots](09_WEBOTS_REFERENCES.md) |
| 실패 대응 | [12_FAILURE_SCENARIOS.md](12_FAILURE_SCENARIOS.md) (14 + 3 cases) |
| GPU | [13_GPU_DECISION.md](13_GPU_DECISION.md) (NOT RECOMMENDED) |

## 2. 검증 표기
- ✅ 소스 코드 직접 확인 (repo·commit은 01 참고) · 📄 논문/공식 문서만 · ⚠️ 미확인/추정/당일 확인 필요
- DOI는 doi.org handle + Crossref 제목 대조로 확인. 초기에 틀렸던 4개(González-Baños, Wirth & Pellenz, Gyrodometry, FUEL)는 교정 완료.
- "initial tuning suggestion" = 레퍼런스 값을 우리 로봇 규모로 옮긴 시작값 (검증값 아님).
- 성능 수치는 이 개발 PC의 순수 Python 3.10 실측 (04 §1).

## 3. 수행한 PASS

| PASS | 내용 | 상태 |
|---|---|---|
| 1 | 기존 후보 10개 + m-explore 재조사 (소스) | 완료 |
| 2 | 분야별 gap 확인 (mapping 동적, 안전, 복구, 귀환, Webots 대회) | 완료 |
| 3 | 추가 구현 조사: hector exploration, rrt_exploration, frontier_exploration, GBPlanner, FUEL, VLFM, OctoMap, STVL, obstacle_detector, move_base, PythonRobotics, Erebus, webots_ros2 e-puck | 완료 |
| 4 | 후속/평가 연구: Holz 2010, Juliá 2012, Basilico 2011, Kulich 2019, Gervet 2023, Macenski 2023, Placed 2023, SubT 논문 | 완료 (Basilico 세부 기준은 ⚠️) |
| 5 | 우리 baseline과 1:1 비교 + 실측 벤치마크 | 완료 |
| 6 | 과한 알고리즘 제거 (DO NOT 목록) | 완료 (10 §6) |
| 7 | 최종 architecture | 완료 (10) |
| 8 | 실패 시나리오 검증 | 완료 (12) |
| 9 | 우선순위 재평가 | 완료 (11 §1) |
| 10 | 최종 정리 + 00/10/11 교차 검토 | 완료 (§6) |

## 4. 핵심 발견 15

1. m-explore의 점수는 **Euclidean 거리 − 경계 셀 수**이고, `orientation_scale`은 **코드에서 쓰이지 않는다**; 거리엔 해상도가 이중으로 곱해져 gain이 압도한다. ✅
2. hector의 **Exploration Transform**(frontier 전체에서 경로비용+벽근접 페널티 wavefront)이 RoboCup Rescue용 고전 해법이고, frontier 소진 시 **inner exploration**으로 "안 가본 곳"을 찾는다 → 우리 **view frontier**의 근거. ✅
3. 로봇에서 **BFS 거리장 1회(12 ms)** 가 frontier마다 A*(49 ms씩)보다 훨씬 싸고 "벽 반대편" 문제를 해결한다. (실측)
4. TSP 계층 탐색(TARE/FUEL/GTSP)의 이득은 2D 소규모에서 **≤12.5%, 사무실형에선 −4.5%**. 📄 Kulich 2019 → 버린다.
5. Hysteresis는 표준: rrt_exploration(×2.0, 3 m), VLFM(sticky + acyclic), FAR(momentum 3), TARE(비대칭 임계). m-explore엔 없다. ✅
6. GBPlanner의 귀환 식: `path_len/v_homing > (min(시간예산, 배터리) − 20 s)` → 우리 시간 예산 복귀. ✅
7. m-explore-ros2 `return_to_init`은 **실패 처리가 없다**. ✅
8. Nav2 progress checker는 목표거리 아닌 **변위(0.5 m/10 s)** 기반, TB3는 0.1 m로 축소. ✅
9. Nav2 기본 recovery: **costmap clear → Spin 1.57 → Wait 5 s → BackUp 0.30 m**, 재시도 6회, 모두 충돌 검사 후 실행. ✅
10. Nav2 Collision Monitor는 **costmap을 우회해 raw scan에 작동**(STOP/SLOWDOWN/APPROACH, min_points 4). ✅
11. OctoMap log-odds 기본값(hit 0.85, miss −0.4, clamp [−2, 3.5]) → 사람 ghost가 **~9 miss(≈1.8 s)**면 사라진다. 현재 baseline은 영구. ✅
12. Webots 문서: 대회는 **비동기 컨트롤러**를 쓴다 → 계산 시간 = 제어 지연. ✅
13. practice e-puck 카메라는 **48°, 52×39 px** → LiDAR 탐색이 끝나도 카메라 사각이 남는다; target 확실 검출 거리 ~1 m 수준. ✅
14. Webots 대회 사례(Erebus): **20 s 정지 → 자동 LoP**, victim 식별엔 **1 s 정지** 필요. 멈춰서 기다리는 recovery에 상한이 필요. ✅
15. 실제 환경 ObjectNav에서 **모듈형 90% vs end-to-end 23%** → GPU·학습 모델은 불필요. 📄 Gervet 2023

## 5. 당일 반드시 확인할 것 (⚠️)
공식 로봇 장치명·기하·속도 / LiDAR 순서·FOV·maxRange·minRange·noise·mount / 카메라 FOV·해상도·mount / synchronization 값 / 시간 제한과 기준(시뮬 vs 실시간) / target 특징·크기·개수·"도착" 정의·정지 시간 / GPS·Compass·Recognition·Supervisor 허용 여부 / 정지 시간 페널티 유무 / 시작 pose.

## 6. 교차 검토 결과 (00 ↔ 10 ↔ 11)

| 항목 | 00 | 10 | 11 | 일치 |
|---|---|---|---|---|
| 구현 순서 | S0–S11 참조 | §5 S0–S11 | §2 S0–S11 | ✅ |
| 첫 5기능 (Q1) | – | §11 Q1 | F2/F3, F5, F8(+F6), F9, F7 = 모두 P0 | ✅ (Q1은 중요도, 구현 순서는 의존성 기준으로 S 순서) |
| DWA | 버림 | P3 이하 | F21 arc sampler P3, DWA 미포함 | ✅ |
| Scan matching | 조건부 | P2 조건부 CSM | F19 P2 | ✅ |
| GPU | NOT RECOMMENDED | 13 참조 | 미포함 | ✅ |
| 새 dependency | 없음 | NumPy만, OpenCV는 사람 확인 | §5 | ✅ |
| 신규 파일 | – | navigation.py (제안, 대안: control.py) | S5 | ✅ |
| 주기 | – | §2.2 | S0 스케줄러, F15 | ✅ |
| ROS 전용 가정 | 없음 | ROS 요소는 전부 개념 치환 | prompt에 ROS 없음 | ✅ |
| 인터페이스 | 불변 | 불변 (target world 위치 공유 시 사람 확인) | prompt에 명시 | ✅ |

### 6.1 팀 리뷰 반영 (2026-09-30)
- 10 §2 아키텍처 그림: Localization·Mapping·Perception을 **병렬**로 수정 (Detection이 Mapping에 종속되지 않음).
- `REQUIRED_TARGETS` 파라미터 추가 (10 §7, 07 §6) — target 개수 미확정.
- **M0 Walking Skeleton** 단계 추가 (11 §3.1, 10 §8). 부분 수용: 영구 ghost 금지(1줄 clearing), 거리장 기반 frontier, MVP의 시간 예산 복귀는 유지.

## 7. 한계
- Webots에서 새 알고리즘을 실제로 돌려본 결과는 없다 (코드 수정 금지). 성능 수치는 순수 Python 오프라인 벤치.
- Basilico & Amigoni 2011의 구체적 기준·가중은 원문 미열람(초록만).
- Nav2 문서 일부 URL은 버전별로 이동하므로 링크가 깨지면 docs.nav2.org에서 제목으로 검색.
- 대회 규정 미공개 항목은 모두 config와 당일 확인 목록으로 분리했다.
