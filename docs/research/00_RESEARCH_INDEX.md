# 00. Deep Reference Research — Index

**목적**: 기존 Deep Reference Research를 공식 PNU TECH WEEK **TurtleBot3Burger / Webots R2025a** 환경으로 재검증한다. 알고리즘 근거는 보존하고 연습용 robot-specific 가정·센서 의존성·튜닝을 교정한다. **문서 검토만 완료하며 S0–S11 구현은 팀 confirmation 이후 별도 작업이다.**

**작성**: 2026-09-29 ~ 2026-09-30. 프로젝트 코드(controllers/, worlds/, tests/, scripts/)와 기존 문서는 **수정하지 않았다**. 레퍼런스 repo는 세션 scratchpad에 읽기 전용으로 shallow clone했고, 설치한 dependency는 없다.

---

## 1. 읽는 순서

| 목적 | 문서 |
|---|---|
| 결론만 | **[10_FINAL_ARCHITECTURE.md](10_FINAL_ARCHITECTURE.md)** (FINAL RECOMMENDATION, 재검증 Q1–Q10) |
| 팀 공유 후 구현 계획 검토 | **[11_IMPLEMENTATION_ROADMAP.md](11_IMPLEMENTATION_ROADMAP.md)** (P0–P3, S0–S11, 수용 기준, prompt 초안) |
| 어떤 자료를 읽을까 | [01_REFERENCE_MATRIX.md](01_REFERENCE_MATRIX.md) (Matrix, TOP10, 카드, "먼저 볼 파일") |
| 분야별 근거 | [02 탐색](02_EXPLORATION.md) · [03 지도/위치](03_MAPPING_LOCALIZATION.md) · [04 전역계획](04_GLOBAL_PLANNING.md) · [05 지역계획/안전/동적장애물](05_LOCAL_PLANNING_SAFETY.md) · [06 복구](06_RECOVERY.md) · [07 target 수색](07_TARGET_SEARCH.md) · [08 복귀](08_RETURN_HOME.md) · [09 Webots](09_WEBOTS_REFERENCES.md) |
| 실패 대응 | [12_FAILURE_SCENARIOS.md](12_FAILURE_SCENARIOS.md) (기존 17 + 공식 환경 전환 12 cases) |
| GPU | [13_GPU_DECISION.md](13_GPU_DECISION.md) (NOT RECOMMENDED) |

## 2. 검증 표기
- ✅ 소스 코드 직접 확인 (repo·commit은 01 참고) · 📄 논문/공식 문서만 · ⚠️ 미확인/추정/당일 확인 필요
- DOI는 doi.org handle + Crossref 제목 대조로 확인. 초기에 틀렸던 4개(González-Baños, Wirth & Pellenz, Gyrodometry, FUEL)는 교정 완료.
- "initial tuning suggestion" = 레퍼런스 값을 우리 로봇 규모로 옮긴 시작값 (검증값 아님).
- [OFFICIAL] 공식 repo·notebook·R2025a PROTO, [MEASURED] PC 실측, [REFERENCE] 외부 코드·논문, [INITIAL TUNING] 미검증 시작값, [DAY-OF] 최종 환경/규정 확인. 보조 태그 [DERIVED]는 계산값이다.
- **[ORGANIZER] = organizer-confirmed**, 사용자가 전달한 운영진 직접 확인. Encoder·2D LiDAR 필수, IMU 선택, Compass/GPS 미사용. GPU 선택, 쉬운 CV 또는 필요 시 DL, 사전 지도·target 위치 없음, target 시각 정보 당일 제공, target 방문 후 복귀, 정적/동적 충돌 방지 및 안전거리. 평가: mission achievement / technical implementation / driving stability / creativity.
- 성능은 old practice-PC 또는 이전 pass PC 기록(04 §1, 09 §8); 공식 robot/world runtime 성능이 아니다. 이번 재개 pass에서는 벤치 재실행 없이 원 스크립트·결과를 대조했다.

### 공식 repo audit

- [repository](https://github.com/kyu-rae-kim/PNU-TECHWEEK-260930), `main`, **383de18193b2644a6b662e87c4736f1c314a8e73**, 확인 **2026-09-30**. `git ls-remote origin refs/heads/main`과 local HEAD 일치.
- 프로젝트 밖 기존 local copy `D:/Dev/projects/PNU-TECHWEEK-260930`를 read-only로 열람. 기존 notebook 로컬 수정(세미콜론 1개 삭제)은 보존하고 공식 사실은 **git HEAD blob**과 대조했다.
- 우선순위: **운영진 직접 확인 > 공식 repo/notebook/world/controller > 공식 계획안 > Webots R2025a docs/PROTO > 외부 reference > initial tuning**.
- 공식 예제 사용과 대회 허용은 다르다. Supervisor pose는 demo/test 참고로 분리하며 대회 controller에 넣지 않는다. 자세한 source locator는 [09](09_WEBOTS_REFERENCES.md).

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
| 10 | 기존 연구 최종 정리 | 이전 pass 완료 |
| 11 | 공식 repo 재검증 및 00–13 일치 검토 | 2026-09-30 완료, 구현 미착수 |

## 4. 핵심 발견 15

1. m-explore의 점수는 **Euclidean 거리 − 경계 셀 수**이고, `orientation_scale`은 **코드에서 쓰이지 않는다**; 거리엔 해상도가 이중으로 곱해져 gain이 압도한다. ✅
2. hector의 **Exploration Transform**(frontier 전체에서 경로비용+벽근접 페널티 wavefront)이 RoboCup Rescue용 고전 해법이고, frontier 소진 시 **inner exploration**으로 "안 가본 곳"을 찾는다 → 우리 **view frontier**의 근거. ✅
3. 로봇에서 **BFS 거리장 1회(12 ms)** 가 frontier마다 A*(49 ms씩)보다 훨씬 싸고 "벽 반대편" 문제를 해결한다. ([MEASURED] old practice-PC 160×160)
4. TSP 계층 탐색(TARE/FUEL/GTSP)의 이득은 2D 소규모에서 **≤12.5%, 사무실형에선 −4.5%**. 📄 Kulich 2019 → 버린다.
5. Hysteresis는 표준: rrt_exploration(×2.0, 3 m), VLFM(sticky + acyclic), FAR(momentum 3), TARE(비대칭 임계). m-explore엔 없다. ✅
6. GBPlanner의 귀환 식: `path_len/v_homing > (min(시간예산, 배터리) − 20 s)` → 우리 시간 예산 복귀. ✅
7. m-explore-ros2 `return_to_init`은 **실패 처리가 없다**. ✅
8. Nav2 progress checker는 목표거리 아닌 **변위(0.5 m/10 s)** 기반, TB3는 0.1 m로 축소. ✅
9. Nav2 기본 recovery: **costmap clear → Spin 1.57 → Wait 5 s → BackUp 0.30 m**, 재시도 6회, 모두 충돌 검사 후 실행. ✅
10. Nav2 Collision Monitor는 **costmap을 우회해 raw scan에 작동**(STOP/SLOWDOWN/APPROACH, min_points 4). ✅
11. OctoMap hit/miss/clamp는 [REFERENCE]. 우리 hysteresis −0.2에서는 **10 miss** 필요 [DERIVED]; 128 ms scan 시작안이면 약 1.28 s의 유효 free 관측 [INITIAL TUNING]. 가려진 ghost는 시간만으로 지워지지 않는다. 현재 baseline은 영구. ✅
12. Webots 문서의 비동기 대회 설명은 일반론. **공식 robot 월드 6개는 sync TRUE**이며 대회 최종 설정은 [DAY-OF]. 계산 초과는 동기에서는 wall-clock 손실, 비동기에서는 추가 운동 지연 위험. ✅
13. 공식 camera **640×480, 약 60°** [OFFICIAL]도 360° LiDAR와 coverage가 다르다. view frontier·Initial Active Scan 유지; 검출 가능 거리는 당일 target으로 측정. ✅
14. Erebus **20 s LoP / 1 s victim stop**은 [REFERENCE, NOT TECH WEEK RULE]. WAIT timeout 원칙만 차용. ✅
15. 실제 환경 ObjectNav에서 **모듈형 90% vs end-to-end 23%** → 모듈형 설계의 참고 근거. 이 비교만으로 GPU 필요 여부를 결정하지 않는다. 📄 Gervet 2023

## 5. 남은 Day-of check

최종 world/timestep/synchronization 및 센서 mount가 예제와 같은지, target 외형·크기·개수·도착 조건·정지 시간, 전체 미션 시간과 sim/real 기준, 시작 pose 형식, 아레나 크기, 최종 OS/Python/library 환경, 제3자 dependency 제한, 분리된 Supervisor 개발 평가 허용 범위, 정지 페널티. 이미 확인된 필수/선택 센서는 다시 질문하지 않는다.

## 6. 최종 결정 요약 (00 ↔ 10 ↔ 11)

| 항목 | 일치한 추천 |
|---|---|
| 센서/위치 | Encoder 필수 base + gyro ENABLED 권장(팀 선택·encoder-only 폴백) + 측정상 필요 시 LiDAR scan matching; GPS/Compass/Supervisor pose 미사용 |
| 지도/탐색 | log-odds occupancy grid, distance field·frontier·IG·hysteresis·blacklist, camera coverage/view frontier |
| 경로/안전 | A*, 8-connected option, RPP-lite, raw LiDAR SafetyMonitor, ProgressMonitor·유한 recovery |
| 비전 | OpenCV classical CV + M-of-N·world memory·dedup, NumPy-only 폴백; YOLO/GPU는 필요 입증 후 |
| 주기 | 64 ms 기준 odometry/control/safety 매 step; mapping/detection 각 약 128 ms 시작; coverage 약 0.256–0.512 s; 계획 이벤트 + 약 1 s 상태 검사. 모두 10 §2.2 tuning, runtime 검증 전 |
| geometry | notebook radius 0.105 m, PROTO 외접 약 0.110 m; 안전 원형 footprint 0.111 m + margin 0.05 m = 0.161 m(약 0.16 m) 시작 |
| 복귀 | local home=(0,0,0), localization+map; ETA safety factor 1.4, margin 15 s [INITIAL TUNING] |
| 단계 | S0 audit/이행/계측 → S1 encoder·optional gyro → S2 follower/safety → S3 planning → S4 mapping → S5 navigator → S6 home → S7 exploration → S8 target → S9 FSM → S10 안정화/coverage → S11 측정 기반 확장 |
| 현재 범위 | 연구 14개 문서만; 설치·구현·world 변경·commit/push 없음 |

### 6.1 팀 리뷰 반영 (2026-09-30)
- 10 §2 아키텍처 그림: Localization·Mapping·Perception을 **병렬**로 수정 (Detection이 Mapping에 종속되지 않음).
- `REQUIRED_TARGETS` 파라미터 추가 (10 §7, 07 §6) — target 개수 미확정.
- **M0 Walking Skeleton** 단계 추가 (11 §3.1, 10 §8). 부분 수용: 영구 ghost 금지(1줄 clearing), 거리장 기반 frontier, MVP의 시간 예산 복귀는 유지.

## 7. 한계
- Webots에서 새 알고리즘을 실제로 돌려본 결과는 없다 (코드 수정 금지). 성능 수치는 순수 Python 오프라인 벤치.
- Basilico & Amigoni 2011의 구체적 기준·가중은 원문 미열람(초록만).
- Nav2 문서 일부 URL은 버전별로 이동하므로 링크가 깨지면 docs.nav2.org에서 제목으로 검색.
- 대회 규정 미공개 항목은 모두 config와 당일 확인 목록으로 분리했다.


## 8. Stale assumption audit 완료 (2026-09-30)

요청된 문자열을 research 00–13 전체에서 검색하고 문맥별로 판정했다. **현재 competition 추천값으로 남은 e-puck geometry / old camera / 16 ms 가정은 없다.** 실제 코드의 practice 값은 이번 범위 밖이므로 09 §11의 이행 목록에 기록만 했다.

| 검색군 | 남긴 문맥과 위치 | 판정 |
|---|---|---|
| e-puck / epuck | 00 조사 이력, 01/02/03/04/06/07/09 reference·practice, 05 폐기된 비교, 10/11 이행 위험·practice profile, 12 실패 시나리오 | historical/reference 또는 변경 대상 기록 |
| GPS / Compass | organizer-confirmed 미사용, 공식 sample이 읽는 사실, 옛 debug 기준 폐기, competition 입력 차단·실패 검출 | exclusion / demo-only, localization 입력 아님 |
| Supervisor | 공식 demo 및 외부 Erebus source, 격리된 개발 평가의 허용 범위 확인, competition pose 혼입 방지 | test/reference-only |
| 16 ms | 03/04 폐기한 scheduler 전제, 09 practice world, 10 현재 코드의 이행 위험 | historical only |
| 52×39 / 48° | 09 §12의 E-puck 사양 기록 한 곳 | practice only; 공식 camera는 640×480/약 60° |
| 0.02 / 0.037 | 09 옛 wheel/body 치수; 0.02는 별도로 camera/LDS mount 좌표 및 scan-matching 탐색 간격에도 쓰임 | 옛 치수·OFFICIAL/DERIVED 좌표·INITIAL TUNING을 문맥별 구분 |
| NumPy-only | 00/07/10/11/12의 detector 폴백 | OpenCV 권장 baseline과 모순 없음 |
| 360 / LiDAR / basicTimeStep / geometry / timing | 360 samples·회전각·R2025a fact, runtime UNCONFIRMED, PC benchmark, tuning을 분리 | OFFICIAL / DERIVED / MEASURED / INITIAL TUNING |

기존 외부 source URL은 삭제되지 않도록 원 git 문서와 대조하여 보존했다. 공식 source는 09 §0 및 01 §0의 commit/tag로 고정했다. Markdown 링크·code fence·whitespace를 검사했고, 구현 및 공식 reference 파일 변경은 별도 git diff와 hash 비교로 확인한다. 이번 pass에서 Webots·controller 테스트·benchmark 재실행·dependency 설치는 하지 않았다.
