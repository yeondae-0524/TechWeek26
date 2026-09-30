# 07. Target Search · Confirmation · Tracking · Approach

> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 이번 해커톤은 "Object Detection 자체는 복잡하지 않다"고 안내됨 → detector보다 **검색 전략·확인·위치추정·중복처리·접근**에 집중.

---

## 1. ObjectNav 연구에서 가져올 것 / 버릴 것

| 연구 | 핵심 | 가져올 것 (학습 없이) | 버릴 것 |
|---|---|---|---|
| **SemExp** (Chaplot 2020, [arXiv:2007.00643](https://arxiv.org/abs/2007.00643)) ✅ [`agents/sem_exp.py`](https://github.com/devendrachaplot/Object-Goal-Navigation/blob/master/agents/sem_exp.py) | semantic map + 학습 global policy(장기 goal) + **결정적 local policy(FMM 계획)** | (1) 목표 발견 시 goal을 **목표 셀 팽창 영역**으로 두고 계획 (2) collision map (3) 방문 셀 통과 가능 | global policy 학습, semantic segmentation 네트워크 |
| **PONI** (Ramakrishnan 2022, [arXiv:2201.10029](https://arxiv.org/abs/2201.10029)) ✅ | "Where to look?" = **area potential + object potential**, 둘을 가중합 (`area_weight_coef 0.5`, `dist_weight_coef 0.3` 인자 확인) | "탐색 이득(area) + 목표 존재 가능성(object)" 분리 개념 → 우리는 object potential 대신 **카메라 미탐색 영역**(02 §3.4) | potential 예측 네트워크 |
| **VLFM** (Yokoyama 2024, [arXiv:2312.03275](https://arxiv.org/abs/2312.03275)) ✅ [`itm_policy.py`](https://github.com/bdaiinstitute/vlfm/blob/main/vlfm/policy/itm_policy.py) | frontier를 VLM value로 점수 | **sticky frontier + acyclic enforcer** (02 §3.2) | VLM, GPU |
| **Gervet et al., Science Robotics 2023** [doi:10.1126/scirobotics.adf6991](https://doi.org/10.1126/scirobotics.adf6991) 📄 | 실제 집 6곳 비교: modular 시뮬 81% → 실제 **90%**, end-to-end 77% → **23%** | **모듈형(지도+계획+단순 인식) 구조가 가장 견고**하다는 근거 → 우리 구조 정당화 | – |

## 2. 우리 카메라의 현실 (practice e-puck, ✅ [E-puck.proto R2025a](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto))

- `camera_fieldOfView 0.84 rad (48°)`, `52 × 39 px`, noise 0.
- 초점거리(px): `f = (W/2)/tan(FOV/2) = 26 / tan(0.42) ≈ 58.2 px` → 1 px ≈ 0.016 rad(0.9°).
- 목표가 폭 w_obj m이면 거리 d에서 픽셀 폭 ≈ `f·w_obj/d`. 예: 0.1 m 물체 → d=1 m에서 5.8 px, 2 m에서 2.9 px ⇒ **확실한 검출 거리는 ~1 m 안팎**(목표 크기에 따라 당일 재계산).
- Webots `Recognition` 노드(카메라 객체 인식)는 **ground truth**를 준다 ✅ [recognition.md](https://cyberbotics.com/doc/reference/recognition). GPS처럼 **규정상 금지일 가능성이 높다** → 사용 전 확인, 기본은 미사용.

## 3. 검색 전략: "어디를 봐야 하나?"

1. **Initial Active Scan** (INITIALIZE 직후): LiDAR는 이미 360°라 지도 이득은 거의 없고, **카메라 48°로 360° 훑기**가 목적. 7~8 방향 × 정지 0.3~0.5 s ≈ 5~8 s. 정지 1 s 동안 **gyro bias 추정**도 같이 (03).
2. EXPLORE 1단계: LiDAR frontier + camera IG(G3) (02 §5).
3. EXPLORE 2단계: frontier 소진 후 **view frontier** (카메라 미탐색 표면을 볼 수 있는 FREE 셀) — hector inner exploration ✅의 S&R 버전.
4. 이동 중 카메라는 **진행 방향만** 본다 → 목표 frontier 도착 시 **짧은 "look-around" (±60° 스윕)** 를 넣으면 coverage가 크게 오른다 (TARE/FUEL이 viewpoint에 yaw를 포함시키는 이유 ✅ `findViewpoints`).
5. `camera_seen` grid: 매 detection 주기에 카메라 frustum 안 ray(예: 9~15개)로 LiDAR grid를 ray-cast, 거리 ≤ `CAM_SEARCH_RANGE`(검출 가능 거리)까지 OCCUPIED/FREE 셀을 seen 처리.

## 4. Target 확인 (multi-frame confirmation)

- 원리: 레이더/추적 분야의 **M-of-N track confirmation** (Blackman & Popoli, *Design and Analysis of Modern Tracking Systems*, 1999 📄).
- 우리: 최근 N=5 검출 주기 중 M=3회 이상 검출 **그리고** 각 검출로 추정한 world 위치의 분산 < 0.1 m → CONFIRMED.
- 면적 하한(`min_area` px), 종횡비, (가능하면) 바닥/벽 위치 일관성으로 오탐 억제.
- 삭제: 연속 K회 미검출 + 로봇이 그 위치를 **볼 수 있는 자세**인데도 안 보임 → tentative 삭제 (단순 "안 보임"은 가림일 수 있음).

## 5. Target world 위치 추정 (bearing + LiDAR range)

```text
bearing  β = atan2((W/2 - cx), f)            # + = 왼쪽(CCW), 로봇 기준 (카메라가 전방이라 가정; 마운트 오프셋은 config)
range    r = median(LiDAR ranges within β ± 2°)   # 목표가 LiDAR 평면 높이에 있을 때
fallback r ≈ f · W_obj / w_px                # 목표 실제 크기를 당일 알면 (크기 기반)
world    (x_t, y_t) = pose ⊕ (r cosβ, r sinβ)
```
- 두 추정(LiDAR vs 크기)이 크게 다르면 LiDAR가 다른 물체(앞의 장애물)를 본 것 → 크기 기반 사용 또는 보류.
- 목표가 벽에 붙은 표식이면 LiDAR는 벽 거리를 준다 → 그대로 유효.

## 6. 중복 처리 (multiple targets, Case 10)
- `targets = [{id, xy, n_obs, confirmed, visited, last_seen}]` — 새 확인 위치가 기존 target과 `DEDUP_RADIUS`(목표 크기 + 위치오차, 시작 0.25~0.3 m) 이내면 병합(가중 평균).
- 방문 완료된 target은 detection 결과에서 **무시**(재접근 루프 방지).
- 여러 미방문 target: 거리장(04)으로 가장 가까운 것부터 (TSP 불필요 — 개수 적음).
- 종료: `config.REQUIRED_TARGETS` (개수 미공개 → 파라미터화). `N`이면 N개 방문 즉시 RETURN_HOME, `None`이면 탐색·view frontier 소진 또는 시간 예산 트리거까지 계속.

## 7. 일시적 가림 / 소실 대응 (Case 8)
1. 마지막 bearing과 world 위치를 `TARGET_MEMORY_S`(2~3 s) 유지.
2. 소실 시: 제자리에서 마지막 bearing 쪽으로 회전(±30°) → 재획득 시 계속.
3. 여전히 없음: **저장된 world 위치로 계획 기반 접근**(시야와 무관하게 이동) → 근처에서 look-around.
4. 그 위치에 도착했는데 안 보임: tentative면 폐기, confirmed면 "주변 반경 0.3 m 탐색" 후 blacklist(TTL).

## 8. 접근 (Q13) — 2단계가 가장 단순하고 안정적

| 단계 | 방법 | 근거 |
|---|---|---|
| A. 원거리 | world 위치 → `relax_goal`로 **standoff 셀**(목표에서 d_stand, 가시선 확보, inflated FREE) 선택 → A* + RPP-lite | SemExp: 목표 영역 팽창 goal ✅; FAR goal 재평가 ✅ |
| B. 근거리 (목표가 시야에 있고 ~0.5 m 이내) | **이미지 기반 visual servo**: `ω = −k_ω·(cx − W/2)/(W/2)`, `v = v_max·clip((d − d_stop)/d_slow, 0, 1)`, d는 bearing 방향 LiDAR 거리 | Chaumette & Hutchinson 2006 [doi:10.1109/MRA.2006.250573](https://doi.org/10.1109/MRA.2006.250573) 📄 (IBVS의 가장 단순한 형태) |
| C. 도착 | LiDAR d ≤ d_stop 또는 픽셀 면적 ≥ A_stop → 정지, **1~2 s 유지**, target visited | Erebus: supervisor가 **로봇이 1 s 이상 정지해야** victim 식별 처리 ✅ (`time_stopped() >= 1.0`) — 당일 규정의 "도착" 정의 확인 필요 ⚠️ |

- 안전 모니터는 B/C 단계에서도 항상 작동. 목표 근처 장애물(Case 9)은 A 단계의 standoff 선택에서 inflated FREE + 가시선 조건으로 해결.
- `d_stop`은 규정의 "위치까지 이동" 기준(예: 몇 cm 이내)에 맞춰 config에서 설정.

## 9. Exploration goal vs Target goal 충돌 (02/10 공통)
- target CONFIRMED → 즉시 APPROACH_TARGET 선점 (탐색 goal 폐기, blacklist 아님).
- 단, **남은 시간 < 접근 + 복귀 ETA**면 접근하지 않고 위치만 기록 후 RETURN_HOME (08).
- 접근 완료 → EXPLORE 재개 시 progress grace + frontier 재선택 (hysteresis 초기화).

## 10. 인터페이스 주의
- 고정 규격 `target = {"found", "cx", "direction", "area"}`는 **바꾸지 않는다**. 확인/추적/world 위치는 `detection.py` 내부의 `TargetTracker` 클래스가 보유하고 main이 조회.
- target world 위치를 다른 모듈과 공유 포맷으로 만들려면 AGENTS.md 규칙대로 **INTERFACES.md·interfaces.py·tests 동시 수정 + 사람 확인** 필요.

## 11. 테스트 (Webots 없이)
- bearing 공식: cx=W/2 → 0, cx=0 → +FOV/2 근처.
- range: 합성 스캔에서 β±2° 중앙값.
- world 위치: 알려진 pose/bearing/range → 좌표 일치.
- M-of-N: 3/5 검출 → confirmed, 2/5 → tentative, 위치 분산 크면 미확인.
- dedup: 0.2 m 떨어진 두 검출 → 1개, 0.6 m → 2개.
- visited target 재검출 무시.
- 소실 시퀀스: memory 내 재획득, 초과 시 world 위치 접근 모드.
