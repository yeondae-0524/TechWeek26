# 07. Target Search · Confirmation · Tracking · Approach

> 수치 해석: 공식 사양은 [OFFICIAL]/[DERIVED], 외부 비교표·논문 수치는 [REFERENCE], PC timing은 [MEASURED] (04·09의 조건 한정). 별도 출처 없는 우리 거리·횟수·속도·주기·시간 목표는 모두 [INITIAL TUNING], 대회 최종 조건은 [DAY-OF CHECK]. [ORGANIZER]는 organizer-confirmed이다.


> ✅ 소스 확인 · 📄 논문만 · ⚠️ 추정. 이번 해커톤은 "Object Detection 자체는 복잡하지 않다"고 안내됨 → detector보다 **검색 전략·확인·위치추정·중복처리·접근**에 집중.

---

## 1. ObjectNav 연구에서 가져올 것 / 버릴 것

| 연구 | 핵심 | 가져올 것 (학습 없이) | 버릴 것 |
|---|---|---|---|
| **SemExp** (Chaplot 2020, [arXiv:2007.00643](https://arxiv.org/abs/2007.00643)) ✅ [`agents/sem_exp.py`](https://github.com/devendrachaplot/Object-Goal-Navigation/blob/master/agents/sem_exp.py) | semantic map + 학습 global policy(장기 goal) + **결정적 local policy(FMM 계획)** | (1) 목표 발견 시 goal을 **목표 셀 팽창 영역**으로 두고 계획 (2) collision map (3) 방문 셀 통과 가능 | global policy 학습, semantic segmentation 네트워크 |
| **PONI** (Ramakrishnan 2022, [arXiv:2201.10029](https://arxiv.org/abs/2201.10029)) ✅ | "Where to look?" = **area potential + object potential**, 둘을 가중합 (`area_weight_coef 0.5`, `dist_weight_coef 0.3` 인자 확인) | "탐색 이득(area) + 목표 존재 가능성(object)" 분리 개념 → 우리는 object potential 대신 **카메라 미탐색 영역**(02 §3.4) | potential 예측 네트워크 |
| **VLFM** (Yokoyama 2024, [arXiv:2312.03275](https://arxiv.org/abs/2312.03275)) ✅ [`itm_policy.py`](https://github.com/bdaiinstitute/vlfm/blob/main/vlfm/policy/itm_policy.py) | frontier를 VLM value로 점수 | **sticky frontier + acyclic enforcer** (02 §3.2) | VLM, GPU |
| **Gervet et al., Science Robotics 2023** [doi:10.1126/scirobotics.adf6991](https://doi.org/10.1126/scirobotics.adf6991) 📄 | 실제 집 6곳 비교: modular 시뮬 81% → 실제 **90%**, end-to-end 77% → **23%** | **모듈형(지도+계획+단순 인식) 구조가 가장 견고**하다는 근거 → 우리 구조 정당화 | – |

## 2. 공식 카메라와 detector 결정

[OFFICIAL] 로봇이 있는 공식 월드 6개 모두 Camera `640×480`, 수평 `fieldOfView=1.0472 rad`(약 60°), extensionSlot 안 translation `(0.05,0,-0.08)`, 별도 회전 없음. TB3 slot 위치를 더하면 robot-local `(0.02,0,0.073) m` [DERIVED]. 실제 지면 높이는 로봇 pose/기울기에 따라 달라진다. [09 §5](09_WEBOTS_REFERENCES.md)의 소스 표 참고.

- `f_x=(640/2)/tan(1.0472/2)≈554.3 px`, 중심 근처 1 px≈0.103° [DERIVED]. `c_x0=320`은 이상적 영상 중심 convention이며 실제 pixel-center 보정은 S0에서 검증한다.
- 폭 0.10 m 물체가 정면 3 m에 있으면 약 18.5 px [DERIVED, 예시 크기]. 이것은 **검출 성공 거리 측정이 아니다**. 유효 coverage 거리 3 m는 [INITIAL TUNING], target 외형·크기·조명은 [DAY-OF CHECK].
- 예전 [E-puck.proto R2025a](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto)는 practice/reference only이며 현재 카메라 튜닝에 사용하지 않는다.
- Webots [Recognition](https://cyberbotics.com/doc/reference/recognition)은 simulator 객체 정보를 제공하므로 우리 competition controller에서 미사용한다. 허용 규정을 추측하지 않는다. GPS/Compass도 미사용 [organizer-confirmed].

**추천 baseline: OpenCV classical CV + 다중 프레임 확인.** 공식 `tb3_segmentation`/`tb3_teleop_cam`은 cv2·NumPy로 BGRA→BGR, blur, LAB threshold, contour, centroid를 사용한다. notebook은 HSV 및 morphology도 교육한다. 우리 선택은 HSV/LAB → morphology → contour/connected component → cx·area → M-of-N이다. 공식 예제의 색 임계는 target 규격이 아니다.

공식 notebook 설치 예: `numpy==1.23.5`, `opencv-python==4.8.0.74`, `scikit-image==0.19.3` [OFFICIAL]. **Official teaching material explicitly uses OpenCV; final competition dependency rule confirmation pending.** NumPy-only는 최소 의존성 폴백이다. `tb3_cam`은 cv2를 import하지만 영상 처리 부분은 주석이고, `tb3_teleop_yolo`는 YOLO11n을 CPU로 호출한다. YOLO 예제 존재가 필요성을 뜻하지 않는다(13). 이번 pass에서는 설치하지 않는다.

## 3. 검색 전략: "어디를 봐야 하나?"

1. **Initial Active Scan**: 60° camera로 360°를 관측한다. [INITIAL TUNING] 45° 간격 8방향, 각 0.3–0.5 s 관측으로 중첩을 확보한다. 총 시간은 회전 시간 + 관측 대기 + 감속/정착 시간이며 5–8 s로 단정하지 않는다. Gyro를 선택했을 때만 회전 전 정지 bias를 추정한다. SPIN clearance가 검증되지 않으면 scan을 중단하고 안전 정지한다.
2. EXPLORE 1단계: LiDAR frontier + camera IG(G3) (02 §5).
3. EXPLORE 2단계: frontier 소진 후 **view frontier** (카메라 미탐색 FREE 바닥·표면을 볼 수 있는 도달 가능 FREE 셀) — hector inner exploration ✅의 S&R 버전.
4. 이동 중 카메라는 **진행 방향만** 본다 → 목표 frontier 도착 시 **짧은 "look-around" (±60° 스윕)** 를 넣으면 coverage가 크게 오른다 (TARE/FUEL이 viewpoint에 yaw를 포함시키는 이유 ✅ `findViewpoints`).
5. `camera_seen` grid: 약 0.256–0.512 s마다 [INITIAL TUNING], 실제 frame이 갱신됐을 때 카메라 frustum 안 ray(예: 9~15개)로 LiDAR grid를 ray-cast, 거리 ≤ `CAM_SEARCH_RANGE`(검출 가능 거리)까지 OCCUPIED/FREE 셀을 seen 처리.

## 4. Target 확인 (multi-frame confirmation)

- 원리: 레이더/추적 분야의 **M-of-N track confirmation** (Blackman & Popoli, *Design and Analysis of Modern Tracking Systems*, 1999 📄).
- 우리 [INITIAL TUNING]: 최근 N=5 검출 주기 중 M=3회 이상, map 위치 표준편차 <0.10 m(분산이라면 <0.01 m²) → CONFIRMED. 거리 미확정이면 bearing-only tentative로 두고 map 위치를 만들지 않는다.
- 면적 하한(`min_area` px), 종횡비, (가능하면) 바닥/벽 위치 일관성으로 오탐 억제.
- 삭제: 연속 K회 미검출 + 로봇이 그 위치를 **볼 수 있는 자세**인데도 안 보임 → tentative 삭제 (단순 "안 보임"은 가림일 수 있음).

## 5. Target map 위치 추정 (camera bearing + 조건부 range)

```text
bearing  β = atan2((W/2 - cx), f)            # + = 왼쪽(CCW), 로봇 기준 (카메라가 전방이라 가정; 마운트 오프셋은 config)
range    r = median(LiDAR ranges within β ± 2°)   # 목표가 LiDAR 평면 높이에 있을 때
fallback r ≈ f · W_obj / w_px                # 목표 실제 크기를 당일 알면 (크기 기반)
world    (x_t, y_t) = pose ⊕ (sensor_offset + sensor_rotation · sensor_point)
```
- 두 추정(LiDAR vs 크기)이 크게 다르면 LiDAR가 다른 물체(앞의 장애물)를 본 것 → 크기 기반 사용 또는 보류.
- 목표가 벽 표식이면 동일 벽·시선이 확인된 경우에만 LiDAR를 결합한다. Camera와 LiDAR 원점이 0.05 m 다르므로 특히 근거리에서 같은 bearing index만으로 대응시키지 않는다.
- 공식 바닥 공/사과는 LiDAR 평면 아래일 수 있다. 바닥 접점이 보이고 평면 가정이 맞을 때만 ground-plane 교차로 거리 추정한다. 수평 camera의 접점 행 `v_b>c_y`이면 전방 깊이 `X=f_y·h/(v_b-c_y)`, `Y=(c_x0-cx)·X/f_x`; 높이 h·기울기·mount 변환 반영. 지평선 근처/가림/바닥 불명은 기각한다. 알려진 폭 기반 `f·W_obj/w_px`도 전방 깊이이며 극좌표 거리와 구분한다. 근거가 없으면 거리 UNCONFIRMED로 유지한다.

## 6. 중복 처리 (multiple targets, Case 10)
- `targets = [{id, xy, n_obs, confirmed, visited, last_seen}]` — 새 확인 위치가 기존 target과 `DEDUP_RADIUS`(목표 크기 + 위치오차, 시작 0.25~0.3 m) 이내면 병합(가중 평균).
- 방문 완료된 target은 detection 결과에서 **무시**(재접근 루프 방지).
- 여러 미방문 target: 거리장(04)으로 가장 가까운 것부터 (TSP 불필요 — 개수 적음).
- 종료: `config.REQUIRED_TARGETS` (개수 미공개 → 파라미터화). `N`이면 N개 방문 즉시 RETURN_HOME, `None`이면 탐색·view frontier 소진 또는 시간 예산 트리거까지 계속.

## 7. 일시적 가림 / 소실 대응 (Case 8)
1. 마지막 bearing과 map 위치를 `TARGET_MEMORY_S`(2~3 s) 유지.
2. 소실 시: 제자리에서 마지막 bearing 쪽으로 회전(±30°) → 재획득 시 계속.
3. 여전히 없음: **저장된 map 위치로 계획 기반 접근**(시야와 무관하게 이동) → 근처에서 look-around.
4. 그 위치에 도착했는데 안 보임: tentative면 폐기, confirmed면 "주변 반경 0.3 m 탐색" 후 blacklist(TTL).

## 8. 접근 (Q13) — 2단계가 가장 단순하고 안정적

| 단계 | 방법 | 근거 |
|---|---|---|
| A. 원거리 | map 위치 → `relax_goal`로 **standoff 셀**(목표에서 d_stand, 가시선 확보, inflated FREE) 선택 → A* + RPP-lite | SemExp: 목표 영역 팽창 goal ✅; FAR goal 재평가 ✅ |
| B. 근거리 (~0.5 m는 INITIAL TUNING) | **visual servo**: `ω = −k_ω·(cx−W/2)/(W/2)`, 신뢰 가능한 target 거리로 v 제한. 거리 미확정이면 전진 보류·재관측 | Chaumette & Hutchinson 2006 [doi:10.1109/MRA.2006.250573](https://doi.org/10.1109/MRA.2006.250573) 📄 |
| C. 도착 | 대회 도착 조건 [DAY-OF] 충족 시 정지·visited. 픽셀 면적만으로 임의 완료 판정하지 않음 | Erebus 정지 1 s는 [REFERENCE, NOT TECH WEEK RULE]. 우리 정지 유지시간도 [INITIAL TUNING]이며 공식 도착 조건이 우선 |

- 안전 모니터는 B/C 단계에서도 항상 작동. 목표 근처 장애물(Case 9)은 A 단계의 standoff 선택에서 inflated FREE + 가시선 조건으로 해결.
- `d_stop`은 규정의 "위치까지 이동" 기준(예: 몇 cm 이내)에 맞춰 config에서 설정.

## 9. Exploration goal vs Target goal 충돌 (02/10 공통)
- target CONFIRMED → 즉시 APPROACH_TARGET 선점 (탐색 goal 폐기, blacklist 아님).
- 단, **남은 시간 < 접근 + 복귀 ETA**면 접근하지 않고 위치만 기록 후 RETURN_HOME (08).
- 접근 완료 → EXPLORE 재개 시 progress grace + frontier 재선택 (hysteresis 초기화).

## 10. 인터페이스 주의
- 고정 규격 `target = {"found", "cx", "direction", "area"}`는 **바꾸지 않는다**. 확인/추적/map 위치는 `detection.py` 내부의 `TargetTracker` 클래스가 보유하고 main이 조회.
- target map 위치를 다른 모듈과 공유 포맷으로 만들려면 AGENTS.md 규칙대로 **INTERFACES.md·interfaces.py·tests 동시 수정 + 사람 확인** 필요.

## 11. 테스트 (Webots 없이)
- bearing 공식: cx=W/2 → 0, cx=0 → +FOV/2 근처.
- range: 합성 스캔에서 β±2° 중앙값.
- map 위치: 알려진 pose/bearing/range → 좌표 일치.
- M-of-N: 3/5 검출 → confirmed, 2/5 → tentative, 위치 분산 크면 미확인.
- dedup: 0.2 m 떨어진 두 검출 → 1개, 0.6 m → 2개.
- visited target 재검출 무시.
- 소실 시퀀스: memory 내 재획득, 초과 시 map 위치 접근 모드.
