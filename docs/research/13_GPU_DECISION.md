# 13. GPU 사용 가치 분석

## 결론: GPU는 baseline에 NOT RECOMMENDED; 필요 시 선택적 fallback

[organizer-confirmed] GPU 필수 아님, 필요하면 팀당 1대 수준·RTX server 사용 가능. 단순 target에는 **OpenCV classical CV + 다중 프레임 확인(CPU)** 권장. target 특징은 [DAY-OF]이며 단순함이 검출 성공을 보장하지 않는다. 공식 사실은 [09](09_WEBOTS_REFERENCES.md).

## 1. 근거

| 근거 | 내용 | 출처 |
|---|---|---|
| 과제 안내 | "Object Detection 자체는 복잡하게 출제하지 않음", 시각적 특징은 당일 제공 | [organizer-confirmed] |
| 카메라 해상도 | 640×480, 약 60°; 해상도로 DNN을 배제할 근거 없음 | [OFFICIAL] 공식 6개 robot world |
| 모듈형 vs 학습형 | 외부 연구에서 modular 90% vs end-to-end 23% [REFERENCE]; CPU CV 대 GPU detector 직접 비교는 아님 | Gervet 2023 [doi:10.1126/scirobotics.adf6991](https://doi.org/10.1126/scirobotics.adf6991) 📄 |
| 학습 탐색 모델 | SemExp/PONI/VLFM은 Habitat 데이터셋·학습·GPU·대형 의존성 필요 | ✅ 각 repo 확인 |
| 규칙 | 새 dependency는 사람 확인 필요 (AGENTS.md), 현재 표준 라이브러리 + NumPy만 | 프로젝트 규칙 |
| 우리 병목 | 기존 PC A* 128–514 ms, 합성 영상 CV median 2.47 ms; target runtime 병목은 S0 계측 전 UNCONFIRMED | 04 실측 |

## 2. 비교

| | CPU only (권장) | GPU |
|---|---|---|
| 인식 | HSV/LAB·morphology·contour + M-of-N; 공식 해상도 runtime 계측 필요 | YOLO 등 neural detector |
| 탐색 | frontier + IG | semantic exploration (SemExp/PONI/VLFM) |
| 이동 | A* + RPP-lite | learned navigation |
| 점수 향상 가능성 | 과제가 단순하면 동등 | 복잡한 target에서만 |
| 위험 | 조명/색 변화 | CUDA/드라이버 설치, 모델 가중치, 학습 데이터 없음, 추론 지연, 새 의존성 |
| 설치 비용 | OpenCV 환경 확인 필요 | GPU runtime·가중치·추론 환경 확인 필요 |

## 3. 공식 YOLO sample의 의미

[OFFICIAL] `tb3_teleop_yolo.py`는 ultralytics `yolo11n.pt`를 로드하고 `model.to("cpu")`를 호출한다. 저장소에 가중치는 없고 `models/YOLO/.gitkeep`만 있다. **YOLO sample exists ≠ YOLO necessary**. notebook은 CPU/CUDA 학습 라이브러리 설치 예를 제공하지만 대회 dependency 규정은 별도 확인한다. 이번 pass는 설치·실행하지 않았다.

## 4. 당일 판단 절차
1. 제공된 target 특징이 **색(HSV)으로 분리 가능** → CPU HSV/LAB로 실제 배경·거리·가림별 검출률과 오탐 측정.
2. 색이 배경과 겹침 → 형태(contour 원형도/종횡비) + 크기 + 다중 프레임 확인 (CPU).
3. 그래도 불가 → OpenCV template matching / ORB (CPU, 공식 교육에서 사용 확인; 최종 dependency 제한 및 팀 환경 확인).
4. 그 이후에만 CPU 경량 DNN(OpenCV DNN 모듈) 검토. CPU가 지연 예산을 못 맞추고 DL이 검출 실패를 실제로 줄일 때만 GPU/RTX server를 검토한다. 통신 지연·실패 시 정지/CPU 폴백까지 평가한다. **GPU는 필수 아님.**
