# 13. GPU 사용 가치 분석

## 결론: **NOT RECOMMENDED** (대회 당일 target이 색/형태로 분리 불가능할 때만 CPU 경량 대안을 먼저 검토)

## 1. 근거

| 근거 | 내용 | 출처 |
|---|---|---|
| 과제 안내 | "Object Detection 자체는 복잡하게 출제하지 않음", 시각적 특징은 당일 제공 | 해커톤 안내 |
| 카메라 해상도 | practice e-puck 52×39 px — DNN 입력으로 정보량이 거의 없음 (공식 로봇이 다르면 재평가) | ✅ E-puck.proto |
| 모듈형 vs 학습형 | 실제 환경에서 modular 90% vs end-to-end 23% | Gervet 2023 [doi:10.1126/scirobotics.adf6991](https://doi.org/10.1126/scirobotics.adf6991) 📄 |
| 학습 탐색 모델 | SemExp/PONI/VLFM은 Habitat 데이터셋·학습·GPU·대형 의존성 필요 | ✅ 각 repo 확인 |
| 규칙 | 새 dependency는 사람 확인 필요 (AGENTS.md), 현재 표준 라이브러리 + NumPy만 | 프로젝트 규칙 |
| 우리 병목 | 성능 병목은 Python 계획/매핑(수십 ms)이지 인식이 아님 | 04 실측 |

## 2. 비교

| | CPU only (권장) | GPU |
|---|---|---|
| 인식 | HSV/형태/contour, <1 ms (52×39) | YOLO 등 neural detector |
| 탐색 | frontier + IG | semantic exploration (SemExp/PONI/VLFM) |
| 이동 | A* + RPP-lite | learned navigation |
| 점수 향상 가능성 | 과제가 단순하면 동등 | 복잡한 target에서만 |
| 위험 | 조명/색 변화 | CUDA/드라이버 설치, 모델 가중치, 학습 데이터 없음, 추론 지연, 새 의존성 |
| 설치 비용 | 0 | 높음 |

## 3. 당일 판단 절차
1. 제공된 target 특징이 **색(HSV)으로 분리 가능** → CPU HSV (현재 성공 사례 있음).
2. 색이 배경과 겹침 → 형태(contour 원형도/종횡비) + 크기 + 다중 프레임 확인 (CPU).
3. 그래도 불가 → OpenCV template matching / ORB (CPU, OpenCV는 이미 연습에 사용; baseline 편입 시 사람 확인).
4. 그 이후에만 CPU 경량 DNN(OpenCV DNN 모듈) 검토. **GPU는 끝까지 필수 아님.**
