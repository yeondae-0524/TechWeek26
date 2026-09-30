# 이전 독립 주행 시험 controller

실제 주행 구현과 팀 연결은 `../rescue_robot/main.py`로 통합했습니다.
상세 실행 방법은 `../rescue_robot/README.md`를 참고하세요.
`control.py`는 기존 테스트의 import 경로를 유지하는 연결 모듈입니다.
기존 독립 시험 진입점과 odometry는 남아 있지만 팀 실행 진입점은 `rescue_robot`입니다.
`worlds/breakroom_control_test.wbt`도 이제 `rescue_robot`을 실행합니다.
