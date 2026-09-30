import os
import sys

from controller import Robot
import cv2
import numpy as np


# 선택 사항: 환경변수 DETECTION_LOG가 지정되면 콘솔 출력을 그 파일로도 남긴다
# (자동 테스트용. 평소 실행에는 영향 없음)
class _Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, text):
        for s in self.streams:
            s.write(text)
            s.flush()

    def flush(self):
        for s in self.streams:
            s.flush()


if os.environ.get("DETECTION_LOG"):
    sys.stdout = _Tee(sys.stdout, open(os.environ["DETECTION_LOG"], "w", encoding="utf-8"))


robot = Robot()
timestep = int(robot.getBasicTimeStep())

# -------------------------
# Camera 설정
# -------------------------

camera = robot.getDevice("camera")
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

print("Detection controller started!")
print("Camera connected!")
print(f"width = {width}")
print(f"height = {height}")

# -------------------------
# Detection 파라미터
# -------------------------

# HSV에서 빨강은 0 근처 + 180 근처 두 영역으로 나뉨
# 채도(S) 하한을 150으로 올려 바닥의 붉은 갈색 나무 무늬(S≈90~110)는 제외하고
# 목표물(S≈220)만 검출한다.
LOWER_RED1 = np.array([0, 150, 80])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 150, 80])
UPPER_RED2 = np.array([180, 255, 255])

# e-puck 카메라가 52x39라서 아주 작은 영역도 인정
MIN_AREA = 5

# 표시용 확대 배율 (detection 계산에는 사용하지 않음)
DISPLAY_SCALE = 8


def get_frame():
    """Webots camera image(BGRA) → OpenCV BGR frame. 이미지가 없으면 None."""
    image = camera.getImage()
    if image is None:
        return None
    frame = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)


def detect_red_target(frame):
    """가장 큰 빨간 영역을 찾아 (target, mask, bbox)를 반환한다."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1) | cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)

    target = {
        "found": False,
        "cx": None,
        "direction": None,
        "area": 0
    }
    bbox = None

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return target, mask, bbox

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area <= MIN_AREA:
        return target, mask, bbox

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2

    # 가로 폭 3등분: LEFT / CENTER / RIGHT
    if cx < width / 3:
        direction = "LEFT"
    elif cx < width * 2 / 3:
        direction = "CENTER"
    else:
        direction = "RIGHT"

    target = {
        "found": True,
        "cx": cx,
        "direction": direction,
        "area": float(area)
    }
    bbox = (x, y, w, h)
    return target, mask, bbox


def show_debug(frame, mask, bbox):
    """원본 frame 대신 확대본 위에 bounding box / 중심점을 그려서 표시."""
    display = cv2.resize(frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                         interpolation=cv2.INTER_NEAREST)
    mask_display = cv2.resize(mask, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                              interpolation=cv2.INTER_NEAREST)

    # LEFT / CENTER / RIGHT 경계선
    for i in (1, 2):
        lx = int(width * i / 3 * DISPLAY_SCALE)
        cv2.line(display, (lx, 0), (lx, display.shape[0]), (200, 200, 200), 1)

    if bbox is not None:
        x, y, w, h = bbox
        s = DISPLAY_SCALE
        cx = x + w // 2
        cy = y + h // 2
        # 검출된 물체에 초록색 사각형
        cv2.rectangle(display, (x * s, y * s), ((x + w) * s, (y + h) * s), (0, 255, 0), 2)
        # 물체 중심점 (파란색)
        cv2.circle(display, (cx * s + s // 2, cy * s + s // 2), 5, (255, 0, 0), -1)

    cv2.imshow("E-puck Camera", display)
    cv2.imshow("Red Mask", mask_display)
    cv2.waitKey(1)


# 두 디버그 창이 겹치지 않도록 나란히 배치
cv2.namedWindow("E-puck Camera", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("Red Mask", cv2.WINDOW_AUTOSIZE)
cv2.moveWindow("E-puck Camera", 40, 60)
cv2.moveWindow("Red Mask", 40 + width * DISPLAY_SCALE + 30, 60)

last_state = None
image_received = False

while robot.step(timestep) != -1:

    frame = get_frame()
    if frame is None:
        continue

    if not image_received:
        print("Camera image received!")
        image_received = True

    target, mask, bbox = detect_red_target(frame)

    # 매 프레임 출력하지 않고 상태(found/direction)가 바뀔 때만 출력
    state = target["direction"] if target["found"] else "NOT_FOUND"
    if state != last_state:
        if target["found"]:
            print(
                f"TARGET FOUND | "
                f"cx={target['cx']} | "
                f"direction={target['direction']} | "
                f"area={target['area']:.1f}"
            )
        else:
            print("TARGET NOT FOUND")
        print(f"target = {target}")
        last_state = state

    show_debug(frame, mask, bbox)

cv2.destroyAllWindows()
