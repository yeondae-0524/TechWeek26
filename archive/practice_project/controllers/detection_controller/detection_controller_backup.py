from controller import Robot
import cv2
import numpy as np


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
print(f"Camera resolution: {width} x {height}")

last_result = None


while robot.step(timestep) != -1:

    # -------------------------
    # 1. Camera image 읽기
    # -------------------------

    image = camera.getImage()

    if image is None:
        continue

    frame = np.frombuffer(image, np.uint8)
    frame = frame.reshape((height, width, 4))

    # Webots BGRA → OpenCV BGR
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)


    # -------------------------
    # 2. BGR → HSV
    # -------------------------

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    # -------------------------
    # 3. 빨간색 추출
    #
    # HSV에서 빨강은 범위가
    # 0 근처 + 180 근처로 나뉨
    # -------------------------

    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = mask1 | mask2


    # -------------------------
    # 4. 빨간 영역 찾기
    # -------------------------

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )


    target = {
        "found": False,
        "cx": None,
        "direction": None,
        "area": 0
    }


    if contours:

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # e-puck 카메라가 52x39라서
        # 아주 작은 영역도 인정
        if area > 5:

            x, y, w, h = cv2.boundingRect(largest)

            cx = x + w // 2
            cy = y + h // 2


            # -------------------------
            # 5. LEFT / CENTER / RIGHT
            # -------------------------

            if cx < width / 3:
                direction = "LEFT"

            elif cx > width * 2 / 3:
                direction = "RIGHT"

            else:
                direction = "CENTER"


            target = {
                "found": True,
                "cx": cx,
                "direction": direction,
                "area": area
            }


            # 검출된 물체에 초록색 사각형
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                1
            )

            # 물체 중심점
            cv2.circle(
                frame,
                (cx, cy),
                1,
                (255, 0, 0),
                -1
            )


    # -------------------------
    # 6. console 출력
    # -------------------------

    if target["found"]:
        result = target["direction"]

        # 매 프레임마다 출력하면 너무 많이 찍히므로
        # 상태가 변할 때만 출력
        if result != last_result:

            print(
                f"TARGET FOUND | "
                f"cx={target['cx']} | "
                f"direction={target['direction']} | "
                f"area={target['area']:.1f}"
            )

            last_result = result

    else:

        if last_result != "NOT_FOUND":
            print("TARGET NOT FOUND")
            last_result = "NOT_FOUND"


    # -------------------------
    # 7. 사람이 보기 좋게 확대
    # -------------------------

    display = cv2.resize(
        frame,
        None,
        fx=8,
        fy=8,
        interpolation=cv2.INTER_NEAREST
    )

    mask_display = cv2.resize(
        mask,
        None,
        fx=8,
        fy=8,
        interpolation=cv2.INTER_NEAREST
    )


    cv2.imshow("E-puck Camera", display)
    cv2.imshow("Red Mask", mask_display)

    cv2.waitKey(1)