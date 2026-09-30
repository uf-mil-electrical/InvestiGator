import cv2
import numpy as np
import time
from picamera2 import Picamera2

picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (870, 480), "format": "RGB888"}
    )
)

picam2.start()

last_blue = False
candidate_blue = False
candidate_blue_start = time.monotonic()
blue_start = None
blue_changes = []

last_red = False
candidate_red = False
candidate_red_start = time.monotonic()
red_start = None
red_changes = []

last_green = False
candidate_green = False
candidate_green_start = time.monotonic()
green_start = None
green_changes = []

while True:
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    h, s, v = hsv[center_y, center_x]

    cv2.circle(
        frame,
        (center_x, center_y),
        5,
        (255, 255, 255),
        -1
    )

    text = "H:{h} S:{s} V:{v}"

    text_size = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        2
    )[0]

    cv2.putText(
        frame,
        text,
        (width - text_size[0] - 20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    blue_min = np.array([100, 150, 110])
    blue_max = np.array([135, 255, 150])

    green_min = np.array([40, 80, 80])
    green_max = np.array([85, 255, 255])

    red_min1 = np.array([0, 100, 80])
    red_max1 = np.array([10, 255, 255])

    red_min2 = np.array([160, 150, 120])
    red_max2 = np.array([190, 255, 255])

    blue_mask = cv2.inRange(hsv, blue_min, blue_max)

    green_mask = cv2.inRange(hsv, green_min, green_max)

    red_mask1 = cv2.inRange(hsv, red_min1, red_max1)
    red_mask2 = cv2.inRange(hsv, red_min2, red_max2)
    red_mask = red_mask1 | red_mask2

    blue = cv2.countNonZero(blue_mask) / blue_mask.size > 0.02
    red = cv2.countNonZero(red_mask) / red_mask.size > 0.02
    green = cv2.countNonZero(green_mask) / green_mask.size > 0.02

    now = time.monotonic()

    if blue != candidate_blue:
        candidate_blue = blue
        candidate_blue_start = now

    if candidate_blue != last_blue and now - candidate_blue_start >= 0.2:
        last_blue = candidate_blue
        blue_changes.append(now)

        if last_blue:
            blue_start = now
        else:
            blue_start = None

    if red != candidate_red:
        candidate_red = red
        candidate_red_start = now

    if candidate_red != last_red and now - candidate_red_start >= 0.2:
        last_red = candidate_red
        red_changes.append(now)

        if last_red:
            red_start = now
        else:
            red_start = None

    if green != candidate_green:
        candidate_green = green
        candidate_green_start = now

    if candidate_green != last_green and now - candidate_green_start >= 0.2:
        last_green = candidate_green
        green_changes.append(now)

        if last_green:
            green_start = now
        else:
            green_start = None

    blue_changes = [t for t in blue_changes if now - t < 4]
    red_changes = [t for t in red_changes if now - t < 4]
    green_changes = [t for t in green_changes if now - t < 4]

    if len(blue_changes) >= 4:
        blue_label = "FLASHING BLUE"
    elif blue and blue_start is not None and now - blue_start >= 3:
        blue_label = "SOLID BLUE"
    elif blue:
        blue_label = "BLUE"
    else:
        blue_label = ""

    if len(red_changes) >= 4:
        red_label = "FLASHING RED"
    elif red and red_start is not None and now - red_start >= 3:
        red_label = "SOLID RED"
    elif red:
        red_label = "RED"
    else:
        red_label = ""

    if len(green_changes) >= 4:
        green_label = "FLASHING GREEN"
    elif green and green_start is not None and now - green_start >= 3:
        green_label = "SOLID GREEN"
    elif green:
        green_label = "GREEN"
    else:
        green_label = ""

    cv2.putText(
        frame,
        blue_label,
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 0, 0),
        2
    )

    cv2.putText(
        frame,
        red_label,
        (30, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        2
    )

    cv2.putText(
        frame,
        green_label,
        (30, 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Camera", frame)
    cv2.imshow("Blue Mask", blue_mask)
    cv2.imshow("Red Mask", red_mask)
    cv2.imshow("Green Mask", green_mask)

    if cv2.waitKey(1) == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()