from picamera2 import Picamera2
import cv2
import time

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)

picam2.configure(config)
picam2.start()

time.sleep(2)

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_ARUCO_MIP_36h12
)

detector = cv2.aruco.ArucoDetector(
    dictionary,
    cv2.aruco.DetectorParameters()
)

print("Point camera at marker. Press q to quit.")

while True:
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        print("DETECTED IDS:", ids.flatten())
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow("ArUco Test", frame)

    if cv2.waitKey(1) == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()