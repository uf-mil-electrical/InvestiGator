from multiprocessing import Process, Queue, Event
from multiprocessing.synchronize import Event as EventType
from typing import List, Tuple, Optional
from collections import namedtuple
import depthai as dai
from pathlib import Path
from datetime import datetime

import numpy as np
import cv2
try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None
import time

RECORDING_DIR = Path(__file__).parent.parent / "recordings"
RECODRING_RESOLUTION = (1280, 720)
DETECTION_RESOLUTION = (1280, 720)
BLUE_MIN = np.array([100, 150, 110])
BLUE_MAX = np.array([135, 255, 220])

GREEN_MIN = np.array([40, 80, 80])
GREEN_MAX = np.array([85, 255, 255])

RED_MIN_1 = np.array([0, 100, 80])
RED_MAX_1 = np.array([10, 255, 255])

RED_MIN_2 = np.array([160, 150, 120])
RED_MAX_2 = np.array([179, 255, 255])

# TODO: Dictionary for the pose queue, paths of model blobs

# Fiducial Properties
FIDUCIAL_IDS = [0, 4]
MARKER_SIZE_M = 0.186

# DepthAI Camera Properties
DIST_COEFF = np.array(
    [ 4.36490011e+00,  1.47384214e+00, -6.14358260e-06,  9.69038447e-05,
    3.05115115e-02,  4.72419739e+00,  2.81013250e+00,  2.22624362e-01,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    -2.07635248e-03,  1.58486480e-03], 
dtype=np.float32)

DIST_COEFF_ZERO = np.array([0,0,0,0,0], dtype=np.float32)

CAM_MATRIX = np.array([ 
    [563.78790283, 0.          , 618.17443848],
    [  0.        , 563.56225586, 410.73165894],
    [  0.        , 0.          , 1.          ]
], dtype=np.float32)

# Generic Camera Properties
DIST_COEFF_GEN = np.array(
    [2.05104476e+01, 2.82916238e+00, 5.71330358e-04, 2.07891196e-03, 
     -2.35404136e+00, 2.05169528e+01, 1.13939475e+00, -9.54340464e-02, 
     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 
     0.00000000e+00, 0.00000000e+00],
    dtype=np.float32)

CAM_MATRIX_GEN = np.array([
    [1412.41925199, 0.00000000, 685.21145798],
    [0.00000000, 1409.77774826, 382.57045326],
    [0.00000000, 0.00000000, 1.00000000],
], dtype=np.float32)

# [x,y,z] coordinates of each corner
MARKER_CORNERS_CENTER = np.array([
    [-MARKER_SIZE_M / 2, -MARKER_SIZE_M / 2, 0],  # Top left
    [ MARKER_SIZE_M / 2, -MARKER_SIZE_M / 2, 0],  # Top right
    [ MARKER_SIZE_M / 2,  MARKER_SIZE_M / 2, 0],  # Bottom right
    [-MARKER_SIZE_M / 2,  MARKER_SIZE_M / 2, 0]   # Bottom left
], dtype=np.float32)

MarkerDetection = namedtuple("MarkerDetection",["X_Offset_m", "Y_Offset_m", "Z_Offset_m"])
SafePassageDetection = namedtuple("SafePassageDetection",["marker_type", "x_pixel", "y_pixel"])

class Camera:
    """
    Manages Luxonis Oak-D-W camera and processes image detections in a separate process.
    Pose estimates are output through a multiprocessing queue.
    Detections:
        - UAV Recovery: Fiducial Marker
        - UAV Search and Rescue: RoboR and RoboN
        - UAV Replinishment: RoboPad and colored Tins
    """

    def __init__(self, detection_queue: Queue, preview: bool = False):
        
        self.detection_queue: Queue[MarkerDetection] = detection_queue
        self.preview = preview
        self.running: EventType = Event()

        self.mode = "Recording"
        self.video_process: Optional[Process] = None


    def aruco_detector(self) -> cv2.aruco.ArucoDetector:
        """
        Create and return a cv2.aruco.ArucoDetector.
        """
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_MIP_36h12)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)
        return detector


    def detect_filtered_markers(self, frame: np.ndarray, detector: cv2.aruco.ArucoDetector, filter_list: Tuple[int,...] = tuple(FIDUCIAL_IDS)) -> Tuple[List[np.ndarray], np.ndarray]:
        """
        Detect markers and return corners and ids filtered by filter_list.
        """
        # Notes:
        # - corners is a list of arrays with shape (1, 4, 2) -> (marker, corners, (x,y))
        # ids is an array with shape (N, 1) -> (marker, id)

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return [], np.array([], dtype=np.int32)

        bool_array = np.isin(ids, filter_list).flatten()

        corners_filtered = [corner for corner, valid in zip(corners, bool_array) if valid]
        ids_filtered = ids[bool_array].astype(np.int32)

        if len(ids_filtered) == 0:
            return [], np.array([], dtype=np.int32)

        return corners_filtered, ids_filtered


    def start(self):
        """
        Start camera process if not already running.
        """
        if self.video_process is not None and self.video_process.is_alive():
            return
        
        self.running.set()
        self.video_process = Process(target=self.video_loop, daemon=True, args=(self.running,))
        self.video_process.start()


    def stop(self, timeout=2.0):
        """
        Stop the video_loop process if it is running and join the process with the calling process.
        """
        self.running.clear()
        if self.video_process is None:
            return
        
        self.video_process.join(timeout=timeout)

        if self.video_process.is_alive():
            self.video_process.terminate()
            self.video_process.join(timeout=1.0)

        self.video_process = None
        

    def switch_mode(self, mode: str = "Recording"):
        """
        Stop current video process if necessary and start a new process with new mode and model detector.
        """
        if mode == self.mode:
            return
        self.stop()
        self.mode = mode
        self.start()

    def draw_center_box(self, frame: np.ndarray):
        """
        Draw a square to center of preview window to show approximate alignment between center of marker and center of window.
        """
        height, width = frame.shape[:2]
        pt1 = width // 2 - 10, height // 2 - 10
        pt2 = width // 2 + 10, height // 2 + 10
        cv2.rectangle(frame, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)

    def detect_water(self, mask):
        """
        Return valid color "blobs" from a color mask in order to filter out water and other unimportant colors.
        """
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if 30 < area < 4000:
                x, y, w, h = cv2.boundingRect(contour)
                blobs.append({
                    "x": x + w // 2,
                    "y": y + h // 2,
                    "box": (x, y, w, h)
                })

        return blobs
    def update_buoy_locations(self, tracks, blobs, now):
        """
        Match color blobs to detected buoy locations to remember places of them.
        """
        matched = set()
        for blob in blobs:
            closest_id = None
            closest_distance = 80
            for track_id, track in tracks.items():
                dx = blob["x"] - track["x"]
                dy = blob["y"] - track["y"]
                distance = (dx * dx + dy * dy) ** 0.5

                if distance < closest_distance:
                    closest_distance = distance
                    closest_id = track_id

            if closest_id is None:
                closest_id = len(tracks)
                tracks[closest_id] = {
                    "x": blob["x"],
                    "y": blob["y"],
                    "box": blob["box"],
                    "on": True,
                    "on_start": now,
                    "last_change": now,
                    "changes": [],
                    "last_report": 0
                }
            track = tracks[closest_id]
            track["x"] = blob["x"]
            track["y"] = blob["y"]
            track["box"] = blob["box"]

            matched.add(closest_id)
            if not track["on"]:
                track["on"] = True
                track["on_start"] = now
                track["last_change"] = now
                track["changes"].append(now)

        for track_id, track in tracks.items():
            if track_id not in matched and track["on"]:
                if now - track["last_change"] >= 0.2:
                    track["on"] = False
                    track["last_change"] = now
                    track["changes"].append(now)

            track["changes"] = [
                t for t in track["changes"]
                if now - t < 6
            ]
    def buoy_meaning(self, tracks, color, frame, now):
        """
        Identify buoy meaning.
        """
        for track in tracks.values():

            marker_type = None

            if color == "BLUE":
                if len(track["changes"]) >= 3:
                    marker_type = "ENTRY"

                elif track["on"] and now - track["on_start"] >= 3:
                    marker_type = "EXIT"

            elif color == "RED":
                if len(track["changes"]) >= 3:
                    marker_type = "RED"

            elif color == "GREEN":
                if len(track["changes"]) >= 3:
                    marker_type = "GREEN"

            if marker_type is None:
                continue

            x, y, w, h = track["box"]

            if self.preview:
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    marker_type,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

            if now - track["last_report"] >= 1:
                self.detection_queue.put(
                    SafePassageDetection(
                        marker_type,
                        track["x"],
                        track["y"]
                    )
                )
                track["last_report"] = now

    def process_safe_passage_frame(self, frame, blue_tracks, red_tracks, green_tracks):
        """
        Detect Task 1 light beacons for safe passage and to then identify which it is and what it is communicating.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(
            hsv,
            BLUE_MIN,
            BLUE_MAX
        )
        green_mask = cv2.inRange(
            hsv,
            GREEN_MIN,
            GREEN_MAX
        )
        red_mask_1 = cv2.inRange(
            hsv,
            RED_MIN_1,
            RED_MAX_1
        )
        red_mask_2 = cv2.inRange(
            hsv,
            RED_MIN_2,
            RED_MAX_2
        )
        red_mask = red_mask_1 | red_mask_2
        blue_blobs = self.detect_water(blue_mask)
        red_blobs = self.detect_water(red_mask)
        green_blobs = self.detect_water(green_mask)

        now = time.monotonic()
        self.update_buoy_locations(
            blue_tracks,
            blue_blobs,
            now
        )
        self.update_buoy_locations(
            red_tracks,
            red_blobs,
            now
        )
        self.update_buoy_locations(
            green_tracks,
            green_blobs,
            now
        )

        self.buoy_meaning(
            blue_tracks,
            "BLUE",
            frame,
            now
        )

        self.buoy_meaning(
            red_tracks,
            "RED",
            frame,
            now
        )

        self.buoy_meaning(
            green_tracks,
            "GREEN",
            frame,
            now
        )

        return frame
    def process_fiducial_frame(self, frame: np.ndarray, detector: cv2.aruco.ArucoDetector, cam_matrix: np.ndarray, dist_coeffs: np.ndarray) -> np.ndarray:
        """
        Process input frame and search for fiducial markers. Draw markers to screen if preview is enabled.
        """
        grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_filtered_markers(grey_frame, detector)

        if len(ids) == 0 or len(corners) == 0:
            if self.preview:
                self.draw_center_box(frame)
            return frame

        for i, (marker_corners, marker_id) in enumerate(zip(corners, ids)):
            success, rvec, tvec = cv2.solvePnP(MARKER_CORNERS_CENTER, marker_corners.reshape(-1,2), cam_matrix, dist_coeffs)

            if not success:
                continue

            # Convert tvec from camera frame (+X is horizonal, +Y is vertical) to FRD (+X is forward, +Y is right)
            tvec = tvec.flatten()
            delta_xyz_m = MarkerDetection(-1*tvec[1], tvec[0], tvec[2])
            self.detection_queue.put(delta_xyz_m)

            if self.preview:
                cv2.aruco.drawDetectedMarkers(frame, [marker_corners], marker_id)

                pixel_coordinates = f"ID: {ids[i][0]} | X: {delta_xyz_m[0]:.3f}m Y: {delta_xyz_m[1]:.3f}m Z: {delta_xyz_m[2]:.3f}m"
                cv2.putText(frame, pixel_coordinates, (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                marker_center = np.array([0,0,0], dtype=np.float32)
                marker_center, _ = cv2.projectPoints(marker_center, rvec, tvec, cam_matrix, dist_coeffs)
                marker_center = tuple(marker_center[0][0].astype(int))
                cv2.circle(frame, marker_center, 10, (0,255,0), 1)

                frame_center = (frame.shape[1]//2, frame.shape[0]//2)
                cv2.line(frame, marker_center, frame_center, (255,0,0), 1)

                self.draw_center_box(frame)
            
        return frame

    def depthai_loop(self, running: EventType):
        """
        Process images from DepthAI camera and output detection data to detection_queue.
        """
        with dai.Pipeline() as pipeline:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            
            videoEncoder = pipeline.create(dai.node.VideoEncoder).build(cam.requestOutput(RECODRING_RESOLUTION, dai.ImgFrame.Type.NV12, enableUndistortion=True))
            videoEncoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)
            record = pipeline.create(dai.node.RecordVideo)
            record.setRecordVideoFile(RECORDING_DIR / datetime.now().strftime("%d-%m-%y_%H-%M"))
            videoEncoder.out.link(record.input)
            
            detector = None

            if self.mode in ("UAV Recovery", "Recording"):
                video_queue = cam.requestOutput(size=DETECTION_RESOLUTION, enableUndistortion=True, fps=30).createOutputQueue()
                detector = self.aruco_detector()
            
            else:
                print(f"Selected mode not supported: {self.mode}. Defaulting to recording mode.")
                video_queue = cam.requestOutput(size=DETECTION_RESOLUTION, enableUndistortion=True, fps=30).createOutputQueue()

            pipeline.start()
            try:
                while running.is_set():    
                    frame = video_queue.get()
                    assert isinstance(frame, dai.ImgFrame)
                    frame = frame.getCvFrame()

                    if self.mode == "UAV Recovery" and detector is not None:
                        frame = self.process_fiducial_frame(frame, detector, CAM_MATRIX, DIST_COEFF_ZERO)

                    if self.preview:
                        cv2.imshow("video", frame)

                        if cv2.waitKey(1) == ord("q"):
                            running.clear()
                            break
            except KeyboardInterrupt:
                pass
            finally:
                cv2.destroyAllWindows()

    def picamera_loop(self, running: EventType):
        """
        Process images from Raspberry Pi camera and output detections to detection_queue.
        """
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": DETECTION_RESOLUTION, "format": "RGB888"}
        )

        picam2.configure(config)
        picam2.start()

        detector = None
        blue_tracks = {}
        red_tracks = {}
        green_tracks = {}

        if self.mode == "UAV Recovery":
            detector = self.aruco_detector()

        try:
            while running.is_set():

                frame = picam2.capture_array()

                if self.mode == "UAV Recovery" and detector is not None:
                    frame = self.process_fiducial_frame(
                        frame,
                        detector,
                        CAM_MATRIX_GEN,
                        DIST_COEFF_GEN
                    )

                elif self.mode == "Safe Passage":
                    frame = self.process_safe_passage_frame(
                        frame,
                        blue_tracks,
                        red_tracks,
                        green_tracks
                    )

                if self.preview:
                    cv2.imshow("video", frame)

                    if cv2.waitKey(1) == ord("q"):
                        running.clear()
                        break

        except KeyboardInterrupt:
            pass

        finally:
            picam2.stop()
            cv2.destroyAllWindows()
                    
    def generic_camera_loop(self, running: EventType):
        """
        Process images from USB camera and output detection data to detection_queue.
        """
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)

        detector = None

        if not cap.isOpened():
            print("Could not open USB camera.")
            return

        if self.mode in ("UAV Recovery", "Recording"):
            detector = self.aruco_detector()

        try:
            while running.is_set():
                ret, frame = cap.read()

                if not ret:
                    continue

                if self.mode == "UAV Recovery" and detector is not None:
                    frame = self.process_fiducial_frame(frame, detector, CAM_MATRIX_GEN, DIST_COEFF_GEN)

                if self.preview:
                    frame = cv2.resize(frame, (1024, 576))
                    cv2.imshow("video", frame)

                    if cv2.waitKey(1) == ord("q"):
                        running.clear()
                        break

        except KeyboardInterrupt:  
            pass

        finally:              
            cap.release()
            cv2.destroyAllWindows()

    def video_loop(self, running: EventType):
        """
        Main loop for video processing.
        """
        if len(dai.Device.getAllAvailableDevices()) != 0:
            self.depthai_loop(running)

        elif Picamera2 is not None:
            print("No DepthAI camera found. Using Raspberry Pi camera.")
            self.picamera_loop(running)

        else:
            print("No DepthAI or Raspberry Pi camera found. Using generic USB camera.")
            self.generic_camera_loop(running)
                
if __name__ == "__main__":
    
    queue = Queue()
    camera = Camera(queue, preview=True)
    camera.switch_mode("UAV Recovery")
    camera.start()
    
    try:
        while (camera.running.is_set()):
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        camera.stop()
    