from multiprocessing import Process, Queue, Event
from typing import List, Tuple, Optional

import depthai as dai
import numpy as np
import cv2
import time

# TODO: Dictionary for the pose queue, paths of model blobs

# Fiducial Properties
FIDUCIAL_IDS = [0, 4]
MARKER_SIZE_M = 0.18

# Camera Properties
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

# [x,y,z] coordinates of each corner
MARKER_CORNERS_TOP_LEFT = np.array([
    [0,             0,              0], # Top left
    [MARKER_SIZE_M, 0,              0], # Top right
    [MARKER_SIZE_M, MARKER_SIZE_M,  0], # Bottom right
    [0,             MARKER_SIZE_M,  0]  # Bottom left
], dtype=np.float32)

MARKER_CORNERS_CENTER = np.array([
    [-MARKER_SIZE_M / 2,  MARKER_SIZE_M / 2, 0],  # Top left
    [ MARKER_SIZE_M / 2,  MARKER_SIZE_M / 2, 0],  # Top right
    [ MARKER_SIZE_M / 2, -MARKER_SIZE_M / 2, 0],  # Bottom right
    [-MARKER_SIZE_M / 2, -MARKER_SIZE_M / 2, 0]   # Bottom left
], dtype=np.float32)


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
        
        self.detection_queue = detection_queue
        self.preview = preview
        self.running = Event()

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
        self.video_process = Process(target=self.video_loop, daemon=True)
        self.video_process.start()


    def stop(self, timeout: float | None = None):
        """
        Stop the video_loop process if it is running and join the process with the calling process.
        """
        self.running.clear()
        if self.video_process is None:
            return
        
        self.video_process.join(timeout=timeout)

        if self.video_process.is_alive():
            self.video_process.terminate()

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

    def process_fiducial_frame(self, frame: np.ndarray, detector: cv2.aruco.ArucoDetector) -> np.ndarray:
        """
        Process input frame and search for fiducial markers. Draw markers to screen if preview is enabled.
        """
        grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_filtered_markers(grey_frame, detector)

        if len(ids) == 0 or len(corners) == 0:
            self.draw_center_box(frame)
            return frame

        for i, (marker_corners, marker_id) in enumerate(zip(corners, ids)):
            success, rvec, tvec = cv2.solvePnP(MARKER_CORNERS_CENTER, marker_corners.reshape(-1,2), CAM_MATRIX, DIST_COEFF_ZERO)

            if not success:
                continue

            # Convert tvec from camera frame (+X is horizonal, +Y is vertical) to FRD (+X is forward, +Y is right)
            tvec = tvec.flatten()
            delta_xyz_m = [-tvec[1], tvec[0], tvec[2]]
            self.detection_queue.put(delta_xyz_m)

            if self.preview:
                cv2.aruco.drawDetectedMarkers(frame, [marker_corners], marker_id)

                pixel_coordinates = f"ID: {ids[i][0]} | X: {delta_xyz_m[0]:.3f}m Y: {delta_xyz_m[1]:.3f}m Z: {delta_xyz_m[2]:.3f}m"
                cv2.putText(frame, pixel_coordinates, (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                marker_center = np.array([0,0,0], dtype=np.float32)
                marker_center, _ = cv2.projectPoints(marker_center, rvec, tvec, CAM_MATRIX, DIST_COEFF_ZERO)
                marker_center = tuple(marker_center[0][0].astype(int))
                cv2.circle(frame, marker_center, 10, (0,255,0), 1)

                self.draw_center_box(frame)
            
        return frame

    def video_loop(self):
        """
        Process images from camera and output detection data to detection_queue.
        """
        with dai.Pipeline() as pipeline:
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            
            if self.mode in ("UAV Recovery", "Recording"):
                video_queue = cam.requestOutput(size=(1280,720), enableUndistortion=True, fps=30).createOutputQueue()
                detector = self.aruco_detector()

            pipeline.start()
            while self.running.is_set():    
                frame = video_queue.get()
                assert isinstance(frame, dai.ImgFrame)
                frame = frame.getCvFrame()

                if self.mode == "UAV Recovery":
                    frame = self.process_fiducial_frame(frame, detector)


                if self.preview:
                    cv2.imshow("video", frame)

                    if cv2.waitKey(1) == ord("q"):
                        self.running.clear()
                        break

            cv2.destroyAllWindows()
            return
            
if __name__ == "__main__":
    
    queue = Queue()
    camera = Camera(queue, preview=True)
    camera.switch_mode("UAV Recovery")
    camera.start()
    
    try:
        while (camera.running.is_set()):
            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
    