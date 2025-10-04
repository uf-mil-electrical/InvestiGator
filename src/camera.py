# Start pipeline
# Assemble pipeline
# Stop pipeline
# Switch model
# Calculate pose (fiducial)
# Calculate pose (nn/depth)
import depthai as dai
from multiprocessing import Process, Queue, Event
import numpy as np
import cv2
from typing import List, Tuple, Optional

# TODO: Dictionary for the pose queue, paths of model blobs

# Camera Properties
distortionCoefficients = np.array([ 4.36490011e+00,  1.47384214e+00, -6.14358260e-06,  9.69038447e-05,
  3.05115115e-02,  4.72419739e+00,  2.81013250e+00,  2.22624362e-01,
  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
 -2.07635248e-03,  1.58486480e-03], dtype=np.float32)

camMatrix = np.array([  [563.78790283,   0.        , 618.17443848],
                        [  0.        , 563.56225586, 410.73165894],
                        [  0.          , 0.          , 1.        ]], dtype=np.float32)

world_points = np.array([[0.,0.,0.],
                        [1.,0.,0.],
                        [1.,1.,0.],
                        [0.,1.,0.]], dtype=np.float32)

flip_matrix = np.array([[0, 1, 0],
                        [1, 0, 0],
                        [0, 0, -1]], dtype=np.float32)

# Fiducial Properties
FIDUCIAL_IDS = [0, 4]


class Camera:
    """
    Manages Luxonis Oak-D-W camera and processes image detections in a separate process.
    Pose estimates are output through a multiprocessing queue.
    Detections:
        - Boat Landing: Fiducial Marker
        - Search and Report: RoboR and RoboN
        - Tin Delivery: RoboPad and colored Tins
    """

    def __init__(self, pose_queue: Queue, preview: bool = False):
        
        self.pose_queue = pose_queue
        self.preview = preview
        self.running = Event()

        def aruco_detector() -> cv2.aruco.ArucoDetector:
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_MIP_36h12)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)
            return detector
        
        self.detector = aruco_detector()
        self.current_model = None

    def fiducial_pipeline(self) -> dai.Pipeline:
        """
        Configure fiducial pipeline to be loaded to the camera.
        """

        pipeline = dai.Pipeline()


        return pipeline
    
    def search_report_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()
        return pipeline

    def uav_replenishment_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()
        return pipeline

    def detect_filtered_markers(self, frame: np.ndarray, filter_list: List[int] = FIDUCIAL_IDS) -> Tuple[Optional[List[np.ndarray]], Optional[np.ndarray]]:
        """
        Detect markers and return corners and ids filtered by filter_list.
        """
        # corners is a list of arrays with shape (1, 4, 2)
        #   Where 1 is the marker, 4 are the corners, and 2 are the (x,y) coordinates
        # ids is an array with shape (N, 1) 
        #   Where N is the marker and 1 is the id value

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None:
            return None, None

        bool_array = np.isin(ids, filter_list).flatten()

        corners_filtered = [corner for corner, valid in zip(corners, bool_array) if valid]
        ids_filtered = ids[bool_array]

        if len(ids_filtered) == 0:
            return None, None

        return corners_filtered, ids_filtered

    def start(self):
        pass

    def stop(self):
        pass

    def switch_model(self):
        pass

    def calculate_fiducial_pose(self):
        pass

