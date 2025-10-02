# Start pipeline
# Assemble pipeline
# Stop pipeline
# Switch model
# Calculate pose (fiducial)
# Calculate pose (nn/depth)
import depthai as dai
from multiprocessing import Process, Queue, Event

# TODO: Dictionary for the pose queue, paths of model blobs

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

        self.current_model = None

    def define_pipeline(self) -> dai.Pipeline:
        """
        Configure pipeline to be loaded to the camera. This includes the current model.
        """

        pipeline = dai.Pipeline()
        

        return pipeline

    def start(self):
        pass

    def stop(self):
        pass

    def switch_model(self):
        pass

    def calculate_fiducial_pose(self):
        pass

