import cv2
import depthai as dai
import numpy as np
import time

# From https://docs.luxonis.com/software-v3/depthai/examples/camera/camera_output/
# From https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

distortionCoefficients = np.array([ 4.36490011e+00,  1.47384214e+00, -6.14358260e-06,  9.69038447e-05,
  3.05115115e-02,  4.72419739e+00,  2.81013250e+00,  2.22624362e-01,
  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
 -2.07635248e-03,  1.58486480e-03], dtype=np.float32)

camMatrix = np.array([[563.78790283,   0.        , 618.17443848],
 [  0.        , 563.56225586, 410.73165894],
 [  0.          , 0.          , 1.        ]], dtype=np.float32)

world_points = np.array([[0.,0.,0.],
            [1.,0.,0.],
            [1.,1.,0.],
            [0.,1.,0.]  
        ], dtype=np.float32)

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    videoQueue = cam.requestOutput((640,400), enableUndistortion=True).createOutputQueue()

    # Aruco marker set up
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_MIP_36h12)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)

    pipeline.start()
    while pipeline.isRunning():
        # Get frames from video stream
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)

        # Convert frame to gray scale, and detect aruco markers
        frame = videoIn.getCvFrame()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(frame)
        
        # Check that any detected markers match the one we are looking for
        if ids is not None:
            for i in range(len(ids)):
                if ids[i][0] == 4 or ids[i][0] == 0:
                    # When the marker is found, alter the data to fit the expected input of drawDetectedMarkers
                    ids_ = np.array(ids[i][0], dtype=np.int32).reshape(-1,1)
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], ids_)

                    # Get pose estimate from cv2.solvePnP
                    _, rvecs, tvecs = cv2.solvePnP(world_points, corners[i], camMatrix, distortionCoefficients)
                    rotation_matrix, _ = cv2.Rodrigues(rvecs)
                    flip_matrix = np.array([
                        [0, 1, 0],
                        [1, 0, 0],
                        [0, 0, -1]
                      ], dtype=np.float32)
                    transformed_rotation_matrix = rotation_matrix @ flip_matrix
                    rvecs_transformed, _ = cv2.Rodrigues(transformed_rotation_matrix)

                    cv2.drawFrameAxes(frame, camMatrix, distortionCoefficients, rvecs_transformed, tvecs, 0.75, 2)

                    # Get xyz coordinates
                    x = tvecs[0][0]
                    y = tvecs[1][0]
                    z = tvecs[2][0]

                    coordinates = f"ID: {ids[i][0]} | X: {x:.2f} Y: {y:.2f} Z: {z:.2f}"

                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    color = (0, 255, 0) # Green
                    thickness = 1

                    top_left_corner = (int(corners[i][0][0], int(corners[i][0][0])))
                    text_position = (top_left_corner[0], top_left_corner[1] - 10)

                    cv2.putText(frame, coordinates, text_position, font, font_scale, color, thickness, cv2.LINE_AA)

        cv2.imshow("video", frame)

        if cv2.waitKey(1) == ord("q"):
            break