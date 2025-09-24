#!/usr/bin/env python3

import depthai as dai
import json
import numpy as np

def get_camera_coefficients(device: dai.Device):
    """
    Gets the factory calibration information from the Luxonis OAKD-Wide Camera and prints them out to numpy arrays for use with OpenCV.
    """

    calibrationData = device.readCalibration()
    # RGB Cam == CAM_A
    distortion_A = np.array(calibrationData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
    intrinsics_A = np.array(calibrationData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A))

    # LEFT Cam == CAM_B
    distortion_B = np.array(calibrationData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B))
    intrinsics_B = np.array(calibrationData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B))

    # RIGHT Cam == CAM_C
    distortion_C = np.array(calibrationData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C))
    intrinsics_C = np.array(calibrationData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C))

    # Print the necessary calibration data
    print("Distortion_A:", distortion_A)
    print("Intrinsics A:", intrinsics_A)

if __name__ == '__main__':
    # Connect to the camera
    device = dai.Device(dai.UsbSpeed.HIGH)

    # Print the camera data and save it to an np.array 
    get_camera_coefficients(device)
