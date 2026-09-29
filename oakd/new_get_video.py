"""
DepthAI Camera Script for ArduPilot Drone
------------------------------------------
- Streams all connected cameras during flight
- Listens for MAVLink DO_DIGICAM_CONTROL or COMMAND_LONG (MAV_CMD_DO_DIGICAM_CONTROL)
  to trigger a CAM_A image capture
- Saves captured frames to disk with timestamps for training data collection

Dependencies:
    pip install opencv-python depthai pymavlink

Usage:
    python get_video.py --connect /dev/ttyAMA0  # UART to flight controller
    python get_video.py --connect udp:0.0.0.0:14550  # UDP (simulation / telemetry radio)
"""

import cv2
import depthai as dai
import threading
import time
import os
import argparse
from datetime import datetime
from pymavlink import mavutil
import sys, tty, termios
SAVE_DIR = "captured_frames"          
CAM_A_SOCKET = dai.CameraBoardSocket.CAM_A
DISPLAY_SCALE = 0.5                   
CAPTURE_COOLDOWN_S = 0.5              

# MAVLink command to trigger camera capture
# DO_DIGICAM_CONTROL (203) = ArduPilot camera shutter
TRIGGER_CMD = mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL  



capture_event = threading.Event()     # Set this to request a capture
shutdown_event = threading.Event()    # Set this to stop all threads

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def mavlink_listener(connection_string: str) -> None:
    """Connect to the flight controller and watch for camera-trigger commands."""
    print(f"[MAVLink] Connecting to {connection_string} …")
    try:
        mav = mavutil.mavlink_connection(connection_string, baud=115200)
        mav.wait_heartbeat(timeout=30)
        print(f"[MAVLink] Heartbeat received from system "
              f"{mav.target_system} / component {mav.target_component}")
    except Exception as exc:
        print(f"[MAVLink] Connection failed: {exc}")
        shutdown_event.set()
        return

    last_capture_time = 0.0

    while not shutdown_event.is_set():
        msg = mav.recv_match(
            type=["COMMAND_LONG", "DO_DIGICAM_CONTROL"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue

        triggered = False

        if msg.get_type() == "COMMAND_LONG":
            if msg.command == TRIGGER_CMD:
                #1 means "shoot"
                triggered = (msg.param1 == 1)

        elif msg.get_type() == "DO_DIGICAM_CONTROL":
            triggered = (msg.shot == 1)

        if triggered:
            now = time.monotonic()
            if now - last_capture_time >= CAPTURE_COOLDOWN_S:
                last_capture_time = now
                capture_event.set()
                print("[MAVLink] Capture command received → queuing frame save")
            else:
                print("[MAVLink] Capture command ignored (cooldown)")


# Image saving helper
def save_frame(frame, save_dir: str) -> str:
    """Save a single OpenCV frame to *save_dir* with a timestamp filename."""
    os.makedirs(save_dir, exist_ok=True)
    ts = datetime.utcnow().strftime("%Y%m%dT%H%M%S_%f")
    filename = os.path.join(save_dir, f"cam_a_{ts}.png")
    cv2.imwrite(filename, frame)
    return filename


# Main
def main(connection_string: str) -> None:
    os.makedirs(SAVE_DIR, exist_ok=True)

    # Start MAVLink listener in a background thread
    mav_thread = threading.Thread(
        target=mavlink_listener,
        args=(connection_string,),
        daemon=True,
    )
    mav_thread.start()

    print("[Camera] Initialising DepthAI pipeline …")
    device = dai.Device()

    with dai.Pipeline(device) as pipeline:
        output_queues: dict[str, dai.DataOutputQueue] = {}
        cam_a_queue_name: str | None = None

        sockets = device.getConnectedCameras()
        for socket in sockets:
            cam = pipeline.create(dai.node.Camera).build(socket)
            queue = cam.requestFullResolutionOutput().createOutputQueue()
            name = str(socket)
            output_queues[name] = queue

            # Track which queue key corresponds to CAM_A
            if socket == CAM_A_SOCKET:
                cam_a_queue_name = name
                print(f"[Camera] CAM_A mapped to queue '{name}'")

        if cam_a_queue_name is None:
            print("[Camera] WARNING: CAM_A not found among connected cameras. "
                  "Capture will be skipped.")

        pipeline.start()
        print("[Camera] Pipeline running. Press 'c' in preview window or send "
              "MAVLink DO_DIGICAM_CONTROL to capture. Press 'q' to quit.")

        latest_cam_a_frame = None   # Always holds the most recent CAM_A frame

        while pipeline.isRunning() and not shutdown_event.is_set():

            # ---------- Grab latest frames from all cameras ----------
            for name, queue in output_queues.items():
                msg = queue.get()
                if not isinstance(msg, dai.ImgFrame):
                    continue

                frame = msg.getCvFrame()

                # Cache the CAM_A frame for saving
                if name == cam_a_queue_name:
                    latest_cam_a_frame = frame.copy()

                # Scaled preview
                preview = cv2.resize(
                    frame,
                    (0, 0),
                    fx=DISPLAY_SCALE,
                    fy=DISPLAY_SCALE,
                )
                #cv2.imshow(name, preview)

            # Capture trigger event
            # Trigger sources: MAVLink event OR keyboard 'c'
            key = getch()

            if key == 'c':
                path = save_frame(latest_cam_a_frame, SAVE_DIR)
                print(f"[Capture] Saved → {path}")

            if key == ord("q"):
                print("[Camera] Quit requested.")
                break

        shutdown_event.set()

    cv2.destroyAllWindows()
    print("[Camera] Pipeline stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DepthAI drone camera with MAVLink capture trigger")
    parser.add_argument(
        "--connect",
        default="udp:0.0.0.0:14550",
        help="MAVLink connection string (default: udp:0.0.0.0:14550)",
    )
    args = parser.parse_args()
    main(args.connect)
