import cv2
import depthai as dai
import argparse
import time
import signal
from pathlib import Path
from datetime import datetime

time_stamp = datetime.now().strftime("%d-%m-%y_%H-%M")
output_dir = Path(__file__).parent.parent / "recordings"

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default=time_stamp, help="Output file name (without extension)")
args = parser.parse_args()

output_path = output_dir / f"{args.output}.mp4"

with dai.Pipeline() as pipeline:

    def signal_handler(sig, frame):
        print("Interrupted, stopping the pipeline")
        pipeline.stop()
    signal.signal(signal.SIGINT, signal_handler)

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    
    videoQueue = cam.requestOutput((640,400), enableUndistortion=True).createOutputQueue()

    videoEncoder = pipeline.create(dai.node.VideoEncoder).build(cam.requestOutput((1280,720), dai.ImgFrame.Type.NV12, enableUndistortion=True))
    videoEncoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)

    record = pipeline.create(dai.node.RecordVideo)
    record.setRecordVideoFile(output_path)

    videoEncoder.out.link(record.input)

    pipeline.start()
    print("Recording video. Press Ctrl+C to stop.")

    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break