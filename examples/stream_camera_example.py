"""
See section 9.1.2 in the Picamera2 manual: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
"""
import time
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import PyavOutput

picam2 = Picamera2()
main = {'size': (1920, 1080), 'format': 'YUV420'}
controls = {'FrameRate': 30}
config = picam2.create_video_configuration(main, controls=controls)
picam2.configure(config)
encoder = H264Encoder(bitrate=10000000)
output = PyavOutput("rtsp://127.0.0.1:8554/cam", format="rtsp")
print("Camera starting")
picam2.start_recording(encoder, output)
try:
    while True:
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Camera stopping")