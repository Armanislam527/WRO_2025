import logging
import time
import cv2
from threading import Thread
from vision.frame_buffer import FrameBuffer
import utils.logger as logger

class CameraInterface:
    def __init__(self, frame_buffer: FrameBuffer):
        self.frame_buffer = frame_buffer
        self.cap = None
        self.is_capturing = False
        self.capture_thread = None
        self.logger = logging.getLogger(__name__)

    def start_capture(self) -> bool:
        """Start capturing frames from the camera and add them to the frame buffer."""
        if self.is_capturing:
            self.logger.warning("Capture already running.")
            return False

        for attempt in range(3):  # Retry 3 times
            pipeline = (
                "libcamerasrc camera-name=/base/soc/i2c0mux/i2c@1/ov5647@36 ! "
                "video/x-raw,width=320,height=240,framerate=10/1 ! "
                "videoconvert ! appsink"
            )
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.is_capturing = True
                self.capture_thread = Thread(target=self._capture_loop)
                self.capture_thread.start()
                self.logger.info("CameraInterface: Capture started successfully.")
                return True
            self.logger.warning(f"Camera open failed, attempt {attempt + 1}/3.")
            time.sleep(1)
        
        self.logger.error("Camera capture failed after retries.")
        return False

    def stop_capture(self):
        """Stop capturing frames and clean up."""
        if not self.is_capturing:
            return
        self.logger.info("CameraInterface: Stopping capture...")
        self.is_capturing = False
        if self.capture_thread:
            self.capture_thread.join()
        if self.cap:
            self.cap.release()
        self.logger.info("CameraInterface: Capture stopped.")

    def _capture_loop(self):
        """Capture frames in a loop and add to frame buffer."""
        while self.is_capturing:
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size != 0:
                self.frame_buffer.add_frame(frame)
                self.logger.debug("Frame added to buffer.")
            else:
                self.logger.warning("Failed to capture frame.")
            time.sleep(0.1)  # ~10 FPS