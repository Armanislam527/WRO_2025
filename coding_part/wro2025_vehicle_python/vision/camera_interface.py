# vision/camera_interface.py
"""Interface to the Raspberry Pi camera using rpicam-vid subprocess."""

import subprocess
import threading
import time
import logging
import os
from typing import Optional

# Import our modules
import config.vehicle_config as cfg
if __debug__: # Only import if debug mode is enabled (or always for type hints)
    from vision.frame_buffer import FrameBuffer

logger = logging.getLogger(__name__)

class CameraInterface:
    """
    Manages the rpicam-vid process to capture JPEG frames
    and puts them into a FrameBuffer.
    Optimized for performance on Pi Zero 2W.
    """

    def __init__(self, frame_buffer):
        self.frame_buffer = frame_buffer
        self.cap = None
        self.is_capturing = False
        self.capture_thread = None
    def start_capture(self):
        for _ in range(3):  # Retry 3 times
            pipeline = "libcamerasrc camera-name=/base/soc/i2c0mux/i2c@1/ov5647@36 ! video/x-raw,width=320,height=240,framerate=10/1 ! videoconvert ! appsink"
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.is_capturing = True
                self.capture_thread = Thread(target=self._capture_loop)
                self.capture_thread.start()
                logger.info("CameraInterface: Capture started successfully.")
                return True
            logger.warning("Camera open failed, retrying...")
            time.sleep(1)
        logger.error("Camera capture failed after retries.")
        return False
    def stop_capture(self):
        """Stops the capture process and thread."""
        if not self._is_capturing:
            return

        logger.info("CameraInterface: Stopping capture...")
        self._stop_capture.set()

        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
            if self._capture_thread.is_alive():
                logger.warning("CameraInterface: Capture thread did not stop gracefully.")

        if self._process:
            self._terminate_process()

        self._is_capturing = False
        self.frame_buffer.clear()
        logger.info("CameraInterface: Capture stopped.")

    def _terminate_process(self):
        """Terminate the rpicam-vid process cleanly."""
        try:
            os.killpg(os.getpgid(self._process.pid), 15) # SIGTERM
            self._process.wait(timeout=1.5)
        except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
            try:
                os.killpg(os.getpgid(self._process.pid), 9) # SIGKILL
            except (ProcessLookupError, PermissionError):
                pass
        except Exception as e:
            logger.error(f"CameraInterface: Error terminating process: {e}")
        finally:
            self._process = None

    def is_running(self) -> bool:
        return self._is_capturing

    def _capture_loop(self):
      while self.is_capturing:
          ret, frame = self.cap.read()
          if ret and frame is not None and not frame.size == 0:
              self.frame_buffer.add_frame(frame)
              logger.debug("Frame added to buffer.")
          time.sleep(0.1)  # ~10 FPS
      self.cap.release()
# Example usage (if run as script - needs mocks)
if __name__ == "__main__":
    import utils.logger
    utils.logger.setup_logging()
    
    # This would typically be tested with actual FrameBuffer and main application flow
    print("CameraInterface class defined.")