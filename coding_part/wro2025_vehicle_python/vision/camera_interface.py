# vision/camera_interface.py
"""Interface to the Raspberry Pi camera using OpenCV + GStreamer.

Captures frames and pushes JPEG-encoded bytes to FrameBuffer for downstream
processing. Designed for Raspberry Pi Zero 2 W performance constraints.
"""

import logging
import time
from threading import Thread
from typing import Optional

import cv2

from vision.frame_buffer import FrameBuffer
import config.vehicle_config as cfg

logger = logging.getLogger(__name__)


class CameraInterface:
    """Capture frames from the Pi camera and enqueue JPEG bytes."""

    def __init__(self, frame_buffer: FrameBuffer):
        self.frame_buffer = frame_buffer
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_capturing = False
        self.capture_thread: Optional[Thread] = None

    def start_capture(self) -> bool:
        if self.is_capturing:
            logger.warning("CameraInterface: Capture already running.")
            return False

        # Prefer a low-res, low-fps pipeline for the Pi Zero 2 W
        width = max(1, int(getattr(cfg, 'CAMERA_WIDTH', 320)))
        height = max(1, int(getattr(cfg, 'CAMERA_HEIGHT', 240)))
        fps = max(1, int(getattr(cfg, 'CAMERA_FPS', 10)))

        # Use camera_auto_detect=1 from firmware config; do not force camera-name
        # Low-latency pipeline: add leaky queue and appsink drop to avoid backlog on Pi Zero 2 W
        pipeline = (
            f"libcamerasrc ! video/x-raw,width={width},height={height},framerate={fps}/1 ! "
            "videoconvert ! video/x-raw,format=RGB ! "
            "queue leaky=downstream max-size-buffers=1 ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        for attempt in range(3):
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.is_capturing = True
                self.capture_thread = Thread(target=self._capture_loop, daemon=True)
                self.capture_thread.start()
                logger.info("CameraInterface: Capture started successfully.")
                return True
            logger.warning(f"CameraInterface: Open failed (attempt {attempt + 1}/3). Retrying...")
            time.sleep(1)

        logger.error("CameraInterface: Capture failed after retries.")
        return False

    def stop_capture(self):
        if not self.is_capturing:
            return
        logger.info("CameraInterface: Stopping capture...")
        self.is_capturing = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
            self.cap = None
        self.frame_buffer.clear()
        logger.info("CameraInterface: Capture stopped.")

    def is_running(self) -> bool:
        return self.is_capturing

    def _capture_loop(self):
        consecutive_failures = 0
        while self.is_capturing and self.cap:
            ret, frame = self.cap.read()
            if not ret or frame is None or frame.size == 0:
                logger.warning("CameraInterface: Failed to capture frame.")
                consecutive_failures += 1
                # Backoff a little on repeated failures to reduce CPU load
                time.sleep(min(0.2, 0.02 * consecutive_failures))
                # If too many consecutive failures, try to reinitialize the capture
                if consecutive_failures >= 25:
                    logger.error("CameraInterface: Too many capture failures, attempting to reinitialize camera.")
                    try:
                        if self.cap:
                            self.cap.release()
                        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                        consecutive_failures = 0
                        if not self.cap.isOpened():
                            logger.error("CameraInterface: Reinitialize failed, stopping capture.")
                            self.is_capturing = False
                            break
                    except Exception as e:
                        logger.error(f"CameraInterface: Exception during reinitialize: {e}")
                        self.is_capturing = False
                        break
                continue

            # Encode as JPEG to reduce size across threads
            success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if success:
                self.frame_buffer.put(encoded.tobytes())
                logger.debug("CameraInterface: JPEG frame enqueued.")
            else:
                logger.debug("CameraInterface: JPEG encode failed, skipping frame.")

            consecutive_failures = 0
            time.sleep(0.08)  # target ~10-12 FPS


if __name__ == "__main__":
    import utils.logger as logutil
    logutil.setup_logging()
    print("CameraInterface ready.")