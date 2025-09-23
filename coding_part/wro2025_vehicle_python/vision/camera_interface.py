# vision/camera_interface.py (Performance Refinement)
"""Interface to the Raspberry Pi camera using rpicam-vid subprocess."""

import subprocess
import threading
import time
import logging
import os
from typing import Optional

import config.vehicle_config as cfg
if __debug__:
    from vision.frame_buffer import FrameBuffer

logger = logging.getLogger(__name__)

class CameraInterface:
    """
    Manages the rpicam-vid process to capture JPEG frames
    and puts them into a FrameBuffer.
    Optimized for performance on Pi Zero 2W.
    """

    def __init__(self, frame_buffer: 'FrameBuffer'):
        if not frame_buffer:
            raise ValueError("CameraInterface requires a valid FrameBuffer.")
        self.frame_buffer = frame_buffer

        self._process: Optional[subprocess.Popen] = None
        self._capture_thread: Optional[threading.Thread] = None
        self._stop_capture = threading.Event()
        self._is_capturing = False

        # Camera parameters - can be tuned for performance vs. quality
        # Lower resolution and FPS can significantly improve processing speed
        self.width = cfg.CAMERA_WIDTH # e.g., 320
        self.height = cfg.CAMERA_HEIGHT # e.g., 240
        self.fps = cfg.CAMERA_FPS # e.g., 20 or 15
        self.device = cfg.CAMERA_DEVICE

        logger.info(f"CameraInterface initialized (W: {self.width}, H: {self.height}, FPS: {self.fps}).")

    def start_capture(self) -> bool:
        """
        Starts the rpicam-vid subprocess and the capture thread.
        """
        if self._is_capturing:
            logger.warning("CameraInterface: Capture already running.")
            return False

        try:
            # --- OPTIMIZED rpicam-vid COMMAND ---
            # -t 0: Run indefinitely
            # --inline: Embed JPEG headers for easier decoding
            # --nopreview: No on-screen preview to save resources
            # --timeout 0: Alias for -t 0
            # --width, --height, --framerate: Set capture parameters
            # --awbgainred, --awbgainblue: Fixed AWB can be faster than auto
            # --brightness, --contrast, --saturation: Tune for better initial image
            # --roi: Can specify a digital zoom/ROI at capture time (e.g., 0,0.3,1,0.4)
            cmd = [
                "rpicam-vid", "-t", "0", "--inline", "--nopreview",
                "-o", "-", "--width", str(self.width), "--height", str(self.height),
                "--framerate", str(self.fps)
                # Add more tuning options if needed:
                # "--awbgains", "1.2,1.0", # Example fixed AWB
                # "--roi", "0,0.3,1,0.4" # Example ROI crop at capture
            ]
            if self.device:
                 cmd.extend(["--camera", self.device])

            logger.info(f"CameraInterface: Starting capture with command: {' '.join(cmd)}")
            
            # Use preexec_fn for clean process group management
            # bufsize=0 for unbuffered, or a small positive value. 1 is line-buffered (text).
            # For binary streams like this, unbuffered or a small buffer might be better.
            # Let's use default buffering for now, or a small buffer like 8192.
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, # Capture stderr
                # bufsize=8192, # Use a small buffer
                preexec_fn=os.setsid
            )
            
            self._stop_capture.clear()
            # Start capture thread with high priority if possible (not easily done in Python standard lib)
            # Daemon=True ensures it doesn't prevent main program exit
            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True, name="CameraCaptureThread")
            self._capture_thread.start()
            self._is_capturing = True
            logger.info("CameraInterface: Capture started successfully.")
            return True

        except Exception as e:
            logger.error(f"CameraInterface: Failed to start capture: {e}", exc_info=True)
            self._is_capturing = False
            if self._process:
                self._terminate_process()
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
        """Optimized background thread for reading JPEG data."""
        logger.debug("CameraInterface: Capture loop started.")
        
        if not self._process or not self._process.stdout:
            logger.error("CameraInterface: Process or stdout is None.")
            return

        jpeg_data = bytearray()
        previous_byte = 0
        SOI_MARKER = 0xD8
        EOI_MARKER = 0xD9
        state = "FIND_SOI"

        try:
            while not self._stop_capture.is_set():
                # Read in larger chunks for efficiency on Pi
                chunk = self._process.stdout.read(8192) # Increased chunk size
                if not chunk:
                    logger.debug("CameraInterface: EOF reached on stdout.")
                    break

                for byte_val in chunk:
                    if state == "FIND_SOI":
                        if previous_byte == 0xFF and byte_val == SOI_MARKER:
                            jpeg_data.clear()
                            jpeg_data.extend([0xFF, SOI_MARKER])
                            state = "ACCUMULATE"
                        previous_byte = byte_val

                    elif state == "ACCUMULATE":
                        jpeg_data.append(byte_val)
                        if previous_byte == 0xFF and byte_val == EOI_MARKER:
                            # Found complete JPEG
                            # Put JPEG bytes directly into buffer
                            self.frame_buffer.put(bytes(jpeg_data)) # bytes() copies the bytearray
                            # Consider: self.frame_buffer.put(jpeg_data) if FrameBuffer can handle bytearray
                            # and you want to avoid the copy. But bytearray is mutable.
                            state = "FIND_SOI"
                        elif len(jpeg_data) > 5 * 1024 * 1024: # 5MB sanity limit
                            logger.warning("CameraInterface: JPEG too large, discarding.")
                            state = "FIND_SOI"
                            jpeg_data.clear()
                        previous_byte = byte_val

                # Minimal sleep to prevent excessive CPU spin if read is very fast
                # time.sleep(0.0001) # 0.1ms - often not needed with blocking read

        except Exception as e:
            logger.error(f"CameraInterface: Error in capture loop: {e}", exc_info=True)
        finally:
            logger.debug("CameraInterface: Capture loop finished.")
