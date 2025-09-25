# vision/image_processor.py (Performance Refinement)
"""Processes image frames for signs and edges, producing VisionSnapshots. Optimized."""

import cv2
import numpy as np
import threading
import time
import logging
from typing import Optional

import config.vehicle_config as cfg
from vision.frame_buffer import FrameBuffer
from vision.vision_snapshot import VisionSnapshot, DetectedSign, DetectedEdgeLine
from vision.color_detector import ColorDetector
from vision.edge_detector import EdgeDetector

logger = logging.getLogger(__name__)

class ImageProcessor:
    """
    Pulls frames, decodes, detects features, produces VisionSnapshots.
    Runs in a separate thread. Optimized for Pi Zero 2W.
    """

    def __init__(self, frame_buffer: FrameBuffer):
        if not frame_buffer:
            raise ValueError("ImageProcessor requires a valid FrameBuffer.")
        self.frame_buffer = frame_buffer

        self._is_initialized = False
        self._is_processing = False
        self._stop_processing = threading.Event()
        self._processing_thread: Optional[threading.Thread] = None

        self._latest_snapshot = VisionSnapshot()
        self._snapshot_lock = threading.Lock()
        self._new_snapshot_available = False

        # Processing components - initialize once
        self.color_detector = ColorDetector()
        self.edge_detector = EdgeDetector()

        # --- PERFORMANCE: Pre-define ROI ---
        # These will be scaled based on the actual frame size in processFrame
        self._base_roi_x = 0
        self._base_roi_y = int(cfg.CAMERA_HEIGHT * 0.3) # Exclude top 30%
        self._base_roi_width = cfg.CAMERA_WIDTH
        self._base_roi_height = int(cfg.CAMERA_HEIGHT * 0.5) # Use middle 50%
        self._current_roi = (self._base_roi_x, self._base_roi_y, self._base_roi_width, self._base_roi_height)

        logger.info("ImageProcessor initialized.")

    def initialize(self) -> bool:
        """Perform initialization."""
        # Potentially load tuned parameters here
        self._is_initialized = True
        logger.debug("ImageProcessor: Initialized.")
        return True

    def start_processing(self) -> bool:
        """Starts the image processing thread."""
        if not self._is_initialized:
            logger.error("ImageProcessor: Cannot start, not initialized.")
            return False
        if self._is_processing:
            logger.warning("ImageProcessor: Processing already running.")
            return False

        self._stop_processing.clear()
        # Use a specific name for the thread for debugging
        self._processing_thread = threading.Thread(target=self._processing_loop, daemon=True, name="ImageProcessingThread")
        self._processing_thread.start()
        self._is_processing = True
        logger.info("ImageProcessor: Processing started.")
        return True

    def stop_processing(self):
        """Stops the image processing thread."""
        if not self._is_processing:
            return

        logger.info("ImageProcessor: Stopping processing...")
        self._stop_processing.set()

        if self._processing_thread and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=2.0)
            if self._processing_thread.is_alive():
                logger.warning("ImageProcessor: Processing thread did not stop gracefully.")

        self._is_processing = False
        logger.info("ImageProcessor: Processing stopped.")

    def is_running(self) -> bool:
        return self._is_processing

    def get_latest_snapshot(self) -> Optional[VisionSnapshot]:
        """Get the latest VisionSnapshot."""
        with self._snapshot_lock:
            if self._new_snapshot_available:
                self._new_snapshot_available = False
                # Return a copy to prevent race conditions if processing updates it
                # Deep copy of lists is important
                return VisionSnapshot(
                    timestamp=self._latest_snapshot.timestamp,
                    left_signs=self._latest_snapshot.left_signs.copy(),
                    right_signs=self._latest_snapshot.right_signs.copy(),
                    left_edge_lines=[l for l in self._latest_snapshot.left_edge_lines], # Shallow copy of list of immutable tuples
                    right_edge_lines=[l for l in self._latest_snapshot.right_edge_lines],
                    avg_left_edge_x=self._latest_snapshot.avg_left_edge_x,
                    avg_right_edge_x=self._latest_snapshot.avg_right_edge_x,
                    left_edge_std=self._latest_snapshot.left_edge_std,
                    right_edge_std=self._latest_snapshot.right_edge_std,
                    frame_id=self._latest_snapshot.frame_id,
                    processing_time_ms=self._latest_snapshot.processing_time_ms,
                    signs_searched=self._latest_snapshot.signs_searched,
                    edges_searched=self._latest_snapshot.edges_searched
                )
            else:
                return None

    def _processing_loop(self):
        """Optimized background processing loop."""
        logger.debug("ImageProcessor: Processing loop started.")
        frame_counter = 0

        try:
            while not self._stop_processing.is_set():
                logger.debug("ImageProcessor: Attempting to get frame from buffer...")
                # Get frame with a short timeout to allow checking stop flag
                jpeg_data = self.frame_buffer.get(timeout=0.05) # 50ms timeout
                if jpeg_data is None:
                    logger.debug("ImageProcessor: Frame buffer get timed out.")
                    continue # Timeout or no frame, loop again
                logger.debug("ImageProcessor: Got frame ({len(jpeg_data)} bytes) from buffer.")
                frame_counter += 1
                start_time = time.perf_counter() # Use perf_counter for better precision
                logger.debug(f"ImageProcessor: Got frame {frame_counter} ({len(jpeg_data)} bytes) from buffer.")

                # --- PERFORMANCE: Efficient JPEG Decoding ---
                # np.frombuffer creates a view, cv2.imdecode decodes in place
                np_arr = np.frombuffer(jpeg_data, np.uint8)
                # Use IMREAD_COLOR for BGR (OpenCV default), or IMREAD_GRAYSCALE if only gray needed later
                frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame_bgr is None:
                    logger.warning(f"ImageProcessor: Failed to decode frame {frame_counter}.")
                    continue

                # --- PERFORMANCE: Dynamic ROI based on frame size ---
                h, w = frame_bgr.shape[:2]
                logger.debug(f"ImageProcessor: Frame {frame_counter} decoded ({w}x{h}). Applying ROI: {self._current_roi}")
                # Scale ROI if frame size differs from config (though it shouldn't if rpicam-vid is set correctly)
                scale_x = w / cfg.CAMERA_WIDTH if cfg.CAMERA_WIDTH > 0 else 1
                scale_y = h / cfg.CAMERA_HEIGHT if cfg.CAMERA_HEIGHT > 0 else 1
                roi_x = int(self._base_roi_x * scale_x)
                roi_y = int(self._base_roi_y * scale_y)
                roi_w = int(self._base_roi_width * scale_x)
                roi_h = int(self._base_roi_height * scale_y)
                # Ensure ROI is within bounds
                roi_x = max(0, roi_x)
                roi_y = max(0, roi_y)
                roi_w = min(w - roi_x, roi_w)
                roi_h = min(h - roi_y, roi_h)
                self._current_roi = (roi_x, roi_y, roi_w, roi_h)

                # --- PERFORMANCE: Apply ROI ---
                if roi_w > 0 and roi_h > 0:
                    roi_frame_bgr = frame_bgr[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
                else:
                    logger.warning(f"ImageProcessor: Invalid ROI for frame {frame_counter}.")
                    continue

                # --- PERFORMANCE: Convert Color Spaces ONCE ---
                # Convert ROI BGR to HSV for color detection
                hsv_roi = cv2.cvtColor(roi_frame_bgr, cv2.COLOR_BGR2HSV)
                # Convert ROI BGR to Grayscale for edge detection
                gray_roi = cv2.cvtColor(roi_frame_bgr, cv2.COLOR_BGR2GRAY)

                # --- Create and Populate Snapshot ---
                snapshot = VisionSnapshot()
                snapshot.frame_id = frame_counter
                snapshot.timestamp = time.time() # Or start_time if you prefer

                # --- PERFORMANCE: Parallel-like Processing ---
                # While Python threads are limited by GIL for CPU-bound tasks,
                # I/O and calls to C-extensions (like OpenCV) can run in parallel.
                # The detection functions are mostly C++ OpenCV calls, so threading can help
                # with I/O wait or context switches.

                # Detect Signs (Color)
                snapshot.signs_searched = True
                # These calls are mostly in C++, so GIL is released during computation
                red_found, red_centroid, red_conf, red_bbox, red_area = self.color_detector.detect_red_sign(hsv_roi)
                green_found, green_centroid, green_conf, green_bbox, green_area = self.color_detector.detect_green_sign(hsv_roi)

                if red_found:
                    # Adjust centroid and bbox coordinates relative to full frame
                    adj_centroid = (red_centroid[0] + roi_x, red_centroid[1] + roi_y) if red_centroid else (0, 0)
                    adj_bbox = (red_bbox[0] + roi_x, red_bbox[1] + roi_y, red_bbox[2], red_bbox[3])
                    sign = DetectedSign(
                        sign_type=DetectedSign.SignType.RIGHT,
                        bbox=adj_bbox,
                        confidence=red_conf,
                        centroid=adj_centroid,
                        area=red_area
                    )
                    snapshot.right_signs.append(sign)

                if green_found:
                    adj_centroid = (green_centroid[0] + roi_x, green_centroid[1] + roi_y) if green_centroid else (0, 0)
                    adj_bbox = (green_bbox[0] + roi_x, green_bbox[1] + roi_y, green_bbox[2], green_bbox[3])
                    sign = DetectedSign(
                        sign_type=DetectedSign.SignType.LEFT,
                        bbox=adj_bbox,
                        confidence=green_conf,
                        centroid=adj_centroid,
                        area=green_area
                    )
                    snapshot.left_signs.append(sign)

                # Detect Edges (Canny + Hough)
                snapshot.edges_searched = True
                # Calculate average edge positions and get lines
                avg_left_x, left_std, left_lines = self.edge_detector.calculate_average_edge_x(gray_roi, 'left')
                avg_right_x, right_std, right_lines = self.edge_detector.calculate_average_edge_x(gray_roi, 'right')

                # Adjust average edge X coordinates relative to full frame
                snapshot.avg_left_edge_x = avg_left_x + roi_x if avg_left_x >= 0 else avg_left_x
                snapshot.avg_right_edge_x = avg_right_x + roi_x if avg_right_x >= 0 else avg_right_x
                snapshot.left_edge_std = left_std
                snapshot.right_edge_std = right_std

                # Populate detailed line information with adjusted coordinates
                for x1, y1, x2, y2, length, angle, strength in left_lines:
                    line_full = (x1 + roi_x, y1 + roi_y, x2 + roi_x, y2 + roi_y)
                    edge_line = DetectedEdgeLine(line=line_full, length=length, angle=angle, strength=strength)
                    snapshot.left_edge_lines.append(edge_line)

                for x1, y1, x2, y2, length, angle, strength in right_lines:
                    line_full = (x1 + roi_x, y1 + roi_y, x2 + roi_x, y2 + roi_y)
                    edge_line = DetectedEdgeLine(line=line_full, length=length, angle=angle, strength=strength)
                    snapshot.right_edge_lines.append(edge_line)

                # --- Finalize Snapshot ---
                end_time = time.perf_counter()
                snapshot.processing_time_ms = (end_time - start_time) * 1000
                logger.debug(f"ImageProcessor: Processed frame {frame_counter} in {snapshot.processing_time_ms:.2f}ms")
                # --- Make Snapshot Available ---
                with self._snapshot_lock:
                    self._latest_snapshot = snapshot
                    self._new_snapshot_available = True

                # --- PERFORMANCE: Log FPS/Processing Time ---
                if frame_counter % 30 == 0: # Log every 30 frames
                    logger.debug(f"ImageProcessor: Frame {frame_counter} processed in {snapshot.processing_time_ms:.2f}ms.")

        except Exception as e:
            logger.error(f"ImageProcessor: Error in processing loop: {e}", exc_info=True)
        finally:
            logger.debug("ImageProcessor: Processing loop finished.")
