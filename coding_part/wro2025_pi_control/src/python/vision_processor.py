#!/usr/bin/env python3

# vision_processor.py (Refined for Efficiency)
# Computer Vision for WRO 2025 Self-Driving Car
# Detects track lines, traffic signs, and parking spots using Pi Camera (picamera2) and OpenCV.
# Uses direct video stream (capture_array) for performance.

import time
import json
import numpy as np
import cv2
from picamera2 import Picamera2, Preview
import os
import logging

# --- Configuration ---
VISION_OUTPUT_FILE = "/tmp/vision_output.json"
STATE_FILE = "/tmp/pi_state.json"
LOG_FILE = "/tmp/vision_processor.log"

# Define color ranges in HSV (Hue, Saturation, Value) for lighting robustness
# These values MUST BE CALIBRATED on the actual competition field
COLOR_RANGES = {
    "orange": ((10, 100, 100), (25, 255, 255)),
    "blue": ((100, 100, 100), (130, 255, 255)),
    "red": ((0, 100, 100), (10, 255, 255)),      # Lower red
    "red2": ((170, 100, 100), (180, 255, 255)),  # Upper red (HSV wrap)
    "green": ((40, 100, 100), (80, 255, 255)),
    "magenta": ((140, 100, 100), (170, 255, 255))
}

# ROI parameters (x, y, width, height as fractions of image)
# Focus processing on relevant areas to reduce computation time
ROI_LINE = (0.2, 0.7, 0.6, 0.25)  # Bottom center for line following
ROI_SIGN = (0.3, 0.3, 0.4, 0.4)   # Center for sign detection
ROI_PARKING = (0.4, 0.8, 0.2, 0.15) # Bottom for parking blocks

# Minimum area for contours to be considered valid (reduces noise)
MIN_CONTOUR_AREA = 500

# --- Logging Setup ---
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO, # Use DEBUG for more detail during testing
    format='%(asctime)s - [VisionProcessor] - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# --- Helper Functions (Optimized) ---

def apply_roi(image, roi_fraction):
    """Extract Region of Interest."""
    h, w = image.shape[:2]
    x_start = int(roi_fraction[0] * w)
    y_start = int(roi_fraction[1] * h)
    roi_width = int(roi_fraction[2] * w)
    roi_height = int(roi_fraction[3] * h)
    return image[y_start:y_start+roi_height, x_start:x_start+roi_width], (x_start, y_start)

def find_largest_contour_optimized(mask):
    """Find the largest contour above the minimum area."""
    # Use RETR_EXTERNAL to only get outer contours, CHAIN_APPROX_SIMPLE to reduce points
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Find contour with max area directly
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
            return largest_contour
    return None

def detect_line_optimized(image_roi):
    """Detect track line (orange/blue) in ROI."""
    # Convert ROI to HSV
    hsv_roi = cv2.cvtColor(image_roi, cv2.COLOR_BGR2HSV)
    combined_mask = np.zeros_like(hsv_roi[:,:,0], dtype=np.uint8)

    # Combine masks for orange and blue
    for color_name in ["orange", "blue"]:
        lower, upper = COLOR_RANGES[color_name]
        mask = cv2.inRange(hsv_roi, lower, upper)
        combined_mask = cv2.bitwise_or(combined_mask, mask)

    line_contour = find_largest_contour_optimized(combined_mask)
    line_error = 0
    if line_contour is not None:
        # Calculate centroid
        M = cv2.moments(line_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            roi_center_x = image_roi.shape[1] / 2.0
            # Error relative to ROI center
            line_error = cx - roi_center_x
            logger.debug(f"Line Error (ROI): {line_error}")

    return line_error

def detect_sign_optimized(image_roi):
    """Detect traffic sign (red/green) in ROI."""
    hsv_roi = cv2.cvtColor(image_roi, cv2.COLOR_BGR2HSV)
    combined_mask = np.zeros_like(hsv_roi[:,:,0], dtype=np.uint8)

    # Combine masks for red (split) and green
    for color_name in ["red", "red2", "green"]:
        lower, upper = COLOR_RANGES[color_name]
        mask = cv2.inRange(hsv_roi, lower, upper)
        combined_mask = cv2.bitwise_or(combined_mask, mask)

    sign_contour = find_largest_contour_optimized(combined_mask)
    sign_detected = False
    sign_type = None
    sign_direction = None
    if sign_contour is not None:
        # Determine sign type based on which mask contributed most
        red_mask1 = cv2.inRange(hsv_roi, COLOR_RANGES["red"][0], COLOR_RANGES["red"][1])
        red_mask2 = cv2.inRange(hsv_roi, COLOR_RANGES["red2"][0], COLOR_RANGES["red2"][1])
        red_mask_combined = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv_roi, COLOR_RANGES["green"][0], COLOR_RANGES["green"][1])

        # Check overlap with contours using drawContours and bitwise_and
        contour_mask = np.zeros_like(hsv_roi[:,:,0], dtype=np.uint8)
        cv2.drawContours(contour_mask, [sign_contour], -1, 255, -1)

        red_overlap = cv2.countNonZero(cv2.bitwise_and(red_mask_combined, contour_mask))
        green_overlap = cv2.countNonZero(cv2.bitwise_and(green_mask, contour_mask))

        if red_overlap > MIN_CONTOUR_AREA:
            sign_type = "red"
        elif green_overlap > MIN_CONTOUR_AREA:
            sign_type = "green"

        if sign_type:
            sign_detected = True
            # Determine direction based on centroid
            M = cv2.moments(sign_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                roi_center_x = image_roi.shape[1] / 2.0
                sign_direction = "left" if cx < roi_center_x else "right"
            logger.info(f"Sign: {sign_type}, Dir: {sign_direction}")

    return sign_detected, sign_type, sign_direction

def detect_parking_optimized(image_roi):
    """Detect parking blocks (magenta) in ROI."""
    hsv_roi = cv2.cvtColor(image_roi, cv2.COLOR_BGR2HSV)
    magenta_mask = cv2.inRange(hsv_roi, COLOR_RANGES["magenta"][0], COLOR_RANGES["magenta"][1])
    parking_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area
    valid_contours = [c for c in parking_contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]

    parking_detected = len(valid_contours) >= 2
    parking_aligned = False
    parking_distance = -1 # Difficult to estimate without depth

    if parking_detected:
        # Sort by area, get the two largest
        sorted_contours = sorted(valid_contours, key=cv2.contourArea, reverse=True)[:2]
        if len(sorted_contours) == 2:
            # Get centroids
            M1, M2 = cv2.moments(sorted_contours[0]), cv2.moments(sorted_contours[1])
            if M1["m00"] != 0 and M2["m00"] != 0:
                cx1, cy1 = int(M1["m10"]/M1["m00"]), int(M1["m01"]/M1["m00"])
                cx2, cy2 = int(M2["m10"]/M2["m00"]), int(M2["m01"]/M2["m00"])

                # Calculate angle between blocks (ideally vertical for parking)
                angle_rad = np.arctan2(abs(cy2 - cy1), abs(cx2 - cx1))
                angle_deg = np.degrees(angle_rad)
                parking_aligned = abs(90 - angle_deg) < 10 # Tolerance in degrees
                logger.info(f"Parking: Aligned={parking_aligned}, Angle={angle_deg:.2f}")

    return parking_detected, parking_aligned, parking_distance

def write_vision_output(output_data):
    """Write results to shared JSON file."""
    try:
        with open(VISION_OUTPUT_FILE, 'w') as f:
            json.dump(output_data, f)
        # logger.debug(f"Vision output: {output_data}") # Comment out for performance
    except IOError as e:
        logger.error(f"Error writing vision output: {e}")

# --- Main Vision Processing Loop ---
def main():
    logger.info("Starting Optimized Vision Processor...")

    # --- Initialize Pi Camera (Optimized config) ---
    try:
        picam2 = Picamera2()
        # Use a lower resolution for faster FPS on Pi Zero 2W
        # Consider 'QVGA' (320x240) or 'VGA' (640x480) depending on performance needs
        config = picam2.create_video_configuration(main={"size": (320, 240), "format": "RGB888"})
        # config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}) # Higher res, slower
        picam2.configure(config)
        picam2.start()
        logger.info("Pi Camera (320x240) initialized and started.")
    except Exception as e:
        logger.critical(f"Failed to initialize Pi Camera: {e}")
        return

    # --- Main Loop ---
    last_time = time.time()
    fps_counter = 0
    fps_interval = 1.0
    fps = 0

    try:
        while True:
            start_time = time.time()

            # --- 1. Capture Image (Efficient Stream Read) ---
            im = picam2.capture_array() # Captures RGB array directly

            # --- 2. Read State (Challenge Type) ---
            challenge_type = "OPEN"
            try:
                with open(STATE_FILE, 'r') as f:
                    state_data = json.load(f)
                    challenge_type = state_data.get("challenge_type", "OPEN")
            except (FileNotFoundError, json.JSONDecodeError) as e:
                logger.warning(f"Could not read state file: {e}. Using default: {challenge_type}")

            # --- 3. Process Image Based on Challenge Type ---
            output_data = {
                "line_error": 0,
                "sign_detected": False,
                "sign_type": None,
                "sign_direction": None,
                "parking_detected": False,
                "parking_aligned": False,
                "parking_distance": -1
            }

            # Prioritize based on challenge
            if challenge_type in ["OPEN", "OBSTACLE"]:
                line_roi, _ = apply_roi(im, ROI_LINE)
                output_data["line_error"] = detect_line_optimized(line_roi)

            if challenge_type == "OBSTACLE":
                # Check for signs first (higher priority during approach)
                sign_roi, _ = apply_roi(im, ROI_SIGN)
                sd, st, sdir = detect_sign_optimized(sign_roi)
                output_data["sign_detected"] = sd
                output_data["sign_type"] = st
                output_data["sign_direction"] = sdir

                # Check for parking (relevant near end)
                # Only run parking detection if sign detection didn't find anything
                # or if we are specifically in the parking approach phase (state check)
                if not sd: # Only if no sign is currently being processed
                    parking_roi, _ = apply_roi(im, ROI_PARKING)
                    pd, pa, pdist = detect_parking_optimized(parking_roi)
                    output_data["parking_detected"] = pd
                    output_data["parking_aligned"] = pa
                    output_data["parking_distance"] = pdist

            # --- 4. Write Output Data (Non-blocking write is ideal, but JSON is usually fast enough) ---
            write_vision_output(output_data)

            # --- 5. Calculate and Log FPS ---
            fps_counter += 1
            elapsed_time = time.time() - last_time
            if elapsed_time >= fps_interval:
                fps = fps_counter / elapsed_time
                logger.info(f"Vision FPS: {fps:.2f}")
                fps_counter = 0
                last_time = time.time()

            # Optional: Control loop frequency if processing is very fast
            # desired_fps = 15
            # loop_time = time.time() - start_time
            # sleep_time = max(1.0/desired_fps - loop_time, 0)
            # if sleep_time > 0:
            #     time.sleep(sleep_time)

    except KeyboardInterrupt:
        logger.info("Vision Processor interrupted.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        logger.info("Stopping Pi Camera...")
        picam2.stop()
        logger.info("Vision Processor stopped.")

if __name__ == "__main__":
    main()