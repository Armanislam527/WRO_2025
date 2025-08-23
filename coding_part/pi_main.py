#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Main control script for the WRO 2025 Future Engineers Self-Driving Car (Raspberry Pi Side).
Handles high-level logic, computer vision, sensor fusion, path planning, and communication with the Arduino Nano.
"""

import serial
import time
import logging
from collections import deque
# Import for USB Camera (e.g., using OpenCV)
# If you switch to Pi Camera CSI, you might use `picamera2` instead.
import cv2
import numpy as np

# --- Configuration ---
# Serial Port Configuration (Check on your Pi/Alpine which port is correct)
# Common ones: '/dev/ttyAMA0', '/dev/serial0', '/dev/ttyS0'
SERIAL_PORT = '/dev/serial0' # Example - Adjust based on your setup
BAUD_RATE = 115200

# Camera Configuration
CAMERA_INDEX = 0  # Usually 0 for the first USB camera
FRAME_WIDTH = 320 # Reduce resolution for faster processing if needed
FRAME_HEIGHT = 240
FRAME_RATE = 30 # Target FPS (actual might be lower)

# State Machine States
STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_PARKING = "PARKING" # Specific for Obstacle Challenge
STATE_STOPPED = "STOPPED"

# Lap Counting Configuration (Using IMU Gyro for turns)
REQUIRED_TURNS_PER_LAP = 4
REQUIRED_LAPS = 3
TURN_THRESHOLD_DEGREES = 70 # Minimum angle to count as a significant turn
TURN_RESET_THRESHOLD_DEGREES = 20 # Angle below which turn detection resets
GYRO_INTEGRAL_WINDOW_SIZE = 5 # Number of recent gyro readings to integrate for a smoother turn estimate

# Parking Configuration (Example placeholders)
PARKING_LOT_DETECTED_THRESHOLD = 0.8 # Confidence threshold for parking lot detection
PARALLEL_PARK_STEPS = 5 # Number of steps in the parking sequence

# Control Parameters (Example - these will need tuning)
BASE_SPEED = 70 # Base motor speed (0-255, or whatever range your Nano expects)
MAX_SPEED = 100
MIN_SPEED = 30
STEERING_P_GAIN = 0.8 # Proportional gain for steering based on line position error
STEERING_D_GAIN = 0.3 # Derivative gain
# --- End Configuration ---

# --- Global Variables ---
state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE) # For smoothing gyro integration
parking_sequence_step = 0
start_time = None # To track elapsed time if needed

# Sensor Data Storage (Updated by data from Nano)
# IR: FL, FR, R45, L45
ir_sensor_states = [1, 1, 1, 1] # Assume initially see line (active low sensors often)
# US: Front, Rear (in mm)
ultrasonic_distances_mm = [9999, 9999] # Assume far away initially
# IMU: Raw values from Nano (Accel X,Y,Z, Gyro X,Y,Z)
imu_data_raw = [0, 0, 0, 0, 0, 0]

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Communication with Nano ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) # 1 second timeout for reading
    logger.info(f"Connected to Arduino Nano on {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    logger.error(f"Failed to connect to serial port {SERIAL_PORT}: {e}")
    exit(1)

# --- Camera Setup ---
cap = None
def initialize_camera():
    global cap
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        logger.error("Error: Could not open USB camera.")
        return False
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
    logger.info(f"Camera initialized (Index: {CAMERA_INDEX}, Res: {FRAME_WIDTH}x{FRAME_HEIGHT})")
    return True

# --- Helper Functions for Communication ---
def send_command_to_nano(command_str):
    """Sends a command string to the Arduino Nano via serial."""
    try:
        ser.write((command_str + '\n').encode('utf-8'))
        logger.debug(f"Sent to Nano: {command_str}")
    except Exception as e:
        logger.error(f"Error sending command to Nano: {e}")

def parse_sensor_data(data_str):
    """Parses sensor data string received from Nano."""
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw
    parts = data_str.split(',')
    if len(parts) >= 13 and parts[0] == "SENSORS":
        try:
            # IR Sensors (D6, D7, D8, D9)
            ir_sensor_states[0] = int(parts[1]) # FL
            ir_sensor_states[1] = int(parts[2]) # FR
            ir_sensor_states[2] = int(parts[3]) # R45
            ir_sensor_states[3] = int(parts[4]) # L45
            # Ultrasonic Sensors (Front, Rear)
            ultrasonic_distances_mm[0] = int(parts[5]) # Front
            ultrasonic_distances_mm[1] = int(parts[6]) # Rear
            # IMU Data (Accel X,Y,Z, Gyro X,Y,Z)
            imu_data_raw[0] = int(parts[7])   # AccelX
            imu_data_raw[1] = int(parts[8])   # AccelY
            imu_data_raw[2] = int(parts[9])   # AccelZ
            imu_data_raw[3] = int(parts[10])  # GyroX
            imu_data_raw[4] = int(parts[11])  # GyroY
            imu_data_raw[5] = int(parts[12])  # GyroZ (Z-axis rotation is key for turns)
            logger.debug(f"Parsed Sensor Data: IR={ir_sensor_states}, US={ultrasonic_distances_mm}, IMU={imu_data_raw}")
        except (ValueError, IndexError) as e:
            logger.warning(f"Error parsing sensor data '{data_str}' - {e}")
    else:
        logger.warning(f"Received unexpected serial data format: {data_str}")

# --- Core Control Logic Functions ---

def process_camera_frame(frame):
    """
    Main computer vision function.
    Analyzes the frame to determine track position, detect signs, parking lot, etc.
    Returns:
        error (float): Lateral error from track center (positive = right of center).
        sign_info (dict or None): Information about the most relevant detected sign.
        parking_lot_info (dict or None): Information about detected parking lot.
    """
    # IMPORTANT: THESE VALUES NEED TO BE CALIBRATED FOR YOUR SETUP!
    # HSV Color Ranges (Hue 0-179, Sat 0-255, Val 0-255)
    LOWER_TRACK = np.array([0, 0, 180])      # Example: Low saturation, high value
    UPPER_TRACK = np.array([180, 50, 255])    # Example: Any hue, low-mid saturation, high value
    LOWER_WALL = np.array([0, 0, 0])         # Example: Very low value
    UPPER_WALL = np.array([180, 255, 50])     # Example: Any hue, any saturation, low value
    LOWER_RED1 = np.array([0, 100, 100])     # Lower red range (Hue wraps around in OpenCV)
    UPPER_RED1 = np.array([10, 255, 255])
    LOWER_RED2 = np.array([160, 100, 100])    # Upper red range
    UPPER_RED2 = np.array([179, 255, 255])
    LOWER_GREEN = np.array([40, 100, 100])   # Example green range
    UPPER_GREEN = np.array([80, 255, 255])
    LOWER_MAGENTA = np.array([130, 100, 100]) # Example magenta range
    UPPER_MAGENTA = np.array([160, 255, 255])

    # ROI Settings
    ROI_Y_START = 0.6
    ROI_Y_END = 1.0
    CROP_Y_START = int(FRAME_HEIGHT * ROI_Y_START)
    CROP_Y_END = int(FRAME_HEIGHT * ROI_Y_END)
    CROP_X_START = 0
    CROP_X_END = FRAME_WIDTH

    # Processing Parameters
    MIN_LINE_LENGTH_FRAC = 0.1
    MAX_LINE_GAP_FRAC = 0.05
    HOUGH_THRESHOLD = 30
    MIN_SIGN_AREA = 100
    SIGN_ASPECT_RATIO_TOL = 0.3

    error = 0.0
    sign_info = None
    parking_lot_info = None

    if frame is None:
        logger.warning("Received None frame in process_camera_frame")
        return error, sign_info, parking_lot_info

    try:
        height, width = frame.shape[:2]
        # --- 1. Region of Interest (ROI) Cropping ---
        roi = frame[CROP_Y_START:CROP_Y_END, CROP_X_START:CROP_X_END]
        roi_height, roi_width = roi.shape[:2]

        # --- 2. Color Space Conversion ---
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- 3. Color Thresholding for Track and Walls ---
        track_mask = cv2.inRange(hsv_roi, LOWER_TRACK, UPPER_TRACK)
        wall_mask = cv2.inRange(hsv_roi, LOWER_WALL, UPPER_WALL)

        # --- 4. Morphological Operations to clean masks ---
        kernel = np.ones((5, 5), np.uint8)
        track_mask = cv2.morphologyEx(track_mask, cv2.MORPH_OPEN, kernel)
        track_mask = cv2.morphologyEx(track_mask, cv2.MORPH_CLOSE, kernel)
        wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_OPEN, kernel)
        wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, kernel)

        # --- 5. Find Track Boundaries using Edge Detection + HoughLinesP ---
        # Combine track and wall info or use edge detection on the ROI
        # Method: Use Canny edge detection on the ROI, then HoughLinesP
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to reduce noise
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        # Canny edge detection
        edges = cv2.Canny(blurred_roi, 50, 150, apertureSize=3)

        # Hough Line Transform to find lines
        min_line_length = int(roi_width * MIN_LINE_LENGTH_FRAC)
        max_line_gap = int(roi_width * MAX_LINE_GAP_FRAC)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=HOUGH_THRESHOLD,
                                minLineLength=min_line_length, maxLineGap=max_line_gap)

        left_lines = []
        right_lines = []
        center_x = roi_width // 2

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate slope to distinguish left/right boundaries
                if x2 != x1: # Avoid division by zero
                    slope = (y2 - y1) / (x2 - x1)
                    # Assume mostly vertical lines for track edges
                    # Filter based on slope if needed, or just position
                    line_center_x = (x1 + x2) / 2
                    if line_center_x < center_x - 20: # Left of center
                        left_lines.append(line[0])
                    elif line_center_x > center_x + 20: # Right of center
                        right_lines.append(line[0])
                # else: Vertical line, handle if necessary

        # --- 6. Calculate Track Center and Error ---
        # Find average x-position of left and right boundaries at a specific y-level
        # (e.g., near the bottom of the ROI)
        y_level = roi_height * 0.7 # Look at lines around 70% down the ROI

        left_x_coords = []
        right_x_coords = []

        for line in left_lines:
            x1, y1, x2, y2 = line
            # Interpolate/extrapolate to y_level if the line spans it
            if min(y1, y2) <= y_level <= max(y1, y2) and x2 != x1:
                 # Simple linear interpolation
                 x_at_y_level = x1 + (y_level - y1) * (x2 - x1) / (y2 - y1)
                 left_x_coords.append(x_at_y_level)

        for line in right_lines:
            x1, y1, x2, y2 = line
            if min(y1, y2) <= y_level <= max(y1, y2) and x2 != x1:
                 x_at_y_level = x1 + (y_level - y1) * (x2 - x1) / (y2 - y1)
                 right_x_coords.append(x_at_y_level)

        avg_left_x = np.mean(left_x_coords) if left_x_coords else None
        avg_right_x = np.mean(right_x_coords) if right_x_coords else None

        if avg_left_x is not None and avg_right_x is not None:
            track_center_x = (avg_left_x + avg_right_x) / 2.0
            error = track_center_x - center_x # Positive error = car is right of track center
            logger.debug(f"Track Center X: {track_center_x:.2f}, ROI Center: {center_x}, Error: {error:.2f}")
        elif avg_left_x is not None:
            # Only left boundary seen, assume car is too far right
            error = 50.0 # Large positive error
            logger.debug("Only left boundary detected, assuming far right.")
        elif avg_right_x is not None:
            # Only right boundary seen, assume car is too far left
            error = -50.0 # Large negative error
            logger.debug("Only right boundary detected, assuming far left.")
        else:
            # No boundaries found, maintain previous error or assume center?
            # This is a critical failure state, might need better handling
            logger.warning("No track boundaries found!")
            error = 0.0 # Hold last known good value or implement fallback


        # --- 7. Color Thresholding for Signs and Parking Lot ---
        red_mask1 = cv2.inRange(hsv_roi, LOWER_RED1, UPPER_RED1)
        red_mask2 = cv2.inRange(hsv_roi, LOWER_RED2, UPPER_RED2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_mask = cv2.inRange(hsv_roi, LOWER_GREEN, UPPER_GREEN)
        magenta_mask = cv2.inRange(hsv_roi, LOWER_MAGENTA, UPPER_MAGENTA)

        # Clean sign masks
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        magenta_mask = cv2.morphologyEx(magenta_mask, cv2.MORPH_OPEN, kernel)
        magenta_mask = cv2.morphologyEx(magenta_mask, cv2.MORPH_CLOSE, kernel)

        # --- 8. Find Contours for Signs ---
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        magenta_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # --- 9. Analyze Contours for Signs ---
        detected_signs = []
        for cnt in red_contours:
            area = cv2.contourArea(cnt)
            if area > MIN_SIGN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h if h != 0 else 0
                if abs(aspect_ratio - 1.0) < SIGN_ASPECT_RATIO_TOL: # Check if roughly square
                    # Calculate centroid
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # Determine if sign is left or right of car center
                        position = "left" if cx < center_x else "right"
                        detected_signs.append({'type': 'red', 'position': position, 'cx': cx, 'cy': cy, 'area': area, 'bbox': (x, y, w, h)})

        for cnt in green_contours:
            area = cv2.contourArea(cnt)
            if area > MIN_SIGN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h if h != 0 else 0
                if abs(aspect_ratio - 1.0) < SIGN_ASPECT_RATIO_TOL:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        position = "left" if cx < center_x else "right"
                        detected_signs.append({'type': 'green', 'position': position, 'cx': cx, 'cy': cy, 'area': area, 'bbox': (x, y, w, h)})

        # --- 10. Analyze Contours for Parking Lot ---
        # Parking lot is defined by two magenta barriers
        parking_lot_bars = []
        for cnt in magenta_contours:
             area = cv2.contourArea(cnt)
             if area > MIN_SIGN_AREA / 2: # Might be smaller than signs
                 x, y, w, h = cv2.boundingRect(cnt)
                 aspect_ratio = float(w) / h if h != 0 else 0
                 # Parking barriers are long and thin rectangles
                 if w > h and aspect_ratio > 2.0: # Adjust ratio as needed
                     M = cv2.moments(cnt)
                     if M["m00"] != 0:
                         cx = int(M["m10"] / M["m00"])
                         cy = int(M["m01"] / M["m00"])
                         parking_lot_bars.append({'cx': cx, 'cy': cy, 'bbox': (x, y, w, h), 'area': area})

        # --- 11. Determine Most Relevant Sign (Basic Logic) ---
        # In a real scenario, you'd integrate this with lap counting and state logic.
        # For now, just pick the closest sign in the forward direction.
        if detected_signs:
            # Assume signs are in front. Pick the one closest to the center line vertically (lower y)
            # or the one with the largest area.
            closest_sign = min(detected_signs, key=lambda s: s['cy']) # Find sign lowest in image (closest)
            sign_info = closest_sign
            logger.debug(f"Detected Sign: {sign_info}")

        # --- 12. Determine Parking Lot Info ---
        if len(parking_lot_bars) >= 2:
            # Sort bars by x position
            parking_lot_bars.sort(key=lambda b: b['cx'])
            left_bar = parking_lot_bars[0]
            right_bar = parking_lot_bars[-1] # Take the rightmost if more than 2

            # Calculate center of parking lot entrance
            lot_center_x = (left_bar['cx'] + right_bar['cx']) / 2.0
            lot_position = "left" if lot_center_x < center_x else "right"

            parking_lot_info = {
                'detected': True,
                'center_x': lot_center_x,
                'position': lot_position,
                'left_bar': left_bar,
                'right_bar': right_bar
            }
            logger.debug(f"Parking Lot Detected: Center X={lot_center_x:.2f}")

        # --- 13. Optional: Visualization for Debugging (Comment out for performance) ---
        # Draw lines on ROI for debugging
        # debug_roi = roi.copy()
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         cv2.line(debug_roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # # Draw track center/error line
        # cv2.line(debug_roi, (int(center_x), 0), (int(center_x), roi_height), (255, 0, 0), 2) # Center line
        # if avg_left_x is not None:
        #     cv2.line(debug_roi, (int(avg_left_x), int(y_level)-10), (int(avg_left_x), int(y_level)+10), (255, 255, 0), 2)
        # if avg_right_x is not None:
        #     cv2.line(debug_roi, (int(avg_right_x), int(y_level)-10), (int(avg_right_x), int(y_level)+10), (255, 255, 0), 2)
        # if avg_left_x is not None and avg_right_x is not None:
        #      cv2.line(debug_roi, (int(track_center_x), int(y_level)-10), (int(track_center_x), int(y_level)+10), (0, 0, 255), 2) # Track center

        # Display masks or debug image if needed (for development)
        # cv2.imshow("Track Mask", track_mask)
        # cv2.imshow("Debug ROI", debug_roi)
        # cv2.waitKey(1) # Needed for imshow to update

    except Exception as e:
        logger.error(f"Error in process_camera_frame: {e}", exc_info=True)
        # In case of error, return default values or last known good values
        # error, sign_info, parking_lot_info remain as initialized or last set

    return error, sign_info, parking_lot_info


def calculate_control_commands(error, sign_info, parking_lot_info):
    """
    Calculates motor and servo commands based on sensor data and state.
    Returns:
        left_motor_speed (int): Speed for left motor.
        right_motor_speed (int): Speed for right motor.
        servo_angle (int): Angle for steering servo.
    """
    global state, lap_count, parking_sequence_step

    left_motor_speed = 0
    right_motor_speed = 0
    servo_angle = 90 # Center

    if state == STATE_DRIVING_LAPS:
        # --- Basic Line Following using Error ---
        # Simple Proportional control for steering
        p_term = STEERING_P_GAIN * error
        # Optional D term (requires previous error)
        # d_term = STEERING_D_GAIN * (error - previous_error) / dt
        # For now, just P
        steering_adjustment = p_term

        # Map steering adjustment to servo angle (limit it)
        servo_angle = int(90 + steering_adjustment)
        servo_angle = np.clip(servo_angle, 45, 135) # Limit steering range

        # Set base speed
        left_motor_speed = BASE_SPEED
        right_motor_speed = BASE_SPEED

        # Apply differential steering based on servo angle
        # Simple method: reduce speed on the inside wheel
        if servo_angle < 85: # Turn Left
            right_motor_speed = min(BASE_SPEED + 20, MAX_SPEED)
            left_motor_speed = max(BASE_SPEED - 20, MIN_SPEED)
        elif servo_angle > 95: # Turn Right
            left_motor_speed = min(BASE_SPEED + 20, MAX_SPEED)
            right_motor_speed = max(BASE_SPEED - 20, MIN_SPEED)

        logger.debug(f"Driving: Error={error:.2f}, SteerAdj={steering_adjustment:.2f}, Servo={servo_angle}, L={left_motor_speed}, R={right_motor_speed}")

    elif state == STATE_PARKING:
        # --- Placeholder Parking Logic ---
        # This would involve a sequence of commands
        logger.info(f"Parking sequence step: {parking_sequence_step}")
        if parking_sequence_step == 0:
            # Example: Move forward a bit
            left_motor_speed = BASE_SPEED
            right_motor_speed = BASE_SPEED
            servo_angle = 90
            parking_sequence_step += 1
        elif parking_sequence_step == 1:
            # Example: Turn right
            left_motor_speed = BASE_SPEED
            right_motor_speed = -BASE_SPEED # Reverse right
            servo_angle = 135
            parking_sequence_step += 1
        # ... Add more steps ...
        else:
            # Stop when sequence is done (or based on sensor feedback)
            left_motor_speed = 0
            right_motor_speed = 0
            servo_angle = 90
            state = STATE_STOPPED # Transition to stopped after parking
        # --- End Placeholder Parking Logic ---

    elif state == STATE_STOPPED:
        left_motor_speed = 0
        right_motor_speed = 0
        servo_angle = 90

    # Ensure speeds are within valid range (Nano code also constrains, but good practice here)
    left_motor_speed = int(np.clip(left_motor_speed, -255, 255))
    right_motor_speed = int(np.clip(right_motor_speed, -255, 255))

    return left_motor_speed, right_motor_speed, servo_angle

def update_lap_count(gyro_z_rate):
    """
    Updates the lap count based on IMU gyro Z-axis data (detecting turns).
    This is called periodically, likely within the main loop based on sensor data timing.
    """
    global turn_count, last_turn_integral, gyro_z_history, lap_count

    # Convert gyro Z rate (raw or scaled) to degrees/second
    # This requires knowing the MPU6050 sensitivity (e.g., +/- 2000 deg/s scale)
    # Sensitivity factor example (needs calibration/lookup): ~0.061 degrees/sec per LSB for 2000dps
    GYRO_SENSITIVITY_DPS_PER_LSB = 0.061
    gyro_z_dps = gyro_z_rate * GYRO_SENSITIVITY_DPS_PER_LSB

    # Add current rate to history for smoothing
    gyro_z_history.append(gyro_z_dps)

    # Simple integral approximation (sum over time window)
    # dt (time delta) depends on how often this function is called.
    # If called roughly every 100ms (SENSOR_SEND_INTERVAL), dt = 0.1s
    dt = 0.1 # Adjust if SENSOR_SEND_INTERVAL changes
    current_integral = sum(gyro_z_history) * dt / len(gyro_z_history) if len(gyro_z_history) > 0 else 0

    turn_angle = current_integral - last_turn_integral
    last_turn_integral = current_integral

    logger.debug(f"Gyro Z Rate: {gyro_z_rate}, DPS: {gyro_z_dps:.2f}, Angle Turned: {turn_angle:.2f}deg")

    # Simple turn detection logic (needs refinement)
    if abs(turn_angle) > TURN_THRESHOLD_DEGREES and turn_count < REQUIRED_TURNS_PER_LAP:
        turn_count += 1
        logger.info(f"Turn detected! Count: {turn_count}/4")
        # Reset integral history to avoid double counting?
        # gyro_z_history.clear()
        # last_turn_integral = current_integral

    # Reset turn detection if rotation slows down significantly
    if abs(turn_angle) < TURN_RESET_THRESHOLD_DEGREES and turn_count > 0:
         # Potentially reset turn_count if needed, or just rely on reaching 4
         pass

    # Check if a lap is completed
    if turn_count >= REQUIRED_TURNS_PER_LAP:
        lap_count += 1
        turn_count = 0
        last_turn_integral = 0.0
        gyro_z_history.clear()
        logger.info(f"*** LAP COMPLETED! Lap Count: {lap_count}/3 ***")

        # Check if 3 laps are done
        if lap_count >= REQUIRED_LAPS:
            logger.info("*** THREE LAPS COMPLETED! ***")
            # Transition logic depends on challenge
            # For Open: Stop or prepare to stop in finish section
            # For Obstacle: Start looking for parking lot / initiate parking
            # This check and transition would ideally be in the main loop after calling this function.
            # For now, we'll handle it there based on lap_count.

# --- Main Execution Block ---
def main():
    global state, lap_count, start_time

    logger.info("Initializing WRO 2025 Car Controller (Pi Side)...")

    if not initialize_camera():
        logger.error("Camera initialization failed. Exiting.")
        ser.close()
        exit(1)

    start_time = time.time()
    logger.info("Entering main control loop...")

    try:
        while True:
            # --- 1. Check for incoming serial data from Nano ---
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                if line:
                    logger.debug(f"Received from Nano: {line}")
                    if line == "START":
                        if state == STATE_WAITING_FOR_START:
                            logger.info("START signal received from Nano!")
                            send_command_to_nano("ACK_START") # Acknowledge
                            state = STATE_DRIVING_LAPS
                            logger.info("State changed to DRIVING_LAPS")
                            # Optional: Small delay before motors start moving
                            time.sleep(0.5)
                    elif line.startswith("SENSORS"):
                        parse_sensor_data(line)
                        # Trigger lap counting based on new IMU data
                        # Assuming IMU Gyro Z is imu_data_raw[5]
                        update_lap_count(imu_data_raw[5])
                    # Handle other potential commands from Nano if needed

            # --- 2. Main State Machine Logic ---
            if state == STATE_WAITING_FOR_START:
                # Do minimal processing, just wait for START command
                time.sleep(0.1) # Don't consume too much CPU

            elif state == STATE_DRIVING_LAPS:
                # --- a. Capture Camera Frame ---
                ret, frame = cap.read()
                if not ret:
                    logger.warning("Failed to grab frame from camera.")
                    continue # Skip processing if no frame

                # --- b. Process Sensors & Camera ---
                # Sensor data (`ir_sensor_states`, `ultrasonic_distances_mm`, `imu_data_raw`)
                # is updated asynchronously via the serial parsing thread/callback.
                # We use the latest available values here.

                # Process the frame for vision
                error, sign_info, parking_lot_info = process_camera_frame(frame)

                # --- c. Check Lap Completion and Transition ---
                # The `update_lap_count` function called above updates `lap_count`.
                # We check the count here to decide on state transitions.
                if lap_count >= REQUIRED_LAPS:
                     # For demo, let's assume we are in Obstacle mode if we reach 3 laps
                     # In reality, you'd know the challenge type.
                     logger.info("Three laps completed. Initiating Parking sequence.")
                     state = STATE_PARKING
                     parking_sequence_step = 0 # Reset parking steps
                     lap_count = 0 # Reset for potential next run logic if needed

                # --- d. Calculate Control Commands ---
                left_speed, right_speed, servo_angle = calculate_control_commands(error, sign_info, parking_lot_info)

                # --- e. Send Commands to Nano ---
                send_command_to_nano(f"MOTOR,{left_speed},{right_speed}")
                send_command_to_nano(f"SERVO,{servo_angle}")

                # --- f. Loop Timing ---
                # Add a small delay to control loop speed if necessary.
                # Processing speed and camera FPS will largely dictate this.
                # time.sleep(0.05) # Adjust as needed, or remove if processing is the bottleneck.

            elif state == STATE_PARKING:
                 # Sensor and camera processing would still happen
                 # Placeholder: Just send commands from the calculate function
                 # In reality, `process_camera_frame` would detect the lot,
                 # and `calculate_control_commands` would execute the sequence.
                 # For now, we simulate it with internal state.
                 # Let's trigger the parking command calculation once per loop iteration here.
                 dummy_error = 0
                 dummy_sign_info = None
                 dummy_parking_info = None # Could be used to trigger parking start
                 left_speed, right_speed, servo_angle = calculate_control_commands(dummy_error, dummy_sign_info, dummy_parking_info)
                 send_command_to_nano(f"MOTOR,{left_speed},{right_speed}")
                 send_command_to_nano(f"SERVO,{servo_angle}")

                 # Add delay for parking steps
                 time.sleep(1) # 1 second per parking step for demo

            elif state == STATE_STOPPED:
                # Ensure everything is stopped
                send_command_to_nano("MOTOR,0,0")
                send_command_to_nano("SERVO,90")
                logger.info("Vehicle stopped. Waiting...")
                time.sleep(1) # Stay in stopped state

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # --- Cleanup ---
        logger.info("Cleaning up resources...")
        send_command_to_nano("MOTOR,0,0") # Ensure motors stop
        if cap and cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()
        ser.close()
        logger.info("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()
