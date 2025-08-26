#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import logging
from collections import deque
import cv2
import numpy as np

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

CAMERA_INDEX = 2
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_RATE = 15 # Target, though actual might differ

# State Machine States
STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_PARKING = "PARKING"
STATE_STOPPED = "STOPPED"

# Lap Counting Configuration (Placeholder if IMU fails)
REQUIRED_TURNS_PER_LAP = 4
REQUIRED_LAPS = 3
TURN_THRESHOLD_DEGREES = 70
TURN_RESET_THRESHOLD_DEGREES = 20
GYRO_INTEGRAL_WINDOW_SIZE = 5

# Parking Configuration
PARKING_LOT_DETECTED_THRESHOLD = 0.8
PARALLEL_PARK_STEPS = 5

# Control Parameters
BASE_SPEED = 70
MAX_SPEED = 100
MIN_SPEED = 30
STEERING_P_GAIN = 0.8
STEERING_D_GAIN = 0.3
# --- End Configuration ---

# --- Global Variables ---
state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE)
parking_sequence_step = 0
start_time = None

# Sensor Data Storage
ir_sensor_states = [1, 1, 1, 1]
ultrasonic_distances_mm = [9999, 9999]
imu_data_raw = [0, 0, 0, 0, 0, 0]

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Communication with Nano ---
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    logger.info(f"Connected to Arduino Nano on {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    logger.error(f"Failed to connect to serial port {SERIAL_PORT}: {e}")
    exit(1)

# --- Camera Setup ---
cap = None
def initialize_camera():
    global cap
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            logger.error("Error: Could not open USB camera.")
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
        time.sleep(0.2) # Give camera time to settle
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"Camera reports: Width={actual_width}, Height={actual_height}, FPS={actual_fps}")
        logger.info(f"Camera initialized (Index: {CAMERA_INDEX}, Res: {actual_width}x{actual_height} @ {actual_fps} FPS)")
        return True
    except Exception as e:
        logger.error(f"Error initializing camera: {e}", exc_info=True)
        return False

# --- Helper Functions for Communication ---
def send_command_to_nano(command_str):
    global ser
    if ser is None:
        logger.warning("Serial port not initialized. Cannot send command.")
        return
    try:
        if not ser.is_open:
             logger.warning(f"Serial port {SERIAL_PORT} is not open. Attempting to reopen...")
             ser.open()
        ser.write((command_str + '\n').encode('utf-8'))
        ser.flush() # Ensure command is sent immediately
        logger.debug(f"Sent to Nano: {command_str}")
    except Exception as e:
        logger.error(f"Error sending command to Nano: {e}")
        try:
            if ser and ser.is_open:
                ser.close()
        except:
            pass
        ser = None

def parse_sensor_data(data_str):
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw
    try:
        parts = data_str.split(',')
        if len(parts) >= 13 and parts[0] == "SENSORS":
            try:
                # Parse sensor data
                ir_sensor_states[0] = int(parts[1]) # FL
                ir_sensor_states[1] = int(parts[2]) # FR
                ir_sensor_states[2] = int(parts[3]) # R45
                ir_sensor_states[3] = int(parts[4]) # L45
                ultrasonic_distances_mm[0] = int(parts[5]) # Front
                ultrasonic_distances_mm[1] = int(parts[6]) # Rear
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
    except Exception as e:
        logger.error(f"Error in parse_sensor_data: {e}")

# --- Core Control Logic Functions ---
def process_camera_frame(frame):
    # HSV Color Ranges
    LOWER_TRACK = np.array([0, 0, 180])
    UPPER_TRACK = np.array([180, 50, 255])
    LOWER_WALL = np.array([0, 0, 0])
    UPPER_WALL = np.array([180, 255, 50])
    LOWER_RED1 = np.array([0, 100, 100])
    UPPER_RED1 = np.array([10, 255, 255])
    LOWER_RED2 = np.array([160, 100, 100])
    UPPER_RED2 = np.array([179, 255, 255])
    LOWER_GREEN = np.array([40, 100, 100])
    UPPER_GREEN = np.array([80, 255, 255])
    LOWER_MAGENTA = np.array([130, 100, 100])
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
        # ROI Cropping
        roi = frame[CROP_Y_START:CROP_Y_END, CROP_X_START:CROP_X_END]
        roi_height, roi_width = roi.shape[:2]

        # Color Space Conversion
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Color Thresholding
        track_mask = cv2.inRange(hsv_roi, LOWER_TRACK, UPPER_TRACK)
        wall_mask = cv2.inRange(hsv_roi, LOWER_WALL, UPPER_WALL)

        # Morphological Operations
        kernel = np.ones((5, 5), np.uint8)
        track_mask = cv2.morphologyEx(track_mask, cv2.MORPH_OPEN, kernel)
        track_mask = cv2.morphologyEx(track_mask, cv2.MORPH_CLOSE, kernel)
        wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_OPEN, kernel)
        wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, kernel)

        # Edge Detection
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        edges = cv2.Canny(blurred_roi, 50, 150, apertureSize=3)

        # Hough Line Transform
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
                if x2 != x1: # Avoid division by zero
                    line_center_x = (x1 + x2) / 2
                    if line_center_x < center_x - 20:
                        left_lines.append(line[0])
                    elif line_center_x > center_x + 20:
                        right_lines.append(line[0])

        # Calculate Track Center and Error
        y_level = roi_height * 0.7
        left_x_coords = []
        right_x_coords = []

        for line in left_lines:
            x1, y1, x2, y2 = line
            if min(y1, y2) <= y_level <= max(y1, y2) and x2 != x1:
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
            error = track_center_x - center_x
            logger.debug(f"Track Center X: {track_center_x:.2f}, ROI Center: {center_x}, Error: {error:.2f}")
        elif avg_left_x is not None:
            error = 50.0 # Large positive error
            logger.debug("Only left boundary detected, assuming far right.")
        elif avg_right_x is not None:
            error = -50.0 # Large negative error
            logger.debug("Only right boundary detected, assuming far left.")
        else:
            logger.warning("No track boundaries found!")
            error = 0.0 # Hold last known good value

        # Color Thresholding for Signs and Parking Lot
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

        # Find Contours for Signs
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        magenta_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Analyze Contours for Signs
        detected_signs = []
        for cnt in red_contours:
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

        # Analyze Contours for Parking Lot
        parking_lot_bars = []
        for cnt in magenta_contours:
             area = cv2.contourArea(cnt)
             if area > MIN_SIGN_AREA / 2:
                 x, y, w, h = cv2.boundingRect(cnt)
                 aspect_ratio = float(w) / h if h != 0 else 0
                 if w > h and aspect_ratio > 2.0:
                     M = cv2.moments(cnt)
                     if M["m00"] != 0:
                         cx = int(M["m10"] / M["m00"])
                         cy = int(M["m01"] / M["m00"])
                         parking_lot_bars.append({'cx': cx, 'cy': cy, 'bbox': (x, y, w, h), 'area': area})

        # Determine Most Relevant Sign
        if detected_signs:
            closest_sign = min(detected_signs, key=lambda s: s['cy'])
            sign_info = closest_sign
            logger.debug(f"Detected Sign: {sign_info}")

        # Determine Parking Lot Info
        if len(parking_lot_bars) >= 2:
            parking_lot_bars.sort(key=lambda b: b['cx'])
            left_bar = parking_lot_bars[0]
            right_bar = parking_lot_bars[-1]
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

    except Exception as e:
        logger.error(f"Error in process_camera_frame: {e}", exc_info=True)

    return error, sign_info, parking_lot_info

def calculate_control_commands(error, sign_info, parking_lot_info):
    global state, lap_count, parking_sequence_step

    left_motor_speed = 0
    right_motor_speed = 0
    servo_angle = 90

    if state == STATE_DRIVING_LAPS:
        # Basic Line Following using Error
        p_term = STEERING_P_GAIN * error
        steering_adjustment = p_term
        servo_angle = int(90 + steering_adjustment)
        servo_angle = np.clip(servo_angle, 45, 135)
        left_motor_speed = BASE_SPEED
        right_motor_speed = BASE_SPEED
        # Apply differential steering
        if servo_angle < 85: # Turn Left
            right_motor_speed = min(BASE_SPEED + 20, MAX_SPEED)
            left_motor_speed = max(BASE_SPEED - 20, MIN_SPEED)
        elif servo_angle > 95: # Turn Right
            left_motor_speed = min(BASE_SPEED + 20, MAX_SPEED)
            right_motor_speed = max(BASE_SPEED - 20, MIN_SPEED)
        logger.debug(f"Driving: Error={error:.2f}, SteerAdj={steering_adjustment:.2f}, Servo={servo_angle}, L={left_motor_speed}, R={right_motor_speed}")

    elif state == STATE_PARKING:
        logger.info(f"Parking sequence step: {parking_sequence_step}")
        if parking_sequence_step == 0:
            left_motor_speed = BASE_SPEED
            right_motor_speed = BASE_SPEED
            servo_angle = 90
            parking_sequence_step += 1
        elif parking_sequence_step == 1:
            left_motor_speed = BASE_SPEED
            right_motor_speed = -BASE_SPEED # Reverse right
            servo_angle = 135
            parking_sequence_step += 1
        else:
            left_motor_speed = 0
            right_motor_speed = 0
            servo_angle = 90
            state = STATE_STOPPED

    elif state == STATE_STOPPED:
        left_motor_speed = 0
        right_motor_speed = 0
        servo_angle = 90

    # Ensure speeds are within valid range
    left_motor_speed = int(np.clip(left_motor_speed, -255, 255))
    right_motor_speed = int(np.clip(right_motor_speed, -255, 255))

    return left_motor_speed, right_motor_speed, servo_angle

def update_lap_count(gyro_z_rate):
    """Attempt to update lap count, but handle IMU failure gracefully."""
    global turn_count, last_turn_integral, gyro_z_history, lap_count

    # If IMU data is all zeros, lap counting won't work.
    # We can add a timeout-based fallback if needed, but for now, log the issue.
    if all(v == 0 for v in imu_data_raw):
        logger.warning("IMU data is all zeros. Lap counting disabled.")
        return # Skip lap counting if IMU failed

    try:
        GYRO_SENSITIVITY_DPS_PER_LSB = 0.061
        gyro_z_dps = gyro_z_rate * GYRO_SENSITIVITY_DPS_PER_LSB
        gyro_z_history.append(gyro_z_dps)
        dt = 0.1 # Approximate, based on sensor send interval
        current_integral = sum(gyro_z_history) * dt / len(gyro_z_history) if len(gyro_z_history) > 0 else 0
        turn_angle = current_integral - last_turn_integral
        last_turn_integral = current_integral
        logger.debug(f"Gyro Z Rate: {gyro_z_rate}, DPS: {gyro_z_dps:.2f}, Angle Turned: {turn_angle:.2f}deg")

        if abs(turn_angle) > TURN_THRESHOLD_DEGREES and turn_count < REQUIRED_TURNS_PER_LAP:
            turn_count += 1
            logger.info(f"Turn detected! Count: {turn_count}/4")

        if abs(turn_angle) < TURN_RESET_THRESHOLD_DEGREES and turn_count > 0:
             pass

        if turn_count >= REQUIRED_TURNS_PER_LAP:
            lap_count += 1
            turn_count = 0
            last_turn_integral = 0.0
            gyro_z_history.clear()
            logger.info(f"*** LAP COMPLETED! Lap Count: {lap_count}/3 ***")

            if lap_count >= REQUIRED_LAPS:
                logger.info("*** THREE LAPS COMPLETED! ***")

    except Exception as e:
        logger.error(f"Error in update_lap_count: {e}")

# --- Main Execution Block ---
def main():
    global state, lap_count, start_time, ser, cap

    logger.info("Initializing WRO 2025 Car Controller (Pi Side)...")

    if not initialize_camera():
        logger.error("Camera initialization failed. Exiting.")
        if ser and ser.is_open:
            try:
                ser.close()
            except:
                pass
        exit(1)

    start_time = time.time()
    logger.info("Entering main control loop...")

    try:
        while True:
            # --- 1. Check for incoming serial data from Nano ---
            if ser and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                if line:
                    logger.debug(f"Received from Nano: {line}")
                    if line == "START":
                        # Critical Fix: Immediately transition to DRIVING on START
                        logger.info("START signal received from Nano!")
                        state = STATE_DRIVING_LAPS
                        logger.info("State changed to DRIVING_LAPS")
                        # Send immediate acknowledgment (though not strictly required for new logic)
                        send_command_to_nano("ACK_START")
                        # Small delay before motors might start
                        time.sleep(0.2)
                    elif line.startswith("SENSORS"):
                        parse_sensor_data(line)
                        # Trigger lap counting based on new IMU data
                        update_lap_count(imu_data_raw[5]) # Gyro Z
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
                    time.sleep(0.1) # Brief pause before retrying
                    continue # Skip processing if no frame

                # --- b. Process Sensors & Camera ---
                error, sign_info, parking_lot_info = process_camera_frame(frame)

                # --- c. Check Lap Completion and Transition ---
                # The `update_lap_count` function called above updates `lap_count`.
                # We check the count here to decide on state transitions.
                # Note: If IMU fails, lap_count will stay 0, so we might need a timer fallback.
                if lap_count >= REQUIRED_LAPS:
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
                time.sleep(0.05) # Adjust based on processing needs

            elif state == STATE_PARKING:
                 # Sensor and camera processing would still happen
                 # Placeholder: Just send commands from the calculate function
                 dummy_error = 0
                 dummy_sign_info = None
                 dummy_parking_info = None
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
        if ser and ser.is_open:
            ser.close()
        logger.info("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()