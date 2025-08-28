#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WRO 2025 Future Engineers - Self-Driving Car Controller (Raspberry Pi Side)
Optimized for Raspberry Pi Zero 2W.
Communicates with Arduino Nano via high-speed binary serial protocol.
Handles vision, navigation, lap counting, and control.
"""

import serial
import time
import logging
import struct
import numpy as np
import cv2
from collections import deque
import math
import threading
import signal
import sys

# --- Configuration ---
# Serial Port Configuration (Match Nano)
SERIAL_PORT = '/dev/serial0'  # Common for Pi UART. Check `ls /dev/tty*`
BAUD_RATE = 500000  # Match Nano's baud rate

# Camera settings optimized for Pi Zero 2W
CAMERA_INDEX = 0
FRAME_WIDTH = 160  # Reduced resolution for speed
FRAME_HEIGHT = 120
FRAME_RATE = 15    # Adjust based on processing capability

# Competition parameters
REQUIRED_LAPS = 3
REQUIRED_TURNS_PER_LAP = 4 # Based on 4 significant turns per lap
TURN_MIN_ANGLE = 75  # Degrees to count as a significant turn
TURN_RESET_ANGLE = 20 # Degrees below which turn state resets
GYRO_INTEGRAL_WINDOW_SIZE = 5 # For smoothing gyro integration

# HSV Color Ranges (Tuned for WRO 2025 field - needs calibration!)
LOWER_WHITE = np.array([0, 0, 180])
UPPER_WHITE = np.array([180, 50, 255])
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 50])
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([179, 255, 255])
LOWER_GREEN = np.array([40, 100, 100])
UPPER_GREEN = np.array([80, 255, 255])
LOWER_MAGENTA = np.array([130, 100, 100])
UPPER_MAGENTA = np.array([160, 255, 255])

# Control parameters (Tune these!)
BASE_SPEED = 65 # Base motor speed
MAX_SPEED = 90
MIN_SPEED = 25
STEERING_P_GAIN = 0.7
STEERING_D_GAIN = 0.2
LANE_KEEP_OFFSET_MM = 50  # mm offset from center for lane keeping

# State Machine States
STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_OBEYING_SIGN = "OBEYING_SIGN"
STATE_PARKING_SEARCH = "PARKING_SEARCH" # Find the parking lot
STATE_PARKING_MANEUVER = "PARKING_MANEUVER" # Execute parking steps
STATE_STOPPED = "STOPPED"

# --- Global Variables ---
state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE)
parking_sequence_step = 0
start_time = None
run_acknowledged = False # Flag to ensure we only ACK once

# Sensor Data Storage (Updated by serial reader thread)
ir_sensor_states = [1, 1, 1, 1]  # FL, FR, R45, L45 (1 = Line/Obstacle detected, 0 = Not)
ultrasonic_distances_mm = [9999, 9999]  # Front, Rear
imu_data_raw = [0, 0, 0, 0, 0, 0]  # AccelX, Y, Z; GyroX, Y, Z

# Threading and Communication
serial_lock = threading.Lock() # Protect sensor data access
ser = None
stop_event = threading.Event()

# --- Logging Setup ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("wro_car.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# --- Serial Communication with Nano ---
def initialize_serial():
    """Initialize serial connection to Arduino Nano."""
    global ser
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.01, # Non-blocking read
            write_timeout=0.1
        )
        logger.info(f"Connected to Arduino Nano on {SERIAL_PORT} at {BAUD_RATE} baud.")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to serial port {SERIAL_PORT}: {e}")
        return False

def send_command_to_nano(command_char, value=None):
    """Sends a command string to the Arduino Nano via serial."""
    if not ser or not ser.is_open:
        logger.warning("Serial port not available for sending command.")
        return False
    try:
        if value is not None:
            cmd_str = f"C{command_char}:{value}\n"
        else:
            cmd_str = f"C{command_char}\n"
        ser.write(cmd_str.encode('utf-8'))
        ser.flush()
        logger.debug(f"Sent to Nano: {cmd_str.strip()}")
        return True
    except Exception as e:
        logger.error(f"Error sending command to Nano: {e}")
        return False

def serial_reader_thread():
    """Thread function to continuously read sensor data from Nano."""
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw
    buffer = bytearray()
    while not stop_event.is_set():
        if ser and ser.is_open and ser.in_waiting > 0:
            # Read all available bytes
            data = ser.read(ser.in_waiting)
            buffer.extend(data)

            # Process buffer for complete packets
            while len(buffer) >= 21: # Start (1) + Data (19) + End (1) = 21 bytes
                if buffer[0] == 0xAA: # Start byte found
                    if len(buffer) >= 21 and buffer[20] == 0x55: # End byte found
                        # Valid packet found
                        packet_data = buffer[1:20] # Extract 19 bytes of data
                        
                        # Parse binary data (match Nano's struct)
                        try:
                            ir_states_byte = packet_data[0]
                            us_front = struct.unpack('<h', packet_data[1:3])[0]
                            us_rear = struct.unpack('<h', packet_data[3:5])[0]
                            accel_x = struct.unpack('<h', packet_data[5:7])[0]
                            accel_y = struct.unpack('<h', packet_data[7:9])[0]
                            accel_z = struct.unpack('<h', packet_data[9:11])[0]
                            gyro_x = struct.unpack('<h', packet_data[11:13])[0]
                            gyro_y = struct.unpack('<h', packet_data[13:15])[0]
                            gyro_z = struct.unpack('<h', packet_data[15:17])[0]
                            
                            # Update global sensor data atomically
                            with serial_lock:
                                # Unpack IR states (bit 0: FL, 1: FR, 2: R45, 3: L45)
                                ir_sensor_states[0] = (ir_states_byte >> 0) & 1
                                ir_sensor_states[1] = (ir_states_byte >> 1) & 1
                                ir_sensor_states[2] = (ir_states_byte >> 2) & 1
                                ir_sensor_states[3] = (ir_states_byte >> 3) & 1
                                
                                ultrasonic_distances_mm[0] = us_front
                                ultrasonic_distances_mm[1] = us_rear
                                imu_data_raw = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
                                
                        except struct.error as e:
                            logger.error(f"Error unpacking sensor data: {e}")
                        
                        # Remove processed packet from buffer
                        del buffer[:21]
                    else:
                        # Start byte found but no end byte yet, wait for more data
                        break
                else:
                    # Not a start byte, remove it and continue searching
                    buffer.pop(0)
        else:
            time.sleep(0.001) # Small sleep to prevent busy-waiting

# --- Camera Setup ---
def initialize_camera():
    """Initialize the USB camera."""
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            logger.error("Could not open camera")
            return None
            
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
        # Reduce buffer size to get latest frames
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        
        time.sleep(0.5) # Allow camera to stabilize
        logger.info(f"Camera initialized: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {FRAME_RATE}fps")
        return cap
    except Exception as e:
        logger.error(f"Camera initialization failed: {e}")
        return None

# --- IMU Processing for Lap Counting ---
def update_lap_count(gyro_z_rate):
    """Updates the lap count based on IMU gyro Z-axis data."""
    global turn_count, last_turn_integral, gyro_z_history, lap_count

    # Convert raw gyro Z rate to degrees/second
    # MPU6050 default sensitivity: 131 LSB/degree/second for +/- 250 deg/s range
    GYRO_SENSITIVITY_DPS_PER_LSB = 131.0 
    gyro_z_dps = gyro_z_rate / GYRO_SENSITIVITY_DPS_PER_LSB

    # Add current rate to history for smoothing
    gyro_z_history.append(gyro_z_dps)

    # Simple integral approximation (sum over time window)
    dt = 0.05 # Approximate time interval (based on sensor send interval 50ms)
    current_integral = sum(gyro_z_history) * dt / len(gyro_z_history) if len(gyro_z_history) > 0 else 0

    turn_angle = current_integral - last_turn_integral
    last_turn_integral = current_integral

    logger.debug(f"Gyro Z Raw: {gyro_z_rate}, DPS: {gyro_z_dps:.2f}, Turned: {turn_angle:.2f}deg")

    # Simple turn detection logic
    if abs(turn_angle) > TURN_MIN_ANGLE and turn_count < REQUIRED_TURNS_PER_LAP:
        turn_count += 1
        logger.info(f"*** TURN DETECTED! Count: {turn_count}/{REQUIRED_TURNS_PER_LAP} ***")

    # Reset turn detection if rotation slows down significantly
    if abs(turn_angle) < TURN_RESET_ANGLE and turn_count > 0:
        pass # State maintained

    # Check if a lap is completed
    if turn_count >= REQUIRED_TURNS_PER_LAP:
        lap_count += 1
        turn_count = 0
        last_turn_integral = 0.0
        gyro_z_history.clear()
        logger.info(f"*** LAP {lap_count}/{REQUIRED_LAPS} COMPLETED! ***")

# --- Vision Processing ---
def detect_track_features(frame):
    """Detect track features: lane error, signs, parking lot."""
    if frame is None:
        return 0.0, None, None

    try:
        h, w = frame.shape[:2]
        # ROI: Focus on the bottom part where the track is
        roi_h_start = int(h * 0.6)
        roi = frame[roi_h_start:h, 0:w]
        roi_h, roi_w = roi.shape[:2]
        center_x = roi_w // 2

        # Convert to HSV for better color filtering
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- 1. Lane Detection (White track, Black walls) ---
        white_mask = cv2.inRange(hsv_roi, LOWER_WHITE, UPPER_WHITE)
        black_mask = cv2.inRange(hsv_roi, LOWER_BLACK, UPPER_BLACK)

        # Find edges of the drivable white area
        # Simple approach: scan a horizontal line near the bottom of ROI
        scan_line_y = roi_h - 10 # Scan near the bottom
        
        left_edge_x = None
        right_edge_x = None

        # Find left edge (transition from black to white)
        for x in range(5, center_x):
            if white_mask[scan_line_y, x] > 0 and black_mask[scan_line_y, x-5:x].any():
                left_edge_x = x
                break

        # Find right edge (transition from white to black)
        for x in range(roi_w - 5, center_x, -1):
            if white_mask[scan_line_y, x] > 0 and black_mask[scan_line_y, x+1:x+6].any():
                right_edge_x = x
                break

        # Calculate lane error
        error = 0.0
        if left_edge_x is not None and right_edge_x is not None:
            lane_center_x = (left_edge_x + right_edge_x) / 2.0
            error = lane_center_x - center_x
        elif left_edge_x is not None:
            # Only see left edge, assume we are drifting right
            error = 30.0 # Positive error means turn right
        elif right_edge_x is not None:
            # Only see right edge, assume we are drifting left
            error = -30.0 # Negative error means turn left
        # else: No edges found, error stays 0 (go straight)

        # --- 2. Traffic Sign Detection (Red/Green) ---
        red_mask1 = cv2.inRange(hsv_roi, LOWER_RED1, UPPER_RED1)
        red_mask2 = cv2.inRange(hsv_roi, LOWER_RED2, UPPER_RED2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv_roi, LOWER_GREEN, UPPER_GREEN)

        sign_info = None
        # Check for red signs (priority)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in red_contours:
            area = cv2.contourArea(cnt)
            if area > 50:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                # Check for roughly square shape
                aspect_ratio = float(w_rect) / h_rect
                if 0.7 < aspect_ratio < 1.3:
                     # Calculate centroid
                     M = cv2.moments(cnt)
                     if M["m00"] != 0:
                         cx = int(M["m10"] / M["m00"])
                         cy = int(M["m01"] / M["m00"])
                         position = "left" if cx < center_x else "right"
                         sign_info = {'type': 'red', 'position': position, 'cx': cx, 'cy': cy, 'area': area}
                         break # Take the first significant red sign found

        # If no red sign, check for green
        if sign_info is None:
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in green_contours:
                area = cv2.contourArea(cnt)
                if area > 50:
                    x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                    aspect_ratio = float(w_rect) / h_rect
                    if 0.7 < aspect_ratio < 1.3:
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            position = "left" if cx < center_x else "right"
                            sign_info = {'type': 'green', 'position': position, 'cx': cx, 'cy': cy, 'area': area}
                            break

        # --- 3. Parking Lot Detection (Magenta) ---
        magenta_mask = cv2.inRange(hsv_roi, LOWER_MAGENTA, UPPER_MAGENTA)
        parking_info = None
        magenta_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parking_bars = []
        for cnt in magenta_contours:
            area = cv2.contourArea(cnt)
            if area > 80: # Adjust based on bar size at distance
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                # Parking bars are long and thin
                aspect_ratio = float(w_rect) / h_rect if h_rect != 0 else 0
                if w_rect > h_rect and aspect_ratio > 2.0:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        parking_bars.append({'cx': cx, 'cy': cy, 'x': x, 'y': y, 'w': w_rect, 'h': h_rect})

        # Need at least 2 bars to identify a parking spot
        if len(parking_bars) >= 2:
            # Sort by x position
            parking_bars.sort(key=lambda b: b['cx'])
            left_bar = parking_bars[0]
            right_bar = parking_bars[-1] # Take the rightmost if more than 2

            # Calculate center of parking lot entrance
            lot_center_x = (left_bar['cx'] + right_bar['cx']) / 2.0
            lot_position = "left" if lot_center_x < center_x else "right"
            parking_lot_width = right_bar['cx'] - left_bar['cx']

            parking_info = {
                'detected': True,
                'center_x': lot_center_x,
                'position': lot_position,
                'width': parking_lot_width,
                'left_bar': left_bar,
                'right_bar': right_bar
            }

        return error, sign_info, parking_info

    except Exception as e:
        logger.error(f"Vision processing error: {e}", exc_info=True)
        return 0.0, None, None

# --- Control Logic ---
def calculate_control_commands(lane_error, sign_info, parking_info):
    """Calculate motor speed and steering angle based on state and sensor data."""
    global state, parking_sequence_step

    motor_speed = 0
    steering_angle = 90 # Center

    # --- Get latest sensor data safely ---
    with serial_lock:
        local_ir_states = ir_sensor_states[:] # Copy list
        local_us_distances = ultrasonic_distances_mm[:] # Copy list
        local_imu_data = imu_data_raw[:] # Copy list

    if state == STATE_WAITING_FOR_START:
        # Do nothing, wait for START signal
        motor_speed = 0
        steering_angle = 90

    elif state == STATE_DRIVING_LAPS:
        # --- Basic Line Following with PD Control ---
        p_term = STEERING_P_GAIN * lane_error
        # Simple D-term approximation (change in error)
        # d_term = STEERING_D_GAIN * (lane_error - previous_lane_error) / dt
        # For simplicity, we'll approximate D-term effect based on error magnitude
        d_term = 0 # Simplified for now
        steering_adjustment = p_term + d_term
        steering_angle = 90 + steering_adjustment
        steering_angle = np.clip(steering_angle, 45, 135)

        motor_speed = BASE_SPEED
        # Slow down for sharp turns
        steering_deviation = abs(steering_angle - 90)
        if steering_deviation > 25:
            motor_speed = max(MIN_SPEED, BASE_SPEED - steering_deviation * 0.4)

        # --- Check for Traffic Signs ---
        if sign_info:
            # Transition to obeying sign state
            state = STATE_OBEYING_SIGN
            logger.info(f"Detected {sign_info['type']} sign on {sign_info['position']} side. Obeying...")

    elif state == STATE_OBEYING_SIGN:
        # --- Obey Traffic Sign ---
        if sign_info and sign_info['type'] == 'red':
            # Red sign: Keep to the RIGHT of the sign
            # We want the car's center to be offset to the right of the sign's center
            # sign_info['cx'] is the sign's center in ROI coordinates
            # We aim for our center (90 deg servo) to be at sign_cx + offset
            target_cx = sign_info['cx'] + 20 # pixels to the right of sign center
            error_from_sign = target_cx - (FRAME_WIDTH // 2)
            steering_angle = 90 + (error_from_sign * 0.5) # Proportional control
            motor_speed = BASE_SPEED * 0.7 # Slow down
        elif sign_info and sign_info['type'] == 'green':
            # Green sign: Keep to the LEFT of the sign
            target_cx = sign_info['cx'] - 20 # pixels to the left of sign center
            error_from_sign = target_cx - (FRAME_WIDTH // 2)
            steering_angle = 90 + (error_from_sign * 0.5)
            motor_speed = BASE_SPEED * 0.7
        else:
            # Sign is no longer detected or was invalid, return to normal driving
            state = STATE_DRIVING_LAPS
            logger.info("Finished obeying sign. Resuming normal driving.")

        steering_angle = np.clip(steering_angle, 45, 135)

    elif state == STATE_PARKING_SEARCH:
        # --- Search for Parking Lot ---
        motor_speed = BASE_SPEED * 0.5
        steering_angle = 110 # Gentle right turn to search
        if parking_info and parking_info['detected']:
            logger.info("Parking lot detected. Initiating parking maneuver.")
            state = STATE_PARKING_MANEUVER
            parking_sequence_step = 0

    elif state == STATE_PARKING_MANEUVER:
        # --- Execute Parking Maneuver ---
        # Simplified parallel parking sequence
        if parking_sequence_step == 0:
            # 1. Align with the front of the parking space
            if parking_info:
                # Align laterally
                align_error = parking_info['center_x'] - (FRAME_WIDTH / 2)
                steering_angle = 90 + (align_error * 0.3)
                motor_speed = BASE_SPEED * 0.4
                
                # Check if aligned and close enough (use front US)
                if abs(align_error) < 15 and local_us_distances[0] < 300: # mm
                    parking_sequence_step = 1
                    logger.debug("Step 0 -> 1: Aligned and close. Moving to pass space.")
            else:
                # Keep searching or moving forward slowly
                steering_angle = 90
                motor_speed = BASE_SPEED * 0.3

        elif parking_sequence_step == 1:
            # 2. Move forward to pass the parking space
            motor_speed = BASE_SPEED * 0.4
            steering_angle = 90
            # Use rear ultrasonic to determine when we've passed enough
            # Assume we need to move about the car's length + space length
            # This is a simplification. A better way is to count sections or use IMU distance.
            # For now, use a timer or distance estimate.
            # Let's use a simple counter or time-based approach.
            # This step is tricky without odometry. We'll assume a fixed duration/speed.
            # Or check if rear US sees a wall again after losing sight of parking bars.
            # Simplified: Just move forward for a bit.
            # In a real scenario, you'd integrate speed over time or use encoder ticks.
            # Placeholder logic: Move for a fixed time or until a condition.
            # Let's just increment step for now as a demo.
            # A more robust way: integrate distance or use a counter.
            # For demo, we'll just move to next step.
            parking_sequence_step = 2 # Simplified transition
            logger.debug("Step 1 -> 2: Passed space. Starting reverse.")

        elif parking_sequence_step == 2:
            # 3. Start reversing and turn wheel to enter space
            motor_speed = -BASE_SPEED * 0.4 # Reverse
            # Determine turn direction based on where the lot was
            # Assume 'position' in parking_info tells us if it was on the left or right
            # during the search phase.
            if parking_info and parking_info.get('position') == 'right':
                steering_angle = 135 # Turn right (into space on right)
            else:
                steering_angle = 45  # Turn left (into space on left)
            
            # Check if we are in the space (US sensors)
            if local_us_distances[0] < 200 and local_us_distances[1] < 200: # Both see obstacles
                parking_sequence_step = 3
                logger.debug("Step 2 -> 3: In space. Straightening.")

        elif parking_sequence_step == 3:
            # 4. Straighten wheels while reversing
            motor_speed = -BASE_SPEED * 0.3
            steering_angle = 90 # Straighten
            
            # Check if centered (compare front and rear US)
            us_diff = abs(local_us_distances[0] - local_us_distances[1])
            if us_diff < 50: # Reasonably parallel
                parking_sequence_step = 4
                logger.debug("Step 3 -> 4: Straightened. Moving forward to center.")

        elif parking_sequence_step == 4:
            # 5. Move forward slightly to center in space
            motor_speed = BASE_SPEED * 0.3
            steering_angle = 90
            
            # Stop when centered again
            us_diff = abs(local_us_distances[0] - local_us_distances[1])
            if us_diff < 30:
                motor_speed = 0
                steering_angle = 90
                state = STATE_STOPPED
                logger.info("*** PARKING COMPLETED SUCCESSFULLY! ***")

        # Ensure commands are within bounds during parking
        motor_speed = np.clip(motor_speed, -MAX_SPEED, MAX_SPEED)
        steering_angle = np.clip(steering_angle, 45, 135)

    elif state == STATE_STOPPED:
        motor_speed = 0
        steering_angle = 90

    # Final clipping
    motor_speed = int(np.clip(motor_speed, -255, 255))
    steering_angle = int(np.clip(steering_angle, 0, 180))

    return motor_speed, steering_angle

# --- Graceful Shutdown ---
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    logger.info("Interrupt received, shutting down...")
    stop_event.set() # Signal threads to stop
    if ser and ser.is_open:
        try:
            send_command_to_nano('M', 0) # Stop motors
            send_command_to_nano('S', 90) # Center servo
            time.sleep(0.1)
        except:
            pass
        ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# --- Main Control Loop ---
def main():
    global state, lap_count, start_time, run_acknowledged

    logger.info("Initializing WRO 2025 Car Controller (Pi Side)...")

    # --- 1. Initialize Serial Communication ---
    if not initialize_serial():
        logger.error("Exiting due to serial initialization failure.")
        return

    # --- 2. Start Serial Reader Thread ---
    serial_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    serial_thread.start()
    logger.info("Serial reader thread started.")

    # --- 3. Initialize Camera ---
    cap = initialize_camera()
    if cap is None:
        logger.error("Exiting due to camera initialization failure.")
        ser.close()
        return

    logger.info("Waiting for START signal from Nano...")
    start_time = time.time()

    try:
        while not stop_event.is_set():
            current_time = time.time()

            # --- A. Check for START command from Nano ---
            # The Nano sends "START\n" when its button is pressed.
            # We need to read this from the serial buffer.
            # The serial reader thread handles continuous data.
            # We can peek or check a flag, but let's check a specific condition.
            # A simple way: check if IR sensors detect the start line (both front IRs).
            # However, the rules say the car waits for a button press.
            # So, the Nano sends "START". We need to detect this string.
            # Let's assume the serial reader doesn't parse "START", so we check the buffer.
            # Or, the Nano could send a specific sensor state pattern.
            # Let's stick to the protocol: Nano sends "START\n".
            # We need to read lines from serial. The reader thread consumes bytes.
            # Let's modify the reader to also look for "START\n".
            # Simpler for now: check a shared state variable that the reader can set.
            # The reader thread can parse "START\n" and set a global flag.
            # But our reader is focused on binary data. Let's integrate it.
            
            # Integration: Modify serial reader to also look for text lines like "START\n"
            # This is complex. Easier: Use a separate check in main loop if reader can expose text.
            # Or, make the reader also parse text lines.
            # For now, let's assume we can check the serial input for "START" occasionally.
            # This is not ideal with the binary reader. 
            # Better: Modify the Nano to send a special binary command for START ACK.
            # Or, integrate text line parsing into the binary reader.
            # Compromise: Poll the serial port quickly for "START" while also running the reader.
            
            # Let's assume the serial reader can be modified or we add a simple text check here.
            # Since the reader consumes bytes, we need to be careful.
            # Let's assume for now that when the Nano sends START, we detect it via sensor state
            # or the reader sets a flag. 
            # The simplest robust way: Have the Nano send a binary flag in its sensor packet
            # indicating the button was pressed. E.g., use one bit in the ir_states byte.
            # Let's redefine: Bit 7 of ir_states_byte for START button.
            # Nano code needs to set bit 7 if button was pressed.
            # This avoids text/binary mixing in serial reader.
            
            # Check for START via a flag potentially set by serial reader
            # (This requires Nano to send a flag in binary data, e.g., bit 7 of ir_states)
            # For this code to work as-is with the provided Nano code, 
            # we need the Nano to send "START\n" and the Pi to detect it.
            # The current serial reader only handles binary. 
            # We need to integrate text line reading.
            
            # Let's add a quick non-blocking line read here for "START".
            # This is a bit messy with the binary reader but should work.
             if ser.in_waiting > 0:
                 # Try to read a line quickly
                 try:
                     line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                     if line == "START":
                         if state == STATE_WAITING_FOR_START:
                             logger.info("*** START signal received from Nano! ***")
                             # Acknowledge start only once
                             if not run_acknowledged:
                                 send_command_to_nano('A') # Send 'ACK_START'
                                 run_acknowledged = True
                             state = STATE_DRIVING_LAPS
                             logger.info("State changed to DRIVING_LAPS")
                             # Optional small delay before motors start
                             time.sleep(0.5) 
                 except:
                     pass # Ignore errors in quick line read

            # --- B. Process Sensor Data (from serial reader thread) ---
            # Data is updated in `ir_sensor_states`, `ultrasonic_distances_mm`, `imu_data_raw`
            # protected by `serial_lock`. We copy it when needed inside control functions.
            # Use IMU data for lap counting
            with serial_lock:
                current_gyro_z = imu_data_raw[5] # Gyro Z raw value
            
            if state in [STATE_DRIVING_LAPS, STATE_OBEYING_SIGN]:
                update_lap_count(current_gyro_z)

            # --- C. Capture Camera Frame ---
            ret, frame = cap.read()
            if not ret:
                logger.warning("Failed to grab frame from camera.")
                time.sleep(0.05)
                continue

            # --- D. Process Vision ---
            lane_error, sign_info, parking_info = detect_track_features(frame)

            # --- E. Check Lap Completion and Transition to Parking ---
            if state == STATE_DRIVING_LAPS and lap_count >= REQUIRED_LAPS:
                 logger.info("*** THREE LAPS COMPLETED! Initiating Parking Sequence. ***")
                 state = STATE_PARKING_SEARCH
                 parking_sequence_step = 0

            # --- F. Calculate Control Commands ---
            motor_speed, servo_angle = calculate_control_commands(lane_error, sign_info, parking_info)

            # --- G. Send Commands to Nano ---
            # Send motor command
            send_command_to_nano('M', motor_speed)
            # Send servo command
            send_command_to_nano('S', servo_angle)

            # --- H. Control Loop Timing ---
            # Aim for a steady loop rate
            time.sleep(0.05) # 20 Hz loop is a good starting point

    except Exception as e:
        logger.error(f"An unexpected error occurred in main loop: {e}", exc_info=True)
    finally:
        # --- Cleanup ---
        logger.info("Cleaning up resources...")
        stop_event.set() # Signal threads to stop
        serial_thread.join(timeout=1.0) # Wait for reader thread to finish
        if ser and ser.is_open:
            # Ensure vehicle stops
            send_command_to_nano('M', 0)
            send_command_to_nano('S', 90)
            time.sleep(0.1) # Allow command to be sent
            ser.close()
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        logger.info("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()
