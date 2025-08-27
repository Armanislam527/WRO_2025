#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import logging
from collections import deque
import cv2
import numpy as np
import math

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
CAMERA_INDEX = 2
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_RATE = 15

# State constants
STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_PARKING = "PARKING"
STATE_STOPPED = "STOPPED"

# Lap and turn counting
REQUIRED_TURNS_PER_LAP = 4
REQUIRED_LAPS = 3
TURN_THRESHOLD_DEGREES = 70
GYRO_INTEGRAL_WINDOW_SIZE = 5

# Control parameters
BASE_SPEED = 70
MAX_SPEED = 100
MIN_SPEED = 30
STEERING_P_GAIN = 0.8
STEERING_D_GAIN = 0.3

# Color detection parameters
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([179, 255, 255])
LOWER_GREEN = np.array([40, 100, 100])
UPPER_GREEN = np.array([80, 255, 255])
LOWER_MAGENTA = np.array([130, 100, 100])
UPPER_MAGENTA = np.array([160, 255, 255])
MIN_SIGN_AREA = 100

# Edge detection parameters
CANNY_THRESHOLD1 = 50
CANNY_THRESHOLD2 = 150
HOUGH_THRESHOLD = 30
MIN_LINE_LENGTH = 30
MAX_LINE_GAP = 10

# Global variables
state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE)
parking_sequence_step = 0
start_time = None
driving_direction = 1  # 1 for forward, -1 for backward
last_corner_detection_time = 0
corner_detection_cooldown = 2.0  # seconds

# Sensor data
ir_sensor_states = [1, 1, 1, 1]
ultrasonic_distances_mm = [9999, 9999]
imu_data_raw = [0, 0, 0, 0, 0, 0]

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Hardware objects
ser = None
cap = None

def initialize_serial():
    global ser
    try:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        logger.info(f"Connected to Arduino Nano on {SERIAL_PORT}")
        return True
    except Exception as e:
        logger.error(f"Serial initialization failed: {e}")
        return False
def initialize_camera():
    global cap
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            logger.error("Could not open USB camera")
            return False
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
        time.sleep(0.1)
        
        logger.info(f"Camera initialized: {FRAME_WIDTH}x{FRAME_HEIGHT}")
        return True
    except Exception as e:
        logger.error(f"Camera initialization failed: {e}")
        return False

def send_command_to_nano(command_str):
    global ser
    # Add to your main loop:
    if ser is None or not ser.is_open:
        if time.time() - last_serial_attempt > 5.0:  # Try every 5 seconds
            logger.info("Attempting to reconnect to Arduino...")
            if initialize_serial():
                # Reset state after reconnection
                state = STATE_WAITING_FOR_START
                lap_count = 0
                turn_count = 0
            last_serial_attempt = time.time()
        continue
    
    try:
        ser.write((command_str + '\n').encode('utf-8'))
        logger.debug(f"Sent: {command_str}")
    except Exception as e:
        logger.error(f"Serial write error: {e}")
        try:
            ser.close()
            ser = None
        except:
            pass
    # In Pi main loop:
    last_heartbeat = time.time()
    heartbeat_interval = 1.0  # Send heartbeat every second
    
    if time.time() - last_heartbeat > heartbeat_interval:
        send_command_to_nano("PING")
        last_heartbeat = time.time()

def parse_sensor_data(data_str):
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw
    try:
        # Clean the input string
        data_str = data_str.strip()
        if not data_str.startswith("SENSORS"):
            return
            
        parts = data_str.split(',')
        if len(parts) >= 13:
            # Validate all values are integers
            ir_sensor_states = [int(parts[i]) for i in range(1, 5)]
            ultrasonic_distances_mm = [int(parts[i]) for i in range(5, 7)]
            imu_data_raw = [int(parts[i]) for i in range(7, 13)]
        else:
            logger.warning(f"Incomplete sensor data: {data_str}")
    except Exception as e:
        logger.warning(f"Sensor data parsing error: {e}, Data: {data_str}")
def detect_signs(hsv_frame):
    # Detect red signs
    red_mask1 = cv2.inRange(hsv_frame, LOWER_RED1, UPPER_RED1)
    red_mask2 = cv2.inRange(hsv_frame, LOWER_RED2, UPPER_RED2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    # Detect green signs
    green_mask = cv2.inRange(hsv_frame, LOWER_GREEN, UPPER_GREEN)
    
    # Detect magenta parking zones
    magenta_mask = cv2.inRange(hsv_frame, LOWER_MAGENTA, UPPER_MAGENTA)
    
    # Find contours
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    magenta_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    sign_info = None
    parking_info = None
    
    # Process red signs (must pass on right)
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > MIN_SIGN_AREA:
            x, y, w, h = cv2.boundingRect(contour)
            sign_info = {'type': 'red', 'x': x, 'y': y, 'width': w, 'height': h, 'area': area}
            break
    
    # Process green signs (must pass on left)
    if sign_info is None:
        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area > MIN_SIGN_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                sign_info = {'type': 'green', 'x': x, 'y': y, 'width': w, 'height': h, 'area': area}
                break
    
    # Process magenta parking zones
    parking_bars = []
    for contour in magenta_contours:
        area = cv2.contourArea(contour)
        if area > MIN_SIGN_AREA:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            # Look for rectangular shapes (parking zone markers)
            if aspect_ratio > 2.0 or aspect_ratio < 0.5:
                parking_bars.append({'x': x, 'y': y, 'width': w, 'height': h, 'area': area})
    
    if len(parking_bars) >= 2:
        # Sort by x position and determine parking side
        parking_bars.sort(key=lambda b: b['x'])
        left_bar = parking_bars[0]
        right_bar = parking_bars[-1]
        
        parking_info = {
            'detected': True,
            'left_bar': left_bar,
            'right_bar': right_bar,
            'center_x': (left_bar['x'] + right_bar['x'] + left_bar['width'] + right_bar['width']) / 2,
            'side': 'left' if left_bar['x'] < hsv_frame.shape[1] / 2 else 'right'
        }
            
    return sign_info, parking_info

def detect_edges_and_corners(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
    
    # Detect lines using Hough Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESHOLD, minLineLength=MIN_LINE_LENGTH, maxLineGap=MAX_LINE_GAP)
    
    # Detect corners using Harris corner detection
    gray_float = np.float32(gray)
    corners = cv2.cornerHarris(gray_float, 2, 3, 0.04)
    corners = cv2.dilate(corners, None)
    
    return edges, lines, corners

def calculate_navigation_error(edges, lines, frame_width):
    if lines is None:
        return 0.0
    
    left_lines = []
    right_lines = []
    center_x = frame_width // 2
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 != x1:
            slope = (y2 - y1) / (x2 - x1)
            line_center = (x1 + x2) / 2
            if line_center < center_x - 20:
                left_lines.append(line[0])
            elif line_center > center_x + 20:
                right_lines.append(line[0])
    
    # Calculate error from edge following
    error = 0.0
    y_level = edges.shape[0] * 0.7
    left_x = []
    right_x = []
    
    for line in left_lines:
        x1, y1, x2, y2 = line
        if min(y1, y2) <= y_level <= max(y1, y2):
            x_at_y = x1 + (y_level - y1) * (x2 - x1) / (y2 - y1)
            left_x.append(x_at_y)
    
    for line in right_lines:
        x1, y1, x2, y2 = line
        if min(y1, y2) <= y_level <= max(y1, y2):
            x_at_y = x1 + (y_level - y1) * (x2 - x1) / (y2 - y1)
            right_x.append(x_at_y)
    
    if left_x and right_x:
        left_avg = np.mean(left_x)
        right_avg = np.mean(right_x)
        error = (left_avg + right_avg) / 2 - center_x
    elif left_x:
        error = 50.0
    elif right_x:
        error = -50.0
    
    return error

def detect_corner(corners, threshold=0.01):
    if corners is None:
        return False
    
    # Count the number of strong corners
    strong_corners = np.sum(corners > threshold * corners.max())
    return strong_corners > 10  # Adjust threshold as needed

def process_camera_frame(frame):
    if frame is None:
        return 0.0, None, None, False

    try:
        height, width = frame.shape[:2]
        roi = frame[int(height*0.4):height, 0:width]  # Use more of the frame for better edge detection
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Detect signs and parking zones
        sign_info, parking_info = detect_signs(hsv)
        
        # Detect edges and corners
        edges, lines, corners = detect_edges_and_corners(roi)
        
        # Calculate navigation error
        error = calculate_navigation_error(edges, lines, width)
        
        # Check for corners
        corner_detected = detect_corner(corners)
        
        return error, sign_info, parking_info, corner_detected
        
    except Exception as e:
        logger.error(f"Frame processing error: {e}")
        return 0.0, None, None, False

def calculate_control_commands(error, sign_info, parking_info, corner_detected):
    global state, parking_sequence_step, driving_direction, last_corner_detection_time

    motor_speed = 0
    servo_angle = 90

    try:
        if state == STATE_DRIVING_LAPS:
            # Adjust steering based on edge following error
            steering = STEERING_P_GAIN * error
            servo_angle = int(np.clip(90 + steering, 45, 135))
            motor_speed = BASE_SPEED * driving_direction
            
            # Adjust for signs
            if sign_info:
                if sign_info['type'] == 'red':
                    # Red sign - must pass on right
                    if sign_info['x'] < width/2:  # Sign is on left
                        servo_angle = min(servo_angle + 20, 135)  # Steer right
                    # else: sign is on right, no adjustment needed
                    
                elif sign_info['type'] == 'green':
                    # Green sign - must pass on left
                    if sign_info['x'] > width/2:  # Sign is on right
                        servo_angle = max(servo_angle - 20, 45)  # Steer left
                    # else: sign is on left, no adjustment needed
            
            # Handle corner detection for lap counting
            current_time = time.time()
            if corner_detected and (current_time - last_corner_detection_time) > corner_detection_cooldown:
                last_corner_detection_time = current_time
                return motor_speed, servo_angle, True  # Signal corner detected
            
        elif state == STATE_PARKING:
            if parking_info:
                parking_side = parking_info['side']
                
                if parking_sequence_step == 0:
                    # Position the car for parking
                    motor_speed = BASE_SPEED / 2
                    if parking_side == 'right':
                        servo_angle = 60  # Steer slightly left to position for right-side parking
                    else:
                        servo_angle = 120  # Steer slightly right to position for left-side parking
                    parking_sequence_step += 1
                    
                elif parking_sequence_step == 1:
                    # Reverse into parking spot
                    motor_speed = -BASE_SPEED / 2
                    if parking_side == 'right':
                        servo_angle = 135  # Steer right for right-side parking
                    else:
                        servo_angle = 45  # Steer left for left-side parking
                    parking_sequence_step += 1
                    
                elif parking_sequence_step == 2:
                    # Straighten out
                    motor_speed = -BASE_SPEED / 3
                    servo_angle = 90
                    parking_sequence_step += 1
                    
                else:
                    motor_speed = 0
                    state = STATE_STOPPED
            else:
                # No parking info, use default sequence
                if parking_sequence_step == 0:
                    motor_speed = BASE_SPEED / 2
                    parking_sequence_step += 1
                elif parking_sequence_step == 1:
                    motor_speed = -BASE_SPEED / 2
                    servo_angle = 135
                    parking_sequence_step += 1
                else:
                    motor_speed = 0
                    state = STATE_STOPPED
                
        elif state == STATE_STOPPED:
            motor_speed = 0
            
        motor_speed = int(np.clip(motor_speed, -255, 255))
        
    except Exception as e:
        logger.error(f"Control calculation error: {e}")
        motor_speed = 0
        servo_angle = 90
        
    return motor_speed, servo_angle, False

def update_lap_count(gyro_z_rate, corner_detected):
    global turn_count, last_turn_integral, gyro_z_history, lap_count, driving_direction

    try:
        # Use both IMU data and corner detection for lap counting
        gyro_z_dps = gyro_z_rate * 0.061
        gyro_z_history.append(gyro_z_dps)
        
        dt = 0.1
        current_integral = sum(gyro_z_history) * dt / len(gyro_z_history) if gyro_z_history else 0
        turn_angle = current_integral - last_turn_integral
        last_turn_integral = current_integral

        # Check if a turn is detected by IMU or vision
        turn_detected = (abs(turn_angle) > TURN_THRESHOLD_DEGREES) or corner_detected
        
        if turn_detected and turn_count < REQUIRED_TURNS_PER_LAP:
            turn_count += 1
            logger.info(f"Turn {turn_count}/{REQUIRED_TURNS_PER_LAP} detected")

        if turn_count >= REQUIRED_TURNS_PER_LAP:
            lap_count += 1
            turn_count = 0
            last_turn_integral = 0.0
            gyro_z_history.clear()
            logger.info(f"Lap {lap_count}/{REQUIRED_LAPS} completed")
            
            # Reverse direction for the next lap if needed
            driving_direction *= -1
            logger.info(f"Changing driving direction to {'forward' if driving_direction > 0 else 'reverse'}")
            
            if lap_count >= REQUIRED_LAPS:
                logger.info("All laps completed")
                
    except Exception as e:
        logger.error(f"Lap counting error: {e}")

def main():
    global state, lap_count, start_time, ser, cap, driving_direction

    logger.info("Initializing WRO 2025 Car Controller")

    if not initialize_serial():
        logger.error("Serial initialization failed")
        return
        
    if not initialize_camera():
        logger.error("Camera initialization failed")
        if ser and ser.is_open:
            ser.close()
        return

    start_time = time.time()
    logger.info("Starting main control loop")

    last_frame_time = time.time()
    frame_interval = 1.0 / FRAME_RATE

    try:
        while True:
            current_time = time.time()
            
            try:
                if ser and ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').rstrip()
                    if line:
                        logger.debug(f"Received from Nano: {line}")
                        if line == "START" and state == STATE_WAITING_FOR_START:
                            logger.info("Start signal received")
                            send_command_to_nano("ACK_START")
                            state = STATE_DRIVING_LAPS
                            driving_direction = 1  # Start in forward direction
                        elif line.startswith("SENSORS"):
                            parse_sensor_data(line)
            except Exception as e:
                logger.error(f"Serial read error: {e}")
                try:
                    if ser and ser.is_open:
                        ser.close()
                    ser = None
                    time.sleep(1)
                    initialize_serial()
                except:
                    pass

            if state == STATE_WAITING_FOR_START:
                time.sleep(0.1)
                continue

            if current_time - last_frame_time < frame_interval:
                time.sleep(0.001)
                continue
                
            last_frame_time = current_time

            try:
                ret, frame = cap.read()
                if not ret:
                    logger.warning("Failed to read frame")
                    time.sleep(0.1)
                    continue
                    
                error, sign_info, parking_info, corner_detected = process_camera_frame(frame)
                
                # Update lap count with both IMU and vision data
                if state == STATE_DRIVING_LAPS:
                    update_lap_count(imu_data_raw[5], corner_detected)
                
                if lap_count >= REQUIRED_LAPS and state == STATE_DRIVING_LAPS:
                    logger.info("Transitioning to parking state")
                    state = STATE_PARKING
                    parking_sequence_step = 0
                    
                motor_speed, servo_angle, _ = calculate_control_commands(error, sign_info, parking_info, corner_detected)
                
                send_command_to_nano(f"MOTOR,{motor_speed}")
                send_command_to_nano(f"SERVO,{servo_angle}")
                
            except Exception as e:
                logger.error(f"Frame processing error: {e}")
                time.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        logger.info("Cleaning up")
        try:
            send_command_to_nano("MOTOR,0")
            send_command_to_nano("SERVO,90")
        except:
            pass
            
        try:
            if cap and cap.isOpened():
                cap.release()
        except:
            pass
            
        try:
            if ser and ser.is_open:
                ser.close()
        except:
            pass
            
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()