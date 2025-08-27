#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import logging
from collections import deque
import cv2
import numpy as np

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
TURN_RESET_THRESHOLD_DEGREES = 20
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
MIN_SIGN_AREA = 100

# Global variables
state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE)
parking_sequence_step = 0
start_time = None

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
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
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
    if ser is None or not ser.is_open:
        return
    
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

def parse_sensor_data(data_str):
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw
    try:
        parts = data_str.split(',')
        if len(parts) >= 13 and parts[0] == "SENSORS":
            ir_sensor_states = [int(parts[i]) for i in range(1, 5)]
            ultrasonic_distances_mm = [int(parts[i]) for i in range(5, 7)]
            imu_data_raw = [int(parts[i]) for i in range(7, 13)]
    except Exception as e:
        logger.warning(f"Sensor data parsing error: {e}")

def detect_signs(hsv_frame):
    # Detect red signs
    red_mask1 = cv2.inRange(hsv_frame, LOWER_RED1, UPPER_RED1)
    red_mask2 = cv2.inRange(hsv_frame, LOWER_RED2, UPPER_RED2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    # Detect green signs
    green_mask = cv2.inRange(hsv_frame, LOWER_GREEN, UPPER_GREEN)
    
    # Find contours
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    sign_info = None
    
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
            
    return sign_info

def process_camera_frame(frame):
    if frame is None:
        return 0.0, None, None

    try:
        height, width = frame.shape[:2]
        roi = frame[int(height*0.6):height, 0:width]
        h, w = roi.shape[:2]
        center_x = w // 2
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Detect signs
        sign_info = detect_signs(hsv)
        
        # Line following
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, minLineLength=w*0.1, maxLineGap=w*0.05)
        
        error = 0.0
        if lines is not None:
            left_lines = []
            right_lines = []
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 != x1:
                    slope = (y2 - y1) / (x2 - x1)
                    line_center = (x1 + x2) / 2
                    if line_center < center_x - 20:
                        left_lines.append(line[0])
                    elif line_center > center_x + 20:
                        right_lines.append(line[0])
            
            # Calculate error
            y_level = h * 0.7
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
        
        # Parking lot detection (simplified)
        parking_lot_info = None
        
        return error, sign_info, parking_lot_info
        
    except Exception as e:
        logger.error(f"Frame processing error: {e}")
        return 0.0, None, None

def calculate_control_commands(error, sign_info, parking_lot_info):
    global state, parking_sequence_step

    motor_speed = 0
    servo_angle = 90

    try:
        if state == STATE_DRIVING_LAPS:
            steering = STEERING_P_GAIN * error
            servo_angle = int(np.clip(90 + steering, 45, 135))
            motor_speed = BASE_SPEED
            
            # Adjust for signs
            if sign_info:
                if sign_info['type'] == 'red':
                    # Red sign - must pass on right
                    if sign_info['x'] < FRAME_WIDTH/2:  # Sign is on left
                        servo_angle = min(servo_angle + 20, 135)  # Steer right
                    # else: sign is on right, no adjustment needed
                    
                elif sign_info['type'] == 'green':
                    # Green sign - must pass on left
                    if sign_info['x'] > FRAME_WIDTH/2:  # Sign is on right
                        servo_angle = max(servo_angle - 20, 45)  # Steer left
                    # else: sign is on left, no adjustment needed
            
        elif state == STATE_PARKING:
            if parking_sequence_step == 0:
                motor_speed = BASE_SPEED
                parking_sequence_step += 1
            elif parking_sequence_step == 1:
                motor_speed = -BASE_SPEED
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
        
    return motor_speed, servo_angle

def update_lap_count(gyro_z_rate):
    global turn_count, last_turn_integral, gyro_z_history, lap_count

    try:
        gyro_z_dps = gyro_z_rate * 0.061
        gyro_z_history.append(gyro_z_dps)
        
        dt = 0.1
        current_integral = sum(gyro_z_history) * dt / len(gyro_z_history) if gyro_z_history else 0
        turn_angle = current_integral - last_turn_integral
        last_turn_integral = current_integral

        if abs(turn_angle) > TURN_THRESHOLD_DEGREES and turn_count < REQUIRED_TURNS_PER_LAP:
            turn_count += 1
            logger.info(f"Turn {turn_count}/{REQUIRED_TURNS_PER_LAP}")

        if turn_count >= REQUIRED_TURNS_PER_LAP:
            lap_count += 1
            turn_count = 0
            last_turn_integral = 0.0
            gyro_z_history.clear()
            logger.info(f"Lap {lap_count}/{REQUIRED_LAPS} completed")
            
            if lap_count >= REQUIRED_LAPS:
                logger.info("All laps completed")
                
    except Exception as e:
        logger.error(f"Lap counting error: {e}")

def main():
    global state, lap_count, start_time, ser, cap

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
                        if line == "START" and state == STATE_WAITING_FOR_START:
                            logger.info("Start signal received")
                            send_command_to_nano("ACK_START")
                            state = STATE_DRIVING_LAPS
                        elif line.startswith("SENSORS"):
                            parse_sensor_data(line)
                            if state == STATE_DRIVING_LAPS:
                                update_lap_count(imu_data_raw[5])
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
                    
                error, sign_info, parking_lot_info = process_camera_frame(frame)
                
                if lap_count >= REQUIRED_LAPS and state == STATE_DRIVING_LAPS:
                    logger.info("Transitioning to parking state")
                    state = STATE_PARKING
                    parking_sequence_step = 0
                    
                motor_speed, servo_angle = calculate_control_commands(error, sign_info, parking_lot_info)
                
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