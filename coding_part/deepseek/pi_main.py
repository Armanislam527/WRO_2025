#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WRO 2025 Future Engineers - Self-Driving Car Controller
Optimized for Raspberry Pi Zero 2W
"""

import serial
import time
import logging
import struct
import numpy as np
import cv2
from collections import deque
import math

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 500000  # Match Nano's baud rate

# Camera settings optimized for Pi Zero 2W
CAMERA_INDEX = 0
FRAME_WIDTH = 160
FRAME_HEIGHT = 120
FRAME_RATE = 10

# Competition parameters
REQUIRED_LAPS = 3
TRACK_WIDTH = 1000  # mm (will vary based on randomization)
VEHICLE_LENGTH = 150  # mm (approximate)

# HSV Color Ranges (optimized for WRO 2025 field)
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

# Control parameters
BASE_SPEED = 70
MAX_SPEED = 100
MIN_SPEED = 30
STEERING_P_GAIN = 0.8
STEERING_D_GAIN = 0.3
LANE_KEEP_OFFSET = 50  # mm offset from center for lane keeping

# State Machine States
STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_OBEYING_SIGN = "OBEYING_SIGN"
STATE_PARKING = "PARKING"
STATE_STOPPED = "STOPPED"

# --- Global Variables ---
state = STATE_WAITING_FOR_START
lap_count = 0
current_section = 0
last_section = -1
section_enter_time = 0
direction = 1  # 1 for clockwise, -1 for counter-clockwise
parking_sequence_step = 0
start_time = None

# Sensor Data Storage
ir_sensor_states = [1, 1, 1, 1]  # FL, FR, R45, L45
ultrasonic_distances_mm = [9999, 9999]  # Front, Rear
imu_data = [0, 0, 0, 0, 0, 0]  # AccelX, Y, Z; GyroX, Y, Z

# Lap counting with IMU
imu_calibration_samples = 100
imu_calibrated = False
gyro_bias = [0, 0, 0]
angle_z = 0
last_imu_time = None
section_angles = []  # Store angles when entering sections for lap counting

# Path planning
planned_path = []
current_target_index = 0

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

# --- Communication with Nano ---
class NanoCommunicator:
    def __init__(self, port, baud_rate):
        self.ser = None
        self.port = port
        self.baud_rate = baud_rate
        self.connect()
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            logger.info(f"Connected to Arduino Nano on {self.port} at {self.baud_rate} baud.")
            # Flush any existing data
            self.ser.flushInput()
            self.ser.flushOutput()
        except Exception as e:
            logger.error(f"Failed to connect to serial port {self.port}: {e}")
            self.ser = None
            
    def read_sensors(self):
        if not self.ser or not self.ser.is_open:
            return False
            
        # Look for start byte
        while self.ser.in_waiting >= 21:  # Start byte + 19 data bytes + end byte
            byte = self.ser.read(1)
            if byte == b'\xaa':  # Start byte
                data = self.ser.read(19)  # Read sensor data
                end_byte = self.ser.read(1)  # Read end byte
                
                if end_byte == b'\x55':  # Valid packet
                    # Parse binary data
                    ir_states = data[0]
                    us_front = struct.unpack('<h', data[1:3])[0]
                    us_rear = struct.unpack('<h', data[3:5])[0]
                    accel_x = struct.unpack('<h', data[5:7])[0]
                    accel_y = struct.unpack('<h', data[7:9])[0]
                    accel_z = struct.unpack('<h', data[9:11])[0]
                    gyro_x = struct.unpack('<h', data[11:13])[0]
                    gyro_y = struct.unpack('<h', data[13:15])[0]
                    gyro_z = struct.unpack('<h', data[15:17])[0]
                    
                    # Update global variables
                    global ir_sensor_states, ultrasonic_distances_mm, imu_data
                    ir_sensor_states = [
                        (ir_states >> 0) & 1,  # FL
                        (ir_states >> 1) & 1,  # FR
                        (ir_states >> 2) & 1,  # R45
                        (ir_states >> 3) & 1   # L45
                    ]
                    ultrasonic_distances_mm = [us_front, us_rear]
                    imu_data = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
                    
                    return True
                    
        return False
        
    def send_command(self, command, value):
        if not self.ser or not self.ser.is_open:
            logger.warning("Serial port not available")
            return False
            
        try:
            cmd_str = f"{command},{value}\n"
            self.ser.write(cmd_str.encode('utf-8'))
            self.ser.flush()
            logger.debug(f"Sent command: {cmd_str.strip()}")
            return True
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            self.ser.close()
            self.ser = None
            return False
            
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# Initialize Nano communication
nano_com = NanoCommunicator(SERIAL_PORT, BAUD_RATE)

# --- Camera Setup ---
def initialize_camera():
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            logger.error("Could not open camera")
            return None
            
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
        
        # Allow camera to stabilize
        time.sleep(0.1)
        
        logger.info(f"Camera initialized: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {FRAME_RATE}fps")
        return cap
    except Exception as e:
        logger.error(f"Camera initialization failed: {e}")
        return None

# --- IMU Processing ---
def calibrate_imu():
    global imu_calibrated, gyro_bias, imu_calibration_samples
    
    logger.info("Calibrating IMU...")
    gyro_samples = []
    
    for _ in range(imu_calibration_samples):
        if nano_com.read_sensors():
            gyro_samples.append(imu_data[3:6])  # Gyro X, Y, Z
        time.sleep(0.01)
    
    if gyro_samples:
        gyro_bias = np.mean(gyro_samples, axis=0)
        imu_calibrated = True
        logger.info(f"IMU calibrated. Bias: {gyro_bias}")
    else:
        logger.warning("IMU calibration failed")

def update_orientation():
    global angle_z, last_imu_time
    
    if not imu_calibrated or not nano_com.read_sensors():
        return
        
    current_time = time.time()
    
    if last_imu_time is None:
        last_imu_time = current_time
        return
        
    dt = current_time - last_imu_time
    last_imu_time = current_time
    
    # Apply bias correction and integrate gyro Z
    gyro_z_corrected = imu_data[5] - gyro_bias[2]
    angle_z += gyro_z_corrected * dt * (math.pi / 180.0)  # Convert to radians
    
    # Keep angle between -pi and pi
    angle_z = (angle_z + math.pi) % (2 * math.pi) - math.pi

# --- Vision Processing ---
def detect_track_features(frame):
    """Detect track features with optimized processing for Pi Zero"""
    if frame is None:
        return None, None, None
        
    try:
        # Reduce processing area (focus on lower part of image)
        h, w = frame.shape[:2]
        roi = frame[int(h*0.6):h, 0:w]
        
        # Convert to HSV for color processing
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create masks for different colors
        white_mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
        black_mask = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)
        red_mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        red_mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        magenta_mask = cv2.inRange(hsv, LOWER_MAGENTA, UPPER_MAGENTA)
        
        # Find contours for signs and parking markers
        sign_info = detect_signs(red_mask, green_mask, w)
        parking_info = detect_parking(magenta_mask, w)
        
        # Calculate lane position error
        error = calculate_lane_error(white_mask, black_mask, w)
        
        return error, sign_info, parking_info
        
    except Exception as e:
        logger.error(f"Vision processing error: {e}")
        return 0, None, None

def detect_signs(red_mask, green_mask, frame_width):
    """Detect red and green traffic signs"""
    center_x = frame_width // 2
    sign_info = None
    
    # Process red signs
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in red_contours:
        area = cv2.contourArea(cnt)
        if area > 50:  # Minimum area threshold
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / h
            if 0.8 < aspect_ratio < 1.2:  # Square-ish aspect ratio
                sign_info = {
                    'type': 'red',
                    'position': 'left' if x + w/2 < center_x else 'right',
                    'x': x + w/2,
                    'y': y + h/2,
                    'confidence': min(area / 100, 1.0)
                }
                break
    
    # Process green signs if no red sign found
    if sign_info is None:
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in green_contours:
            area = cv2.contourArea(cnt)
            if area > 50:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / h
                if 0.8 < aspect_ratio < 1.2:
                    sign_info = {
                        'type': 'green',
                        'position': 'left' if x + w/2 < center_x else 'right',
                        'x': x + w/2,
                        'y': y + h/2,
                        'confidence': min(area / 100, 1.0)
                    }
                    break
                    
    return sign_info

def detect_parking(magenta_mask, frame_width):
    """Detect magenta parking markers"""
    center_x = frame_width // 2
    parking_info = None
    
    contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    markers = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:  # Minimum area for parking markers
            x, y, w, h = cv2.boundingRect(cnt)
            markers.append({
                'x': x + w/2,
                'y': y + h/2,
                'width': w,
                'height': h
            })
    
    # Need at least 2 markers to identify a parking spot
    if len(markers) >= 2:
        # Sort by x position
        markers.sort(key=lambda m: m['x'])
        
        # Check if markers form a parking spot (similar y, appropriate distance)
        if abs(markers[0]['y'] - markers[-1]['y']) < 20:  # Similar height
            parking_width = markers[-1]['x'] - markers[0]['x']
            if 150 < parking_width < 300:  # Reasonable parking width
                parking_info = {
                    'detected': True,
                    'x': (markers[0]['x'] + markers[-1]['x']) / 2,
                    'width': parking_width,
                    'position': 'left' if markers[0]['x'] < center_x else 'right'
                }
    
    return parking_info

def calculate_lane_error(white_mask, black_mask, frame_width):
    """Calculate lane position error for line following"""
    center_x = frame_width // 2
    
    # Find left and right lane edges
    left_edge = find_edge(white_mask, 'left')
    right_edge = find_edge(white_mask, 'right')
    
    if left_edge is not None and right_edge is not None:
        # Both edges detected - follow center
        lane_center = (left_edge + right_edge) / 2
        error = lane_center - center_x
    elif left_edge is not None:
        # Only left edge detected - stay at offset from left
        error = left_edge + LANE_KEEP_OFFSET - center_x
    elif right_edge is not None:
        # Only right edge detected - stay at offset from right
        error = right_edge - LANE_KEEP_OFFSET - center_x
    else:
        # No edges detected - continue straight
        error = 0
        
    return error

def find_edge(mask, side):
    """Find the left or right edge in the mask"""
    h, w = mask.shape[:2]
    search_bottom = h - 5  # Look near the bottom of the ROI
    
    if side == 'left':
        # Search from left to right
        for x in range(5, w):
            if mask[search_bottom, x] > 0:
                return x
    else:  # right side
        # Search from right to left
        for x in range(w-5, 5, -1):
            if mask[search_bottom, x] > 0:
                return x
                
    return None

# --- Lap Counting and Navigation ---
def update_lap_count():
    """Update lap count based on section transitions and IMU data"""
    global lap_count, current_section, last_section, section_enter_time, section_angles
    
    # Read sensor data
    if not nano_com.read_sensors():
        return
        
    # Detect section changes based on IR sensors (simplified)
    front_ir_active = ir_sensor_states[0] == 0 or ir_sensor_states[1] == 0
    
    if front_ir_active:
        # Check if we've entered a new section
        current_time = time.time()
        if current_time - section_enter_time > 1.0:  # Debounce
            current_section = (current_section + direction) % 8
            
            # Store angle when entering section for lap counting
            update_orientation()
            section_angles.append(angle_z)
            
            # Check for completed lap (passed through all 8 sections)
            if last_section == 7 and current_section == 0 and direction == 1:
                lap_count += 1
                logger.info(f"Lap {lap_count} completed!")
            elif last_section == 0 and current_section == 7 and direction == -1:
                lap_count += 1
                logger.info(f"Lap {lap_count} completed!")
                
            last_section = current_section
            section_enter_time = current_time

# --- Control Logic ---
def calculate_control_commands(vision_error, sign_info, parking_info):
    """Calculate motor and steering commands based on current state"""
    global state, parking_sequence_step
    
    motor_speed = BASE_SPEED
    steering_angle = 90  # Center position
    
    if state == STATE_DRIVING_LAPS:
        # Basic line following
        steering_angle = 90 + vision_error * STEERING_P_GAIN
        steering_angle = max(45, min(135, steering_angle))
        
        # Slow down in curves
        steering_deviation = abs(steering_angle - 90)
        if steering_deviation > 20:
            motor_speed = max(MIN_SPEED, BASE_SPEED - steering_deviation * 0.5)
            
        # Check for traffic signs
        if sign_info and sign_info['confidence'] > 0.7:
            state = STATE_OBEYING_SIGN
            logger.info(f"Detected {sign_info['type']} sign on {sign_info['position']} side")
            
    elif state == STATE_OBEYING_SIGN:
        # Obey traffic sign by moving to the correct side
        if sign_info['type'] == 'red':
            # Red sign - keep to right
            target_offset = -LANE_KEEP_OFFSET if sign_info['position'] == 'right' else LANE_KEEP_OFFSET
        else:  # green sign
            # Green sign - keep to left
            target_offset = LANE_KEEP_OFFSET if sign_info['position'] == 'left' else -LANE_KEEP_OFFSET
            
        steering_angle = 90 + target_offset * 0.5
        motor_speed = BASE_SPEED * 0.8  # Slow down when obeying signs
        
        # Return to normal driving after passing the sign
        if sign_info is None or sign_info['confidence'] < 0.3:
            state = STATE_DRIVING_LAPS
            
    elif state == STATE_PARKING:
        # Parking maneuver
        motor_speed, steering_angle = execute_parking_maneuver(parking_info)
        
    elif state == STATE_STOPPED:
        motor_speed = 0
        steering_angle = 90
        
    return motor_speed, steering_angle

def execute_parking_maneuver(parking_info):
    """Execute parallel parking maneuver"""
    global parking_sequence_step, state
    
    motor_speed = 0
    steering_angle = 90
    
    if parking_sequence_step == 0:
        # Position vehicle alongside parking space
        if parking_info and parking_info['detected']:
            # Move to align with parking space
            position_error = parking_info['x'] - (FRAME_WIDTH / 2)
            steering_angle = 90 + position_error * 0.5
            motor_speed = BASE_SPEED * 0.5
            
            if abs(position_error) < 20:  # Aligned
                parking_sequence_step = 1
        else:
            # Search for parking space
            steering_angle = 90 + 30  # Gentle turn
            motor_speed = BASE_SPEED * 0.5
            
    elif parking_sequence_step == 1:
        # Move forward to pass the parking space
        motor_speed = BASE_SPEED * 0.5
        steering_angle = 90
        # Use ultrasonic to determine when we've passed enough
        if ultrasonic_distances_mm[1] < 300:  # Close to rear obstacle
            parking_sequence_step = 2
            
    elif parking_sequence_step == 2:
        # Reverse into parking space with steering
        motor_speed = -BASE_SPEED * 0.4
        steering_angle = 135 if direction == 1 else 45  # Turn into space
        
        # Check if we're in the space using ultrasonics
        if ultrasonic_distances_mm[0] < 150 and ultrasonic_distances_mm[1] < 150:
            parking_sequence_step = 3
            
    elif parking_sequence_step == 3:
        # Straighten out
        motor_speed = -BASE_SPEED * 0.3
        steering_angle = 90
        
        # Check if centered
        if abs(ultrasonic_distances_mm[0] - ultrasonic_distances_mm[1]) < 50:
            parking_sequence_step = 4
            
    elif parking_sequence_step == 4:
        # Move forward to center in space
        motor_speed = BASE_SPEED * 0.3
        steering_angle = 90
        
        # Stop when centered
        if abs(ultrasonic_distances_mm[0] - ultrasonic_distances_mm[1]) < 30:
            motor_speed = 0
            state = STATE_STOPPED
            logger.info("Parking completed!")
            
    return motor_speed, steering_angle

# --- Main Control Loop ---
def main():
    global state, lap_count, start_time, direction
    
    logger.info("WRO 2025 Car Controller Initializing...")
    
    # Initialize camera
    cap = initialize_camera()
    if cap is None:
        logger.error("Failed to initialize camera. Exiting.")
        return
        
    # Calibrate IMU
    calibrate_imu()
    
    logger.info("Waiting for START signal from Nano...")
    start_time = time.time()
    
    try:
        while True:
            # Read sensor data from Nano
            nano_com.read_sensors()
            
            # Check for START signal
            if state == STATE_WAITING_FOR_START and ir_sensor_states[0] == 0 and ir_sensor_states[1] == 0:
                state = STATE_DRIVING_LAPS
                logger.info("START detected! Beginning lap driving.")
                # Acknowledge start
                nano_com.send_command("ACK", "START")
                
            # Get camera frame
            ret, frame = cap.read()
            if not ret:
                logger.warning("Failed to capture frame")
                time.sleep(0.1)
                continue
                
            # Process vision
            vision_error, sign_info, parking_info = detect_track_features(frame)
            
            # Update lap count
            update_lap_count()
            
            # Check if we've completed the required laps
            if lap_count >= REQUIRED_LAPS and state == STATE_DRIVING_LAPS:
                state = STATE_PARKING
                parking_sequence_step = 0
                logger.info("All laps completed. Beginning parking maneuver.")
                
            # Calculate control commands
            motor_speed, steering_angle = calculate_control_commands(
                vision_error, sign_info, parking_info)
                
            # Send commands to Nano
            nano_com.send_command("MOTOR", motor_speed)
            nano_com.send_command("SERVO", steering_angle)
            
            # Control loop rate
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
    finally:
        # Cleanup
        nano_com.send_command("MOTOR", 0)
        nano_com.send_command("SERVO", 90)
        if cap is not None:
            cap.release()
        nano_com.close()
        logger.info("Shutdown complete")

if __name__ == "__main__":
    main()