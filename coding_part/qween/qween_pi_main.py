#!/usr/bin/env python3
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

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 500000
CAMERA_INDEX = 2
FRAME_WIDTH = 160
FRAME_HEIGHT = 120
FRAME_RATE = 15

REQUIRED_LAPS = 3
REQUIRED_TURNS_PER_LAP = 4
TURN_MIN_ANGLE = 75
TURN_RESET_ANGLE = 20
GYRO_INTEGRAL_WINDOW_SIZE = 5

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

BASE_SPEED = 65
MAX_SPEED = 90
MIN_SPEED = 25
STEERING_P_GAIN = 0.7
STEERING_D_GAIN = 0.2
LANE_KEEP_OFFSET_MM = 50

STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_OBEYING_SIGN = "OBEYING_SIGN"
STATE_PARKING_SEARCH = "PARKING_SEARCH"
STATE_PARKING_MANEUVER = "PARKING_MANEUVER"
STATE_STOPPED = "STOPPED"

state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE)
parking_sequence_step = 0
start_time = None
run_acknowledged = False
start_signal_received_flag = False

ir_sensor_states = [1, 1, 1, 1]
ultrasonic_distances_mm = [9999, 9999]
imu_data_raw = [0, 0, 0, 0, 0, 0]

serial_lock = threading.Lock()
ser = None
stop_event = threading.Event()

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', handlers=[logging.FileHandler("wro_car.log"), logging.StreamHandler()])
logger = logging.getLogger(__name__)

def initialize_serial():
    global ser
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.01, write_timeout=0.1)
        logger.info(f"Connected to Arduino Nano on {SERIAL_PORT} at {BAUD_RATE} baud.")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to serial port {SERIAL_PORT}: {e}")
        return False

def send_command_to_nano(command_char, value=None):
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
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw, start_signal_received_flag
    buffer = bytearray()
    while not stop_event.is_set():
        if ser and ser.is_open and ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
            while len(buffer) >= 21:
                if buffer[0] == 0xAA:
                    if len(buffer) >= 21 and buffer[20] == 0x55:
                        packet_data = buffer[1:20]
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
                            with serial_lock:
                                ir_sensor_states[0] = (ir_states_byte >> 0) & 1
                                ir_sensor_states[1] = (ir_states_byte >> 1) & 1
                                ir_sensor_states[2] = (ir_states_byte >> 2) & 1
                                ir_sensor_states[3] = (ir_states_byte >> 3) & 1
                                if not start_signal_received_flag and (ir_states_byte >> 7) & 1:
                                    logger.info("START flag detected in sensor packet!")
                                    start_signal_received_flag = True
                                ultrasonic_distances_mm[0] = us_front
                                ultrasonic_distances_mm[1] = us_rear
                                imu_data_raw = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
                        except struct.error as e:
                            logger.error(f"Error unpacking sensor data: {e}")
                        del buffer[:21]
                    else:
                        break
                else:
                    buffer.pop(0)
        else:
            time.sleep(0.001)

def initialize_camera():
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            logger.error("Could not open camera")
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        time.sleep(0.5)
        logger.info(f"Camera initialized: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {FRAME_RATE}fps")
        return cap
    except Exception as e:
        logger.error(f"Camera initialization failed: {e}")
        return None

def update_lap_count(gyro_z_rate):
    global turn_count, last_turn_integral, gyro_z_history, lap_count
    GYRO_SENSITIVITY_DPS_PER_LSB = 131.0
    gyro_z_dps = gyro_z_rate / GYRO_SENSITIVITY_DPS_PER_LSB
    gyro_z_history.append(gyro_z_dps)
    dt = 0.05
    current_integral = sum(gyro_z_history) * dt / len(gyro_z_history) if len(gyro_z_history) > 0 else 0
    turn_angle = current_integral - last_turn_integral
    last_turn_integral = current_integral
    logger.debug(f"Gyro Z Raw: {gyro_z_rate}, DPS: {gyro_z_dps:.2f}, Turned: {turn_angle:.2f}deg")
    if abs(turn_angle) > TURN_MIN_ANGLE and turn_count < REQUIRED_TURNS_PER_LAP:
        turn_count += 1
        logger.info(f"*** TURN DETECTED! Count: {turn_count}/{REQUIRED_TURNS_PER_LAP} ***")
    if abs(turn_angle) < TURN_RESET_ANGLE and turn_count > 0:
        pass
    if turn_count >= REQUIRED_TURNS_PER_LAP:
        lap_count += 1
        turn_count = 0
        last_turn_integral = 0.0
        gyro_z_history.clear()
        logger.info(f"*** LAP {lap_count}/{REQUIRED_LAPS} COMPLETED! ***")

def detect_track_features(frame):
    if frame is None:
        return 0.0, None, None
    try:
        h, w = frame.shape[:2]
        roi_h_start = int(h * 0.6)
        roi = frame[roi_h_start:h, 0:w]
        roi_h, roi_w = roi.shape[:2]
        center_x = roi_w // 2
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv_roi, LOWER_WHITE, UPPER_WHITE)
        black_mask = cv2.inRange(hsv_roi, LOWER_BLACK, UPPER_BLACK)
        scan_line_y = roi_h - 10
        left_edge_x = None
        right_edge_x = None
        for x in range(5, center_x):
            if white_mask[scan_line_y, x] > 0 and black_mask[scan_line_y, x-5:x].any():
                left_edge_x = x
                break
        for x in range(roi_w - 5, center_x, -1):
            if white_mask[scan_line_y, x] > 0 and black_mask[scan_line_y, x+1:x+6].any():
                right_edge_x = x
                break
        error = 0.0
        if left_edge_x is not None and right_edge_x is not None:
            lane_center_x = (left_edge_x + right_edge_x) / 2.0
            error = lane_center_x - center_x
        elif left_edge_x is not None:
            error = 30.0
        elif right_edge_x is not None:
            error = -30.0
        red_mask1 = cv2.inRange(hsv_roi, LOWER_RED1, UPPER_RED1)
        red_mask2 = cv2.inRange(hsv_roi, LOWER_RED2, UPPER_RED2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv_roi, LOWER_GREEN, UPPER_GREEN)
        sign_info = None
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in red_contours:
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
                         sign_info = {'type': 'red', 'position': position, 'cx': cx, 'cy': cy, 'area': area}
                         break
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
        magenta_mask = cv2.inRange(hsv_roi, LOWER_MAGENTA, UPPER_MAGENTA)
        parking_info = None
        magenta_contours, _ = cv2.findContours(magenta_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parking_bars = []
        for cnt in magenta_contours:
            area = cv2.contourArea(cnt)
            if area > 80:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                aspect_ratio = float(w_rect) / h_rect if h_rect != 0 else 0
                if w_rect > h_rect and aspect_ratio > 2.0:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        parking_bars.append({'cx': cx, 'cy': cy, 'x': x, 'y': y, 'w': w_rect, 'h': h_rect})
        if len(parking_bars) >= 2:
            parking_bars.sort(key=lambda b: b['cx'])
            left_bar = parking_bars[0]
            right_bar = parking_bars[-1]
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

def calculate_control_commands(lane_error, sign_info, parking_info):
    global state, parking_sequence_step
    motor_speed = 0
    steering_angle = 90
    with serial_lock:
        local_ir_states = ir_sensor_states[:]
        local_us_distances = ultrasonic_distances_mm[:]
        local_imu_data = imu_data_raw[:]
    if state == STATE_WAITING_FOR_START:
        motor_speed = 0
        steering_angle = 90
    elif state == STATE_DRIVING_LAPS:
        p_term = STEERING_P_GAIN * lane_error
        d_term = 0
        steering_adjustment = p_term + d_term
        steering_angle = 90 + steering_adjustment
        steering_angle = np.clip(steering_angle, 45, 135)
        motor_speed = BASE_SPEED
        steering_deviation = abs(steering_angle - 90)
        if steering_deviation > 25:
            motor_speed = max(MIN_SPEED, BASE_SPEED - steering_deviation * 0.4)
        if sign_info:
            state = STATE_OBEYING_SIGN
            logger.info(f"Detected {sign_info['type']} sign on {sign_info['position']} side. Obeying...")
    elif state == STATE_OBEYING_SIGN:
        if sign_info and sign_info['type'] == 'red':
            target_cx = sign_info['cx'] + 20
            error_from_sign = target_cx - (FRAME_WIDTH // 2)
            steering_angle = 90 + (error_from_sign * 0.5)
            motor_speed = BASE_SPEED * 0.7
        elif sign_info and sign_info['type'] == 'green':
            target_cx = sign_info['cx'] - 20
            error_from_sign = target_cx - (FRAME_WIDTH // 2)
            steering_angle = 90 + (error_from_sign * 0.5)
            motor_speed = BASE_SPEED * 0.7
        else:
            state = STATE_DRIVING_LAPS
            logger.info("Finished obeying sign. Resuming normal driving.")
        steering_angle = np.clip(steering_angle, 45, 135)
    elif state == STATE_PARKING_SEARCH:
        motor_speed = BASE_SPEED * 0.5
        steering_angle = 110
        if parking_info and parking_info['detected']:
            logger.info("Parking lot detected. Initiating parking maneuver.")
            state = STATE_PARKING_MANEUVER
            parking_sequence_step = 0
    elif state == STATE_PARKING_MANEUVER:
        if parking_sequence_step == 0:
            if parking_info:
                align_error = parking_info['center_x'] - (FRAME_WIDTH / 2)
                steering_angle = 90 + (align_error * 0.3)
                motor_speed = BASE_SPEED * 0.4
                if abs(align_error) < 15 and local_us_distances[0] < 300:
                    parking_sequence_step = 1
                    logger.debug("Step 0 -> 1: Aligned and close. Moving to pass space.")
            else:
                steering_angle = 90
                motor_speed = BASE_SPEED * 0.3
        elif parking_sequence_step == 1:
            motor_speed = BASE_SPEED * 0.4
            steering_angle = 90
            parking_sequence_step = 2
            logger.debug("Step 1 -> 2: Passed space. Starting reverse.")
        elif parking_sequence_step == 2:
            motor_speed = -BASE_SPEED * 0.4
            if parking_info and parking_info.get('position') == 'right':
                steering_angle = 135
            else:
                steering_angle = 45
            if local_us_distances[0] < 200 and local_us_distances[1] < 200:
                parking_sequence_step = 3
                logger.debug("Step 2 -> 3: In space. Straightening.")
        elif parking_sequence_step == 3:
            motor_speed = -BASE_SPEED * 0.3
            steering_angle = 90
            us_diff = abs(local_us_distances[0] - local_us_distances[1])
            if us_diff < 50:
                parking_sequence_step = 4
                logger.debug("Step 3 -> 4: Straightened. Moving forward to center.")
        elif parking_sequence_step == 4:
            motor_speed = BASE_SPEED * 0.3
            steering_angle = 90
            us_diff = abs(local_us_distances[0] - local_us_distances[1])
            if us_diff < 30:
                motor_speed = 0
                steering_angle = 90
                state = STATE_STOPPED
                logger.info("*** PARKING COMPLETED SUCCESSFULLY! ***")
        motor_speed = np.clip(motor_speed, -MAX_SPEED, MAX_SPEED)
        steering_angle = np.clip(steering_angle, 45, 135)
    elif state == STATE_STOPPED:
        motor_speed = 0
        steering_angle = 90
    motor_speed = int(np.clip(motor_speed, -255, 255))
    steering_angle = int(np.clip(steering_angle, 0, 180))
    return motor_speed, steering_angle

def signal_handler(sig, frame):
    logger.info("Interrupt received, shutting down...")
    stop_event.set()
    if ser and ser.is_open:
        try:
            send_command_to_nano('M', 0)
            send_command_to_nano('S', 90)
            time.sleep(0.1)
        except:
            pass
        ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    global state, lap_count, start_time, run_acknowledged, start_signal_received_flag
    logger.info("Initializing WRO 2025 Car Controller (Pi Side)...")
    if not initialize_serial():
        logger.error("Exiting due to serial initialization failure.")
        return
    serial_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    serial_thread.start()
    logger.info("Serial reader thread started.")
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
            if state == STATE_WAITING_FOR_START and start_signal_received_flag:
                logger.info("*** START signal received from Nano (via sensor packet)! ***")
                if not run_acknowledged:
                    send_command_to_nano('A')
                    run_acknowledged = True
                state = STATE_DRIVING_LAPS
                logger.info("State changed to DRIVING_LAPS")
                time.sleep(0.5)
                start_signal_received_flag = False
            with serial_lock:
                current_gyro_z = imu_data_raw[5]
            if state in [STATE_DRIVING_LAPS, STATE_OBEYING_SIGN]:
                update_lap_count(current_gyro_z)
            ret, frame = cap.read()
            if not ret:
                logger.warning("Failed to grab frame from camera.")
                time.sleep(0.05)
                continue
            lane_error, sign_info, parking_info = detect_track_features(frame)
            if state == STATE_DRIVING_LAPS and lap_count >= REQUIRED_LAPS:
                 logger.info("*** THREE LAPS COMPLETED! Initiating Parking Sequence. ***")
                 state = STATE_PARKING_SEARCH
                 parking_sequence_step = 0
            motor_speed, servo_angle = calculate_control_commands(lane_error, sign_info, parking_info)
            send_command_to_nano('M', motor_speed)
            send_command_to_nano('S', servo_angle)
            time.sleep(0.05)
    except Exception as e:
        logger.error(f"An unexpected error occurred in main loop: {e}", exc_info=True)
    finally:
        logger.info("Cleaning up resources...")
        stop_event.set()
        serial_thread.join(timeout=1.0)
        if ser and ser.is_open:
            send_command_to_nano('M', 0)
            send_command_to_nano('S', 90)
            time.sleep(0.1)
            ser.close()
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        logger.info("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()