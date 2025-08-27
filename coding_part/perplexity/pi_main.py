#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import logging
from collections import deque
import cv2
import numpy as np

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

CAMERA_INDEX = 2
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_RATE = 15

STATE_WAITING_FOR_START = "WAITING_FOR_START"
STATE_DRIVING_LAPS = "DRIVING_LAPS"
STATE_PARKING = "PARKING"
STATE_STOPPED = "STOPPED"

REQUIRED_TURNS_PER_LAP = 4
REQUIRED_LAPS = 3
TURN_THRESHOLD_DEGREES = 70
TURN_RESET_THRESHOLD_DEGREES = 20
GYRO_INTEGRAL_WINDOW_SIZE = 5

PARKING_LOT_DETECTED_THRESHOLD = 0.8
PARALLEL_PARK_STEPS = 5

BASE_SPEED = 70
MAX_SPEED = 100
MIN_SPEED = 30
STEERING_P_GAIN = 0.8
STEERING_D_GAIN = 0.3

state = STATE_WAITING_FOR_START
lap_count = 0
turn_count = 0
last_turn_integral = 0.0
gyro_z_history = deque(maxlen=GYRO_INTEGRAL_WINDOW_SIZE)
parking_sequence_step = 0
start_time = None

ir_sensor_states = [1, 1, 1, 1]
ultrasonic_distances_mm = [9999, 9999]
imu_data_raw = [0, 0, 0, 0, 0, 0]

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    logger.info(f"Connected to Arduino Nano on {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    logger.error(f"Failed to connect to serial port {SERIAL_PORT}: {e}")
    exit(1)
except Exception as e:
    logger.error(f"Unexpected error during serial setup: {e}")
    exit(1)

cap = None
def initialize_camera():
    global cap
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            logger.error(f"Error: Could not open USB camera (index: {CAMERA_INDEX}). isOpened() returned False.")
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
        time.sleep(0.1)
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"Camera reports: Width: {actual_width}, Height: {actual_height}, FPS: {actual_fps}")
        logger.info(f"Camera initialized (Index: {CAMERA_INDEX}, Res: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {actual_fps} FPS)")
        return True
    except Exception as e:
        logger.error(f"Error initializing camera: {e}", exc_info=True)
        return False

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
        logger.debug(f"Sent to Nano: {command_str}")
    except (OSError, serial.SerialException) as e:
        logger.error(f"Serial write error: {e}")
        try:
            if ser and ser.is_open:
                ser.close()
        except:
            pass
        ser = None
    except Exception as e:
        logger.error(f"Unexpected error sending command to Nano: {e}")

def parse_sensor_data(data_str):
    global ir_sensor_states, ultrasonic_distances_mm, imu_data_raw
    try:
        parts = data_str.split(',')
        if len(parts) >= 13 and parts[0] == "SENSORS":
            try:
                ir_sensor_states[0] = int(parts[1])
                ir_sensor_states[1] = int(parts[2])
                ir_sensor_states[2] = int(parts[3])
                ir_sensor_states[3] = int(parts[4])
                ultrasonic_distances_mm[0] = int(parts[5])
                ultrasonic_distances_mm[1] = int(parts[6])
                imu_data_raw[0] = int(parts[7])
                imu_data_raw[1] = int(parts[8])
                imu_data_raw[2] = int(parts[9])
                imu_data_raw[3] = int(parts[10])
                imu_data_raw[4] = int(parts[11])
                imu_data_raw[5] = int(parts[12])
                logger.debug(f"Parsed Sensor Data: IR={ir_sensor_states}, US={ultrasonic_distances_mm}, IMU={imu_data_raw}")
            except (ValueError, IndexError) as e:
                logger.warning(f"Error parsing sensor data '{data_str}' - {e}")
        else:
            logger.warning(f"Received unexpected serial data format: {data_str}")
    except Exception as e:
        logger.error(f"Error in parse_sensor_data: {e}")

def process_camera_frame(frame):
    # This function performs lane and obstacle detection, returns error values for steering and info for signs/parking
    # For brevity, keep implementation ready to integrate or enhance as needed for your vision processing approach.
    error = 0.0
    sign_info = None
    parking_lot_info = None
    # Implement lane detection and sign detection logic here or keep your existing robust implementation
    
    # Placeholder example:
    # error = computed_steering_error_from_lane_detection(frame)
    # sign_info = detected_signs
    # parking_lot_info = detected_parking_lot_info
    return error, sign_info, parking_lot_info

def calculate_control_commands(error, sign_info, parking_lot_info):
    global state, lap_count, parking_sequence_step

    left_motor_speed = 0
    right_motor_speed = 0
    servo_angle = 90

    try:
        if state == STATE_DRIVING_LAPS:
            p_term = STEERING_P_GAIN * error
            steering_adjustment = p_term
            servo_angle = int(90 + steering_adjustment)
            servo_angle = np.clip(servo_angle, 45, 135)

            left_motor_speed = BASE_SPEED
            right_motor_speed = BASE_SPEED

            if servo_angle < 85:
                right_motor_speed = min(BASE_SPEED + 20, MAX_SPEED)
                left_motor_speed = max(BASE_SPEED - 20, MIN_SPEED)
            elif servo_angle > 95:
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
                right_motor_speed = -BASE_SPEED
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

        left_motor_speed = int(np.clip(left_motor_speed, -255, 255))
        right_motor_speed = int(np.clip(right_motor_speed, -255, 255))
    except Exception as e:
        logger.error(f"Error in calculate_control_commands: {e}")
        left_motor_speed = 0
        right_motor_speed = 0
        servo_angle = 90

    return left_motor_speed, right_motor_speed, servo_angle

def update_lap_count(gyro_z_rate):
    global turn_count, last_turn_integral, gyro_z_history, lap_count

    try:
        GYRO_SENSITIVITY_DPS_PER_LSB = 0.061
        gyro_z_dps = gyro_z_rate * GYRO_SENSITIVITY_DPS_PER_LSB

        gyro_z_history.append(gyro_z_dps)

        dt = 0.1
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
            try:
                if ser and ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').rstrip()
                    if line:
                        logger.debug(f"Received from Nano: {line}")
                        if line == "START":
                            if state == STATE_WAITING_FOR_START:
                                logger.info("START signal received from Nano!")
                                send_command_to_nano("ACK_START")
                                state = STATE_DRIVING_LAPS
                                logger.info("State changed to DRIVING_LAPS")
                                time.sleep(0.5)
                        elif line.startswith("SENSORS"):
                            parse_sensor_data(line)
                            update_lap_count(imu_data_raw[5])
            except (OSError, serial.SerialException) as e:
                logger.error(f"Serial read error: {e}")
                try:
                    if ser and ser.is_open:
                        ser.close()
                except:
                    pass
                ser = None
                time.sleep(2)
                try:
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                    logger.info(f"Re-connected to Arduino Nano on {SERIAL_PORT} at {BAUD_RATE} baud.")
                except Exception as reconnect_err:
                    logger.error(f"Failed to reconnect to serial port {SERIAL_PORT}: {reconnect_err}")
            except Exception as e:
                logger.error(f"Unexpected error in serial reading loop: {e}")

            if state == STATE_WAITING_FOR_START:
                time.sleep(0.1)

            elif state == STATE_DRIVING_LAPS:
                frame_capture_success = False
                frame = None
                try:
                    if cap is not None:
                        ret, frame = cap.read()
                        if ret:
                            frame_capture_success = True
                        else:
                            logger.warning("Failed to grab frame from camera (ret=False).")
                    else:
                        logger.error("Camera object (cap) is None.")
                except cv2.error as e:
                    logger.error(f"OpenCV error during frame capture: {e}")
                except Exception as e:
                    logger.error(f"Unexpected error during frame capture: {e}")

                if not frame_capture_success:
                    time.sleep(0.1)
                    continue

                try:
                    error, sign_info, parking_lot_info = process_camera_frame(frame)
                except Exception as e:
                    logger.error(f"Error processing camera frame: {e}")
                    error, sign_info, parking_lot_info = 0.0, None, None

                if lap_count >= REQUIRED_LAPS:
                    logger.info("Three laps completed. Initiating Parking sequence.")
                    state = STATE_PARKING
                    parking_sequence_step = 0
                    lap_count = 0

                try:
                    left_speed, right_speed, servo_angle = calculate_control_commands(error, sign_info, parking_lot_info)
                except Exception as e:
                    logger.error(f"Error calculating control commands: {e}")
                    left_speed, right_speed, servo_angle = 0, 0, 90

                send_command_to_nano(f"MOTOR,{left_speed},{right_speed}")
                send_command_to_nano(f"SERVO,{servo_angle}")

            elif state == STATE_PARKING:
                try:
                    dummy_error = 0
                    dummy_sign_info = None
                    dummy_parking_info = None
                    left_speed, right_speed, servo_angle = calculate_control_commands(dummy_error, dummy_sign_info, dummy_parking_info)
                    send_command_to_nano(f"MOTOR,{left_speed},{right_speed}")
                    send_command_to_nano(f"SERVO,{servo_angle}")
                    time.sleep(1)
                except Exception as e:
                    logger.error(f"Error in PARKING state: {e}")

            elif state == STATE_STOPPED:
                send_command_to_nano("MOTOR,0,0")
                send_command_to_nano("SERVO,90")
                logger.info("Vehicle stopped. Waiting...")
                time.sleep(1)

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as e:
        logger.error(f"An unexpected error occurred in main loop: {e}", exc_info=True)
    finally:
        logger.info("Cleaning up resources...")
        try:
            send_command_to_nano("MOTOR,0,0")
        except Exception as e:
            logger.error(f"Error sending final stop command: {e}")

        try:
            if cap and cap.isOpened():
                cap.release()
        except Exception as e:
            logger.error(f"Error releasing camera: {e}")

        try:
            cv2.destroyAllWindows()
        except Exception as e:
            logger.error(f"Error destroying CV windows: {e}")

        try:
            if ser and ser.is_open:
                ser.close()
        except Exception as e:
            logger.error(f"Error closing serial port: {e}")

        logger.info("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()
