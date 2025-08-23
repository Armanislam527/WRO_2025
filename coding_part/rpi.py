#!/usr/bin/env python3
"""
pi_driver_stub.py
Simple Pi Zero 2W vision -> serial controller.
Usage:
  python3 pi_driver_stub.py --serial /dev/ttyACM0 --camera 0
If you use GPIO UART instead, set --serial /dev/serial0 and ensure
you disabled the serial console in raspi-config and used the TTL level-shifter.
"""
import cv2
import numpy as np
import serial
import time
import argparse
import threading

parser = argparse.ArgumentParser()
parser.add_argument("--serial", default="/dev/ttyACM0",
                    help="Serial device to Arduino (e.g. /dev/ttyACM0 or /dev/serial0)")
parser.add_argument("--baud", type=int, default=115200)
parser.add_argument("--camera", default=0,
                    help="cv2 camera index or GStreamer string")
args = parser.parse_args()

SERIAL = args.serial
BAUD = args.baud
CAM_IDX = int(args.camera)

# open serial
ser = None
try:
    ser = serial.Serial(SERIAL, BAUD, timeout=0.1)
    print("Opened serial", SERIAL)
except Exception as e:
    print("Failed to open serial", SERIAL, e)
    print("You can still run vision-only. Exiting.")
    raise SystemExit(1)

# serial reader thread to print telemetry


def serial_reader():
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print("[ARU]", line)
        except Exception as e:
            print("Serial read err", e)
            time.sleep(0.1)


t = threading.Thread(target=serial_reader, daemon=True)
t.start()


def send_cmd(cmd):
    try:
        ser.write((cmd.strip() + "\n").encode('utf-8'))
    except Exception as e:
        print("Serial write", e)


# open camera
cap = cv2.VideoCapture(CAM_IDX)
if not cap.isOpened():
    print("Failed to open camera index:", CAM_IDX)
    raise SystemExit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

last_sent = 0
SEND_INTERVAL = 0.15  # seconds between control commands

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # RED detection (two ranges)
        lower_r1 = np.array([0, 120, 70])
        upper_r1 = np.array([10, 255, 255])
        lower_r2 = np.array([170, 120, 70])
        upper_r2 = np.array([180, 255, 255])
        mask_r = cv2.inRange(hsv, lower_r1, upper_r1)
        mask_r2 = cv2.inRange(hsv, lower_r2, upper_r2)
        mask_red = cv2.bitwise_or(mask_r, mask_r2)

        # GREEN detection
        lower_g = np.array([36, 50, 70])
        upper_g = np.array([89, 255, 255])
        mask_green = cv2.inRange(hsv, lower_g, upper_g)

        # Find largest red/green contour (if any)
        sign = None
        cnts_r, _ = cv2.findContours(
            mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts_r:
            c = max(cnts_r, key=cv2.contourArea)
            if cv2.contourArea(c) > 800:
                sign = 'R'
        cnts_g, _ = cv2.findContours(
            mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts_g:
            c = max(cnts_g, key=cv2.contourArea)
            if cv2.contourArea(c) > 800:
                sign = 'G'

        # Simple lane centre heuristic: invert Canny edges on lower half and compute centroid
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        inv = cv2.bitwise_not(edges)
        roi = inv[120:240, :]  # bottom half
        m = cv2.moments(roi)
        img_cx = 160
        if m['m00'] > 1000:
            cx = int(m['m10'] / m['m00'])
            centroid_x = cx
            img_cx = centroid_x
        # error relative to image center
        error = img_cx - 160

        # steering proportional controller
        Kp = -0.14  # tune this (sign chosen so positive error steers right)
        angle = int(90 + Kp * error)
        angle = max(40, min(140, angle))

        # override steering when sign seen (simple policy)
        if sign == 'R':
            angle = 120  # steer to right (tune)
        elif sign == 'G':
            angle = 60   # steer to left (tune)

        # speed based on steering demand
        base_speed = 180
        speed = int(max(60, base_speed - min(120, abs(error) * 0.6)))
        if abs(error) > 80:  # sharp turn -> reduce more
            speed = max(60, speed - 40)

        # send at controlled rate
        now = time.time()
        if now - last_sent > SEND_INTERVAL:
            send_cmd(f"STR {angle}")
            send_cmd(f"DRV {speed} F")
            last_sent = now

        # quick visual debug (optional)
        cv2.circle(frame, (160, 60), 4, (255, 0, 0), -1)
        cv2.line(frame, (160, 0), (160, 240), (100, 100, 100), 1)
        cv2.putText(frame, f"ANG {angle}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"SPD {speed}", (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if sign:
            cv2.putText(frame, f"SIGN {sign}", (8, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # show one small window locally (remove for headless Pi)
        cv2.imshow("driver", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting, sending STOP")
    send_cmd("STOP")
finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
