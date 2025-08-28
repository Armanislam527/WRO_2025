#!/usr/bin/env python3
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def main():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            write_timeout=1
        )
        
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        time.sleep(2)  # Allow Arduino to reset
        
        ser.flushInput()
        ser.flushOutput()
        
        while True:
            # Send PING command
            ser.write(b"PING\n")
            print("Sent: PING")
            
            # Read response
            response = ser.readline().decode('utf-8').strip()
            if response:
                print(f"Received: {response}")
            
            time.sleep(2)  # Wait 2 seconds between pings
            
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
