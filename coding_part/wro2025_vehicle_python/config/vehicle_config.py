# config/vehicle_config.py

"""Configuration file for the WRO 2025 Vehicle."""

import os

# --- Serial Communication ---
# SERIAL_PORT = os.environ.get("WRO_SERIAL_PORT", "/dev/serial0") # Default, can be overridden by environment variable
SERIAL_PORT = "/dev/ttyUSB0" # For testing with USB-Serial adapter, change as needed
SERIAL_BAUDRATE = 115200

# --- Camera ---
# Default camera settings (can be adjusted)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_DEVICE = "/base/soc/i2c0mux/i2c@1/imx219@10" # Default for Pi Camera Module 2, adjust if needed for OV5647 or libcamera

# --- Sensor Data Packet (Matching C++ CompactSensorData) ---
# Size of the CompactSensorData struct from C++
COMPACT_SENSOR_DATA_SIZE = 18 # Update if the C++ struct changes

# --- Mission States (Matching C++ enums) ---
class ChallengeType:
    UNKNOWN = 0
    OPEN = 1
    OBSTACLE = 2

class MissionPhase:
    PRE_START = 0
    WAITING_FOR_GO = 1
    DRIVING_LAPS = 2
    STOPPING_AT_START = 3 # Used for Open Challenge after 3 laps
    FINDING_PARKING = 4   # Used for Obstacle Challenge after 3 laps
    PARKING = 5
    EMERGENCY_STOPPED = 6
    COMPLETED = 7

# --- Navigation Commands (Matching C++ NavigationCommand::Action) ---
class NavigationAction:
    GO_STRAIGHT = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    STOP_IN_START = 3 # Stop in the starting section (after 3 laps in Open)
    EXECUTE_PARKING = 4 # Initiate the parking sequence (after 3 laps in Obstacle)
    EMERGENCY_STOP = 5
    # Add more actions if defined in C++

# --- Colors (Based on WRO 2025 Rules) ---
# These are base values. Calibration will adjust actual HSV ranges used.
COLORS = {
    "TRACK_WHITE": {
        "name": "Track Mat (White)",
        "rgb": (255, 255, 255), # Approximate
        "description": "Primary background color of the track."
    },
    "WALL_BLACK": {
        "name": "Interior Walls (Black)",
        "rgb": (0, 0, 0), # Approximate
        "description": "Color of the track walls for edge detection."
    },
    "SIGN_RED": {
        "name": "Red Traffic Sign",
        "rgb": (238, 39, 55), # As per rules
        "description": "Indicates turn/move right in Obstacle Challenge."
    },
    "SIGN_GREEN": {
        "name": "Green Traffic Sign",
        # Assuming a correct green value (rule snippet had red value)
        "rgb": (68, 214, 44), # Common WRO green, verify from full PDF
        "description": "Indicates turn/move left in Obstacle Challenge."
    },
    "PARKING_MAGENTA": {
        "name": "Parking Lot Boundaries (Magenta)",
        "rgb": (255, 0, 255), # As per rules
        "description": "Physical boundaries of the parking space."
    }
    # Add other specified colors if needed for detection (e.g., seat lines)
}

# --- Logging ---
# Basic logging configuration can be set up later
LOG_LEVEL = os.environ.get("WRO_LOG_LEVEL", "DEBUG") # Can be DEBUG, INFO, WARNING, ERROR

# --- Parking ---
# Placeholder for parking parameters, likely determined dynamically
PARKING_LOT_WIDTH_CM = 20.0 # As per rules
# Length is 1.5 * vehicle length - needs measurement
