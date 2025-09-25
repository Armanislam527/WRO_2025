"""
Vehicle configuration aligned with WRO 2025 rules
Ensures compliance with competition requirements
Optimized for Pi Zero 2W processing capabilities
"""

import os
from dataclasses import dataclass
from typing import Tuple

@dataclass
class CompetitionRules:
    """WRO 2025 Future Engineers competition parameters"""
    MAX_VEHICLE_DIMENSIONS: Tuple[int, int, int] = (300, 200, 300)  # mm (length, width, height)
    MAX_WEIGHT_KG: float = 1.5
    TRACK_WIDTH_MM: Tuple[int, int] = (600, 1000)  # Narrow and wide track variations
    PARKING_LOT_WIDTH_MM: int = 200
    TRAFFIC_SIGN_HEIGHT_MM: int = 100
    
    # Scoring parameters (for decision making)
    POINTS_FULL_PARKING: int = 15
    POINTS_PARTIAL_PARKING: int = 7
    PENALTY_SIGN_MOVED: int = -2

@dataclass
class VehiclePhysicalParams:
    """Our specific vehicle physical characteristics"""
    WHEELBASE_MM: float = 150.0  # Distance between front and rear axles
    TRACK_WIDTH_MM: float = 120.0  # Distance between left and right wheels
    WHEEL_DIAMETER_MM: float = 60.0
    MAX_STEERING_ANGLE: float = 30.0  # degrees
    MAX_SPEED_MMS: float = 500.0  # mm per second
    
    # Camera mounting parameters (critical for vision calculations)
    CAMERA_HEIGHT_MM: float = 100.0  # Height from ground
    CAMERA_PITCH_ANGLE: float = -15.0  # degrees (negative = pointing slightly down)

@dataclass  
class PerformanceConstraints:
    """Pi Zero 2W performance optimization parameters"""
    MAX_FPS: int = 15  # Reduced for Pi Zero 2W
    TARGET_PROCESSING_TIME_MS: int = 66  # ~15 FPS
    SERIAL_UPDATE_RATE_HZ: int = 20  # Match Nano's expected rate
    VISION_SKIP_FRAMES: int = 1  # Process every 2nd frame to reduce load
    
    # Memory limits for Pi Zero 2W
    MAX_FRAME_BUFFERS: int = 2
    RESIZE_WIDTH: int = 320  # Process at lower resolution
    RESIZE_HEIGHT: int = 240

class NanoProtocolConfig:
    """Exact protocol matching with Arduino Nano code"""
    # Protocol constants (must match nano_code.txt)
    PACKET_START_BYTE = 0xAA
    PACKET_MAX_LENGTH = 32
    
    # Command IDs (from serial_protocol.h)
    CMD_SET_MOTOR_SPEED = 0x01
    CMD_SET_SERVO_ANGLE = 0x02
    CMD_EMERGENCY_STOP = 0x03
    CMD_HEARTBEAT = 0x04
    CMD_REQUEST_SENSOR_DATA = 0x05
    CMD_PI_GO_SIGNAL = 0x06
    CMD_SENSOR_DATA = 0x10
    CMD_ALL_SENSOR_DATA_COMPACT = 0x13
    
    # Motor mapping (must match nano's mapping)
    @staticmethod
    def map_speed_to_nano(speed_percent: float) -> int:
        """Convert -100% to +100% to Nano's expected byte value (-128 to 127)"""
        speed_byte = int((speed_percent / 100.0) * 127)
        return max(-128, min(127, speed_byte))
    
    @staticmethod
    def map_servo_angle(angle_degrees: float) -> int:
        """Convert steering angle to servo pulse width (0-180)"""
        return max(0, min(180, int(angle_degrees)))

# Global instances for easy access
RULES = CompetitionRules()
VEHICLE = VehiclePhysicalParams()
PERF = PerformanceConstraints()
NANO_PROTOCOL = NanoProtocolConfig()