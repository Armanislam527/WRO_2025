"""
Serial communication configuration matching Arduino Nano protocol
Ensures perfect synchronization with your existing Nano codebase
"""

import serial
from vehicle_config import NANO_PROTOCOL

class SerialConfig:
    """Serial communication parameters matching Nano setup"""
    PORT = "/dev/serial0"  # GPIO UART on Pi Zero
    BAUDRATE = 115200      # Must match Nano's SERIAL_BAUD_RATE
    TIMEOUT = 0.1          # Short timeout for responsive communication
    PARITY = serial.PARITY_NONE
    STOPBITS = serial.STOPBITS_ONE
    BYTESIZE = serial.EIGHTBITS
    
    # Heartbeat timing (must match Nano's WATCHDOG_TIMEOUT)
    HEARTBEAT_INTERVAL_MS = 400  # Slightly less than Nano's 500ms timeout
    SENSOR_REQUEST_INTERVAL_MS = 50  # Match Nano's 20Hz sending rate

class PacketConfig:
    """Packet structure matching Nano's serial_protocol.h"""
    
    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """Calculate XOR checksum matching Nano's implementation"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    @staticmethod
    def create_packet(command: int, data: bytes = b'') -> bytes:
        """Create packet matching Nano's protocol exactly"""
        if len(data) > NANO_PROTOCOL.PACKET_MAX_LENGTH:
            raise ValueError(f"Data too long: {len(data)} bytes")
        
        # Build packet: [START, CMD, LEN, DATA..., CHECKSUM]
        packet_data = bytes([command, len(data)]) + data
        checksum = PacketConfig.calculate_checksum(packet_data)
        
        return bytes([NANO_PROTOCOL.PACKET_START_BYTE]) + packet_data + bytes([checksum])

class EmergencyConfig:
    """Emergency and safety parameters"""
    MAX_MOTOR_ACCELERATION = 10  # Speed change per cycle for smooth control
    EMERGENCY_STOP_DELAY_MS = 100
    COMMUNICATION_TIMEOUT_MS = 1000
    
    # Safe defaults
    SAFE_SPEED = 0
    SAFE_STEERING_ANGLE = 90  # Center position

# Global configuration instances
SERIAL_CONF = SerialConfig()
PACKET_CONF = PacketConfig()
SAFETY_CONF = EmergencyConfig()