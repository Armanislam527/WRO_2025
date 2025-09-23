# sensors/sensor_data.py

"""Defines data structures for sensor information, matching the C++ structures."""

import struct
from dataclasses import dataclass
from typing import Tuple

# --- Sensor Data Structures (High-level, matching C++ SensorData) ---

@dataclass
class USSensorData:
    """Represents data from Ultrasonic Sensors."""
    front_distance: float = 0.0  # Distance in cm (float for precision)
    right_distance: float = 0.0
    back_distance: float = 0.0
    left_distance: float = 0.0

@dataclass
class IMUData:
    """Represents data from the IMU (MPU6050)."""
    # Using floats as they are typically decimal values
    accel_x: float = 0.0  # Acceleration in g
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0   # Angular velocity in degrees/s
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    temperature: float = 0.0 # Temperature in Celsius (if sent)

# --- Main Sensor Data Class (High-level) ---
@dataclass
class SensorData:
    """Main container for all sensor data received from the Nano."""
    us_data: USSensorData = None
    imu_data: IMUData = None

    def __post_init__(self):
        # Ensure nested objects are initialized if not provided
        if self.us_data is None:
            self.us_data = USSensorData()
        if self.imu_data is None:
            self.imu_data = IMUData()

    def __str__(self):
        return f"SensorData(US: {self.us_data}, IMU: {self.imu_data})"


# --- Compact Sensor Data (Matching C++ struct for Serial Protocol) ---
# C++ struct definition from logtext.txt:
# struct __attribute__((packed)) CompactSensorData{
#     uint16_t frontDistance; // 0-65535 cm - 2 bytes
#     uint16_t rightDistance; // 0-65535 cm - 2 bytes
#     uint16_t backDistance;  // 0-65535 cm - 2 bytes
#     uint16_t leftDistance;  // 0-65535 cm - 2 bytes
#     int16_t accelX;         // Scaled IMU data (e.g., milli-g) - 2 bytes
#     int16_t accelY;         // Scaled IMU data (e.g., milli-g) - 2 bytes
#     int16_t accelZ;         // Scaled IMU data (e.g., milli-g) - 2 bytes
#     int16_t gyroX;          // Scaled IMU data (e.g., milli-deg/s) - 2 bytes
#     int16_t gyroY;          // Scaled IMU data (e.g., milli-deg/s) - 2 bytes
#     int16_t gyroZ;          // Scaled IMU data (e.g., milli-deg/s) - 2 bytes
#     uint8_t checksum;       // 1 byte
# }; // Total size = 21 bytes

COMPACT_SENSOR_DATA_FORMAT = '<HHHHhhhhhhB' # Little-endian: 4*uint16 + 6*int16 + 1*uint8 = 21 bytes
COMPACT_SENSOR_DATA_SIZE = struct.calcsize(COMPACT_SENSOR_DATA_FORMAT) # Should be 21

def calculate_checksum(data_bytes: bytes) -> int:
    """Calculate the checksum (XOR of all bytes)."""
    checksum = 0
    for byte_val in data_bytes:
        checksum ^= byte_val
    return checksum

def pack_compact_sensor_data(data: SensorData) -> bytes:
    """
    Pack SensorData into the compact binary format for serial transmission.
    Matches the C++ CompactSensorData struct.
    Note: Scaling factors (e.g., milli-g, milli-deg/s) are assumed.
    """
    try:
        # Scale values to int16_t range
        # Distances are uint16, so direct conversion to cm (assuming input is in cm)
        # IMU values are scaled. Common scales: 1000 for milli-units.
        scaled_front = int(data.us_data.front_distance) # uint16, cm
        scaled_right = int(data.us_data.right_distance)
        scaled_back = int(data.us_data.back_distance)
        scaled_left = int(data.us_data.left_distance)
        
        scaled_accel_x = int(data.imu_data.accel_x * 1000) # int16, milli-g
        scaled_accel_y = int(data.imu_data.accel_y * 1000)
        scaled_accel_z = int(data.imu_data.accel_z * 1000)
        scaled_gyro_x = int(data.imu_data.gyro_x * 1000)  # int16, milli-deg/s
        scaled_gyro_y = int(data.imu_data.gyro_y * 1000)
        scaled_gyro_z = int(data.imu_data.gyro_z * 1000)
        
        # Clamp values to int16 range to prevent struct.pack errors
        def clamp_int16(val):
            return max(-32768, min(32767, val))
            
        scaled_accel_x = clamp_int16(scaled_accel_x)
        scaled_accel_y = clamp_int16(scaled_accel_y)
        scaled_accel_z = clamp_int16(scaled_accel_z)
        scaled_gyro_x = clamp_int16(scaled_gyro_x)
        scaled_gyro_y = clamp_int16(scaled_gyro_y)
        scaled_gyro_z = clamp_int16(scaled_gyro_z)
        
        # Clamp distances to uint16 range
        def clamp_uint16(val):
            return max(0, min(65535, val))
        scaled_front = clamp_uint16(scaled_front)
        scaled_right = clamp_uint16(scaled_right)
        scaled_back = clamp_uint16(scaled_back)
        scaled_left = clamp_uint16(scaled_left)

        # Pack data without checksum first
        packed_data_without_checksum = struct.pack(
            COMPACT_SENSOR_DATA_FORMAT[:-1], # Exclude the 'B' for checksum
            scaled_front,
            scaled_right,
            scaled_back,
            scaled_left,
            scaled_accel_x,
            scaled_accel_y,
            scaled_accel_z,
            scaled_gyro_x,
            scaled_gyro_y,
            scaled_gyro_z
        )
        
        # Calculate checksum on the packed data (excluding the checksum byte place)
        checksum = calculate_checksum(packed_data_without_checksum)
        
        # Append checksum
        final_packed_data = packed_data_without_checksum + bytes([checksum])
        return final_packed_data
        
    except (struct.error, ValueError, OverflowError) as e:
        print(f"Error packing sensor data: {e}")
        # Return empty bytes or a specific error indicator?
        return b''

def unpack_compact_sensor_data(data_bytes: bytes) -> Tuple[SensorData, bool]:
    """
    Unpack binary data received via serial into SensorData.
    Returns (SensorData object, is_valid_checksum).
    """
    if len(data_bytes) != COMPACT_SENSOR_DATA_SIZE:
        print(f"Incorrect data size for unpacking CompactSensorData. Expected {COMPACT_SENSOR_DATA_SIZE}, got {len(data_bytes)}")
        return SensorData(), False

    try:
        # Unpack all data including the checksum byte
        unpacked = struct.unpack(COMPACT_SENSOR_DATA_FORMAT, data_bytes)
        
        # Extract values
        front_dist, right_dist, back_dist, left_dist = unpacked[0:4]
        accel_x_raw, accel_y_raw, accel_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw = unpacked[4:10]
        received_checksum = unpacked[10]

        # Calculate checksum on the received data excluding the checksum byte
        calculated_checksum = calculate_checksum(data_bytes[:-1])

        is_valid = (received_checksum == calculated_checksum)

        # Convert back to appropriate units (reverse scaling)
        sensor_data = SensorData(
            us_data=USSensorData(
                front_distance=float(front_dist), # uint16 cm
                right_distance=float(right_dist),
                back_distance=float(back_dist),
                left_distance=float(left_dist)
            ),
            imu_data=IMUData(
                 accel_x=accel_x_raw / 1000.0, # milli-g back to g
                 accel_y=accel_y_raw / 1000.0,
                 accel_z=accel_z_raw / 1000.0,
                 gyro_x=gyro_x_raw / 1000.0,   # milli-deg/s back to deg/s
                 gyro_y=gyro_y_raw / 1000.0,
                 gyro_z=gyro_z_raw / 1000.0
                 # temperature not included in compact data
            )
        )
        return sensor_data, is_valid

    except struct.error as e:
        print(f"Error unpacking CompactSensorData: {e}")
        return SensorData(), False


# Example usage (if run as script)
if __name__ == "__main__":
    # Create sample data
    sample_data = SensorData(
        us_data=USSensorData(front_distance=15.5, right_distance=20.0, back_distance=10.2, left_distance=25.8),
        imu_data=IMUData(accel_x=0.1, accel_y=-0.05, accel_z=9.81, gyro_x=0.1, gyro_y=-0.2, gyro_z=15.3)
    )
    print("Original Data:", sample_data)

    # Pack it
    packed = pack_compact_sensor_data(sample_data)
    print(f"Packed Data ({len(packed)} bytes):", packed.hex())
    print("Expected Size:", COMPACT_SENSOR_DATA_SIZE)

    # Unpack it
    if len(packed) == COMPACT_SENSOR_DATA_SIZE:
        unpacked_data, valid_checksum = unpack_compact_sensor_data(packed)
        print("Unpacked Data:", unpacked_data)
        print("Checksum Valid:", valid_checksum)
    else:
        print("Packing failed, size mismatch.")
