// sensor_data.h
// Defines the structure for sensor data received from the Nano

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <cstdint> // For fixed-width integer types

// This struct should exactly match the CompactSensorData struct on the Nano
// for easy casting/interpretation of the binary data packet.
struct __attribute__((packed)) SensorData
{
    uint16_t frontDistance; // 0-65535 cm
    uint16_t rightDistance; // 0-65535 cm
    uint16_t backDistance;  // 0-65535 cm
    uint16_t leftDistance;  // 0-65535 cm
    int16_t accelX;         // Scaled IMU data (e.g., milli-g)
    int16_t accelY;         // Scaled IMU data (e.g., milli-g)
    int16_t accelZ;         // Scaled IMU data (e.g., milli-g)
    int16_t gyroX;          // Scaled IMU data (e.g., milli-deg/s)
    int16_t gyroY;          // Scaled IMU data (e.g., milli-deg/s)
    int16_t gyroZ;          // Scaled IMU data (e.g., milli-deg/s)
    // Total: 20 bytes
};

#endif // SENSOR_DATA_H