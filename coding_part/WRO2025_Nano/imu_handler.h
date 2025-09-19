// imu_handler.h
// Handles communication with the MPU6050 IMU sensor

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>

// Structure to hold IMU data
struct IMUData
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temperature;
    // For more advanced implementations, you might add:
    // float yaw, pitch, roll; // If using DMP or sensor fusion
};

class IMUHandler
{
public:
    IMUHandler();
    bool initialize();
    bool readData(IMUData &data);
    void calibrate();

private:
    bool _initialized;
    // Calibration offsets
    float accelXOffset;
    float accelYOffset;
    float accelZOffset;
    float gyroXOffset;
    float gyroYOffset;
    float gyroZOffset;

    // Private helper functions
    int16_t readRegister16(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readRawData(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
};

#endif // IMU_HANDLER_H