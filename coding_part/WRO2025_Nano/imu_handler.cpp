// imu_handler.cpp
// Implementation of the IMU handler for MPU6050

#include "imu_handler.h"
#include <Wire.h>

// MPU6050 I2C address
#define MPU6050_ADDRESS 0x68

// MPU6050 registers
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_WHO_AM_I 0x75

// Configuration registers
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

// Power management
#define MPU6050_PWR1_SLEEP_BIT 6
#define MPU6050_PWR1_CLKSEL_BIT 2
#define MPU6050_PWR1_CLKSEL_LENGTH 3

// Gyro configuration
#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

// Accelerometer configuration
#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_ACCEL_FS_8 0x02
#define MPU6050_ACCEL_FS_16 0x03

IMUHandler::IMUHandler() : _initialized(false),
                           accelXOffset(0), accelYOffset(0), accelZOffset(0),
                           gyroXOffset(0), gyroYOffset(0), gyroZOffset(0)
{
}

bool IMUHandler::initialize()
{
    Wire.begin();

    // Check if sensor is connected
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_RA_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 1, true);

    if (Wire.available())
    {
        uint8_t deviceId = Wire.read();
        if (deviceId != 0x68)
        {
            return false; // Not the expected device ID
        }
    }
    else
    {
        return false; // No response from sensor
    }

    // Wake up the MPU-6050
    writeRegister(MPU6050_RA_PWR_MGMT_1, 0x00);

    // Set clock source to X Gyro
    writeRegister(MPU6050_RA_PWR_MGMT_1, 0x01);

    // Configure accelerometer range (+/- 2g)
    writeRegister(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << 3);

    // Configure gyro range (+/- 250 deg/s)
    writeRegister(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250 << 3);

    // Set digital low pass filter
    writeRegister(MPU6050_RA_CONFIG, 0x03); // ~44Hz bandwidth

    _initialized = true;
    return true;
}

bool IMUHandler::readData(IMUData &data)
{
    if (!_initialized)
    {
        return false;
    }

    int16_t ax, ay, az, gx, gy, gz;
    readRawData(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw values to meaningful units
    // Accelerometer: 2g range, 16384 LSB/g
    data.accelX = (ax - accelXOffset) / 16384.0f;
    data.accelY = (ay - accelYOffset) / 16384.0f;
    data.accelZ = (az - accelZOffset) / 16384.0f;

    // Gyroscope: 250 deg/s range, 131 LSB/(deg/s)
    data.gyroX = (gx - gyroXOffset) / 131.0f;
    data.gyroY = (gy - gyroYOffset) / 131.0f;
    data.gyroZ = (gz - gyroZOffset) / 131.0f;

    // Temperature: 340 LSB/°C, -15.5556 LSB/°C offset
    // Note: This is a simplified temperature conversion
    data.temperature = 0.0f; // Placeholder - would need proper implementation

    return true;
}

void IMUHandler::calibrate()
{
    if (!_initialized)
    {
        return;
    }

    // Reset offsets
    accelXOffset = 0;
    accelYOffset = 0;
    accelZOffset = 0;
    gyroXOffset = 0;
    gyroYOffset = 0;
    gyroZOffset = 0;

    // Collect samples for calibration
    const int samples = 1000;
    long accelXSum = 0, accelYSum = 0, accelZSum = 0;
    long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

    for (int i = 0; i < samples; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        readRawData(&ax, &ay, &az, &gx, &gy, &gz);

        accelXSum += ax;
        accelYSum += ay;
        accelZSum += az;
        gyroXSum += gx;
        gyroYSum += gy;
        gyroZSum += gz;

        delay(5); // Small delay between samples
    }

    // Calculate average offsets
    accelXOffset = accelXSum / samples;
    accelYOffset = accelYSum / samples;
    accelZOffset = accelZSum / samples - 16384; // Subtract 1g for Z-axis
    gyroXOffset = gyroXSum / samples;
    gyroYOffset = gyroYSum / samples;
    gyroZOffset = gyroZSum / samples;
}

int16_t IMUHandler::readRegister16(uint8_t reg)
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 2, true);

    if (Wire.available() < 2)
    {
        return 0;
    }

    int16_t value = (Wire.read() << 8) | Wire.read();
    return value;
}

void IMUHandler::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

void IMUHandler::readRawData(int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz)
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true); // Read 14 bytes (accel + temp + gyro)

    if (Wire.available() == 14)
    {
        *ax = (Wire.read() << 8) | Wire.read();
        *ay = (Wire.read() << 8) | Wire.read();
        *az = (Wire.read() << 8) | Wire.read();
        // Skip temperature for now
        Wire.read();
        Wire.read();
        *gx = (Wire.read() << 8) | Wire.read();
        *gy = (Wire.read() << 8) | Wire.read();
        *gz = (Wire.read() << 8) | Wire.read();
    }
}