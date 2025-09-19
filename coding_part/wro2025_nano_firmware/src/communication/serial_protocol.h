// serial_protocol.h
// Defines the serial communication protocol between Nano and Pi

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

// Packet structure constants
#define PACKET_START_BYTE 0xAA
#define PACKET_MAX_LENGTH 32
#define CMD_ALL_SENSOR_DATA_COMPACT 0x13 // New command for compact data
// Command IDs from Pi to Nano
#define CMD_SET_MOTOR_SPEED 0x01
#define CMD_SET_SERVO_ANGLE 0x02
#define CMD_EMERGENCY_STOP 0x03
#define CMD_HEARTBEAT 0x04
#define CMD_REQUEST_SENSOR_DATA 0x05

// Command IDs from Nano to Pi
#define CMD_SENSOR_DATA 0x10
#define CMD_START_ACK 0x11
#define CMD_ERROR 0x12

// Structure for packet handling
struct Packet
{
    byte command;
    byte length;
    byte data[PACKET_MAX_LENGTH];
    byte checksum;
};
struct __attribute__((packed)) CompactSensorData
{
    uint16_t frontDistance; // 0-65535 cm (covers US range well) - 2 bytes
    uint16_t rightDistance; // 0-65535 cm - 2 bytes
    uint16_t backDistance;  // 0-65535 cm - 2 bytes
    uint16_t leftDistance;  // 0-65535 cm - 2 bytes
    int16_t accelX;         // Scaled IMU data (e.g., milli-g) - 2 bytes
    int16_t accelY;         // Scaled IMU data (e.g., milli-g) - 2 bytes
    int16_t accelZ;         // Scaled IMU data (e.g., milli-g) - 2 bytes
    int16_t gyroX;          // Scaled IMU data (e.g., milli-deg/s) - 2 bytes
    int16_t gyroY;          // Scaled IMU data (e.g., milli-deg/s) - 2 bytes
    int16_t gyroZ;          // Scaled IMU data (e.g., milli-deg/s) - 2 bytes
                            // Total: 20 bytes
};

// Function prototypes
byte calculateChecksum(const byte *data, size_t length);
bool isValidPacket(const Packet &packet);
void createPacket(Packet &packet, byte command, byte length, const byte *data);

#endif // SERIAL_PROTOCOL_H