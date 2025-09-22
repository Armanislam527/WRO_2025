// serial_handler.h
// Manages serial communication with the Arduino Nano

#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <string>
#include <cstdint>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
// Include the sensor data struct that matches Nano's CompactSensorData
#include "../sensors/sensor_data.h"

// Forward declarations for packet structures if needed, or include serial_protocol.h
// #include "serial_protocol.h"

class SerialHandler
{
public:
    SerialHandler();
    ~SerialHandler();

    // Open and close the serial port
    bool openPort(const std::string &port, uint32_t baudRate);
    void closePort();

    // --- Sending Commands TO Nano ---
    bool sendMotorSpeedCommand(int8_t speed);  // -128 to 127
    bool sendServoAngleCommand(uint8_t angle); // 0 to 180
    bool sendHeartbeat();
    bool sendGoCommand();     // Pi's "GO" signal
    bool sendEmergencyStop(); // Just in case Pi needs to E-Stop

    // --- Receiving Data FROM Nano ---
    // Get the latest sensor data received
    bool getLatestSensorData(SensorData &data);

    // Check for specific packets
    bool hasStartAck() const;
    void acknowledgeStartAck(); // Clear the start ack flag

    // Thread-safe method to process incoming data (called by receive thread)
    void processData(const uint8_t *buffer, size_t length);

private:
    std::string portName;
    int serialFd; // File descriptor for the serial port
    std::atomic<bool> isOpen;

    // Receiving thread
    std::thread receiveThread;
    std::atomic<bool> stopReceiving;

    // Data buffering and synchronization
    mutable std::mutex dataMutex;
    SensorData latestSensorData;
    std::atomic<bool> sensorDataUpdated;

    size_t rxBufferIndex;
    std::atomic<bool> startAckReceived;

    // Buffer for incoming raw bytes
    static const size_t RX_BUFFER_SIZE = 256;
    uint8_t rxBuffer[RX_BUFFER_SIZE];

    // Packet parsing state
    enum class ParseState
    {
        IDLE,
        START_RECEIVED,
        COMMAND_RECEIVED,
        LENGTH_RECEIVED,
        DATA_RECEIVED,
        CHECKSUM_RECEIVED
    };
    ParseState currentState;

    // --- Private Helper Functions ---
    void receiveLoop();                                  // Function run by the receive thread
    void resetParser();                                  // Reset packet parser state
    bool writeBytes(const uint8_t *data, size_t length); // Send raw bytes

    // Packet creation helpers (mirror Nano's protocol)
    void createAndSendPacket(uint8_t command, uint8_t length, const uint8_t *data);
    uint8_t calculateChecksum(const uint8_t *data, size_t length);
};

#endif // SERIAL_HANDLER_H