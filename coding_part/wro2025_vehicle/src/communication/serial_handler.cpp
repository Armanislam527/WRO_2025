// serial_handler.cpp
// Implementation of SerialHandler

#include "serial_handler.h"
#include <iostream>
#include <unistd.h>  // read, write, close
#include <fcntl.h>   // open, O_RDWR, O_NOCTTY, O_NDELAY
#include <errno.h>   // errno
#include <termios.h> // tcgetattr, tcsetattr, cfsetispeed, cfsetospeed, tcflush
#include <cstring>   // memset, memcpy
#include <stdexcept> // std::runtime_error

// Include command definitions (mirror Nano's serial_protocol.h)
// Assuming these are defined in a common header or we define them here for now
#ifndef CMD_SET_MOTOR_SPEED
#define CMD_SET_MOTOR_SPEED 0x01
#define CMD_SET_SERVO_ANGLE 0x02
#define CMD_EMERGENCY_STOP 0x03
#define CMD_HEARTBEAT 0x04
#define CMD_PI_GO_SIGNAL 0x06            // Pi's GO command
#define CMD_START_ACK 0x11               // Nano's start button ack
#define CMD_ALL_SENSOR_DATA_COMPACT 0x13 // Nano's compact sensor data
#define PACKET_START_BYTE 0xAA
#endif

SerialHandler::SerialHandler()
    : serialFd(-1), isOpen(false), stopReceiving(false), sensorDataUpdated(false), rxBufferIndex(0), startAckReceived(false), currentState(ParseState::IDLE)
{
    resetParser();
    // Initialize latestSensorData to zero or default values if needed
    memset(&latestSensorData, 0, sizeof(SensorData));
}

SerialHandler::~SerialHandler()
{
    closePort();
}

bool SerialHandler::openPort(const std::string &port, uint32_t baudRate)
{
    if (isOpen)
    {
        std::cerr << "Serial port already open." << std::endl;
        return false;
    }

    portName = port;
    serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd == -1)
    {
        perror("open_port: Unable to open serial port");
        return false;
    }

    struct termios options;
    tcgetattr(serialFd, &options);

    // Set baud rate
    speed_t baudSymbol;
    switch (baudRate)
    {
    case 9600:
        baudSymbol = B9600;
        break;
    case 19200:
        baudSymbol = B19200;
        break;
    case 38400:
        baudSymbol = B38400;
        break;
    case 57600:
        baudSymbol = B57600;
        break;
    case 115200:
        baudSymbol = B115200;
        break;
    // Add more cases if needed
    default:
        std::cerr << "Unsupported baud rate: " << baudRate << std::endl;
        close(serialFd);
        serialFd = -1;
        return false;
    }
    cfsetispeed(&options, baudSymbol);
    cfsetospeed(&options, baudSymbol);

    // Configure for raw input/output
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Mask character size bits
    options.c_cflag |= CS8;              // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    options.c_oflag &= ~OPOST;                          // Raw output

    // Set read behavior: minimum chars = 0, timeout = 1 decisecond (0.1 seconds)
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(serialFd, TCSANOW, &options);
    tcflush(serialFd, TCIOFLUSH);

    isOpen = true;
    stopReceiving = false;

    // Start the receiving thread
    receiveThread = std::thread(&SerialHandler::receiveLoop, this);

    std::cout << "Serial port " << portName << " opened at " << baudRate << " baud." << std::endl;
    return true;
}

void SerialHandler::closePort()
{
    if (isOpen)
    {
        stopReceiving = true;
        if (receiveThread.joinable())
        {
            receiveThread.join();
        }
        close(serialFd);
        serialFd = -1;
        isOpen = false;
        std::cout << "Serial port " << portName << " closed." << std::endl;
    }
}

// --- Sending Commands ---

bool SerialHandler::sendMotorSpeedCommand(int8_t speed)
{
    if (!isOpen)
        return false;
    uint8_t data = static_cast<uint8_t>(speed);
    createAndSendPacket(CMD_SET_MOTOR_SPEED, 1, &data);
    return true; // Assume success for now
}

bool SerialHandler::sendServoAngleCommand(uint8_t angle)
{
    if (!isOpen)
        return false;
    createAndSendPacket(CMD_SET_SERVO_ANGLE, 1, &angle);
    return true;
}

bool SerialHandler::sendHeartbeat()
{
    if (!isOpen)
        return false;
    createAndSendPacket(CMD_HEARTBEAT, 0, nullptr);
    return true;
}

bool SerialHandler::sendGoCommand()
{
    if (!isOpen)
        return false;
    createAndSendPacket(CMD_PI_GO_SIGNAL, 0, nullptr);
    return true;
}

bool SerialHandler::sendEmergencyStop()
{
    if (!isOpen)
        return false;
    createAndSendPacket(CMD_EMERGENCY_STOP, 0, nullptr);
    return true;
}

// --- Receiving Data ---

bool SerialHandler::getLatestSensorData(SensorData &data)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    if (sensorDataUpdated)
    {
        data = latestSensorData;
        sensorDataUpdated = false; // Consume the data
        return true;
    }
    return false; // No new data
}

bool SerialHandler::hasStartAck() const
{
    return startAckReceived.load();
}

void SerialHandler::acknowledgeStartAck()
{
    startAckReceived.store(false);
}

// --- Internal Logic ---

void SerialHandler::receiveLoop()
{
    uint8_t buffer[256];
    while (!stopReceiving)
    {
        int n = read(serialFd, buffer, sizeof(buffer));
        if (n > 0)
        {
            processData(buffer, n);
        }
        else if (n < 0 && errno != EAGAIN)
        {
            // Handle read error
            perror("Serial read error");
            // Depending on error, might want to close port or signal error
        }
        // Small sleep to prevent busy-waiting on read
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SerialHandler::processData(const uint8_t *buffer, size_t length)
{
    for (size_t i = 0; i < length; ++i)
    {
        uint8_t incomingByte = buffer[i];

        switch (currentState)
        {
        case ParseState::IDLE:
            if (incomingByte == PACKET_START_BYTE)
            {
                currentState = ParseState::START_RECEIVED;
            }
            break;

        case ParseState::START_RECEIVED:
            // This byte is the command
            rxBuffer[0] = incomingByte; // Store command in buffer[0]
            currentState = ParseState::COMMAND_RECEIVED;
            break;

        case ParseState::COMMAND_RECEIVED:
            // This byte is the length
            rxBuffer[1] = incomingByte; // Store length in buffer[1]
            rxBufferIndex = 0;          // Reset data index
            if (incomingByte == 0)
            {
                currentState = ParseState::DATA_RECEIVED; // No data, go straight to checksum
            }
            else
            {
                currentState = ParseState::LENGTH_RECEIVED;
            }
            break;

        case ParseState::LENGTH_RECEIVED:
            // Collect data bytes
            if (rxBufferIndex < sizeof(rxBuffer) - 2)
            {                                               // -2 for command, length
                rxBuffer[rxBufferIndex + 2] = incomingByte; // Store data starting from index 2
                rxBufferIndex++;
                if (rxBufferIndex >= rxBuffer[1])
                { // Length byte is rxBuffer[1]
                    currentState = ParseState::DATA_RECEIVED;
                }
            }
            else
            {
                // Buffer overflow, reset
                resetParser();
            }
            break;

        case ParseState::DATA_RECEIVED:
        {
            // This byte is the checksum
            rxBuffer[rxBufferIndex + 2] = incomingByte; // Store checksum
            currentState = ParseState::CHECKSUM_RECEIVED;

            // Validate packet
            uint8_t receivedChecksum = rxBuffer[rxBufferIndex + 2];
            uint8_t calculatedChecksum = calculateChecksum(rxBuffer, rxBufferIndex + 2);

            if (receivedChecksum == calculatedChecksum)
            {
                // Process valid packet
                uint8_t command = rxBuffer[0];
                uint8_t dataLength = rxBuffer[1];

                if (command == CMD_START_ACK && dataLength >= 1)
                {
                    // Nano acknowledged the start button press
                    startAckReceived.store(true);
                    std::cout << "Received START_ACK from Nano." << std::endl;
                }
                else if (command == CMD_ALL_SENSOR_DATA_COMPACT && dataLength == sizeof(SensorData))
                {
                    // Received compact sensor data
                    std::lock_guard<std::mutex> lock(dataMutex);
                    // Cast the data part of the buffer directly to SensorData
                    // Buffer layout: [CMD][LEN][DATA...][CHK] -> Data starts at index 2
                    memcpy(&latestSensorData, &rxBuffer[2], sizeof(SensorData));
                    sensorDataUpdated.store(true);
                    // std::cout << "Received new sensor data packet." << std::endl; // Verbose
                }
                // Add handling for other packet types (CMD_ERROR, etc.) if needed
            }
            else
            {
                std::cerr << "Serial packet checksum error!" << std::endl;
            }
        }
            resetParser();
            break;
        case ParseState::CHECKSUM_RECEIVED:
            // This state is handled and transitions in DATA_RECEIVED case.
            // If we somehow land here, it's an unexpected state.
            [[fallthrough]]; // C++17 attribute, or just let it fall through
        default:
            resetParser();
            break;
        }
    }
}

void SerialHandler::resetParser()
{
    currentState = ParseState::IDLE;
    rxBufferIndex = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
}

bool SerialHandler::writeBytes(const uint8_t *data, size_t length)
{
    if (!isOpen)
        return false;
    ssize_t result = write(serialFd, data, length);
    if (result == -1)
    {
        perror("Serial write error");
        return false;
    }
    return (static_cast<size_t>(result) == length);
}

void SerialHandler::createAndSendPacket(uint8_t command, uint8_t length, const uint8_t *data)
{
    // Calculate total packet size
    size_t packetSize = 3 + length; // START + CMD + LEN + DATA + CHK
    uint8_t *packet = new uint8_t[packetSize];

    packet[0] = PACKET_START_BYTE;
    packet[1] = command;
    packet[2] = length;

    if (length > 0 && data != nullptr)
    {
        memcpy(&packet[3], data, length);
    }

    uint8_t checksum = calculateChecksum(&packet[1], 1 + length); // CHK = CMD ^ LEN ^ DATA...
    packet[3 + length] = checksum;

    writeBytes(packet, packetSize);
    delete[] packet;
}

uint8_t SerialHandler::calculateChecksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i)
    {
        checksum ^= data[i];
    }
    return checksum;
}