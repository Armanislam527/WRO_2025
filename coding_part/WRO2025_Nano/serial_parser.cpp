// serial_parser.cpp
// Implementation of the serial parser

#include "serial_parser.h"

SerialParser::SerialParser() : currentState(STATE_IDLE), dataBytesReceived(0)
{
    // Initialize the current packet
    memset(&currentPacket, 0, sizeof(Packet));
}

bool SerialParser::readPacket(Packet &packet)
{
    while (Serial.available())
    {
        byte incomingByte = Serial.read();

        switch (currentState)
        {
        case STATE_IDLE:
            if (incomingByte == PACKET_START_BYTE)
            {
                currentState = STATE_START_RECEIVED;
            }
            break;

        case STATE_START_RECEIVED:
            currentPacket.command = incomingByte;
            currentState = STATE_COMMAND_RECEIVED;
            break;

        case STATE_COMMAND_RECEIVED:
            currentPacket.length = incomingByte;
            // Validate length
            if (currentPacket.length > PACKET_MAX_LENGTH)
            {
                resetParser();
            }
            else
            {
                dataBytesReceived = 0;
                currentState = STATE_LENGTH_RECEIVED;
            }
            break;

        case STATE_LENGTH_RECEIVED:
            if (dataBytesReceived < currentPacket.length)
            {
                currentPacket.data[dataBytesReceived] = incomingByte;
                dataBytesReceived++;
            }

            // Check if we've received all data bytes
            if (dataBytesReceived >= currentPacket.length)
            {
                currentState = STATE_DATA_RECEIVED;
            }
            break;

        case STATE_DATA_RECEIVED:
            currentPacket.checksum = incomingByte;
            currentState = STATE_CHECKSUM_RECEIVED;

            // Validate packet
            if (isValidPacket(currentPacket))
            {
                // Copy to output parameter
                memcpy(&packet, &currentPacket, sizeof(Packet));
                resetParser();
                return true; // Valid packet received
            }
            else
            {
                // Invalid packet, reset parser
                resetParser();
            }
            break;

        default:
            resetParser();
            break;
        }
    }

    return false; // No complete packet received
}

void SerialParser::sendPacket(const Packet &packet)
{
    // Send start byte
    Serial.write(PACKET_START_BYTE);

    // Send command
    Serial.write(packet.command);

    // Send length
    Serial.write(packet.length);

    // Send data
    for (int i = 0; i < packet.length; i++)
    {
        Serial.write(packet.data[i]);
    }

    // Send checksum
    Serial.write(packet.checksum);
}

void SerialParser::sendSensorData(unsigned int frontDist, unsigned int rightDist,
                                  unsigned int backDist, unsigned int leftDist)
{
    Packet packet;
    byte data[8];

    // Pack distance data (2 bytes per distance)
    data[0] = (frontDist >> 8) & 0xFF;
    data[1] = frontDist & 0xFF;
    data[2] = (rightDist >> 8) & 0xFF;
    data[3] = rightDist & 0xFF;
    data[4] = (backDist >> 8) & 0xFF;
    data[5] = backDist & 0xFF;
    data[6] = (leftDist >> 8) & 0xFF;
    data[7] = leftDist & 0xFF;

    createPacket(packet, CMD_SENSOR_DATA, 8, data);
    sendPacket(packet);
}

void SerialParser::sendStartAck(bool started)
{
    Packet packet;
    byte data[1] = {started ? 0x01 : 0x00};

    createPacket(packet, CMD_START_ACK, 1, data);
    sendPacket(packet);
}

void SerialParser::sendError(byte errorCode)
{
    Packet packet;
    byte data[1] = {errorCode};

    createPacket(packet, CMD_ERROR, 1, data);
    sendPacket(packet);
}

void SerialParser::resetParser()
{
    currentState = STATE_IDLE;
    dataBytesReceived = 0;
    memset(&currentPacket, 0, sizeof(Packet));
}
void SerialParser::sendCompactSensorData(const CompactSensorData &data)
{
    Packet packet;

    // Cast the struct directly to byte array for transmission
    // This is very fast as it avoids manual packing/unpacking
    const byte *dataPtr = reinterpret_cast<const byte *>(&data);
    byte dataLength = sizeof(CompactSensorData); // Should be 20

    // Use the existing createPacket function with our new command ID
    createPacket(packet, CMD_ALL_SENSOR_DATA_COMPACT, dataLength, dataPtr);

    // Send the packet using the existing robust sendPacket method
    sendPacket(packet);
}