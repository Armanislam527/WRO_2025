// serial_parser.h
// Handles parsing of incoming serial data and sending packets

#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>
#include "serial_protocol.h"

class SerialParser
{
public:
    SerialParser();
    bool readPacket(Packet &packet);
    void sendPacket(const Packet &packet);
    void sendSensorData(unsigned int frontDist, unsigned int rightDist,
                        unsigned int backDist, unsigned int leftDist);
    void sendStartAck(bool started);
    void sendError(byte errorCode);
    void sendCompactSensorData(const CompactSensorData &data);

private:
    enum ParseState
    {
        STATE_IDLE,
        STATE_START_RECEIVED,
        STATE_COMMAND_RECEIVED,
        STATE_LENGTH_RECEIVED,
        STATE_DATA_RECEIVED,
        STATE_CHECKSUM_RECEIVED
    };

    ParseState currentState;
    Packet currentPacket;
    byte dataBytesReceived;

    void resetParser();
};

#endif // SERIAL_PARSER_H