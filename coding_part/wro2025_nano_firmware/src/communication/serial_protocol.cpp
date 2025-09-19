// serial_protocol.cpp
// Implementation of serial communication protocol utilities

#include "serial_protocol.h"

// Calculate checksum using XOR
byte calculateChecksum(const byte *data, size_t length)
{
    byte checksum = 0;
    for (size_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

// Verify if a packet has a valid checksum
bool isValidPacket(const Packet &packet)
{
    // Create temporary buffer for checksum calculation
    byte tempBuffer[PACKET_MAX_LENGTH + 2]; // +2 for command and length
    tempBuffer[0] = packet.command;
    tempBuffer[1] = packet.length;

    // Copy data bytes
    for (int i = 0; i < packet.length && i < PACKET_MAX_LENGTH; i++)
    {
        tempBuffer[i + 2] = packet.data[i];
    }

    // Calculate expected checksum
    byte expectedChecksum = calculateChecksum(tempBuffer, packet.length + 2);

    // Compare with received checksum
    return (expectedChecksum == packet.checksum);
}

// Create a packet with the specified parameters
void createPacket(Packet &packet, byte command, byte length, const byte *data)
{
    packet.command = command;
    packet.length = length;

    // Copy data (up to maximum length)
    for (int i = 0; i < length && i < PACKET_MAX_LENGTH; i++)
    {
        packet.data[i] = data[i];
    }

    // Create temporary buffer for checksum calculation
    byte tempBuffer[PACKET_MAX_LENGTH + 2]; // +2 for command and length
    tempBuffer[0] = packet.command;
    tempBuffer[1] = packet.length;

    // Copy data bytes
    for (int i = 0; i < packet.length && i < PACKET_MAX_LENGTH; i++)
    {
        tempBuffer[i + 2] = packet.data[i];
    }

    // Calculate checksum
    packet.checksum = calculateChecksum(tempBuffer, packet.length + 2);
}