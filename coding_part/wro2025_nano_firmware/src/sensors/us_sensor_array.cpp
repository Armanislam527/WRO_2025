// us_sensor_array.cpp
// Implementation of the ultrasonic sensor array handler

#include "us_sensor_array.h"

USSensorArray::USSensorArray() : usFront(nullptr), usRight(nullptr), usBack(nullptr), usLeft(nullptr),
                                 maxDistance(200)
{
}

void USSensorArray::initialize(uint8_t frontTrigPin, uint8_t frontEchoPin,
                               uint8_t rightTrigPin, uint8_t rightEchoPin,
                               uint8_t backTrigPin, uint8_t backEchoPin,
                               uint8_t leftTrigPin, uint8_t leftEchoPin)
{
    // Delete existing objects if they exist
    if (usFront)
        delete usFront;
    if (usRight)
        delete usRight;
    if (usBack)
        delete usBack;
    if (usLeft)
        delete usLeft;

    // Create new NewPing objects for each sensor
    usFront = new NewPing(frontTrigPin, frontEchoPin, maxDistance);
    usRight = new NewPing(rightTrigPin, rightEchoPin, maxDistance);
    usBack = new NewPing(backTrigPin, backEchoPin, maxDistance);
    usLeft = new NewPing(leftTrigPin, leftEchoPin, maxDistance);
}

bool USSensorArray::readData(USSensorData &data)
{
    // Check if sensors are initialized
    if (!usFront || !usRight || !usBack || !usLeft)
    {
        return false;
    }

    // Read distances from each sensor
    // Using ping_cm() for simplicity, though ping() with microsecond timing would be more precise
    data.frontDistance = usFront->ping_cm();
    data.rightDistance = usRight->ping_cm();
    data.backDistance = usBack->ping_cm();
    data.leftDistance = usLeft->ping_cm();

    return true;
}

void USSensorArray::setMaxDistance(unsigned int maxDistance)
{
    this->maxDistance = maxDistance;

    // If sensors are already initialized, we need to reinitialize them
    // This is a limitation of the NewPing library
    // In a real implementation, you might want to store the pin numbers and reinitialize
}