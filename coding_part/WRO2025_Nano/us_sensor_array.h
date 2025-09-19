// us_sensor_array.h
// Handles the ultrasonic sensor array for distance measurement

#ifndef US_SENSOR_ARRAY_H
#define US_SENSOR_ARRAY_H

#include <Arduino.h>
#include <NewPing.h>

// Structure to hold ultrasonic sensor data
struct USSensorData
{
    unsigned int frontDistance;
    unsigned int rightDistance;
    unsigned int backDistance;
    unsigned int leftDistance;
};

class USSensorArray
{
public:
    USSensorArray();
    void initialize(uint8_t frontTrigPin, uint8_t frontEchoPin,
                    uint8_t rightTrigPin, uint8_t rightEchoPin,
                    uint8_t backTrigPin, uint8_t backEchoPin,
                    uint8_t leftTrigPin, uint8_t leftEchoPin);
    bool readData(USSensorData &data);
    void setMaxDistance(unsigned int maxDistance);

private:
    NewPing *usFront;
    NewPing *usRight;
    NewPing *usBack;
    NewPing *usLeft;
    unsigned int maxDistance;
};

#endif // US_SENSOR_ARRAY_H