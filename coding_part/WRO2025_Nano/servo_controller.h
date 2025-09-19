// servo_controller.h
// Handles control of the vehicle's steering servo

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController
{
public:
    ServoController();
    void initialize(uint8_t servoPin);
    void setAngle(int angle); // Angle from 0 to 180 degrees
    void center();
    int getCurrentAngle() const;
    void detach();
    void reattach();

private:
    Servo _steeringServo;
    uint8_t _servoPin;
    int _currentAngle;
    bool _isAttached;
};

#endif // SERVO_CONTROLLER_H