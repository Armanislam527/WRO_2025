// servo_controller.cpp
// Implementation of the servo controller

#include "servo_controller.h"

ServoController::ServoController() : _servoPin(0), _currentAngle(90), _isAttached(false)
{
}

void ServoController::initialize(uint8_t servoPin)
{
    _servoPin = servoPin;
    _currentAngle = 90; // Default to center position

    // Attach the servo to the specified pin
    _steeringServo.attach(_servoPin);
    _isAttached = true;

    // Set to center position initially
    _steeringServo.write(_currentAngle);
}

void ServoController::setAngle(int angle)
{
    // Constrain angle to valid range
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    _currentAngle = angle;

    // Only write to servo if it's attached
    if (_isAttached)
    {
        _steeringServo.write(_currentAngle);
    }
}

void ServoController::center()
{
    setAngle(90);
}

int ServoController::getCurrentAngle() const
{
    return _currentAngle;
}

void ServoController::detach()
{
    if (_isAttached)
    {
        _steeringServo.detach();
        _isAttached = false;
    }
}

void ServoController::reattach()
{
    if (!_isAttached)
    {
        _steeringServo.attach(_servoPin);
        _isAttached = true;
        // Restore the last known position
        _steeringServo.write(_currentAngle);
    }
}