// motor_controller.cpp
// Implementation of the motor controller for L298N

#include "motor_controller.h"

MotorController::MotorController() : _enaPin(0), _in1Pin(0), _in2Pin(0), _currentSpeed(0)
{
}

void MotorController::initialize(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin)
{
    _enaPin = enaPin;
    _in1Pin = in1Pin;
    _in2Pin = in2Pin;
    digitalWrite(_enaPin, LOW);
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    // Set pin modes
    pinMode(_enaPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    _currentSpeed = 0;
    // Initialize motor to stopped state
    stop();
}

void MotorController::setSpeed(int speed)
{
    // Constrain speed to valid range
    if (speed > 255)
        speed = 255;
    if (speed < -255)
        speed = -255;

    _currentSpeed = speed;

    if (speed == 0)
    {
        // Stop the motor
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, LOW);
        analogWrite(_enaPin, 0);
    }
    else if (speed > 0)
    {
        // Forward direction
        _setMotorDirection(true);
        analogWrite(_enaPin, speed);
    }
    else
    {
        // Reverse direction
        _setMotorDirection(false);
        analogWrite(_enaPin, -speed);
    }
}

void MotorController::stop()
{
    _currentSpeed = 0;
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    analogWrite(_enaPin, 0);
}

int MotorController::getCurrentSpeed() const
{
    return _currentSpeed;
}

void MotorController::_setMotorDirection(bool forward)
{
    if (forward)
    {
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
    }
    else
    {
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
    }
}