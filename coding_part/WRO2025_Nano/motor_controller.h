// motor_controller.h
// Handles control of the vehicle's driving motor via L298N

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController
{
public:
    MotorController();
    void initialize(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin);
    void setSpeed(int speed); // Speed from -255 (full reverse) to 255 (full forward)
    void stop();
    int getCurrentSpeed() const;

private:
    uint8_t _enaPin;
    uint8_t _in1Pin;
    uint8_t _in2Pin;
    int _currentSpeed;
    void _setMotorDirection(bool forward);
};

#endif // MOTOR_CONTROLLER_H