// e_stop.cpp
// Implementation of the emergency stop handler

#include "e_stop.h"
// We'll need to include the motor and servo controllers
// In a full implementation, you might use forward declarations or pass references
// For now, we'll assume these functions exist or are declared elsewhere
// #include "actuators/motor_controller.h"
// #include "actuators/servo_controller.h"

// Since we don't have direct access to motor/servo controllers here,
// we'll use the pin definitions from config.h
#include "config.h"

EStopHandler::EStopHandler() : _eStopActive(false)
{
}

void EStopHandler::initialize()
{
    // Emergency stop state is managed internally
    // In a more complex system, you might have a physical e-stop button
    // For now, we rely on software commands
    _eStopActive = false;
}

void EStopHandler::activateEStop()
{
    // Only activate if not already active
    if (!_eStopActive)
    {
        _eStopActive = true;

        // Immediately stop the motor
        // In a full implementation, you would call motorController.stop()
        digitalWrite(MOTOR_ENA_PIN, LOW);
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, LOW);

        // Center the servo (safe steering position)
        // In a full implementation, you would call servoController.center()
        // For now, we'll assume a servo object is available or we control the pin directly
        // This is a simplification - normally you'd use the Servo library functions

        // Indicate emergency stop activation with LED
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
}

bool EStopHandler::isEStopActive() const
{
    return _eStopActive;
}

void EStopHandler::resetEStop()
{
    // Only reset if currently in e-stop state
    if (_eStopActive)
    {
        _eStopActive = false;

        // Turn off emergency stop indicator
        digitalWrite(STATUS_LED_PIN, LOW);
    }
}

void EStopHandler::handleEStopCommand()
{
    // Simply activate the emergency stop when command is received
    activateEStop();
}