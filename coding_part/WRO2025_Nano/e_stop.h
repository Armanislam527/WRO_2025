// e_stop.h
// Handles emergency stop functionality for immediate vehicle halt

#ifndef E_STOP_H
#define E_STOP_H

#include <Arduino.h>

class EStopHandler
{
public:
    EStopHandler();

    // Initialize the emergency stop handler
    void initialize();

    // Activate emergency stop - immediately stops all vehicle movement
    void activateEStop();

    // Check if emergency stop is currently active
    bool isEStopActive() const;

    // Reset emergency stop condition (only call after resolving the cause)
    void resetEStop();

    // Handle emergency stop command from serial communication
    void handleEStopCommand();

private:
    bool _eStopActive;
};

#endif // E_STOP_H