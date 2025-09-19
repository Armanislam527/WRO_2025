// watchdog.h
// Implements a safety watchdog to monitor communication with the Pi

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>

class Watchdog
{
public:
    Watchdog();
    void initialize(unsigned long timeoutMs);
    void feed();                    // Reset the watchdog timer
    bool isExpired();               // Check if the watchdog has timed out
    unsigned long getElapsedTime(); // Get time since last feed

private:
    unsigned long _timeout;
    unsigned long _lastFeedTime;
    bool _initialized;
};

#endif // WATCHDOG_H