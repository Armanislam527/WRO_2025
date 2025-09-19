// watchdog.cpp
// Implementation of the safety watchdog

#include "watchdog.h"

Watchdog::Watchdog() : _timeout(0), _lastFeedTime(0), _initialized(false)
{
}

void Watchdog::initialize(unsigned long timeoutMs)
{
    _timeout = timeoutMs;
    _lastFeedTime = millis();
    _initialized = true;
}

void Watchdog::feed()
{
    if (_initialized)
    {
        _lastFeedTime = millis();
    }
}

bool Watchdog::isExpired()
{
    if (!_initialized)
    {
        return false; // If not initialized, don't trigger
    }

    unsigned long currentTime = millis();

    // Handle millis() overflow (occurs every ~49 days)
    if (currentTime < _lastFeedTime)
    {
        // Overflow occurred, treat as expired if timeout was very long (unlikely)
        // In most cases, this means we recently fed the watchdog
        return false;
    }

    return (currentTime - _lastFeedTime) > _timeout;
}

unsigned long Watchdog::getElapsedTime()
{
    if (!_initialized)
    {
        return 0;
    }

    unsigned long currentTime = millis();

    // Handle millis() overflow
    if (currentTime < _lastFeedTime)
    {
        // This is a simplification - in an overflow case, the elapsed time
        // would be very large, indicating expiration
        return _timeout + 1; // Ensure it appears expired
    }

    return currentTime - _lastFeedTime;
}