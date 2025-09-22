// src/utils/logger.h
// A simple conditional logger with verbosity levels for debugging.

#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <string>

// --- VERBOSITY LEVELS ---
// Define LOG_LEVEL to control verbosity.
// Higher numbers mean more verbose output.
// 0 = OFF, 1 = ERROR, 2 = WARN, 3 = INFO, 4 = DEBUG, 5 = VERBOSE
// You can set this via CMake: add_compile_definitions(LOG_LEVEL=4)
#ifndef LOG_LEVEL
#define LOG_LEVEL 0 // Default to OFF
#endif
// Macro to extract filename from __FILE__ for cleaner logs
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
// --- MACROS BASED ON LOG_LEVEL ---
#if LOG_LEVEL >= 1
#define LOG_ERROR(msg)                                                                           \
    do                                                                                           \
    {                                                                                            \
        std::cerr << "[ERROR] (" << __FILENAME__ << ":" << __LINE__ << ") " << msg << std::endl; \
    } while (0)
#else
#define LOG_ERROR(msg) \
    do                 \
    {                  \
    } while (0)
#endif

#if LOG_LEVEL >= 2
#define LOG_WARN(msg)                                                                            \
    do                                                                                           \
    {                                                                                            \
        std::cerr << "[WARN]  (" << __FILENAME__ << ":" << __LINE__ << ") " << msg << std::endl; \
    } while (0)
#else
#define LOG_WARN(msg) \
    do                \
    {                 \
    } while (0)
#endif

#if LOG_LEVEL >= 3
#define LOG_INFO(msg)                                                                            \
    do                                                                                           \
    {                                                                                            \
        std::cerr << "[INFO]  (" << __FILENAME__ << ":" << __LINE__ << ") " << msg << std::endl; \
    } while (0)
#else
#define LOG_INFO(msg) \
    do                \
    {                 \
    } while (0)
#endif

#if LOG_LEVEL >= 4
#define LOG_DEBUG(msg)                                                                           \
    do                                                                                           \
    {                                                                                            \
        std::cerr << "[DEBUG] (" << __FILENAME__ << ":" << __LINE__ << ") " << msg << std::endl; \
    } while (0)
#else
#define LOG_DEBUG(msg) \
    do                 \
    {                  \
    } while (0)
#endif

#if LOG_LEVEL >= 5
#define LOG_VERBOSE(msg)                                                                         \
    do                                                                                           \
    {                                                                                            \
        std::cerr << "[VERB]  (" << __FILENAME__ << ":" << __LINE__ << ") " << msg << std::endl; \
    } while (0)
#else
#define LOG_VERBOSE(msg) \
    do                   \
    {                    \
    } while (0)
#endif

// Macro to extract filename from __FILE__ for cleaner logs
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#endif // LOGGER_H