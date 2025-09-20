// config/vehicle_config.h
// Central configuration file for vehicle parameters

#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <string>
#include <cstdint>

namespace config
{

    // --- Serial Communication ---
    // Default serial port for communication with Nano
    // This can be overridden at runtime or via command line arguments
    extern const std::string DEFAULT_NANO_SERIAL_PORT; // e.g., "/dev/ttyUSB0", "/dev/serial0"
    extern const uint32_t DEFAULT_NANO_BAUD_RATE;      // e.g., 115200

    // --- Camera ---
    // Default camera device (V4L2)
    extern const std::string DEFAULT_CAMERA_DEVICE; // e.g., "/dev/video0"
    extern const int DEFAULT_CAMERA_WIDTH;          // e.g., 640
    extern const int DEFAULT_CAMERA_HEIGHT;         // e.g., 480
    extern const int DEFAULT_CAMERA_FPS;            // e.g., 30

    // --- Vehicle Physical Parameters ---
    // These might be used for navigation/path planning calculations
    extern const float VEHICLE_WIDTH_MM;  // mm
    extern const float VEHICLE_LENGTH_MM; // mm
    extern const float WHEELBASE_MM;      // mm (distance between front and rear axles)

    // --- Mission Parameters ---
    // Thresholds for positional memory comparisons
    extern const float POSITIONAL_SIMILARITY_THRESHOLD_INITIAL;  // For stopping at start
    extern const float POSITIONAL_SIMILARITY_THRESHOLD_PRE_STOP; // For fine-tuning stop

    // Lap counting thresholds (to be refined)
    extern const float MIN_DISTANCE_TO_COUNT_LAP_MM; // Minimum distance from a section boundary to count a lap

} // namespace config

#endif // VEHICLE_CONFIG_H