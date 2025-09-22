// config/vehicle_config.cpp
// Definitions for vehicle configuration constants

#include "vehicle_config.h"

namespace config
{

    // --- Serial Communication ---
    const std::string DEFAULT_NANO_SERIAL_PORT = "/dev/serial0"; // Common default, change as needed
    const uint32_t DEFAULT_NANO_BAUD_RATE = 115200;

    // --- Camera ---
    // const std::string DEFAULT_CAMERA_DEVICE = "/dev/video0";
    const std::string DEFAULT_CAMERA_DEVICE = ""; // Let rpicam-vid find it, or use "/base/soc/i2c0mux/i2c@1/ov5647@36"
    const int DEFAULT_CAMERA_WIDTH = 640;
    const int DEFAULT_CAMERA_HEIGHT = 480;
    const int DEFAULT_CAMERA_FPS = 20; // Adjust based on processing capability

    // --- Vehicle Physical Parameters ---
    const float VEHICLE_WIDTH_MM = 200.0f;  // From rules
    const float VEHICLE_LENGTH_MM = 300.0f; // From rules
    const float WHEELBASE_MM = 200.0f;      // Example, adjust based on actual build

    // --- Mission Parameters ---
    const float POSITIONAL_SIMILARITY_THRESHOLD_INITIAL = 0.85f;  // 85% similarity to consider "at initial position"
    const float POSITIONAL_SIMILARITY_THRESHOLD_PRE_STOP = 0.95f; // 95% similarity for pre-stop position
    const float MIN_DISTANCE_TO_COUNT_LAP_MM = 200.0f;            // Example threshold

} // namespace config