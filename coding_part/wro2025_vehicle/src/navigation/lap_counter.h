// navigation/lap_counter.h
// Counts the number of laps completed by the vehicle

#ifndef LAP_COUNTER_H
#define LAP_COUNTER_H

#include <atomic>
#include <memory>
// Assuming SensorData and VisionSnapshot are defined in these headers
#include "../sensors/sensor_data.h"
#include "positional_memory.h" // For VisionSnapshot

// Forward declarations for dependencies if needed
class CourseMap; // To be implemented later

class LapCounter
{
public:
    LapCounter();
    ~LapCounter() = default;

    // Initialize the lap counter, potentially with a reference to CourseMap
    bool initialize(); // Add CourseMap* if needed later

    // Reset the counter to zero laps and clear state
    void reset();

    // Update the lap counter based on new sensor/vision data
    // This should be called regularly in the main loop
    void update(const SensorData &sensorData, const VisionSnapshot &visionData);

    // Get the current lap number (0-based: 0, 1, 2)
    int getCurrentLap() const;

    // Check if a lap has just been completed
    bool isLapCompleted() const;

    // Acknowledge that the lap completion has been processed
    void acknowledgeLapCompletion();

    // Check if the vehicle is currently in the start/finish section
    // This is crucial for the final stopping logic
    bool isInStartSection() const;

    // Setter for start section state (might be set by other logic like PositionalMemory comparison)
    void setInStartSection(bool inStart);

private:
    std::atomic<int> currentLap;
    std::atomic<bool> lapCompletedFlag; // Flag to indicate a new lap was completed
    std::atomic<bool> inStartSectionFlag;

    // State for lap counting logic
    // This is a simplified placeholder. Real logic might involve:
    // - Tracking passage through specific sections (e.g., counting 8 sections for a lap)
    // - Using positional memory to identify the start section
    // - Integrating with CourseMap

    // Example simple state variables (needs refinement):
    int sectionsPassedInCurrentLap; // Counter for sections passed
    bool hasLeftStartSection;       // Flag to ensure we leave start before counting a lap

    // Helper functions for lap counting logic
    void checkForSectionTransition(const SensorData &sensorData, const VisionSnapshot &visionData);
    void checkForLapCompletion();
};

#endif // LAP_COUNTER_H