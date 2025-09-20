// navigation/lap_counter.cpp
// Implementation of LapCounter

#include "lap_counter.h"
#include <iostream>
#include <cmath> // For fabs, etc. if needed

LapCounter::LapCounter()
    : currentLap(0), lapCompletedFlag(false), inStartSectionFlag(false),
      sectionsPassedInCurrentLap(0), hasLeftStartSection(false)
{
    // Constructor initializes atomic flags and member variables
}

bool LapCounter::initialize()
{
    // Currently no complex initialization needed
    // Could initialize pointers to CourseMap or load parameters later
    reset(); // Ensure a clean state on initialization
    return true;
}

void LapCounter::reset()
{
    currentLap.store(0);
    lapCompletedFlag.store(false);
    inStartSectionFlag.store(false);
    sectionsPassedInCurrentLap = 0;
    hasLeftStartSection = false;
    // std::cout << "LapCounter: Reset to initial state." << std::endl;
}

void LapCounter::update(const SensorData &sensorData, const VisionSnapshot &visionData)
{
    // 1. Determine if we are in the start section
    // This is a placeholder. Real logic would compare sensor/vision data
    // against a known start section profile or use PositionalMemory.
    // For now, we'll assume another part of the system (e.g., main loop logic
    // using PositionalMemory) calls setInStartSection().
    // bool currentlyInStart = determineIfInStartSection(sensorData, visionData);
    // setInStartSection(currentlyInStart); // This will update the atomic flag

    // 2. Check for transitions between sections (simplified logic)
    checkForSectionTransition(sensorData, visionData);

    // 3. Check if a lap has been completed based on sections passed and start section logic
    checkForLapCompletion();
}

int LapCounter::getCurrentLap() const
{
    return currentLap.load();
}

bool LapCounter::isLapCompleted() const
{
    return lapCompletedFlag.load();
}

void LapCounter::acknowledgeLapCompletion()
{
    lapCompletedFlag.store(false);
}

bool LapCounter::isInStartSection() const
{
    return inStartSectionFlag.load();
}

void LapCounter::setInStartSection(bool inStart)
{
    inStartSectionFlag.store(inStart);
    // std::cout << "LapCounter: In Start Section set to " << (inStart ? "true" : "false") << std::endl;
}

// --- Private Helper Functions ---

// Placeholder for section transition logic
// A real implementation might:
// - Use US sensors to detect distance to walls changing significantly
// - Use vision to detect track features or the absence of signs
// - Integrate with a CourseMap to know expected section boundaries
void LapCounter::checkForSectionTransition(const SensorData &sensorData, const VisionSnapshot &visionData)
{
    // This is a very simplified example logic.
    // It assumes that passing a certain distance or a significant change in US readings
    // indicates entering a new section. This is highly dependent on track layout and vehicle speed.

    // Example: Increment section counter if we've traveled a certain estimated distance
    // This is not robust and needs refinement.
    static float estimatedDistance = 0.0f;
    static SensorData lastSensorData = {}; // Initialize with default values
    static bool firstUpdate = true;

    if (firstUpdate)
    {
        lastSensorData = sensorData;
        firstUpdate = false;
        return;
    }

    // Simple distance estimation using front sensor change (very rough)
    // A better approach would integrate speed over time or use encoders/IMU
    float frontDelta = static_cast<float>(sensorData.frontDistance) - static_cast<float>(lastSensorData.frontDistance);
    // Assume if front distance changed by a large amount, we might have turned a corner
    // This is flawed logic and just for demonstration.
    if (std::abs(frontDelta) > 300.0f)
    { // e.g., 30cm change
        sectionsPassedInCurrentLap++;
        // std::cout << "LapCounter: Estimated section transition. Sections passed: " << sectionsPassedInCurrentLap << std::endl;
        lastSensorData = sensorData; // Update for next comparison

        // Simple heuristic: If we think we left the start section
        if (!hasLeftStartSection && sectionsPassedInCurrentLap >= 1)
        {
            hasLeftStartSection = true;
            // std::cout << "LapCounter: Detected leaving start section." << std::endl;
        }
    }
    // Add more sophisticated logic here based on actual sensor fusion and track knowledge.
}

void LapCounter::checkForLapCompletion()
{
    // A lap is completed after passing 8 sections (as per rules)
    if (sectionsPassedInCurrentLap >= 8)
    {
        // Additional check: We must have left the start section and then returned
        // This check is simplified. Real logic is more complex.
        if (hasLeftStartSection /* && isInStartSection() */)
        { // isInStartSection check depends on external call to setInStartSection
            int newLap = currentLap.load() + 1;
            if (newLap <= 3)
            { // Only count up to 3 laps
                currentLap.store(newLap);
                lapCompletedFlag.store(true);
                sectionsPassedInCurrentLap = 0; // Reset for next lap
                hasLeftStartSection = false;    // Reset for next lap
                std::cout << "LapCounter: Lap " << newLap << " completed!" << std::endl;
            }
        }
        else
        {
            // We completed 8 sections but never left start, or are not back in start
            // This might indicate an error or a different track configuration logic is needed.
            // For now, we'll just reset the section counter to prevent overflow/confusion
            // unless we are sure we are in a new lap phase.
            // A more robust system would track absolute position or use landmarks.
            // This logic needs careful design based on actual sensor data interpretation.
            // As a fallback, if we get too many sections without a lap, reset.
            if (sectionsPassedInCurrentLap > 12)
            { // Arbitrary large number
                std::cerr << "LapCounter: Warning - Section counter seems stuck. Resetting." << std::endl;
                sectionsPassedInCurrentLap = 0;
                hasLeftStartSection = false;
            }
        }
    }
}

// Placeholder for start section detection
// bool LapCounter::determineIfInStartSection(const SensorData& sensorData, const VisionSnapshot& visionData) {
//     // This would contain the actual logic to compare current state with the known start section state.
//     // It could use PositionalMemory, CourseMap, or direct sensor/vision analysis.
//     // For example:
//     // return positionalMemory->isLikelyAtInitialPosition(sensorData, visionData, 0.9f);
//     return false; // Placeholder
// }