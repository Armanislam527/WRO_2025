// positional_memory.h
// Stores and compares positional data for start/stop and parking logic

#ifndef POSITIONAL_MEMORY_H
#define POSITIONAL_MEMORY_H

#include <vector>
#include <cstdint>
// Assuming we'll get sensor data in a format similar to Nano's CompactSensorData
#include "../sensors/sensor_data.h" // This will define SensorData or similar

// Forward declarations or includes for vision data structures if needed
// For now, let's assume we store ROI descriptors or simple color/edge info
struct VisionSnapshot
{
    // Placeholder for vision data
    // Could be keypoint descriptors, histogram data, simple flags
    // For initial version, maybe just store if signs were seen and where
    bool signLeftDetected;
    bool signRightDetected;
    float avgLeftEdgeX;  // Average X position of left edge in ROI
    float avgRightEdgeX; // Average X position of right edge in ROI
    // Add more fields as vision processing becomes concrete
};

class PositionalMemory
{
public:
    PositionalMemory();

    // Store the initial state when the mission officially starts (after GO signal)
    void storeInitialPosition(const SensorData &sensorSnapshot, const VisionSnapshot &visionSnapshot);

    // Store the position just before attempting to stop in the start section
    // This might be slightly different from initial due to sensor drift or initial movement
    void storePreStopPosition(const SensorData &sensorSnapshot, const VisionSnapshot &visionSnapshot);

    // Store the position when entering the parking lot area (Obstacle challenge)
    void storeParkingEntrancePosition(const SensorData &sensorSnapshot, const VisionSnapshot &visionSnapshot);

    // Compare current state with stored initial state
    float compareWithInitial(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData) const;

    // Compare current state with stored pre-stop state
    float compareWithPreStop(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData) const;

    // Check if we are likely back at the initial position (for stopping)
    bool isLikelyAtInitialPosition(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData, float threshold = 0.8f) const;

    // Check if we are likely at the pre-stop position (for fine-tuning stop)
    bool isLikelyAtPreStopPosition(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData, float threshold = 0.9f) const;

    // Getters for stored data (useful for debugging or navigation)
    bool hasInitialData() const { return initialDataStored; }
    bool hasPreStopData() const { return preStopDataStored; }
    const SensorData &getInitialSensorData() const { return initialSensorData; }
    const VisionSnapshot &getInitialVisionData() const { return initialVisionSnapshot; }

private:
    bool initialDataStored;
    bool preStopDataStored;
    bool parkingEntranceDataStored;

    SensorData initialSensorData;
    VisionSnapshot initialVisionSnapshot;

    SensorData preStopSensorData;
    VisionSnapshot preStopVisionSnapshot;

    SensorData parkingEntranceSensorData;
    VisionSnapshot parkingEntranceVisionSnapshot;

    // Helper function to calculate a simple similarity score between sensor data
    // This is a placeholder - real implementation would be more complex
    float calculateSensorSimilarity(const SensorData &data1, const SensorData &data2) const;

    // Helper function to calculate a simple similarity score between vision data
    float calculateVisionSimilarity(const VisionSnapshot &data1, const VisionSnapshot &data2) const;
};

#endif // POSITIONAL_MEMORY_H