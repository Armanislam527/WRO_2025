// positional_memory.cpp
// Implementation of PositionalMemory

#include "positional_memory.h"
#include <cmath>     // For fabs, sqrt
#include <algorithm> // For max
#include <iostream>  // For debugging

PositionalMemory::PositionalMemory()
    : initialDataStored(false), preStopDataStored(false), parkingEntranceDataStored(false)
{
    // Initialize stored data structs to zero/default
    // In practice, you might initialize SensorData members explicitly
}

void PositionalMemory::storeInitialPosition(const SensorData &sensorSnapshot, const VisionSnapshot &visionSnapshot)
{
    initialSensorData = sensorSnapshot;
    initialVisionSnapshot = visionSnapshot;
    initialDataStored = true;
    // std::cout << "PositionalMemory: Initial position stored." << std::endl;
}

void PositionalMemory::storePreStopPosition(const SensorData &sensorSnapshot, const VisionSnapshot &visionSnapshot)
{
    preStopSensorData = sensorSnapshot;
    preStopVisionSnapshot = visionSnapshot;
    preStopDataStored = true;
    // std::cout << "PositionalMemory: Pre-stop position stored." << std::endl;
}

void PositionalMemory::storeParkingEntrancePosition(const SensorData &sensorSnapshot, const VisionSnapshot &visionSnapshot)
{
    parkingEntranceSensorData = sensorSnapshot;
    parkingEntranceVisionSnapshot = visionSnapshot;
    parkingEntranceDataStored = true;
    // std::cout << "PositionalMemory: Parking entrance position stored." << std::endl;
}

float PositionalMemory::compareWithInitial(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData) const
{
    if (!initialDataStored)
        return 0.0f;

    float sensorSim = calculateSensorSimilarity(initialSensorData, currentSensorData);
    float visionSim = calculateVisionSimilarity(initialVisionSnapshot, currentVisionData);
    // Simple average, could be weighted
    return (sensorSim + visionSim) / 2.0f;
}

float PositionalMemory::compareWithPreStop(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData) const
{
    if (!preStopDataStored)
        return 0.0f;

    float sensorSim = calculateSensorSimilarity(preStopSensorData, currentSensorData);
    float visionSim = calculateVisionSimilarity(preStopVisionSnapshot, currentVisionData);
    // Simple average, could be weighted
    return (sensorSim + visionSim) / 2.0f;
}

bool PositionalMemory::isLikelyAtInitialPosition(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData, float threshold) const
{
    float similarity = compareWithInitial(currentSensorData, currentVisionData);
    return similarity >= threshold;
}

bool PositionalMemory::isLikelyAtPreStopPosition(const SensorData &currentSensorData, const VisionSnapshot &currentVisionData, float threshold) const
{
    float similarity = compareWithPreStop(currentSensorData, currentVisionData);
    return similarity >= threshold;
}

// --- Placeholder Similarity Calculations ---
// These need to be refined based on actual data ranges and importance

float PositionalMemory::calculateSensorSimilarity(const SensorData &data1, const SensorData &data2) const
{
    // Example using US sensors - normalize and calculate inverse distance
    // Assume max distance is 2000mm (2m) for normalization
    const float MAX_DIST = 2000.0f;
    float totalSim = 0.0f;
    int count = 4; // front, right, back, left

    float diff_front = std::abs(static_cast<float>(data1.frontDistance) - static_cast<float>(data2.frontDistance)) / MAX_DIST;
    float diff_right = std::abs(static_cast<float>(data1.rightDistance) - static_cast<float>(data2.rightDistance)) / MAX_DIST;
    float diff_back = std::abs(static_cast<float>(data1.backDistance) - static_cast<float>(data2.backDistance)) / MAX_DIST;
    float diff_left = std::abs(static_cast<float>(data1.leftDistance) - static_cast<float>(data2.leftDistance)) / MAX_DIST;

    // Similarity is 1 - normalized difference (0 difference = 1.0 sim, max difference = 0.0 sim)
    totalSim += (1.0f - std::min(1.0f, diff_front));
    totalSim += (1.0f - std::min(1.0f, diff_right));
    totalSim += (1.0f - std::min(1.0f, diff_back));
    totalSim += (1.0f - std::min(1.0f, diff_left));

    // Simple average
    return totalSim / static_cast<float>(count);
}

float PositionalMemory::calculateVisionSimilarity(const VisionSnapshot &data1, const VisionSnapshot &data2) const
{
    // Example using boolean sign detection and edge positions
    float totalSim = 0.0f;
    int count = 4;

    // Sign detection similarity (1.0 if same, 0.0 if different)
    float signLeftSim = (data1.signLeftDetected == data2.signLeftDetected) ? 1.0f : 0.0f;
    float signRightSim = (data1.signRightDetected == data2.signRightDetected) ? 1.0f : 0.0f;
    totalSim += signLeftSim;
    totalSim += signRightSim;

    // Edge position similarity (assuming image width is 640, so max diff is 640)
    const float MAX_EDGE_DIFF = 640.0f;
    float edgeLeftDiff = std::abs(data1.avgLeftEdgeX - data2.avgLeftEdgeX) / MAX_EDGE_DIFF;
    float edgeRightDiff = std::abs(data1.avgRightEdgeX - data2.avgRightEdgeX) / MAX_EDGE_DIFF;
    totalSim += (1.0f - std::min(1.0f, edgeLeftDiff));
    totalSim += (1.0f - std::min(1.0f, edgeRightDiff));

    return totalSim / static_cast<float>(count);
}