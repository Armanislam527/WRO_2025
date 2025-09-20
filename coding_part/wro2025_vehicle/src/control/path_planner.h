// control/path_planner.h
// Makes high-level path planning decisions

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <cstdint>
#include <memory>
// Assuming necessary data structures are defined
#include "../sensors/sensor_data.h"
#include "../navigation/positional_memory.h" // For VisionSnapshot
#include "../navigation/mission_state.h"     // For ChallengeType, MissionPhase

// Forward declarations
class LapCounter;   // Dependency
class SignDetector; // To be implemented

// Structure to hold the high-level navigation decision
struct NavigationCommand
{
    enum class Action
    {
        FOLLOW_LEFT_LANE,
        FOLLOW_RIGHT_LANE,
        GO_STRAIGHT,
        STOP_IN_START,
        FIND_PARKING_ENTRANCE,
        EXECUTE_PARKING,
        EMERGENCY_STOP
    };

    Action action;
    float desiredSpeed; // Normalized or mapped speed command (-1.0 to 1.0, or 0-100%)

    NavigationCommand() : action(Action::GO_STRAIGHT), desiredSpeed(0.0f) {}
    NavigationCommand(Action a, float speed) : action(a), desiredSpeed(speed) {}
};

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner() = default;

    // Initialize with dependencies
    bool initialize(std::shared_ptr<LapCounter> lapCounter /*, std::shared_ptr<SignDetector> signDetector*/);

    // Generate the next navigation command based on current state
    NavigationCommand plan(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState);

private:
    std::shared_ptr<LapCounter> lapCounter;
    // std::shared_ptr<SignDetector> signDetector; // Add when SignDetector is implemented

    // Internal state for decision making
    ::ChallengeType lastKnownChallenge;

    // Helper functions for specific planning logic
    NavigationCommand planForOpenChallenge(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState);
    NavigationCommand planForObstacleChallenge(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState);
    NavigationCommand planForStopping(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState);
    NavigationCommand planForParking(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState);

    // Basic obstacle avoidance logic (fallback or supplement)
    bool needsObstacleAvoidance(const SensorData &sensorData, float &turnDirection); // turnDirection: -1=left, 1=right, 0=none
};

#endif // PATH_PLANNER_H