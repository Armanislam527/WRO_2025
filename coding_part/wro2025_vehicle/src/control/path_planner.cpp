// control/path_planner.cpp
// Implementation of PathPlanner

#include "path_planner.h"
#include <iostream>
#include <algorithm> // For std::max, std::min

PathPlanner::PathPlanner() : lastKnownChallenge(MissionState::ChallengeType::UNKNOWN)
{
    // Constructor
}

bool PathPlanner::initialize(std::shared_ptr<LapCounter> lapCounter /*, std::shared_ptr<SignDetector> signDetector*/)
{
    if (!lapCounter /* || !signDetector */)
    {
        std::cerr << "PathPlanner: Invalid dependencies provided." << std::endl;
        return false;
    }
    this->lapCounter = lapCounter;
    // this->signDetector = signDetector;
    std::cout << "PathPlanner: Initialized with dependencies." << std::endl;
    return true;
}

NavigationCommand PathPlanner::plan(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState)
{

    // Update internal state if challenge type changed
    if (missionState.getChallengeType() != lastKnownChallenge)
    {
        lastKnownChallenge = missionState.getChallengeType();
        std::cout << "PathPlanner: Challenge type changed to "
                  << (lastKnownChallenge == MissionState::ChallengeType::OPEN_CHALLENGE ? "Open" : "Obstacle")
                  << std::endl;
    }

    // Dispatch to specific planning logic based on mission phase and challenge type
    switch (missionState.getPhase())
    {
    case MissionPhase::DRIVING_LAPS:
        if (missionState.isOpenChallenge())
        {
            return planForOpenChallenge(sensorData, visionData, missionState);
        }
        else if (missionState.isObstacleChallenge())
        {
            return planForObstacleChallenge(sensorData, visionData, missionState);
        }
        break;

    case MissionPhase::STOPPING_AT_START:
        return planForStopping(sensorData, visionData, missionState);
        break;

    case MissionPhase::FINDING_PARKING:
    case MissionPhase::PARKING:
        if (missionState.isObstacleChallenge())
        {
            return planForParking(sensorData, visionData, missionState);
        }
        break;

    case MissionPhase::EMERGENCY_STOPPED:
        return NavigationCommand(NavigationCommand::Action::EMERGENCY_STOP, 0.0f);
        break;

    // Other phases (PRE_START, WAITING_FOR_GO, COMPLETED) should not require path planning
    default:
        // Default action if not in a driving phase
        return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, 0.0f);
        break;
    }

    // Default fallback (should ideally not be reached in normal flow)
    std::cerr << "PathPlanner: No specific plan for current state. Defaulting to GO_STRAIGHT." << std::endl;
    return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, 0.5f);
}

// --- Private Helper Functions ---

NavigationCommand PathPlanner::planForOpenChallenge(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState)
{
    // In Open Challenge, there are no signs. Strategy is usually wall following.
    // Common strategies: Left Wall Following, Right Wall Following.
    // For simplicity, let's assume a basic left wall following strategy.

    // Basic obstacle avoidance check
    float turnDirection = 0.0f;
    if (needsObstacleAvoidance(sensorData, turnDirection))
    {
        float speed = 0.4f; // Slow down for obstacle
        if (turnDirection < 0)
        {
            // Turn left
            return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, speed); // Assume steering is handled by lower level
        }
        else if (turnDirection > 0)
        {
            // Turn right
            return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, speed);
        }
        else
        {
            // Stop or back up?
            return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, 0.0f);
        }
    }

    // Default left wall following logic (simplified)
    // Aim to maintain a certain distance from the left wall
    int16_t leftDistance = sensorData.leftDistance;
    int16_t frontDistance = sensorData.frontDistance;

    float desiredSpeed = 0.6f; // Medium speed
    // Action is implicit in speed/steering calculation in this simplified view
    // A more complex planner might explicitly set FOLLOW_LEFT_LANE

    // Simple proportional controller for steering based on left distance
    // Target distance from left wall (mm)
    const float TARGET_LEFT_DISTANCE = 200.0f;
    const float MAX_STEERING_ADJUSTMENT = 20.0f; // Degrees

    float distanceError = static_cast<float>(leftDistance) - TARGET_LEFT_DISTANCE;
    // Clamp error to prevent overreaction
    distanceError = std::max(-300.0f, std::min(300.0f, distanceError));

    // This logic would typically result in a servo angle command in the VehicleController
    // For example: base_angle + Kp * error
    // If error is positive (too far from wall), turn left (decrease servo angle)
    // If error is negative (too close to wall), turn right (increase servo angle)

    // For now, we just return a command indicating intent and speed
    // The actual steering calculation happens in VehicleController/Actuators
    return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, desiredSpeed);
}

NavigationCommand PathPlanner::planForObstacleChallenge(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState)
{
    // In Obstacle Challenge, follow traffic signs.
    // Green sign means follow left lane.
    // Red sign means follow right lane.

    // Basic obstacle avoidance check (takes precedence)
    float turnDirection = 0.0f;
    if (needsObstacleAvoidance(sensorData, turnDirection))
    {
        float speed = 0.4f;
        return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, speed);
    }

    // Determine lane to follow based on sign detection
    // This assumes VisionSnapshot has been processed to detect signs
    bool followLeft = false;
    bool followRight = false;

    // Simplified logic: if both signs detected, prioritize based on last known rule or proximity
    // If only one sign, follow that rule.
    // If no signs, default to a safe strategy (e.g., center, or last known rule)

    if (visionData.signLeftDetected && visionData.signRightDetected)
    {
        // Conflict resolution needed. Simple approach: follow the closer one.
        // Assuming centroid X coordinates are available in VisionSnapshot
        // (This requires updating VisionSnapshot struct and image_processor)
        // For now, let's assume a simple flag or last known state is used.
        // Placeholder logic:
        followLeft = true; // Default to left if conflict, or use more sophisticated logic
        // std::cout << "PathPlanner: Conflict - Both signs detected. Following default (Left)." << std::endl;
    }
    else if (visionData.signLeftDetected)
    {
        followLeft = true;
        // std::cout << "PathPlanner: Green sign detected. Following left lane." << std::endl;
    }
    else if (visionData.signRightDetected)
    {
        followRight = true;
        // std::cout << "PathPlanner: Red sign detected. Following right lane." << std::endl;
    }
    else
    {
        // No signs detected. Continue with last known strategy or default.
        // This is a common scenario between signs.
        // std::cout << "PathPlanner: No signs detected. Continuing last strategy or default." << std::endl;
        // For now, we'll assume a default action like GO_STRAIGHT
        // A more robust system might remember the last sign or use path prediction.
    }

    float desiredSpeed = 0.6f; // Medium speed

    if (followLeft)
    {
        return NavigationCommand(NavigationCommand::Action::FOLLOW_LEFT_LANE, desiredSpeed);
    }
    else if (followRight)
    {
        return NavigationCommand(NavigationCommand::Action::FOLLOW_RIGHT_LANE, desiredSpeed);
    }
    else
    {
        // No specific lane command, go straight
        return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, desiredSpeed);
    }
}

NavigationCommand PathPlanner::planForStopping(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState)
{
    // This phase is for stopping in the start section after 3 laps.
    // The main logic for *when* to stop should be in main.cpp using PositionalMemory.
    // The PathPlanner's role here might be to refine the approach or execute the final stop.

    // Indicate to the control system that stopping is the goal.
    // The actual stopping maneuver (speed=0, precise positioning) is handled by VehicleController
    // based on this command and potentially further input from PositionalMemory/Vision.

    // For now, simply command a stop.
    // A more advanced version might include a "slow_approach" action.
    std::cout << "PathPlanner: Planning for STOP in start section." << std::endl;
    return NavigationCommand(NavigationCommand::Action::STOP_IN_START, 0.0f);
}

NavigationCommand PathPlanner::planForParking(const SensorData &sensorData, const VisionSnapshot &visionData, const MissionState &missionState)
{
    // Handle parking phases
    if (missionState.getPhase() == MissionPhase::FINDING_PARKING)
    {
        // Logic to search for the parking lot entrance.
        // This could involve:
        // - Looking for a specific gap in the outer wall (using vision/US)
        // - Following the outer wall until the gap is found
        // - Entering a state where the vehicle is aligned with the gap

        // Simplified placeholder: Assume we are looking for a large opening on the right
        // indicated by a sudden increase in right sensor distance.
        // A real implementation would be more robust.
        if (sensorData.rightDistance > 400)
        { // Example threshold for gap
            std::cout << "PathPlanner: Parking entrance potentially found (large right distance)." << std::endl;
            // This would likely trigger a state change in MissionState to PARKING
            // and a specific parking maneuver command.
            // For now, we can signal readiness.
            return NavigationCommand(NavigationCommand::Action::EXECUTE_PARKING, 0.3f); // Slow approach speed
        }
        else
        {
            // Continue searching, e.g., follow outer wall
            return NavigationCommand(NavigationCommand::Action::FOLLOW_RIGHT_LANE, 0.5f);
        }
    }
    else if (missionState.getPhase() == MissionPhase::PARKING)
    {
        // Execute the specific parallel parking maneuver.
        // This is a complex sequence of movements.
        // The command here signals that the complex maneuver should be executed.
        // The actual sequence of motor/servo commands would be handled by a dedicated
        // ParkingAssistant module or state machine within VehicleController.
        std::cout << "PathPlanner: Executing PARKING maneuver." << std::endl;
        return NavigationCommand(NavigationCommand::Action::EXECUTE_PARKING, 0.0f); // Speed controlled internally by parking sequence
    }

    // Default fallback for parking phases
    return NavigationCommand(NavigationCommand::Action::GO_STRAIGHT, 0.0f);
}

bool PathPlanner::needsObstacleAvoidance(const SensorData &sensorData, float &turnDirection)
{
    turnDirection = 0.0f; // Default: no turn needed

    // Check front sensor for immediate obstacles
    if (sensorData.frontDistance < 150)
    { // e.g., 15cm
        // Obstacle ahead, need to decide turn direction
        // Use left/right sensors to decide
        if (sensorData.leftDistance > sensorData.rightDistance)
        {
            turnDirection = -1.0f; // Turn left
        }
        else
        {
            turnDirection = 1.0f; // Turn right
        }
        return true;
    }

    // Optional: Check side sensors for very close walls that might require correction
    // This is more for fine-tuning path than major avoidance.
    // Example:
    // if (sensorData.leftDistance < 50) { turnDirection = 1.0f; return true; }
    // if (sensorData.rightDistance < 50) { turnDirection = -1.0f; return true; }

    return false; // No immediate obstacle avoidance needed
}