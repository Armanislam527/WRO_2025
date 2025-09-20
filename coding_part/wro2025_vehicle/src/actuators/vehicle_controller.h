// actuators/vehicle_controller.h
// Translates high-level navigation commands into low-level actuator commands

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <memory>
#include <atomic>
// Assuming NavigationCommand is defined in path_planner.h
#include "../control/path_planner.h"
// Assuming SerialHandler is defined in communication/serial_handler.h
#include "../communication/serial_handler.h"

class VehicleController
{
public:
    VehicleController();
    ~VehicleController() = default;

    // Initialize with the SerialHandler for sending commands
    bool initialize(std::shared_ptr<SerialHandler> serialHandler);

    // Execute a navigation command
    // This should be called regularly in the main loop with the latest command from PathPlanner
    void executeCommand(const NavigationCommand &command);

    // Start the parallel parking sequence
    // This will take over control until parking is complete or fails
    void startParkingSequence();

    // Check if a complex maneuver (like parking) is currently active
    bool isManeuverActive() const;

    // Emergency stop - immediately sends stop command
    void emergencyStop();

private:
    std::shared_ptr<SerialHandler> serialHandler;
    std::atomic<bool> parkingActive;
    std::atomic<bool> maneuverInProgress; // General flag for any complex maneuver

    // State for parking sequence
    enum class ParkingState
    {
        NOT_PARKING,
        APPROACHING_SPOT,   // Initial approach to align with parking spot
        REVERSING_IN,       // First reverse maneuver
        TURNING_IN,         // Turn wheels to align inside
        REVERSING_IN_FINAL, // Final reverse to position fully
        TURNING_STRAIGHT,   // Straighten wheels
        PARKING_COMPLETE,
        PARKING_FAILED
    };
    std::atomic<ParkingState> parkingState;
    int parkingStepCounter; // Simple step counter for timing/state progression

    // Helper functions for specific actions
    void sendMotorCommand(float normalizedSpeed);    // -1.0 to 1.0
    void sendServoCommand(float normalizedSteering); // -1.0 (full left) to 1.0 (full right), 0.0 (center)

    // Parking sequence logic
    void updateParkingSequence();

    // Map high-level actions to low-level commands
    void handleFollowLeftLane(float desiredSpeed);
    void handleFollowRightLane(float desiredSpeed);
    void handleGoStraight(float desiredSpeed);
    void handleStopInStart();
    void handleExecuteParking(); // Triggers parking sequence
    void handleEmergencyStop();
};

#endif // VEHICLE_CONTROLLER_H