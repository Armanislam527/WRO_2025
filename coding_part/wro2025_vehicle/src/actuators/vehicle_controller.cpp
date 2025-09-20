// actuators/vehicle_controller.cpp
// Implementation of VehicleController

#include "vehicle_controller.h"
#include <iostream>
#include <cmath>  // For round, clamp functions if needed
#include <thread> // For potential small delays if needed
#include <chrono> // For potential small delays if needed

VehicleController::VehicleController()
    : parkingActive(false), maneuverInProgress(false), parkingState(ParkingState::NOT_PARKING), parkingStepCounter(0)
{
    // Constructor initializes atomic flags and member variables
}

bool VehicleController::initialize(std::shared_ptr<SerialHandler> serialHandler)
{
    if (!serialHandler)
    {
        std::cerr << "VehicleController: Invalid SerialHandler provided." << std::endl;
        return false;
    }
    this->serialHandler = serialHandler;
    std::cout << "VehicleController: Initialized with SerialHandler." << std::endl;
    return true;
}

void VehicleController::executeCommand(const NavigationCommand &command)
{
    // If a complex maneuver like parking is active, prioritize its internal logic
    // unless it's an emergency stop.
    if (isManeuverActive() && command.action != NavigationCommand::Action::EMERGENCY_STOP)
    {
        if (parkingActive)
        {
            updateParkingSequence(); // Continue parking logic
        }
        // Ignore other commands while a maneuver is active
        return;
    }

    // Handle the command based on its action
    switch (command.action)
    {
    case NavigationCommand::Action::FOLLOW_LEFT_LANE:
        handleFollowLeftLane(command.desiredSpeed);
        break;
    case NavigationCommand::Action::FOLLOW_RIGHT_LANE:
        handleFollowRightLane(command.desiredSpeed);
        break;
    case NavigationCommand::Action::GO_STRAIGHT:
        handleGoStraight(command.desiredSpeed);
        break;
    case NavigationCommand::Action::STOP_IN_START:
        handleStopInStart();
        break;
    case NavigationCommand::Action::EXECUTE_PARKING:
        handleExecuteParking(); // This starts the parking sequence
        break;
    case NavigationCommand::Action::EMERGENCY_STOP:
        handleEmergencyStop();
        break;
    default:
        // Unknown action, treat as stop or go straight with zero speed?
        std::cerr << "VehicleController: Unknown NavigationCommand action." << std::endl;
        sendMotorCommand(0.0f);
        sendServoCommand(0.0f); // Center steering
        break;
    }
}

void VehicleController::startParkingSequence()
{
    if (isManeuverActive())
    {
        std::cerr << "VehicleController: Cannot start parking, another maneuver is active." << std::endl;
        return;
    }
    parkingActive.store(true);
    maneuverInProgress.store(true);
    parkingState.store(ParkingState::APPROACHING_SPOT);
    parkingStepCounter = 0;
    std::cout << "VehicleController: Starting parking sequence." << std::endl;
}

void VehicleController::updateParkingSequence()
{
    if (!parkingActive)
        return;

    // Simple state machine for parking
    // In a real implementation, this would be much more sophisticated,
    // using encoders, IMU, or precise timing/distance estimates.
    // This is a basic placeholder sequence.

    // Increment step counter (this would ideally be based on time or distance)
    parkingStepCounter++;

    switch (parkingState.load())
    {
    case ParkingState::APPROACHING_SPOT:
        // 1. Approach the spot slowly, aligning the rear axle with the spot start
        sendMotorCommand(0.3f); // Slow forward
        sendServoCommand(0.0f); // Straight
        if (parkingStepCounter > 50)
        { // Placeholder condition
            parkingState.store(ParkingState::REVERSING_IN);
            parkingStepCounter = 0;
            std::cout << "VehicleController: Parking - Reversing into spot." << std::endl;
        }
        break;

    case ParkingState::REVERSING_IN:
        // 2. Start reversing, turn wheels hard right (or left, depending on side)
        sendMotorCommand(-0.4f); // Slow reverse
        sendServoCommand(1.0f);  // Hard right turn (assuming 1.0 is right)
        if (parkingStepCounter > 100)
        { // Placeholder condition
            parkingState.store(ParkingState::TURNING_IN);
            parkingStepCounter = 0;
            std::cout << "VehicleController: Parking - Turning wheels in." << std::endl;
        }
        break;

    case ParkingState::TURNING_IN:
        // 3. Turn wheels hard left (or right) to align inside the spot
        sendMotorCommand(-0.3f); // Continue slow reverse
        sendServoCommand(-1.0f); // Hard left turn
        if (parkingStepCounter > 50)
        { // Placeholder condition
            parkingState.store(ParkingState::REVERSING_IN_FINAL);
            parkingStepCounter = 0;
            std::cout << "VehicleController: Parking - Final reverse." << std::endl;
        }
        break;

    case ParkingState::REVERSING_IN_FINAL:
        // 4. Continue reversing to pull fully into the spot
        sendMotorCommand(-0.3f); // Slow reverse
        sendServoCommand(-1.0f); // Keep left turn
        if (parkingStepCounter > 80)
        { // Placeholder condition
            parkingState.store(ParkingState::TURNING_STRAIGHT);
            parkingStepCounter = 0;
            std::cout << "VehicleController: Parking - Straightening wheels." << std::endl;
        }
        break;

    case ParkingState::TURNING_STRAIGHT:
        // 5. Straighten the wheels
        sendMotorCommand(0.0f); // Stop
        sendServoCommand(0.0f); // Center steering
        // Small delay to let steering settle?
        if (parkingStepCounter > 20)
        { // Placeholder condition
            parkingState.store(ParkingState::PARKING_COMPLETE);
            parkingStepCounter = 0;
            std::cout << "VehicleController: Parking - Sequence complete." << std::endl;
        }
        break;

    case ParkingState::PARKING_COMPLETE:
        // Parking finished successfully
        parkingActive.store(false);
        maneuverInProgress.store(false);
        parkingStepCounter = 0;
        // Ensure stopped
        sendMotorCommand(0.0f);
        sendServoCommand(0.0f);
        std::cout << "VehicleController: Parking completed successfully." << std::endl;
        break;

    case ParkingState::PARKING_FAILED:
        // Parking failed, stop and exit sequence
        parkingActive.store(false);
        maneuverInProgress.store(false);
        parkingStepCounter = 0;
        sendMotorCommand(0.0f);
        sendServoCommand(0.0f);
        std::cout << "VehicleController: Parking failed." << std::endl;
        break;

    default:
        // Should not happen
        parkingActive.store(false);
        maneuverInProgress.store(false);
        sendMotorCommand(0.0f);
        sendServoCommand(0.0f);
        std::cerr << "VehicleController: Unexpected parking state." << std::endl;
        break;
    }
}

bool VehicleController::isManeuverActive() const
{
    return maneuverInProgress.load();
}

void VehicleController::emergencyStop()
{
    sendMotorCommand(0.0f);
    sendServoCommand(0.0f);
    // Cancel any active maneuvers
    parkingActive.store(false);
    maneuverInProgress.store(false);
    parkingState.store(ParkingState::NOT_PARKING);
    parkingStepCounter = 0;
    std::cout << "VehicleController: Emergency stop executed." << std::endl;
}

// --- Private Helper Functions ---

void VehicleController::sendMotorCommand(float normalizedSpeed)
{
    // Clamp speed to valid range
    if (normalizedSpeed > 1.0f)
        normalizedSpeed = 1.0f;
    if (normalizedSpeed < -1.0f)
        normalizedSpeed = -1.0f;

    // Map normalized speed (-1.0 to 1.0) to Nano's motor command range (-128 to 127)
    int8_t nanoSpeed = static_cast<int8_t>(std::round(normalizedSpeed * 127.0f));
    if (serialHandler)
    {
        serialHandler->sendMotorSpeedCommand(nanoSpeed);
    }
}

void VehicleController::sendServoCommand(float normalizedSteering)
{
    // Clamp steering to valid range
    if (normalizedSteering > 1.0f)
        normalizedSteering = 1.0f;
    if (normalizedSteering < -1.0f)
        normalizedSteering = -1.0f;

    // Map normalized steering (-1.0 to 1.0) to servo angle (0 to 180)
    // Assuming -1.0 = 45 degrees (hard left), 0.0 = 90 degrees (center), 1.0 = 135 degrees (hard right)
    // This mapping can be adjusted based on vehicle characteristics
    uint8_t servoAngle = static_cast<uint8_t>(std::round(90.0f + normalizedSteering * 45.0f));
    // Ensure angle is within 0-180
    if (servoAngle > 180)
        servoAngle = 180;
    // if (servoAngle < 0)
    //     servoAngle = 0;

    if (serialHandler)
    {
        serialHandler->sendServoAngleCommand(servoAngle);
    }
}

// --- Action Handlers ---

void VehicleController::handleFollowLeftLane(float desiredSpeed)
{
    // This is a simplified interpretation.
    // A more advanced system might use a PID controller based on sensor/edge data.
    // For now, assume a basic steering command to follow the left.
    sendMotorCommand(desiredSpeed);
    sendServoCommand(-0.3f); // Slight left turn command
}

void VehicleController::handleFollowRightLane(float desiredSpeed)
{
    sendMotorCommand(desiredSpeed);
    sendServoCommand(0.3f); // Slight right turn command
}

void VehicleController::handleGoStraight(float desiredSpeed)
{
    sendMotorCommand(desiredSpeed);
    sendServoCommand(0.0f); // Center steering
}

void VehicleController::handleStopInStart()
{
    // Command to stop the vehicle
    sendMotorCommand(0.0f);
    sendServoCommand(0.0f);
    // The logic for *when* to call this and ensuring it stops in the correct place
    // should primarily reside in main.cpp using PositionalMemory.
    std::cout << "VehicleController: Executing STOP command." << std::endl;
}

void VehicleController::handleExecuteParking()
{
    // This command initiates the parking sequence
    startParkingSequence();
}

void VehicleController::handleEmergencyStop()
{
    emergencyStop();
}