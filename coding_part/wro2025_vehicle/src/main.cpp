// main.cpp (Updated/Refined parts)
// ... (existing includes) ...
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <signal.h>
#include <memory> // For shared_ptr

// Include our module headers
#include "config/vehicle_config.h"
#include "communication/serial_handler.h"
#include "sensors/sensor_data.h"
#include "vision/camera_interface.h"
#include "vision/image_processor.h"
#include "navigation/mission_state.h"
#include "navigation/positional_memory.h"
#include "navigation/lap_counter.h"
#include "control/path_planner.h"
#include "actuators/vehicle_controller.h"

// --- Global State ---
std::atomic<bool> g_shutdownRequested(false);

// --- Forward Declarations ---
void signalHandler(int signal);
void initializeSystem();
bool initializeModules();
void runMainLoop();
void handlePreStart();
void handleWaitingForGo();
void handleDriving();
void handleStoppingAtStart();
void handleParking();
void handleCompleted();
void handleEmergencyStopped();

// --- Module Pointers (Global for simplicity in main.cpp) ---
std::shared_ptr<SerialHandler> g_serialHandler;
std::shared_ptr<CameraInterface> g_cameraInterface;
std::shared_ptr<ImageProcessor> g_imageProcessor;
std::shared_ptr<LapCounter> g_lapCounter;
std::shared_ptr<PathPlanner> g_pathPlanner;
std::shared_ptr<VehicleController> g_vehicleController;
std::shared_ptr<MissionState> g_missionState;
std::shared_ptr<PositionalMemory> g_positionalMemory;

int main(int argc, char *argv[])
{
    std::cout << "WRO 2025 Future Engineers - Raspberry Pi Controller" << std::endl;
    std::cout << "Initializing system..." << std::endl;

    signal(SIGINT, signalHandler);

    try
    {
        initializeSystem();
        if (!initializeModules())
        {
            std::cerr << "Failed to initialize one or more modules. Exiting." << std::endl;
            return 1;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error during initialization: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "System initialized. Entering main loop..." << std::endl;

    while (!g_shutdownRequested)
    {
        runMainLoop();
        // Small sleep to prevent excessive CPU usage in the main thread loop
        // The actual processing frequencies are controlled by the module threads
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "Shutting down..." << std::endl;
    // Modules will clean up in their destructors
    std::cout << "Shutdown complete." << std::endl;
    return 0;
}

void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nCtrl+C received. Requesting shutdown..." << std::endl;
        g_shutdownRequested = true;
    }
}

void initializeSystem()
{
    // TODO: Initialize logging, load config files if needed
    std::cout << "System core initialized." << std::endl;
}

bool initializeModules()
{
    bool success = true;

    // --- 1. Serial Handler ---
    g_serialHandler = std::make_shared<SerialHandler>();
    if (!g_serialHandler->openPort(config::DEFAULT_NANO_SERIAL_PORT, config::DEFAULT_NANO_BAUD_RATE))
    {
        std::cerr << "Failed to open serial port: " << config::DEFAULT_NANO_SERIAL_PORT << std::endl;
        return false;
    }
    std::cout << "SerialHandler initialized." << std::endl;

    // --- 2. Camera Interface ---
    g_cameraInterface = std::make_shared<CameraInterface>();
    if (!g_cameraInterface->initialize(config::DEFAULT_CAMERA_DEVICE, config::DEFAULT_CAMERA_WIDTH, config::DEFAULT_CAMERA_HEIGHT, config::DEFAULT_CAMERA_FPS))
    {
        std::cerr << "Failed to initialize CameraInterface." << std::endl;
        success = false;
    }
    else
    {
        if (!g_cameraInterface->startCapture())
        {
            std::cerr << "Failed to start camera capture." << std::endl;
            success = false;
        }
        else
        {
            std::cout << "CameraInterface initialized and capturing." << std::endl;
        }
    }

    // --- 3. Image Processor ---
    if (success)
    {
        g_imageProcessor = std::make_shared<ImageProcessor>();
        // ImageProcessor depends on the CameraInterface's frame buffer
        auto frameBuffer = g_cameraInterface->getFrameBuffer();
        if (!g_imageProcessor->initialize(frameBuffer))
        {
            std::cerr << "Failed to initialize ImageProcessor." << std::endl;
            success = false;
        }
        else
        {
            if (!g_imageProcessor->startProcessing())
            {
                std::cerr << "Failed to start image processing." << std::endl;
                success = false;
            }
            else
            {
                std::cout << "ImageProcessor initialized and processing." << std::endl;
            }
        }
    }

    // --- 4. Mission State & Positional Memory ---
    g_missionState = std::make_shared<MissionState>();
    g_positionalMemory = std::make_shared<PositionalMemory>();
    std::cout << "MissionState and PositionalMemory initialized." << std::endl;

    // --- 5. Lap Counter ---
    g_lapCounter = std::make_shared<LapCounter>();
    if (!g_lapCounter->initialize())
    {
        std::cerr << "Failed to initialize LapCounter." << std::endl;
        success = false;
    }
    std::cout << "LapCounter initialized." << std::endl;

    // --- 6. Path Planner ---
    g_pathPlanner = std::make_shared<PathPlanner>();
    // PathPlanner depends on LapCounter
    if (!g_pathPlanner->initialize(g_lapCounter))
    {
        std::cerr << "Failed to initialize PathPlanner." << std::endl;
        success = false;
    }
    std::cout << "PathPlanner initialized." << std::endl;

    // --- 7. Vehicle Controller ---
    g_vehicleController = std::make_shared<VehicleController>();
    // VehicleController depends on SerialHandler
    if (!g_vehicleController->initialize(g_serialHandler))
    {
        std::cerr << "Failed to initialize VehicleController." << std::endl;
        success = false;
    }
    std::cout << "VehicleController initialized." << std::endl;

    return success;
}

void runMainLoop()
{
    // This is the central orchestrator of the vehicle's actions.

    // 1. Get latest sensor data from Nano
    SensorData latestSensorData;
    bool newSensorData = g_serialHandler->getLatestSensorData(latestSensorData);

    // 2. Get latest vision snapshot from ImageProcessor
    VisionSnapshot latestVisionData;
    bool newVisionData = g_imageProcessor->getLatestSnapshot(latestVisionData);

    // 3. Update Mission State based on serial events (like START_ACK)
    if (g_serialHandler->hasStartAck())
    {
        if (g_missionState->getPhase() == MissionPhase::PRE_START)
        {
            g_missionState->setPhase(MissionPhase::WAITING_FOR_GO);
            std::cout << "[MainLoop] Nano button press acknowledged. Waiting for GO command." << std::endl;
            g_serialHandler->acknowledgeStartAck();
        }
    }

    // 4. Execute logic based on the current mission phase
    switch (g_missionState->getPhase())
    {
    case MissionPhase::PRE_START:
        handlePreStart();
        break;
    case MissionPhase::WAITING_FOR_GO:
        handleWaitingForGo();
        break;
    case MissionPhase::DRIVING_LAPS:
        handleDriving();
        break;
    case MissionPhase::STOPPING_AT_START:
        handleStoppingAtStart(); // This will use PositionalMemory
        break;
    case MissionPhase::FINDING_PARKING:
    case MissionPhase::PARKING:
        handleParking(); // This will use VehicleController's parking logic
        break;
    case MissionPhase::COMPLETED:
        handleCompleted();
        break;
    case MissionPhase::EMERGENCY_STOPPED:
        handleEmergencyStopped();
        break;
    default:
        std::cerr << "[MainLoop] Unknown mission phase!" << std::endl;
        g_shutdownRequested = true;
        break;
    }

    // 5. Send periodic heartbeat to Nano (unless in a complex maneuver)
    // This is crucial for the watchdog.
    static int heartbeatCounter = 0;
    heartbeatCounter++;
    if (heartbeatCounter >= 20)
    { // Send every ~100ms (20 * 5ms loop delay)
        if (g_serialHandler && g_missionState->isDrivingPhase() && !g_vehicleController->isManeuverActive())
        {
            g_serialHandler->sendHeartbeat();
        }
        heartbeatCounter = 0;
    }
}

// --- Phase Handlers ---

void handlePreStart()
{
    // Passive state, waiting for START_ACK from Nano
    // std::cout << "[PreStart] Waiting for Nano button press..." << std::endl;
}

void handleWaitingForGo()
{
    // Send GO command immediately after button ack (as per simple logic)
    // A more advanced system might wait for sensor/vision stabilization
    std::cout << "[WaitingForGo] Sending GO signal to Nano..." << std::endl;
    g_serialHandler->sendGoCommand();

    // Assume Open Challenge initially, or determine based on config/file
    g_missionState->setChallengeType(ChallengeType::OPEN_CHALLENGE);
    g_missionState->setPhase(MissionPhase::DRIVING_LAPS);
    g_missionState->setCurrentLap(0);

    std::cout << "[WaitingForGo] GO signal sent. Starting mission: "
              << (g_missionState->isOpenChallenge() ? "Open Challenge" : "Obstacle Challenge") << std::endl;
}

void handleDriving()
{
    static bool initialDataStored = false;

    // Get latest data (assumed to be retrieved in runMainLoop scope for efficiency)
    // SensorData latestSensorData, VisionSnapshot latestVisionData are available

    if (newSensorData && newVisionData)
    {
        // --- 1. Update Lap Counter ---
        g_lapCounter->update(latestSensorData, latestVisionData);

        // --- 2. Determine if in Start Section (using LapCounter logic or PositionalMemory) ---
        // For now, rely on LapCounter's internal logic or a simple check
        // A more robust approach would be:
        // bool isInStart = g_positionalMemory->isLikelyAtInitialPosition(latestSensorData, latestVisionData, 0.8f);
        // g_lapCounter->setInStartSection(isInStart);
        // g_missionState->setInStartSection(isInStart);

        // Simpler check for now:
        g_missionState->setInStartSection(g_lapCounter->isInStartSection());

        // --- 3. Store Initial Position (Once, after stable start) ---
        if (!initialDataStored && g_missionState->getCurrentLap() >= 0)
        { // Or wait a few loops
            g_positionalMemory->storeInitialPosition(latestSensorData, latestVisionData);
            g_positionalMemory->storePreStopPosition(latestSensorData, latestVisionData); // Initial guess
            initialDataStored = true;
            std::cout << "[Driving] Initial positional data stored." << std::endl;
        }

        // --- 4. Check for Lap Completion and Transition ---
        if (g_lapCounter->isLapCompleted())
        {
            int lapJustCompleted = g_lapCounter->getCurrentLap(); // Gets the lap number that was just finished
            std::cout << "[Driving] Lap " << lapJustCompleted << " completed." << std::endl;
            g_lapCounter->acknowledgeLapCompletion(); // Reset the flag

            if (lapJustCompleted >= 3)
            {
                // All 3 laps done. Transition based on challenge type.
                if (g_missionState->isOpenChallenge())
                {
                    g_missionState->setPhase(MissionPhase::STOPPING_AT_START);
                    std::cout << "[Driving] All laps completed (Open). Transitioning to STOPPING_AT_START." << std::endl;
                }
                else if (g_missionState->isObstacleChallenge())
                {
                    g_missionState->setPhase(MissionPhase::FINDING_PARKING);
                    std::cout << "[Driving] All laps completed (Obstacle). Transitioning to FINDING_PARKING." << std::endl;
                }
            }
        }

        // --- 5. Plan Path and Execute ---
        NavigationCommand navCommand = g_pathPlanner->plan(latestSensorData, latestVisionData, *g_missionState);
        g_vehicleController->executeCommand(navCommand);
    }
}

void handleStoppingAtStart()
{
    // This phase is for stopping in the start section after 3 laps in Open Challenge
    // or after parking in Obstacle Challenge.

    static bool stopCommandSent = false;
    static bool finalStopConfirmed = false;

    if (newSensorData && newVisionData)
    {
        // --- 1. Check if we are back at the initial position ---
        // Use a high threshold for the final stop to be quite sure.
        bool isAtInitial = g_positionalMemory->isLikelyAtPreStopPosition(latestSensorData, latestVisionData, 0.92f);

        if (!stopCommandSent)
        {
            // --- 2. Send initial stop command ---
            std::cout << "[StoppingAtStart] Initiating stop sequence." << std::endl;
            NavigationCommand stopCmd(NavigationCommand::Action::STOP_IN_START, 0.0f);
            g_vehicleController->executeCommand(stopCmd);
            stopCommandSent = true;
        }

        // --- 3. Fine-tune stopping based on position ---
        if (isAtInitial && !finalStopConfirmed)
        {
            std::cout << "[StoppingAtStart] Confidently at start position. Finalizing stop." << std::endl;
            // Send another stop command to ensure it's solid, or rely on VehicleController state
            NavigationCommand finalStopCmd(NavigationCommand::Action::STOP_IN_START, 0.0f);
            g_vehicleController->executeCommand(finalStopCmd);
            finalStopConfirmed = true;
            // Small delay to ensure vehicle comes to a complete halt before declaring completion
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            g_missionState->setPhase(MissionPhase::COMPLETED);
            std::cout << "[StoppingAtStart] Vehicle stopped in start section. Mission COMPLETED." << std::endl;
        }
        else if (!isAtInitial && stopCommandSent)
        {
            // If we sent stop but are not there, we might have overshot or need adjustment
            // A more advanced system would implement a PID controller here using positional error
            // For now, we rely on the initial stop command and the high threshold for confirmation.
            // std::cout << "[StoppingAtStart] Not yet at start position, waiting for confirmation." << std::endl;
        }
    }
}

void handleParking()
{
    // In Obstacle Challenge, after 3 laps, find and execute parking.
    // The VehicleController handles the complex parking sequence.
    // This handler mainly triggers the start and monitors completion.

    static bool parkingInitiated = false;

    if (newSensorData && newVisionData)
    {
        if (!parkingInitiated && g_missionState->getPhase() == MissionPhase::FINDING_PARKING)
        {
            // --- 1. Logic to find parking lot entrance ---
            // This could involve checking sensor data for a large gap on the outer wall.
            // For simplicity, we'll assume the PathPlanner's EXECUTE_PARKING command
            // signals readiness, or we could trigger it after a certain condition.

            // Let's trigger parking when we are in the start section (where parking lot is)
            // and have completed 3 laps.
            if (g_missionState->isInStartSection())
            {
                std::cout << "[Parking] In start section after 3 laps. Initiating parking sequence." << std::endl;
                NavigationCommand parkCmd(NavigationCommand::Action::EXECUTE_PARKING, 0.0f);
                g_vehicleController->executeCommand(parkCmd); // This starts the internal parking sequence
                parkingInitiated = true;
                // Transition state
                g_missionState->setPhase(MissionPhase::PARKING);
            }
            // Else, PathPlanner in DRIVING_LAPS phase should guide vehicle towards start section
            // where the parking lot is located.
        }

        // --- 2. Monitor parking progress ---
        // The VehicleController's internal state machine handles this.
        // We can check if it's done.
        if (parkingInitiated && !g_vehicleController->isManeuverActive())
        {
            // Parking sequence (successful or failed) has finished.
            // A more robust system would check the final state (PARKING_COMPLETE/FAILED)
            // inside VehicleController. For now, we assume success if it finishes.
            std::cout << "[Parking] Parking sequence finished." << std::endl;
            // After parking, the final goal is to stop (which parking should do)
            // and declare mission complete.
            g_missionState->setPhase(MissionPhase::COMPLETED);
            std::cout << "[Parking] Vehicle parked. Mission COMPLETED." << std::endl;
        }

        // --- 3. While finding parking or parking is active, let PathPlanner guide ---
        if (!parkingInitiated || g_vehicleController->isManeuverActive())
        {
            // If not yet initiated or parking is in progress (but not fully handed over),
            // PathPlanner can still provide guidance (e.g., navigate to start section)
            if (g_missionState->getPhase() == MissionPhase::FINDING_PARKING)
            {
                NavigationCommand navCommand = g_pathPlanner->plan(latestSensorData, latestVisionData, *g_missionState);
                // Be careful not to override the parking command once sent.
                // VehicleController should ignore new commands during parking.
                // For simplicity here, we assume parking command is sent once and takes over.
                // A better approach might be a flag or specific check in VehicleController.
                if (!g_vehicleController->isManeuverActive())
                {
                    g_vehicleController->executeCommand(navCommand);
                }
            }
        }
    }
}

void handleCompleted()
{
    std::cout << "[Completed] Mission successfully finished!" << std::endl;
    g_shutdownRequested = true;
}

void handleEmergencyStopped()
{
    std::cout << "[EmergencyStopped] Vehicle has been emergency stopped." << std::endl;
    // Ensure vehicle is stopped via VehicleController
    if (g_vehicleController)
    {
        g_vehicleController->emergencyStop();
    }
    g_shutdownRequested = true; // Or wait for reset
}