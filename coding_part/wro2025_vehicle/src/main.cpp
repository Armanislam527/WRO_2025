// main.cpp
// Entry point for the WRO 2025 Pi-side application

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
#include "navigation/mission_state.h" // Include for MissionPhase enum
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
void shutdownModules();
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
extern std::atomic<bool> g_shutdownRequested;
void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        std::cout << "\nSignal (" << signal << ") received. Requesting graceful shutdown..." << std::endl;
        g_shutdownRequested.store(true); // Set the atomic flag
    }
    else
    {
        std::cout << "\nUnhandled signal (" << signal << ") received. Requesting shutdown..." << std::endl;
        g_shutdownRequested.store(true);
    }
}
int main(int argc, char *argv[])
{
    (void)argc; // Explicitly mark as unused
    (void)argv;
    std::cout << "WRO 2025 Future Engineers - Raspberry Pi Controller" << std::endl;
    std::cout << "Initializing system..." << std::endl;
    if (signal(SIGINT, signalHandler) == SIG_ERR || signal(SIGTERM, signalHandler) == SIG_ERR)
    {
        std::cerr << "Warning: Failed to register signal handlers." << std::endl;
    }

    // bool initSuccess = false;

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
                std::cout << "ImageProcessor initialized and processing..." << std::endl;
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
void shutdownModules()
{
    std::cout << "Shutting down modules..." << std::endl;

    // --- STOP MODULES IN REVERSE ORDER OF DEPENDENCY ---
    // Stop high-level processing first.

    // 1. Stop Image Processing
    if (g_imageProcessor)
    {
        std::cout << "Stopping ImageProcessor..." << std::endl;
        g_imageProcessor->stopProcessing(); // Signal processing thread to stop
        // stopProcessing should handle thread joining internally
        // g_imageProcessor.reset(); // Optional: Explicitly reset if needed sooner
    }

    // 2. Stop Camera Capture
    if (g_cameraInterface)
    {
        std::cout << "Stopping CameraInterface..." << std::endl;
        g_cameraInterface->stopCapture(); // Signal capture thread to stop
        // stopCapture should handle thread joining and rpicam-vid process termination
        // g_cameraInterface.reset(); // Optional: Explicitly reset if needed sooner
    }

    // 3. Stop Vehicle Controller (ensure motors are off)
    if (g_vehicleController)
    {
        std::cout << "Stopping VehicleController..." << std::endl;
        g_vehicleController->emergencyStop(); // Ensure vehicle is stopped
        // g_vehicleController.reset(); // Optional reset
    }

    // 4. Close Serial Port
    if (g_serialHandler)
    {
        std::cout << "Closing SerialHandler..." << std::endl;
        g_serialHandler->closePort(); // Close the serial connection
        // g_serialHandler.reset(); // Optional reset
    }

    // Reset shared_ptrs to allow modules to clean up
    // This should happen naturally as they go out of scope, but explicit reset
    // can be useful if specific destruction order matters or to release resources immediately.
    g_pathPlanner.reset();
    g_lapCounter.reset();
    g_positionalMemory.reset();
    g_missionState.reset();
    g_imageProcessor.reset();
    g_cameraInterface.reset();
    g_vehicleController.reset();
    g_serialHandler.reset();

    std::cout << "Module shutdown complete." << std::endl;
}
void runMainLoop()
{
    // This is the central orchestrator of the vehicle's actions.

    // 3. Update Mission State based on serial events (like START_ACK)
    // MOVED: Declaration and retrieval of sensor/vision data to specific handler functions.

    if (g_serialHandler->hasStartAck())
    {
        // --- FIX: Use MissionPhase::PRE_START ---
        if (g_missionState->getPhase() == MissionPhase::PRE_START)
        {
            // --- FIX: Use MissionPhase::WAITING_FOR_GO ---
            g_missionState->setPhase(MissionPhase::WAITING_FOR_GO);
            std::cout << "[MainLoop] Nano button press acknowledged. Waiting for GO command." << std::endl;
            g_serialHandler->acknowledgeStartAck();
        }
    }

    // 4. Execute logic based on the current mission phase
    switch (g_missionState->getPhase())
    {
    // --- FIX: Use MissionPhase:: for all cases ---
    case MissionPhase::PRE_START:
        handlePreStart();
        break;
    case MissionPhase::WAITING_FOR_GO:
        handleWaitingForGo();
        break;
    case MissionPhase::DRIVING_LAPS:
        handleDriving(); // Data retrieval happens inside handleDriving
        break;
    case MissionPhase::STOPPING_AT_START:
        handleStoppingAtStart(); // Data retrieval happens inside handleStoppingAtStart
        break;
    case MissionPhase::FINDING_PARKING:
    case MissionPhase::PARKING:
        handleParking(); // Data retrieval happens inside handleParking
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
    // --- FIX: Use MissionPhase::DRIVING_LAPS ---
    g_missionState->setPhase(MissionPhase::DRIVING_LAPS);
    g_missionState->setCurrentLap(0);

    std::cout << "[WaitingForGo] GO signal sent. Starting mission: "
              << (g_missionState->isOpenChallenge() ? "Open Challenge" : "Obstacle Challenge") << std::endl;
}

void handleDriving()
{
    static bool initialDataStored = false;

    // --- MOVE THE DECLARATIONS AND RETRIEVAL HERE ---
    // 1. Get latest sensor data from Nano
    SensorData latestSensorData;
    bool newSensorData = g_serialHandler->getLatestSensorData(latestSensorData);

    // 2. Get latest vision snapshot from ImageProcessor
    VisionSnapshot latestVisionData;
    bool newVisionData = g_imageProcessor->getLatestSnapshot(latestVisionData);

    // --- USE THE VARIABLES HERE ---
    if (newSensorData && newVisionData)
    {
        // --- 1. Update Lap Counter ---
        g_lapCounter->update(latestSensorData, latestVisionData);

        // --- 2. Determine if in Start Section ---
        g_missionState->setInStartSection(g_lapCounter->isInStartSection());

        // --- 3. Store Initial Position (Once, after stable start) ---
        if (!initialDataStored && g_missionState->getCurrentLap() >= 0)
        {
            g_positionalMemory->storeInitialPosition(latestSensorData, latestVisionData);
            g_positionalMemory->storePreStopPosition(latestSensorData, latestVisionData);
            initialDataStored = true;
            std::cout << "[Driving] Initial positional data stored." << std::endl;
        }

        // --- 4. Check for Lap Completion and Transition ---
        if (g_lapCounter->isLapCompleted())
        {
            int lapJustCompleted = g_lapCounter->getCurrentLap();
            std::cout << "[Driving] Lap " << lapJustCompleted << " completed." << std::endl;
            g_lapCounter->acknowledgeLapCompletion();

            if (lapJustCompleted >= 3)
            {
                if (g_missionState->isOpenChallenge())
                {
                    // --- FIX: Use MissionPhase::STOPPING_AT_START ---
                    g_missionState->setPhase(MissionPhase::STOPPING_AT_START);
                    std::cout << "[Driving] All laps completed (Open). Transitioning to STOPPING_AT_START." << std::endl;
                }
                else if (g_missionState->isObstacleChallenge())
                {
                    // --- FIX: Use MissionPhase::FINDING_PARKING ---
                    g_missionState->setPhase(MissionPhase::FINDING_PARKING);
                    std::cout << "[Driving] All laps completed (Obstacle). Transitioning to FINDING_PARKING." << std::endl;
                }
            }
        }

        // --- 5. Plan Path and Execute ---
        NavigationCommand navCommand = g_pathPlanner->plan(latestSensorData, latestVisionData, *g_missionState);
        g_vehicleController->executeCommand(navCommand);
    }
    // --- END OF SCOPE FOR VARIABLES ---
}

void handleStoppingAtStart()
{
    // This phase is for stopping in the start section after 3 laps in Open Challenge
    // or after parking in Obstacle Challenge.

    static bool stopCommandSent = false;
    static bool finalStopConfirmed = false;

    // --- MOVE THE DECLARATIONS AND RETRIEVAL HERE ---
    // Get latest data
    SensorData latestSensorData;
    bool newSensorData = g_serialHandler->getLatestSensorData(latestSensorData);

    VisionSnapshot latestVisionData;
    bool newVisionData = g_imageProcessor->getLatestSnapshot(latestVisionData);

    // --- USE THE VARIABLES HERE ---
    if (newSensorData && newVisionData)
    {
        // --- 1. Check if we are back at the initial position ---
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
            NavigationCommand finalStopCmd(NavigationCommand::Action::STOP_IN_START, 0.0f);
            g_vehicleController->executeCommand(finalStopCmd);
            finalStopConfirmed = true;
            // Small delay to ensure vehicle comes to a complete halt before declaring completion
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // --- FIX: Use MissionPhase::COMPLETED ---
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
    // --- END OF SCOPE FOR VARIABLES ---
}

void handleParking()
{
    // In Obstacle Challenge, after 3 laps, find and execute parking.

    static bool parkingInitiated = false;

    // --- MOVE THE DECLARATIONS AND RETRIEVAL HERE ---
    // Get latest data
    SensorData latestSensorData;
    bool newSensorData = g_serialHandler->getLatestSensorData(latestSensorData);

    VisionSnapshot latestVisionData;
    bool newVisionData = g_imageProcessor->getLatestSnapshot(latestVisionData);

    // --- USE THE VARIABLES HERE ---
    if (newSensorData && newVisionData)
    {
        // --- FIX: Use MissionPhase::FINDING_PARKING ---
        if (!parkingInitiated && g_missionState->getPhase() == MissionPhase::FINDING_PARKING)
        {
            if (g_missionState->isInStartSection())
            {
                std::cout << "[Parking] In start section after 3 laps. Initiating parking sequence." << std::endl;
                NavigationCommand parkCmd(NavigationCommand::Action::EXECUTE_PARKING, 0.0f);
                g_vehicleController->executeCommand(parkCmd);
                parkingInitiated = true;
                // --- FIX: Use MissionPhase::PARKING ---
                g_missionState->setPhase(MissionPhase::PARKING);
            }
        }

        // --- 2. Monitor parking progress ---
        if (parkingInitiated && !g_vehicleController->isManeuverActive())
        {
            std::cout << "[Parking] Parking sequence finished." << std::endl;
            // --- FIX: Use MissionPhase::COMPLETED ---
            g_missionState->setPhase(MissionPhase::COMPLETED);
            std::cout << "[Parking] Vehicle parked. Mission COMPLETED." << std::endl;
        }

        // --- 3. While finding parking or parking is active, let PathPlanner guide ---
        if (!parkingInitiated || g_vehicleController->isManeuverActive())
        {
            // --- FIX: Use MissionPhase::FINDING_PARKING ---
            if (g_missionState->getPhase() == MissionPhase::FINDING_PARKING)
            {
                NavigationCommand navCommand = g_pathPlanner->plan(latestSensorData, latestVisionData, *g_missionState);
                if (!g_vehicleController->isManeuverActive())
                {
                    g_vehicleController->executeCommand(navCommand);
                }
            }
        }
    }
    // --- END OF SCOPE FOR VARIABLES ---
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