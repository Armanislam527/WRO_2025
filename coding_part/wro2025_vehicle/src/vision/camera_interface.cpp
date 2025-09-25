// src/vision/camera_interface.cpp
// Implementation of CameraInterface using rpicam-vid subprocess and OpenCV

#include "camera_interface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>           // For building command string
#include <unistd.h>          // For ::sleep (if needed for startup delay)
#include <cstdio>            // For perror
#include <memory>            // For make_shared
#include "../utils/logger.h" // Include the logger

// --- CONSTRUCTOR ---
CameraInterface::CameraInterface()
    : devicePath(""), frameWidth(640), frameHeight(480), targetFPS(20),
      cameraProcess(nullptr), cameraCommand(""),
      isInitialized(false), isCapturing(false),
      stopCaptureFlag(false), previous_byte(0)
{
    frameBuffer = std::make_shared<FrameBuffer>();
    LOG_DEBUG("CameraInterface: Constructed.");
}

// --- DESTRUCTOR ---
CameraInterface::~CameraInterface()
{
    LOG_DEBUG("CameraInterface: Destructor called.");
    stopCapture(); // Ensure process is stopped and cleaned up
    // frameBuffer shared_ptr will clean itself up automatically
}

// --- INITIALIZE ---
bool CameraInterface::initialize(const std::string &device, int width, int height, int fps)
{
    if (isInitialized.load())
    {
        LOG_WARN("CameraInterface already initialized.");
        return false;
    }

    devicePath = device;
    frameWidth = width;
    frameHeight = height;
    targetFPS = fps;
    cameraProcess = nullptr; // Ensure it's null before starting
    cameraCommand = "";      // Ensure it's empty before building

    // --- BUILD THE rpicam-vid COMMAND ---
    std::ostringstream cmdStream;
    cmdStream << "rpicam-vid --inline --timeout 0 --nopreview";
    cmdStream << " --codec mjpeg"; // MJPEG is good for streaming and OpenCV decoding
    cmdStream << " --width " << frameWidth;
    cmdStream << " --height " << frameHeight;
    cmdStream << " --framerate " << targetFPS;
    cmdStream << " --output -"; // Output to stdout

    cameraCommand = cmdStream.str();
    LOG_INFO("CameraInterface: Prepared command: " << cameraCommand);

    isInitialized.store(true);
    LOG_INFO("CameraInterface: Initialization marked as successful.");
    return true;
}

// --- START CAPTURE ---
bool CameraInterface::startCapture()
{
    if (!isInitialized.load())
    {
        LOG_ERROR("CameraInterface not initialized. Cannot start capture.");
        return false;
    }
    if (isCapturing.load())
    {
        LOG_WARN("CameraInterface capture already started.");
        return false;
    }

    stopCaptureFlag.store(false);

    // --- START THE rpicam-vid PROCESS ---
    LOG_DEBUG("CameraInterface: Attempting to start rpicam-vid process...");
    cameraProcess = popen(cameraCommand.c_str(), "r");
    if (!cameraProcess)
    {
        perror("CameraInterface: Failed to start rpicam-vid process");
        LOG_ERROR("CameraInterface: Failed to start rpicam-vid process: " << strerror(errno));
        isInitialized.store(false); // Mark as failed to initialize properly
        return false;
    }
    LOG_INFO("CameraInterface: rpicam-vid process started successfully.");

    isCapturing.store(true);
    captureThread = std::thread(&CameraInterface::captureLoop, this);

    LOG_INFO("CameraInterface initialized and capturing.");
    return true;
}

// --- STOP CAPTURE ---
void CameraInterface::stopCapture()
{
    LOG_DEBUG("CameraInterface: stopCapture() called.");
    if (isCapturing.load())
    {
        LOG_INFO("CameraInterface: Signaling capture thread to stop...");
        stopCaptureFlag.store(true);
        if (captureThread.joinable())
        {
            LOG_DEBUG("CameraInterface: Joining capture thread...");
            captureThread.join();
            LOG_INFO("CameraInterface: Capture thread joined.");
        }
        isCapturing.store(false);
        LOG_INFO("CameraInterface: Capture state marked as stopped.");
    }

    // --- CLEAN UP THE SUBPROCESS ---
    if (cameraProcess)
    {
        LOG_DEBUG("CameraInterface: Closing rpicam-vid process...");
        int result = pclose(cameraProcess);
        cameraProcess = nullptr;
        if (result == -1)
        {
            perror("CameraInterface: Error closing rpicam-vid process");
            LOG_ERROR("CameraInterface: Error closing rpicam-vid process: " << strerror(errno));
        }
        else
        {
            LOG_INFO("CameraInterface: rpicam-vid process closed (exit code: " << result << ").");
        }
    }
    LOG_DEBUG("CameraInterface: stopCapture() completed.");
}

// --- IS RUNNING ---
bool CameraInterface::isRunning() const
{
    bool running = isInitialized.load() && isCapturing.load();
    LOG_VERBOSE("CameraInterface: isRunning() returning " << (running ? "true" : "false"));
    return running;
}

// --- CAPTURE LOOP (PRIVATE) ---
void CameraInterface::captureLoop()
{
    LOG_INFO("CameraInterface: captureLoop thread started.");

    const size_t BUFFER_SIZE = 1024 * 1024; // 1MB buffer
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    std::vector<uint8_t> jpegData; // To accumulate JPEG data

    // Simple state machine to find JPEG SOI (0xFFD8) and EOI (0xFFD9)
    enum class JpegState
    {
        FIND_SOI,
        ACCUMULATE,
        FOUND_EOI
    };
    JpegState state = JpegState::FIND_SOI;
    const uint8_t SOI_MARKER[] = {0xFF, 0xD8};
    const uint8_t EOI_MARKER[] = {0xFF, 0xD9};

    // Reset parser state at the beginning
    previous_byte = 0;
    jpegData.clear();
    state = JpegState::FIND_SOI;

    while (!stopCaptureFlag.load())
    {
        LOG_VERBOSE("CameraInterface: captureLoop iteration started.");

        // --- READ DATA FROM rpicam-vid ---
        size_t bytesRead = fread(buffer.data(), sizeof(uint8_t), BUFFER_SIZE, cameraProcess);

        if (bytesRead == 0)
        {
            if (feof(cameraProcess))
            {
                LOG_WARN("CameraInterface: rpicam-vid stream ended (EOF).");
                break; // Exit loop if stream ends
            }
            if (ferror(cameraProcess))
            {
                LOG_ERROR("CameraInterface: Error reading from rpicam-vid stream.");
                clearerr(cameraProcess); // Clear error flag and try again
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // If bytesRead is 0 but no error/EOF, just try again quickly
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        LOG_DEBUG("CameraInterface: Read " << bytesRead << " bytes from rpicam-vid.");

        // --- PROCESS THE READ DATA TO EXTRACT JPEG FRAMES ---
        for (size_t i = 0; i < bytesRead; ++i)
        {
            uint8_t byte = buffer[i];
            LOG_VERBOSE("Processing byte " << i << ", value 0x" << std::hex << static_cast<int>(byte) << std::dec);

            switch (state)
            {
            case JpegState::FIND_SOI:
                LOG_VERBOSE("State: FIND_SOI");
                // Look for the start of a JPEG image (0xFFD8)
                // Handle potential 0xFF stuffing (0xFF00 -> 0xFF)
                if (previous_byte == SOI_MARKER[0])
                {
                    if (byte == 0xD8)
                    {
                        // Found SOI (0xFFD8)
                        LOG_DEBUG("Found SOI (0xFFD8)");
                        jpegData.clear();
                        jpegData.push_back(0xFF);
                        jpegData.push_back(0xD8);
                        state = JpegState::ACCUMULATE;
                    }
                    // else if (byte == 0x00) {
                    //     // Stuffed 0xFF, ignore the 0x00
                    //     // previous_byte remains 0xFF for next comparison
                    // }
                    // else {
                    //     // Unexpected byte after 0xFF, treat as normal byte
                    //     previous_byte = byte;
                    // }
                }
                else
                {
                    // Keep the last byte to check for SOI in next iteration
                    previous_byte = byte;
                }
                break;

            case JpegState::ACCUMULATE:
                LOG_VERBOSE("State: ACCUMULATE (size: " << jpegData.size() << ")");
                jpegData.push_back(byte);
                // Check for potential EOI marker (0xFFD9)
                // We need at least 2 bytes to check the last two
                if (jpegData.size() >= 2)
                {
                    // Check if the last two bytes are EOI (0xFFD9)
                    if (jpegData[jpegData.size() - 2] == EOI_MARKER[0] &&
                        jpegData[jpegData.size() - 1] == EOI_MARKER[1])
                    {
                        LOG_DEBUG("Found EOI (0xFFD9). JPEG size: " << jpegData.size() << " bytes.");
                        state = JpegState::FOUND_EOI;
                    }
                }
                // Always update previous byte in ACCUMULATE state
                previous_byte = byte;
                break;

            case JpegState::FOUND_EOI:
                LOG_INFO("Complete JPEG frame candidate found (size: " << jpegData.size() << " bytes). Attempting decode...");
                // We have a complete JPEG in jpegData
                {
                    // --- DECODE JPEG TO cv::Mat ---
                    // Create a cv::Mat header for the JPEG data
                    cv::Mat jpegMat(1, jpegData.size(), CV_8UC1, jpegData.data());
                    LOG_DEBUG("Created cv::Mat header for JPEG data (size: " << jpegData.size() << " bytes).");
                    // Use cv::imdecode to decode the JPEG data into a BGR Mat
                    cv::Mat frame = cv::imdecode(jpegMat, cv::IMREAD_COLOR);
                    LOG_DEBUG("cv::imdecode called.");

                    if (frame.empty())
                    {
                        LOG_WARN("Failed to decode JPEG frame (size: " << jpegData.size() << " bytes). Data might be corrupt.");
                    }
                    else
                    {
                        LOG_INFO("Successfully decoded JPEG frame (size: " << frame.cols << "x" << frame.rows << ").");
                        // --- STORE THE DECODED FRAME IN THE BUFFER ---
                        if (frameBuffer)
                        {
                            frameBuffer->putFrame(frame);
                            LOG_INFO("Put decoded frame into FrameBuffer.");
                        }
                        else
                        {
                            LOG_ERROR("FrameBuffer is null. Cannot put frame.");
                        }
                    }
                }
                // Reset state and buffer for the next frame
                jpegData.clear();
                state = JpegState::FIND_SOI;
                // Push the current byte back to start checking for next SOI
                // This handles the case where SOI could start immediately after EOI
                previous_byte = byte;
                break;
            } // End switch(state)
        } // End for(bytesRead)
        LOG_VERBOSE("Finished processing chunk of " << bytesRead << " bytes.");
    } // End while loop

    LOG_INFO("CameraInterface: captureLoop thread exiting.");
}
// --- END OF CAPTURE LOOP ---