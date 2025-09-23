// src/vision/camera_interface.cpp
// Implementation of CameraInterface using rpicam-vid subprocess and OpenCV

#include "camera_interface.h"
#include <iostream>
#include <chrono>
#include <fcntl.h>
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
    // Constructor initializes atomic flags and member variables
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
    // --inline: Embeds JPEG headers in each frame for easier decoding
    // --timeout 0: Run indefinitely
    // --nopreview: Suppress on-screen preview
    // --codec mjpeg: Output Motion JPEG stream
    // --width/--height/--framerate: Set capture parameters
    // --output - : Output to stdout (which we will read)
    std::ostringstream cmdStream;
    cmdStream << "rpicam-vid --inline --timeout 0 --nopreview";
    cmdStream << " --codec mjpeg"; // MJPEG is good for streaming and OpenCV decoding
    cmdStream << " --width " << frameWidth;
    cmdStream << " --height " << frameHeight;
    cmdStream << " --framerate " << targetFPS;
    cmdStream << " --output -"; // Output to stdout

    cameraCommand = cmdStream.str();
    LOG_INFO("CameraInterface: Prepared command: " << cameraCommand);

    // Test if rpicam-vid command exists (optional but good practice)
    // You could run 'which rpicam-vid' or try opening the process and checking.

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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Check if the process is still running using feof/ferror on a non-blocking read attempt
    // A more robust way is to use waitpid with WNOHANG, but this is simpler for now.
    int fd = fileno(cameraProcess); // Get file descriptor
    if (fd != -1)
    {
        // Set file descriptor to non-blocking
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        char dummy;
        ssize_t result = read(fd, &dummy, 1); // Try a non-blocking read
        if (result == 0)
        { // EOF immediately means process exited
            LOG_ERROR("CameraInterface: rpicam-vid process exited immediately after start (EOF). Check command or camera access.");
            pclose(cameraProcess); // Clean up
            cameraProcess = nullptr;
            isInitialized.store(false);
            return false;
        }
        else if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        { // Real read error
            LOG_ERROR("CameraInterface: Error checking rpicam-vid process health: " << strerror(errno));
            pclose(cameraProcess);
            cameraProcess = nullptr;
            isInitialized.store(false);
            return false;
        }
        // If result > 0 or EAGAIN/EWOULDBLOCK, process seems healthy for now
        // Reset file descriptor to blocking (fread handles blocking)
        fcntl(fd, F_SETFL, flags);
    }
    else
    {
        LOG_WARN("CameraInterface: Could not get file descriptor for rpicam-vid process for health check.");
    }
    isCapturing.store(true);
    captureThread = std::thread(&CameraInterface::captureLoop, this);

    LOG_INFO("CameraInterface: rpicam-vid process started successfully.");
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
    std::vector<uint8_t> jpegData;

    // Simple state machine for JPEG parsing
    enum class JpegState
    {
        FIND_SOI,
        ACCUMULATE
    };
    JpegState state = JpegState::FIND_SOI;

    // Keep track of the previous byte for marker detection
    uint8_t previousByte = 0;

    while (!stopCaptureFlag.load())
    {
        LOG_VERBOSE("CameraInterface: captureLoop iteration started.");

        size_t bytesRead = fread(buffer.data(), sizeof(uint8_t), BUFFER_SIZE, cameraProcess);

        if (bytesRead == 0)
        {
            if (feof(cameraProcess))
            {
                LOG_WARN("CameraInterface: rpicam-vid stream ended (EOF).");
                // cameraAcquisitionFailed = true;
                break;
            }
            if (ferror(cameraProcess))
            {
                LOG_ERROR("CameraInterface: Error reading from rpicam-vid stream.");
                clearerr(cameraProcess);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        LOG_VERBOSE("CameraInterface: Read " << bytesRead << " bytes from rpicam-vid.");

        for (size_t i = 0; i < bytesRead; ++i)
        {
            uint8_t currentByte = buffer[i];
            LOG_VERBOSE("Processing byte " << i << ", value 0x" << std::hex << static_cast<int>(currentByte) << std::dec);

            switch (state)
            {
            case JpegState::FIND_SOI:
                LOG_VERBOSE("State: FIND_SOI");
                // Look for SOI marker (0xFFD8)
                if (previousByte == 0xFF && currentByte == 0xD8)
                {
                    LOG_DEBUG("Found SOI (0xFFD8)");
                    jpegData.clear();
                    jpegData.push_back(0xFF);
                    jpegData.push_back(0xD8);
                    state = JpegState::ACCUMULATE;
                }
                else
                {
                    // Update previous byte for next iteration
                    previousByte = currentByte;
                }
                break;

            case JpegState::ACCUMULATE:
                LOG_VERBOSE("State: ACCUMULATE (size: " << jpegData.size() << ")");
                jpegData.push_back(currentByte);

                // Check for EOI marker (0xFFD9)
                // We need at least 2 bytes to check the last two
                if (jpegData.size() >= 2 &&
                    jpegData[jpegData.size() - 2] == 0xFF &&
                    jpegData[jpegData.size() - 1] == 0xD9)
                {
                    LOG_DEBUG("CameraInterface: Found EOI (0xFFD9). JPEG size: " << jpegData.size() << " bytes.");
                    // Check if the last two bytes are 0xFFD9
                    cv::Mat jpegMat(1, jpegData.size(), CV_8UC1, jpegData.data());
                    cv::Mat frame = cv::imdecode(jpegMat, cv::IMREAD_COLOR);

                    if (frame.empty())
                    {
                        LOG_WARN("Failed to decode JPEG frame (size: " << jpegData.size() << " bytes).");
                    }
                    else
                    {
                        LOG_DEBUG("CameraInterface: Successfully decoded JPEG frame (size: " << frame.cols << "x" << frame.rows << ").");
                        frameBuffer->putFrame(frame);
                        LOG_INFO("Put decoded frame into FrameBuffer.");
                    }

                    // Clear jpegData for the next frame
                    jpegData.clear();
                    state = JpegState::FIND_SOI;
                    // Reset previousByte to 0 to start fresh search for SOI
                    previousByte = 0;
                }
            }
            // Always update previous byte in ACCUMULATE state
            previousByte = currentByte;
            break;
        } // End for (processing bytes)

        LOG_VERBOSE("Finished processing chunk of " << bytesRead << " bytes.");
    } // End while loop

    LOG_INFO("CameraInterface: captureLoop thread exiting.");
}