// src/vision/camera_interface.cpp
#include "camera_interface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>           // For building command string
#include <unistd.h>          // For ::sleep (if needed for startup delay)
#include "../utils/logger.h" // Include the logger
CameraInterface::CameraInterface()
    : devicePath(""), frameWidth(640), frameHeight(480), targetFPS(20),
      cameraProcess(nullptr), isInitialized(false), isCapturing(false), stopCaptureFlag(false)
{
    frameBuffer = std::make_shared<FrameBuffer>();
}

CameraInterface::~CameraInterface()
{
    stopCapture(); // Ensure process is stopped and cleaned up
    // frameBuffer shared_ptr will clean itself up
}

bool CameraInterface::initialize(const std::string &device, int width, int height, int fps)
{
    if (isInitialized.load())
    {
        std::cerr << "CameraInterface already initialized." << std::endl;
        return false;
    }

    devicePath = device; // Store for potential use in command or logging
    frameWidth = width;
    frameHeight = height;
    targetFPS = fps;

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
    std::cout << "CameraInterface: Prepared command: " << cameraCommand << std::endl;

    // Test if rpicam-vid command exists (optional but good practice)
    // You could run 'which rpicam-vid' or try opening the process and checking.

    isInitialized.store(true);
    return true;
}

bool CameraInterface::startCapture()
{
    if (!isInitialized.load())
    {
        std::cerr << "CameraInterface not initialized. Cannot start capture." << std::endl;
        return false;
    }
    if (isCapturing.load())
    {
        std::cerr << "CameraInterface capture already started." << std::endl;
        return false;
    }

    stopCaptureFlag.store(false);

    // --- START THE rpicam-vid PROCESS ---
    cameraProcess = popen(cameraCommand.c_str(), "r");
    if (!cameraProcess)
    {
        perror("CameraInterface: Failed to start rpicam-vid process");
        isInitialized.store(false); // Mark as failed to initialize properly
        return false;
    }

    isCapturing.store(true);
    captureThread = std::thread(&CameraInterface::captureLoop, this);

    std::cout << "CameraInterface: rpicam-vid process started." << std::endl;
    return true;
}

void CameraInterface::stopCapture()
{
    if (isCapturing.load())
    {
        stopCaptureFlag.store(true);
        if (captureThread.joinable())
        {
            captureThread.join();
        }
        isCapturing.store(false);
        std::cout << "CameraInterface: Capture thread stopped." << std::endl;
    }

    // --- CLEAN UP THE SUBPROCESS ---
    if (cameraProcess)
    {
        int result = pclose(cameraProcess);
        cameraProcess = nullptr;
        if (result == -1)
        {
            perror("CameraInterface: Error closing rpicam-vid process");
        }
        else
        {
            std::cout << "CameraInterface: rpicam-vid process closed (exit code: " << result << ")." << std::endl;
        }
    }
}

bool CameraInterface::isRunning() const
{
    return isInitialized.load() && isCapturing.load();
}

void CameraInterface::captureLoop()
{
    const size_t BUFFER_SIZE = frameWidth * frameHeight * 3; // Estimate for raw BGR, adjust if needed
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    std::vector<uint8_t> jpegData; // To accumulate JPEG data

    // A simple state machine to find JPEG SOI (0xFFD8) and EOI (0xFFD9)
    enum class JpegState
    {
        FIND_SOI,
        ACCUMULATE,
        FOUND_EOI
    };
    JpegState state = JpegState::FIND_SOI;
    const uint8_t SOI_MARKER[] = {0xFF, 0xD8};
    const uint8_t EOI_MARKER[] = {0xFF, 0xD9};

    while (!stopCaptureFlag.load())
    {
        LOG_VERBOSE("CameraInterface: captureLoop iteration started.");
        // --- READ DATA FROM rpicam-vid ---
        // fread returns number of items read, not bytes necessarily if size > 1
        // Using size=1, count=BUFFER_SIZE is safer for byte streams.
        size_t bytesRead = fread(buffer.data(), sizeof(uint8_t), BUFFER_SIZE, cameraProcess);

        if (bytesRead == 0)
        {
            if (feof(cameraProcess))
            {

                LOG_ERROR("CameraInterface: rpicam-vid stream ended (EOF).");
                break; // Exit loop if stream ends
            }
            if (ferror(cameraProcess))
            {
                LOG_ERROR("CameraInterface: Error reading from rpicam-vid stream.");
                // Depending on error, might break or continue
                clearerr(cameraProcess); // Clear error flag and try again
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            LOG_VERBOSE("CameraInterface: Read " << bytesRead << " bytes from rpicam-vid.");
            // If bytesRead is 0 but no error/EOF, just try again quickly
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // --- PROCESS THE READ DATA TO EXTRACT JPEG FRAMES ---
        for (size_t i = 0; i < bytesRead; ++i)
        {
            uint8_t byte = buffer[i];
            LOG_VERBOSE("CameraInterface: Processing byte " << i << " value " << static_cast<int>(byte)); // Too verbose
            switch (state)
            {
            case JpegState::FIND_SOI:
                LOG_VERBOSE("CameraInterface: State FIND_SOI");
                // Look for the start of a JPEG image (0xFFD8)
                if (jpegData.size() >= 1 && jpegData.back() == SOI_MARKER[0] && byte == SOI_MARKER[1])
                {
                    // Found SOI, start accumulating
                    jpegData.clear(); // Clear any partial data
                    jpegData.push_back(SOI_MARKER[0]);
                    jpegData.push_back(SOI_MARKER[1]);
                    state = JpegState::ACCUMULATE;
                }
                else
                {
                    // Keep the last byte to check for SOI in next iteration
                    if (jpegData.size() > 1)
                        jpegData.clear();
                    jpegData.push_back(byte);
                }
                break;

            case JpegState::ACCUMULATE:
                LOG_VERBOSE("CameraInterface: State ACCUMULATE");
                jpegData.push_back(byte);
                // Check if the last two bytes are EOI (0xFFD9)
                if (jpegData.size() >= 2 &&
                    jpegData[jpegData.size() - 2] == EOI_MARKER[0] &&
                    jpegData[jpegData.size() - 1] == EOI_MARKER[1])
                {
                    state = JpegState::FOUND_EOI;
                }
                break;

            case JpegState::FOUND_EOI:
                LOG_DEBUG("CameraInterface: Found complete JPEG frame candidate (size: " << jpegData.size() << " bytes).");
                // We have a complete JPEG in jpegData
                {
                    // --- DECODE JPEG TO cv::Mat ---
                    // Create a cv::Mat header for the JPEG data
                    cv::Mat jpegMat(1, jpegData.size(), CV_8UC1, jpegData.data());
                    // Use cv::imdecode to decode the JPEG data into a BGR Mat
                    cv::Mat frame = cv::imdecode(jpegMat, cv::IMREAD_COLOR);

                    if (frame.empty())
                    {
                        LOG_WARN("CameraInterface: Failed to decode JPEG frame (size: " << jpegData.size() << " bytes).");
                    }
                    else
                    {
                        LOG_DEBUG("CameraInterface: Successfully decoded JPEG frame (size: " << frame.cols << "x" << frame.rows << ").");
                        // --- STORE THE DECODED FRAME IN THE BUFFER ---
                        frameBuffer->putFrame(frame);
                        // std::cout << "CameraInterface: Captured and decoded a frame." << std::endl; // Verbose
                        LOG_INFO("CameraInterface: Put decoded frame into FrameBuffer.");
                    }
                }
                // Reset state and buffer for the next frame
                jpegData.clear();
                state = JpegState::FIND_SOI;
                // Push the current byte back to start checking for next SOI
                jpegData.push_back(byte);
                break;
            }
        }
        LOG_VERBOSE("CameraInterface: Finished processing chunk of " << bytesRead << " bytes.");
        // Small sleep to prevent busy-waiting if reading is very fast
        // The blocking nature of fread should mostly prevent this.
        // std::this_thread::sleep_for(std::chrono::microseconds(100));
    } // End while loop

    LOG_INFO("CameraInterface: captureLoop thread exiting.");
}