// vision/camera_interface.cpp
// Implementation of CameraInterface using OpenCV

#include "camera_interface.h"
#include <iostream>
#include <chrono>
#include <thread>

CameraInterface::CameraInterface()
    : isInitialized(false), isCapturing(false), stopCaptureFlag(false), newFrameAvailable(false)
{
    // Constructor initializes atomic flags and member variables
    frameBuffer = std::make_shared<FrameBuffer>();
}

CameraInterface::~CameraInterface()
{
    stopCapture();
    if (capture.isOpened())
    {
        capture.release();
    }
}

bool CameraInterface::initialize(const std::string &device, int width, int height, int fps)
{
    if (isInitialized)
    {
        std::cerr << "Camera already initialized." << std::endl;
        return false;
    }

    devicePath = device;
    frameWidth = width;
    frameHeight = height;
    targetFPS = fps;

    // Attempt to open the camera device using OpenCV
    // OpenCV will use V4L2 backend on Linux
    capture.open(devicePath);
    if (!capture.isOpened())
    {
        std::cerr << "Error: Could not open camera device " << devicePath << std::endl;
        return false;
    }

    // Set camera properties
    capture.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
    capture.set(cv::CAP_PROP_FPS, targetFPS);

    // Verify settings (OpenCV might not always respect them exactly)
    int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    int actualFPS = static_cast<int>(capture.get(cv::CAP_PROP_FPS));

    std::cout << "Camera initialized: " << devicePath << std::endl;
    std::cout << "Requested resolution: " << frameWidth << "x" << frameHeight << " @ " << targetFPS << "fps" << std::endl;
    std::cout << "Actual resolution: " << actualWidth << "x" << actualHeight << " @ " << actualFPS << "fps" << std::endl;

    // Simple check
    if (actualWidth <= 0 || actualHeight <= 0)
    {
        std::cerr << "Error: Failed to set camera resolution." << std::endl;
        capture.release();
        return false;
    }

    isInitialized = true;
    return true;
}

bool CameraInterface::startCapture()
{
    if (!isInitialized)
    {
        std::cerr << "Camera not initialized. Cannot start capture." << std::endl;
        return false;
    }
    if (isCapturing)
    {
        std::cerr << "Camera capture already started." << std::endl;
        return false;
    }

    stopCaptureFlag = false;
    isCapturing = true;
    captureThread = std::thread(&CameraInterface::captureLoop, this);

    std::cout << "Camera capture started." << std::endl;
    return true;
}

void CameraInterface::stopCapture()
{
    if (isCapturing)
    {
        stopCaptureFlag = true;
        if (captureThread.joinable())
        {
            captureThread.join();
        }
        isCapturing = false;
        std::cout << "Camera capture stopped." << std::endl;
    }
}

/**bool CameraInterface::getLatestFrame(cv::Mat &frame)
{
    if (!isInitialized)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(frameMutex);
    if (newFrameAvailable)
    {
        frame = latestFrame.clone(); // Clone to provide a copy
        newFrameAvailable = false;
        return true;
    }
    return false; // No new frame available
}**/

bool CameraInterface::isRunning() const
{
    return isInitialized && isCapturing;
}

void CameraInterface::captureLoop()
{
    cv::Mat frame;
    const auto frameInterval = std::chrono::milliseconds(1000 / targetFPS);

    while (!stopCaptureFlag)
    {
        auto frameStart = std::chrono::steady_clock::now();

        if (!capture.read(frame))
        {
            std::cerr << "Error: Failed to grab frame from camera." << std::endl;
            // Depending on requirements, you might want to stop capture or retry
            // For now, let's just wait and try again
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        // Store the captured frame IN THE BUFFER
        frameBuffer->putFrame(frame);
        // Store the captured frame
        /**{
            std::lock_guard<std::mutex> lock(frameMutex);
            latestFrame = frame;
            newFrameAvailable = true;
        }**/
        // std::cout << "Captured new frame." << std::endl; // Verbose

        // Maintain target FPS (approximate)
        auto frameEnd = std::chrono::steady_clock::now();
        auto frameDuration = std::chrono::duration_cast<std::chrono::milliseconds>(frameEnd - frameStart);
        if (frameDuration < frameInterval)
        {
            auto sleepTime = frameInterval - frameDuration;
            std::this_thread::sleep_for(sleepTime);
        }
    }
}