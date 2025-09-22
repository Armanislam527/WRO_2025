// src/vision/camera_interface.h
#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <atomic>
#include <thread>
// #include <opencv2/videoio.hpp> // Remove or comment out, no longer using cv::VideoCapture
#include <cstdio>         // For FILE*, popen, pclose
#include <memory>         // For std::unique_ptr
#include "frame_buffer.h" // Include the buffer header

class CameraInterface
{
public:
    CameraInterface();
    ~CameraInterface(); // Important for cleanup

    // Initialize the camera with specified parameters
    // Note: Some parameters like FPS might be set via rpicam-vid arguments
    bool initialize(const std::string &device = "/base/soc/i2c0mux/i2c@1/ov5647@36", // Use libcamera ID if known, or let rpicam-vid find it
                    int width = 640, int height = 480, int fps = 20);

    // Start capturing frames in a background thread
    bool startCapture();

    // Stop capturing frames
    void stopCapture();

    // Check if the camera is initialized and capturing
    bool isRunning() const;

    // Get the shared frame buffer
    std::shared_ptr<FrameBuffer> getFrameBuffer() const { return frameBuffer; }

private:
    std::string devicePath; // Might be used for rpicam-vid arguments or logging
    int frameWidth;
    int frameHeight;
    int targetFPS;

    // --- CHANGED: Use FILE* for subprocess ---
    FILE *cameraProcess;       // Handle for the rpicam-vid process
    std::string cameraCommand; // The command used to start rpicam-vid

    std::atomic<bool> isInitialized;
    std::atomic<bool> isCapturing;
    std::thread captureThread;
    std::atomic<bool> stopCaptureFlag;

    std::shared_ptr<FrameBuffer> frameBuffer;

    // Function run by the capture thread
    void captureLoop();
};

#endif // CAMERA_INTERFACE_H