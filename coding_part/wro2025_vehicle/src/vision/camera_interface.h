// vision/camera_interface.h
// Interface for camera capture

#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <opencv2/opencv.hpp> // We'll assume OpenCV for now
#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include "frame_buffer.h"
class CameraInterface
{
public:
    CameraInterface();
    ~CameraInterface();

    // Initialize the camera with specified parameters
    bool initialize(const std::string &device = "/dev/video0",
                    int width = 640, int height = 480, int fps = 30);

    // Start capturing frames in a background thread
    bool startCapture();

    // Stop capturing frames
    void stopCapture();

    // Retrieve the latest captured frame
    // Returns true if a new frame was available and copied
    bool getLatestFrame(cv::Mat &frame);

    // Check if the camera is initialized and capturing
    bool isRunning() const;
    std::shared_ptr<FrameBuffer> getFrameBuffer() const { return frameBuffer; }

private:
    std::string devicePath;
    int frameWidth;
    int frameHeight;
    int targetFPS;

    cv::VideoCapture capture; // OpenCV VideoCapture object

    std::atomic<bool> isInitialized;
    std::atomic<bool> isCapturing;
    std::thread captureThread;
    std::atomic<bool> stopCaptureFlag;

    // Synchronization for the latest frame
    mutable std::mutex frameMutex;
    // cv::Mat latestFrame;
    // std::atomic<bool> newFrameAvailable;
    std::shared_ptr<FrameBuffer> frameBuffer;
    // Function run by the capture thread
    void captureLoop();
};

#endif // CAMERA_INTERFACE_H