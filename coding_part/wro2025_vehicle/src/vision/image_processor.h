// vision/image_processor.h
// Core image processing logic

#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
// #include "camera_interface.h"               // To get frames
#include "frame_buffer.h"                    // Include the buffer header
#include "../navigation/positional_memory.h" // For VisionSnapshot
// --- ADD INCLUDES FOR NEW MODULES ---
#include "color_detector.h"
#include "edge_detector.h"
// #include "roi_selector.h" // Future if needed
class ImageProcessor
{
public:
    ImageProcessor();
    ~ImageProcessor();

    // Initialize with a camera source
    bool initialize(std::shared_ptr<FrameBuffer> frameBuf);

    // Start processing frames in a background thread
    bool startProcessing();

    // Stop processing frames
    void stopProcessing();

    // Get the latest processed vision snapshot
    bool getLatestSnapshot(VisionSnapshot &snapshot);

    // Check if processing is active
    bool isRunning() const;

private:
    // std::shared_ptr<CameraInterface> cameraSource;
    std::shared_ptr<FrameBuffer> frameBufferSource;
    std::atomic<bool> isInitialized;
    std::atomic<bool> isProcessing;
    std::thread processingThread;
    std::atomic<bool> stopProcessingFlag;

    // Synchronization for the latest snapshot
    mutable std::mutex snapshotMutex;
    VisionSnapshot latestSnapshot;
    std::atomic<bool> newSnapshotAvailable;

    // Processing parameters (could be moved to config)
    cv::Rect roi; // Region of Interest for sign detection
    ColorDetector colorDetector_;
    EdgeDetector edgeDetector_;
    // HSV ranges for Red and Green (from rules: RGB(238, 39, 55) and RGB(68, 214, 44))
    // These will need tuning
    cv::Scalar redLower1, redUpper1; // Red wraps around in HSV
    cv::Scalar redLower2, redUpper2;
    cv::Scalar greenLower, greenUpper;

    // Function run by the processing thread
    void processingLoop();

    // Core processing functions
    void processFrame(const cv::Mat &frame, VisionSnapshot &outputSnapshot);
    // void detectSigns(const cv::Mat &hsvFrame, bool &leftSign, bool &rightSign, cv::Point &leftCentroid, cv::Point &rightCentroid);
    // void detectEdges(const cv::Mat &frame, float &avgLeftEdgeX, float &avgRightEdgeX); // Simplified
};

#endif // IMAGE_PROCESSOR_H