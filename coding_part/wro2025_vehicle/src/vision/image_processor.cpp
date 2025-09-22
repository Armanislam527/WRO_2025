// src/vision/image_processor.cpp
// Implementation of ImageProcessor (Coordinator using modular detectors)

#include "image_processor.h"
#include <iostream>
#include <cmath>     // For round, atan2, sqrt etc.
#include <algorithm> // For std::max, std::min if needed
// --- INCLUDES FOR NEW MODULES ---
#include "color_detector.h"
#include "edge_detector.h"
#include "../utils/logger.h" // Include the logger
// --- END OF INCLUDES ---

ImageProcessor::ImageProcessor()
    : isInitialized(false), isProcessing(false), stopProcessingFlag(false), newSnapshotAvailable(false)
{
    // Define initial ROI (e.g., top half of the image)
    // This will be adjusted based on the actual frame size in processFrame
    roi = cv::Rect(0, 0, 640, 240);

    // --- INITIALIZE DETECTORS WITH DEFAULT PARAMETERS ---
    // Parameters can be tuned here or via a configuration mechanism later
    // ColorDetector uses defaults set in its constructor
    // Example of setting custom HSV ranges if needed:
    // colorDetector_.setRedHSVRange(cv::Scalar(0, 120, 120), cv::Scalar(5, 255, 255),
    //                               cv::Scalar(175, 120, 120), cv::Scalar(180, 255, 255));
    // colorDetector_.setGreenHSVRange(cv::Scalar(45, 60, 60), cv::Scalar(75, 255, 255));

    // EdgeDetector uses defaults set in its constructor
    // Example of setting custom Canny/Hough parameters if needed:
    // edgeDetector_.setCannyThresholds(40, 120);
    // edgeDetector_.setHoughParams(1.0, CV_PI / 180, 40, 25.0, 8.0);
    // --- END OF INITIALIZATION ---
}

ImageProcessor::~ImageProcessor()
{
    stopProcessing();
}

bool ImageProcessor::initialize(std::shared_ptr<FrameBuffer> frameBuf)
{
    if (isInitialized)
    {
        std::cerr << "ImageProcessor already initialized." << std::endl;
        return false;
    }
    if (!frameBuf)
    {
        std::cerr << "Invalid frame buffer provided." << std::endl;
        return false;
    }

    frameBufferSource = frameBuf;
    isInitialized = true;
    std::cout << "ImageProcessor initialized with frame buffer." << std::endl;
    return true;
}

bool ImageProcessor::startProcessing()
{
    if (!isInitialized)
    {
        std::cerr << "ImageProcessor not initialized. Cannot start processing." << std::endl;
        return false;
    }
    if (isProcessing)
    {
        std::cerr << "Image processing already started." << std::endl;
        return false;
    }

    stopProcessingFlag = false;
    isProcessing = true;
    processingThread = std::thread(&ImageProcessor::processingLoop, this);

    std::cout << "Image processing started." << std::endl;
    return true;
}

void ImageProcessor::stopProcessing()
{
    if (isProcessing)
    {
        stopProcessingFlag = true;
        if (processingThread.joinable())
        {
            processingThread.join();
        }
        isProcessing = false;
        std::cout << "Image processing stopped." << std::endl;
    }
}

bool ImageProcessor::getLatestSnapshot(VisionSnapshot &snapshot)
{
    if (!isInitialized)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(snapshotMutex);
    if (newSnapshotAvailable)
    {
        snapshot = latestSnapshot;
        newSnapshotAvailable = false;
        return true;
    }
    return false; // No new snapshot available
}

bool ImageProcessor::isRunning() const
{
    return isInitialized && isProcessing;
}

void ImageProcessor::processingLoop()
{
    cv::Mat frame;
    VisionSnapshot currentSnapshot;
    int wait_counter = 0; // Counter for wait messages

    while (!stopProcessingFlag)
    {
        LOG_VERBOSE("ImageProcessor: processingLoop iteration started.");
        // Get the latest frame from the frame buffer
        // Check if a new frame is available first
        if (frameBufferSource && frameBufferSource->hasNewFrame())
        {
            LOG_DEBUG("ImageProcessor: frameBufferSource->hasNewFrame() returned true.");
            // Attempt to get the frame
            if (frameBufferSource->getFrame(frame))
            {
                LOG_INFO("ImageProcessor: Successfully got a frame from FrameBuffer (size: " << frame.cols << "x" << frame.rows << ").");
                // Process the frame
                processFrame(frame, currentSnapshot);
                LOG_INFO("ImageProcessor: Completed processFrame call.");

                // Store the processed snapshot
                {
                    std::lock_guard<std::mutex> lock(snapshotMutex);
                    latestSnapshot = currentSnapshot;
                    newSnapshotAvailable = true;
                }
                LOG_INFO("ImageProcessor: Updated latestSnapshot and signaled new data.");
                // Reset wait counter as we processed a frame
                wait_counter = 0;
            }
            else
            {
                LOG_WARN("ImageProcessor: hasNewFrame() was true, but getFrame() failed. Possible race condition or buffer issue.");
            }
        }
        else
        {
            // No new frame is available, handle waiting
            // Small sleep to prevent busy-waiting
            wait_counter++;
            if (wait_counter % 500 == 0) // Log every 500 cycles (~1 second at 2ms sleep)
            {
                LOG_INFO("ImageProcessor: Waiting for new frame in processingLoop (cycle " << wait_counter << ").");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        LOG_VERBOSE("ImageProcessor: processingLoop iteration ended.");
    } // End while loop
    LOG_INFO("ImageProcessor: processingLoop thread exiting.");
}
void ImageProcessor::processFrame(const cv::Mat &frame, VisionSnapshot &outputSnapshot)
{
    LOG_DEBUG("ImageProcessor::processFrame called.");
    // std::cout << "ImageProcessor::processFrame called." << std::endl; // Verbose
    // Use LOG_DEBUG if logger is active
    // Initialize output
    outputSnapshot.signLeftDetected = false;
    outputSnapshot.signRightDetected = false;
    outputSnapshot.avgLeftEdgeX = -1.0f; // Indicate not found
    outputSnapshot.avgRightEdgeX = -1.0f;

    if (frame.empty())
    {
        // std::cerr << "ImageProcessor::processFrame received an empty frame." << std::endl; // Verbose
        // Use LOG_WARN if logger is active
        LOG_WARN("ImageProcessor::processFrame received an empty frame.");
        return; // Nothing to process
    }
    // std::cout << "ImageProcessor::processFrame processing frame of size " << frame.cols << "x" << frame.rows << std::endl; // Verbose
    // Use LOG_DEBUG if logger is active
    LOG_DEBUG("ImageProcessor::processFrame processing frame of size " << frame.cols << "x" << frame.rows);

    // Adjust ROI based on current frame size if needed
    // Simple adjustment: top half. In a more complex scenario, this could be dynamic.
    if (roi.width != frame.cols || roi.height != frame.rows / 2)
    {
        roi = cv::Rect(0, 0, frame.cols, frame.rows / 2);
        // std::cout << "ImageProcessor: ROI adjusted to " << roi.width << "x" << roi.height << std::endl; // Verbose
        // Use LOG_DEBUG if logger is active
        LOG_DEBUG("ImageProcessor: ROI adjusted to " << roi.width << "x" << roi.height);
    }

    // Extract ROI
    cv::Mat roiFrame = frame(roi);

    // --- 1. Detect Traffic Signs using ColorDetector ---
    // Convert ROI to HSV for color detection
    cv::Mat hsvFrame;
    cv::cvtColor(roiFrame, hsvFrame, cv::COLOR_BGR2HSV);

    cv::Point leftCentroid, rightCentroid;
    bool leftSignFound = colorDetector_.detectGreenSign(hsvFrame, leftCentroid);
    bool rightSignFound = colorDetector_.detectRedSign(hsvFrame, rightCentroid);

    outputSnapshot.signLeftDetected = leftSignFound;
    outputSnapshot.signRightDetected = rightSignFound;
    // Note: Centroids are relative to the ROI. If full-frame coordinates are needed later,
    // adjust them by adding roi.x, roi.y. For now, they are sufficient for relative positioning logic.

    // --- Optional: Resolve conflicts if both detected ---
    // E.g., if centroids are very close, or one is much larger, prioritize.
    // Define a minimum expected distance between signs (adjust based on testing)
    const double minExpectedSignDistance = 50.0; // Example value in pixels

    if (leftSignFound && rightSignFound)
    {
        double distance = cv::norm(leftCentroid - rightCentroid);
        if (distance < minExpectedSignDistance)
        {
            // std::cerr << "ImageProcessor: Sign conflict detected (distance: " << distance << "). Resolving..." << std::endl; // Verbose
            // Likely a false positive, choose the one with larger area or better solidity
            // A more sophisticated method could be in ColorDetector itself.
            // Simple heuristic: Prefer left sign (arbitrary, or based on last known state)
            // For now, let's disable the right sign.
            outputSnapshot.signRightDetected = false;
            rightCentroid = cv::Point(-1, -1); // Invalidate centroid
            // std::cerr << "ImageProcessor: Conflict resolved. Disabled right sign." << std::endl; // Verbose
            // Use LOG_WARN/INFO if logger is active
            LOG_WARN("ImageProcessor: Conflict resolved. Disabled right sign.");
        }
    }
    // --- End of Sign Detection ---

    // --- 2. Detect Track Edges using EdgeDetector ---
    std::vector<cv::Vec4i> detectedLines;
    edgeDetector_.detectEdgesAndLines(roiFrame, detectedLines);
    edgeDetector_.findAverageEdgePositions(detectedLines, roiFrame.cols, outputSnapshot.avgLeftEdgeX, outputSnapshot.avgRightEdgeX);
    // --- End of Edge Detection ---

    // std::cout << "ImageProcessor::processFrame completed." << std::endl; // Verbose
    // Use LOG_DEBUG if logger is active
    LOG_DEBUG("ImageProcessor::processFrame completed.");
}
// --- REMOVED OLD MONOLITHIC detectSigns AND detectEdges FUNCTIONS ---
// Their logic has been moved to ColorDetector and EdgeDetector respectively.
// --- END OF REMOVAL ---