// src/control/image_processor.cpp
// Implementation of ImageProcessor (Coordinator using modular detectors)

#include "image_processor.h"
#include <iostream>
#include <cmath>             // For round, atan2, sqrt etc.
#include <algorithm>         // For std::max, std::min if needed
#include "../utils/logger.h" // Include the logger

ImageProcessor::ImageProcessor()
    : isInitialized(false), isProcessing(false), stopProcessingFlag(false), newSnapshotAvailable(false)
{
    // Define initial ROI (e.g., top half of the image)
    roi = cv::Rect(0, 0, 640, 240); // Will be adjusted based on actual frame size

    // --- INITIALIZE DETECTORS WITH DEFAULT PARAMETERS ---
    // ColorDetector uses defaults set in its constructor
    // EdgeDetector uses defaults set in its constructor
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

    LOG_INFO("ImageProcessor: processingLoop thread started.");

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
                // Process the frame ONLY IF getFrame was successful
                processFrame(frame, currentSnapshot);
                LOG_INFO("ImageProcessor: Completed processFrame call.");

                // Store the processed snapshot
                {
                    std::lock_guard<std::mutex> lock(snapshotMutex);
                    latestSnapshot = currentSnapshot;
                    newSnapshotAvailable = true;
                }
                LOG_INFO("ImageProcessor: Updated latestSnapshot and signaled new data.");
                wait_counter = 0; // Reset wait counter after processing
            }
            else
            {
                LOG_WARN("ImageProcessor: hasNewFrame() was true, but getFrame() failed. Possible race condition or buffer issue.");
                // Small sleep to prevent busy-waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
        }
        else
        {
            // No new frame, small sleep to prevent busy-waiting
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

    // Initialize output
    outputSnapshot.signLeftDetected = false;
    outputSnapshot.signRightDetected = false;
    outputSnapshot.avgLeftEdgeX = -1.0f; // Indicate not found
    outputSnapshot.avgRightEdgeX = -1.0f;

    if (frame.empty())
    {
        LOG_WARN("ImageProcessor::processFrame received an empty frame.");
        return; // Nothing to process
    }
    LOG_DEBUG("ImageProcessor::processFrame processing frame of size " << frame.cols << "x" << frame.rows);

    // Adjust ROI based on current frame size if needed
    if (roi.width != frame.cols || roi.height != frame.rows / 2)
    {
        roi = cv::Rect(0, 0, frame.cols, frame.rows / 2);
        LOG_DEBUG("ImageProcessor: ROI adjusted to " << roi.width << "x" << roi.height);
    }

    // Extract ROI
    cv::Mat roiFrame = frame(roi);

    // --- 1. Detect Traffic Signs using ColorDetector ---
    // Convert ROI to HSV for color detection
    cv::Mat hsvFrame;
    cv::cvtColor(roiFrame, hsvFrame, cv::COLOR_BGR2HSV);

    cv::Point leftCentroid, rightCentroid;
    // Pass the full ROI frame dimensions for potential conflict resolution logic
    colorDetector_.detectSigns(hsvFrame, outputSnapshot.signLeftDetected, outputSnapshot.signRightDetected, leftCentroid, rightCentroid);

    // --- Optional: Resolve conflicts if both detected ---
    // E.g., if centroids are very close, or one is much larger, prioritize.
    // Define a minimum expected distance between signs (adjust based on testing)
    const double minExpectedSignDistance = 50.0; // Example value in pixels

    if (outputSnapshot.signLeftDetected && outputSnapshot.signRightDetected)
    {
        double distance = cv::norm(leftCentroid - rightCentroid);
        if (distance < minExpectedSignDistance)
        {
            LOG_WARN("ImageProcessor: Sign conflict detected (distance: " << distance << "). Resolving...");
            // Likely a false positive, choose the one with larger area or better solidity
            // A more sophisticated method could be in ColorDetector itself.
            // Simple heuristic: Prefer left sign (arbitrary, or based on last known state)
            // For now, let's disable the right sign.
            outputSnapshot.signRightDetected = false;
            rightCentroid = cv::Point(-1, -1); // Invalidate centroid
            LOG_WARN("ImageProcessor: Conflict resolved. Disabled right sign.");
        }
    }
    // --- End of Sign Detection ---

    // --- 2. Detect Track Edges using EdgeDetector ---
    std::vector<cv::Vec4i> detectedLines;
    edgeDetector_.detectEdgesAndLines(roiFrame, detectedLines);
    edgeDetector_.findAverageEdgePositions(detectedLines, roiFrame.cols, outputSnapshot.avgLeftEdgeX, outputSnapshot.avgRightEdgeX);
    // --- End of Edge Detection ---

    LOG_DEBUG("ImageProcessor::processFrame completed.");
}
// --- REMOVED OLD MONOLITHIC detectSigns AND detectEdges FUNCTIONS ---
// Their logic has been moved to ColorDetector and EdgeDetector respectively.
// --- END OF REMOVAL ---