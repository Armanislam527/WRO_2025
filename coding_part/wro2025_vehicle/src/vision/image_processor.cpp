// vision/image_processor.cpp
// Implementation of ImageProcessor

#include "image_processor.h"
#include <iostream>
#include <cmath> // For round

ImageProcessor::ImageProcessor()
    : isInitialized(false), isProcessing(false), stopProcessingFlag(false), newSnapshotAvailable(false)
{
    // Define initial ROI (e.g., top half of the image)
    roi = cv::Rect(0, 0, 640, 240); // Will be adjusted based on actual frame size

    // Define initial HSV ranges for Red and Green pillars
    // These are starting points and WILL need adjustment based on actual lighting/ camera
    // Red HSV (wraps around 0/180)
    redLower1 = cv::Scalar(0, 100, 100);
    redUpper1 = cv::Scalar(10, 255, 255);
    redLower2 = cv::Scalar(170, 100, 100);
    redUpper2 = cv::Scalar(180, 255, 255);
    // Green HSV
    greenLower = cv::Scalar(40, 50, 50);
    greenUpper = cv::Scalar(80, 255, 255);
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
    { // Check for valid shared pointer
        std::cerr << "Invalid frame buffer provided." << std::endl;
        return false;
    }

    // Store the shared pointer to the frame buffer
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

    while (!stopProcessingFlag)
    {
        // Get the latest frame from the camera
        if (frameBufferSource && frameBufferSource->hasNewFrame())
        {
            if (frameBufferSource->getFrame(frame))
            {
                // Process the frame
                processFrame(frame, currentSnapshot);

                // Store the processed snapshot
                {
                    std::lock_guard<std::mutex> lock(snapshotMutex);
                    latestSnapshot = currentSnapshot;
                    newSnapshotAvailable = true;
                }
                // std::cout << "Processed new frame snapshot." << std::endl; // Verbose
            }
        }
        else
        {
            // No new frame, small sleep to prevent busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

void ImageProcessor::processFrame(const cv::Mat &frame, VisionSnapshot &outputSnapshot)
{
    // Initialize output
    outputSnapshot.signLeftDetected = false;
    outputSnapshot.signRightDetected = false;
    outputSnapshot.avgLeftEdgeX = -1.0f; // Indicate not found
    outputSnapshot.avgRightEdgeX = -1.0f;

    if (frame.empty())
    {
        return; // Nothing to process
    }

    // Adjust ROI based on current frame size if needed
    if (roi.width != frame.cols || roi.height != frame.rows / 2)
    {
        // Simple adjustment: top half
        roi = cv::Rect(0, 0, frame.cols, frame.rows / 2);
    }

    // Extract ROI
    cv::Mat roiFrame = frame(roi);

    // Convert to HSV for color detection
    cv::Mat hsvFrame;
    cv::cvtColor(roiFrame, hsvFrame, cv::COLOR_BGR2HSV);

    // --- 1. Detect Traffic Signs ---
    cv::Point leftCentroid, rightCentroid;
    detectSigns(hsvFrame, outputSnapshot.signLeftDetected, outputSnapshot.signRightDetected, leftCentroid, rightCentroid);

    // --- 2. Detect Track Edges (Simplified) ---
    // A more robust method like Canny + HoughLinesP would be better
    detectEdges(roiFrame, outputSnapshot.avgLeftEdgeX, outputSnapshot.avgRightEdgeX);
}

void ImageProcessor::detectSigns(const cv::Mat &hsvFrame, bool &leftSign, bool &rightSign, cv::Point &leftCentroid, cv::Point &rightCentroid)
{
    leftSign = false;
    rightSign = false;
    leftCentroid = cv::Point(-1, -1);
    rightCentroid = cv::Point(-1, -1);

    // --- Detect Red (Right Lane Sign) ---
    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(hsvFrame, redLower1, redUpper1, redMask1);
    cv::inRange(hsvFrame, redLower2, redUpper2, redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);

    // Noise reduction
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> redContours;
    cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest red contour (assuming it's the sign)
    double largestRedArea = 0;
    int largestRedIndex = -1;
    for (size_t i = 0; i < redContours.size(); i++)
    {
        double area = cv::contourArea(redContours[i]);
        if (area > largestRedArea && area > 100)
        { // Minimum area threshold
            largestRedArea = area;
            largestRedIndex = i;
        }
    }

    if (largestRedIndex != -1)
    {
        rightSign = true;
        // Calculate centroid
        cv::Moments m = cv::moments(redContours[largestRedIndex]);
        if (m.m00 != 0)
        {
            rightCentroid.x = static_cast<int>(m.m10 / m.m00);
            rightCentroid.y = static_cast<int>(m.m01 / m.m00);
            // Adjust centroid to full frame coordinates if needed
            // rightCentroid.y += roi.y; // If ROI is not the full frame top part
        }
    }

    // --- Detect Green (Left Lane Sign) ---
    cv::Mat greenMask;
    cv::inRange(hsvFrame, greenLower, greenUpper, greenMask);

    // Noise reduction
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_CLOSE, kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> greenContours;
    cv::findContours(greenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest green contour
    double largestGreenArea = 0;
    int largestGreenIndex = -1;
    for (size_t i = 0; i < greenContours.size(); i++)
    {
        double area = cv::contourArea(greenContours[i]);
        if (area > largestGreenArea && area > 100)
        { // Minimum area threshold
            largestGreenArea = area;
            largestGreenIndex = i;
        }
    }

    if (largestGreenIndex != -1)
    {
        leftSign = true;
        // Calculate centroid
        cv::Moments m = cv::moments(greenContours[largestGreenIndex]);
        if (m.m00 != 0)
        {
            leftCentroid.x = static_cast<int>(m.m10 / m.m00);
            leftCentroid.y = static_cast<int>(m.m01 / m.m00);
            // Adjust centroid to full frame coordinates if needed
            // leftCentroid.y += roi.y;
        }
    }

    // Optional: Determine sign position relative to image center
    // int imageCenterX = hsvFrame.cols / 2;
    // if (leftSign && leftCentroid.x > imageCenterX) {
    //     // Green sign is on the right half, might be a false positive or mis-identified
    //     // Could add logic to handle this
    // }
    // if (rightSign && rightCentroid.x < imageCenterX) {
    //     // Red sign is on the left half, might be a false positive or mis-identified
    // }
}

void ImageProcessor::detectEdges(const cv::Mat &frame, float &avgLeftEdgeX, float &avgRightEdgeX)
{
    avgLeftEdgeX = -1.0f;
    avgRightEdgeX = -1.0f;

    // Simplified edge detection using color thresholding on grayscale
    // Assumes white track lines on dark background or distinct edge colors
    // A more robust approach would use Canny edge detection and HoughLinesP

    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 0);

    // Threshold to get binary image (white lines on black)
    // This threshold is critical and environment-dependent
    cv::Mat binaryFrame;
    cv::threshold(grayFrame, binaryFrame, 180, 255, cv::THRESH_BINARY); // Adjust 180

    // Morphological operations to clean up the image
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binaryFrame, binaryFrame, cv::MORPH_CLOSE, kernel);

    // Analyze columns to find edges
    // Sum pixel values in each column
    cv::Mat colSums;
    cv::reduce(binaryFrame, colSums, 0, cv::REDUCE_SUM, CV_32F); // Sum along rows (result is 1xN)

    // Find potential left and right edges
    // A simple approach: find the first and last column with significant sum
    float threshold = binaryFrame.rows * 255 * 0.3f; // 30% of max possible sum for a column
    int leftEdgeX = -1;
    int rightEdgeX = -1;

    // Scan from left for left edge
    for (int i = 0; i < colSums.cols; ++i)
    {
        if (colSums.at<float>(0, i) > threshold)
        {
            leftEdgeX = i;
            break;
        }
    }

    // Scan from right for right edge
    for (int i = colSums.cols - 1; i >= 0; --i)
    {
        if (colSums.at<float>(0, i) > threshold)
        {
            rightEdgeX = i;
            break;
        }
    }

    // Assign results
    if (leftEdgeX != -1)
    {
        avgLeftEdgeX = static_cast<float>(leftEdgeX);
    }
    if (rightEdgeX != -1)
    {
        avgRightEdgeX = static_cast<float>(rightEdgeX);
    }

    // Note: This is a very basic edge detection.
    // For better results, consider:
    // 1. Canny edge detection: cv::Canny(grayFrame, edges, lowThresh, highThresh);
    // 2. Hough Line Transform: cv::HoughLinesP(edges, lines, ...);
    // 3. Fitting lines to the detected edges.
}