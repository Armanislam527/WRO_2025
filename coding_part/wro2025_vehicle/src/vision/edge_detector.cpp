// src/vision/edge_detector.cpp
// Implementation of EdgeDetector for WRO 2025

#include "edge_detector.h"
#include <iostream>
#include <cmath>             // For round, atan2, sqrt etc.
#include <algorithm>         // For std::max, std::min if needed
#include "../utils/logger.h" // Include the logger

// --- CONSTRUCTOR ---
EdgeDetector::EdgeDetector()
    : cannyLowThreshold_(50), cannyHighThreshold_(150),
      houghRho_(1.0), houghTheta_(CV_PI / 180),
      houghThreshold_(50), houghMinLineLength_(30.0), houghMaxLineGap_(10.0),
      useMorphology_(true)
{
    // --- PRE-CREATE MORPHOLOGICAL KERNEL ---
    // Using a standard rectangular kernel for closing/dilation
    morphKernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    LOG_DEBUG("EdgeDetector constructed.");
}

// --- CORE DETECTION FUNCTION ---
void EdgeDetector::detectEdgesAndLines(const cv::Mat &frame, std::vector<cv::Vec4i> &lines)
{
    lines.clear(); // Ensure output vector is clean

    if (frame.empty())
    {
        LOG_WARN("EdgeDetector::detectEdgesAndLines received an empty frame.");
        return; // Nothing to process
    }
    LOG_VERBOSE("EdgeDetector::detectEdgesAndLines processing frame of size " << frame.cols << "x" << frame.rows);

    // --- PREPROCESS THE FRAME ---
    cv::Mat grayFrame;
    preprocessFrame(frame, grayFrame);

    // --- APPLY CANNY EDGE DETECTION ---
    cv::Mat edges;
    cv::Canny(grayFrame, edges, cannyLowThreshold_, cannyHighThreshold_);
    LOG_DEBUG("EdgeDetector: Canny edge detection applied.");

    // --- APPLY MORPHOLOGICAL OPERATIONS (OPTIONAL BUT USEFUL) ---
    if (useMorphology_)
    {
        cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, morphKernel_); // Close small gaps
        // cv::morphologyEx(edges, edges, cv::MORPH_DILATE, morphKernel_); // Optional: thicken lines
        LOG_DEBUG("EdgeDetector: Morphological operations applied.");
    }

    // --- APPLY HOUGH LINE TRANSFORM TO DETECT LINES ---
    cv::HoughLinesP(edges, lines, houghRho_, houghTheta_, houghThreshold_,
                    houghMinLineLength_, houghMaxLineGap_);
    LOG_DEBUG("EdgeDetector: HoughLinesP detected " << lines.size() << " lines.");
    // --- END OF DETECTION ---
}

// --- ANALYZE DETECTED LINES TO FIND AVERAGE EDGE POSITIONS ---
void EdgeDetector::findAverageEdgePositions(const std::vector<cv::Vec4i> &lines, int frameWidth, float &avgLeftEdgeX, float &avgRightEdgeX)
{
    // --- INITIALIZE OUTPUTS ---
    avgLeftEdgeX = -1.0f; // Indicate not found
    avgRightEdgeX = -1.0f;

    if (lines.empty() || frameWidth <= 0)
    {
        LOG_WARN("EdgeDetector::findAverageEdgePositions received empty lines or invalid frame width.");
        return;
    }

    // --- ANALYZE DETECTED LINES TO FIND LEFT AND RIGHT TRACK EDGES ---
    // We'll look for near-vertical lines (angles close to 90 or 270 degrees)
    // and separate them based on their x-coordinate (left half vs right half)

    float leftEdgeSumX = 0.0f;
    int leftEdgeCount = 0;
    float rightEdgeSumX = 0.0f;
    int rightEdgeCount = 0;

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::Point pt1(l[0], l[1]);
        cv::Point pt2(l[2], l[3]);

        // --- CALCULATE ANGLE OF THE LINE ---
        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;

        // --- FILTER FOR NEAR-VERTICAL LINES ---
        // Lines pointing upwards (left edge) or downwards (right edge) on the track
        // Allow a tolerance for slight tilts
        if ((angle > 70 && angle < 110) || (angle > -110 && angle < -70))
        {
            // --- CALCULATE LINE CENTER X COORDINATE ---
            float lineCenterX = (l[0] + l[2]) / 2.0f;
            // float lineCenterY = (l[1] + l[3]) / 2.0f; // Useful for weighting if needed, but currently unused
            // (void)lineCenterY; // Explicitly mark as unused if kept for future use

            // --- SEPARATE BASED ON X-COORDINATE RELATIVE TO FRAME CENTER ---
            if (lineCenterX < frameWidth / 2.0f)
            {
                // Potentially a left edge line
                leftEdgeSumX += lineCenterX;
                leftEdgeCount++;
            }
            else
            {
                // Potentially a right edge line
                rightEdgeSumX += lineCenterX;
                rightEdgeCount++;
            }
            // Optional: Draw lines for debugging (requires access to the frame Mat)
            // cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
    }

    // --- CALCULATE AVERAGE X POSITIONS ---
    if (leftEdgeCount > 0)
    {
        avgLeftEdgeX = leftEdgeSumX / leftEdgeCount;
        LOG_DEBUG("EdgeDetector: Average left edge X calculated: " << avgLeftEdgeX << " (from " << leftEdgeCount << " lines).");
    }
    if (rightEdgeCount > 0)
    {
        avgRightEdgeX = rightEdgeSumX / rightEdgeCount;
        LOG_DEBUG("EdgeDetector: Average right edge X calculated: " << avgRightEdgeX << " (from " << rightEdgeCount << " lines).");
    }

    // --- NOTE ON ROBUSTNESS ---
    // This is a basic averaging approach.
    // For better results, consider:
    // 1. Weighting lines by length or how vertical they are.
    // 2. Clustering lines (e.g., k-means on x-coordinates) to find the most prominent edges.
    // 3. Using the vanishing point concept if perspective is significant.
    // 4. Adaptive thresholding for Canny based on image statistics.
    LOG_VERBOSE("EdgeDetector::findAverageEdgePositions completed.");
}
// --- END OF PUBLIC INTERFACE FUNCTIONS ---

// --- PRIVATE HELPER FUNCTIONS ---

// --- PREPROCESS FRAME FOR EDGE DETECTION ---
void EdgeDetector::preprocessFrame(const cv::Mat &inputFrame, cv::Mat &outputGray)
{
    if (inputFrame.empty())
    {
        LOG_WARN("EdgeDetector::preprocessFrame received an empty input frame.");
        outputGray = cv::Mat();
        return;
    }

    // --- CONVERT TO GRAYSCALE IF NEEDED ---
    if (inputFrame.channels() == 3)
    {
        cv::cvtColor(inputFrame, outputGray, cv::COLOR_BGR2GRAY);
    }
    else if (inputFrame.channels() == 1)
    {
        outputGray = inputFrame; // Already grayscale
    }
    else
    {
        LOG_WARN("EdgeDetector::preprocessFrame: Unsupported number of channels (" << inputFrame.channels() << "). Converting anyway.");
        cv::cvtColor(inputFrame, outputGray, cv::COLOR_BGR2GRAY);
    }
    LOG_DEBUG("EdgeDetector::preprocessFrame: Converted to grayscale (size: " << outputGray.cols << "x" << outputGray.rows << ").");

    // --- APPLY GAUSSIAN BLUR TO REDUCE NOISE ---
    // This is crucial for effective Canny edge detection
    cv::GaussianBlur(outputGray, outputGray, cv::Size(5, 5), 0);
    LOG_DEBUG("EdgeDetector::preprocessFrame: Applied Gaussian blur.");
}
// --- END OF EdgeDetector::preprocessFrame ---