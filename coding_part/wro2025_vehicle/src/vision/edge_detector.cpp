// src/vision/edge_detector.cpp
// Implementation of EdgeDetector

#include "edge_detector.h"
#include <cmath> // For M_PI, atan2, fabs
// #include <iostream> // Optional for debugging

EdgeDetector::EdgeDetector()
    : cannyLowThreshold_(50), cannyHighThreshold_(150),
      houghRho_(1.0), houghTheta_(CV_PI / 180),
      houghThreshold_(50), houghMinLineLength_(30.0), houghMaxLineGap_(10.0),
      useMorphology_(true)
{
    // Pre-create morphological kernel
    morphKernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
}

void EdgeDetector::preprocessFrame(const cv::Mat &inputFrame, cv::Mat &outputGray)
{
    if (inputFrame.channels() == 3)
    {
        cv::cvtColor(inputFrame, outputGray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        outputGray = inputFrame;
    }
    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(outputGray, outputGray, cv::Size(5, 5), 0);
}

void EdgeDetector::detectEdgesAndLines(const cv::Mat &frame, std::vector<cv::Vec4i> &lines)
{
    lines.clear(); // Ensure output vector is clean

    if (frame.empty())
    {
        return; // Nothing to process
    }

    cv::Mat grayFrame;
    preprocessFrame(frame, grayFrame);

    // --- Use Canny Edge Detection ---
    cv::Mat edges;
    cv::Canny(grayFrame, edges, cannyLowThreshold_, cannyHighThreshold_);

    // --- Use Morphological Operations to strengthen lines (optional) ---
    if (useMorphology_)
    {
        cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, morphKernel_);
        // cv::morphologyEx(edges, edges, cv::MORPH_DILATE, morphKernel_); // Optional: thicken lines
    }

    // --- Use Hough Line Transform to detect lines ---
    cv::HoughLinesP(edges, lines, houghRho_, houghTheta_, houghThreshold_,
                    houghMinLineLength_, houghMaxLineGap_);
}

void EdgeDetector::findAverageEdgePositions(const std::vector<cv::Vec4i> &lines, int frameWidth, float &avgLeftEdgeX, float &avgRightEdgeX)
{
    avgLeftEdgeX = -1.0f; // Indicate not found
    avgRightEdgeX = -1.0f;

    float leftEdgeSumX = 0.0f;
    int leftEdgeCount = 0;
    float rightEdgeSumX = 0.0f;
    int rightEdgeCount = 0;

    const float frameCenterX = static_cast<float>(frameWidth) / 2.0f;

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::Point pt1(l[0], l[1]);
        cv::Point pt2(l[2], l[3]);

        // Calculate angle of the line
        double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;

        // Filter for near-vertical lines (allowing some tilt)
        // Lines pointing upwards (left edge) or downwards (right edge) on the track
        // Adjust angle thresholds if needed.
        if ((angle > 70 && angle < 110) || (angle > -110 && angle < -70))
        {
            float lineCenterX = (l[0] + l[2]) / 2.0f;
            // float lineCenterY = (l[1] + l[3]) / 2.0f; // Useful for weighting if needed

            // Separate based on x-coordinate relative to frame center
            if (lineCenterX < frameCenterX)
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

    // Calculate average X positions
    if (leftEdgeCount > 0)
    {
        avgLeftEdgeX = leftEdgeSumX / leftEdgeCount;
    }
    if (rightEdgeCount > 0)
    {
        avgRightEdgeX = rightEdgeSumX / rightEdgeCount;
    }
}