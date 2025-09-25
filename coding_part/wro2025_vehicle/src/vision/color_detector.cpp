// src/vision/color_detector.cpp
// Implementation of ColorDetector for WRO 2025

#include "color_detector.h"
#include <iostream>
#include <cmath>             // For round, sqrt etc.
#include <algorithm>         // For std::max, std::min if needed
#include "../utils/logger.h" // Include the logger

// --- CONSTRUCTOR ---
ColorDetector::ColorDetector()
    : minAreaThreshold_(100.0), minAspectRatio_(0.3f), maxAspectRatio_(3.0f),
      minSolidity_(0.5f), minExtent_(0.2f)
{
    // --- DEFAULT HSV RANGES ---
    // Red HSV (wraps around 0/180)
    // These ranges are starting points and WILL need tuning based on actual lighting/camera
    redLower1_ = cv::Scalar(0, 100, 100);
    redUpper1_ = cv::Scalar(10, 255, 255);
    redLower2_ = cv::Scalar(170, 100, 100);
    redUpper2_ = cv::Scalar(180, 255, 255);
    // Green HSV
    greenLower_ = cv::Scalar(40, 50, 50);
    greenUpper_ = cv::Scalar(80, 255, 255);

    // --- PRE-CREATE MORPHOLOGICAL KERNEL ---
    // Using a slightly larger kernel for potentially better noise reduction
    morphKernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    LOG_DEBUG("ColorDetector constructed.");
}

// --- SETTERS FOR HSV RANGES ---
void ColorDetector::setRedHSVRange(cv::Scalar lower1, cv::Scalar upper1, cv::Scalar lower2, cv::Scalar upper2)
{
    redLower1_ = lower1;
    redUpper1_ = upper1;
    redLower2_ = lower2;
    redUpper2_ = upper2;
    LOG_DEBUG("ColorDetector: Red HSV ranges updated.");
}

void ColorDetector::setGreenHSVRange(cv::Scalar lower, cv::Scalar upper)
{
    greenLower_ = lower;
    greenUpper_ = upper;
    LOG_DEBUG("ColorDetector: Green HSV range updated.");
}

// --- CORE DETECTION FUNCTION ---
void ColorDetector::detectSigns(const cv::Mat &hsvFrame, bool &leftSign, bool &rightSign, cv::Point &leftCentroid, cv::Point &rightCentroid)
{
    // --- INITIALIZE OUTPUTS ---
    leftSign = false;
    rightSign = false;
    leftCentroid = cv::Point(-1, -1);
    rightCentroid = cv::Point(-1, -1);

    if (hsvFrame.empty())
    {
        LOG_WARN("ColorDetector::detectSigns received an empty HSV frame.");
        return; // Nothing to process
    }
    LOG_VERBOSE("ColorDetector::detectSigns processing frame of size " << hsvFrame.cols << "x" << hsvFrame.rows);

    // --- 1. DETECT RED TRAFFIC SIGN (RIGHT LANE) ---
    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(hsvFrame, redLower1_, redUpper1_, redMask1);
    cv::inRange(hsvFrame, redLower2_, redUpper2_, redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);

    // --- NOISE REDUCTION FOR RED MASK ---
    // Slightly stronger morphological operations
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, morphKernel_);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, morphKernel_);
    cv::morphologyEx(redMask, redMask, cv::MORPH_DILATE, morphKernel_); // Slight dilation to connect nearby pixels

    // --- FIND AND FILTER RED CONTOURS ---
    std::vector<std::vector<cv::Point>> redContours;
    cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestRedArea = 0;
    int bestRedIndex = -1;
    for (size_t i = 0; i < redContours.size(); i++)
    {
        double area = cv::contourArea(redContours[i]);
        if (area > minAreaThreshold_) // Apply area threshold
        {
            // --- APPLY ADDITIONAL FILTERING FOR ROBUSTNESS ---
            // 1. Aspect Ratio (closer to 1 for circle/square)
            cv::Rect boundingRect = cv::boundingRect(redContours[i]);
            float aspectRatio = static_cast<float>(boundingRect.width) / boundingRect.height;
            if (aspectRatio < minAspectRatio_ || aspectRatio > maxAspectRatio_)
                continue; // Filter extreme ratios

            // 2. Extent (Ratio of contour area to bounding rectangle area)
            double rectArea = boundingRect.area();
            double extent = area / rectArea;
            if (extent < minExtent_)
                continue; // Filter very sparse shapes

            // 3. Solidity (Ratio of contour area to its convex hull area)
            std::vector<cv::Point> hull;
            cv::convexHull(redContours[i], hull);
            double hullArea = cv::contourArea(hull);
            // Avoid division by zero
            if (hullArea > 0)
            {
                double solidity = area / hullArea;
                if (solidity < minSolidity_)
                    continue; // Filter shapes with large holes or concavities
            }
            else if (minSolidity_ > 0)
            {
                continue; // If hullArea is 0 but minSolidity > 0, reject
            }

            // If it passes filters, check if it's the largest so far
            if (area > bestRedArea)
            {
                bestRedArea = area;
                bestRedIndex = i;
            }
        }
    }

    if (bestRedIndex != -1)
    {
        rightSign = true;
        // --- CALCULATE CENTROID USING MOMENTS ---
        cv::Moments m = cv::moments(redContours[bestRedIndex]);
        if (m.m00 != 0)
        {
            rightCentroid.x = static_cast<int>(m.m10 / m.m00);
            rightCentroid.y = static_cast<int>(m.m01 / m.m00);
            // Note: Centroid is relative to the input hsvFrame ROI.
            // If full-frame coordinates are needed later, adjust by adding roi.x, roi.y.
        }
        LOG_DEBUG("ColorDetector: Red sign detected (area: " << bestRedArea << ", centroid: (" << rightCentroid.x << ", " << rightCentroid.y << ")).");
    }

    // --- 2. DETECT GREEN TRAFFIC SIGN (LEFT LANE) ---
    cv::Mat greenMask;
    cv::inRange(hsvFrame, greenLower_, greenUpper_, greenMask);

    // --- NOISE REDUCTION FOR GREEN MASK ---
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_OPEN, morphKernel_);
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_CLOSE, morphKernel_);
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_DILATE, morphKernel_); // Slight dilation

    // --- FIND AND FILTER GREEN CONTOURS ---
    std::vector<std::vector<cv::Point>> greenContours;
    cv::findContours(greenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestGreenArea = 0;
    int bestGreenIndex = -1;
    for (size_t i = 0; i < greenContours.size(); i++)
    {
        double area = cv::contourArea(greenContours[i]);
        if (area > minAreaThreshold_) // Apply area threshold
        {
            // --- APPLY ADDITIONAL FILTERING FOR ROBUSTNESS ---
            // 1. Aspect Ratio
            cv::Rect boundingRect = cv::boundingRect(greenContours[i]);
            float aspectRatio = static_cast<float>(boundingRect.width) / boundingRect.height;
            if (aspectRatio < minAspectRatio_ || aspectRatio > maxAspectRatio_)
                continue;

            // 2. Extent
            double rectArea = boundingRect.area();
            double extent = area / rectArea;
            if (extent < minExtent_)
                continue;

            // 3. Solidity
            std::vector<cv::Point> hull;
            cv::convexHull(greenContours[i], hull);
            double hullArea = cv::contourArea(hull);
            // Avoid division by zero
            if (hullArea > 0)
            {
                double solidity = area / hullArea;
                if (solidity < minSolidity_)
                    continue;
            }
            else if (minSolidity_ > 0)
            {
                continue; // If hullArea is 0 but minSolidity > 0, reject
            }

            // If it passes filters, check if it's the largest so far
            if (area > bestGreenArea)
            {
                bestGreenArea = area;
                bestGreenIndex = i;
            }
        }
    }

    if (bestGreenIndex != -1)
    {
        leftSign = true;
        // --- CALCULATE CENTROID USING MOMENTS ---
        cv::Moments m = cv::moments(greenContours[bestGreenIndex]);
        if (m.m00 != 0)
        {
            leftCentroid.x = static_cast<int>(m.m10 / m.m00);
            leftCentroid.y = static_cast<int>(m.m01 / m.m00);
            // Note: Centroid is relative to the input hsvFrame ROI.
        }
        LOG_DEBUG("ColorDetector: Green sign detected (area: " << bestGreenArea << ", centroid: (" << leftCentroid.x << ", " << leftCentroid.y << ")).");
    }

    // --- OPTIONAL: RESOLVE CONFLICTS IF BOTH DETECTED ---
    // Define a minimum expected distance between signs (adjust based on testing)
    const double minExpectedSignDistance = 50.0; // Example value in pixels

    if (leftSign && rightSign)
    {
        double distance = cv::norm(leftCentroid - rightCentroid);
        if (distance < minExpectedSignDistance)
        {
            LOG_WARN("ColorDetector: Sign conflict detected (distance: " << distance << "). Resolving...");
            // Likely a false positive, choose the one with larger area or better solidity
            // Simple heuristic: Prefer left sign (arbitrary, or based on last known state)
            // For now, let's disable the right sign.
            rightSign = false;
            rightCentroid = cv::Point(-1, -1); // Invalidate centroid
            LOG_WARN("ColorDetector: Conflict resolved. Disabled right sign.");
        }
    }
    // --- END OF SIGN DETECTION ---
    LOG_VERBOSE("ColorDetector::detectSigns completed.");
}

bool ColorDetector::filterContour(const std::vector<cv::Point> &contour, const cv::Mat &hsvFrame)
{
    // Explicitly mark the parameter as intentionally unused to silence warning
    (void)hsvFrame;

    double area = cv::contourArea(contour);
    if (area < minAreaThreshold_)
        return false;

    // 1. Aspect Ratio (closer to 1 for circle/square)
    cv::Rect boundingRect = cv::boundingRect(contour);
    float aspectRatio = (float)boundingRect.width / boundingRect.height;
    if (aspectRatio < minAspectRatio_ || aspectRatio > maxAspectRatio_)
        return false;

    // 2. Extent (Ratio of contour area to bounding rectangle area)
    double rectArea = boundingRect.area();
    double extent = area / rectArea;
    if (extent < minExtent_)
        return false;

    // 3. Solidity (Ratio of contour area to its convex hull area)
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hullArea = cv::contourArea(hull);
    // Avoid division by zero, though hullArea should be >= area
    if (hullArea > 0)
    {
        double solidity = area / hullArea;
        if (solidity < minSolidity_)
            return false;
    }
    else if (minSolidity_ > 0)
    {
        return false; // If hullArea is 0 but minSolidity > 0, reject
    }

    return true; // Passed all filters
}