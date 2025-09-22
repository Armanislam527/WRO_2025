// src/vision/color_detector.cpp
// Implementation of ColorDetector

#include "color_detector.h"
#include <iostream> // For optional debugging prints

ColorDetector::ColorDetector()
    : minAreaThreshold_(100.0), minAspectRatio_(0.3f), maxAspectRatio_(3.0f),
      minSolidity_(0.5f), minExtent_(0.2f)
{
    // Default HSV ranges (from previous ImageProcessor logic)
    // Red HSV (wraps around 0/180)
    redLower1_ = cv::Scalar(0, 100, 100);
    redUpper1_ = cv::Scalar(10, 255, 255);
    redLower2_ = cv::Scalar(170, 100, 100);
    redUpper2_ = cv::Scalar(180, 255, 255);
    // Green HSV
    greenLower_ = cv::Scalar(40, 50, 50);
    greenUpper_ = cv::Scalar(80, 255, 255);

    // Pre-create morphological kernel
    morphKernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
}

void ColorDetector::setRedHSVRange(cv::Scalar lower1, cv::Scalar upper1, cv::Scalar lower2, cv::Scalar upper2)
{
    redLower1_ = lower1;
    redUpper1_ = upper1;
    redLower2_ = lower2;
    redUpper2_ = upper2;
}

void ColorDetector::setGreenHSVRange(cv::Scalar lower, cv::Scalar upper)
{
    greenLower_ = lower;
    greenUpper_ = upper;
}

bool ColorDetector::detectRedSign(const cv::Mat &hsvFrame, cv::Point &centroid)
{
    centroid = cv::Point(-1, -1); // Initialize to invalid

    // --- Detect Red ---
    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(hsvFrame, redLower1_, redUpper1_, redMask1);
    cv::inRange(hsvFrame, redLower2_, redUpper2_, redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);

    // Noise reduction
    cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, morphKernel_);
    cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, morphKernel_);
    cv::morphologyEx(redMask, redMask, cv::MORPH_DILATE, morphKernel_); // Slight dilation

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the best red contour (largest valid one)
    double bestArea = 0;
    int bestIndex = -1;
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (filterContour(contours[i], hsvFrame))
        {
            double area = cv::contourArea(contours[i]);
            if (area > bestArea)
            {
                bestArea = area;
                bestIndex = i;
            }
        }
    }

    if (bestIndex != -1)
    {
        // Calculate centroid using moments
        cv::Moments m = cv::moments(contours[bestIndex]);
        if (m.m00 != 0)
        {
            centroid.x = static_cast<int>(m.m10 / m.m00);
            centroid.y = static_cast<int>(m.m01 / m.m00);
        }
        return true; // Valid red sign found
    }
    return false; // No valid red sign found
}

bool ColorDetector::detectGreenSign(const cv::Mat &hsvFrame, cv::Point &centroid)
{
    centroid = cv::Point(-1, -1); // Initialize to invalid

    // --- Detect Green ---
    cv::Mat greenMask;
    cv::inRange(hsvFrame, greenLower_, greenUpper_, greenMask);

    // Noise reduction
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_OPEN, morphKernel_);
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_CLOSE, morphKernel_);
    cv::morphologyEx(greenMask, greenMask, cv::MORPH_DILATE, morphKernel_); // Slight dilation

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(greenMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the best green contour (largest valid one)
    double bestArea = 0;
    int bestIndex = -1;
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (filterContour(contours[i], hsvFrame))
        {
            double area = cv::contourArea(contours[i]);
            if (area > bestArea)
            {
                bestArea = area;
                bestIndex = i;
            }
        }
    }

    if (bestIndex != -1)
    {
        // Calculate centroid using moments
        cv::Moments m = cv::moments(contours[bestIndex]);
        if (m.m00 != 0)
        {
            centroid.x = static_cast<int>(m.m10 / m.m00);
            centroid.y = static_cast<int>(m.m01 / m.m00);
        }
        return true; // Valid green sign found
    }
    return false; // No valid green sign found
}

bool ColorDetector::filterContour(const std::vector<cv::Point> &contour, const cv::Mat &hsvFrame)
{
    (void)hsvFrame;
    double area = cv::contourArea(contour);
    if (area < minAreaThreshold_)
        return false;

    // 1. Aspect Ratio (closer to 1 for circle/square)
    cv::Rect boundingRect = cv::boundingRect(contour);
    float aspectRatio = static_cast<float>(boundingRect.width) / boundingRect.height;
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