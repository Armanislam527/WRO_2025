// src/vision/color_detector.h
// Dedicated class for detecting Red/Green traffic sign pillars

#ifndef COLOR_DETECTOR_H
#define COLOR_DETECTOR_H

#include <opencv2/opencv.hpp>

class ColorDetector
{
public:
    ColorDetector();
    ~ColorDetector() = default;

    // Configure HSV ranges for Red (wraps around in HSV)
    void setRedHSVRange(cv::Scalar lower1, cv::Scalar upper1, cv::Scalar lower2, cv::Scalar upper2);
    // Configure HSV range for Green
    void setGreenHSVRange(cv::Scalar lower, cv::Scalar upper);

    // --- CORRECTED: Combined detection method ---
    // Detect both Red and Green signs in an HSV image
    // leftSign, rightSign: Output, true if sign detected
    // leftCentroid, rightCentroid: Output, center point of detected signs (-1,-1 if not found)
    void detectSigns(const cv::Mat &hsvFrame, bool &leftSign, bool &rightSign, cv::Point &leftCentroid, cv::Point &rightCentroid);
    // --- END OF CORRECTION ---

    // --- Advanced Filtering Parameters (can be tuned) ---
    void setMinAreaThreshold(double area) { minAreaThreshold_ = area; }
    void setMinAspectRatio(float ratio) { minAspectRatio_ = ratio; }
    void setMaxAspectRatio(float ratio) { maxAspectRatio_ = ratio; }
    void setMinSolidity(float solidity) { minSolidity_ = solidity; }
    void setMinExtent(float extent) { minExtent_ = extent; }

private:
    // HSV ranges for Red (two parts due to wrap-around)
    cv::Scalar redLower1_, redUpper1_;
    cv::Scalar redLower2_, redUpper2_;
    // HSV range for Green
    cv::Scalar greenLower_, greenUpper_;

    // --- Filtering Parameters ---
    double minAreaThreshold_; // Minimum contour area
    float minAspectRatio_;    // Width/Height min
    float maxAspectRatio_;    // Width/Height max
    float minSolidity_;       // Area / Convex Hull Area min
    float minExtent_;         // Area / Bounding Rect Area min

    // Internal helper for contour filtering
    bool filterContour(const std::vector<cv::Point> &contour, const cv::Mat &hsvFrame);

    // Morphological kernel (reused)
    cv::Mat morphKernel_;
};

#endif // COLOR_DETECTOR_H