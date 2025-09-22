// src/vision/edge_detector.h
// Dedicated class for detecting track edges using Canny + HoughLinesP

#ifndef EDGE_DETECTOR_H
#define EDGE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector> // For std::vector<cv::Vec4i>

class EdgeDetector
{
public:
    EdgeDetector();
    ~EdgeDetector() = default;

    // --- Detection Parameters ---
    void setCannyThresholds(int low, int high)
    {
        cannyLowThreshold_ = low;
        cannyHighThreshold_ = high;
    }
    void setHoughParams(double rho, double theta, int threshold, double minLineLength, double maxLineGap)
    {
        houghRho_ = rho;
        houghTheta_ = theta;
        houghThreshold_ = threshold;
        houghMinLineLength_ = minLineLength;
        houghMaxLineGap_ = maxLineGap;
    }
    void setMorphologyEnabled(bool enable) { useMorphology_ = enable; }

    // Detect edges and lines in a BGR or Grayscale frame
    // lines: Output, vector of detected lines (cv::Vec4i: x1, y1, x2, y2)
    void detectEdgesAndLines(const cv::Mat &frame, std::vector<cv::Vec4i> &lines);

    // Analyze detected lines to find average left and right edge X positions
    // frameWidth: Width of the input frame (needed for center calculation)
    // avgLeftEdgeX, avgRightEdgeX: Output, average X positions (-1.0 if not found)
    void findAverageEdgePositions(const std::vector<cv::Vec4i> &lines, int frameWidth, float &avgLeftEdgeX, float &avgRightEdgeX);

private:
    // --- Canny Parameters ---
    int cannyLowThreshold_;
    int cannyHighThreshold_;

    // --- HoughLinesP Parameters ---
    double houghRho_;
    double houghTheta_;
    int houghThreshold_;
    double houghMinLineLength_;
    double houghMaxLineGap_;

    // --- Morphology ---
    bool useMorphology_;
    cv::Mat morphKernel_;

    // --- Internal Helpers ---
    void preprocessFrame(const cv::Mat &inputFrame, cv::Mat &outputGray);
};

#endif // EDGE_DETECTOR_H