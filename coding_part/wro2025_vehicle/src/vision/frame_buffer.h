// src/vision/frame_buffer.h
// A simple thread-safe buffer to hold the latest frame for processing

#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>

class FrameBuffer
{
public:
    FrameBuffer();
    ~FrameBuffer() = default;

    // Put a new frame into the buffer (Producer)
    // Uses move semantics for efficiency if possible, otherwise copies
    void putFrame(const cv::Mat &frame);

    // Get the latest frame from the buffer (Consumer)
    // Returns true if a frame was available and successfully copied
    // 'frame' will contain a copy of the latest frame
    bool getFrame(cv::Mat &frame);

    // Check if a new frame is available (Quick check)
    // Returns true if a new frame has been put but not yet gotten
    bool hasNewFrame() const;

private:
    mutable std::mutex bufferMutex; // Protects access to the frame data (latestFrame)
    cv::Mat latestFrame;            // The actual frame data

    // Atomic flag for signaling new frame availability
    // Declared mutable to allow modification in const member function hasNewFrame
    mutable std::atomic<bool> newFrameFlag{false};
    // ^^^ CRITICAL: Direct initialization to false in member initializer list
    // This ensures it starts in a known state, preventing perpetual 'true' returns.
};

#endif // FRAME_BUFFER_H