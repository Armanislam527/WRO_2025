// vision/frame_buffer.h
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
    // Returns true if a frame was available
    // 'frame' will contain a copy of the latest frame
    bool getFrame(cv::Mat &frame);

    // Check if a new frame is available
    bool hasNewFrame() const;

private:
    mutable std::mutex bufferMutex; // Protects access to the frame
    cv::Mat latestFrame;
    std::atomic<bool> newFrameFlag{false}; // Atomic flag for quick check
};

#endif // FRAME_BUFFER_H