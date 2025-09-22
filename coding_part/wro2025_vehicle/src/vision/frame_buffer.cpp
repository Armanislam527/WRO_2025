// vision/frame_buffer.cpp
// Implementation of FrameBuffer

#include "frame_buffer.h"
#include "../utils/logger.h" // For logging if needed
FrameBuffer::FrameBuffer()
{
    latestFrame = cv::Mat();
    LOG_DEBUG("FrameBuffer constructed.");
    // Constructor - atomic flag is initialized in the member initializer list
}

void FrameBuffer::putFrame(const cv::Mat &frame)
{
    if (frame.empty())
    {
        LOG_WARN("putFrame called with an empty frame. Ignoring.");
        return; // Don't store empty frames
    }
    std::lock_guard<std::mutex> lock(bufferMutex);
    latestFrame = frame;                                 // Store a copy
    newFrameFlag.store(true, std::memory_order_release); // Signal new frame is available
    LOG_DEBUG("putFrame: Stored new frame (size: " << frame.cols << "x" << frame.rows << "). Flag set.");
}

bool FrameBuffer::getFrame(cv::Mat &frame)
{
    std::lock_guard<std::mutex> lock(bufferMutex);
    if (newFrameFlag.load(std::memory_order_acquire))
    {
        frame = latestFrame;                                  // Provide a copy to the consumer
        newFrameFlag.store(false, std::memory_order_release); // Mark frame as consumed
        LOG_DEBUG("getFrame: Retrieved frame (size: " << frame.cols << "x" << frame.rows << "). Flag cleared.");
        return true;
    }
    return false; // No new frame was available
}

bool FrameBuffer::hasNewFrame() const
{
    // Quick check without locking the mutex for the frame data
    bool available = newFrameFlag.load(std::memory_order_acquire);
    // LOG_VERBOSE("hasNewFrame: " << (available ? "true" : "false")); // Very frequent
    return available;
}