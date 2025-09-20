// vision/frame_buffer.cpp
// Implementation of FrameBuffer

#include "frame_buffer.h"

FrameBuffer::FrameBuffer()
{
    // Constructor - atomic flag is initialized in the member initializer list
}

void FrameBuffer::putFrame(const cv::Mat &frame)
{
    if (frame.empty())
    {
        return; // Don't store empty frames
    }
    std::lock_guard<std::mutex> lock(bufferMutex);
    latestFrame = frame.clone();                         // Store a copy
    newFrameFlag.store(true, std::memory_order_release); // Signal new frame is available
}

bool FrameBuffer::getFrame(cv::Mat &frame)
{
    std::lock_guard<std::mutex> lock(bufferMutex);
    if (newFrameFlag.load(std::memory_order_acquire))
    {
        frame = latestFrame.clone();                          // Provide a copy to the consumer
        newFrameFlag.store(false, std::memory_order_release); // Mark frame as consumed
        return true;
    }
    return false; // No new frame was available
}

bool FrameBuffer::hasNewFrame() const
{
    // Quick check without locking the mutex for the frame data
    return newFrameFlag.load(std::memory_order_acquire);
}