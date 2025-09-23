// src/vision/frame_buffer.cpp
// Implementation of FrameBuffer

#include "frame_buffer.h"
#include "../utils/logger.h" // Include the logger for debugging

FrameBuffer::FrameBuffer()
    // CRITICAL: Explicitly initialize the atomic flag to false in the member initializer list
    // This prevents undefined behavior from an uninitialized atomic bool.
    : bufferMutex(), latestFrame(), newFrameFlag(false)
{
    LOG_DEBUG("FrameBuffer: Constructed. newFrameFlag initialized to false.");
    // latestFrame is default-constructed by cv::Mat()
    // newFrameFlag is initialized to false above
}

void FrameBuffer::putFrame(const cv::Mat &frame)
{
    LOG_VERBOSE("FrameBuffer: putFrame called.");
    if (frame.empty())
    {
        LOG_WARN("FrameBuffer: putFrame called with an empty frame. Ignoring.");
        return; // Don't store empty frames
    }

    // --- CRITICAL SECTION: ACCESSING latestFrame ---
    std::lock_guard<std::mutex> lock(bufferMutex);
    // --- OPTIMIZATION: Use assignment instead of clone() ---
    // This leverages cv::Mat's internal reference counting.
    // The pixel data is NOT copied here. latestFrame now shares
    // the data with the input 'frame'.
    latestFrame = frame; // FAST ASSIGNMENT
    LOG_DEBUG("FrameBuffer: putFrame assigned frame (size: " << frame.cols << "x" << frame.rows << ").");

    // Signal that a new (potentially different) frame is available.
    // Release ensures this write is visible to other threads.
    newFrameFlag.store(true, std::memory_order_release);
    LOG_DEBUG("FrameBuffer: putFrame set newFrameFlag to true.");
    // --- END OF CRITICAL SECTION ---
}

bool FrameBuffer::getFrame(cv::Mat &frame)
{
    LOG_VERBOSE("FrameBuffer: getFrame called.");
    // --- CRITICAL SECTION: ACCESSING latestFrame ---
    std::lock_guard<std::mutex> lock(bufferMutex);
    // Acquire ensures we see the latest value written by putFrame's release
    if (newFrameFlag.load(std::memory_order_acquire))
    {
        LOG_DEBUG("FrameBuffer: getFrame found newFrameFlag=true.");
        // --- OPTIMIZATION: Use assignment instead of clone() ---
        // This leverages cv::Mat's internal reference counting.
        // The pixel data is NOT copied here. 'frame' now shares
        // the data with 'latestFrame'.
        frame = latestFrame; // FAST ASSIGNMENT
        LOG_DEBUG("FrameBuffer: getFrame assigned latestFrame to output (size: " << frame.cols << "x" << frame.rows << ").");

        // Mark the frame as consumed.
        // Release ensures this write is visible to other threads.
        newFrameFlag.store(false, std::memory_order_release);
        LOG_DEBUG("FrameBuffer: getFrame set newFrameFlag to false (consumed).");
        // --- END OF CRITICAL SECTION ---
        return true; // Indicate that a frame was successfully retrieved and consumed
    }
    LOG_DEBUG("FrameBuffer: getFrame found newFrameFlag=false (no new frame).");
    // --- END OF CRITICAL SECTION ---
    return false; // No new frame was available
}

bool FrameBuffer::hasNewFrame() const
{
    LOG_VERBOSE("FrameBuffer: hasNewFrame called.");
    // Quick check without locking the mutex for the frame data
    // Acquire ensures we see the latest value written by putFrame's release
    bool available = newFrameFlag.load(std::memory_order_acquire);
    LOG_VERBOSE("FrameBuffer: hasNewFrame returning " << (available ? "true" : "false"));
    return available;
}