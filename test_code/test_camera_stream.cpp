// test_camera_stream.cpp
// Minimal test to verify rpicam-vid stream access and basic OpenCV decoding.

#include <iostream>
#include <cstdio>             // For popen, pclose, fread
#include <vector>             // For storing JPEG data
#include <opencv2/opencv.hpp> // For cv::imdecode, cv::Mat
#include <chrono>             // For timing
#include <thread>             // For sleep

int main()
{
    const std::string camera_command =
        "rpicam-vid --inline --timeout 0 --nopreview --codec mjpeg --width 320 --height 240 --framerate 10 --output -";

    const size_t BUFFER_SIZE = 1024 * 1024; // 1MB read buffer
    std::vector<uint8_t> read_buffer(BUFFER_SIZE);
    std::vector<uint8_t> jpeg_data; // To accumulate a single JPEG frame

    // --- 1. Launch rpicam-vid ---
    std::cout << "Attempting to launch rpicam-vid..." << std::endl;
    std::cout << "Command: " << camera_command << std::endl;
    FILE *camera_process = popen(camera_command.c_str(), "r");
    if (!camera_process)
    {
        std::perror("Failed to start rpicam-vid process");
        return 1;
    }
    std::cout << "rpicam-vid process started successfully." << std::endl;

    // Record start time for timeout
    auto start_time = std::chrono::steady_clock::now();

    // --- 2. State machine for JPEG parsing ---
    enum class JpegState
    {
        FIND_SOI,
        ACCUMULATE
    };
    JpegState state = JpegState::FIND_SOI;
    uint8_t previous_byte = 0;
    // Remove unused marker arrays
    // const uint8_t SOI_MARKER[] = {0xFF, 0xD8};
    // const uint8_t EOI_MARKER[] = {0xFF, 0xD9};
    bool frame_found = false;
    int frames_decoded = 0;
    const int MAX_FRAMES_TO_DECODE = 3; // Try to decode this many frames

    std::cout << "Starting to read stream and parse JPEG frames..." << std::endl;

    // --- 3. Main read/parse loop ---
    while (!frame_found && frames_decoded < MAX_FRAMES_TO_DECODE)
    {
        // Check for timeout (e.g., 10 seconds)
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (duration.count() > 10)
        {
            std::cout << "Timeout (10s) reached. Stopping." << std::endl;
            break;
        }

        size_t bytes_read = fread(read_buffer.data(), sizeof(uint8_t), BUFFER_SIZE, camera_process);

        if (bytes_read == 0)
        {
            if (feof(camera_process))
            {
                std::cout << "rpicam-vid stream ended (EOF)." << std::endl;
                break;
            }
            if (ferror(camera_process))
            {
                std::perror("Error reading from rpicam-vid stream");
                break;
            }
            // Small sleep if no data
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::cout << "Read " << bytes_read << " bytes from stream." << std::endl;

        // --- 4. Process the chunk of data ---
        for (size_t i = 0; i < bytes_read; ++i)
        {
            uint8_t current_byte = read_buffer[i]; // FIX 1: Use correct variable name
            // std::cout << "Processing byte " << i << ", value 0x" << std::hex << static_cast<int>(current_byte) << std::dec << std::endl; // Very verbose

            switch (state)
            {
            case JpegState::FIND_SOI:
                // std::cout << "State: FIND_SOI" << std::endl; // Less verbose
                // Look for SOI marker (0xFFD8)
                if (previous_byte == 0xFF && current_byte == 0xD8)
                {
                    // std::cout << "Found SOI (0xFFD8)" << std::endl; // Less verbose
                    jpeg_data.clear();
                    jpeg_data.push_back(0xFF);
                    jpeg_data.push_back(0xD8);
                    state = JpegState::ACCUMULATE;
                }
                // Update previous byte for next iteration
                previous_byte = current_byte;
                break;

            case JpegState::ACCUMULATE:
                // std::cout << "State: ACCUMULATE (size: " << jpeg_data.size() << ")" << std::endl; // Less verbose
                jpeg_data.push_back(current_byte);

                // Check for EOI marker (0xFFD9)
                if (jpeg_data.size() >= 2 &&
                    jpeg_data[jpeg_data.size() - 2] == 0xFF &&
                    jpeg_data[jpeg_data.size() - 1] == 0xD9)
                {
                    // std::cout << "Found EOI (0xFFD9). JPEG size: " << jpeg_data.size() << " bytes." << std::endl; // Less verbose
                    state = JpegState::FIND_SOI; // Reset state for next frame

                    // --- PROCESS COMPLETE JPEG ---
                    {
                        // Create cv::Mat header for the JPEG data
                        cv::Mat jpeg_mat(1, jpeg_data.size(), CV_8UC1, jpeg_data.data());
                        // Use cv::imdecode to decode the JPEG data into a BGR Mat
                        cv::Mat frame = cv::imdecode(jpeg_mat, cv::IMREAD_COLOR);

                        if (frame.empty())
                        {
                            std::cout << "Warning: Failed to decode JPEG frame (size: " << jpeg_data.size() << " bytes)." << std::endl; // FIX 2, 3: Use std::cout
                        }
                        else
                        {
                            std::cout << "Info: Successfully decoded JPEG frame (size: " << frame.cols << "x" << frame.rows << ")." << std::endl; // FIX 2, 3: Use std::cout
                            frames_decoded++;

                            // Save the frame to verify
                            std::string filename = "test_frame_decoded_" + std::to_string(frames_decoded) + ".jpg";
                            if (cv::imwrite(filename, frame))
                            {
                                std::cout << "Saved decoded frame as '" << filename << "'." << std::endl;
                            }
                            else
                            {
                                std::cout << "Failed to save decoded frame as '" << filename << "'." << std::endl;
                            }

                            if (frames_decoded >= 1)
                            { // Consider success after 1 frame
                                frame_found = true;
                            }
                        }
                    }
                    // Reset for the next frame
                    jpeg_data.clear();
                    previous_byte = 0; // Reset search
                }
                // Always update previous byte in ACCUMULATE state
                previous_byte = current_byte;
                break;
            }
        } // End for processing bytes
    } // End while loop

    // --- 5. Cleanup ---
    std::cout << "Closing rpicam-vid process..." << std::endl;
    int result = pclose(camera_process);
    if (result == -1)
    {
        std::perror("Error closing rpicam-vid process");
    }
    else
    {
        std::cout << "rpicam-vid process closed (exit code: " << result << ")." << std::endl;
    }

    // --- 6. Report final status ---
    if (frame_found && frames_decoded > 0)
    {
        std::cout << "\n*** SUCCESS: Camera stream accessed and " << frames_decoded << " frame(s) decoded. ***" << std::endl;
        std::cout << "This indicates the basic camera pipeline (rpicam-vid -> C++ -> OpenCV) works." << std::endl;
        return 0; // Success
    }
    else
    {
        std::cout << "\n*** FAILURE: Could not access camera stream or decode a frame within timeout. ***" << std::endl;
        std::cout << "Check for:" << std::endl;
        std::cout << "  - Conflicting camera processes (use 'ps aux | grep rpicam' and 'pkill -f rpicam')." << std::endl;
        std::cout << "  - Camera hardware connection." << std::endl;
        std::cout << "  - Correct rpicam-vid command and permissions." << std::endl;
        std::cout << "  - OpenCV installation (cv::imdecode)." << std::endl;
        std::cout << "  - rpicam-vid command output (run it manually)." << std::endl;
        return 1; // Failure
    }
}