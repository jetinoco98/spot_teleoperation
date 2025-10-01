#include "StreamCapture.hpp"
#include <iostream>
#include <chrono>
#include <mutex>

// ============================================================================
//                            Constructor/Destructor
// ============================================================================

StreamCapture::StreamCapture(const std::string& stream_address) : stop_capture_(false) {
    // Build the pipeline string
    pipeline_ = "rtspsrc protocols=tcp location=" + stream_address + " latency=0 drop-on-latency=true "
    "! rtph264depay ! h264parse ! decodebin ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink sync=false max-buffers=1 drop=true";
}

StreamCapture::~StreamCapture() {
    try {
        stop();
    } catch (const std::exception& e) {
        std::cerr << "Failed to cleanup StreamCapture: " << e.what() << std::endl;
    }
}


// ============================================================================
//                                 Public Methods
// ============================================================================

bool StreamCapture::start() {
    cv_capture_.open(pipeline_, cv::CAP_GSTREAMER);
    if (!cv_capture_.isOpened()) {
        std::cerr << "Error: Unable to open video stream." << std::endl;
        return false;
    }

    // Configure OpenCV VideoCapture for low latency
    cv_capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Minimize buffer size

    // Get the width and height of the video stream
    width = static_cast<int>(cv_capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(cv_capture_.get(cv::CAP_PROP_FRAME_HEIGHT));

    capture_thread_ = std::thread(&StreamCapture::captureLoop, this);
    return true;
}


// ============================================================================
//                                Private Methods
// ============================================================================

void StreamCapture::captureLoop() {
    cv::Mat frame;
    auto last_success = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10); // Customize the timeout
    
    // Loop while the stop flag is not set
    while (!stop_capture_.load()) {
        // try to grab a new image
        if (cv_capture_.read(frame)) {
            last_success = std::chrono::steady_clock::now(); // Reset the timer

            std::lock_guard<std::mutex> lock(buffer_.mtx);

            // Define the first ROI (left part of the frame)
            cv::Rect roi1(0, 0, frame.cols / 2, frame.rows);
            buffer_.leftImage = frame(roi1).clone(); // Use clone() to copy the data

            // Define the second ROI (right part of the frame)
            cv::Rect roi2(frame.cols / 2, 0, frame.cols / 2, frame.rows);
            buffer_.rightImage = frame(roi2).clone(); // Use clone() to copy the data

            // Set new_frame flag INSIDE the mutex lock to prevent race conditions
            buffer_.new_frame = true; // Indicate that a new frame is available
        }
        else{
            // Check for timeout
            auto now = std::chrono::steady_clock::now();
            if (now - last_success > timeout) {
                std::cerr << "[ERROR] No frame received for 10 seconds. Stopping capture thread..." << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }
}


void StreamCapture::stop() {
    // Signal the capture thread to stop
    stop_capture_.store(true);
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    cv_capture_.release();
    cv::destroyAllWindows();
}
