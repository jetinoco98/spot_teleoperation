#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <mutex>
#include <chrono>
#include <atomic>

struct VideoCaptureFrameBuffer {
    cv::Mat leftImage;
    cv::Mat rightImage;
    std::mutex mtx;
    bool new_frame = false;  // Initialize to false to prevent undefined behavior
};

class StreamCapture {
public:
    StreamCapture(const std::string& stream_address);
    ~StreamCapture();

    bool start();
    void stop();

    int width = 0;
    int height = 0;

    VideoCaptureFrameBuffer buffer_;

private:
    void captureLoop();

    std::string pipeline_;
    cv::VideoCapture cv_capture_;
    std::thread capture_thread_;
    std::atomic<bool> stop_capture_;  // Atomic flag to signal thread termination
};