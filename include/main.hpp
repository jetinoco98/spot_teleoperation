#pragma once

#define NOMINMAX
#include <Windows.h>
#include <conio.h>

// Standard Libraries
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cmath>
#include <stddef.h>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <mutex>

// ZeroMQ: for inter-process communication
#include <zmq.hpp>

// Local include files
#include "StreamCapture.hpp"
#include "OculusRenderer.hpp"

// Function declaration
void select_ip(const std::vector<std::string>& ips, std::string& ip_address, int defaultIndex);
bool init_oculus(ovrSession& session);
void run_python_script(const std::string& relativeScriptPath, const std::string& args = "");
void get_oculus_input(float* data, ovrSession& session);
void wait_before_exit();

// Structures
struct RPY {
    float roll, pitch, yaw;
};

// ZMQ Class declaration
class ZMQPublisher {
public:
    ZMQPublisher(const std::string& address);
    void send(const std::string& topic, const void* data, size_t size);
    ~ZMQPublisher();

private:
    zmq::context_t context_;
    zmq::socket_t publisher_;
};