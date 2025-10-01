#include "main.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <limits>


int main(int argc, char* argv[]) 
{
    // Select the IP address
    std::vector<std::string> ips = {
        "100.119.186.122", // Tailscale VPN IP
        "192.168.20.3",    // Local IP
        "48.209.18.239"    // Microsoft Azure Server IP
    };

    std::string ip_address;
    int defaultIndex = 1; // Default to the second IP (Local IP)
    select_ip(ips, ip_address, defaultIndex);
    std::string stream_address = "rtsp://" + ip_address + ":8554/spot-stream";

    // Initialize Oculus SDK and create a session
    ovrSession oculus_session = nullptr;
    if (!init_oculus(oculus_session)) {
        wait_before_exit();
        return -1;
    }

    // Initialize the stream capture process
    StreamCapture stream(stream_address);
    if (!stream.start()) {
        wait_before_exit();
        return -1;
    }

    // Initialize the Oculus renderer
    OculusRenderer renderer(oculus_session);
    if (!renderer.initialize(stream.width, stream.height)) {
        wait_before_exit();
        return -1;
    }

    // Execute python scripts
    run_python_script("\\..\\..\\scripts\\main.py", ip_address);

	// Initialize ZMQ Socket
    ZMQPublisher zmq("tcp://localhost:5555");

    // Create data variable for HDM & Touch Controller State
    float data[18];

    // Start input + ZMQ publisher loop in a thread at 25ms (~40Hz)
    printf("Starting IO thread...\n");
    std::thread io_thread([&]() {
        while (true) {
            get_oculus_input(data, oculus_session);
            zmq.send("from_hmd", data, sizeof(data));
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    });

    printf("Starting Render loop...\n");
    while (true) {
        if (!renderer.update(stream.buffer_)) {break;}
    }
    
    wait_before_exit();
    return 0;
}


// ================================================================
//                        FUNCTION DEFINITIONS
// ================================================================

void wait_before_exit() {
#ifdef _WIN32
    std::cout << "\nPress any key to exit . . .";
    std::cout.flush();
    _getch();
#endif
}


void select_ip(const std::vector<std::string>& ips, std::string& ip_address, int defaultIndex) {
    std::cout << "Available IPs:\n";
    for (size_t i = 0; i < ips.size(); ++i) {
        std::cout << i + 1 << ". " << ips[i];
        if (static_cast<int>(i) == defaultIndex)
            std::cout << " (default)";
        std::cout << "\n";
    }

    std::cout << "\nChoose an IP by number, or press Enter for default: ";
    std::string input;
    std::getline(std::cin, input);

    int choice = defaultIndex; // default
    if (!input.empty()) {
        try {
            int num = std::stoi(input);
            if (num >= 1 && num <= static_cast<int>(ips.size())) {
                choice = num - 1;
            }
            // else invalid -> keep default
        } catch (...) {
            // Non-numeric input -> keep default
        }
    }

    ip_address = ips[choice];
}


bool init_oculus(ovrSession& session) {
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to initialize Oculus SDK" << std::endl;
        return false;
    }

    ovrGraphicsLuid luid;
    result = ovr_Create(&session, &luid);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to create Oculus session" << std::endl;
        ovr_Shutdown();
        return false;
    }
    return true;
}


void run_python_script(const std::string& relativeScriptPath, const std::string& args) {
    char currentDir[MAX_PATH];
    GetCurrentDirectoryA(MAX_PATH, currentDir);

    std::string command = "start cmd /k python \""
                        + std::string(currentDir)
                        + relativeScriptPath
                        + "\" " + args;

    if (system(command.c_str()) != 0) {
        throw std::runtime_error("Error executing Python script: " + relativeScriptPath);
    }
}


void get_oculus_input(float* data, ovrSession& session) {
    ovrInputState InputState;
    ovr_GetInputState(session, ovrControllerType_Touch, &InputState);

    ovrVector2f rightStick = InputState.Thumbstick[ovrHand_Right];
    ovrVector2f leftStick  = InputState.Thumbstick[ovrHand_Left];

    ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);

    // float yaw, pitch, roll;
    OVR::Posef pose;
    pose = ts.HeadPose.ThePose;
    // Get Euler angles from the pose
    float yaw, pitch, roll;
    pose.Rotation.GetEulerAngles<OVR::Axis_Y, OVR::Axis_X, OVR::Axis_Z>(&yaw, &pitch, &roll);

    // Pack orientation and thumbsticks
    data[0] = yaw;
    data[1] = -pitch;
    data[2] = -roll;
    data[3] = leftStick.x;
    data[4] = leftStick.y;
    data[5] = rightStick.x;
    data[6] = rightStick.y;

    // Buttons: A, B, X, Y
    data[7] = (InputState.Buttons & ovrButton_A) ? 1.0f : 0.0f;
    data[8] = (InputState.Buttons & ovrButton_B) ? 1.0f : 0.0f;
    data[9] = (InputState.Buttons & ovrButton_X) ? 1.0f : 0.0f;
    data[10] = (InputState.Buttons & ovrButton_Y) ? 1.0f : 0.0f;

    // Thumbstick clicks
    data[11] = (InputState.Buttons & ovrButton_LThumb) ? 1.0f : 0.0f;
    data[12] = (InputState.Buttons & ovrButton_RThumb) ? 1.0f : 0.0f;

    // Triggers
    data[13] = InputState.IndexTrigger[ovrHand_Left];
    data[14] = InputState.IndexTrigger[ovrHand_Right];
    data[15] = InputState.HandTrigger[ovrHand_Left];   // grip
    data[16] = InputState.HandTrigger[ovrHand_Right];  // grip

    // New data
    data[17] = pose.Translation.y; // <-- This is the height from floor
}


// ================================================================
//                ZMQPublisher Class Implementation
// ================================================================

ZMQPublisher::ZMQPublisher(const std::string& address)
    : context_(1), publisher_(context_, zmq::socket_type::pub)
{
    publisher_.connect(address);
}

void ZMQPublisher::send(const std::string& topic, const void* data, size_t size) {
    zmq::message_t topic_msg(topic.data(), topic.size());
    zmq::message_t data_msg(size);
    std::memcpy(data_msg.data(), data, size);

    publisher_.send(topic_msg, zmq::send_flags::sndmore);
    publisher_.send(data_msg, zmq::send_flags::none);
}

ZMQPublisher::~ZMQPublisher() {
    publisher_.close();
    context_.shutdown();
    context_.close();
}