# Immersive Teleoperation System for the SPOT Robot

This repository contains a modular software architecture designed for teleoperation of the Boston Dynamics Spot robot using the following immersive and non-immersive control methods:
- Mouse and Keyboard
- Gaming Controller
- Meta Quest VR
- KATVR Platform


**Authors:** 
- Josue Tinoco, jetinoco98@gmail.com
- Ali Yousefi, ali.yousefi@edu.unige.it
- Carmine Tommaso Recchiuto, carmine.recchiuto@dibris.unige.it
- Antonio Sgorbissa, antonio.sgorbissa@unige.it

‎ 

©2025 RICE Lab - DIBRIS, University of Genova
<p align="left">
<img src="https://github.com/user-attachments/assets/0fdac2aa-7100-4caa-9191-df72cb55c8be" width="150" title="rice_logo">
</p>

## Package Description
The package includes source code developed for the following functionalities:

### For the Remote PC connected to SPOT (Jetson Nano):

The code is found on the `scripts/spot` folder.

1. `spot_client.py`: Responsible for receiving the data and calling the commands from `spot_interface.py` for controlling the Spot robot.
2. `spot_interface.py`: Contains the functions to control the Spot robot using the Spot SDK.
2. `zed_interface.py`: Starts an RTSP stream for the ZED camera using a GStreamer pipeline.
3. `input_controller.py`: Processes the VR headset Roll-Pitch-Yaw data through an LQR filter for smooth tracking, and the joystick data through a threshold based multiplier to be used for velocity control.


### For the Local PC (Windows):

The code is found at the root of the `scripts`, `src` and `include` folders.

1. `main.cpp`: Initializes the VR headset, connects to the RTSP stream from the ZED camera, and renders the stereo images on the VR headset. It also reads the control commands from the joystick and sends them to the `main.py` script.
2. `main.py`: Creates a Graphical User Interface (GUI), receives the Meta Quest data from `main.cpp`, obtains the input data from the rest of the control sources (mouse-keyboard, gaming controller, KATVR platform) and then processes all the data to be sent to the remote `spot_client.py` script.


## Dependencies
**The required software on the Local PC are the following:**
- Windows 64 bits
- python (3.7.0 or later)
- Visual Studio Code
- CMake Extension for VSCode
- [GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-windows.html?gi-language=missing:%20GSTREAMER_LIBRARY%20GSTREAMER_BASE_LIBRARY%20GSTREAMER_BASE_INCLUDE_DIR)
- [FFmpeg](https://ffmpeg.org/download.html#build-windows)
- The OculusSDK, GLEW, SDL and OpenCV dependencies are already included in the `external` folder and will be automatically detected by CMake.
- All python dependencies can be installed using the `requirements.txt` file located in the `scripts` folder:
    ```
    pip install -r requirements.txt
    ```

**For the Jetson board:**
- Ubuntu 18.04
- python (3.7.0 or later)
- [Spot SDK](https://dev.bostondynamics.com/)
- [ZED SDK 3.x](https://www.stereolabs.com/developers) 
- [ZED Python API](https://www.stereolabs.com/docs/app-development/python/install)
- [Gstreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=missing:%20GSTREAMER_LIBRARY%20GSTREAMER_BASE_LIBRARY%20GSTREAMER_BASE_INCLUDE_DIR)
- [Mediamtx](https://github.com/bluenviron/mediamtx#corrupted-frames)
- [Mosquitto](https://mosquitto.org/download/)
- All python dependencies can be installed using the `requirements.txt` file located in the `scripts/spot` folder:
    ```
    pip install -r requirements.txt
    ```

## Build
**On the Local PC:**
Clone the source files from the repository and follow the instructions below:
1. On VSCode, open the repository folder.
2. Press `Ctrl+Shift+P`, type and select `CMake: Select a Kit`, then choose the `Visual Studio 2022 Release - amd64` option.
3. Press `Ctrl+Shift+P`, type and select `CMake: Select Variant`, then choose the `Release` option.
4. Press `Ctrl+Shift+P`, type and select `CMake: Configure`, then wait for the configuration to finish.
5. Press `Ctrl+Shift+P`, type and select `CMake: Build`, or simply press the build button at the left side of the bottom bar of the VSCode window, then wait for the build to finish.
6. The resulting executable and all the required DLL files will be located in the `build/Release` folder.

**On the Jetson:**
Simply clone the `scripts/spot` folder from the repository.

You also have to manually build OpenCV with GStreamer, as it does not come by default with the pip installation of OpenCV ([Tutorial](https://galaktyk.medium.com/how-to-build-opencv-with-gstreamer-b11668fa09c)).


## Usage
**On the remote side (Jetson):**

Start the `mediamtx` server:
```
./mediamtx
```
Then, run the `spot_client.py` script as follows:
```
python3 spot_client.py <IP_ADDRESS>
```
The `<IP_ADDRESS>` could be the public IP address of a cloud server, the local IP address of the Jetson in the local network (e.g., `192.168.1.x`), or a Tailscale IP address (e.g., `100.x.x.x`).

**On the local side (Windows):** 

**For Immersive Teleoperation with VR:** Run the `spot_teleoperation.exe` file by simply double-clicking it. The GUI will automatically spawn, and you will be able to select the appropriate control method by clicking the corresponding buttons or the mapped keys (1-4).

To use the KATVR platform, make sure to also run the following programs:
- `KAT Industry`
- `KATVR Integration`

**For Teleoperation when VR is not available:** Simply execute the `main.py` script, or use the already provided `run.bat` file which can be placed as a shortcut on the desktop for easier access, and will automatically open a terminal and run the script.

**Disclaimer:**
The video stream from the GUI itself is only available when using Mouse and Keyboard controls, and is not optimized for latency. If using the Game Controller method, the stream will stop, so it is required to execute the `stream.bat` file located in the `scripts` folder that opens the video stream on a separate window with optimized latency.