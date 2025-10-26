# --- ZMQ CONFIGURATION
# Note: For communication with the C++ application.
ZMQ_IP = "localhost"
ZMQ_PORT = 5555
ZMQ_SUB_TOPIC = "from_hmd"

# --- UDP CONFIGURATION
# Note: For communication with the Unreal Engine application (KATVR).
UDP_IP = "127.0.0.1"
UDP_PORT = 8002

# --- Additional options (mainly used during development)
USING_STEREO_CAMERA = True  # Set to True if using ZED stereo camera
LOG_KATVR_DATA = False      # Set to True to log KATVR data to CSV in /logs
