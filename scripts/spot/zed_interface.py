import threading
import pyzed.sl as sl
import cv2
import argparse

class ZEDInterface:
    # Resolution and FPS configuration constants
    RESOLUTION_CONFIG = {
        '720': {'resolution': sl.RESOLUTION.HD720, 'fps': 60},    # HD720: max 60 FPS
        '1080': {'resolution': sl.RESOLUTION.HD1080, 'fps': 30},  # HD1080: max 30 FPS
        '2k': {'resolution': sl.RESOLUTION.HD2K, 'fps': 15}       # HD2K: max 15 FPS
    }
    
    def __init__(self, resolution='720', stop_event=None):
        self.stop_event = stop_event
        self.zed = sl.Camera()
        input_type = sl.InputType()
        init = sl.InitParameters(input_t=input_type)
        
        # Set resolution and FPS based on argument
        if resolution in self.RESOLUTION_CONFIG:
            config = self.RESOLUTION_CONFIG[resolution]
            init.camera_resolution = config['resolution']
            init.camera_fps = config['fps']
            self.fps = config['fps']  # Store FPS for pipeline use
        else:
            print(f"Warning: Unknown resolution '{resolution}', defaulting to 720p")
            init.camera_resolution = sl.RESOLUTION.HD720
            init.camera_fps = 60
            self.fps = 60
            
        init.depth_mode = sl.DEPTH_MODE.NONE
        if not self.stop_event:
            self.stop_event = threading.Event() # Create its own stop event if not provided

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)

        self.runtime = sl.RuntimeParameters()
        self.image_size_out = self.zed.get_camera_information().camera_configuration.resolution
        self.image_zed_out = sl.Mat(self.image_size_out.width, self.image_size_out.height, sl.MAT_TYPE.U8_C4)

    def get_image(self):
        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed_out, sl.VIEW.SIDE_BY_SIDE, sl.MEM.CPU, self.image_size_out)
            image_ocv_out = self.image_zed_out.get_data()
            image = cv2.cvtColor(image_ocv_out, cv2.COLOR_RGBA2RGB)
            return image
        else:
            print("Error grabbing image from ZED")
            return None

    def start_streaming(self, broker_address):
        # Get the actual ZED camera resolution
        zed_width = self.image_size_out.width
        zed_height = self.image_size_out.height
        
        pipeline = (
            'appsrc ! queue max-size-buffers=2 leaky=downstream ! videoconvert ! videoscale ! video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! ' \
            'nvvidconv ! nvv4l2h264enc bitrate=3000000 preset=ultrafast tune=zerolatency ! h264parse ! queue max-size-buffers=2 leaky=downstream ! ' \
            'rtspclientsink protocols=tcp location=rtsp://%s:8554/spot-stream sync=false'
            % (zed_width, zed_height, self.fps, broker_address)
        )

        image_size = (zed_width, zed_height)
        out_send = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, self.fps, image_size, True)
        
        if not out_send.isOpened():
            print('VideoWriter not opened - check MediaMTX server and pipeline')
            return

        print(f"\n *** Launched RTSP Streaming at rtsp://{broker_address}:8554/spot-stream ***")
        print(f" *** Streaming Resolution: {zed_width}x{zed_height} @ {self.fps} FPS ***\n")

        while not self.stop_event.is_set():
            image = self.get_image()
            if image is not None:
                try:
                    out_send.write(image)
                except Exception as e:
                    print(f"Error writing frame: {e}")
            else:
                print("Warning: No image from ZED camera")

    def shutdown(self):
        self.stop_event.set()
        self.zed.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='ZED Camera RTSP Streaming')
    parser.add_argument(
        'ip_address', 
        type=str, 
        nargs='?', 
        default='100.119.186.122', 
        help='IP address for the RTSP stream'
    )
    parser.add_argument(
        'resolution',
        type=str,
        nargs='?',
        default='720',
        choices=['720', '1080', '2k'],
        help='Camera resolution: 720, 1080, or 2k (default: 720)'
    )
    args = parser.parse_args()
    
    try:
        zed_interface = ZEDInterface(args.resolution)
        zed_interface.start_streaming(args.ip_address)
    except KeyboardInterrupt:
        print("ZEDInterface interrupted by user.")
    finally:
        zed_interface.shutdown()
        print("ZEDInterface shutdown complete.")