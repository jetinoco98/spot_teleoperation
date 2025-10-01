#!/usr/bin/env python

import argparse
import math
import paho.mqtt.client as mqtt
import json
import threading
import time
from spot_interface import SpotInterface
from zed_interface import ZEDInterface
from input_controller import Controller
    
USE_LQR = False  # Use LQR for HDM controls

class SpotClient:
    def __init__(self, broker_address):
        self.stop_event = threading.Event()  # Event to signal termination
        # Message frequency monitoring
        self.message_count = 0
        self.last_frequency_check = time.time()
        self.all_frequency_samples = []  # Store ALL frequency samples for true average
        # MQTT related variables
        self.broker_address = broker_address
        self.sub_topic = "spot/inputs"
        self.pub_topic = "spot/data"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address, 1883)
        self.client.loop_start()
        self.controller = Controller()   # Controller for HDM & Touch controls
        # Variables to store and control inputs
        self.inputs = None
        self.inputs_last_message_time = 0
        
        # Initialize Spot robot interface
        self.robot = SpotInterface()
        self.robot.initialize()

        # Start a thread to publish robot data at 10Hz
        threading.Thread(target=self.publish_data_loop, daemon=True).start()
    
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe(self.sub_topic)

    def on_message(self, client, userdata, msg):
        # === MESSAGES RECEIVED ON STANDARD CONTROLS ===
        if msg.topic == self.sub_topic:
            payload = msg.payload.decode()
            inputs = json.loads(payload)
            if not isinstance(inputs, dict):
                print(f"[SpotClient] Received invalid data: {payload}")
                return
            self.inputs = inputs
            self.inputs_last_message_time = time.time()
            self.message_count += 1
        
    def publish_data_loop(self):
        """Continuously publish robot data at 10Hz in a separate thread"""
        interval = 1/10  # 10Hz
        next_time = time.time()
        
        while not self.stop_event.is_set():
            try:
                robot_data = self.robot.get_data()
                payload = json.dumps(robot_data)
                self.client.publish(self.pub_topic, payload)
            except Exception as e:
                print(f"[SpotClient] Error publishing robot data: {e}")
            
            # Maintain fixed timing
            next_time += interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.time()  # We're behind schedule; reset next_time
        
    def process_inputs(self):
        # Start/Stop commands
        if self.inputs.get('start'):
            self.robot.initialize()
        elif self.inputs.get('shutdown'):
            self.robot.shutdown()

        # Stand/Sit commands
        if self.inputs.get('stand'):
            self.robot.stand()
        elif self.inputs.get('sit'):
            self.robot.sit()

        # Special KATVR Auto Sit/Stand functionality
        if self.inputs.get('source') == 'katvr':
            self.robot.auto_sit_in_katvr(
                self.inputs.get('hmd_height'),
                self.inputs.get('stand')
            )
            self.robot.auto_stand_in_katvr(
                self.inputs.get('hmd_height'),
            )

        # Stop processing if the robot is not standing
        if not self.robot.is_standing:
            return
        
        # Set the robot's body frame YPR orientation
        if USE_LQR: 
            hmd_inputs = [
                self.inputs.get('yaw'),        # HMD Yaw (radians)
                self.inputs.get('pitch'),      # HMD Pitch (radians)
                self.inputs.get('roll'),       # HMD Roll (radians)
            ]
            spot_measures = self.robot.get_body_orientation()
            hmd_controls = self.controller.get_hmd_controls(hmd_inputs, spot_measures)
            self.robot.set_lqr_based_orientation(hmd_controls)
        else:
            self.robot.set_orientation(
                yaw=self.inputs.get('yaw', 0.0), 
                pitch=self.inputs.get('pitch', 0.0), 
                roll=self.inputs.get('roll', 0.0)
            )
        
        # Set the robot's velocities based on KATVR data
        if self.inputs.get('source') == 'katvr':
            self.robot.set_velocity(
                    v_x=self.inputs.get('move_forward', 0.0),  # Direct velocity value (m/s)
                    v_y=self.inputs.get('move_lateral', 0.0),  # Direct velocity value (m/s)
                    v_rot=None,  # No direct rotation for KATVR
                )
        else:
            # Joystick inputs for velocity control
            touch_inputs = [
                self.inputs.get('move_forward', 0.0),   # Joystick value [-1,1]
                self.inputs.get('move_lateral', 0.0),   # Joystick value [-1,1]
                self.inputs.get('rotate', 0.0),         # Joystick value [-1,1]
            ]
            touch_controls = self.controller.get_touch_controls(touch_inputs)
            self.robot.set_touch_controls(touch_controls)

        # Look Mode
        if self.inputs.get('look_mode'):
            self.robot.set_velocity(0,0,0)  # Stop all movement when in Look Mode
            
            if self.inputs.get('source') == 'katvr':
                self.robot.calibrated_with_katvr = False  # Reset calibration state
                self.robot.set_height(joystick_value=self.inputs.get('move_forward', 0.0))

            elif self.inputs.get('source') == 'oculus':
                self.robot.set_height(joystick_value=self.inputs.get('robot_height', 0.0))
        else:
            self.robot.set_height(direct_value=0.0)

        # KATVR: Automatic Look Mode (PID Deadzone based adjustment)
        if self.inputs.get('source') == 'katvr' and self.inputs.get('look_mode', False):
            if abs(self.robot._v_x > 0) or abs(self.robot._v_y > 0) or abs(self.robot._v_rot > 0.05):
                self.robot.last_movement_time = time.time()

            self.robot.inactivity_time = time.time() - self.robot.last_movement_time
            deadzone = 2.0  # Default deadzone for PID controller (degrees)

            if not self.inputs.get('look_mode'):
                range = 2  # Increase over 2 seconds of inactivity
                inactivity_ratio = self.robot.inactivity_time / range
                normalized_inactivity = max(0, min(inactivity_ratio, 1))
                deadzone = 2 + 13 * normalized_inactivity  # Deadzone increases with inactivity for a max of 10°

            self.robot.update_pid_controller(
            dead_zone_degrees=deadzone,  # Deadzone for PID controller (degrees)
            )

        # KATVR: Rotation based on KATVR yaw
        if self.inputs.get('source') == 'katvr' and self.inputs.get('rotate', 0.0) == 0.0:
            self.robot.compute_angular_velocity_from_katvr(
                self.inputs.get('katvr_yaw', 0.0),
            )

        # Robot height on PC GUI Controls
        if self.inputs.get('source') == 'pc':
            self.robot.set_height(joystick_value=self.inputs.get('robot_height', 0.0))

        # Robot height on Game Controller
        if self.inputs.get('source') == 'xbox':
            self.robot.set_height(direct_value=self.inputs.get('robot_height', 0.0))

        # === Send the command to the robot
        # Note: This command will also set the robot's orientation and height.
        self.robot.send_velocity_command()


    def control_loop(self):
        interval = 1.0 / 20  # 20Hz
        next_time = time.time()

        while not self.stop_event.is_set():
            time_diff = time.time() - self.inputs_last_message_time

            if self.inputs and (time_diff < 1.0):
                self.process_inputs()
            else:
                if self.robot.is_standing:
                    self.robot.set_idle_mode()

            self.robot.store_current_state()  # Stores the current state of the robot just after processing inputs

            # Maintain fixed control loop timing
            next_time += interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.time()  # We're behind schedule; reset next_time

        # Cleanup on exit
        self.robot.shutdown()
        self.client.loop_stop()
        self.client.disconnect()
        print("Spot Client disconnected.")
        

def main(broker_address, no_zed=False):
    stop_event = threading.Event()
    
    # Initialize Spot client
    spot = SpotClient(broker_address)
    spot.stop_event = stop_event
    
    # Conditionally initialize ZED client
    zed = None
    zed_thread = None
    if not no_zed:
        zed = ZEDInterface()
        zed.stop_event = stop_event
        zed_thread = threading.Thread(target=zed.start_streaming, args=(broker_address,))
        zed_thread.start()
    else:
        print("ZED camera disabled with --no-zed argument")
    
    # Start Spot control loop
    spot_thread = threading.Thread(target=spot.control_loop)
    spot_thread.start()

    try:
        while True:
            time.sleep(0.1) # Prevent busy-waiting
            
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Signaling threads to stop...")
        stop_event.set() # Signal all threads to stop

    finally:
        if zed_thread:
            zed_thread.join(timeout=10)
            if zed_thread.is_alive():
                print("ZED thread did not terminate gracefully.")
        
        spot_thread.join(timeout=10) 
        if spot_thread.is_alive():
            print("SPOT thread did not terminate gracefully.")
        
        print("Main program has finished execution.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send video stream to RTSP server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='100.119.186.122',
        help='The IP address of the RTSP server'
    )
    parser.add_argument(
        '--no-zed',
        action='store_true',
        help='Disable ZED camera initialization'
    )
    args = parser.parse_args()
    main(args.ip_address, args.no_zed)
