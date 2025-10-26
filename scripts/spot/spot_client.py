#!/usr/bin/env python

import argparse
import paho.mqtt.client as mqtt
import json
import threading
import time
from typing import Callable
from spot_interface import SpotInterface
from zed_interface import ZEDInterface
from scripts.spot.input_controllers import HMDController, JoystickMapper, KatvrInterface, USE_LQR


class SpotClient:
    def __init__(self, broker_address: str, stop_event: threading.Event):
        self.stop_event = stop_event
        # MQTT related variables
        self.broker_address = broker_address
        self.sub_topic = "spot/inputs"
        self.pub_topic = "spot/data"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address, 1883)
        self.client.loop_start()
        self.hmd_controller = HMDController()   # Controller for HDM inputs
        # Variables to store and control inputs
        self.inputs = None
        self.inputs_last_message_time = 0
        # Initialize Spot (robot) interface
        self.robot = SpotInterface()
        self.robot.initialize()
        # Create KatVR interface instance
        self.kat_interface = KatvrInterface(self.robot)
        # Start a thread to publish robot data at 10Hz
        threading.Thread(target=self.publish_data_loop, daemon=True).start()
    
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe(self.sub_topic)

    def on_message(self, client, userdata, msg):
        if msg.topic == self.sub_topic:
            payload = msg.payload.decode()
            inputs = json.loads(payload)
            if not isinstance(inputs, dict):
                print(f"[SpotClient] Received invalid data: {payload}")
                return
            self.inputs = inputs
            self.inputs_last_message_time = time.time()
        
    def publish_data_loop(self):
        def task():
            try:
                robot_data = self.robot.get_data()
                payload = json.dumps(robot_data)
                self.client.publish(self.pub_topic, payload)
            except Exception as e:
                print(f"[SpotClient] Error publishing robot data: {e}")

        # Run the publishing task at 10Hz
        run_at_fixed_rate(10, self.stop_event, task)
        
    def process_inputs(self, inputs: dict):
        # --- START/STOP
        if inputs.get('start'):
            self.robot.initialize()
            self.kat_interface.is_calibrated = False  # Reset calibration state
        elif inputs.get('shutdown'):
            self.robot.shutdown()

        # --- STAND/SIT
        if inputs.get('stand'):
            self.robot.stand()
        elif inputs.get('sit'):
            self.robot.sit()

        # KATVR Auto Sit/Stand functionality
        if inputs.get('source') == 'katvr':
            self.kat_interface.auto_sit(
                inputs.get('hmd_height'),
                inputs.get('stand')
            )
            self.kat_interface.auto_stand(
                inputs.get('hmd_height'),
            )

        # Stop processing if the robot is not standing
        if not self.robot.is_standing:
            return

        # --- ORIENTATION CONTROL
        hmd_inputs = [
            inputs.get('yaw'),        # HMD Yaw (radians)
            inputs.get('pitch'),      # HMD Pitch (radians)
            inputs.get('roll'),       # HMD Roll (radians)
        ]
        spot_measures = self.robot.get_body_orientation()
        hmd_controls = self.hmd_controller.compute_controls(hmd_inputs, spot_measures)

        if USE_LQR:
            self.robot.set_lqr_based_orientation(hmd_controls)
        else:
            self.robot.set_orientation(
                yaw=hmd_controls[0],
                pitch=hmd_controls[1],
                roll=0
            )
        
        # --- VELOCITY CONTROL
        if inputs.get('source') == 'katvr':
            self.robot.set_velocity(
                    v_x=inputs.get('move_forward', 0.0),  # Direct velocity value (m/s)
                    v_y=inputs.get('move_lateral', 0.0),  # Direct velocity value (m/s)
                    v_rot=None,  # No direct rotation for KATVR
                )
        else:
            joystick_inputs = [
                inputs.get('move_forward', 0.0),   # Joystick value [-1,1]
                inputs.get('move_lateral', 0.0),   # Joystick value [-1,1]
                inputs.get('rotate', 0.0),         # Joystick value [-1,1]
            ]
            velocities = JoystickMapper().compute_controls(joystick_inputs)
            self.robot.set_velocity(
                v_x=velocities[0],  # Forward/Backward
                v_y=velocities[1],  # Left/Right
                v_rot=velocities[2] # Rotation
            )
            
        # --- MANUAL LOOK MODE: 
        # Holds the robot in place and lets the user set the robot height using the joystick.
        # ***Is activated while holding a trigger button.
        if inputs.get('look_mode'):
            self.robot.set_velocity(0,0,0)  # Stop all movement when in Look Mode
            
            if inputs.get('source') == 'katvr':
                self.kat_interface.is_calibrated = False
                self.robot.set_height(joystick_value=inputs.get('move_forward', 0.0))

            elif inputs.get('source') == 'oculus':
                self.robot.set_height(joystick_value=inputs.get('robot_height', 0.0))
        else:
            self.robot.set_height(direct_value=0.0)

        # KATVR: AUTOMATIC LOOK MODE (Requires dynamic PID deadzone adjustment)
        if inputs.get('source') == 'katvr':
            self.kat_interface.dynamic_deadzone_adjustment(
                look_mode=inputs.get('look_mode', False)
            )

        # KATVR: Rotation based on KATVR yaw
        if inputs.get('source') == 'katvr' and inputs.get('rotate', 0.0) == 0.0:
            self.kat_interface.compute_angular_velocity_from_katvr(
                inputs.get('katvr_yaw', 0.0),
            )

        # ROBOT HEIGHT CONTROL. For non-immersive sources.
        if inputs.get('source') == 'pc':
            self.robot.set_height(joystick_value=inputs.get('robot_height', 0.0))
        if inputs.get('source') == 'gamepad':
            self.robot.set_height(direct_value=inputs.get('robot_height', 0.0))

        # === Send the command to the robot
        # This command will also set the robot's orientation and height.
        # The robot's body frame yaw will NOT change when the robot is moving.
        self.robot.send_velocity_command()

    def control_loop(self):
        def task():
            time_diff = time.time() - self.inputs_last_message_time

            # The inputs received are recent. Connection OK.
            if self.inputs and (time_diff < 1.0):
                self.process_inputs(self.inputs)
            else:
                # Connection LOST. Stop the robot if it is standing.
                if self.robot.is_standing:
                    self.robot.stop()
            
            # Store the current state for next iteration
            self.robot.store_processed_state()

        run_at_fixed_rate(20, self.stop_event, task)

        # Cleanup after loop exits
        self.robot.shutdown()
        self.client.loop_stop() 
        self.client.disconnect()
        print("Spot Client disconnected.")


# --- Utility Functions ---
def run_at_fixed_rate(rate_hz: float, stop_event: threading.Event, task: Callable[[], None]):
    """Runs `task()` repeatedly at `rate_hz` until stop_event is set."""
    interval = 1.0 / rate_hz
    next_time = time.monotonic()
    while not stop_event.is_set():
        # Execute the function
        task()
        # Maintain fixed timing
        next_time += interval
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            stop_event.wait(timeout=sleep_time)
        else:
            next_time = time.monotonic()


### --- Main Program ---
def main(broker_address, no_zed=False):
    """
    Initializes the MQTT client and the Spot SDK, and the ZED camera in a separate thread (if not disabled).
    """
    stop_event = threading.Event()
    
    # Initialize Spot client
    spot = SpotClient(broker_address, stop_event)
    
    # Initialize ZED client
    zed = None
    zed_thread = None
    if not no_zed:
        zed = ZEDInterface(stop_event=stop_event)
        zed_thread = threading.Thread(target=zed.start_streaming, args=(broker_address,))
        zed_thread.start()
    else:
        print("ZED camera initialization disabled with --no-zed argument")
    
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
