import threading
import zmq
import socket
import time
import struct
import math
import json
import os
import csv
from datetime import datetime
import paho.mqtt.client as mqtt
from data_processor import KatvrDataProcessor
from gamepad import GameController
from config import UDP_IP, UDP_PORT
from config import ZMQ_PORT, ZMQ_SUB_TOPIC
from config import LOG_KATVR_DATA


# ====================== DATA MANAGER (MAIN ENTRY) ======================
class DataManager:
    def __init__(self, broker_address: str):
        self.oculus = OculusHandler()
        self.katvr = KatvrHandler(KatvrDataProcessor(), self.oculus)
        self.game = GamepadHandler()
        self.gui = GUIHandler()
        self.spot = SpotHandler(self, broker_address=broker_address)
        self.monitor = ActivityMonitor(self.oculus, self.katvr, self.spot)

    def get_inputs(self):
        return InputBuilder.build(self.gui, self.oculus, self.katvr, self.game)

    def start(self):
        self.oculus.start()
        self.katvr.start()
        self.spot.start()
        self.monitor.start()

    def stop(self):
        self.oculus.stop()
        self.katvr.stop()
        self.spot.stop()
        self.monitor.stop()

    def update_gui(self, data: dict):
        """
        Updates the GUI data with the dictionary of the compilation of all modular blocks added to the ModularGUI class.

        This update must be done through the DataManager because it requires access to both the Oculus and KatVR data,
        for calibration purposes.
        """
        self.gui.update(data)

        if self.gui.commands.get("Calibrate: HMD Only"):
            self.oculus.calibrate_yaw()
        if self.gui.commands.get("Calibrate: HMD+KATVR"):
            self.katvr.calibrate_yaw()

    
# ====================== OCULUS HANDLER ======================
class OculusHandler:
    """
    Handles receiving and processing Oculus device data via ZMQ.
    """
    def __init__(self):
        self.data = {}
        self.last_time = None
        self.hmd_yaw_offset = 0.0
        self.thread = None
        self.stop_event = threading.Event()

    def start(self):
        self.thread = threading.Thread(target=self._zmq_receiver, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        print("[DataManager] [OculusHandler] ZMQ thread stopped.")

    def _zmq_receiver(self):
        context = zmq.Context()
        socket_zmq = context.socket(zmq.SUB)
        socket_zmq.bind(f"tcp://*:{ZMQ_PORT}")
        socket_zmq.setsockopt_string(zmq.SUBSCRIBE, ZMQ_SUB_TOPIC)
        print("[DataManager] [OculusHandler] ZMQ listening...")

        while not self.stop_event.is_set():
            try:
                zmq_topic = socket_zmq.recv_string()
                message = socket_zmq.recv()
                if zmq_topic == ZMQ_SUB_TOPIC:
                    self.update_data(message)
            except Exception as e:
                print(f"[DataManager] [OculusHandler] Error: {e}")
                break

        print("[DataManager] [OculusHandler] ZMQ thread stopped.")

    def update_data(self, message):
        self.last_time = time.time()
        d = list(struct.unpack('18f', message))
        keys = [
            'hmd_yaw', 'hmd_pitch', 'hmd_roll',
            'left_joystick_x', 'left_joystick_y',
            'right_joystick_x', 'right_joystick_y',
            'button_a', 'button_b', 'button_x', 'button_y',
            'button_lt', 'button_rt',
            'left_trigger', 'right_trigger',
            'left_grip', 'right_grip',
            'hmd_height'
        ]
        self.data = dict(zip(keys, d))
        if self.data.get('button_rt', 0.0) > 0.5:
            self.calibrate_yaw()

    def calibrate_yaw(self):
        """
        Calibrates the yaw offset of the HMD with respect to its origin. 
        
        This is useful during Oculus-only control of SPOT, in case the user has rotated from the original facing direction,
        thus requiring a recalibration of the yaw zero point.
        """
        self.hmd_yaw_offset = self.data.get('hmd_yaw', 0.0)
        print(f"[DataManager] [OculusHandler] Calibrated yaw offset: {self.hmd_yaw_offset:.3f}")

    def clear(self):
        self.data = {}
        self.last_time = None


# ====================== KATVR HANDLER ======================
class KatvrHandler:
    """
    Handles receiving and processing KatVR device data via raw UDP.
    """
    def __init__(self, processor: KatvrDataProcessor, oculus: OculusHandler):
        self.data = {"status": False}
        self.oculus = oculus
        self.last_time = None
        self.processor = processor
        self.hmd_katvr_yaw_offset = 0.0
        self.thread = None
        self.stop_event = threading.Event()

        self.log_file = None
        self.csv_writer = None
        self.csv_fieldnames = []
        self.log_file_path = None

    def start(self):
        if LOG_KATVR_DATA:
            # Get script directory (not current working directory)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            logs_dir = os.path.join(script_dir, "logs")
            os.makedirs(logs_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file_path = os.path.join(logs_dir, f"katvr_{timestamp}.csv")
            self.log_file = open(self.log_file_path, mode='w', newline='')
            print(f"[DataManager] [KatvrHandler] Logging enabled. Saving to: {self.log_file_path}")

        self.thread = threading.Thread(target=self._udp_receiver, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        print("[DataManager] [KatvrHandler] UDP thread stopped.")

    def _udp_receiver(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print("[DataManager] [KatvrHandler] UDP listening...")

        while not self.stop_event.is_set():
            try:
                raw, _ = sock.recvfrom(1024)
                data = struct.unpack("3f", raw[:12])
                self.update_data(data)
            except Exception as e:
                print(f"[DataManager] [KatvrHandler] Error: {e}")
                break

    def update_data(self, unpacked):
        self.last_time = time.time()
        self.data = self.processor.process_inputs(
            delta_time=unpacked[0],
            yaw_virtual=unpacked[1],
            velocity=unpacked[2]
        )

        # Check the calibration trigger from Oculus RT button
        if self.oculus.data.get('button_rt', 0.0) > 0.5:
            self.calibrate_yaw()

        # Optional: Log data to CSV if enabled
        if LOG_KATVR_DATA and self.log_file:
            if not self.csv_writer:
                # Only use non-dict keys
                filtered_keys = [k for k, v in self.data.items() if not isinstance(v, dict)]
                self.csv_fieldnames = ["timestamp"] + filtered_keys
                self.csv_writer = csv.DictWriter(self.log_file, fieldnames=self.csv_fieldnames)
                self.csv_writer.writeheader()

            row = {"timestamp": time.time()}
            for key in self.csv_fieldnames[1:]:  # skip 'timestamp'
                row[key] = self.data.get(key, None)
            self.csv_writer.writerow(row)
            self.log_file.flush()

    def calibrate_yaw(self):
        """
        Calibrates the yaw offset of the KATVR platform with respect to the Oculus HMD yaw.
        """
        oculus_yaw = self.oculus.data.get('hmd_yaw', 0.0)
        if oculus_yaw is None:
            return
        # Calculate the offset between Oculus HMD yaw and KatVR yaw
        self.hmd_katvr_yaw_offset = oculus_yaw - math.radians(self.data.get('yaw', 0.0))
        print(f"[DataManager] [KatvrHandler] Calibrated HMD+KATVR offset: {self.hmd_katvr_yaw_offset:.3f}")

    def clear(self):
        self.data = {"status": False}
        self.last_time = None
        self.processor.handle_connection_lost()


# ====================== GUI HANDLER ======================
class GUIHandler:
    """
    Stores and manages GUI-related data.
    """
    def __init__(self):
        self.data = {}

    def update(self, data: dict):
        """ Do NOT use this method directly, use DataManager.update_gui() instead. """
        self.data = data

    def source(self) -> str:
        """
        Returns a string with the data source name.
        Values could be `OCULUS`, `KATVR`, or `PC`.
        """
        sources: dict = self.data.get("Source", {})
        for key, value in sources.items():
            if bool(value):
                return key
        return "Unknown"

    @property
    def left_joystick(self) -> dict:
        return self.data.get("Left Joystick", {})

    @property
    def right_joystick(self) -> dict:
        return self.data.get("Right Joystick", {})

    @property
    def square_joystick(self) -> dict:
        return self.data.get("Square Joystick", {})

    @property
    def commands(self) -> dict:
        return self.data.get("Commands", {})
    

# ====================== GAMEPAD HANDLER ======================
class GamepadHandler:
    def __init__(self):
        self.controller = None

    def start(self):
        if not self.controller:
            try:
                self.controller = GameController()
            except Exception as e:
                self.controller = None

    def stop(self):
        if self.controller:
            self.controller.close()
        self.controller = None


# ====================== SPOT HANDLER ======================
class SpotHandler:
    """
    Handles two-way communication with the remote SPOT Client via MQTT.
    """
    def __init__(self, data_manager: DataManager, broker_address="localhost", rate=20):
        self.data_manager = data_manager
        self.data : dict = {}
        self.last_time = None
        # MQTT configuration
        self.pub_topic = "spot/inputs"
        self.sub_topic = "spot/data"
        self.rate = rate
        self.broker_address = broker_address
        self.broker_port = 1883
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect # Add on_disconnect callback
        self.running = False
        self.connected = False # Track connection status
        self.connection_thread = None

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[DataManager] [SpotHandler] MQTT client connected successfully.")
            self.connected = True
            self.client.subscribe(self.sub_topic)
        else:
            self.connected = False

    def on_disconnect(self, client, userdata, rc):
        print(f"[DataManager] [SpotHandler] MQTT client disconnected with result code {rc}.")
        self.connected = False

    def _connect_loop(self):
        """
        Internal method to continuously try connecting to the MQTT broker.
        """
        while self.running:
            if not self.connected:
                try:
                    print(f"[DataManager] [SpotHandler] Attempting to connect to MQTT broker at {self.broker_address}:{self.broker_port}...")
                    self.client.connect(self.broker_address, self.broker_port, keepalive=60)
                    self.client.loop_forever()
                except Exception as e:
                    print(f"[DataManager] [SpotHandler] MQTT connection attempt failed: {e}. Retrying in 5 seconds...")
                    self.connected = False # Ensure connected status is false on exception
                    time.sleep(5) # Wait before retrying
            else:
                # Waiting for a disconnect
                time.sleep(1)

    def start(self):
        self.running = True
        self.connection_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self.connection_thread.start()
        threading.Thread(target=self.publish_loop, daemon=True).start()
        print("[DataManager] [SpotHandler] MQTT services started.")

    def publish_loop(self):
        interval = 1.0 / self.rate
        next_time = time.time()
        while self.running:
            if self.connected:
                try:
                    payload = json.dumps(self.data_manager.get_inputs())
                    self.client.publish(self.pub_topic, payload)
                    # print(f"[DataManager] Published through MQTT: {payload}")
                except Exception as e:
                    print(f"[DataManager] [SpotHandler] Error publishing message to MQTT Publisher: {e}")

            next_time += interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.time() # Reset next_time if behind schedule

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            self.data = data
            self.last_time = time.time()
        except Exception as e:
            print(f"[DataManager] [SpotHandler] Error processing MQTT message: {e}")

    def clear(self):
        self.data = {}
        self.last_time = None

    def stop(self):
        self.running = False
        if self.client:
            self.client.disconnect()

        if self.connection_thread and self.connection_thread.is_alive():
            self.connection_thread.join(timeout=1) 
            if self.connection_thread.is_alive():
                print("[DataManager] [SpotHandler] Connection thread did not terminate gracefully.")


# ====================== INPUT BUILDER ======================
class InputBuilder:
    """
    Use the `build()` method to create a dictionary of inputs to be sent to the SPOT Client.
    """
    @staticmethod
    def build(gui: GUIHandler, oculus: OculusHandler, katvr: KatvrHandler, game: GamepadHandler):
        """
        Builds a dictionary of inputs based on the current state of the GUI, Oculus, KatVR, and Gamepad.

        The created inputs depend on the source of the data:
        - If the source is `OCULUS`, it uses Oculus data.
        - If the source is `KATVR`, it combines Oculus and KatVR data.
        - If the source is `PC`, it uses GUI joystick inputs. 
        - If the source is `GAME`, it uses Game controller data.
        """
        def normalize_angle(angle):
            return (angle + math.pi) % (math.pi * 2) - math.pi
    
        data = {}

        if gui.source() == "OCULUS":
            yaw = normalize_angle(oculus.data.get('hmd_yaw', 0.0) - oculus.hmd_yaw_offset)
            data = {
                'source': 'oculus',
                'yaw': yaw,
                'pitch': oculus.data.get('hmd_pitch', 0.0),
                'roll': oculus.data.get('hmd_roll', 0.0),
                'move_forward': oculus.data.get('left_joystick_y', 0.0),
                'move_lateral': oculus.data.get('left_joystick_x', 0.0),
                'rotate': oculus.data.get('right_joystick_x', 0.0),
                'stand': oculus.data.get('button_a', 0.0) > 0.5 or gui.commands.get("Stand", False),
                'sit': oculus.data.get('button_b', 0.0) > 0.5 or gui.commands.get("Sit", False),
                'look_mode': oculus.data.get('right_grip', 0.0) > 0.5,
                'robot_height': oculus.data.get('right_joystick_y', 0.0),
            }

        elif gui.source() == "KATVR":
            hmd_yaw_relative = normalize_angle(
                oculus.data.get('hmd_yaw', 0.0) -
                math.radians(katvr.data.get('yaw', 0.0)) -
                katvr.hmd_katvr_yaw_offset
            )
            data = {
                'source': 'katvr',
                'yaw': hmd_yaw_relative,
                'pitch': oculus.data.get('hmd_pitch', 0.0),
                'roll': oculus.data.get('hmd_roll', 0.0),
                'move_forward': max(
                    oculus.data.get('right_joystick_y', 0.0),
                    katvr.data.get('forward_velocity', 0.0),
                    key=abs
                ),
                'move_lateral': max(
                    oculus.data.get('right_joystick_x', 0.0),
                    katvr.data.get('lateral_velocity', 0.0),
                    key=abs
                ),
                'rotate': oculus.data.get('left_joystick_x', 0.0),
                'stand': oculus.data.get('button_a', 0.0) > 0.5 or gui.commands.get("Stand", False),
                'sit': oculus.data.get('button_b', 0.0) > 0.5 or gui.commands.get("Sit", False),
                'katvr_yaw': math.radians(katvr.data.get('yaw', 0.0)),
                'look_mode': oculus.data.get('right_grip', 0.0) > 0.5,
                'hmd_height': oculus.data.get('hmd_height', 0.0),
            }

        elif gui.source() == "PC":
            data = {
                'source': 'pc',
                'yaw': -gui.square_joystick.get("x", 0.0),
                'pitch': -gui.square_joystick.get("y", 0.0),
                'roll': 0.0,
                'move_forward': gui.left_joystick.get("y", 0.0),
                'move_lateral': gui.left_joystick.get("x", 0.0),
                'rotate': gui.right_joystick.get("x", 0.0),
                'stand': gui.commands.get("Stand", False),
                'sit': gui.commands.get("Sit", False),
                'start': gui.commands.get("Start", False),
                'shutdown': gui.commands.get("Shutdown", False),
                'robot_height': gui.right_joystick.get("y", 0.0),
            }
        
        elif gui.source() == "GAME":
            game.start()
            if game.controller:
                data = game.controller.get_data()
                data = {
                    'source': 'gamepad',
                    'yaw': data.get('yaw', 0.0),
                    'pitch': data.get('pitch', 0.0),
                    'move_forward': data.get('v_x', 0.0),
                    'move_lateral': data.get('v_y', 0.0),
                    'rotate': data.get('v_rot', 0.0),
                    'stand': data.get('stand_cmd', False) or gui.commands.get("Stand", False),
                    'sit': data.get('sit_cmd', False) or gui.commands.get("Sit", False),
                    'robot_height': data.get('height', 0.0)
                }

        if gui.source() != "GAME":
            game.stop()

        return data


# ====================== ACTIVITY MONITOR ======================
class ActivityMonitor:
    """ Monitors the activity of the data received by the Oculus (ZMQ), KatVR (UDP), and Spot (MQTT) handlers to ensure they are responsive."""
    def __init__(self, oculus: OculusHandler, katvr: KatvrHandler, spot: SpotHandler, timeout=2):
        self.oculus = oculus
        self.katvr = katvr
        self.spot = spot
        self.timeout = timeout
        self.thread = None
        self.stop_event = threading.Event()

    def start(self):
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        print("[DataManager] [ActivityMonitor] Monitoring thread stopped.")

    def _monitor(self):
        while not self.stop_event.is_set():
            self._check(self.oculus.last_time, self.oculus.clear)
            self._check(self.katvr.last_time, self.katvr.clear)
            self._check(self.spot.last_time, self.spot.clear)
            time.sleep(0.1)

    def _check(self, last_time, clear_function):
        if last_time and (time.time() - last_time > self.timeout):
            clear_function()


