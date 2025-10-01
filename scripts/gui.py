import threading
import time
import math
from typing import List, Callable

import tkinter as tk
from PIL import Image, ImageTk
import ctypes
import cv2
cv2.setLogLevel(2)  # Only show errors and critical messages

from data_manager import DataManager


class ModularGUI(tk.Tk):
    """
    The main application window for the SPOT Teleoperation GUI.
    Manages the overall window layout and adds modular blocks.
    """
    def __init__(self, data_manager: DataManager):
        # To ensure proper DPI scaling on high-DPI displays
        ctypes.windll.shcore.SetProcessDpiAwareness(2)

        # Initialize the Tkinter root window
        super().__init__()

        self.tk.call('tk', 'scaling', 1.0)

        GRID_COLUMNS = 16
        GRID_ROWS = 9
        WINDOW_SCALE_FACTOR = 0.95

        # --- Store references to data manager
        # This allows the blocks to send or receive data from the DataManager
        self.data_manager = data_manager

        self.title("SPOT Teleoperation GUI")
        self.configure(bg="black")
        self.resizable(False, False) # Prevent resizing for a fixed grid layout

        # Detect screen size and set window to the selected scale of the actual resolution, maintaining aspect ratio
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        target_w = int(screen_w * WINDOW_SCALE_FACTOR)
        target_h = int(screen_h * WINDOW_SCALE_FACTOR)

        # Maintain 16:9 aspect ratio
        if target_w / 16 * 9 <= target_h:
            self.window_w = target_w
            self.window_h = int(target_w / 16 * 9)
        else:
            self.window_h = target_h
            self.window_w = int(target_h / 9 * 16)

        self.geometry(f"{self.window_w}x{self.window_h}+0+0")
        self.style = StyleManager(self.window_w, self.window_h)

        # Calculate unit dimensions for the grid
        self.grid_unit_w = self.window_w // GRID_COLUMNS
        self.grid_unit_h = self.window_h // GRID_ROWS

        # Set up a protocol for closing the window to ensure resources are released
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.blocks: List[BaseBlock] = []  # List to keep track of all active blocks

        self.stop_event = threading.Event()
        self.data_thread = threading.Thread(target=self._publish_block_data, daemon=True)
        self.data_thread.start()

    def _publish_block_data(self):
        """
        Continuously collects data from blocks to be sent to the DataManager.
        """
        while not self.stop_event.is_set():
            all_data = {}
            for block in self.blocks:
                if hasattr(block, "get_data") and callable(block.get_data):
                    try:
                        block_name = getattr(block, "name", block.__class__.__name__)
                        data = block.get_data()
                        if isinstance(data, dict):
                            all_data[block_name] = data
                    except Exception as e:
                        print(f"[GUI] Error collecting data from {block.__class__.__name__}: {e}")
            if all_data:
                self.data_manager.update_gui(all_data)
            time.sleep(0.04)  # Update Rate: 25 FPS

    def add_block(self, block: "BaseBlock", grid_x: int, grid_y: int):
        """
        Adds a block to the GUI at a specified grid position.
        Creates a Tkinter Frame for the block and calls its build method.
        """
        # Calculate pixel dimensions for the block's frame
        w = block.width_units * self.grid_unit_w
        h = block.height_units * self.grid_unit_h
        # Create a frame to act as the container for the block's UI
        frame = tk.Frame(self, width=w, height=h)
        # Place the frame at the calculated pixel coordinates
        frame.place(x=grid_x * self.grid_unit_w, y=grid_y * self.grid_unit_h)
        # Build the block's content within its dedicated frame
        block.build(frame, width_px=w, height_px=h)
        self.blocks.append(block) # Add the block to the list of managed blocks

    def on_closing(self):
        """
        Handles the window closing event.
        """
        print("\n[GUI] GUI Window closed. Shutting down application...")
        for block in self.blocks:
            block.destroy() # Call the destroy method for each block
        self.destroy() # Destroy the Tkinter root window

class StyleManager:
    """
    A centralized location to keep style data for the GUI, including font sizes and spacing.
    """
    def __init__(self, screen_width: int, screen_height: int):
        self.screen_width = screen_width
        self.screen_height = screen_height

        # Font sizes as % of screen height
        self.FONT_PRESETS = {
            "small": 1.05,
            "normal": 1.2,
            "large": 1.4,
        }

        # Spacing as % of screen width/height
        self.SPACING_PRESETS = {
            "small": 1.5,
            "normal": 3.5,
            "large": 5,
        }

    # === FONT SIZE ===
    def font_size(self, size: str = "normal", screen_pct: float = None) -> int:
        """
        Returns font size in points based on screen height.
        """
        if screen_pct is not None:
            return int(self.screen_height * (screen_pct / 100))
        pct = self.FONT_PRESETS.get(size, self.FONT_PRESETS["normal"])
        return int(self.screen_height * (pct / 100))

    # === SPACING ===
    def spacing_y(self, level: str = "normal", screen_pct: float = None) -> float:
        """
        Returns vertical spacing as a fraction of screen height (0.0-1.0).
        """
        if screen_pct is not None:
            return screen_pct / 100
        pct = self.SPACING_PRESETS.get(level, self.SPACING_PRESETS["normal"])
        return pct / 100

    def spacing_x(self, level: str = "normal", screen_pct: float = None) -> float:
        """
        Returns horizontal spacing as a fraction of screen width (0.0-1.0).
        """
        if screen_pct is not None:
            return screen_pct / 100
        pct = self.SPACING_PRESETS.get(level, self.SPACING_PRESETS["normal"])
        return pct / 100

    # === BUTTON SIZE ===
    def button_size(self, value: float = None) -> int:
    # Example: 2.5% of screen height, adjust as needed
        return int(self.screen_height * value)

    def button_size_2d(self, width_pct: float, height_pct: float):
        """
        Returns button size (width_px, height_px) in absolute screen pixels.
        """
        width_px = int(self.screen_width * (width_pct / 100))
        height_px = int(self.screen_height * (height_pct / 100))
        return width_px, height_px
    
    # === General utility scale ===
    def scaled_size(self, value: float) -> int:
        """
        Returns a scaled size based on a percentage of the screen dimensions.
        """
        return int(self.screen_height * value)

class BaseBlock:
    """
    Base class for all modular GUI blocks.
    Defines common attributes and an abstract build method.
    """
    def __init__(self, app: "ModularGUI", width_units: int, height_units: int):
        self.app = app
        self.width_units = width_units
        self.height_units = height_units
        self.container = None # Will hold the tk.Frame for this block
        self.canvas = None    # Will hold the tk.Canvas if applicable
        self._after_id = None
        self.style = app.style

    def build(self, container: tk.Frame, width_px: int, height_px: int):
        """
        Abstract method to build the block's UI elements within the given container.
        Must be implemented by subclasses.
        """
        self.container = container
        self.width_px = width_px
        self.height_px = height_px

    def schedule_periodic_update(self, interval_ms: int, callback: Callable):
        def wrapper():
            callback()
            self._after_id = self.container.after(interval_ms, wrapper)
        wrapper()

    def destroy(self):
        """
        Placeholder for cleanup logic for the block.
        Subclasses should override this if they have resources to release (e.g., video streams).
        """
        # If the block has a periodic update scheduled, cancel it
        if hasattr(self, '_after_id') and self._after_id:
            self.container.after_cancel(self._after_id)
            self._after_id = None
        # Destroy the container frame itself
        if self.container:
            self.container.destroy()

class MetaQuestDataBlock(BaseBlock):
    def __init__(self, app: ModularGUI, update_rate_ms=50):
        super().__init__(app, width_units=2, height_units=6)
        self.update_rate_ms = update_rate_ms

        self.values = {
            'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'height': 0.0,
            'buttons': {'Y': 0, 'X': 0, 'B': 0, 'A': 0, 'L3': 0, 'R3': 0},
            'joystick_left': {'x': 0.0, 'y': 0.0},
            'joystick_right': {'x': 0.0, 'y': 0.0},
            'LT': 0.0, 'LG': 0.0, 'RT': 0.0, 'RG': 0.0
        }

        self.label_refs = {}
        self.button_shapes = {}
        self.btn_canvas = None

    def build(self, container, width_px, height_px):
        super().build(container, width_px, height_px)
        container.configure(bg="#1e1e1e")
        style = self.app.style

        title_font_size = style.font_size("large")
        label_font_size = style.font_size("normal")
        value_font_size = style.font_size("small")
        y_offset = style.spacing_y("normal") * style.screen_height / height_px
        y_offset_low = style.spacing_y("small") * style.screen_height / height_px

        # Helper to create a label
        def make_label(text, relx, rely, anchor='w', font_size=label_font_size):
            label = tk.Label(
                container, text=text, bg="#1e1e1e", fg="white",
                font=("TkDefaultFont", font_size, "bold")
            )
            label.place(relx=relx, rely=rely, anchor=anchor)
            return label

        # Helper to create a value label and store reference
        def make_value_label(ref_key, val, relx, rely, anchor='w'):
            lbl = tk.Label(
                container, text=f"{val:.2f}", bg="#2e2e2e", fg="white",
                font=("TkDefaultFont", value_font_size, "bold")
            )
            lbl.place(relx=relx, rely=rely, anchor=anchor)
            self.label_refs[ref_key] = lbl

        current_y = 0.0

        # TITLE
        tk.Label(
            container, text="META QUEST 2", bg="#0068e6", fg="white",
            font=("TkDefaultFont", title_font_size, "bold")
        ).place(relx=0, rely=current_y, relwidth=1.0, anchor='nw')

        current_y += y_offset * 1.5

        # HEADSET SECTION
        make_label("➤ Headset", 0.05, current_y)
        current_y += y_offset
        for key in ["yaw", "pitch", "roll", "height"]:
            make_label(key.capitalize(), 0.1, current_y)
            make_value_label(key, self.values[key], 0.4, current_y)
            current_y += y_offset

        # BUTTONS SECTION
        current_y += y_offset_low  # Add a bit more space before next section
        make_label("➤ Buttons", 0.05, current_y)
        current_y += y_offset * 0.5

        btn_canvas_height = height_px * 0.13
        self.btn_canvas = tk.Canvas(
            container, bg="#1e1e1e",
            width=width_px, height=btn_canvas_height,
            highlightthickness=0
        )
        # Place the canvas at current_y relative to block height
        self.btn_canvas.place(x=0, y=int(height_px * current_y))

        # Button positioning calculations using relative width & height of the block
        section_width = width_px / 4

        # Scale positions proportionally to width and height
        x_L3 = section_width / 2 + section_width * 0.1
        x_YX = section_width + section_width / 2
        x_BA = 2 * section_width + section_width / 2
        x_R3 = 3 * section_width + section_width / 2 - section_width * 0.1

        vertical_margin = btn_canvas_height * 0.15
        row_height_segment = (btn_canvas_height - 2 * vertical_margin) / 2
        y_top = vertical_margin + row_height_segment / 2
        y_middle = btn_canvas_height / 2
        y_bottom = btn_canvas_height - vertical_margin - row_height_segment / 2

        def draw_button(label, is_circle, cx, cy):
            color = "#00cc44" if self.values["buttons"][label] else "#888888"

            # Base button size scales with block width and height (removed hard-coded limits)
            btn_size = self.style.button_size(0.028)

            if is_circle:
                size = int(btn_size * 1.5)
                shape_id = self.btn_canvas.create_oval(
                    cx - size//2, cy - size//2,
                    cx + size//2, cy + size//2,
                    fill=color, outline="black"
                )
            else:
                shape_id = self.btn_canvas.create_rectangle(
                    cx - btn_size//2, cy - btn_size//2,
                    cx + btn_size//2, cy + btn_size//2,
                    fill=color, outline="black"
                )
            self.btn_canvas.create_text(cx, cy, text=label, fill="white",
                                        font=("TkDefaultFont", label_font_size, "bold"))
            self.button_shapes[label] = shape_id

        draw_button("L3", True, x_L3, y_middle)
        draw_button("Y", False, x_YX, y_top)
        draw_button("X", False, x_YX, y_bottom)
        draw_button("B", False, x_BA, y_top)
        draw_button("A", False, x_BA, y_bottom)
        draw_button("R3", True, x_R3, y_middle)

        # JOYSTICKS SECTION
        current_y += btn_canvas_height / height_px + y_offset_low  # Advance by canvas height plus spacing
        make_label("➤ Joysticks", 0.05, current_y)

        # Left joystick
        make_label("LX", 0.1, current_y + y_offset)
        make_value_label("joystick_left.x", self.values["joystick_left"]["x"], 0.25, current_y + y_offset)
        make_label("LY", 0.1, current_y + y_offset * 2)
        make_value_label("joystick_left.y", self.values["joystick_left"]["y"], 0.25, current_y + y_offset * 2)

        # Right joystick
        make_label("RX", 0.5, current_y + y_offset)
        make_value_label("joystick_right.x", self.values["joystick_right"]["x"], 0.7, current_y + y_offset)
        make_label("RY", 0.5, current_y + y_offset * 2)
        make_value_label("joystick_right.y", self.values["joystick_right"]["y"], 0.7, current_y + y_offset * 2)

        current_y += y_offset * 3 + y_offset_low  # Advance by joystick height plus spacing

        # TRIGGERS AND GRIPS SECTION
        make_label("➤ Triggers and Grips", 0.05, current_y)
        current_y += y_offset

        make_label("LT", 0.1, current_y)
        make_value_label("LT", self.values["LT"], 0.25, current_y)
        make_label("LG", 0.1, current_y + y_offset)
        make_value_label("LG", self.values["LG"], 0.25, current_y + y_offset)
        make_label("RT", 0.5, current_y)
        make_value_label("RT", self.values["RT"], 0.7, current_y)
        make_label("RG", 0.5, current_y + y_offset)
        make_value_label("RG", self.values["RG"], 0.7, current_y + y_offset)

        # Schedule update
        self.schedule_periodic_update(self.update_rate_ms, self.update_values)

    def update_values(self):
        data = self.app.data_manager.oculus.data

        self.values["yaw"] = data.get("hmd_yaw", 0.0)
        self.values["pitch"] = data.get("hmd_pitch", 0.0)
        self.values["roll"] = data.get("hmd_roll", 0.0)
        self.values["height"] = data.get("hmd_height", 0.0)

        self.values["joystick_left"]["x"] = data.get("left_joystick_x", 0.0)
        self.values["joystick_left"]["y"] = data.get("left_joystick_y", 0.0)
        self.values["joystick_right"]["x"] = data.get("right_joystick_x", 0.0)
        self.values["joystick_right"]["y"] = data.get("right_joystick_y", 0.0)

        self.values["buttons"]["A"] = data.get("button_a", 0)
        self.values["buttons"]["B"] = data.get("button_b", 0)
        self.values["buttons"]["X"] = data.get("button_x", 0)
        self.values["buttons"]["Y"] = data.get("button_y", 0)
        self.values["buttons"]["L3"] = data.get("button_lt", 0)
        self.values["buttons"]["R3"] = data.get("button_rt", 0)

        self.values["LT"] = data.get("left_trigger", 0.0)
        self.values["RT"] = data.get("right_trigger", 0.0)
        self.values["LG"] = data.get("left_grip", 0.0)
        self.values["RG"] = data.get("right_grip", 0.0)

        # Update labels
        for key in ["yaw", "pitch", "roll", "height", "LT", "LG", "RT", "RG"]:
            if key in self.label_refs:
                self.label_refs[key].config(text=f"{self.values[key]:.2f}")

        for side in ["left", "right"]:
            for axis in ["x", "y"]:
                ref_key = f"joystick_{side}.{axis}"
                val = self.values[f"joystick_{side}"][axis]
                if ref_key in self.label_refs:
                    self.label_refs[ref_key].config(text=f"{val:.2f}")

        # Update button colors on canvas
        if self.btn_canvas:
            for btn, val in self.values["buttons"].items():
                shape_id = self.button_shapes.get(btn)
                if shape_id:
                    color = "#00cc44" if val else "#888888"
                    self.btn_canvas.itemconfig(shape_id, fill=color)

class KatvrDataBlock(BaseBlock):
    def __init__(self, app: ModularGUI, update_rate_ms=50):
        super().__init__(app, width_units=2, height_units=3)
        self.update_rate_ms = update_rate_ms
        self._after_id = None
        self.label_refs = {}

        # Map internal keys to display labels
        self.katvr_display_map = {
            'status': 'Status',
            'yaw_virtual': 'Virtual Yaw',
            'yaw': 'Yaw',
            'state': 'State',
            'velocity': 'Velocity',
            'forward_velocity': 'Forward Vel',
            'lateral_velocity': 'Lateral Vel',
        }

    def build(self, container, width_px, height_px):
        super().build(container, width_px, height_px)
        container.configure(bg="#1e1e1e")
        
        # Set up the font sizes and offsets
        style = self.app.style
        title_font_size = style.font_size("large")
        label_font_size = style.font_size("normal")
        value_font_size = style.font_size("small")
        y_offset = style.spacing_y("normal") * style.screen_height / height_px

        # Create the functions to create labels and value labels
        def make_label(text, relx, rely, anchor='w'):
            label = tk.Label(
                container, text=text, bg="#1e1e1e", fg="white", 
                font=("TkDefaultFont", label_font_size, "bold")
            )
            label.place(relx=relx, rely=rely, anchor=anchor)
            return label

        def make_value_label(ref_key, val, relx, rely, anchor='w'):
            formatted_val = self._format_value(ref_key, val)
            lbl = tk.Label(
                container, text=formatted_val, bg="#2e2e2e", fg="white", 
                font=("TkDefaultFont", value_font_size, "bold")
            )
            lbl.place(relx=relx, rely=rely, anchor=anchor)
            self.label_refs[ref_key] = lbl

        current_y = 0.0  # The relative Y position for placing elements

        # Title
        tk.Label(container, text="KATVR", bg="#049fb3", fg="white",
                 font=("TkDefaultFont", title_font_size, "bold")
        ).place(relx=0, rely=current_y, relwidth=1.0, anchor='nw')

        current_y += y_offset * 1.6

        # Data Section
        for data_key, display_name in self.katvr_display_map.items():
            make_label(display_name, relx=0.05, rely=current_y)
            make_value_label(ref_key=data_key, val=0.0, relx=0.5, rely=current_y)
            current_y += y_offset

        self.schedule_periodic_update(self.update_rate_ms, self.update_values)

    def update_values(self):
        values = self.app.data_manager.katvr.data
        for data_key, lbl in self.label_refs.items():
            val = values.get(data_key, '—')
            formatted_val = self._format_value(data_key, val)
            lbl.config(text=formatted_val)

    def _format_value(self, key, val):
        if 'yaw' in key and isinstance(val, (int, float)):
            return f"{int(val)}°"
        if 'status' in key and isinstance(val, bool):
            return "✅ Connected" if val is True else "⚠️ Disconnected"
        elif isinstance(val, (int, float)):
            return f"{val:.2f}"
        return val

class VideoDisplayBlock(BaseBlock):
    """
    A Tkinter block for displaying a video stream from an RTSP address.
    """
    def __init__(self, app, stream_address, using_stereo_camera=False):
        super().__init__(app, width_units=14, height_units=6)
        self.stream_address = stream_address
        self.using_stereo_camera = using_stereo_camera
        self.stop_event = threading.Event()  # Event to signal thread to stop
        self.frame_lock = threading.Lock() # Lock to protect self.frame access

        self.cap = None  # OpenCV VideoCapture object
        self.thread = None  # Thread for video capture
        self.frame = None  # Current frame from the video stream (NumPy array)
        self.photo = None  # Tkinter PhotoImage object for displaying the frame
        self.is_streaming = False # Flag to indicate if stream is actively playing
        self.message_text_id = None # ID for any displayed text message on canvas
        self._after_id = None # To store the ID of the scheduled _update_frame call
        self.connection_status = "DISCONNECTED" # States: "DISCONNECTED", "CONNECTING", "CONNECTED"
        self._reconnect_attempts = 0  # Track reconnection attempts for exponential backoff
        self._lost_frame_count = 0 # Counter for consecutive lost frames
        self.MAX_LOST_FRAMES = 5 # Threshold for declaring a lost connection (adjustable)
        self.orig_size = None # Store original frame size for scaling
        self._last_frame_timestamp = 0  # Time of the last valid frame received
        self.STALE_FRAME_TIMEOUT = 2.0  # seconds before considering a frame stale
        self.frame_delay = 33  # Default to ~30 FPS (33ms) until actual FPS is detected
        
        # Control source monitoring
        self._last_control_source = None
        self._source_monitor_after_id = None 

    def build(self, container, width_px, height_px):
        self.container = container
        self.container.configure(bg="black")

        self.canvas = tk.Canvas(
            self.container,
            bg="black",
            width=width_px,
            height=height_px,
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Immediately display "Stream Unavailable" until a frame is received
        # Using a small after delay to ensure canvas is somewhat ready
        self.app.after(10, self.show_message("🎥 Stream Unavailable"))

        # Start monitoring control source - will determine if stream should start
        self._monitor_control_source()
        # Don't schedule _update_frame here - let start_stream() handle it when needed
    
    def _monitor_control_source(self):
        """
        Monitors the control source and starts/stops stream based on whether source is PC.
        """
        try:
            current_source = self.app.data_manager.gui.source()
            
            # Check if source has changed
            if current_source != self._last_control_source:
                print(f"[GUI] [VideoDisplayBlock] Control source changed from {self._last_control_source} to {current_source}")
                
                # Start streaming when PC is selected OR when source is Unknown (initial state, PC is default)
                if current_source in ["PC", "Unknown"]:
                    if not self.is_streaming and (self.thread is None or not self.thread.is_alive()):
                        print(f"[GUI] [VideoDisplayBlock] Starting stream for {current_source} control source")
                        self.start_stream()
                else:
                    # Stop streaming for all other sources (OCULUS, KATVR, XBOX)
                    if self.is_streaming or (self.thread is not None and self.thread.is_alive()):
                        print(f"[GUI] [VideoDisplayBlock] Stopping stream for {current_source} control source")
                        self.stop_stream()
                        # Show a message indicating stream is closed for non-PC sources
                        self.app.after(10, lambda: self.show_message("🎥 Stream Closed\n(PC source required)"))
                
                self._last_control_source = current_source
        
        except Exception as e:
            print(f"[GUI] [VideoDisplayBlock] Error monitoring control source: {e}")
        
        # Schedule next monitor check
        self._source_monitor_after_id = self.app.after(200, self._monitor_control_source)

    def show_message(self, message, font=("Arial", 24), fill="white"):
        """
        General method to display a text message in the center of the screen.
        
        Args:
            message (str): The text message to display
            font (tuple): Font configuration (family, size)
            fill (str): Text color
        """
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        # If we are currently streaming and displaying frames, hide any message
        # EXCEPT when explicitly showing non-PC source message
        if self.is_streaming and self.frame is not None and "PC source required" not in message:
            self._hide_message()
            return

        # Ensure any previous video frame is cleared before displaying message
        self.canvas.delete("video_frame") # Delete image by its tag

        # Wait until canvas has actual dimensions
        if canvas_width < 10 or canvas_height < 10:
            self.app.after(50, lambda: self.show_message(message, font, fill))
            return

        if self.message_text_id is None:
            # Create the text if it doesn't exist
            self.message_text_id = self.canvas.create_text(
                canvas_width // 2,
                canvas_height // 2,
                text=message,
                fill=fill,
                font=font,
                justify=tk.CENTER,
                tags="message_text" # Tag for easy identification/deletion
            )
        else:
            # If text already exists, just update its content and position
            self.canvas.itemconfigure(self.message_text_id, text=message)
            self.canvas.coords(
                self.message_text_id,
                canvas_width // 2,
                canvas_height // 2
            )
        self.canvas.tag_raise("message_text") # Ensure text is always on top

    def _hide_message(self):
        """
        Hides any displayed text message from the canvas by deleting it.
        """
        if self.message_text_id is not None:
            self.canvas.delete(self.message_text_id)
            self.message_text_id = None

    def start_stream(self):
        """
        Starts the video capture thread.
        Ensures only one thread is running at a time.
        """
        if self.thread is None or not self.thread.is_alive():
            self.stop_event.clear() # Clear stop event to allow thread to run
            self.thread = threading.Thread(target=self._stream_loop, daemon=True)
            self.thread.start()
            
            # Always ensure _update_frame is running (cancel any existing one first)
            if self._after_id is not None:
                try:
                    self.app.after_cancel(self._after_id)
                except:
                    pass  # Ignore if already cancelled
            self._after_id = self.app.after(100, self._update_frame)
            
            if self.connection_status != "CONNECTING":
                print(f"[GUI] [VideoDisplayBlock] Attempting to connect to stream: {self.stream_address}")
                self.connection_status = "CONNECTING"
                # Schedule UI update on main thread to show "Stream Unavailable" immediately
                self.app.after(0, lambda: self.show_message("🎥 Stream Unavailable"))

    def stop_stream(self):
        """
        Signals the video capture thread to stop and waits for it to finish.
        Releases the OpenCV VideoCapture object.
        """
        print("[GUI] [VideoDisplayBlock] Stopping stream...")
        self.stop_event.set() # Set stop event to signal thread to exit loop

        # Cancel any pending _update_frame calls
        if self._after_id is not None:
            try:
                self.app.after_cancel(self._after_id)
            except Exception as e:
                # This can happen if the after call already fired or was already cancelled
                print(f"[GUI] [VideoDisplayBlock] Error canceling _update_frame scheduled call: {e}")
            self._after_id = None

        # Wait for the stream thread to terminate gracefully
        if self.thread and self.thread.is_alive():
            print("[GUI] [VideoDisplayBlock] Waiting for stream thread to stop...")
            self.thread.join(timeout=2.0) # Give it 2 seconds to clean up
            if self.thread.is_alive():
                print("[GUI] [VideoDisplayBlock] Warning: Stream thread did not terminate gracefully within timeout.")

        # Release the OpenCV capture object
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
            print("[GUI] [VideoDisplayBlock] OpenCV VideoCapture released.")

        self.is_streaming = False
        # Update connection status for logging
        if self.connection_status != "DISCONNECTED":
            print("[GUI] [VideoDisplayBlock] Stream stopped.")
            self.connection_status = "DISCONNECTED"
        
        # Clear the frame to ensure no stale data
        with self.frame_lock:
            self.frame = None
        
        # Clear canvas completely
        self.canvas.delete("all")
        self.tk_image = None  # Clear image reference
        self.orig_size = None  # Reset frame size detection
        self._last_frame_timestamp = 0  # Reset timestamp
        
        # Ensure "Stream Unavailable" is shown and any last frame is cleared on stop
        self.app.after(0, lambda: self.show_message("🎥 Stream Unavailable")) # Show text on main thread

    def _initial_connection(self):
        """
        Attempts to establish an initial connection to the RTSP stream.
        Returns True on success, False otherwise.
        """
        # Release any existing capture object before creating a new one
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None

        # Try to open the stream
        self.cap = cv2.VideoCapture(self.stream_address)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Set a shorter connection timeout to avoid the 30-second wait
        self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)  # 5 second timeout

        if not self.cap.isOpened():
            print(f"[GUI] [VideoDisplayBlock] Failed to open stream: {self.stream_address}")
            return False

        # Attempt to read a frame immediately to verify actual data flow
        # Some RTSP streams might report `isOpened()` as true but fail to read.
        ret, test_frame = self.cap.read()
        if not ret or test_frame is None:
            print(f"[GUI] [VideoDisplayBlock] Connected but failed to read initial frame from {self.stream_address}.")
            self.cap.release()
            self.cap = None
            return False

        print(f"[GUI] [VideoDisplayBlock] Successfully connected to stream: {self.stream_address}")
        
        # Set FPS and frame delay based on stream properties or default
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        # Check for invalid/NaN/zero FPS. If invalid, default to 30 FPS.
        if not self.fps or self.fps != self.fps or self.fps < 1.0:
            self.fps = 30.0
        self.frame_delay = int(1000 / self.fps) # Delay in milliseconds

        # Initialize orig_size with the first valid frame's dimensions
        if self.using_stereo_camera:
            height, width = test_frame.shape[:2]
            self.orig_size = (width // 2, height)
        else:
            self.orig_size = (test_frame.shape[1], test_frame.shape[0])
            
        # Store the first valid frame
        with self.frame_lock:
            self.frame = test_frame
            
        return True

    def _stream_loop(self):
        """
        Main streaming loop that runs in a separate thread.
        Handles connection, reconnection, and frame reading.
        """
        while not self.stop_event.is_set():
            if self.cap is None or not self.cap.isOpened():
                self.is_streaming = False
                
                # Only log connection attempt message once per connection cycle
                if self.connection_status != "CONNECTING":
                    print(f"[GUI] [VideoDisplayBlock] Attempting to connect to stream: {self.stream_address} (attempt {self._reconnect_attempts + 1})")
                    self.connection_status = "CONNECTING"
                    # Schedule UI update on main thread to show "Stream Unavailable"
                    self.app.after(0, lambda: self.show_message("🎥 Stream Unavailable"))

                success = self._initial_connection()
                if not success:
                    self._reconnect_attempts = min(self._reconnect_attempts + 1, 5) # Cap at 5 attempts for delay calc
                    # Exponential back-off, capped at 5 seconds
                    delay = min(self._reconnect_attempts, 5) # Delay increases 1, 2, 3, 4, 5 seconds
                    print(f"[GUI] [VideoDisplayBlock] Connection failed. Retrying in {delay} seconds...")
                    self.app.after(0, lambda: self.show_message("🎥 Stream Unavailable"))
                    
                    # Use stop_event.wait() instead of time.sleep() to allow early termination
                    if self.stop_event.wait(delay):
                        break  # If stop_event was set during wait, break out
                    continue # Try connecting again
                else:
                    self.connection_status = "CONNECTED"
                    self._reconnect_attempts = 0  # Reset counter on successful connection
                    self._lost_frame_count = 0 # Reset lost frame counter
                    self.is_streaming = True
                    # Schedule UI update on main thread to hide "Stream Unavailable"
                    self.app.after(0, self._hide_message)

            try:
                ret, frame = self.cap.read()
                
                # Check for stop event again in case it was set during cap.read()
                if self.stop_event.is_set():
                    break

                if ret and frame is not None:
                    with self.frame_lock:
                        self.frame = frame # Update shared frame
                        self._last_frame_timestamp = time.perf_counter()  # Mark time of last valid frame
                    self._lost_frame_count = 0 # Reset lost frame counter on successful read
                    self.is_streaming = True
                    # If we just reconnected, log it and hide text
                    if self.connection_status != "CONNECTED":
                        print("[GUI] [VideoDisplayBlock] Stream reconnected and frame received.")
                        self.connection_status = "CONNECTED"
                        self._reconnect_attempts = 0
                        self.app.after(0, self._hide_message) # Ensure text is hidden
                else:
                    # Frame read failed or frame is None
                    self._lost_frame_count += 1
                    if self._lost_frame_count <= 3:  # Only log first few failures to avoid spam
                        print(f"[GUI] [VideoDisplayBlock] Failed to read frame (consecutive: {self._lost_frame_count}).")

                    if self._lost_frame_count >= self.MAX_LOST_FRAMES:
                        print("[GUI] [VideoDisplayBlock] Max lost frames reached. Declaring connection lost. Retrying...")
                        self.is_streaming = False
                        self.app.after(0, lambda: self.show_message("🎥 Stream Unavailable")) # Show unavailable text
                        
                        if self.connection_status == "CONNECTED": # Only log state change once
                            self.connection_status = "DISCONNECTED"
                        
                        # Release the capture object to force a full re-initialization
                        if self.cap:
                            self.cap.release()
                            self.cap = None
                        
                        self._lost_frame_count = 0 # Reset for next connection attempt
                        if self.stop_event.wait(0.1):  # Short delay before next loop iteration
                            break
                        continue # Continue to the connection block at the top of the loop
                    else:
                        # If just a few frames are lost, keep trying without full reconnection
                        if self.stop_event.wait(0.01):  # Small delay to prevent busy-waiting
                            break
                        continue # Continue to next iteration to try reading another frame

            except Exception as e:
                print(f"[GUI] [VideoDisplayBlock] Exception in stream loop: {e}")
                self.is_streaming = False
                self._lost_frame_count = 0 # Reset on critical error
                self.app.after(0, lambda: self.show_message("🎥 Stream Unavailable")) # Show unavailable text

                if self.cap:
                    self.cap.release()
                    self.cap = None
                
                if self.connection_status == "CONNECTED":
                    self.connection_status = "DISCONNECTED"
                
                # Longer delay after an unexpected exception before retrying
                if self.stop_event.wait(2):
                    break

        # Cleanup after the loop exits
        if self.cap:
            self.cap.release()
            self.cap = None
        print("[GUI] [VideoDisplayBlock] Stream loop terminated.")

    def _update_frame(self):
        if self.stop_event.is_set():
            if self._after_id is not None:
                try:
                    self.app.after_cancel(self._after_id)
                except Exception as e:
                    print(f"[GUI] Error canceling _update_frame during stop: {e}")
                self._after_id = None
            return

        current_frame = None
        with self.frame_lock:
            if self.frame is not None:
                current_frame = self.frame.copy()

        now = time.perf_counter()
        if now - self._last_frame_timestamp > self.STALE_FRAME_TIMEOUT:
            self.is_streaming = False
            self.show_message("🎥 Stream Unavailable")

        elif current_frame is not None:
            try:
                self._hide_message()

                frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
                if self.using_stereo_camera:
                    height, width = frame_rgb.shape[:2]
                    frame_rgb = frame_rgb[:, :width // 2]

                if self.orig_size is None:
                    self.orig_size = (frame_rgb.shape[1], frame_rgb.shape[0])

                canvas_width = self.canvas.winfo_width()
                canvas_height = self.canvas.winfo_height()
                if canvas_width < 10 or canvas_height < 10:
                    raise ValueError("Canvas size too small")

                orig_w, orig_h = self.orig_size
                if orig_w == 0 or orig_h == 0:
                    raise ValueError("Original frame size is zero")

                # Stretch video to fill entire canvas (no aspect ratio preservation)
                new_w, new_h = canvas_width, canvas_height
                resized_frame = cv2.resize(frame_rgb, (new_w, new_h), interpolation=cv2.INTER_AREA)
                image_pil = Image.fromarray(resized_frame)
                self.tk_image = ImageTk.PhotoImage(image=image_pil)

                self.canvas.delete("video_frame")
                # Position at top-left corner (0, 0) since we're filling the entire canvas
                self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image, tags="video_frame")

                self.is_streaming = True

            except Exception as e:
                print(f"[GUI] [VideoDisplayBlock] Error displaying frame: {e}")
                self.is_streaming = False
                self.show_message("🎥 Stream Unavailable")

        else:
            if self.is_streaming:
                print("[GUI] No frame available. Displaying 'Stream Unavailable'.")
            self.is_streaming = False
            self.show_message("🎥 Stream Unavailable")

        # Schedule next update - use longer delay when no stream is active
        delay = int(self.frame_delay) if self.is_streaming else 100  # 100ms when no stream
        self._after_id = self.app.after(delay, self._update_frame)

    def destroy(self):
        """
        Overrides BaseBlock's destroy method to stop the video stream thread
        when the block is no longer needed.
        """
        print("[GUI] Destroying VideoDisplayBlock...")
        
        # Cancel source monitoring
        if self._source_monitor_after_id is not None:
            try:
                self.app.after_cancel(self._source_monitor_after_id)
            except Exception as e:
                print(f"[GUI] Error canceling source monitoring: {e}")
            self._source_monitor_after_id = None
        
        self.stop_stream() # Ensure the stream and thread are properly shut down
        super().destroy()

class JoystickV1Block(BaseBlock):
    """A Tkinter block that simulates a virtual 2D joystick with mouse and keyboard controls."""
    def __init__(self, app, key_config: str ="wasd", name: str = "Joystick"):
        super().__init__(app, width_units=2, height_units=3)
        self.name: str = name
        self.key_map: dict = {}
        # Set the key mapping based on the configuration
        if key_config == "arrows":
            self.key_map = {
                "up": "up",
                "down": "down",
                "left": "left",
                "right": "right"
            }
        else:
            self.key_map = {
                "up": "w",
                "down": "s",
                "left": "a",
                "right": "d"
            }

    def build(self, container, width_px, height_px):
        super().build(container, width_px, height_px)
        container.configure(bg="#000000")
        style = self.app.style

        screen_h = style.screen_height
        title_font_size = style.font_size("large")
        value_font_size = style.font_size("normal")
        y_offset_abs = style.spacing_y("normal")
        y_offset_rel = y_offset_abs * screen_h / height_px

        # === Title ===
        tk.Label(
            container,
            text=f"{self.name}",
            bg="#222244",
            fg="white",
            font=("TkDefaultFont", title_font_size, "bold")
        ).place(relx=0, rely=0, relwidth=1.0, relheight=0.18)

        # === Joystick Canvas ===
        canvas_h_rel = 0.69
        canvas_h = int(height_px * canvas_h_rel)

        self.center_x = width_px // 2
        self.center_y = int(height_px * 0.33)
        self.joystick_radius = min(width_px, canvas_h) // 2 - 10
        self.knob_radius = self.joystick_radius // 5
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.active_keys = set()

        canvas = tk.Canvas(
            container,
            width=width_px,
            height=canvas_h,
            bg="#1e1e1e",
            highlightthickness=0
        )
        canvas.place(x=0, y=int(height_px * 0.18))

        # === XY Value Label ===
        xy_label = tk.Label(
            container,
            text="X: 0.00   Y: 0.00",
            bg="#222244",
            fg="white",
            font=("TkDefaultFont", value_font_size, "bold")
        )
        xy_label.place(relx=0, rely=0.85, relwidth=1.0, relheight=0.15)

        # === Joystick Drawing ===
        def draw_joystick():
            canvas.delete("all")
            # Outer circle
            canvas.create_oval(
                self.center_x - self.joystick_radius, self.center_y - self.joystick_radius,
                self.center_x + self.joystick_radius, self.center_y + self.joystick_radius,
                fill="#2e2e4d", outline="#6666aa", width=4
            )
            # Knob
            knob_cx = self.center_x + int(self.joystick_x * self.joystick_radius)
            knob_cy = self.center_y - int(self.joystick_y * self.joystick_radius)
            canvas.create_oval(
                knob_cx - self.knob_radius, knob_cy - self.knob_radius,
                knob_cx + self.knob_radius, knob_cy + self.knob_radius,
                fill="#00cc44", outline="#222", width=2
            )
            # Cross lines
            canvas.create_line(self.center_x, self.center_y - self.joystick_radius,
                               self.center_x, self.center_y + self.joystick_radius,
                               fill="#444", dash=(2, 2))
            canvas.create_line(self.center_x - self.joystick_radius, self.center_y,
                               self.center_x + self.joystick_radius, self.center_y,
                               fill="#444", dash=(2, 2))

            xy_label.config(text=f"X: {self.joystick_x:.2f}   Y: {self.joystick_y:.2f}")

        draw_joystick()

        def set_joystick(x, y):
            mag = math.hypot(x, y)
            if mag > 1:
                x /= mag
                y /= mag
            self.joystick_x = x
            self.joystick_y = y
            draw_joystick()

        def canvas_to_joystick(event):
            dx = event.x - self.center_x
            dy = self.center_y - event.y
            norm_x = dx / self.joystick_radius
            norm_y = dy / self.joystick_radius
            set_joystick(norm_x, norm_y)

        def on_mouse_drag(event): canvas_to_joystick(event)
        def on_mouse_click(event): canvas_to_joystick(event)
        def on_mouse_release(event): set_joystick(0, 0)

        canvas.bind("<B1-Motion>", on_mouse_drag)
        canvas.bind("<Button-1>", on_mouse_click)
        canvas.bind("<ButtonRelease-1>", on_mouse_release)

        # === Keyboard Input Handling ===
        def update_keys():
            x = 0
            y = 0
            if self.key_map["left"] in self.active_keys and self.key_map["right"] not in self.active_keys:
                x -= 1
            if self.key_map["right"] in self.active_keys and self.key_map["left"] not in self.active_keys:
                x += 1
            if self.key_map["up"] in self.active_keys and self.key_map["down"] not in self.active_keys:
                y += 1
            if self.key_map["down"] in self.active_keys and self.key_map["up"] not in self.active_keys:
                y -= 1
            set_joystick(x, y)

        def on_key_press(event):
            key = event.keysym.lower()
            if key in self.key_map.values():
                self.active_keys.add(key)
                update_keys()

        def on_key_release(event):
            key = event.keysym.lower()
            if key in self.key_map.values():
                self.active_keys.discard(key)
                update_keys()

        container.bind_all("<KeyPress>", on_key_press, add="+")
        container.bind_all("<KeyRelease>", on_key_release, add="+")

    def get_data(self):
        """Returns the current joystick position as a dictionary."""
        return {
            "x": self.joystick_x,
            "y": self.joystick_y
        }

class JoystickV2Block(BaseBlock):
    """A Tkinter block that simulates a square 2D joystick controlled by the right mouse button."""
    def __init__(self, app, name="Square Joystick"):
        super().__init__(app, width_units=2, height_units=3)
        self.name = name

    def build(self, container, width_px, height_px):
        super().build(container, width_px, height_px)
        container.configure(bg="#000000")
        style = self.app.style

        title_font_size = style.font_size("large")
        value_font_size = style.font_size("normal")

        padding_px = int(height_px * 0.02)
        canvas_h_rel = 0.69
        canvas_h = int(height_px * canvas_h_rel)

        self.range_limit = 0.55
        self.square_size = min(width_px, canvas_h) - padding_px - 20
        self.knob_radius = self.square_size // 10
        self.center_x = width_px // 2
        self.center_y = int(height_px * 0.33)
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.tracking_active = False

        # === Title ===
        tk.Label(
            container,
            text=f"{self.name}",
            bg="#222244",
            fg="white",
            font=("TkDefaultFont", title_font_size, "bold")
        ).place(relx=0, rely=0, relwidth=1.0, relheight=0.18)

        # === Canvas ===
        self.canvas = tk.Canvas(
            container,
            width=width_px,
            height=canvas_h,
            bg="#1e1e1e",
            highlightthickness=0
        )
        self.canvas.place(x=0, y=int(height_px * 0.18))

        # === XY Value Label ===
        xy_label = tk.Label(
            container,
            text="X: 0.00   Y: 0.00",
            bg="#222244",
            fg="white",
            font=("TkDefaultFont", value_font_size, "bold")
        )
        xy_label.place(relx=0, rely=0.85, relwidth=1.0, relheight=0.15)

        # === Draw Joystick ===
        def draw_joystick():
            self.canvas.delete("all")
            half_size = self.square_size // 2

            # Square
            self.canvas.create_rectangle(
                self.center_x - half_size, self.center_y - half_size,
                self.center_x + half_size, self.center_y + half_size,
                fill="#2e2e4d", outline="#6666aa", width=4
            )

            # Crosshairs
            self.canvas.create_line(
                self.center_x, self.center_y - half_size,
                self.center_x, self.center_y + half_size,
                fill="#444", dash=(2, 2)
            )
            self.canvas.create_line(
                self.center_x - half_size, self.center_y,
                self.center_x + half_size, self.center_y,
                fill="#444", dash=(2, 2)
            )

            # Knob
            knob_x = self.center_x + (self.joystick_x / self.range_limit) * half_size
            knob_y = self.center_y - (self.joystick_y / self.range_limit) * half_size
            self.canvas.create_oval(
                knob_x - self.knob_radius, knob_y - self.knob_radius,
                knob_x + self.knob_radius, knob_y + self.knob_radius,
                fill="#00cc44", outline="#222", width=2
            )

            # Label
            xy_label.config(text=f"X: {self.joystick_x:.2f}   Y: {self.joystick_y:.2f}")

        # === Input Handlers ===
        def on_right_click(event):
            self.tracking_active = not self.tracking_active
            if not self.tracking_active:
                self.joystick_x = 0.0
                self.joystick_y = 0.0
            draw_joystick()

        def on_motion(event=None):
            if not self.tracking_active:
                return

            # Mouse position
            mouse_x = self.canvas.winfo_pointerx()
            mouse_y = self.canvas.winfo_pointery()

            canvas_x = self.canvas.winfo_rootx()
            canvas_y = self.canvas.winfo_rooty()

            rel_x = mouse_x - canvas_x
            rel_y = mouse_y - canvas_y

            half_size = self.square_size // 2

            norm_x = (rel_x - self.center_x) / half_size
            norm_y = (self.center_y - rel_y) / half_size  # Y flipped

            norm_x = max(-1, min(1, norm_x))
            norm_y = max(-1, min(1, norm_y))

            self.joystick_x = norm_x * self.range_limit
            self.joystick_y = norm_y * self.range_limit

            draw_joystick()

        # === Bindings ===
        self.canvas.bind("<Button-3>", on_right_click)
        self.app.bind_all("<Motion>", on_motion, add="+")

        draw_joystick()

    def get_data(self):
        """Returns the current joystick position as a dictionary."""
        return {
            "x": self.joystick_x,
            "y": self.joystick_y
        }

class ControlSourceBlock(BaseBlock):
    """
    A 4x1 Tkinter block that allows users to select a control source (PC, OCULUS, KATVR, XBOX) using buttons.
    It supports keyboard shortcuts (1, 2, 3, 4) for quick selection.
    """
    def __init__(self, app, name: str = "Source"):
        super().__init__(app, width_units=3, height_units=1)
        self.name = name
        self.sources = ["PC", "OCULUS", "KATVR", "XBOX"]
        self.active_index = 0
        self.state = {src: (i == self.active_index) for i, src in enumerate(self.sources)}
        self.buttons = {}

    def build(self, container, width_px, height_px):
        container.configure(bg="#181818")
        font_size = self.style.font_size("large")

        # Title
        title = tk.Label(
            container, text="Select Control Source", bg="#223344", fg="white", 
            font=("TkDefaultFont", font_size, "bold")
        )
        title.place(relx=0, rely=0, relwidth=1.0, relheight=0.25)

        # Button sizing and layout for 4 buttons
        btn_h = int(height_px * 0.5)
        btn_w = int(width_px * 0.19)  # Reduced width to fit 4 buttons
        btn_y = int(height_px * 0.35)
        spacing = int(width_px * 0.04)  # Reduced spacing

        total_btns_width = btn_w * 4 + spacing * 3
        start_x = (width_px - total_btns_width) // 2

        for i, src in enumerate(self.sources):
            x = start_x + i * (btn_w + spacing)
            btn = tk.Button(
                container,
                text=src,
                font=("TkDefaultFont", font_size, "bold"),
                width=1,
                height=1,
                bg="#00cc44" if self.state[src] else "#444444",
                fg="white",
                activebackground="#00cc44",
                relief="raised",
                bd=3,
                command=lambda idx=i: self.set_active(idx)
            )
            btn.place(x=x, y=btn_y, width=btn_w, height=btn_h)
            self.buttons[src] = btn

        # Keyboard bindings
        def on_key(event):
            if event.keysym == "1":
                self.set_active(0)
            elif event.keysym == "2":
                self.set_active(1)
            elif event.keysym == "3":
                self.set_active(2)
            elif event.keysym == "9":
                self.set_active(3)
        container.bind_all("<KeyPress>", on_key, add="+")

    def set_active(self, idx):
        self.active_index = idx
        for i, src in enumerate(self.sources):
            self.state[src] = (i == idx)
            if src in self.buttons:
                self.buttons[src].configure(bg="#00cc44" if self.state[src] else "#444444")

    def get_data(self):
        """
        Returns a dictionary with the current bool value for each control source.
        Only the active source will be True, others False.
        """
        return dict(self.state)

class CommandControlBlock(BaseBlock):
    """
    A 4x2 block with independently toggled command buttons.
    Includes two groups: 'For SPOT' and 'For Calibration'.
    """
    PRESS_DURATION_SPOT_MS = 150  # Time a SPOT button remains 'active'
    PRESS_DURATION_CALIB_MS = 40  # Time a Calibration button remains 'active'

    def __init__(self, app, name: str = "Commands"):
        super().__init__(app, width_units=3, height_units=2)
        self.name = name
        self.commands_spot = ["Shutdown", "Start", "Sit", "Stand"]
        self.commands_calib = ["Calibrate: HMD Only", "Calibrate: HMD+KATVR"]

        # Initialize states for all buttons
        self.state = {cmd: False for cmd in self.commands_spot + self.commands_calib}
        self.buttons = {}

    def build(self, container, width_px, height_px):
        container.configure(bg="#181818")
        font_size = self.style.font_size("large")

        # Title
        title = tk.Label(
            container, text="Commands", bg="#223344", fg="white", 
            font=("TkDefaultFont", font_size, "bold")
        )
        title.place(relx=0, rely=0, relwidth=1.0, relheight=0.129)

        # Button grid for SPOT (2x2)
        btn_w = width_px * 0.43
        btn_h = height_px * 0.18
        spacing_x = width_px * 0.05
        spacing_y = height_px * 0.02
        start_x = width_px * 0.05
        start_y = height_px * 0.18

        for i, cmd in enumerate(self.commands_spot):
            row = i // 2
            col = i % 2
            x = start_x + col * (btn_w + spacing_x)
            y = start_y + row * (btn_h + spacing_y)
            
            bg_color = "#cc0000" if cmd == "Shutdown" else "#444444"

            btn = tk.Button(
                container,
                text=cmd,
                font=("TkDefaultFont", font_size, "bold"),
                bg=bg_color,
                fg="white",
                activebackground="#ff3333" if cmd == "Shutdown" else "#00cc44",
                relief="raised",
                bd=3,
                command=lambda c=cmd: self.toggle_button(c)
            )
            btn.place(x=x, y=y, width=btn_w, height=btn_h)
            self.buttons[cmd] = btn

        # Vertical buttons for Calibration
        calib_btn_w = (width_px * 0.86) + spacing_x
        calib_btn_h = height_px * 0.16
        calib_start_x = width_px * 0.05
        calib_start_y = height_px * 0.6

        for i, cmd in enumerate(self.commands_calib):
            y = calib_start_y + i * (calib_btn_h + spacing_y)
            btn = tk.Button(
                container,
                text=cmd,
                font=("TkDefaultFont", font_size, "bold"),
                bg="#444444",
                fg="white",
                activebackground="#00cc44",
                relief="raised",
                bd=3,
                command=lambda c=cmd: self.toggle_button(c)
            )
            btn.place(x=calib_start_x, y=y, width=calib_btn_w, height=calib_btn_h)
            self.buttons[cmd] = btn

        # Keybindings for Stand (E) and Sit (Q)
        def on_key(event):
            key = event.keysym.lower()
            if key == "e":
                self.toggle_button("Stand")
            elif key == "q":
                self.toggle_button("Sit")
        container.bind_all("<KeyPress>", on_key, add="+")

    def toggle_button(self, cmd):
        # Set state True
        self.state[cmd] = True
        if cmd == "Shutdown":
            self.buttons[cmd].configure(bg="#ff3333")  # brighter red when active
        else:
            self.buttons[cmd].configure(bg="#00cc44")
        
        # Determine duration based on command type
        if cmd in self.commands_spot:
            duration_ms = self.PRESS_DURATION_SPOT_MS
        else:  # calibration commands
            duration_ms = self.PRESS_DURATION_CALIB_MS
        
        # Use threading to reset after delay
        def reset():
            threading.Timer(duration_ms / 1000.0, lambda: self.reset_button(cmd)).start()
        reset()

    def reset_button(self, cmd):
        self.state[cmd] = False
        if cmd in self.buttons:
            if cmd == "Shutdown":
                self.buttons[cmd].configure(bg="#cc0000")  # back to dark red
            else:
                self.buttons[cmd].configure(bg="#444444")

    def get_data(self):
        """
        Returns a dictionary with the current state of all command buttons.
        Keys are command names, values are booleans.
        """
        return dict(self.state)

class PIDGraphBlock(BaseBlock):
    MAX_POINTS = 100

    def __init__(self, app):
        super().__init__(app, width_units=5, height_units=3)
        self.required_yaw_values = []
        self.current_yaw_values = []
        self.required_yaw_loops = 0
        self.current_yaw_loops = 0
        self.last_required_yaw = None
        self.last_current_yaw = None
        self.time_step = 0
        self.active = False

    @staticmethod
    def normalize_angle(angle):
        while angle >= math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def build(self, container, width_px, height_px):
        self.container = container
        self.canvas_width = width_px
        self.canvas_height = height_px

        style = self.app.style
        self.padding_px = style.scaled_size(0.04)  # Increased padding to 4%
        self.offset_small = style.scaled_size(0.005)
        self.label_height_px = style.scaled_size(0.025)

        self.font_small = ("TkDefaultFont", style.font_size("small"))
        self.font_normal = ("TkDefaultFont", style.font_size("normal"))
        self.font_large = ("TkDefaultFont", style.font_size("large"))

        self.title_label = tk.Label(
            container,
            text="SPOT-KATVR PID Response",
            bg="gray",
            fg="white",
            font=self.font_large
        )
        self.title_label.pack(fill="x")

        self.canvas = tk.Canvas(
            container,
            bg="white",
            width=width_px,
            height=height_px - self.label_height_px
        )
        self.canvas.pack()

        self.schedule_periodic_update(50, self.periodic_update)

    def get_tiled_yaw(self, current, previous, prev_loop_count):
        if previous is None:
            return 0, current
        delta = current - previous
        if delta > math.pi:
            prev_loop_count -= 1
        elif delta < -math.pi:
            prev_loop_count += 1
        full_angle = prev_loop_count * 2 * math.pi + current
        return prev_loop_count, full_angle

    def update_yaw(self, required_yaw, current_yaw, active=False):
        self.active = active
        if not self.active:
            return

        required_yaw = self.normalize_angle(required_yaw)
        current_yaw = self.normalize_angle(current_yaw)

        if self.time_step == 0:
            r_loops, r_full = 0, required_yaw
            c_loops, c_full = 0, current_yaw
        else:
            r_loops, r_full = self.get_tiled_yaw(required_yaw, self.last_required_yaw, self.required_yaw_loops)
            c_loops, c_full = self.get_tiled_yaw(current_yaw, self.last_current_yaw, self.current_yaw_loops)

        self.required_yaw_values.append(r_full)
        self.current_yaw_values.append(c_full)

        self.required_yaw_loops = r_loops
        self.current_yaw_loops = c_loops
        self.last_required_yaw = required_yaw
        self.last_current_yaw = current_yaw

        if len(self.required_yaw_values) > self.MAX_POINTS:
            self.required_yaw_values.pop(0)
            self.current_yaw_values.pop(0)

    def periodic_update(self):
        spot_data = self.app.data_manager.spot.data
        if spot_data and "required_yaw" in spot_data and "current_yaw" in spot_data:
            required_yaw = spot_data.get("required_yaw", 0.0)
            current_yaw = spot_data.get("current_yaw", 0.0)
            if required_yaw is not None and current_yaw is not None:
                self.update_yaw(required_yaw, current_yaw, active=True)
        else:
            self.active = False
        self.draw_graph()
        self.time_step += 1

    def draw_graph(self):
        self.canvas.delete("all")

        if not self.active:
            self.canvas.create_text(
                self.canvas_width // 2,
                self.canvas_height // 2 - 15,
                text="ⓘ Not in use",
                font=self.font_large,
                fill="gray",
                anchor="center"
            )
            return

        if not self.required_yaw_values or not self.current_yaw_values:
            return

        all_values = self.required_yaw_values + self.current_yaw_values
        min_y = min(all_values)
        max_y = max(all_values)

        # Fix 2: Adjust zoom for steady state
        yaw_range = max(max_y - min_y, 0.1) # Ensure a minimum range
        min_margin_radians = math.pi / 4 # Fixed minimum margin (e.g., pi/4 radians)
        
        # Calculate dynamic margin, but ensure it's at least min_margin_radians
        margin = max(yaw_range * 0.1, min_margin_radians) 
        
        min_y -= margin
        max_y += margin

        scale_y = (self.canvas_height - 2 * self.padding_px) / (max_y - min_y)
        scale_x = (self.canvas_width - 2 * self.padding_px) / self.MAX_POINTS

        # Grid lines for multiples of pi
        tile_height = 2 * math.pi
        tile_start = math.floor(min_y / tile_height) - 1
        tile_end = math.ceil(max_y / tile_height) + 1

        # Fix 3: Increase amount of angle values shown in the y-axis
        # Added more fractional pi values
        angle_fractions = [
            (0.0, "0"),
        ]

        for t in range(tile_start, tile_end + 1):
            base = t * tile_height
            for frac, label in angle_fractions:
                val = base + frac * tile_height
                # Only draw if the value is within the current visible range
                if min_y <= val <= max_y:
                    y = self.get_canvas_y(val, min_y, scale_y)
                    self.canvas.create_line(self.padding_px, y, self.canvas_width - self.padding_px, y, fill="#eee")
                    self.canvas.create_text(self.padding_px - self.offset_small, y, text=label, anchor="e", font=self.font_small)

        # X axis lines
        for i in range(0, self.MAX_POINTS, 20):
            x = self.get_canvas_x(i)
            self.canvas.create_line(x, self.padding_px, x, self.canvas_height - self.padding_px, fill="#f5f5f5")
            self.canvas.create_text(x, self.canvas_height - self.padding_px + self.offset_small, text=str(i), anchor="n", font=self.font_small)

        # Main axes
        y0 = self.get_canvas_y(0, min_y, scale_y)
        self.canvas.create_line(self.padding_px, y0, self.canvas_width - self.padding_px, y0, fill="gray")
        self.canvas.create_line(self.padding_px, self.padding_px, self.padding_px, self.canvas_height - self.padding_px, fill="gray")

        # Axis labels
        self.canvas.create_text(self.canvas_width // 2, self.canvas_height - 50,
                                 text="Time (steps)", font=self.font_normal)
        self.canvas.create_text(10, self.canvas_height // 2,
                                 text="Yaw (rad)", angle=90, font=self.font_normal)

        # Data lines
        self.draw_line(self.required_yaw_values, "blue", min_y, scale_y)
        self.draw_line(self.current_yaw_values, "red", min_y, scale_y)

    def draw_line(self, data, color, min_y, scale_y):
        for i in range(1, len(data)):
            x1 = self.get_canvas_x(i - 1)
            x2 = self.get_canvas_x(i)
            y1 = self.get_canvas_y(data[i - 1], min_y, scale_y)
            y2 = self.get_canvas_y(data[i], min_y, scale_y)
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=2)

    def get_canvas_x(self, index):
        return self.padding_px + index * (self.canvas_width - 2 * self.padding_px) / self.MAX_POINTS

    def get_canvas_y(self, value, min_y, scale_y):
        # Y-axis is inverted in canvas, so subtract from height
        return self.canvas_height - self.padding_px - (value - min_y) * scale_y

class DynamicDictBlock(BaseBlock):
    def __init__(self, app: ModularGUI, title: str, data_fn: callable, update_rate_ms=100):
        super().__init__(app, width_units=2, height_units=3)
        self.title = title
        self.data_fn = data_fn
        self.update_rate_ms = update_rate_ms
        self.value_labels_container = None
        self._last_data_snapshot = {}
        self._value_labels = {} # New: To store references to value labels

    def build(self, container, width_px, height_px):
        super().build(container, width_px, height_px)
        container.configure(bg="#1e1e1e")

        style = self.app.style
        title_font_size = style.font_size("large")

        # Title label (placed absolutely)
        title_label = tk.Label(container, text=self.title, bg="#f9b810", fg="black",
                               font=("TkDefaultFont", title_font_size, "bold"))
        title_label.place(relx=0, rely=0.0, relwidth=1.0, anchor='nw')

        # Value container
        self.value_labels_container = tk.Frame(container, bg="#1e1e1e")
        self.value_labels_container.place(relx=0, rely=0.06, relwidth=1.0, relheight=0.9, anchor='nw')

        # Initial build of the content
        self._rebuild_content()
        self.schedule_periodic_update(self.update_rate_ms, self.update_values)

    def _rebuild_content(self):
        """
        Destroys all existing value labels and rebuilds them based on the current data.
        This is called when keys change or for the initial setup.
        """
        # Clear existing labels
        for child in self.value_labels_container.winfo_children():
            child.destroy()
        self._value_labels.clear() # Clear the stored label references

        data = self.data_fn() # Get the latest data for rebuilding

        style = self.app.style
        value_font_size = style.font_size("small")

        if not data:
            no_data_lbl = tk.Label(self.value_labels_container, text="No data has been received",
                                   bg="#1e1e1e", fg="gray",
                                   font=("TkDefaultFont", value_font_size, "italic"),
                                   anchor='center', justify='center')
            no_data_lbl.place(relx=0.5, rely=0.5, anchor='center')
        else:
            y_offset = 0.0
            step = 0.06
            for key, val in data.items():
                display_text = f"{key}: {self._format_value(val)}"
                lbl = tk.Label(self.value_labels_container, text=display_text,
                               bg="#1e1e1e", fg="white",
                               font=("TkDefaultFont", value_font_size),
                               anchor='w', justify='left')
                lbl.place(relx=0.05, rely=y_offset, anchor='nw')
                self._value_labels[key] = lbl # Store reference to the label
                y_offset += step
        self._last_data_snapshot = dict(data) # Update snapshot after rebuilding

    def update_values(self):
        current_data = self.data_fn()

        # Check for key changes (or if data became empty/non-empty)
        # We compare the sets of keys to detect structural changes.
        if set(current_data.keys()) != set(self._last_data_snapshot.keys()) or \
           (not current_data and self._last_data_snapshot) or \
           (current_data and not self._last_data_snapshot):
            self._rebuild_content() # Keys have changed, rebuild everything
        else:
            # Only values might have changed, update existing labels
            updated_any = False
            for key, current_val in current_data.items():
                last_val = self._last_data_snapshot.get(key)
                if current_val != last_val:
                    # Update only if the value for this specific key has changed
                    display_text = f"{key}: {self._format_value(current_val)}"
                    if key in self._value_labels:
                        self._value_labels[key].config(text=display_text)
                        updated_any = True
            if updated_any:
                self._last_data_snapshot = dict(current_data) # Update snapshot only if values changed

    def _format_value(self, val):
        if isinstance(val, float):
            return f"{val:.2f}"
        return str(val)
    
class SpotInputsDataBlock(DynamicDictBlock):
    def __init__(self, app: ModularGUI, update_rate_ms=100): # Increased update rate for demo
        super().__init__(app, title="SPOT INPUTS",
                         data_fn=lambda: app.data_manager.get_inputs(),
                         update_rate_ms=update_rate_ms)

class SpotStatusDataBlock(DynamicDictBlock):
    def __init__(self, app: ModularGUI, update_rate_ms=100): # Increased update rate for demo
        super().__init__(app, title="SPOT STATUS",
                         data_fn=lambda: app.data_manager.spot.data,
                         update_rate_ms=update_rate_ms)

class SwappableBlock(BaseBlock):
    """
    A container block that can hold multiple blocks and swap between them.
    Only one block is visible at a time.
    """
    def __init__(self, app, blocks: List[BaseBlock], swap_key: str = "Tab", name: str = "Swappable"):
        # Use the maximum dimensions from all child blocks
        max_width = max(block.width_units for block in blocks)
        max_height = max(block.height_units for block in blocks)
        super().__init__(app, width_units=max_width, height_units=max_height)
        
        self.name = name
        self.blocks = blocks
        self.swap_key = swap_key.lower()
        self.active_index = 0
        self.block_containers = []  # Store the container frames for each block
        
    def build(self, container, width_px, height_px):
        super().build(container, width_px, height_px)
        container.configure(bg="#1e1e1e")
        
        # Create containers for each block
        for i, block in enumerate(self.blocks):
            # Create a frame for each block
            block_frame = tk.Frame(container, width=width_px, height=height_px)
            block_frame.place(x=0, y=0, width=width_px, height=height_px)
            
            # Build the block in its frame
            block.build(block_frame, width_px, height_px)
            self.block_containers.append(block_frame)
            
            # Hide all blocks except the first one
            if i != self.active_index:
                block_frame.place_forget()
        
        # Add swap indicator
        self._create_swap_indicator(container, width_px, height_px)
        
        # Bind swap key
        container.bind_all(f"<KeyPress-{self.swap_key}>", self._on_swap_key, add="+")
        
    def _create_swap_indicator(self, container, width_px, height_px):
        """Create a small indicator showing current block and swap key"""
        indicator_height = 25
        
        self.indicator_frame = tk.Frame(
            container, 
            bg="#333333", 
            height=indicator_height
        )
        self.indicator_frame.place(
            x=0, 
            y=height_px - indicator_height, 
            width=width_px, 
            height=indicator_height
        )
        
        self.indicator_label = tk.Label(
            self.indicator_frame,
            text=f"{self.active_index + 1}/{len(self.blocks)} | Press {self.swap_key.upper()} to swap",
            bg="#333333",
            fg="white",
            font=("TkDefaultFont", self.style.font_size("normal"))
        )
        self.indicator_label.pack(expand=True)
        
    def _on_swap_key(self, event):
        """Handle swap key press"""
        if event.keysym.lower() == self.swap_key:
            self.swap_to_next()
            
    def swap_to_next(self):
        """Swap to the next block in the sequence"""
        # Hide current block
        self.block_containers[self.active_index].place_forget()
        
        # Move to next block (wrap around)
        self.active_index = (self.active_index + 1) % len(self.blocks)
        
        # Show new active block
        self.block_containers[self.active_index].place(
            x=0, y=0, 
            width=self.width_px, 
            height=self.height_px
        )
        
        # Update indicator
        self.indicator_label.config(
            text=f"{self.active_index + 1}/{len(self.blocks)} | Press {self.swap_key.upper()} to swap"
        )
        
    def swap_to_index(self, index: int):
        """Swap to a specific block by index"""
        if 0 <= index < len(self.blocks):
            # Hide current block
            self.block_containers[self.active_index].place_forget()
            
            # Set new active block
            self.active_index = index
            
            # Show new active block
            self.block_containers[self.active_index].place(
                x=0, y=0, 
                width=self.width_px, 
                height=self.height_px
            )
            
            # Update indicator
            self.indicator_label.config(
                text=f"Block {self.active_index + 1}/{len(self.blocks)} | Press {self.swap_key.upper()} to swap"
            )
    
    def destroy(self):
        """Destroy all child blocks"""
        for block in self.blocks:
            block.destroy()
        super().destroy()

