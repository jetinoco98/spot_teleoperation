"""
data_processor.py

For now, this script only contains the KatvrDataProcessor class, as KATVR is the only device
that requires a more complex data processing logic.
"""

import time
import math
import collections
import numpy as np
import threading


class KatvrDataProcessor:
    """
    Processes KATVR input data in the following way:
    * Computes an output velocity for SPOT based on filtered KATVR velocity
    * Tracks specific yaw changes in a stateful manner to detect lateral walking  
    * Computes angular velocity based on yaw changes
    * Uses adaptive filtering: raw velocity detection -> median filter activation -> EMA smoothing
    """
    # --- Minimum velocity threshold for KATVR device
    # This was obtained from experimentation with the KATVR device. Any velocity below this threshold
    # is considered as "noise" so it is ignored.
    VELOCITY_MIN = 0.75

    # --- FIXED VELOCITIES FOR SPOT
    # Lateral Velocity: The KATVR device only supports lateral walking at a fixed speed.
    OUTPUT_LATERAL_VELOCITY = 0.5 # m/s
    # Backward Velocity: The KATVR device only supports backward walking at a fixed speed.
    OUTPUT_BACKWARD_VELOCITY = 0.3 # m/s
    # Minimum Velocity
    OUTPUT_VELOCITY_MIN = 0.3 # m/s

    # --- VELOCITY MODE SELECTOR
    # Select the velocity mode according to where SPOT will be teleoperated.
    # 1: INDOOR (default)
    # 2: OUTDOOR
    # 3: OPEN FIELD
    KATVR_VELOCITY_MODE = 1

    # Slope values for velocity modes (y = mx)
    M_INDOOR = 0.2
    M_OUTDOOR = 0.33
    M_OPEN_FIELD = 0.4

    '''
    For reference, 
    the M_INDOOR slope achieves the following configurations:
    - SPOT 0.3 m/s at 1.5 m/s KATVR velocity
    - SPOT 0.5 m/s at 2.5 m/s KATVR velocity
    - SPOT 1.0 m/s at 5.0 m/s KATVR velocity

    the M_OUTDOOR slope achieves:
    - SPOT 0.5 m/s at 1.5 m/s KATVR velocity
    - SPOT 1.0 m/s at 3.0 m/s KATVR velocity
    - SPOT 1.5 m/s at 4.5 m/s KATVR velocity

    the M_OPEN_FIELD slope achieves:
    - SPOT 0.5 m/s at 1.25 m/s KATVR velocity
    - SPOT 1.0 m/s at 2.5 m/s KATVR velocity
    - SPOT 1.5 m/s at 3.75 m/s KATVR velocity
    '''

    def __init__(self):
        self._initialize_state()

    def _initialize_state(self):
        # Yaw tracking (degrees)
        self.yaw_virtual = 0
        self.yaw = 0
        self.previous_yaw_virtual = None
        self.previous_yaw = None

        # KATVR movement tracking
        self.velocity = 0
        self.velocity_filtered_light = 0
        self.previous_velocity_filtered_light = 0
        self.velocity_history_light = collections.deque(maxlen=5) # For moving median filter
        self.is_movement_detected = False

        # Median filter activation tracking
        self.median_filter_active = False
        self.movement_count_since_detected = 0
        self.ACTIVATION_THRESHOLD = 5

        # KATVR velocity tracking (EMA)
        self.velocity_filtered_strong = 0
        self.ema_alpha = 0.4

        # Output velocity components
        self.output_velocity = 0
        self.forward_velocity = 0
        self.lateral_velocity = 0
        self.previous_lateral_velocity = 0

        # For angular velocity calculation
        self.delta_time = None
        self.angular_velocity = 0

        # State tracking
        self.state = 'normal'
        self.last_state_change_time = time.time()
        self.yaw_temporal_offset = 0.0
        self.on_debounce = False


    @staticmethod
    def normalize_angle(angle):
        """Normalizes an angle to the range [-180, 180) degrees."""
        return (angle + 180) % 360 - 180
    
    # ===== YAW AND MOVEMENT STATE ======
    def _update_actual_yaw_and_state(self):
        """
        Detects virtual yaw jumps and updates actual yaw and walking state.

        This is needed because when the user activates lateral walking, the KATVR device simply
        shifts the virtual yaw to the left or right by 90 degrees.
        """
        YAW_JUMP_THRESHOLD = 40  # degrees
        WALKING_STATE_MIN_DURATION = 0.4  # seconds

        if self.previous_yaw_virtual is None:
            self.yaw = self.yaw_virtual
            return

        # Calculate the change in yaw
        delta_angle = self.yaw_virtual - self.previous_yaw_virtual
        delta_angle = self.normalize_angle(delta_angle)
        current_time = time.time()

        # Check for jumps in virtual yaw
        if abs(delta_angle) >= YAW_JUMP_THRESHOLD:
            if self.state == "normal":
                if delta_angle > 0:
                    new_state = "walking_left"
                    self.yaw_temporal_offset = -90
                else:
                    new_state = "walking_right"
                    self.yaw_temporal_offset = 90
            else:
                new_state = "normal"
                self.yaw_temporal_offset = 0.0 # Reset offset when returning to normal

            if self.state != new_state:
                self.state = new_state
                self.last_state_change_time = current_time

        # Exit walking state after minimum duration if velocity low
        if self.state in ["walking_left", "walking_right"]:
            if current_time - self.last_state_change_time >= WALKING_STATE_MIN_DURATION:
                if not self.is_movement_detected:
                    self.state = "normal"
                    self.yaw_temporal_offset = 0.0
                    self.last_state_change_time = current_time

        # Update yaw based on state
        if self.state == "normal":
            self.yaw = self.yaw_virtual
            self.yaw_temporal_offset = 0.0
        else:
            self.yaw = self.normalize_angle(self.yaw_virtual + self.yaw_temporal_offset)

    # ===== VELOCITY RELATED METHODS =====
    def _activate_debounce_guard(self):
        time.sleep(0.2)  # Debounce time
        self.on_debounce = False

    def _moving_median_filter(self, history: collections.deque):
        """Computes the moving median of the values in the history."""
        if not history:
            return 0.0
        return np.median(list(history))

    def _process_velocity(self):
        """
        - Applies moving median filter and EMA to the raw velocity and updates the internal variables.
        - Also computes the output forward velocity based on the selected mode.
        """
        if self.on_debounce:
            self.is_movement_detected = False  # Reset movement detection during debounce
            self.median_filter_active = False
            self.movement_count_since_detected = 0
            return

        # 1. Raw velocity sets is_movement_detected to True
        if not self.is_movement_detected and abs(self.velocity) > self.VELOCITY_MIN:
            self.is_movement_detected = True
            self.movement_count_since_detected = 0
            self.median_filter_active = False

        # 2. When movement is detected, count additions to activate median filter after 5 samples
        if self.is_movement_detected:
            self.movement_count_since_detected += 1
            if self.movement_count_since_detected >= self.ACTIVATION_THRESHOLD:
                self.median_filter_active = True

        # Add current velocity to median filter history
        self.velocity_history_light.append(self.velocity)

        # Apply median filter if active
        if self.median_filter_active:
            self.velocity_filtered_light = self._moving_median_filter(self.velocity_history_light)
            
            # 3. When median filter is active, only it can set is_movement_detected to False
            if self.is_movement_detected:
                self.is_movement_detected = abs(self.velocity_filtered_light) > (self.VELOCITY_MIN / 3)
        else:
            # When median filter is not active, use raw velocity
            self.velocity_filtered_light = self.velocity
            
            # 3. When median filter is not active, raw velocity can set is_movement_detected to False
            if self.is_movement_detected:
                self.is_movement_detected = abs(self.velocity) > (self.VELOCITY_MIN / 3)

        # 4. When is_movement_detected goes to False, deactivate median filter
        if not self.is_movement_detected:
            self.median_filter_active = False
            self.movement_count_since_detected = 0

        # 5. Apply EMA filter that gets values from median filter output
        if self.median_filter_active:
            # EMA: new_value = alpha * current_input + (1 - alpha) * previous_value
            self.velocity_filtered_strong = (self.ema_alpha * self.velocity_filtered_light + 
                                           (1 - self.ema_alpha) * self.velocity_filtered_strong)
        else:
            # If median filter is not active, use the light filtered value directly
            self.velocity_filtered_strong = self.velocity_filtered_light

        # Determine the slope 'm' based on the current velocity mode
        m = 0.0
        if self.KATVR_VELOCITY_MODE == 1:  # INDOOR
            m = self.M_INDOOR
        elif self.KATVR_VELOCITY_MODE == 2:  # OUTDOOR
            m = self.M_OUTDOOR
        elif self.KATVR_VELOCITY_MODE == 3:  # OPEN FIELD
            m = self.M_OPEN_FIELD
        else: # Default to INDOOR if an invalid mode is set
            m = self.M_INDOOR

        # Apply the y = mx formula
        self.output_velocity = m * self.velocity_filtered_strong
        
    def _compute_velocity_components(self):
        """
        Computes forward/lateral velocities based on current state.
        """
        if self.state == 'normal':
            # Forward walking
            self.forward_velocity = max(self.OUTPUT_VELOCITY_MIN, self.output_velocity)
            self.lateral_velocity = 0.0
            # Backward walking
            if self.velocity_filtered_light < 0:
                self.forward_velocity = -self.OUTPUT_BACKWARD_VELOCITY

        elif self.state == 'walking_right':
            self.forward_velocity = 0.0
            self.lateral_velocity = -self.OUTPUT_LATERAL_VELOCITY

        elif self.state == 'walking_left':
            self.forward_velocity = 0.0
            self.lateral_velocity = self.OUTPUT_LATERAL_VELOCITY

        # Check for transition from negative to positive velocity
        if (self.previous_velocity_filtered_light < 0 and self.velocity_filtered_light > 0):
            self.on_debounce = True
            threading.Thread(target=self._activate_debounce_guard).start()
            
        # Check for transition from lateral to forward velocity
        if (self.previous_lateral_velocity != 0 and self.lateral_velocity == 0):
            self.on_debounce = True
            threading.Thread(target=self._activate_debounce_guard).start()
            
        if not self.is_movement_detected or self.on_debounce:
            self.forward_velocity = 0.0
            self.lateral_velocity = 0.0
            return

    # ===== ANGULAR VELOCITY =====
    def _compute_angular_velocity(self):
        """Computes angular velocity using yaw changes."""
        if self.previous_yaw is None:
            self.angular_velocity = 0.0
            return

        delta_angle = self.normalize_angle(self.yaw - self.previous_yaw)
        # Avoid division by zero if delta_time is 0 or very small
        if self.delta_time is None or self.delta_time == 0:
            self.angular_velocity = 0.0
        else:
            angular_velocity_deg = delta_angle / self.delta_time
            self.angular_velocity = math.radians(angular_velocity_deg)

    # ===== INTERNAL PROCESSING =====
    def _process_internal_values(self):
        """Main update loop for processing KATVR values."""
        self._update_actual_yaw_and_state()
        self._process_velocity()
        self._compute_velocity_components()
        self._compute_angular_velocity()

        # Update previous values at the end of the processing cycle
        self.previous_yaw_virtual = self.yaw_virtual
        self.previous_yaw = self.yaw
        self.previous_velocity_filtered_light = self.velocity_filtered_light
        self.previous_lateral_velocity = self.lateral_velocity

    # ===== PUBLIC METHODS =====
    def process_inputs(self, delta_time: float, yaw_virtual: float, velocity: float):
        """
        Processes KATVR input data and updates internal state.
        Returns:
            dict: The processed KATVR data in a dictionary format.
        """
        self.delta_time = delta_time
        self.yaw_virtual = yaw_virtual
        self.velocity = velocity
        self._process_internal_values()

        output_data = {
            "status": True,
            "yaw_virtual": self.yaw_virtual,
            "yaw": self.yaw,
            "state": self.state,
            "velocity": self.velocity,
            "velocity_filtered_light": self.velocity_filtered_light,
            "velocity_filtered_strong": self.velocity_filtered_strong,
            "is_movement_detected": self.is_movement_detected,
            "forward_velocity": self.forward_velocity,
            "lateral_velocity": self.lateral_velocity,
            "angular_velocity": self.angular_velocity
        }

        return output_data

    def handle_connection_lost(self):
        """
        Called by manager when no data has been received for a while.
        Resets all internal state variables to initial values.
        """
        self._initialize_state()
