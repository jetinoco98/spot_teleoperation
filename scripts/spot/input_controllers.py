import numpy as np
import do_mpc
import time
import math
from spot_interface import SpotInterface

JOYSTICK_INPUT_THRESHOLD = 0.2  # Threshold for joystick inputs to be considered active
JOYSTICK_MAX_LINEAR_VELOCITY = 0.5  # m/s
JOYSTICK_MAX_ANGULAR_VELOCITY = 0.75  # rad/s
USE_LQR = False  # If False, a direct mapping of HMD inputs to Spot orientation is used


class HMDController:
    def __init__(self):
        self.setpoints = np.array([[0], [0]])
        self.model = self._get_model()
        self.lqr = self._get_lqr(self.model)

    def _get_model(self):
        model = do_mpc.model.LinearModel('discrete')
        _x = model.set_variable(var_type='_x', var_name='x', shape=(2, 1))
        _u = model.set_variable(var_type='_u', var_name='u', shape=(2, 1))
        x_next = _x + _u
        model.set_rhs('x', x_next)
        model.setup()
        return model

    def _get_lqr(self, model):
        lqr = do_mpc.controller.LQR(model)
        lqr.set_param(t_step=0.05)
        lqr.set_param(n_horizon=None)  # infinite horizon
        Q = 10 * np.identity(2)
        R = np.identity(2)
        Rdelu = np.identity(2)
        lqr.set_objective(Q=Q, R=R)
        lqr.set_rterm(delR=Rdelu)
        lqr.setup()
        return lqr

    def compute_controls(self, hmd_inputs, measures):
        # Roll will not be considered
        if USE_LQR:
            self.setpoints = np.array([[hmd_inputs[0]], [hmd_inputs[1]]])
            self.lqr.set_setpoint(xss=self.setpoints, uss=np.array([[0], [0]]))
            x = np.array([[measures[0]], [measures[1]]])
            u = self.lqr.make_step(x)
            return [round(u[0][0], 3), round(u[1][0], 3), 0]
        else:
            return [hmd_inputs[0], hmd_inputs[1], 0]


class JoystickMapper:
    """
    Maps joystick inputs to robot velocity commands.
    """
    def compute_controls(self, inputs):
        return [
            self._compute_control(inputs[0], JOYSTICK_MAX_LINEAR_VELOCITY), # Forward/Backward
            self._compute_control(inputs[1], -JOYSTICK_MAX_LINEAR_VELOCITY), # Left/Right
            self._compute_control(inputs[2], -JOYSTICK_MAX_ANGULAR_VELOCITY) # Rotation
        ]

    def _compute_control(self, value, base_speed):
        if abs(value) > JOYSTICK_INPUT_THRESHOLD:
            return value * base_speed
        return 0
    

class KatvrInterface:
    """
    Processes KatVR specific functionality to robot control commands.
    """
    def __init__(self, robot: SpotInterface):
        self.robot = robot
        # Calibration
        self.yaw_offset = 0.0  # Offset between Spot's odom yaw and KatVR yaw
        self.is_calibrated = False # Calibration state between Spot and KatVR
        # Autosit/Autostand
        self.base_hmd_height = None
        self.autosit_active = False
        # Autolook Mode
        self.inactivity_time = 0
        self.last_movement_time = time.time()
        # === PID Controller
        self.target_yaw = None
        # Parameters
        self.pid_deadzone_degrees = 2.0
        # Controller state
        self._prev_yaw_error = 0.0
        self._prev_time = time.time()
    
    def auto_sit(self, hmd_height, stand_button):
        """
        Makes the robot sit if the HDM height drops below a threshold.
        """
        # The height threshold is the amount of height drop from the base height
        HEIGHT_THRESHOLD = 0.18  # obtained experimentally

        # Inputs not yet available
        if hmd_height is None or stand_button is None:
            return
        
        # If stand button is pressed, set the base height
        if stand_button:
            self.base_hmd_height = hmd_height
            return
        
        # Check if we need to sit
        if self.base_hmd_height is not None:
            lower_limit = self.base_hmd_height - HEIGHT_THRESHOLD
            if hmd_height < lower_limit and self.robot.is_standing:
                self.robot.sit()
                self.autosit_active = True


    def auto_stand(self, hmd_height):
        """
        Makes the robot stand automatically if the user stands up again after having sat down
        and activated the auto-sit feature.
        """
        # Inputs not yet available
        if hmd_height is None:
            return

        # The base height has not been set yet (Stand button not pressed)
        if not self.base_hmd_height:
            return

        # Check if we need to stand
        if self.autosit_active and hmd_height >= self.base_hmd_height - 0.06:
            self.autosit_active = False
            self.robot.stand()
            self.is_calibrated = False  # Reset calibration state for KatVR
    
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalizes an angle to the range [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    

    def compute_angular_velocity_from_katvr(self, katvr_yaw):
        # PID Constants (obtained experimentally)
        KP = 2.0
        KD = 0.05
        
        self.robot.update_current_angles()

        if not self.is_calibrated:
            self.yaw_offset = katvr_yaw - self.robot._yaw_odom
            self.is_calibrated = True
            # Reset Time
            self._prev_time = time.time()
            return 0.0

        # Compute calibrated yaw target for the robot
        self.target_yaw = self.normalize_angle(katvr_yaw - self.yaw_offset)

        # === PID Control ===
        yaw_error = self.normalize_angle(self.target_yaw - self.robot._yaw_odom)

        # Time step for derivative
        now = time.time()
        dt = now - self._prev_time
        self._prev_time = now

        # Derivative term
        pid_d_error = (yaw_error - self._prev_yaw_error) / dt if dt > 0 else 0.0
        self._prev_yaw_error = yaw_error

        # Output calculation
        ANGULAR_VEL_MAX = 1.5  # rad/s. Maximum angular velocity of Spot.
        pid_output = KP * yaw_error + KD * pid_d_error
        pid_saturated_output = max(-ANGULAR_VEL_MAX, min(ANGULAR_VEL_MAX, pid_output))

        # Dead zone logic
        if abs(yaw_error) < abs(math.radians(self.pid_deadzone_degrees)):
            pid_saturated_output = 0.0

        self.robot.set_velocity(
            v_x=None,
            v_y=None,
            v_rot=pid_saturated_output
        )


    def dynamic_deadzone_adjustment(self, look_mode: bool):
        """
        Adjusts the PID controller deadzone based on robot inactivity time.
        """
        # Update inactivity time for the robot
        if abs(self.robot._v_x) > 0 or abs(self.robot._v_y) > 0 or abs(self.robot._v_rot) > 0.05:
            self.robot.last_movement_time = time.time()
        self.robot.inactivity_time = time.time() - self.robot.last_movement_time

        if look_mode:
            deadzone = 2.0  # Default deadzone for PID controller (degrees)
        else:
            RANGE = 2  # Increase over 2 seconds of inactivity
            inactivity_ratio = self.robot.inactivity_time / RANGE
            normalized_inactivity = max(0, min(inactivity_ratio, 1))
            deadzone = 2 + 13 * normalized_inactivity  # Deadzone increases with inactivity for a max of 15°

        self.pid_deadzone_degrees = deadzone
