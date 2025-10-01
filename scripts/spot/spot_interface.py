#!/usr/bin/env python
"""
.. module:: spot_interface
    :platform: Windows
    :synopsis: The spot_interface python script in ``zed-oculus-spot`` package

.. moduleauthor:: Ali Yousefi <ali.yousefi@edu.unige.it>
	Initializes the required service clients. Provides the required method for sending the control 
    signals to the robot, and receving robot angular velocities.
"""
import logging
import time
import math
import numpy as np
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client
from bosdyn.api import estop_pb2
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.robot_state import RobotStateClient
from bosdyn import geometry
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, BODY_FRAME_NAME, get_a_tform_b

LOGGER = logging.getLogger()

ROBOT_REMOTE_IP = '10.0.0.3'
ROBOT_LOCAL_IP = '192.168.80.3' 
ROBOT_IP = ROBOT_REMOTE_IP
ROBOT_USERNAME = 'user'
ROBOT_PASSWORD = 'wruzvkg4rce4'

YAW_MAX = 0.5  # radians
PITCH_MAX = 0.5  # radians
ROLL_MAX = 0.4  # radians
LINEAR_VEL_MAX = 1.0  # m/s
ANGULAR_VEL_MAX = 1.5  # rad/s
VELOCITY_CMD_DURATION = 0.6  # seconds


class SpotInterface:
    def __init__(self):
        # Robot desired state
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._height = 0.0
        self._v_x = 0.0
        self._v_y = 0.0
        self._v_rot = 0.0

        # Robot actual state
        self._actual_yaw = None
        self._actual_pitch = None
        self._actual_roll = None
        self._yaw_odom = None
        self._pitch_odom = None
        self._roll_odom = None
        self.is_initialized = False
        self.is_standing = False
        self.processed_state_data = {}

        # SDK related
        self._estop_keepalive = None

        # --- For KATVR integration
        self.katvr_yaw_offset = 0.0
        self.calibrated_with_katvr = False
        self.target_yaw = None
        self.base_hmd_height = None
        self.autosit_active = False
        # Autolook Mode
        self.inactivity_time = 0
        self.last_movement_time = time.time()
        # PID Controller
        self.set_up_pid_controller()

        
    def initialize(self):
        """ 
        Initializes SDK, authenticates, and ensures the required clients are available. 
        """
        if self.is_initialized:
            print("Spot Robot is already initialized.")
            return
        
        print("Initializing Spot Robot. Wait to a receive confirmation message...")
        self._client_name = "spot_interface"
        sdk = bosdyn.client.create_standard_sdk(self._client_name)
        self._robot = sdk.create_robot(ROBOT_IP)
        self._robot.authenticate(ROBOT_USERNAME, ROBOT_PASSWORD, timeout=60)
        self._robot.sync_with_directory()
        self._robot.time_sync.wait_for_sync()
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self.start()
        

    def start(self):
        """
        Takes the lease of the robot and powers on the motors
        """
        # Gain Control of the robot
        self._lease_client.take()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True, return_at_exit=True)
        self._toggle_estop()

        # Power on the robot
        self._robot.power_on()
        self._robot.is_powered_on()
        self.start_time = time.time()
        self.is_initialized = True
        print("Spot Robot initialized successfully!")


    def _toggle_estop(self):
        """
        Toggles on/off E-Stop. Initial state is ON.
        """
        if not self._estop_keepalive:
            if self._estop_client.get_status().stop_level == estop_pb2.ESTOP_LEVEL_NONE:
                print('Taking E-Stop from another controller')

            #register endpoint with 9 second timeout
            estop_endpoint = EstopEndpoint(client=self._estop_client,
                                                               name=self._client_name,
                                                               estop_timeout=9.0)
            estop_endpoint.force_simple_setup()

            self._estop_keepalive = EstopKeepAlive(estop_endpoint)
        else:
            self._estop_keepalive.stop()
            self._estop_keepalive.shutdown()
            self._estop_keepalive = None


    def shutdown(self):
        """
            Makes the robot to be configured with no orientation offsets, sit down, and powers off the motors.
        """
        if not self.is_initialized:
            print('SpotInterface has not been initialized yet. Cannot shutdown.')
            return
        
        print('Shutting down SpotInterface...')
        self.sit()
        time.sleep(2.5)
        safe_power_off_cmd=RobotCommandBuilder.safe_power_off_command()
        self._robot_command_client.robot_command(command= safe_power_off_cmd)
        time.sleep(2.5)
        self._toggle_estop()  # Toggle E-Stop back ON
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()
        self.is_initialized = False
        print('SpotInterface shutdown complete.')
    

    def _set_mobility_params(self):
        """
        Sets the required mobility parameters, including obstacle avoidance, velocity limits, and orientation offset between the robot 
        and footprint frames.

        Returns:
            mobility_params(spot_command_pb2.MobilityParams)
        """
        obstacles = spot_command_pb2.ObstacleParams(
            disable_vision_body_obstacle_avoidance=False,
            disable_vision_foot_obstacle_avoidance=False,
            disable_vision_foot_constraint_avoidance=False,
            disable_vision_foot_obstacle_body_assist=False,
            disable_vision_negative_obstacles=False,
            obstacle_avoidance_padding=0.1
        )
        
        # Set the orientation of the robot body frame with respect to the footprint frame
        footprint_R_body = geometry.EulerZXY(roll=0.0, pitch=self._pitch, yaw=self._yaw)

        position = geometry_pb2.Vec3(x=0.0, y=0.0, z=self._height)
        rotation = footprint_R_body.to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=LINEAR_VEL_MAX, y=LINEAR_VEL_MAX), angular=ANGULAR_VEL_MAX))
        mobility_params = spot_command_pb2.MobilityParams(obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control, locomotion_hint=spot_command_pb2.HINT_AUTO)
        return mobility_params
        

    def _orientation_cmd_helper(self, yaw=0.0, roll=0.0, pitch=0.0, height=0.0):
        """
        Helper function that commands the robot with an orientation command
            
        Args:
            yaw: Yaw of the robot body. Defaults to 0.0.
            roll: Roll of the robot body. Defaults to 0.0.
            pitch: Pitch of the robot body. Defaults to 0.0.
            height: Height of the robot body from normal stand height. Defaults to 0.0.
        """
        orientation = geometry.EulerZXY(yaw, roll, pitch)
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=height, footprint_R_body=orientation)
        self._robot_command_client.robot_command_async(command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)


    def _velocity_cmd_helper(self, v_x=0.0, v_y=0.0, v_rot=0.0):
        """
        Helper function that commands the robot with a velocity command

        Args:
            v_x: Forward/backward velocity of the robot body.
            v_y: Left/right velocity of the robot body.
            v_rot: Rotational velocity of the robot body.
        """
        mobility_params = self._set_mobility_params() 
        cmd = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=mobility_params)
        self._robot_command_client.robot_command_async(command=cmd, end_time_secs=time.time() + VELOCITY_CMD_DURATION)
    

    def get_body_orientation(self):
        """
        Provides the orientation values of the robot base frame.

        Returns:
            List[float]: A list containing the yaw, pitch, and roll angles of the robot's base frame,
            in the following order: [yaw, pitch, roll].
        """
        
        return [round(self._yaw, 3), round(self._pitch, 3), round(self._roll, 3)]

    
    def stand(self):
        if not self.is_standing:
            blocking_stand(self._robot_command_client)
            self.is_standing = True
            self.calibrated_with_katvr = False  # Reset calibration state


    def sit(self):
        if self.is_standing:
            blocking_sit(self._robot_command_client)
            self.is_standing = False
            self.calibrated_with_katvr = False  # Reset calibration state


    def set_idle_mode(self, height=0.0):
        """
        Sets the robot to idle mode by setting all velocities and orientations to zero.
        """
        self.set_orientation(0, 0, 0)
        self.set_velocity(0, 0, 0)
        self._orientation_cmd_helper(yaw=self._yaw, pitch=self._pitch, roll=self._roll, height=height)
        self.calibrated_with_katvr = False


    def set_velocity(self, v_x: float, v_y: float, v_rot: float):
        """
        Sets the robot's velocity values (body frame) based on the provided inputs.

        Set a variable to None to keep the robot's current value unchanged.

        """
        if v_x is not None:
            self._v_x = max(-LINEAR_VEL_MAX, min(LINEAR_VEL_MAX, v_x))
        if v_y is not None:
            self._v_y = max(-LINEAR_VEL_MAX, min(LINEAR_VEL_MAX, v_y))
        if v_rot is not None:
            self._v_rot = max(-ANGULAR_VEL_MAX, min(ANGULAR_VEL_MAX, v_rot))


    def set_orientation(self, yaw: float, pitch: float, roll: float):
        """
        Updates the robot's orientation values (body frame) based on the HMD orientation.
        
        Set a variable to None to keep the robot's current value unchanged.
        """
        if yaw is not None:
            self._yaw = max(-YAW_MAX, min(YAW_MAX, yaw))
        if pitch is not None:
            self._pitch = max(-PITCH_MAX, min(PITCH_MAX, pitch))
        if roll is not None:
            self._roll = max(-ROLL_MAX, min(ROLL_MAX, roll))


    def set_lqr_based_orientation(self, hmd_controls: list):
        """
        Updates the robot's orientation values (body frame) based on the HMD orientation and LQR computation.

        Args:
            hmd_controls (list): A list containing the processed HMD control values in the order of [dyaw, dpitch, droll].
        """
        # Obtain the HMD control values
        dyaw = hmd_controls[0]
        dpitch = hmd_controls[1]
        droll = hmd_controls[2]

        # Increment the robot's RPY values
        self._yaw += dyaw
        self._pitch += dpitch
        # self._roll = self._roll + droll

        # Clamp the values to the specified limits
        self._yaw = max(-YAW_MAX, min(YAW_MAX, self._yaw))
        self._pitch = max(-PITCH_MAX, min(PITCH_MAX, self._pitch))
        self._roll = max(-ROLL_MAX, min(ROLL_MAX, self._roll))


    def set_touch_controls(self, touch_controls: list):
        """
        Updates the robot's velocity values (body frame) based on the HMD Touch Controller Joysticks.

        Args:
            touch_controls (list): A list containing the computed velocity control signals in the order of [v_x, v_y, v_rot].
        """
        self._v_x = touch_controls[0]
        self._v_y = touch_controls[1]
        self._v_rot = touch_controls[2]

        if self._v_rot != 0:
            self.calibrated_with_katvr = False  # Reset calibration state


    def set_height(self, direct_value: float = 0.0, joystick_value: float = 0.0):
        """
        Sets the height of the robot body frame with respect to the ground. 
        The default height of 0.0 is Spot's ideal height.
        Robot's max height is [-0.15, 0.15] meters.

        Args:
            direct_value (float): Direct height value to set. 
            joystick_value (float): Height adjustment value from a joystick input [-1, 1].
        """
        if joystick_value != 0.0 and abs(joystick_value) <= 1:
            self._height = joystick_value * 0.25  # Adjust height based on joystick input
            return
        
        self._height = max(-0.15, min(0.15, direct_value))

        
    def send_velocity_command(self):
        """
        Sends a velocity command to the robot using its current velocity values.
        """
        self._velocity_cmd_helper(v_x=self._v_x, v_y=self._v_y, v_rot=self._v_rot)


    def store_current_state(self):
        """
        Stores the current robot data in a dictionary for later retrieval.
        """
        self.processed_state_data = {
            'yaw': self._yaw,
            'pitch': self._pitch,
            'roll': self._roll,
            'height': self._height,
            'v_x': self._v_x,
            'v_y': self._v_y,
            'v_rot': self._v_rot,
            'is_standing': self.is_standing,
            'is_initialized': self.is_initialized,
            'current_yaw': self._yaw_odom,
            'required_yaw': self.target_yaw,
            'orientation_yaw': self._actual_yaw,
        }
    
    
    def get_data(self):
        """Returns a dictionary with the robot's most important data."""
        return self.processed_state_data

    # ====================================================================================
    #   METHODS FOR KATVR INTEGRATION
    # ====================================================================================

    def auto_sit_in_katvr(self, hmd_height, stand_button, height_threshold=0.18):
        """
        Makes the robot sit if the HDM height drops below a threshold.
        """
        if hmd_height is None or stand_button is None:
            return
        
        if stand_button == 1.0:
            self.base_hmd_height = hmd_height
            return

        if self.base_hmd_height is not None:
            lower_limit = self.base_hmd_height - height_threshold

            if hmd_height < lower_limit and self.is_standing:
                self.sit()
                self.autosit_active = True


    def auto_stand_in_katvr(self, hmd_height):
        """
        Makes the robot stand automatically if the user stands up again after having sat down
        and activated the auto-sit feature.
        """
        if hmd_height is None:
            return

        if not self.base_hmd_height:
            return

        if self.autosit_active and hmd_height >= self.base_hmd_height - 0.06:
            self.autosit_active = False
            self.stand()


    def update_odometry_angles(self):
        """
        Updates the robot's angles in the odometry frame with respect to the ground.
        """
        robot_state = self._robot_state_client.get_robot_state()
        frame_tree_snapshot = robot_state.kinematic_state.transforms_snapshot
        odom_T_body = get_a_tform_b(frame_tree_snapshot, ODOM_FRAME_NAME, "feet_center")
        quat = geometry.Quaternion(
            w=odom_T_body.rot.w,
            x=odom_T_body.rot.x,
            y=odom_T_body.rot.y,
            z=odom_T_body.rot.z
        )
        euler_odom = geometry.to_euler_zxy(quat)

        self._yaw_odom = euler_odom.yaw
        self._pitch_odom = euler_odom.pitch
        self._roll_odom = euler_odom.roll

        # TEMPORARY
        odom_T_body = get_a_tform_b(frame_tree_snapshot, "feet_center", BODY_FRAME_NAME)
        quat = geometry.Quaternion(
            w=odom_T_body.rot.w,
            x=odom_T_body.rot.x,
            y=odom_T_body.rot.y,
            z=odom_T_body.rot.z
        )
        euler_body = geometry.to_euler_zxy(quat)
        self._actual_yaw = euler_body.yaw


    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalizes an angle to the range [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    

    def compute_angular_velocity_from_katvr(self, katvr_yaw):
        self.update_odometry_angles()

        if not self.calibrated_with_katvr:
            self.katvr_yaw_offset = katvr_yaw - self._yaw_odom
            self.calibrated_with_katvr = True
            # Reset Yaw Error and Time
            self.yaw_error = 0.0
            self._prev_time = time.time()
            return 0.0

        # Compute calibrated yaw target for the robot
        self.target_yaw = self.normalize_angle(katvr_yaw - self.katvr_yaw_offset)

        # === PID Control ===
        self.yaw_error = self.normalize_angle(self.target_yaw - self._yaw_odom)

        # Set variables for PID control
        Kp = self.pid_kp
        Ki = 0.0  # Not used in this implementation
        Kd = self.pid_kd

        # Time step for derivative
        now = time.time()
        dt = now - self._prev_time
        self._prev_time = now

        # Derivative term
        self.pid_d_error = (self.yaw_error - self._prev_yaw_error) / dt if dt > 0 else 0.0
        self._prev_yaw_error = self.yaw_error

        # Output calculation
        self.pid_output = Kp * self.yaw_error + Kd * self.pid_d_error
        self.pid_saturated_output = max(-ANGULAR_VEL_MAX, min(ANGULAR_VEL_MAX, self.pid_output))

        # Dead zone logic
        if abs(self.yaw_error) < abs(math.radians(self.pid_deadzone_degrees)):
            self.pid_saturated_output = 0.0

        self._v_rot = self.pid_saturated_output


    # ====================================================================================
    #   METHODS FOR CUSTOM PD CONTROLLER
    # ====================================================================================

    def set_up_pid_controller(self):
        # Controller parameters
        self.pid_kp = 2.0
        self.pid_kd = 0.05
        self.pid_deadzone_degrees = 2.0
        # Controller state
        self.yaw_error = 0.0
        self.pid_d_error = 0.0
        self.pid_output = 0.0
        self.pid_saturated_output = 0.0
        self._prev_yaw_error = 0.0
        self._prev_time = time.time()


    def update_pid_controller(self, kp=None, kd=None, dead_zone_degrees=None):
        """
        Update the PID controller parameters.
        """
        if kp is not None:
            self.pid_kp = kp
        if kd is not None:
            self.pid_kd = kd
        if dead_zone_degrees is not None and isinstance(dead_zone_degrees, (int, float)):
            self.pid_deadzone_degrees = dead_zone_degrees
