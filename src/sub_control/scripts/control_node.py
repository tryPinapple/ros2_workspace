#!/usr/bin/env python3
"""
================================================================================
ASUQTR Submarine ROS 2 Control Node.
================================================================================
This module acts as the central nervous system for the sub's movement. It bridges 
the gap between high-level autonomous behaviors (FlexBE), manual gamepad inputs, 
and the low-level LQR mathematical solver.
"""

# ==========================================
# PYTHON STANDARD & 3RD PARTY IMPORTS
# ==========================================
import threading
import asyncio
import math
import numpy as np
from enum import IntEnum
import time

# ==========================================
# ROS 2 IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Messages and TF2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from sub_interfaces.msg import ThrusterCommand
from sub_interfaces.action import Control
import tf2_ros

# ==========================================
# ASUQTR IMPORTS
# ==========================================
from sub_control.lqr_solver import SubLQRSolver, THRUST_ALLOC_MAT

# Default cost matrix parameter values for LQR controller
# Q: State error penalties (Position/Angle springs and Velocity dampers)
DEFAULT_Q = [0.5, 0.5, 0.5, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
# R: Thruster energy penalties (Higher = less aggressive thrust)
DEFAULT_R = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
DEFAULT_MAX_THROTTLE = 0.8
DEFAULT_JOY_DEAD_ZONE = 0.1

# ==========================================
# ENUMS
# ==========================================
class ControlMode(IntEnum):
    """
    Defines the operational state of the Submarine.
    
    Attributes:
        BEHAVIOR: Fully autonomous. Listens to Action Server goals.
        LQR_TUNING: Debug mode. Listens to simple pose topics (e.g., from RViz).
        MANUAL: Human override. Listens to Gamepad joystick inputs.
    """
    BEHAVIOR = 0
    LQR_TUNING = 1
    MANUAL = 2

# ==========================================
# NODE DEFINITION
# ==========================================
class ControlNode(Node):
    """
    Main execution class for AUV Control.
    
    This class utilizes a MultiThreadedExecutor. Callback groups are strictly 
    separated into MutuallyExclusiveCallbackGroups to ensure that long-running 
    Action Server loops do not block the high-frequency Odometry callbacks.
    
    To bypass Python Garbage Collection micro-stutters, the core mathematical 
    states (`self.current_state` and `self.target_state`) are statically 
    pre-allocated as 12-element NumPy arrays. All callback updates are performed 
    in-place to achieve near RTOS-level determinism in Python.
    """
    
    def __init__(self):
        """Initialize the node, parameters, pre-allocated memory, ROS publishers/Subscribers
        callback mutex groups and TF2 buffer & frame to allow frame transfer when receiving target
        commands """
        super().__init__('control_node')
        
        # --- Parameters ---
        self.declare_parameter('control_mode', 'behavior')
        self.declare_parameter('state_cost_matrix', DEFAULT_Q)
        self.declare_parameter('thruster_cost_matrix', DEFAULT_R)

        # PERFORMANCE ARCHITECTURE: Pre-allocate State Arrays
        # Initializing with NaN ensures the LQR math functions won't 
        # accidentally process empty zero-data before the first localization message arrives.
        self.current_state = np.full(12, np.nan, dtype=np.float64)
        self.target_state = np.full(12, np.nan, dtype=np.float64) 

        # Load initial parameters
        mode_str = self.get_parameter('control_mode').value.lower()
        self._set_mode_from_string(mode_str)
        self.update_q_matrix(self.get_parameter('state_cost_matrix').value)
        self.update_r_matrix(self.get_parameter('thruster_cost_matrix').value)
        
        # Bind dynamic reconfigure callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- Instantiate Mathematical Engine ---
        self.lqr_solver = SubLQRSolver()
        
        # --- Thread Safety & State ---
        # The Action Server thread can update the target_state at the exact 
        # same time the Odometry thread is reading it for LQR math. This lock
        # prevents race conditions resulting in torn data reads.
        self.target_state_lock = threading.Lock()
        
        # --- Callback Groups ---
        # Those allow to make sure the callback cannot interrupt itself if the topic
        # is spammed. In onther words, it makes the callbacks non reentrant
        self.localization_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.global_reference_frame = 'odom'

        # --- Subscriptions & Publishers  & QoS ---
        # For sensors/thruster command, we only want latest value and avoid latency introduced by RELIABLE QoS
        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.localization_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.localization_callback, only_latest_qos,
            callback_group=self.localization_cb_group
        )
        
        self.debug_target_sub = self.create_subscription(
            PoseStamped, 'debug/target_pose', self.debug_target_callback, only_latest_qos,
            callback_group=self.action_cb_group 
        )
        
        self.gamepad_sub = self.create_subscription(
             Joy, 'dashboard/gamepad', self.gamepad_callback, only_latest_qos,
             callback_group=self.sensor_cb_group
        )
        
        self.thruster_pub = self.create_publisher(
            ThrusterCommand, 'thruster_cmd', only_latest_qos
        )

        # --- Action Server ---
        self._action_server = ActionServer(
            self, Control, 'navigate_sub', self.control_action_callback,
            callback_group=self.action_cb_group
        )

        self.last_mode_switch_button_state = False
        self.get_logger().info("Sub Control Node Initialized.")


    # ==========================================
    # PARAMETER HANDLING
    # ==========================================

    def parameter_callback(self, params):
        """
        Dynamically update node parameters without requiring a restart.
        
        Args:
            params (list): A list of rclpy.parameter.Parameter objects.
            
        Returns:
            SetParametersResult: Success or failure status of the parameter update.
        """
        for param in params:
            if param.name == 'control_mode':
                if self._set_mode_from_string(param.value.lower()):
                    return SetParametersResult(successful=True)
                return SetParametersResult(successful=False, reason="Invalid mode string")
            elif param.name == 'state_cost_matrix':
                if self.update_q_matrix(param.value):
                    return SetParametersResult(successful=True)
                else:
                    return SetParametersResult(successful=False, reason='Invalid Q matrix')
            elif param.name == 'thruster_cost_matrix':
                if self.update_r_matrix(param.value):
                    return SetParametersResult(successful=True)
                else:
                    return SetParametersResult(successful=False, reason='Invalid R matrix')

        # TODO handle multi params. Right now first handled param returns from this callback
        return SetParametersResult(successful=False, reason=f'Unhandled parameters :{params}')
        
    def _set_mode_from_string(self, mode_str):
        """
        Map a string parameter to the internal ControlMode Enum.
        
        Args:
            mode_str (str): The requested control mode as a string.
            
        Returns:
            bool: True if the mode was successfully mapped, False otherwise.
        """
        if mode_str == 'behavior':
            self._current_mode = ControlMode.BEHAVIOR
        elif mode_str == 'lqr_tuning':
            self._current_mode = ControlMode.LQR_TUNING
        elif mode_str == 'manual':
            self._current_mode = ControlMode.MANUAL
        else:
            self.get_logger().warn(f"Unknown control mode: {mode_str}")
            return False
        self.get_logger().info(f"SUB Control mode set as: {self._current_mode.name}")
        return True
    
    def update_q_matrix(self, q_list):
        """
        Convert a 1D Python list parameter into the 2D diagonal numpy array.
        
        Args:
            q_list (list): 12-element list representing state cost weights.
            
        Raises:
            ValueError: If the input list does not contain exactly 12 elements.
        """
        try:
            if isinstance(q_list, list) and len(q_list) == 12:
                self.q_matrix = np.diag(q_list).astype(np.float64)
                return SetParametersResult(successful=True)
            else:
                raise ValueError("Q matrix parameter must be a list of 12 elements")
        except Exception as e:
            return SetParametersResult(successful=False)


    def update_r_matrix(self, r_list):
        """
        Convert list to 2D diagonal array and pre-calculate the inverse.
        
        Args:
            r_list (list): 8-element list representing thruster cost weights.
            
        Raises:
            ValueError: If the input list does not contain exactly 8 elements.
        """
        try:
            if isinstance(r_list, list) and len(r_list) == 8:
                self.r_matrix = np.diag(r_list).astype(np.float64)
                # Math optimization: K = R^-1 * B^T * X. We pre-compute R^-1 here so we 
                # don't waste CPU cycles inverting the matrix during the hot control loop.
                self.inv_r_matrix = np.linalg.inv(self.r_matrix)
                return SetParametersResult(successful=True)
            else:
                raise ValueError("R matrix parameter must be a list of 8 elements")
        except Exception as e:
            return SetParametersResult(successful=False)


    # ==========================================
    # LOCALIZATION & TARGET STATE CALLBACKS
    # ==========================================
    
    def wrap_angles_to_pi(self, angles):
        """
        Wrap an array of angles to the [-pi, pi] range.
        
        This ensures the LQR angular math calculates the shortest rotational path 
        (e.g., executing a 90-degree right turn instead of a 270-degree left turn).
        
        Args:
            angles (np.ndarray): An array of angles in radians.
            
        Returns:
            np.ndarray: The wrapped angles.
        """
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    def localization_callback(self, msg):
        """
        Process incoming localization data, update state, and trigger the LQR solver.
        
        This is the heartbeat of the controller. It executes entirely synchronously 
        to guarantee zero phase-lag between sensor reception and thruster command output.
        
        Args:
            msg (Odometry): The incoming filtered odometry message.
        """
        # Start the profiling stopwatch
        start_time = time.perf_counter()
        
        roll_flu, pitch_flu, yaw_flu = self.quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # ----------------------------------------------------------------------
        # FRAME TRANSLATION: ROS ENU/FLU to Marine NED/FRD
        # ROS strictly uses East-North-Up (World) and Forward-Left-Up (Body).
        # Our physical LQR math model assumes North-East-Down (World) and 
        # Forward-Right-Down (Body). We intercept and translate them here.
        # ----------------------------------------------------------------------
        
        # Update the pre-allocated current_state array IN-PLACE (zero-copy overhead)
        
        # World Frame: ENU -> NED
        self.current_state[0] = msg.pose.pose.position.y     # X becomes North (Y)
        self.current_state[1] = msg.pose.pose.position.x     # Y becomes East (X)
        self.current_state[2] = -msg.pose.pose.position.z    # Z is inverted (Down)
        self.current_state[3] = roll_flu
        self.current_state[4] = -pitch_flu                   # Pitch is inverted
        
        # Shift Yaw 90 degrees so 0 Rad faces North instead of East
        yaw_ned = (math.pi / 2.0) - yaw_flu
        self.current_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi

        # Body Frame: FLU -> FRD (Velocities)
        self.current_state[6] = msg.twist.twist.linear.x
        self.current_state[7] = -msg.twist.twist.linear.y    # Left to Right
        self.current_state[8] = -msg.twist.twist.linear.z    # Up to Down
        self.current_state[9] = msg.twist.twist.angular.x
        self.current_state[10] = -msg.twist.twist.angular.y  # Pitch to -Pitch
        self.current_state[11] = -msg.twist.twist.angular.z  # Yaw to -Yaw
        
        # --- Target State Management ---
        with self.target_state_lock:
            # First-run initialization: If the Sub just booted, set its target 
            # coordinate to exactly where it currently is so it holds position.
            if np.isnan(self.target_state[0]):
                self.get_logger().info("Initializing target_state to current position (zero velocity).")
                self.target_state[0:6] = self.current_state[0:6]
                self.target_state[6:12] = 0.0 # Force target velocities to 0 for station-keeping
                
            # Copy to local thread-safe variable for math below so the Action Server
            # doesn't overwrite the memory addresses mid-calculation.
            target_state_copy = self.target_state.copy()
        
        # --- Execute Control ---
        if self._current_mode in [ControlMode.BEHAVIOR, ControlMode.LQR_TUNING]:
            
            # Native vectorized math to calculate the error
            lqr_error = target_state_copy - self.current_state
            
            # Crucial: Wrap Roll/Pitch/Yaw errors so the sub doesn't aggressively spin 360 degrees
            lqr_error[3:6] = self.wrap_angles_to_pi(lqr_error[3:6])
            
            # Fire the mathematical solver
            thrusters_force = self.lqr_solver.compute_thrust_force(
                self.current_state, lqr_error, self.q_matrix, self.r_matrix, self.inv_r_matrix
            )
            
            # Pack and fire to the hardware node
            thrust_msg = ThrusterCommand(efforts=thrusters_force.tolist())
            self.thruster_pub.publish(thrust_msg)
    	# Stop the stopwatch and calculate duration in milliseconds
        end_time = time.perf_counter()
        exec_time_ms = (end_time - start_time) * 1000.0
        # Log it safely (Only prints once per second to prevent I/O lag)
        self.get_logger().info(f"LQR Math Execution Time: {exec_time_ms:.3f} ms", throttle_duration_sec=1.0)

    def debug_target_callback(self, msg):
        """
        Directly update the target_state if in LQR tuning mode.
        
        **HUMAN-FRIENDLY OVERRIDE**
        To make debugging easier via CLI or the ASUQTR Dashboard, this callback 
        completely bypasses standard ROS ENU/Quaternion rules. 
        
        Position is expected STRICTLY in NED (North, East, Down).
        - msg.pose.position.x = North (meters)
        - msg.pose.position.y = East (meters)
        - msg.pose.position.z = Down (meters)
        
        Orientation is expected STRICTLY in NED Euler Degrees.
        - msg.pose.orientation.x = Roll (Degrees, NED)
        - msg.pose.orientation.y = Pitch (Degrees, NED)
        - msg.pose.orientation.z = Yaw (Degrees, NED)
        
        WARNING: Do NOT use RViz's 2D Nav Goal with this topic. RViz sends 
        ENU quaternions, which this function will wildly misinterpret.
        
        Args:
            msg (PoseStamped): The requested target pose. 
        """
        if self._current_mode != ControlMode.LQR_TUNING:
            return 
            
        with self.target_state_lock:
            if not np.isnan(self.target_state[0]):
                
                # 1. POSITION: Direct NED Mapping
                self.target_state[0] = msg.pose.position.x     # North
                self.target_state[1] = msg.pose.position.y     # East
                self.target_state[2] = msg.pose.position.z     # Down
                
                # 2. ORIENTATION: Direct NED Mapping + Degrees to Radians
                self.target_state[3] = math.radians(msg.pose.orientation.x) # Roll
                self.target_state[4] = math.radians(msg.pose.orientation.y) # Pitch
                
                # 3. Wrap Yaw safely in radians
                yaw_rad = math.radians(msg.pose.orientation.z)
                self.target_state[5] = (yaw_rad + math.pi) % (2 * math.pi) - math.pi
                
                self.get_logger().info(
                    f"Debug Target (NED): N:{self.target_state[0]:.1f}, E:{self.target_state[1]:.1f}, D:{self.target_state[2]:.1f} | "
                    f"R:{msg.pose.orientation.x:.1f}°, P:{msg.pose.orientation.y:.1f}°, Y:{msg.pose.orientation.z:.1f}°"
                )

    # ==========================================
    # GAMEPAD CALLBACK
    # ==========================================
    def gamepad_callback(self, msg):
        """
        Map gamepad joysticks directly to physical movement vectors.
        
        This translates manual analog sticks into thrust vectors using the 
        Thruster Allocation Matrix, completely bypassing the LQR controller.
        
        Args:
            msg (Joy): The incoming gamepad state.
        """
        # Extracting axis, button and data
        button_a = msg.buttons[0]  
        button_b = msg.buttons[1]  
        button_start = msg.buttons[7]  
        left_stick_x = msg.axes[0]  
        left_stick_y = msg.axes[1]  
        right_stick_x = msg.axes[3]  
        right_stick_y = -msg.axes[4]  
        triggers_axis = (msg.axes[2] - msg.axes[5]) / 2  

        if self._is_mode_switch_requested(button_start):
            return
        
        if self._current_mode != ControlMode.MANUAL:
            return

        right_stick_y = self._avoid_joystick_dead_zone(right_stick_y)
        right_stick_x = self._avoid_joystick_dead_zone(right_stick_x)
        left_stick_x = self._avoid_joystick_dead_zone(left_stick_x)
        left_stick_y = self._avoid_joystick_dead_zone(left_stick_y)

        # Build requested 6-DOF force vector from joysticks
        self.raw_cmd_vector = [left_stick_y, left_stick_x, triggers_axis, (button_a - button_b), -right_stick_y, right_stick_x]
        
        # Multiply by Thruster Allocation Matrix to determine which motors must spin 
        # to achieve the desired joystick vector.
        raw_thrusts = np.clip(THRUST_ALLOC_MAT.dot(self.raw_cmd_vector) * DEFAULT_MAX_THROTTLE, -DEFAULT_MAX_THROTTLE, DEFAULT_MAX_THROTTLE)
        
        thrust_msg = ThrusterCommand(efforts=raw_thrusts.tolist())
        self.thruster_pub.publish(thrust_msg)

    # ==========================================
    # ACTION SERVER (FLEXBE INTERFACE)
    # ==========================================
    async def control_action_callback(self, goal_handle):
        """
        Handle incoming autonomous navigation commands from state machines.
        
        Because it is `async`, it can safely block and loop while checking 
        `is_target_reached()` without starving the node's executor or blocking 
        the continuous execution of the LQR math in the localization callback.
        
        Args:
            goal_handle: The ROS 2 action server goal handle.
            
        Returns:
            Control.Result: The success or failure state of the navigation goal.
        """
        if self._current_mode != ControlMode.BEHAVIOR or np.isnan(self.current_state[0]):
            self.get_logger().warn("Rejecting Action Goal: Not in BEHAVIOR mode or missing state.")
            goal_handle.abort()
            return Control.Result(success=False)

        incoming_pose = goal_handle.request.target_pose
        
        try:
            # Validate target against the TF tree (handles frames like camera_link -> odom)
            target_pose_in_map = self.tf_buffer.transform(
                incoming_pose, self.global_reference_frame, timeout=Duration(seconds=1.0)
            )
            
            target_roll_enu, target_pitch_enu, target_yaw_enu = self.quaternion_to_euler(
                target_pose_in_map.pose.orientation.x, target_pose_in_map.pose.orientation.y,
                target_pose_in_map.pose.orientation.z, target_pose_in_map.pose.orientation.w
            )
            
            yaw_ned = (math.pi / 2.0) - target_yaw_enu

            # Write the new objective to the shared memory space
            with self.target_state_lock:
                if not np.isnan(self.target_state[0]):
                    # Apply Target ENU -> NED in-place
                    self.target_state[0] = target_pose_in_map.pose.position.y
                    self.target_state[1] = target_pose_in_map.pose.position.x
                    self.target_state[2] = -target_pose_in_map.pose.position.z
                    self.target_state[3] = target_roll_enu
                    self.target_state[4] = -target_pitch_enu
                    self.target_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
            
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'TF Error: {ex}')
            goal_handle.abort()
            return Control.Result(success=False)

        # --- Wait For Completion Loop ---
        while not self.is_target_reached():
            
            # Handle Preemption (e.g., FlexBE aborted the behavior)
            if goal_handle.is_cancel_requested:
                with self.target_state_lock:
                    # SAFE CANCELLATION: Instantly copy current array into target array memory.
                    # This tells the LQR "Your target is exactly where you are right now,"
                    # causing the Sub to brake heavily and station-keep safely.
                    self.target_state[:] = self.current_state[:]
                goal_handle.canceled()
                return Control.Result(success=False)
            
            # Yield execution back to the executor so other callbacks can run
            await asyncio.sleep(0.1) 
            
        goal_handle.succeed()
        return Control.Result(success=True)

    # ==========================================
    # HELPER FUNCTIONS
    # ==========================================
    def is_target_reached(self):
        """
        Evaluate if the Sub's current 6-DOF position satisfies the goal tolerances.
        
        Returns:
            bool: True if position and angle errors are within tolerance, False otherwise.
        """
        if np.isnan(self.current_state[0]):
            return False
            
        with self.target_state_lock:
            if np.isnan(self.target_state[0]):
                return False
            target_np = self.target_state.copy()
            
        # Fast array math to get absolute error magnitudes
        lqr_error = target_np - self.current_state
        lqr_error[3:6] = self.wrap_angles_to_pi(lqr_error[3:6])
        
        position_errors = lqr_error[0:3] 
        angle_errors = lqr_error[3:6]    
        
        pos_tolerance = 0.5 # 0.5 meters
        angle_tolerance = math.radians(10)  # 10 degrees
        
        if np.any(np.abs(position_errors) > pos_tolerance) or np.any(np.abs(angle_errors) > angle_tolerance):
            return False
        return True
    
    def _is_mode_switch_requested(self, mode_switch_button):
        """
        Intercept a gamepad button press and cycle through the ControlMode enum.
        
        Args:
            mode_switch_button (int): The state of the mapped switch button.
            
        Returns:
            bool: True if a switch occurred, False otherwise.
        """
        retval = False
        if mode_switch_button and not self.last_mode_switch_button_state:
            new_mode = 'manual'
            if self._current_mode == ControlMode.MANUAL:
                new_mode = 'lqr_tuning'
            elif self._current_mode == ControlMode.LQR_TUNING:
                new_mode = 'behavior'
            elif self._current_mode == ControlMode.BEHAVIOR:
                new_mode = 'manual'
                
            # Trigger ROS parameter callback internally to sync state smoothly
            updated_param = Parameter('control_mode', value=new_mode)
            self.set_parameters([updated_param])
            retval = True
        self.last_mode_switch_button_state = mode_switch_button
        return retval
    
    def _avoid_joystick_dead_zone(self, joystick):
        """
        Ignore minor joystick drift to prevent the sub from slowly creeping.
        
        Args:
            joystick (float): The raw analog axis reading.
            
        Returns:
            float: 0.0 if within dead zone, otherwise the raw reading.
        """
        return 0 if (-DEFAULT_JOY_DEAD_ZONE <= joystick <= DEFAULT_JOY_DEAD_ZONE) else joystick

    
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Convert a ROS 2 geometry_msgs Quaternion into standard Euler angles.
        
        Args:
            x (float): Quaternion X.
            y (float): Quaternion Y.
            z (float): Quaternion Z.
            w (float): Quaternion W.
            
        Returns:
            tuple: (roll, pitch, yaw) in radians.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    
    # Run the node with 4 threads so Action Servers and topic Subscriptions 
    # callbacks can process concurrently without blocking each other.
    executor = MultiThreadedExecutor(num_threads=4) 
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
