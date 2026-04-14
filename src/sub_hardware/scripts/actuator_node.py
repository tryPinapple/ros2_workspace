#!/usr/bin/env python3
"""
================================================================================
ASUQTR Submarine's ROS 2 Actuator Node
================================================================================

This node acts as the primary hardware interface between the ROS 2 network and
the physical actuators (thrusters, grippers, lights, and torpedoes) on the Submarine.
It translates logical commands (e.g., Newtons of force) into hardware-level PWM
signals sent over the I2C bus to a PCA9685 PWM driver.

Design Highlights:
- I2C Thread Safety: Uses a MutuallyExclusiveCallbackGroup to prevent I2C bus collisions
  when multiple actuator topics receive commands concurrently under a MultiThreadedExecutor.
- Non-Linear Thrust Mapping: Uses NumPy to interpolate requested forces against
  empirical thruster data, accounting for deadbands and asymmetric thrust curves.
- Dynamic Configuration: Supports live updates to the thruster neutral-point offset
  to compensate for electrical ground loops without restarting the node.
- Safety watchdog to disable thrusters if no cmd is sent

ROS 2 Interface:
- Subscriptions:
  * `thruster_cmd` (sub_interfaces/ThrusterCommand): Output forces in newtons to thrusters
  * `gripper` (std_msgs/Float32): command for sub's gripper actuator
  * `subsea_light` (std_msgs/Float32): command for subsea light 
  * `torpedo` (std_msgs/Float32): command for sub's torpedo actuator
  * `kill_switch` (sensor_msgs/Bool): state change of magnetic kill switch on 
"""

# ==========================================
# PYTHON STANDARD & 3RD PARTY IMPORTS
# ==========================================
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import ContinuousServo, Servo
from busio import I2C
from board import SCL, SDA

# ==========================================
# ROS 2 IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float32
from sub_interfaces.msg import ThrusterCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


# Found experimentally with Adafruit's tuning script for PCA9685 to ensure accurate PWM
DEFAULT_REF_CLK_SPEED = 24821760
DEFAULT_PCA9685_ADDR = 0x40
DEFAULT_PCA9685_FREQ = 60


class SER110XLauncher(Servo):
    """
    Hardware interface for Blue Trail Engineering’s SER-110X depth-rated underwater servo.
    
    This operates as a standard positional servo (0 to 180 degrees mapping).
    The pulse width is varied within the range of 850 – 2350 microseconds to control 
    the absolute position of the servo horn.
    """
    MIN_PULSE = 850
    MAX_PULSE = 2350

    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class T200Thruster(ContinuousServo):
    """
    Hardware interface for Blue Robotics T200 Thrusters.
    
    Unlike standard servos, thruster Electronic Speed Controllers (ESCs) require 
    ContinuousServo logic, where the PWM signal maps to speed/throttle (-1.0 to 1.0) 
    rather than absolute position.
    
    Data Source:
        Performance data derived from the official Blue Robotics T200 Performance Charts 
        at 16V (typical for 4S LiPo setups). 
        Reference: https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
    """
    MIN_PULSE = 1100
    MAX_PULSE = 1900

    # T200 Thruster Performance Data (16V approximation)
    # Measured thrust in Newtons. 
    # Note: Array must be in monotonically increasing order for np.interp to function.
    known_forces_n = np.array([
            -40.22, # Max Reverse (Asymmetric, weaker than forward)
            -27.47, 
            -14.71, 
            -5.89, 
            0.0,    # Deadband start
            0.0,    # Center/Stopped
            0.0,    # Deadband end
            7.85, 
            19.62, 
            34.33, 
            51.50   # Max Forward
        ])

    # Corresponding ContinuousServo API commands [-1.0 to 1.0]
    # These represent the exact PWM values (from 1100us to 1900us) required to generate 
    # the corresponding force in the known_forces_n array.
    known_api_cmds = np.array([
            -1.0,    # 1100 us
            -0.75,   # 1200 us
            -0.50,   # 1300 us
            -0.25,   # 1400 us
            -0.0625, # 1475 us (Deadband boundary - lower)
            0.0,     # 1500 us (Stopped)
            0.0625,  # 1525 us (Deadband boundary - upper)
            0.25,    # 1600 us
            0.50,    # 1700 us
            0.75,    # 1800 us
            1.0      # 1900 us
        ])

    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class NewtonSubseaGripper(ContinuousServo):
    """
    Hardware interface for the Blue Robotics Newton Subsea Gripper.
    
    Operates via a ContinuousServo interface where PWM dictates the speed and 
    direction of the open/close mechanism.
    """
    MIN_PULSE = 1100
    MAX_PULSE = 1900
    
    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class LumenSubseaLight(ContinuousServo):
    """
    Hardware interface for the Blue Robotics
DEFAULT_PCA9685_ADDR = 0x40 Lumen Subsea Light.
    
    Operates via a ContinuousServo interface where the PWM signal maps to 
    brightness intensity rather than position or speed.
    """
    MIN_PULSE = 1100
    MAX_PULSE = 1900

    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class ActuatorNode(Node):
    """
    ROS 2 Node managing I2C physical actuators on the Submarine.
    """
    def __init__(self):
        super().__init__('actuator_node')
        
        # --- Load ROS2 parameters ---
        self.declare_parameter('pca_ref_clk_speed', DEFAULT_REF_CLK_SPEED)
        self.declare_parameter('thruster_throttle_offset', 0.0) # Used to fix neutral drift
        self.declare_parameter('use_flat_thruster_mapping', False)
        self.declare_parameter('flat_thruster_gain_primary', 0.2)
        self.declare_parameter('flat_thruster_gain_secondary', 0.1)
        self.declare_parameter('enable_thrusters_watchdog', True)

        pca_ref_clk_speed = self.get_parameter('pca_ref_clk_speed').value
        self.thruster_throttle_offset = self.get_parameter('thruster_throttle_offset').value
        self.use_flat_thruster_mapping = bool(self.get_parameter('use_flat_thruster_mapping').value)
        self.flat_thruster_gain_primary = float(self.get_parameter('flat_thruster_gain_primary').value)
        self.flat_thruster_gain_secondary = float(self.get_parameter('flat_thruster_gain_secondary').value)
        self.thrusters_watchdog_enabled = self.get_parameter('enable_thrusters_watchdog').value

        # Bind dynamic ROS2 reconfigure callback to allow live tuning
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- Initialize Hardware ---
        # Initialize I2C bus and PCA9685 driver.
        self.pca = PCA9685(I2C(SCL, SDA), address=DEFAULT_PCA9685_ADDR, reference_clock_speed=pca_ref_clk_speed)
        self.pca.frequency = DEFAULT_PCA9685_FREQ
        self.get_logger().info('PCA9685 initiated! Starting thrusters initialization...')

        # Hardware pin assignments (Refer to ASUQTR Pcb Control V01 Sch)
        thruster_pins = [7, 6, 5, 4, 3, 2, 15, 14]
        gripper_pin = 8
        led_pin = 9
        torpedo_pin = 13

        # Instantiate actuator objects using specific PCA channels
        self.thrusters = [T200Thruster(self.pca.channels[i]) for i in thruster_pins]
        self.gripper = NewtonSubseaGripper(self.pca.channels[gripper_pin])
        self.subsea_light = LumenSubseaLight(self.pca.channels[led_pin])
        self.torpedo = SER110XLauncher(self.pca.channels[torpedo_pin])

        # --- ROS 2 Thread Safety ---
        # The I2C bus is not thread-safe. Because we are using a MultiThreadedExecutor, 
        # concurrent messages on different topics (e.g., a gripper command and a thruster command 
        # arriving simultaneously) could attempt to write to the I2C bus at the exact same time, 
        # resulting in an OSError [Errno 121] Remote I/O error.
        # Grouping all actuator callbacks into this mutually exclusive group forces the 
        # executor to lock the I2C bus and process hardware commands sequentially.
        self.i2c_cb_group = MutuallyExclusiveCallbackGroup()

        # --- Subscriptions & QoS ---
        # 1. Define the MATCHING QoS Profile
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.thruster_sub = self.create_subscription(
            ThrusterCommand, 'thruster_cmd', self.thrusters_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.gripper_sub = self.create_subscription(
            Float32, 'gripper', self.gripper_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.subsea_light_sub = self.create_subscription(
            Float32, 'subsea_light', self.subsea_light_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.torpedo_sub = self.create_subscription(
            Float32, 'torpedo', self.torpedo_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.kill_switch_sub = self.create_subscription(
            Bool, 'kill_switch', self.kill_switch_callback, latched_qos, callback_group=self.i2c_cb_group)

        # --- Thruster Watchdog Timer for safety---
        # Set a timout of 1 sec for thruster commands. If timed out, kill thrusters.
        # Since this callback can operate thrusters, i needs to be in the i2c mutex callback group
        # Set it using seconds for human readability
        self.thrusters_watchdog_timeout = Duration(seconds=1.0)        
        self.last_watchdog_kick_time = self.get_clock().now()
        self.thrusters_watchdog_timer = self.create_timer(
            0.05, # Check for thruster cmd timout at 20hz frequency
            self.thrusters_watchdog_callback,
            callback_group=self.i2c_cb_group 
        )

        # --- Initialization State ---
        self.thrusters_need_init = False
        self.init_timer = None        
        self.trigger_thrusters_init()
        

    # ==========================================
    # PARAMETER HANDLING
    # ==========================================

    def parameter_callback(self, params):
        """
        Dynamically handles updates to ROS parameters at runtime.
        Specifically used to tune the thruster neutral offset if the Submarine 
        exhibits drifting while armed but idle.
        """
        successful = True
        reason = ""
        for param in params:
            if param.name == 'thruster_throttle_offset':
                if param.type_ == Parameter.Type.DOUBLE:
                    # Bound the offset to prevent dangerous surges if a typo occurs
                    if -0.15 <= param.value <= 0.15:
                        self.thruster_throttle_offset = param.value
                        self.get_logger().info(f"Successfully updated thruster throttle offset to: {self.thruster_throttle_offset}")
                    else:
                        successful = False
                        reason = "Offset rejected: Value must be safely between -0.15 and 0.15"
                else:
                    successful = False
                    reason = "Type rejected: thruster_throttle_offset must be a float (double)"
            elif param.name == 'use_flat_thruster_mapping':
                if param.type_ == Parameter.Type.BOOL:
                    self.use_flat_thruster_mapping = bool(param.value)
                    self.get_logger().info(f"Flat thruster mapping set to: {self.use_flat_thruster_mapping}")
                else:
                    successful = False
                    reason = "Type rejected: use_flat_thruster_mapping must be a bool"
            elif param.name == 'flat_thruster_gain_primary':
                if param.type_ == Parameter.Type.DOUBLE:
                    if 0.0 <= param.value <= 1.0:
                        self.flat_thruster_gain_primary = float(param.value)
                        self.get_logger().info(f"Flat thruster primary gain set to: {self.flat_thruster_gain_primary}")
                    else:
                        successful = False
                        reason = "Gain rejected: flat_thruster_gain_primary must be between 0.0 and 1.0"
                else:
                    successful = False
                    reason = "Type rejected: flat_thruster_gain_primary must be a float (double)"
            elif param.name == 'flat_thruster_gain_secondary':
                if param.type_ == Parameter.Type.DOUBLE:
                    if 0.0 <= param.value <= 1.0:
                        self.flat_thruster_gain_secondary = float(param.value)
                        self.get_logger().info(f"Flat thruster secondary gain set to: {self.flat_thruster_gain_secondary}")
                    else:
                        successful = False
                        reason = "Gain rejected: flat_thruster_gain_secondary must be between 0.0 and 1.0"
                else:
                    successful = False
                    reason = "Type rejected: flat_thruster_gain_secondary must be a float (double)"

        return SetParametersResult(successful=successful, reason=reason)
    
    # ==========================================
    # ACTUATORS CALLBACKS
    # ==========================================
    def gripper_callback(self, msg):
        """Processes continuous servo commands [-1.0 to 1.0] for the gripper."""
        cmd = msg.data
        if -1.0 <= cmd <= 1.0:
           self.gripper.throttle = cmd
        else:
            self.get_logger().warn(f"Ignored out of bound Gripper command {cmd}! must be [-1.0, 1.0]")

    def torpedo_callback(self, msg):
        """Processes continuous servo commands [-1.0 to 1.0] for the torpedo launcher."""
        cmd = msg.data
        if -1.0 <= cmd <= 1.0:
            self.torpedo.throttle = cmd
        else:
            self.get_logger().warn(f"Ignored out of bound Torpedo command {cmd}! must be [-1.0, 1.0]")

    def subsea_light_callback(self, msg):
        """Processes brightness commands for the subsea lights."""
        cmd = msg.data
        # Servo class uses 'angle' commands from 0 to 180(default temp value)
        # This does not make sense conceptually, but is good enough to command the light for now
        # TODO fix this to make more sense, angle cmd to a light is wrong
        if 0 <= cmd <= 180:
            self.subsea_light.angle = cmd
        else:
            self.get_logger().warn(f"Ignored out of bound Subsea Light command {cmd}! must be [0, 180]")
    
    # ==========================================
    # THRUSTERS CALLBACKS
    # ==========================================
    def thrusters_callback(self, msg):
        """
        Maps requested Newtons of force to PWM throttle signals for the T200 thrusters.
        """
        # Kick thrusters watchdog because new command was received. It has to be kicked
        # even if the thruster are in initialization sequence, to avoid the watchdog trying to
        # send a new command to the thrusters
        self.last_watchdog_kick_time = self.get_clock().now()
        # Drop incoming commands if the ESCs are undergoing their boot/arming sequence
        if self.thrusters_need_init:
            return
        
        efforts = np.asarray(msg.efforts, dtype=np.float64)

        if self.use_flat_thruster_mapping:
            # Temporary debug mode: ignore the non-linear curve and use a flat gain.
            flat_gains = np.array([
                self.flat_thruster_gain_primary,   # 0
                self.flat_thruster_gain_primary,   # 1
                self.flat_thruster_gain_secondary, # 2
                self.flat_thruster_gain_secondary, # 3
                self.flat_thruster_gain_secondary, # 4
                self.flat_thruster_gain_secondary, # 5
                self.flat_thruster_gain_primary,   # 6
                self.flat_thruster_gain_primary,   # 7
            ], dtype=np.float64)
            interpolated_throttles_cmd = efforts * flat_gains
        else:
            # np.interp handles both non-linear mapping and bounds-clipping in one optimized pass.
            interpolated_throttles_cmd = np.interp(
                efforts,
                T200Thruster.known_forces_n,
                T200Thruster.known_api_cmds
            )
        
        # Apply the user-defined electrical neutral offset to fix drifting
        offset_cmds = interpolated_throttles_cmd + self.thruster_throttle_offset
        
        # Hard clip again to ensure the applied offset doesn't push the command past limits
        final_cmds = np.clip(offset_cmds, -1.0, 1.0)
        
        # Transmit to hardware
        for i, throttle in enumerate(final_cmds):
            self.thrusters[i].throttle = throttle

    def thrusters_init_complete_callback(self):
        """
        Fires once the thrusters' initialization timer ends, releasing the lockout on thruster commands.
        """
        self.get_logger().info("Thrusters' Electronic Speed Controller initialization sequence completed! Resuming normal thrusters operation.")
        self.thrusters_need_init = False
        self.init_timer.cancel()

    def kill_switch_callback(self, msg):
        """
        Triggers a safety lockout and re-arms the ESCs when a "killswitch latched" command is received.
        """
        if msg.data and not self.thrusters_need_init:
            self.thrusters_need_init = True         # Lock out incoming commands in the thrusters_callback
            self.trigger_thrusters_init()

    def thrusters_watchdog_callback(self):
        """ 
        Safety Watchdog Monitoring: If no thruster command is received, this can mean that ROS2 nodes
        responsible for thruster force are dead or hanging. This watchdog detects that no thruster command has been
        set in the last safe interval (1 sec, could be changed) and sends 0 commands to thrusters
        to kill them
        """
        # If you need to test this node as standalone, you would want the thrusters' watchdog
        # to be disabled, so it is possible to do so with a config yaml file.
        if self.thrusters_watchdog_enabled:
            now = self.get_clock().now()
            if (now - self.last_watchdog_kick_time) > self.thrusters_watchdog_timeout:
                sec_elapsed = (now - self.last_watchdog_kick_time).nanoseconds / 1e9
                self.get_logger().warn(
                    f"SAFETY FAULT: Last thruster command received {sec_elapsed:.2f} seconds ago. Sending 0N to thrusters!",
                    throttle_duration_sec=5.0 
                )
                for thruster in self.thrusters:
                    thruster.throttle = 0.0


    # ==========================================
    # HELPER FUNCTIONS
    # ==========================================

    def trigger_thrusters_init(self):
        """
        Forces thrusters to neutral (0-PWM) and enforces a 1-second software lockout.
        
        This non-blocking approach guarantees hardware safety while allowing the 
        MultiThreadedExecutor to process other topics (like lights or grippers) 
        during the 1-second Thrusters' ESC boot sequence.
        """
        self.get_logger().info("Starting T200 Thrusters' ESC initialization sequence...")
        
        # Safely force hardware to neutral state
        for thruster in self.thrusters:
            thruster.throttle = 0.0
            
        # Clear any existing timers (e.g., if kill_switch is spammed)
        if self.init_timer is not None:
            self.init_timer.cancel()
            
        # Spawn a one-shot timer to release the lock. Placed in the same callback group 
        # to guarantee it safely accesses shared state without race conditions.
        self.init_timer = self.create_timer(
            1.0, # seconds
            self.thrusters_init_complete_callback, 
            callback_group=self.i2c_cb_group
        )
                

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    
    executor = MultiThreadedExecutor(num_threads=2) 
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
