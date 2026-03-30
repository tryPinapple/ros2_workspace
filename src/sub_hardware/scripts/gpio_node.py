#!/usr/bin/env python3
"""
================================================================================
ASUQTR Sub's ROS 2 GPIO Node
================================================================================
Module: gpio_node.py
Author: ASUQTR Robotics Club
Target: Jetson Xavier NX / AGX

Description:
This node acts as a hardware-to-software gateway. It interfaces directly with 
the Jetson Xavier's 40-pin GPIO header to read physical input states (like switches 
and leak sensors) and write digital outputs (like fan control and safety disables).

Safety Architecture Note (The PCA9685 /OE Pin):
------------------------------------------------
This node implements a dual-layer safety system using the Killswitch. 
1. Hardware Layer: Pulling the physical killswitch electrically cuts power to 
   the thrusters/power circuitry. 
2. Software Layer: When the killswitch is pulled, this node detects the state change 
   and immediately pulls the `/OE` (Output Enable) pin of the PCA9685 PWM driver HIGH. 
   The PCA9685's `/OE` pin is active-LOW, meaning pulling it HIGH instantly disables 
   all PWM signals to the ESCs (Electronic Speed Controllers). This ensures that even 
   if the hardware power cut fails or arcs, the software stops commanding the thrusters.
"""

from Jetson import GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# ==========================================
# CONFIGURATION & CONSTANTS
# ==========================================
# Default Board Pin Numbers (Physical pin numbers on the 40-pin header)
DEF_KILL_SWITCH_PIN = 36
DEF_MAG_SWITCH2_PIN = 38
DEF_MAG_SWITCH3_PIN = 40
DEF_SIGNAL_LEAK_PIN = 7
DEF_ALERT_TEMP_PIN = 15
DEF_MAIN_FAN_PIN = 16
DEF_DISABLE_PWM_PIN = 11

# Default debounce time in milliseconds to prevent noisy mechanical switch signals
DEF_DEBOUNCE_TIME = 200  


class GPIONode(Node):
    """
    ROS 2 Node bridging Jetson GPIO and the ROS network.
    
    This node utilizes interrupt-driven callbacks for inputs (so we don't waste CPU 
    polling the pins) and ROS 2 subscriptions to trigger output pins.
    """
    def __init__(self):
        super().__init__('gpio_node')

        # ----------------------------------------------------------------------
        # 1. HARDWARE SETUP
        # ----------------------------------------------------------------------
        # Use physical pin numbers (1-40) on the Jetson header rather than SOC names
        GPIO.setmode(GPIO.BOARD)  
        
        # ----------------------------------------------------------------------
        # 2. QUALITY OF SERVICE (QoS) PROFILES
        # ----------------------------------------------------------------------
        # Latched QoS: Used for critical states (Killswitch, Leaks). 
        # - RELIABLE ensures the message isn't lost over the network.
        # - TRANSIENT_LOCAL saves the last state. If a thruster node boots up 
        #   5 minutes after the GPIO node, it instantly gets the current switch state.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Volatile QoS: Used for continuous commands (Fans).
        # - BEST_EFFORT prevents network bottlenecks.
        # - VOLATILE ensures late subscribers don't execute old, stale commands.
        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # ----------------------------------------------------------------------
        # 3. THREADING & CALLBACK GROUPS
        # ----------------------------------------------------------------------
        # We put GPIO output commands in a Mutually Exclusive Callback Group.
        # This prevents race conditions where two rapid ROS messages try to 
        # change the physical GPIO bus states at the exact same microsecond.
        self.gpio_cb_group = MutuallyExclusiveCallbackGroup()

        # ----------------------------------------------------------------------
        # 4. OUTPUTS (ROS Subscribers -> GPIO Outputs)
        # ----------------------------------------------------------------------
        self.get_logger().info("Configuring GPIO Outputs...")

        # Main Pod Fans
        self.main_pod_fans_pin = self.declare_parameter('main_pod_fans_pin', DEF_MAIN_FAN_PIN).value
        GPIO.setup(self.main_pod_fans_pin, GPIO.OUT, initial=True)
        self.main_pod_fans_sub = self.create_subscription(
            Bool, 
            'main_pod_fans', 
            self.main_pod_fans_callback, 
            only_latest_qos,
            callback_group=self.gpio_cb_group
        )

        # PWM Disable (/OE Pin on PCA9685)
        # Initial state is False (LOW), meaning PWM is ENABLED by default.
        self.disable_pwms_pin = self.declare_parameter('disable_pwms_pin', DEF_DISABLE_PWM_PIN).value
        GPIO.setup(self.disable_pwms_pin, GPIO.OUT, initial=False) 
        self.disable_pwm_sub = self.create_subscription(
            Bool, 
            'disable_pwm', 
            self.disable_pwm_callback, 
            latched_qos,
            callback_group=self.gpio_cb_group
        )

        # ----------------------------------------------------------------------
        # 5. INPUTS (GPIO Interrupts -> ROS Publishers)
        # ----------------------------------------------------------------------
        self.get_logger().info("Configuring GPIO Inputs and Interrupts...")

        # Kill Switch Setup
        self.kill_switch_pin = self.declare_parameter('kill_switch_pin', DEF_KILL_SWITCH_PIN).value
        debounce = self.declare_parameter('kill_switch_debounce_time', DEF_DEBOUNCE_TIME).value
        
        self.kill_switch_pub = self.create_publisher(Bool, 'kill_switch', latched_qos)
        GPIO.setup(self.kill_switch_pin, GPIO.IN)
        # add_event_detect triggers the callback automatically on RISING or FALLING edges (BOTH)
        GPIO.add_event_detect(self.kill_switch_pin, GPIO.BOTH, callback=self.handle_kill_switch_event, bouncetime=debounce)
        
        # Magnetic Switch 2 Setup
        self.mag_switch2_pin = self.declare_parameter('magnetic_switch2_pin', DEF_MAG_SWITCH2_PIN).value
        debounce = self.declare_parameter('mag_switch2_debounce_time', DEF_DEBOUNCE_TIME).value
        
        self.mag_switch2_pub = self.create_publisher(Bool, 'magnetic_switch_2', latched_qos)
        GPIO.setup(self.mag_switch2_pin, GPIO.IN)
        GPIO.add_event_detect(self.mag_switch2_pin, GPIO.BOTH, callback=self.handle_magnetic_switch2_event, bouncetime=debounce)
        
        # Magnetic Switch 3 Setup
        self.mag_switch3_pin = self.declare_parameter('magnetic_switch3_pin', DEF_MAG_SWITCH3_PIN).value
        debounce = self.declare_parameter('mag_switch3_debounce_time', DEF_DEBOUNCE_TIME).value
        
        self.mag_switch3_pub = self.create_publisher(Bool, 'magnetic_switch_3', latched_qos)
        GPIO.setup(self.mag_switch3_pin, GPIO.IN)
        GPIO.add_event_detect(self.mag_switch3_pin, GPIO.BOTH, callback=self.handle_magnetic_switch3_event, bouncetime=debounce)
        
        # Water Leak Sensor Setup
        self.water_leak_pin = self.declare_parameter('signal_leak_pin', DEF_SIGNAL_LEAK_PIN).value
        
        self.water_leak_pub = self.create_publisher(Bool, 'water_leak', latched_qos)
        GPIO.setup(self.water_leak_pin, GPIO.IN)
        GPIO.add_event_detect(self.water_leak_pin, GPIO.BOTH, callback=self.handle_leak_event)

        # Temperature Alert Setup
        self.temp_alert_pin = self.declare_parameter('alert_temp_pin', DEF_ALERT_TEMP_PIN).value
        
        self.temp_alert_pub = self.create_publisher(Bool, 'temperature_alert', latched_qos)
        GPIO.setup(self.temp_alert_pin, GPIO.IN)
        GPIO.add_event_detect(self.temp_alert_pin, GPIO.BOTH, callback=self.handle_alert_temp_event)

        self.get_logger().info("GPIO Node successfully initialized and spinning.")

    # ==========================================
    # ROS SUBSCRIBER CALLBACKS (Outputs)
    # ==========================================
    
    def main_pod_fans_callback(self, msg: Bool):
        """
        Triggered when a message is received on the 'main_pod_fans' topic.
        Changes the physical state of the fan's GPIO pin.
        """
        # std_msgs/Bool holds its boolean value inside the '.data' attribute
        GPIO.output(self.main_pod_fans_pin, msg.data)

    def disable_pwm_callback(self, msg: Bool):
        """
        Triggered when a message is received on the 'disable_pwm' topic.
        True = Disable PWMs (Pull /OE High)
        False = Enable PWMs (Pull /OE Low)
        """
        GPIO.output(self.disable_pwms_pin, msg.data)


    # ==========================================
    # GPIO INTERRUPT CALLBACKS (Inputs)
    # ==========================================
    # Note: Jetson.GPIO requires these functions to accept one argument (the channel/pin)
    
    def handle_kill_switch_event(self, pin):
        """
        Hardware interrupt triggered when the physical kill switch changes state.
        This handles both the ROS publishing and the redundant PCA9685 /OE safety cutoff.
        """
        is_armed = bool(GPIO.input(pin))
        
        if is_armed:
            self.get_logger().info("Killswitch ARMED. Enabling thruster PWMs.")
            GPIO.output(self.disable_pwms_pin, False) # Pull /OE LOW (Enable)
        else:
            self.get_logger().warn("Killswitch PULLED! Disabling thruster PWMs.")
            GPIO.output(self.disable_pwms_pin, True)  # Pull /OE HIGH (Disable)
            
        # Publish the new state to the ROS network
        msg = Bool()
        msg.data = is_armed
        self.kill_switch_pub.publish(msg)

    def handle_magnetic_switch2_event(self, pin):
        """Interrupt for Magnetic Switch 2 state change."""
        is_latched = bool(GPIO.input(pin))
        self.get_logger().info(f"Magnetic switch 2 event detected: {is_latched}")
        
        msg = Bool()
        msg.data = is_latched
        self.mag_switch2_pub.publish(msg)

    def handle_magnetic_switch3_event(self, pin):
        """Interrupt for Magnetic Switch 3 state change."""
        is_latched = bool(GPIO.input(pin))
        self.get_logger().info(f"Magnetic switch 3 event detected: {is_latched}")
        
        msg = Bool()
        msg.data = is_latched
        self.mag_switch3_pub.publish(msg)

    def handle_leak_event(self, pin):
        """Interrupt for water leak detection."""
        is_leaking = bool(GPIO.input(pin))
        
        if is_leaking:
            self.get_logger().fatal("WATER LEAK DETECTED! Electrical damage imminent!")
            
        msg = Bool()
        msg.data = is_leaking
        self.water_leak_pub.publish(msg)

    def handle_alert_temp_event(self, pin):
        """Interrupt for internal main pod high temperature alert."""
        is_temp_high = bool(GPIO.input(pin))
        
        if is_temp_high:
            self.get_logger().warning("High temperature detected in sub's main pod!")
            
        msg = Bool()
        msg.data = is_temp_high
        self.temp_alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPIONode()
    
    try:
        # Spin the node to keep it alive and listening to callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Shutting down GPIO node...')
    finally:
        # Cleanup is crucial in embedded systems to release the GPIO pins safely
        node.destroy_node()
        GPIO.cleanup()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
