#!/usr/bin/env python3
"""
================================================================================
ASUQTR Sub's ROS 2 Sensor Node
================================================================================
ROS2 Node for the Blue Robotics Bar30 High-Resolution Depth Sensor.

This module initializes the Bar30 sensor over I2C, reads pressure and temperature
to calculate depth, and publishes the data as a ROS2 PoseWithCovarianceStamped message.
It is designed with readability in mind for student robotics projects.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
import time

# Third-party library for the MS5837 pressure sensor (Bar30)
# Ensure this module is available in your workspace or system Python path.
from workspace.packages.sub_hardware.sub_hardware.ms5837 import MS5837_30BA, OSR_4096, DENSITY_FRESHWATER

# -----------------------------------------------------------------------------
# Node Configuration Constants
# -----------------------------------------------------------------------------
SENSOR_I2C_BUS = 1
PUBLISH_RATE_HZ = 20.0  # 20 Hz is a standard update rate for depth control loops
OVERSAMPLING_RATIO = OSR_4096
FLUID_DENSITY = DENSITY_FRESHWATER # Use DENSITY_SALTWATER if diving in the ocean


class SensorNode(Node):
    """
    A ROS2 node that interfaces with the Bar30 depth sensor via I2C.

    This node runs a timer to periodically poll the sensor and publish its calculated
    depth. It uses a mutually exclusive callback group to isolate the blocking I2C
    reads so they do not interfere with other potential operations in this node.
    """

    def __init__(self):
        """
        Initializes the node, sets up the I2C sensor hardware, defines the Quality
        of Service (QoS) profile, and creates the publisher and polling timer.
        """
        # Name the node 'depth_sensor_node' in the ROS2 graph
        super().__init__('depth_sensor_node')

        # ---------------------------------------------------------
        # 1. Hardware Initialization with Retry Logic
        # ---------------------------------------------------------
        self.get_logger().info('Initializing Bar30 sensor on I2C bus...')
        self.depth_sensor = MS5837_30BA(SENSOR_I2C_BUS)
        if self.depth_sensor._bus is None:
            raise IOError('Couldnt initialize I2C bus for BAR30 depth sensor!')
        self.depth_sensor.setFluidDensity(FLUID_DENSITY)
        
        max_retries = 3
        retry_delay = 2.0
        is_initialized = False

        for attempt in range(1, max_retries + 1):
            try:
                # init() can raise an OSError if the bus is locked,
                # or return False if it communicates but fails calibration checks.
                if self.depth_sensor.init():
                    is_initialized = True
                    self.get_logger().info('Bar30 sensor initialized successfully.')
                    break
                else:
                    self.get_logger().warn(
                        f'Bar30 init returned False (Attempt {attempt}/{max_retries}). '
                        f'Retrying in {retry_delay}s...'
                    )
            except OSError as e:
                # Catches OS-level I2C errors like [Errno 11] or [Errno 121]
                self.get_logger().warn(
                    f'I2C OS Error: {e} (Attempt {attempt}/{max_retries}). '
                    f'Retrying in {retry_delay}s...'
                )
            
            # Sleep before the next retry, but don't sleep after the final attempt
            if attempt < max_retries:
                time.sleep(retry_delay)

        # If we exhausted all retries and it's still not initialized, kill the node.
        if not is_initialized:
            self.get_logger().error('Failed to initialize Bar30 sensor after all retries! Halting node.')
        
        # ---------------------------------------------------------
        # 2. ROS2 Publishers & QoS (Quality of Service) Setup
        # ---------------------------------------------------------
        # We use a BEST_EFFORT reliability policy. For high-frequency sensor streams,
        # it is usually better to drop an old, delayed message than to clog the network 
        # trying to resend it. We only care about the *latest* depth reading.
        sensor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.depth_pub = self.create_publisher(
            Odometry, 
            'depth', 
            sensor_qos
        )

        # ---------------------------------------------------------
        # 3. Timers & Concurrency
        # ---------------------------------------------------------
        # Because reading I2C is a "blocking" operation (it pauses Python while waiting 
        # for the hardware), we isolate this timer in its own callback group. This will be
        # necessary if eventually other sensors are added on this i2c bus
        self.i2c_cb_group = MutuallyExclusiveCallbackGroup()
        
        timer_period_seconds = 1.0 / PUBLISH_RATE_HZ
        self.read_depth_timer = self.create_timer(
            timer_period_seconds,
            self._read_depth_callback,
            callback_group=self.i2c_cb_group 
        )
        
        self.get_logger().info(f'Depth sensor node ready, publishing at {PUBLISH_RATE_HZ} Hz.')

    def _read_depth_callback(self):
        """
        Timer callback that triggers the sensor hardware read and publishes the ROS2 message.
        
        Note: The `sensor.read()` method halts execution briefly based on the oversampling 
        ratio. For OSR_4096, it blocks for about 20ms total to read both pressure and temp.
        """
        # 1. Trigger the hardware to update its internal pressure and temperature registers
        if not self.depth_sensor.read(OVERSAMPLING_RATIO):
            self.get_logger().warn('Failed to read from Bar30 sensor. Skipping publish.')
            return

        # 2. Construct the ROS2 message
        msg = Odometry()
        
        # Attach the current ROS2 time to the message header
        msg.header.stamp = self.get_clock().now().to_msg()
        # The frame_id tells TF2 (the transform system) where this sensor is physically mounted
        msg.header.frame_id = 'odom' 
        msg.child_frame_id = 'bar30_link'
        
        # Extract the calculated depth (in meters) and assign it to the Z-axis position.
        # X and Y remain 0.0, as this sensor only knows its vertical depth.
        # The depth increases while going deeper, which means the axis is DOWN (NED frame). For
        # ROS standard, it needs to be in ENU frame, thus the depth is negated to invert the Z axis
        msg.pose.pose.position.z = -self.depth_sensor.depth()
        
        # 3. Define the measurement uncertainty (Variance)
        # The covariance matrix is a flat 36-element array representing a 6x6 matrix 
        # for X, Y, Z, Roll, Pitch, Yaw. The diagonal elements represent the variance.
        # Row 3, Column 3 (which is index 14 in a 0-indexed array) is the Z-axis variance.
        # A variance of 0.0001 m^2 means a standard deviation of 0.01 m (or 1 cm) of noise.
        msg.pose.covariance[14] = 0.0001 
        
        # 4. Publish the data to the rest of the software stack
        self.depth_pub.publish(msg)


def main(args=None):
    """
    Entry point for the ROS2 executable. Initializes the ROS client library,
    spins up the node to keep it alive, and handles clean shutdowns.
    """
    rclpy.init(args=args)
    
    # Instantiate our custom node class
    node = SensorNode()
    
    try:
        # rclpy.spin() blocks and keeps the node running, processing timers and callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Provide a clean, accurate exit message when the user hits Ctrl+C in the terminal
        node.get_logger().info('Keyboard interrupt detected. Shutting down the Sub depth sensor node...')
    finally:
        # Cleanup resources safely before exiting
        node.destroy_node()
        # Check if rclpy is still valid before trying to shut it down
        if rclpy.ok():
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()