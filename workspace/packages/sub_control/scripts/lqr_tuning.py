#!/usr/bin/env python3

# ==========================================
# PYTHON STANDARD & 3RD PARTY IMPORTS
# ==========================================
import time
from enum import IntEnum
import numpy as np
import math

# ==========================================
# ROS 2 IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Messages and TF2
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# ==========================================
# NODE DEFINITION
# ==========================================
class LQRTuning(Node):
    def __init__(self):
        """Initialize the node, parameters, pre-allimport numpy as np
ocated memory, ROS publishers/Subscribers
        callback mutex groups and TF2 buffer & frame to allow frame transfer when receiving target
        commands """
        super().__init__('lqr_tuning')
        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        imu_grp = MutuallyExclusiveCallbackGroup()
        odom_grp = MutuallyExclusiveCallbackGroup()

        self.localization_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.localization_callback, only_latest_qos, callback_group=odom_grp,
        )
        self.imu_sub = self.create_subscription(
            Imu, 'vectornav/imu', self.imu_callback,  only_latest_qos, callback_group=imu_grp,
        )
        
        self.filtered_state = np.full(12, np.nan, dtype=np.float64)
        self.imu_state = np.full(12, np.nan, dtype=np.float64)

        self.filtered_euler_pub = self.create_publisher(
            Quaternion, 'filtered_ned_euler', only_latest_qos
        )
        self.imu_euler_pub = self.create_publisher(
            Quaternion, 'imu_ned_euler', only_latest_qos,
        )
        self.euler_error_pub = self.create_publisher(
            Quaternion, 'ned_euler_error', only_latest_qos
        )
    
    def imu_callback(self, msg):
        # 1. Get radians
        roll_flu, pitch_flu, yaw_flu = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        
        self.imu_state[3] = roll_flu
        self.imu_state[4] = -pitch_flu                   
        
        # 2. Math is safely done in Radians
        yaw_ned = (math.pi / 2.0) - yaw_flu
        self.imu_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
        
        # 3. Publish the NED state in Degrees for human readability
        euler_msg = Quaternion()
        euler_msg.x = math.degrees(self.imu_state[3])
        euler_msg.y = math.degrees(self.imu_state[4])
        euler_msg.z = math.degrees(self.imu_state[5])
        self.imu_euler_pub.publish(euler_msg)

    def localization_callback(self, msg):
        # 1. Get radians
        roll_flu, pitch_flu, yaw_flu = self.quaternion_to_euler(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )
        
        self.filtered_state[3] = roll_flu
        self.filtered_state[4] = -pitch_flu                   
        
        # 2. Math is safely done in Radians
        yaw_ned = (math.pi / 2.0) - yaw_flu
        self.filtered_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
        
        # 3. Publish the NED state in Degrees for human readability
        euler_msg = Quaternion()
        euler_msg.x = math.degrees(self.filtered_state[3])
        euler_msg.y = math.degrees(self.filtered_state[4])
        euler_msg.z = math.degrees(self.filtered_state[5])
        self.filtered_euler_pub.publish(euler_msg)

        # 4. Error in Degrees
        euler_error_msg = Quaternion()
        euler_error_msg.x = math.degrees(self.imu_state[3] - self.filtered_state[3])
        euler_error_msg.y = math.degrees(self.imu_state[4] - self.filtered_state[4])
        euler_error_msg.z = math.degrees(self.imu_state[5] - self.filtered_state[5])
        self.euler_error_pub.publish(euler_error_msg)

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """ Returns Radians. """
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
    node = LQRTuning()
    
    # Run the node with 4 threads so Action Servers and topic Subscriptions 
    # callbacks can process concurrently without blocking each other.
    executor = SingleThreadedExecutor() 
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