#!/usr/bin/env python3

# ==========================================
# PYTHON STANDARD & 3RD PARTY IMPORTS
# ==========================================
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

# ==========================================
# ROS 2 IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

        # ---------------------------------------------------------
        # STATIC FRAME TRANSFORMATION MATRICES
        # ---------------------------------------------------------
        # Maps ENU vectors (East, North, Up) to NED vectors (North, East, Down)
        # X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
        self.M_ned_enu = np.array([
            [ 0.0,  1.0,  0.0],
            [ 1.0,  0.0,  0.0],
            [ 0.0,  0.0, -1.0]
        ])

        # Maps FLU vectors (Fwd, Left, Up) to FRD vectors (Fwd, Right, Down)
        # X_frd = X_flu, Y_frd = -Y_flu, Z_frd = -Z_flu
        self.M_frd_flu = np.array([
            [ 1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0, -1.0]
        ])
    
    def imu_callback(self, msg):
        quat_ros = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        R_enu_flu = R.from_quat(quat_ros).as_matrix()

        R_ned_frd_mat = self.M_ned_enu @ R_enu_flu @ self.M_frd_flu
        
        # 1. Save the SciPy Rotation object to the class for the error calculation
        self.R_imu_ned = R.from_matrix(R_ned_frd_mat)

        yaw_ned, pitch_ned, roll_ned = self.R_imu_ned.as_euler('zyx', degrees=False)
        
        self.imu_state[3] = roll_ned
        self.imu_state[4] = pitch_ned                   
        self.imu_state[5] = yaw_ned
        
        euler_msg = Quaternion()
        euler_msg.x = math.degrees(self.imu_state[3])
        euler_msg.y = math.degrees(self.imu_state[4])
        euler_msg.z = math.degrees(self.imu_state[5])
        self.imu_euler_pub.publish(euler_msg)

    def localization_callback(self, msg):
        quat_ros = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        R_enu_flu = R.from_quat(quat_ros).as_matrix()

        R_ned_frd_mat = self.M_ned_enu @ R_enu_flu @ self.M_frd_flu
        
        # 1. Save the SciPy Rotation object
        self.R_filtered_ned = R.from_matrix(R_ned_frd_mat)

        yaw_ned, pitch_ned, roll_ned = self.R_filtered_ned.as_euler('zyx', degrees=False)
        
        self.filtered_state[3] = roll_ned
        self.filtered_state[4] = pitch_ned                   
        self.filtered_state[5] = yaw_ned
        
        euler_msg = Quaternion()
        euler_msg.x = math.degrees(self.filtered_state[3])
        euler_msg.y = math.degrees(self.filtered_state[4])
        euler_msg.z = math.degrees(self.filtered_state[5])
        self.filtered_euler_pub.publish(euler_msg)

        # ----------------------------------------------------------------------
        # ERROR CALCULATION (Relative Rotation)
        # ----------------------------------------------------------------------
        # Ensure both objects exist before calculating
        if hasattr(self, 'R_imu_ned') and hasattr(self, 'R_filtered_ned'):
            
            # R_error = Inverse(R_filtered) * R_imu
            # SciPy Rotation objects use * for matrix multiplication
            R_error = self.R_filtered_ned.inv() * self.R_imu_ned
            
            # Extract the error angles directly in degrees
            err_yaw, err_pitch, err_roll = R_error.as_euler('zyx', degrees=True)
            
            euler_error_msg = Quaternion()
            euler_error_msg.x = err_roll
            euler_error_msg.y = err_pitch
            euler_error_msg.z = err_yaw
            self.euler_error_pub.publish(euler_error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LQRTuning()
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