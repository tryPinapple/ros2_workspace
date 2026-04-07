#!/usr/bin/env python3

import csv
import math
import os
import sys
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

class YawDriftMonitor(Node):
    def __init__(self):
        super().__init__('yaw_drift_monitor')
        
        self.subscription = self.create_subscription(
            Imu,
            '/vectornav/imu',
            self.imu_callback,
            qos_profile_sensor_data 
        )
        
        self.initial_yaw = None
        self.start_time = None
        self.last_log_time = None
        self.max_drift = 0.0
        
        # Buffer to store data for the CSV dump
        self.data_log = [] 
        
        self.get_logger().info("Yaw Drift Monitor started. Waiting for IMU data...")

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        q = msg.orientation
        
        # Calculate yaw (in radians) from the quaternion
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Lock in the initial yaw and start the timer on the first message
        if self.start_time is None:
            self.start_time = current_time
            self.last_log_time = current_time
            self.initial_yaw = yaw
            self.get_logger().info(f"Initial Yaw locked at: {math.degrees(yaw):.2f}°")
            self.get_logger().info("Monitoring started. Auto-stop programmed for 15 minutes.")
            return

        # Calculate time elapsed
        elapsed_ns = current_time - self.start_time
        elapsed_sec = elapsed_ns / 1e9

        # Calculate drift (shortest angular distance)
        diff = yaw - self.initial_yaw
        diff = (diff + math.pi) % (2.0 * math.pi) - math.pi
        
        drift_deg = math.degrees(abs(diff))
        yaw_deg = math.degrees(yaw)

        # Update maximum drift
        if drift_deg > self.max_drift:
            self.max_drift = drift_deg

        # Append to our in-memory log for the CSV
        self.data_log.append([elapsed_sec, yaw_deg, drift_deg])

        # Throttle the terminal output to every 10 seconds
        if (current_time - self.last_log_time) >= 10 * 1e9:
            mins, secs = divmod(int(elapsed_sec), 60)
            self.get_logger().info(
                f"Time: {mins:02d}:{secs:02d} | "
                f"Yaw: {yaw_deg:.2f}° | "
                f"Drift: {drift_deg:.4f}° | "
                f"Max Drift: {self.max_drift:.4f}°"
            )
            self.last_log_time = current_time

        # Halt condition: 15 minutes (900 seconds)
        if elapsed_sec >= 900.0:
            self.get_logger().info("15-minute mark reached. Halting collection...")
            raise SystemExit # Caught gracefully in main()

    def save_csv(self):
        if not self.data_log:
            self.get_logger().info("No data collected to save.")
            return
            
        filename = f"vn100_yaw_drift_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath = os.path.join(os.getcwd(), filename)
        
        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Elapsed Time (s)", "Yaw (deg)", "Drift Magnitude (deg)"])
                writer.writerows(self.data_log)
            self.get_logger().info(f"SUCCESS: Data saved to {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YawDriftMonitor()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        # KeyboardInterrupt catches Ctrl+C
        # SystemExit catches our 15-minute auto-stop
        pass 
    finally:
        node.save_csv()
        node.get_logger().info(f"Final Max Drift observed: {node.max_drift:.4f}°")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
