import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class MockOdomPublisher(Node):
    def __init__(self):
        super().__init__('mock_odom_publisher')
        
        # Publish to the topic your control node is listening to
        self.publisher_ = self.create_publisher(Odometry, '/odometry/filtered', 10)
        
        # Publish at 10 Hz (0.1 seconds). Adjust this if your LQR needs a faster loop.
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Mock Odometry Publisher started at 10 Hz...')

    def timer_callback(self):
        msg = Odometry()

        # 1. Header & Frames
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 2. Pose (Position and Orientation)
        # Simulating the Sub at a depth of 2 meters
        msg.pose.pose.position.x = 8.0
        msg.pose.pose.position.y = 4.0
        msg.pose.pose.position.z = -2.0
        
        # Identity quaternion (facing forward, flat)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # 3. Twist (Linear and Angular Velocity)
        # Simulating moving forward at 0.5 m/s
        msg.twist.twist.linear.x = 0.5
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mock_odom_publisher = MockOdomPublisher()
    
    try:
        rclpy.spin(mock_odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        mock_odom_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
