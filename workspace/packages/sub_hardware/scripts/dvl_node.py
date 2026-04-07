#!/usr/bin/env python3
"""
================================================================================
ASUQTR Sub's ROS 2 DVL Node
================================================================================
Interfaces with the Water Linked DVL-A50 via serial, parses the NMEA-style
acoustic protocol, translates NED velocities to ROS-standard FLU, and publishes
odometry/altitude data.
"""
from __future__ import annotations
import serial
import threading
import crcmod
from enum import Enum
from collections.abc import Callable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped


# ==========================================
# CONFIGURATION & CONSTANTS
# ==========================================
DEFAULT_DVL_PORT_NAME = '/dev/ttyTHS0'
DVL_A50_BAUDRATE = 115200

# Expected minimum protocol and firmware versions
MIN_PROTOCOL_VERSION = (2, 6, 0)
EXPECTED_FIRMWARE_VERSION = (2, 6, 4)


class DVLProtocolHeader(str, Enum):
    """Protocol header prefixes sent by the DVL."""
    VELOCITY_REPORT = 'wrz'
    PROTOCOL_VERSION = 'wrv'
    PRODUCT_DETAILS = 'wrw'
    BAD_REQUEST = 'wr?'
    BAD_CHECKSUM = 'wr!'


class DVLNode(Node):
    """
    Manages the serial interface to the Water Linked DVL-A50, handling RX/TX,
    checksum validation, and ROS 2 data publishing.
    """

    def __init__(self):
        super().__init__('dvl_node')
        
        # ROS 2 Parameters
        port_name = self.declare_parameter('serial_port_name', DEFAULT_DVL_PORT_NAME).value
        
        # Hardware setup
        self.crc: Callable[[bytes], int] = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        self.serial_port = serial.Serial(port_name, DVL_A50_BAUDRATE, timeout=1.0, exclusive=True)
        
        # Publishers
        self.velocities_pub = self.create_publisher(TwistWithCovarianceStamped, 'dvl/velocities', 10)
        self.altitude_pub = self.create_publisher(PoseStamped, 'dvl/altitude', 10)
        
        # State variables
        self.invalid_velocity_count = 0

        # Initialization Routine
        self._verify_hardware_versions()

        # Start background reading
        self.rx_thread = threading.Thread(target=self._serial_rx_loop, daemon=True)
        self.rx_thread.start()
        self.get_logger().info(f"DVL Node successfully initialized {port_name} and reading serial data.")

    def _verify_hardware_versions(self):
        """Requests and verifies product and protocol versions synchronously."""
        # Check Product Details
        self.send_command(b'wcw')
        self._handle_raw_serial_line(self.serial_port.readline())
        
        # Check Protocol Details
        self.send_command(b'wcv')
        self._handle_raw_serial_line(self.serial_port.readline())

    def _serial_rx_loop(self):
        """Continuously reads blocking serial lines in a background thread."""
        while rclpy.ok():
            line = self.serial_port.readline()
            if line:
                self._handle_raw_serial_line(line)

    def _handle_raw_serial_line(self, raw_data: bytes):
        """
        Validates CRC and decodes raw byte strings into parsed string lists.
        Safely drops malformed packets to prevent node crashes.
        """
        try:
            # 1. Structural Validation
            if b'*' not in raw_data:
                return  # Ignore completely mangled noise
            
            payload, checksum_bytes = raw_data.split(b'*', 1)
            checksum_bytes = checksum_bytes.strip()
            
            # 2. Mathematical Validation
            if self.crc(payload) != int(checksum_bytes, 16):
                self.get_logger().warn(f"CRC check failed for: {raw_data!r}")
                return

            # 3. Payload Extraction
            decoded_payload = payload.decode('utf-8')
            fields = decoded_payload.split(',')
            header = fields[0]

            # Route to appropriate handler
            if header == DVLProtocolHeader.VELOCITY_REPORT:
                self._handle_velocity_report(fields)
            elif header == DVLProtocolHeader.PROTOCOL_VERSION:
                self._handle_protocol_version(fields)
            elif header == DVLProtocolHeader.PRODUCT_DETAILS:
                self._handle_product_details(fields)
            elif header == DVLProtocolHeader.BAD_CHECKSUM:
                self.get_logger().warn('DVL rejected command: Bad Checksum')
            elif header == DVLProtocolHeader.BAD_REQUEST:
                self.get_logger().warn('DVL rejected command: Unknown Request')

        except UnicodeDecodeError:
            self.get_logger().warn("Received non-UTF-8 bytes from DVL.")
        except ValueError as e:
            self.get_logger().warn(f"Failed to parse DVL packet: {e}")

    def _handle_velocity_report(self, fields: list[str]):
        """Parses 'wrz' payload, converting NED data to FLU for publishing."""
        # Expected 'wrz' length is 12. Drop if malformed.
        if len(fields) != 12:
            return

        report_is_valid = fields[4]
        if report_is_valid != 'y':
            self.invalid_velocity_count += 1
            self.get_logger().warn(
                f'Invalid velocity report (Total: {self.invalid_velocity_count})', 
                throttle_duration_sec=1.0
            )
            return

        # Check temperature safety
        temp_alert = bool(int(fields[11][0]))
        if temp_alert:
            self.get_logger().error('DVL TEMPERATURE CRITICAL! Shutting down imminent.')

        # Extract values
        ros_time = self.get_clock().now().to_msg()
        raw_vx, raw_vy, raw_vz = float(fields[1]), float(fields[2]), float(fields[3])
        altitude = float(fields[5])
        covariances = [float(c) for c in fields[7].split(';')]

        # ---------------------------------------------------------
        # Velocity Translation: North-East-Down (NED) to Forward-Left-Up (FLU)
        # ---------------------------------------------------------
        flu_vx = raw_vx
        flu_vy = -raw_vy
        flu_vz = -raw_vz

        # Twist Publisher
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = ros_time
        twist_msg.header.frame_id = "dvl_link"
        
        twist_msg.twist.twist.linear.x = flu_vx
        twist_msg.twist.twist.linear.y = flu_vy
        twist_msg.twist.twist.linear.z = flu_vz
        
        # ---------------------------------------------------------
        # Covariance Matrix Rotation
        # ---------------------------------------------------------
        # Because we inverted Y and Z, the cross-covariances involving Y or Z 
        # MUST also have their signs inverted to maintain mathematical integrity.
        cov_array = list(twist_msg.twist.covariance)
        
        cov_array[0]  = covariances[0]   # cov(x, x)
        cov_array[1]  = -covariances[1]  # cov(x, -y) -> flips
        cov_array[2]  = -covariances[2]  # cov(x, -z) -> flips
        
        cov_array[6]  = -covariances[3]  # cov(-y, x) -> flips
        cov_array[7]  = covariances[4]   # cov(-y, -y) -> stays positive
        cov_array[8]  = covariances[5]   # cov(-y, -z) -> stays positive
        
        cov_array[12] = -covariances[6]  # cov(-z, x) -> flips
        cov_array[13] = covariances[7]   # cov(-z, -y) -> stays positive
        cov_array[14] = covariances[8]   # cov(-z, -z) -> stays positive

        twist_msg.twist.covariance = cov_array
        self.velocities_pub.publish(twist_msg)

        # Altitude Publisher
        pose_msg = PoseStamped()
        pose_msg.header.stamp = ros_time
        pose_msg.header.frame_id = "dvl_link"
        pose_msg.pose.position.z = altitude
        self.altitude_pub.publish(pose_msg)


    def _handle_protocol_version(self, fields: list[str]):
        """Validates 'wrv' payload against required minimums."""
        if len(fields) < 2:
            return

        version_str = fields[1]
        try:
            version_tuple = tuple(map(int, version_str.split('.')))
            if version_tuple >= MIN_PROTOCOL_VERSION:
                self.get_logger().info(f"DVL protocol {version_str} is compatible.")
            else:
                self.get_logger().error(f"DVL protocol {version_str} is too old! Need >= 2.6.0.")
        except ValueError:
            self.get_logger().warn(f"Could not parse protocol version string: {version_str}")

    def _handle_product_details(self, fields: list[str]):
        """Validates 'wrw' payload against expected firmware."""
        if len(fields) < 4:
            return

        firmware_version = fields[2]
        try:
            version_tuple = tuple(map(int, firmware_version.split('.')))
            if version_tuple != EXPECTED_FIRMWARE_VERSION:
                self.get_logger().warn(
                    f"Sub firmware version is {firmware_version}. This driver is optimized for 2.6.4!"
                )
            else:
                self.get_logger().info(f"DVL firmware version {firmware_version} verified.")
        except ValueError:
            self.get_logger().warn(f"Could not parse firmware string: {firmware_version}")

    def send_command(self, tx_data: bytes):
        """Transmits serial request to DVL wrapped with CRC and newline."""
        checksum_int = self.crc(tx_data)
        checksum_bytes = f"{checksum_int:02x}".encode('utf-8')
        
        full_packet = tx_data + b'*' + checksum_bytes + b'\n'
        
        self.serial_port.write(full_packet)
        self.serial_port.flush()
        self.get_logger().debug(f"Sent DVL command: {full_packet!r}")


def main(args=None):
    rclpy.init(args=args)
    node = DVLNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Shutting down Sub DVL node...')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
