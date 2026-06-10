#!/usr/bin/env python3
"""
IMU Bridge Node.
Subscribes to raw serial IMU strings and publishes sensor_msgs/Imu.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        
        # Publisher for standard IMU messages
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # Subscriber for raw serial lines from diffdrive_node
        self.sub_serial = self.create_subscription(
            String, 
            '/serial_imu_raw', 
            self.serial_callback, 
            10
        )
        
        self.get_logger().info("IMU Bridge Node initialized. Listening on /serial_imu_raw")

    def serial_callback(self, msg):
        # Expected format: "i ax ay az gx gy gz"
        line = msg.data
        parts = line.split()
        
        if len(parts) < 7:
            self.get_logger().warn(f"Malformed IMU packet: {line}")
            return
            
        try:
            # Parse values
            # ESP32 sends: ax, ay, az in m/s^2 and gx, gy, gz in rad/s
            ax = float(parts[1])
            ay = float(parts[2])
            az = float(parts[3])
            gx = float(parts[4])
            gy = float(parts[5])
            gz = float(parts[6])
            
            # Create Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Linear acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # Angular velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Set covariances (Phase 2b constraints)
            # Angular velocity covariance: diagonal = [0.01, 0.01, 0.01] (rad/s)²
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            # Linear acceleration covariance: diagonal = [0.1, 0.1, 0.1] (m/s²)²
            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]
            
            # Orientation: unknown (-1 in first element of covariance)
            imu_msg.orientation_covariance[0] = -1.0
            
            self.imu_pub.publish(imu_msg)
            
        except ValueError as e:
            self.get_logger().error(f"Error parsing IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
