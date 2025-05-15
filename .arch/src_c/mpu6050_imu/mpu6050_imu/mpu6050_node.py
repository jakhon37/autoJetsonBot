#!/usr/bin/env python3
"""
mpu6050_node.py
A ROS 2 node to read data from the MPU6050 IMU and publish sensor_msgs/Imu messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

from smbus2 import SMBus

# MPU6050 Registers and Addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

def read_word(bus, reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.declare_parameter('publish_rate', 50)
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info("MPU6050 IMU node started.")
        self.bus = SMBus(1)  # Use I2C bus 1 (common on Raspberry Pi)
        self.init_sensor()

    def init_sensor(self):
        # Wake up MPU6050
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.1)

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Read accelerometer data
        try:
            accel_x = read_word(self.bus, ACCEL_XOUT_H)
            accel_y = read_word(self.bus, ACCEL_XOUT_H + 2)
            accel_z = read_word(self.bus, ACCEL_XOUT_H + 4)
            # Scale (MPU6050 default is ±2g, 16384 LSB/g)
            imu_msg.linear_acceleration.x = accel_x / 16384.0 * 9.81
            imu_msg.linear_acceleration.y = accel_y / 16384.0 * 9.81
            imu_msg.linear_acceleration.z = accel_z / 16384.0 * 9.81

            # Read gyroscope data
            gyro_x = read_word(self.bus, GYRO_XOUT_H)
            gyro_y = read_word(self.bus, GYRO_XOUT_H + 2)
            gyro_z = read_word(self.bus, GYRO_XOUT_H + 4)
            # Scale (default ±250°/s, 131 LSB/(°/s))
            imu_msg.angular_velocity.x = math.radians(gyro_x / 131.0)
            imu_msg.angular_velocity.y = math.radians(gyro_y / 131.0)
            imu_msg.angular_velocity.z = math.radians(gyro_z / 131.0)

            # Optionally fill in orientation (requires sensor fusion); here we leave it zero.
            imu_msg.orientation_covariance[0] = -1  # Indicates no orientation estimate

        except Exception as e:
            self.get_logger().error(f"Error reading MPU6050: {e}")

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MPU6050 node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
