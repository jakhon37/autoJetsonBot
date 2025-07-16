#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial
# pip3 install pyserial

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        
        self.get_logger().set_level(LoggingSeverity.INFO) #WARN or DEBUG or INFO
        # Open the serial port (update '/dev/ttyACM0' if needed)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Opened serial port: /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Publishers for motor RPM readings
        self.left_rpm_pub = self.create_publisher(Float32, 'left_motor_rpm', 10)
        self.right_rpm_pub = self.create_publisher(Float32, 'right_motor_rpm', 10)
        
        # Subscription for motor command (cmd_vel)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer to periodically check for incoming serial data
        self.create_timer(0.1, self.read_serial_data)

    def cmd_vel_callback(self, msg: Twist):
        # Prepare a command string to send to the ESP32S3
        # For example, send linear and angular speeds as comma-separated values:
        linear = msg.linear.x
        angular = msg.angular.z
        command_str = f"CMD:{linear:.2f},{angular:.2f}\n"
        try:
            self.ser.write(command_str.encode('utf-8'))
            self.get_logger().info(f"Sent command: {command_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending command over serial: {e}")


    def read_serial_data(self):
        try:
            raw_line = self.ser.readline()
            if not raw_line:
                # No data available; you can log this at a debug level if desired.
                self.get_logger().debug("No data received on this read cycle.")
                return

            # Decode with error handling to avoid UnicodeDecodeError
            line = raw_line.decode('utf-8', errors='replace').strip()
            self.get_logger().debug(f"Raw serial data: {line}")

            if line.startswith("RPM:"):
                try:
                    _, rpm_values = line.split("RPM:")
                    left_str, right_str = rpm_values.split(',')
                    left_rpm = float(left_str)
                    right_rpm = float(right_str)

                    # Publish the RPM values
                    left_msg = Float32()
                    left_msg.data = left_rpm
                    self.left_rpm_pub.publish(left_msg)

                    right_msg = Float32()
                    right_msg.data = right_rpm
                    self.right_rpm_pub.publish(right_msg)

                    self.get_logger().debug(
                        f"Received RPM - Left: {left_rpm:.2f}, Right: {right_rpm:.2f}")
                except Exception as parse_err:
                    self.get_logger().error(
                        f"Error parsing RPM message: '{line}' Error: {parse_err}")
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")
            
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down MotorBridge node.")
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
