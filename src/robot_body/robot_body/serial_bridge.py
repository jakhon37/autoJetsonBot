# autoJetsonBot/src/robot_body/robot_body/serial_bridge.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.297)   # meters
        self.declare_parameter('wheel_radius', 0.033)       # meters
        self.declare_parameter('max_wheel_linear', 0.5)     # m/s at speed=1.0

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # Start reader thread
        threading.Thread(target=self.read_serial_loop, daemon=True).start()

    def cmd_vel_cb(self, msg: Twist):
        # Differential drive inverse kinematics
        sep = self.get_parameter('wheel_separation').value
        r   = self.get_parameter('wheel_radius').value
        vmax = self.get_parameter('max_wheel_linear').value

        v = msg.linear.x
        w = msg.angular.z
        # compute wheel linear speeds (m/s)
        v_l = v - (w * sep / 2.0)
        v_r = v + (w * sep / 2.0)
        # normalize to [-1,1]
        l_norm = max(min(v_l / vmax, 1.0), -1.0)
        r_norm = max(min(v_r / vmax, 1.0), -1.0)

        cmd = f"LEFT:{l_norm:.3f},RIGHT:{r_norm:.3f}\n"
        self.ser.write(cmd.encode('ascii'))

    def read_serial_loop(self):
        while rclpy.ok():
            line = self.ser.readline().decode('ascii').strip()
            if line.startswith('RPM_LEFT'):
                try:
                    parts = line.split(',')
                    l = float(parts[0].split(':')[1])
                    r = float(parts[1].split(':')[1])
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = ['left_wheel_joint','right_wheel_joint']
                    # convert RPM to rad/s
                    js.velocity = [l * 2.0 * 3.14159 / 60.0,
                                   r * 2.0 * 3.14159 / 60.0]
                    self.joint_pub.publish(js)
                except Exception as e:
                    self.get_logger().warn(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
