#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String  # Moved to top for performance
import tf2_ros
import serial
import math
import time

class DiffDriveBridge(Node):
    def __init__(self):
        super().__init__('diffdrive_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.212)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('encoder_cpr', 3436)
        self.declare_parameter('loop_rate', 50.0)
        self.declare_parameter('publish_tf', True) # Added for EKF compatibility

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.last_time = self.get_clock().now()

        # Serial Connection
        self.ser = None
        self.connect_serial()

        # ROS Infrastructure
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.serial_imu_pub = self.create_publisher(String, '/serial_imu_raw', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for Odom & Feedback loop
        self.timer = self.create_timer(1.0/self.get_parameter('loop_rate').value, self.update_loop)
        self.get_logger().info("DiffDriveBridge Node Started at 50Hz")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01) # Low timeout for high frequency
            self.ser.reset_input_buffer()
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            self.ser = None

    def cmd_vel_callback(self, msg):
        if not self.ser: return

        # Kinematics: Linear/Angular to Wheel Velocities (m/s)
        v_l = msg.linear.x - (msg.angular.z * self.wheel_sep / 2.0)
        v_r = msg.linear.x + (msg.angular.z * self.wheel_sep / 2.0)
        
        # FIX: Kinematics inversion. If the robot spins during straight drive, 
        # it usually means one motor needs its sign flipped.
        v_r_fixed = -v_r 

        cmd = f"m {v_l:.3f} {v_r_fixed:.3f}\r"
        try:
            self.ser.write(cmd.encode())
            self.ser.flush() # CRITICAL: Ensure data is actually sent
            self.get_logger().debug(f"TX: {cmd.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def update_loop(self):
        if not self.ser:
            self.connect_serial()
            return

        try:
            # Drain the buffer to prevent lag
            while self.ser.in_waiting > 0:
                raw_line = self.ser.readline()
                if not raw_line: break
                
                try:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                except:
                    continue

                if not line: continue
                
                if line.startswith('e'):
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            l_count = int(parts[1])
                            r_count = int(parts[2])
                            
                            # FIX: Right encoder inversion to match inverted motor
                            r_count_fixed = -r_count
                            
                            self.calculate_odometry(l_count, r_count_fixed)
                        except ValueError:
                            continue
                
                elif line.startswith('i'):
                    imu_msg = String()
                    imu_msg.data = line
                    self.serial_imu_pub.publish(imu_msg)
                
                elif "READY" in line:
                    self.get_logger().info(f"📡 ESP32 Reboot Detected: {line}")

        except Exception as e:
            self.get_logger().debug(f"Loop error: {e}")

    def calculate_odometry(self, l_count, r_count):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # Convert counts to meters
        dist_l = (l_count / self.encoder_cpr) * (2 * math.pi * self.wheel_rad)
        dist_r = (r_count / self.encoder_cpr) * (2 * math.pi * self.wheel_rad)

        d_l = dist_l - self.left_pos
        d_r = dist_r - self.right_pos
        self.left_pos = dist_l
        self.right_pos = dist_r

        d_dist = (d_l + d_r) / 2.0
        d_th = (d_r - d_l) / self.wheel_sep

        self.x += d_dist * math.cos(self.th + d_th/2.0)
        self.y += d_dist * math.sin(self.th + d_th/2.0)
        self.th += d_th

        v_x = d_dist / dt
        v_th = d_th / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.th)
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_th
        self.odom_pub.publish(odom)

        # Broadcast TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [dist_l / self.wheel_rad, dist_r / self.wheel_rad]
        js.velocity = [d_l / dt / self.wheel_rad, d_r / dt / self.wheel_rad]
        self.joint_pub.publish(js)

        self.last_time = now

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
