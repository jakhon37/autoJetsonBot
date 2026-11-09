#!/usr/bin/env python3
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import tf2_ros

from jetson_bot_diffdrive.odometry import (
    POSE_COVARIANCE_DIAGONAL, TWIST_COVARIANCE_DIAGONAL, WheelOdometry, covariance,
    encoder_dt, parse_encoder_line)

class DiffDriveBridge(Node):
    def __init__(self):
        super().__init__('diffdrive_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.212)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('encoder_cpr', 1497)
        self.declare_parameter('loop_rate', 50.0)
        # Rate of the ESP32's 'e' lines. A slower firmware is fine, a faster one isn't:
        # no line counts as covering less than one period.
        self.declare_parameter('encoder_rate', 50.0)
        self.declare_parameter('publish_tf', True) # Added for EKF compatibility
        # Same names as diff_drive_controller's, which publishes /odom in simulation
        self.declare_parameter('pose_covariance_diagonal', list(POSE_COVARIANCE_DIAGONAL))
        self.declare_parameter('twist_covariance_diagonal', list(TWIST_COVARIANCE_DIAGONAL))

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.pose_covariance = covariance(self.get_parameter('pose_covariance_diagonal').value)
        self.twist_covariance = covariance(self.get_parameter('twist_covariance_diagonal').value)
        
        # State
        self.odometry = WheelOdometry(self.wheel_sep, self.wheel_rad, self.encoder_cpr)
        self.min_enc_dt = 1.0 / self.get_parameter('encoder_rate').value
        self.enc_dt = self.min_enc_dt
        self.last_time = self.get_clock().now()

        # Serial Connection
        self.ser = None
        self._rx_buf = b''
        self._rx_log_left = 8
        self.last_tx_time = 0.0
        self.last_rx_time = 0.0
        self.last_enc_time = 0.0
        self.last_imu_time = 0.0
        self.last_rx_line = ''
        self.last_enc = None
        self.last_imu = None
        self.tx_ok = 0
        self.enc_ok = 0
        self.imu_ok = 0
        self.connect_serial()

        # ROS Infrastructure
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.serial_imu_pub = self.create_publisher(String, '/serial_imu_raw', 10)
        self.status_pub = self.create_publisher(String, '/esp32/status', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for Odom & Feedback loop
        self.timer = self.create_timer(1.0/self.get_parameter('loop_rate').value, self.update_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("DiffDriveBridge Node Started at 50Hz")

    def _close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._rx_buf = b''

    def connect_serial(self):
        port = self.port
        if not os.path.exists(port):
            alt = self._discover_port()
            if alt:
                port = alt
                self.port = alt
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.01)
            try:
                self.ser.dtr = True
                self.ser.rts = False
            except Exception:
                pass
            self.ser.reset_input_buffer()
            self._rx_buf = b''
            self.odometry.reset_baseline()
            self.get_logger().info(f"✅ Connected to ESP32 on {port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error ({port}): {e}")
            self._close_serial()

    def _discover_port(self):
        by_id = '/dev/serial/by-id'
        skip = ('cp210', 'silicon_labs', 'rplidar')
        prefer = ('espressif', 'esp32', 'jtag_serial', '1a86', 'qinheng',
                  'usb_single_serial', 'wch', 'arduino')
        try:
            names = sorted(os.listdir(by_id)) if os.path.isdir(by_id) else []
        except Exception:
            names = []
        for name in names:
            low = name.lower()
            if any(s in low for s in skip):
                continue
            path = os.path.realpath(os.path.join(by_id, name))
            if any(s in low for s in prefer) and os.path.exists(path):
                return path
        for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'):
            if os.path.exists(p):
                return p
        return None

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
            self.last_tx_time = time.time()
            self.tx_ok += 1
            self.get_logger().debug(f"TX: {cmd.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            self._close_serial()

    def _handle_line(self, line):
        self.last_rx_time = time.time()
        self.last_rx_line = line[:80]
        if self._rx_log_left > 0:
            self._rx_log_left -= 1
            self.get_logger().info(f"ESP32 RX: {line}")

        low = line.lower()
        counts = parse_encoder_line(line)
        if counts:
            l_count, r_count = counts
            # FIX: Right encoder inversion to match inverted motor
            self.calculate_odometry(l_count, -r_count)
            self.last_enc = [l_count, r_count]
            self.last_enc_time = time.time()
            self.enc_ok += 1
        elif low.startswith('i '):
            imu_msg = String()
            imu_msg.data = line
            self.serial_imu_pub.publish(imu_msg)
            try:
                self.last_imu = [round(float(p), 3) for p in line.split()[1:7]]
            except (ValueError, IndexError):
                self.last_imu = line
            self.last_imu_time = time.time()
            self.imu_ok += 1
        elif "READY" in line:
            self.get_logger().info(f"📡 ESP32 Reboot Detected: {line}")
            self.odometry.reset_baseline()

    def update_loop(self):
        if not self.ser:
            self.connect_serial()
            return

        try:
            n = self.ser.in_waiting
            if n:
                self._rx_buf += self.ser.read(n)
            lines = []
            while True:
                sep_n = self._rx_buf.find(b'\n')
                sep_r = self._rx_buf.find(b'\r')
                seps = [s for s in (sep_n, sep_r) if s >= 0]
                if not seps:
                    if len(self._rx_buf) > 256:
                        line = self._rx_buf.decode('utf-8', errors='ignore').strip()
                        self._rx_buf = b''
                        if line:
                            lines.append(line)
                    break
                sep = min(seps)
                raw = self._rx_buf[:sep]
                self._rx_buf = self._rx_buf[sep + 1:]
                if self._rx_buf[:1] in (b'\n', b'\r'):
                    self._rx_buf = self._rx_buf[1:]
                line = raw.decode('utf-8', errors='ignore').strip()
                if line:
                    lines.append(line)

            # Timing each 'e' line by when it's handled gives a near-zero dt to the second
            # of two lines read together, and v and w spike. See encoder_dt.
            n_enc = sum(1 for line in lines if parse_encoder_line(line))
            if n_enc:
                now = self.get_clock().now()
                elapsed = (now - self.last_time).nanoseconds / 1e9
                self.enc_dt = encoder_dt(elapsed, n_enc, self.min_enc_dt)
                self.last_time = now
            for line in lines:
                self._handle_line(line)
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            self._close_serial()

    def publish_status(self):
        now = time.time()
        def age(ts):
            return -1.0 if ts <= 0.0 else round(now - ts, 3)
        msg = String()
        payload = {
            'connected': self.ser is not None and getattr(self.ser, 'is_open', False),
            'port': self.port,
            'tx_ok': self.tx_ok,
            'enc_ok': self.enc_ok,
            'imu_ok': self.imu_ok,
            'tx_age_s': age(self.last_tx_time),
            'rx_age_s': age(self.last_rx_time),
            'enc_age_s': age(self.last_enc_time),
            'imu_age_s': age(self.last_imu_time),
            'last_rx': self.last_rx_line,
            'last_enc': self.last_enc,
            'last_imu': self.last_imu,
        }
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        try:
            with open('/tmp/esp32_status.json', 'w') as f:
                json.dump(payload, f)
        except Exception:
            pass

    def calculate_odometry(self, l_count, r_count):
        now = self.get_clock().now()
        v_x, v_th, wheel_angles, wheel_rates = self.odometry.update(
            l_count, r_count, self.enc_dt)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odometry.x
        odom.pose.pose.position.y = self.odometry.y
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.odometry.th)
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_th
        odom.twist.covariance = self.twist_covariance
        self.odom_pub.publish(odom)

        # Broadcast TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.odometry.x
            t.transform.translation.y = self.odometry.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = wheel_angles
        js.velocity = wheel_rates
        self.joint_pub.publish(js)

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
