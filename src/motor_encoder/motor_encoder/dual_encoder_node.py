#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity, set_logger_level
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# --- Encoder Pin Configuration ---

# Left Motor Encoder Pins
ENCODER_LEFT_A_PIN = 17  # Primary pulse channel for left motor
ENCODER_LEFT_B_PIN = 27  # Optional (for direction if needed)

# Right Motor Encoder Pins
ENCODER_RIGHT_A_PIN = 5 #22  # Primary pulse channel for right motor
ENCODER_RIGHT_B_PIN = 6 #23  # Optional (for direction if needed)

class DualEncoderReader(Node):
    def __init__(self):
        super().__init__('dual_encoder_reader')
        
        set_logger_level(self.get_logger().name, LoggingSeverity.WARN)
        
        # Declare a parameter for counts per revolution (assumed the same for both encoders)
        self.declare_parameter('encoder_cpr', 360)
        self.encoder_cpr = self.get_parameter('encoder_cpr').value

        # Set up GPIO using BCM numbering
        GPIO.setmode(GPIO.BCM)
        
        # Setup left encoder pins
        GPIO.setup(ENCODER_LEFT_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_LEFT_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup right encoder pins
        GPIO.setup(ENCODER_RIGHT_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_RIGHT_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Initialize pulse counters and last timestamp variables for each encoder
        self.left_pulse_count = 0
        self.right_pulse_count = 0
        self.left_last_time = time.time()
        self.right_last_time = time.time()

        # Set up interrupt callbacks on the A channels (simple pulse counting)
        GPIO.add_event_detect(ENCODER_LEFT_A_PIN, GPIO.BOTH, callback=self.left_encoder_callback)
        GPIO.add_event_detect(ENCODER_RIGHT_A_PIN, GPIO.BOTH, callback=self.right_encoder_callback)

        # Create ROS publishers for each motor's RPM
        self.left_publisher_ = self.create_publisher(Float32, 'left_motor_rpm', 10)
        self.right_publisher_ = self.create_publisher(Float32, 'right_motor_rpm', 10)

        # Create timers to periodically publish RPM values (every 0.5 seconds)
        self.create_timer(0.5, self.publish_left_rpm)
        self.create_timer(0.5, self.publish_right_rpm)

    def left_encoder_callback(self, channel):
        # Simple pulse counting for the left encoder
        self.left_pulse_count += 1

    def right_encoder_callback(self, channel):
        # Simple pulse counting for the right encoder
        self.right_pulse_count += 1

    def publish_left_rpm(self):
        current_time = time.time()
        dt = current_time - self.left_last_time
        pulses = self.left_pulse_count
        # Reset counter and update time for left encoder
        self.left_pulse_count = 0
        self.left_last_time = current_time

        # Calculate RPM for left motor: (pulses / CPR) * (60 / dt)
        rpm = (pulses / self.encoder_cpr) * (60 / dt)
        msg = Float32()
        msg.data = rpm
        self.left_publisher_.publish(msg)
        self.get_logger().info(f'Left Motor RPM: {rpm:.2f}')

    def publish_right_rpm(self):
        current_time = time.time()
        dt = current_time - self.right_last_time
        pulses = self.right_pulse_count
        # Reset counter and update time for right encoder
        self.right_pulse_count = 0
        self.right_last_time = current_time

        # Calculate RPM for right motor
        rpm = (pulses / self.encoder_cpr) * (60 / dt)
        msg = Float32()
        msg.data = rpm
        self.right_publisher_.publish(msg)
        self.get_logger().info(f'Right Motor RPM: {rpm:.2f}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualEncoderReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
