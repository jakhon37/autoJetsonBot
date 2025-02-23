#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# Configuration: set your GPIO pins here.
ENCODER_A_PIN = 17  # Adjust these pins as needed
ENCODER_B_PIN = 27

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.declare_parameter('encoder_cpr', 360)  # counts per revolution (adjust as needed)
        self.encoder_cpr = self.get_parameter('encoder_cpr').value

        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Variables for counting pulses
        self.pulse_count = 0
        self.last_time = time.time()
        
        # Set up interrupt callback on one channel; for quadrature, you might use both channels.
        GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=self.encoder_callback)

        # ROS Publisher: publish RPM or angular velocity
        self.publisher_ = self.create_publisher(Float32, 'motor_rpm', 10)
        self.create_timer(0.5, self.publish_rpm)  # Publish every 0.5 seconds

    def encoder_callback(self, channel):
        # Read both channels to determine direction if needed.
        a = GPIO.input(ENCODER_A_PIN)
        b = GPIO.input(ENCODER_B_PIN)
        # Simple counting (for unidirectional motion) - improve for direction if needed.
        self.pulse_count += 1

    def publish_rpm(self):
        current_time = time.time()
        dt = current_time - self.last_time
        pulses = self.pulse_count
        self.pulse_count = 0  # reset counter
        self.last_time = current_time

        # Calculate revolutions per minute (RPM)
        # revs = pulses / (counts per revolution)
        rpm = (pulses / self.encoder_cpr) * (60 / dt)
        msg = Float32()
        msg.data = rpm
        self.publisher_.publish(msg)
        self.get_logger().info(f'RPM: {rpm:.2f}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
