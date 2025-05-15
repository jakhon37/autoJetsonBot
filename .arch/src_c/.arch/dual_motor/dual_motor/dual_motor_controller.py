#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# ----------------------------
# Define TB6612FNG Pin Mapping
# ----------------------------

# Motor A (Left Motor) Pins
PWM_A_PIN = 32 #18    # PWMA: PWM signal for speed control
AIN1_PIN  = 16 #23    # AIN1: Direction control 1
AIN2_PIN  = 18 #24    # AIN2: Directi



# Motor B (Right Motor) Pins
PWM_B_PIN = 33 #13    # PWMB: PWM signal for speed control
BIN1_PIN  = 22 #19    # BIN1: Direction control 1
BIN2_PIN  = 24 #26    # BIN2: Direction control 2

class DualMotorDriver(Node):
    def __init__(self):
        super().__init__('dual_motor_driver')
        GPIO.setmode(GPIO.BCM)

        # Setup pins for Motor A (Left)
        GPIO.setup(PWM_A_PIN, GPIO.OUT)
        GPIO.setup(AIN1_PIN, GPIO.OUT)
        GPIO.setup(AIN2_PIN, GPIO.OUT)

        # Setup pins for Motor B (Right)
        GPIO.setup(PWM_B_PIN, GPIO.OUT)
        GPIO.setup(BIN1_PIN, GPIO.OUT)
        GPIO.setup(BIN2_PIN, GPIO.OUT)

        # Initialize PWM on both channels at 1 kHz
        self.pwm_a = GPIO.PWM(PWM_A_PIN, 1000)
        self.pwm_a.start(0)
        self.pwm_b = GPIO.PWM(PWM_B_PIN, 1000)
        self.pwm_b.start(0)

        # Subscribe to cmd_vel (Twist message) for differential drive control
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',   # Expecting geometry_msgs/Twist
            self.cmd_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def cmd_callback(self, msg):
        # Extract forward speed (linear.x) and rotation (angular.z)
        linear_speed  = msg.linear.x   # Should be normalized to [-1, 1]
        angular_speed = msg.angular.z  # Should be normalized to [-1, 1]

        # Compute motor speeds using simple differential drive mixing.
        # Adjust the mixing if your robot requires different scaling.
        left_speed  = max(min(linear_speed - angular_speed, 1.0), -1.0)
        right_speed = max(min(linear_speed + angular_speed, 1.0), -1.0)

        # Update Left Motor (Motor A)
        left_duty = abs(left_speed) * 100  # Active-high PWM: 0% = off, 100% = full speed
        if left_speed > 0:
            # Forward: AIN1 HIGH, AIN2 LOW
            GPIO.output(AIN1_PIN, GPIO.HIGH)
            GPIO.output(AIN2_PIN, GPIO.LOW)
        elif left_speed < 0:
            # Reverse: AIN1 LOW, AIN2 HIGH
            GPIO.output(AIN1_PIN, GPIO.LOW)
            GPIO.output(AIN2_PIN, GPIO.HIGH)
        else:
            # Stop motor: both low (or implement braking if desired)
            GPIO.output(AIN1_PIN, GPIO.LOW)
            GPIO.output(AIN2_PIN, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(left_duty)

        # Update Right Motor (Motor B)
        right_duty = abs(right_speed) * 100
        if right_speed > 0:
            # Forward: BIN1 HIGH, BIN2 LOW
            GPIO.output(BIN1_PIN, GPIO.HIGH)
            GPIO.output(BIN2_PIN, GPIO.LOW)
        elif right_speed < 0:
            # Reverse: BIN1 LOW, BIN2 HIGH
            GPIO.output(BIN1_PIN, GPIO.LOW)
            GPIO.output(BIN2_PIN, GPIO.HIGH)
        else:
            GPIO.output(BIN1_PIN, GPIO.LOW)
            GPIO.output(BIN2_PIN, GPIO.LOW)
        self.pwm_b.ChangeDutyCycle(right_duty)

        # Log the current speeds and duty cycles
        self.get_logger().info(
            f"Left Motor: {left_speed:.2f} (duty: {left_duty:.1f}%), "
            f"Right Motor: {right_speed:.2f} (duty: {right_duty:.1f}%)"
        )

    def destroy_node(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
