
#!/usr/bin/env python3
"""
motor_controller.py
A ROS 2 node to control the drive and steering motors via L9110.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Import RPi.GPIO – if not installed, run: sudo apt install python3-rpi.gpio
import RPi.GPIO as GPIO
import time

# --- GPIO PIN CONFIGURATION (adjust as needed) ---
# Back (drive) motor pins
BACK_MOTOR_PIN1 = 5 #13 #17
BACK_MOTOR_PIN2 = 6 #26 #18

# Front (steering) motor pins
FRONT_MOTOR_PIN1 = 13 #5 #27
FRONT_MOTOR_PIN2 = 26 #6 #22

# PWM Frequency in Hz (adjust based on motor specs)
PWM_FREQUENCY = 100

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.declare_parameter('max_speed', 100)  # maximum PWM duty cycle
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().integer_value

        # Setup GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # Setup pins as outputs
        GPIO.setup(BACK_MOTOR_PIN1, GPIO.OUT)
        GPIO.setup(BACK_MOTOR_PIN2, GPIO.OUT)
        GPIO.setup(FRONT_MOTOR_PIN1, GPIO.OUT)
        GPIO.setup(FRONT_MOTOR_PIN2, GPIO.OUT)

        # Initialize PWM for speed control
        self.back_pwm1 = GPIO.PWM(BACK_MOTOR_PIN1, PWM_FREQUENCY)
        self.back_pwm2 = GPIO.PWM(BACK_MOTOR_PIN2, PWM_FREQUENCY)
        self.front_pwm1 = GPIO.PWM(FRONT_MOTOR_PIN1, PWM_FREQUENCY)
        self.front_pwm2 = GPIO.PWM(FRONT_MOTOR_PIN2, PWM_FREQUENCY)

        # Start PWM with zero duty cycle (stopped)
        self.back_pwm1.start(0)
        self.back_pwm2.start(0)
        self.front_pwm1.start(0)
        self.front_pwm2.start(0)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("Motor Controller Node has been started.")

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist messages into motor PWM commands.
          - msg.linear.x: forward/back speed (range: -1.0 to 1.0)
          - msg.angular.z: steering (range: -1.0 to 1.0)
        """
        # Scale speed to PWM duty cycle
        speed = max(min(msg.linear.x, 1.0), -1.0)  # clamp between -1 and 1
        steer = max(min(msg.angular.z, 1.0), -1.0)   # clamp between -1 and 1

        duty_cycle_speed = abs(speed) * self.max_speed
        duty_cycle_steer = abs(steer) * self.max_speed

        # --- Back Motor Control (Drive) ---
        if speed > 0:
            # Forward: pin1 = duty cycle, pin2 = 0
            self.back_pwm1.ChangeDutyCycle(duty_cycle_speed)
            self.back_pwm2.ChangeDutyCycle(0)
        elif speed < 0:
            # Backward: pin1 = 0, pin2 = duty cycle
            self.back_pwm1.ChangeDutyCycle(0)
            self.back_pwm2.ChangeDutyCycle(duty_cycle_speed)
        else:
            # Stop motor
            self.back_pwm1.ChangeDutyCycle(0)
            self.back_pwm2.ChangeDutyCycle(0)

        # --- Front Motor Control (Steering) ---
        if steer > 0:
            # Turn right: adjust motors accordingly
            self.front_pwm1.ChangeDutyCycle(duty_cycle_steer)
            self.front_pwm2.ChangeDutyCycle(0)
        elif steer < 0:
            # Turn left:
            self.front_pwm1.ChangeDutyCycle(0)
            self.front_pwm2.ChangeDutyCycle(duty_cycle_steer)
        else:
            self.front_pwm1.ChangeDutyCycle(0)
            self.front_pwm2.ChangeDutyCycle(0)

        self.get_logger().debug(
            f"Received cmd_vel: speed={speed}, steer={steer}. "
            f"PWM: drive={duty_cycle_speed}, steer={duty_cycle_steer}")

    def destroy_node(self):
        # Cleanup GPIO on shutdown
        self.back_pwm1.stop()
        self.back_pwm2.stop()
        self.front_pwm1.stop()
        self.front_pwm2.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Motor Controller node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
