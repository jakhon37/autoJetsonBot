#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

# Motor driver control pins; update these based on your wiring.
PWM_PIN = 18       # PWM signal for speed
DIR_PIN = 23       # Direction control

class MotorDriver(Node):
    def __init__(self): 
        super().__init__('motor_driver')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN, GPIO.OUT)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(PWM_PIN, 1000)  # 1 kHz PWM frequency
        self.pwm.start(0)
        
        self.subscription = self.create_subscription(
            Float32,
            'cmd_vel_motor',  # a custom topic for motor commands
            self.cmd_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_callback(self, msg):
        # Here msg.data could be a value from -1.0 to 1.0
        speed = max(min(msg.data, 1.0), -1.0)
        # duty_cycle = abs(speed) * 100  # scale to 0-100%
        duty_cycle = (1 - abs(speed)) * 100
        # if speed >= 0:
        #     GPIO.output(DIR_PIN, GPIO.LOW)
        # else:
        #     GPIO.output(DIR_PIN, GPIO.HIGH)
            
        GPIO.output(DIR_PIN, GPIO.HIGH if speed >= 0 else GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f'Setting motor speed: {speed:.2f} (duty cycle: {duty_cycle:.1f}%)')

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
