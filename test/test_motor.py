#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# --- Configuration ---
PWM_PIN = 18   # PWM pin for motor speed
DIR_PIN = 23   # Direction control pin
FREQUENCY = 1000  # PWM frequency in Hz

# --- Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, FREQUENCY)
pwm.start(0)  # Start with 0% duty cycle (motor off)

try:
    print("Testing motor driver: ramping forward...")
    # Ramp up speed forward (direction high)
    GPIO.output(DIR_PIN, GPIO.HIGH)
    for duty in range(0, 101, 10):  # 0% to 100% in steps of 10%
        pwm.ChangeDutyCycle(duty)
        print(f"Forward: Duty cycle = {duty}%")
        time.sleep(1)
    
    print("Hold at full speed for 2 seconds...")
    time.sleep(2)

    print("Ramping down...")
    for duty in range(100, -1, -10):  # Ramp down
        pwm.ChangeDutyCycle(duty)
        print(f"Forward: Duty cycle = {duty}%")
        time.sleep(1)
    
    print("Testing motor driver: ramping backward...")
    # Ramp up speed backward (direction low)
    GPIO.output(DIR_PIN, GPIO.LOW)
    for duty in range(0, 101, 10):
        pwm.ChangeDutyCycle(duty)
        print(f"Backward: Duty cycle = {duty}%")
        time.sleep(1)
    
    print("Hold at full speed for 2 seconds...")
    time.sleep(2)

    print("Ramping down...")
    for duty in range(100, -1, -10):
        pwm.ChangeDutyCycle(duty)
        print(f"Backward: Duty cycle = {duty}%")
        time.sleep(1)

    print("Motor test complete.")

except KeyboardInterrupt:
    print("Test interrupted by user.")

finally:
    pwm.stop()
    GPIO.cleanup()
