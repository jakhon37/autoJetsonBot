#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# --- Configuration ---
# Motor settings
PWM_PIN = 18
DIR_PIN = 23
FREQUENCY = 1000  # Hz

# Encoder settings
ENCODER_A_PIN = 17
ENCODER_B_PIN = 27
COUNTS_PER_REV = 360

# --- Global Variables ---
pulse_count = 0

# --- Encoder Callback ---
def encoder_callback(channel):
    global pulse_count
    pulse_count += 1

# --- Setup ---
GPIO.setmode(GPIO.BCM)

# Setup motor driver pins
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
pwm = GPIO.PWM(PWM_PIN, FREQUENCY)
pwm.start(0)

# Setup encoder pins
GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=encoder_callback)

try:
    print("Combined test: Running motor at constant speed while reading encoder pulses.")
    # Set motor direction and speed
    GPIO.output(DIR_PIN, GPIO.HIGH)  # Forward direction
    duty_cycle = 50  # 50% duty cycle for moderate speed
    pwm.ChangeDutyCycle(duty_cycle)
    
    # Monitor encoder readings for 10 seconds
    start_time = time.time()
    initial_count = pulse_count
    while time.time() - start_time < 50:
        time.sleep(1)
        current_count = pulse_count - initial_count
        elapsed = time.time() - start_time
        rpm = (current_count / COUNTS_PER_REV) * (60 / elapsed)
        print(f"Time: {elapsed:.1f}s | Pulse count: {current_count} | Estimated RPM: {rpm:.2f}")

    print("Stopping motor.")
    pwm.ChangeDutyCycle(0)
    
except KeyboardInterrupt:
    print("Test interrupted.")

finally:
    pwm.stop()
    GPIO.cleanup()
