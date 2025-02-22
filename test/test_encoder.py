#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# --- Configuration ---
ENCODER_A_PIN = 17  # Encoder channel A
ENCODER_B_PIN = 27  # Encoder channel B (optional: used for direction)
COUNTS_PER_REV = 360  # Set this to your encoder's counts per revolution

# --- Global Variables ---
pulse_count = 0

# --- Callback for Encoder ---
def encoder_callback(channel):
    global pulse_count
    # For simple counting (ignores direction); for full quadrature decoding, check both channels.
    pulse_count += 1

# --- Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach an interrupt to channel A
GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=encoder_callback)

print("Encoder test: Counting pulses for 5 seconds...")
start_time = time.time()
initial_count = pulse_count

# Count pulses for a set period (e.g., 5 seconds)
test_duration = 5  # seconds
time.sleep(test_duration)

end_time = time.time()
final_count = pulse_count
delta_count = final_count - initial_count
elapsed_time = end_time - start_time

# Calculate revolutions per minute (RPM)
# RPM = (counts / counts_per_rev) * (60 / elapsed_time)
rpm = (delta_count / COUNTS_PER_REV) * (60 / elapsed_time)

print(f"Elapsed time: {elapsed_time:.2f} sec")
print(f"Pulse count: {delta_count}")
print(f"Calculated RPM: {rpm:.2f}")

# Clean up GPIO before exiting
GPIO.cleanup()
