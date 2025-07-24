/*
 * arduino_firmware.ino - diffdrive_arduino compatible firmware
 * ROS2 Foxy compatible implementation for Jetson Nano deployment
 * Supports TB6612FNG motor driver with quadrature encoders
 * Author: RovoDev Assistant
 */

#include <Arduino.h>

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Motor Driver Pins (TB6612FNG)
#define MOTOR_A_PWM 32  // Left motor PWM
#define MOTOR_A_IN1 16  // Left motor direction 1
#define MOTOR_A_IN2 18  // Left motor direction 2
#define MOTOR_B_PWM 33  // Right motor PWM  
#define MOTOR_B_IN1 22  // Right motor direction 1
#define MOTOR_B_IN2 24  // Right motor direction 2

// Encoder Pins
#define ENCODER_A_A 17  // Left encoder channel A
#define ENCODER_A_B 27  // Left encoder channel B
#define ENCODER_B_A 5   // Right encoder channel A
#define ENCODER_B_B 6   // Right encoder channel B

// System Configuration
#define BAUD_RATE 115200
#define CONTROL_FREQUENCY 60  // Hz
#define ENCODER_CPR 3436      // Counts per revolution (standardized)
#define WHEEL_RADIUS 0.035    // meters
#define WHEEL_SEPARATION 0.18 // meters

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Encoder counts (volatile for interrupt safety)
volatile long encoder_a_count = 0;
volatile long encoder_b_count = 0;
volatile long encoder_a_prev = 0;
volatile long encoder_b_prev = 0;

// Motor control variables
float target_vel_left = 0.0;   // rad/s
float target_vel_right = 0.0;  // rad/s
float current_vel_left = 0.0;
float current_vel_right = 0.0;

// PID control variables
float kp = 1.0, ki = 0.1, kd = 0.01;
float error_left_prev = 0.0, error_right_prev = 0.0;
float integral_left = 0.0, integral_right = 0.0;

// Timing variables
unsigned long last_control_time = 0;
unsigned long last_encoder_time = 0;
unsigned long last_serial_time = 0;
unsigned long cmd_timeout = 500; // ms

// Communication variables
String input_string = "";
bool string_complete = false;

// =============================================================================
// INTERRUPT SERVICE ROUTINES
// =============================================================================

void IRAM_ATTR encoder_a_isr() {
  // Read both channels for quadrature decoding
  bool a_state = digitalRead(ENCODER_A_A);
  bool b_state = digitalRead(ENCODER_A_B);
  
  // Quadrature decoding logic
  if (a_state != b_state) {
    encoder_a_count++;  // Forward
  } else {
    encoder_a_count--;  // Backward
  }
}

void IRAM_ATTR encoder_b_isr() {
  // Read both channels for quadrature decoding
  bool a_state = digitalRead(ENCODER_B_A);
  bool b_state = digitalRead(ENCODER_B_B);
  
  // Quadrature decoding logic
  if (a_state != b_state) {
    encoder_b_count++;  // Forward
  } else {
    encoder_b_count--;  // Backward
  }
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);
  
  // Initialize motor pins
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  // Initialize encoder pins
  pinMode(ENCODER_A_A, INPUT_PULLUP);
  pinMode(ENCODER_A_B, INPUT_PULLUP);
  pinMode(ENCODER_B_A, INPUT_PULLUP);
  pinMode(ENCODER_B_B, INPUT_PULLUP);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_A), encoder_a_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_A), encoder_b_isr, CHANGE);
  
  // Stop motors initially
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  
  // Initialize timing
  last_control_time = millis();
  last_encoder_time = millis();
  last_serial_time = millis();
  
  // Send ready signal
  Serial.println("Arduino ready for diffdrive_arduino communication");
  
  delay(100);
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  unsigned long current_time = millis();
  
  // Handle serial communication
  handleSerial();
  
  // Control loop at specified frequency
  if (current_time - last_control_time >= (1000 / CONTROL_FREQUENCY)) {
    updateVelocities();
    controlMotors();
    last_control_time = current_time;
  }
  
  // Send encoder data at 50Hz
  if (current_time - last_encoder_time >= 20) {
    sendEncoderData();
    last_encoder_time = current_time;
  }
  
  // Safety timeout check
  if (current_time - last_serial_time > cmd_timeout) {
    target_vel_left = 0.0;
    target_vel_right = 0.0;
  }
  
  delay(1); // Small delay for stability
}

// =============================================================================
// COMMUNICATION FUNCTIONS
// =============================================================================

void handleSerial() {
  while (Serial.available()) {
    char incoming_char = (char)Serial.read();
    
    if (incoming_char == '\n') {
      string_complete = true;
    } else {
      input_string += incoming_char;
    }
  }
  
  if (string_complete) {
    parseCommand(input_string);
    input_string = "";
    string_complete = false;
    last_serial_time = millis();
  }
}

void parseCommand(String command) {
  // Expected format: "L<left_vel>R<right_vel>"
  // Example: "L0.5R-0.3" for left=0.5 rad/s, right=-0.3 rad/s
  
  int l_index = command.indexOf('L');
  int r_index = command.indexOf('R');
  
  if (l_index >= 0 && r_index > l_index) {
    String left_str = command.substring(l_index + 1, r_index);
    String right_str = command.substring(r_index + 1);
    
    target_vel_left = left_str.toFloat();
    target_vel_right = right_str.toFloat();
    
    // Clamp velocities to safe limits
    target_vel_left = constrain(target_vel_left, -10.0, 10.0);
    target_vel_right = constrain(target_vel_right, -10.0, 10.0);
  }
}

void sendEncoderData() {
  // Calculate velocities
  updateVelocities();
  
  // Send in diffdrive_arduino expected format
  Serial.print(encoder_a_count);
  Serial.print(" ");
  Serial.print(encoder_b_count);
  Serial.print(" ");
  Serial.print(current_vel_left, 4);
  Serial.print(" ");
  Serial.println(current_vel_right, 4);
}

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================

void setMotorSpeed(int motor, int speed) {
  // motor: 0=left, 1=right
  // speed: -255 to 255
  
  speed = constrain(speed, -255, 255);
  
  if (motor == 0) { // Left motor
    if (speed >= 0) {
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
    } else {
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      speed = -speed;
    }
    analogWrite(MOTOR_A_PWM, speed);
    
  } else { // Right motor
    if (speed >= 0) {
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
    } else {
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
      speed = -speed;
    }
    analogWrite(MOTOR_B_PWM, speed);
  }
}

void controlMotors() {
  // PID control for both motors
  float dt = 1.0 / CONTROL_FREQUENCY;
  
  // Left motor PID
  float error_left = target_vel_left - current_vel_left;
  integral_left += error_left * dt;
  integral_left = constrain(integral_left, -10.0, 10.0); // Anti-windup
  float derivative_left = (error_left - error_left_prev) / dt;
  float output_left = kp * error_left + ki * integral_left + kd * derivative_left;
  error_left_prev = error_left;
  
  // Right motor PID
  float error_right = target_vel_right - current_vel_right;
  integral_right += error_right * dt;
  integral_right = constrain(integral_right, -10.0, 10.0); // Anti-windup
  float derivative_right = (error_right - error_right_prev) / dt;
  float output_right = kp * error_right + ki * integral_right + kd * derivative_right;
  error_right_prev = error_right;
  
  // Convert to PWM values (simple scaling)
  int pwm_left = (int)(output_left * 25.5); // Scale to PWM range
  int pwm_right = (int)(output_right * 25.5);
  
  // Apply to motors
  setMotorSpeed(0, pwm_left);
  setMotorSpeed(1, pwm_right);
}

// =============================================================================
// VELOCITY CALCULATION
// =============================================================================

void updateVelocities() {
  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // Convert to seconds
  
  if (dt > 0) {
    // Calculate encoder deltas
    long delta_a = encoder_a_count - encoder_a_prev;
    long delta_b = encoder_b_count - encoder_b_prev;
    
    // Convert to angular velocities (rad/s)
    current_vel_left = (delta_a * 2.0 * PI) / (ENCODER_CPR * dt);
    current_vel_right = (delta_b * 2.0 * PI) / (ENCODER_CPR * dt);
    
    // Update previous values
    encoder_a_prev = encoder_a_count;
    encoder_b_prev = encoder_b_count;
  }
  
  last_time = current_time;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void resetEncoders() {
  noInterrupts();
  encoder_a_count = 0;
  encoder_b_count = 0;
  encoder_a_prev = 0;
  encoder_b_prev = 0;
  interrupts();
}

void emergencyStop() {
  target_vel_left = 0.0;
  target_vel_right = 0.0;
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  integral_left = 0.0;
  integral_right = 0.0;
}

// =============================================================================
// DIAGNOSTIC FUNCTIONS
// =============================================================================

void printDiagnostics() {
  Serial.print("Encoders: L=");
  Serial.print(encoder_a_count);
  Serial.print(" R=");
  Serial.print(encoder_b_count);
  Serial.print(" | Velocities: L=");
  Serial.print(current_vel_left, 3);
  Serial.print(" R=");
  Serial.print(current_vel_right, 3);
  Serial.print(" | Targets: L=");
  Serial.print(target_vel_left, 3);
  Serial.print(" R=");
  Serial.println(target_vel_right, 3);
}