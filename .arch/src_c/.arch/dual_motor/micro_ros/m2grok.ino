#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

// -----------------------------
// Pin Definitions (ESP32-S3)
// -----------------------------
// Motor A (Left Motor)
#define LEFT_MOTOR_IN1    3    // TB6612FNG AIN1
#define LEFT_MOTOR_IN2    10   // TB6612FNG AIN2
#define LEFT_MOTOR_PWM    11   // TB6612FNG PWMA

// Motor B (Right Motor)
#define RIGHT_MOTOR_IN1   8    // TB6612FNG BIN1
#define RIGHT_MOTOR_IN2   7    // TB6612FNG BIN2
#define RIGHT_MOTOR_PWM   6    // TB6612FNG PWMB

// Encoder Pins (assumed to be digital pins)
#define LEFT_ENCODER_A    13   // Left encoder channel A
#define LEFT_ENCODER_B    14   // Left encoder channel B (used for direction)
#define RIGHT_ENCODER_A   4    // Right encoder channel A
#define RIGHT_ENCODER_B   5    // Right encoder channel B (used for direction)

// LED for status indication
#define LED_PIN           2

// -----------------------------
// Constants
// -----------------------------
const int ENCODER_CPR = 360;    // counts per revolution (adjust as needed)
const float RPM_TIMER_PERIOD = 0.5; // seconds

// -----------------------------
// Global Variables
// -----------------------------
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;

// For RPM calculation, store last counts and time
volatile int32_t left_last_count = 0;
volatile int32_t right_last_count = 0;
uint32_t last_rpm_time = 0;

// micro-ROS objects
rcl_publisher_t left_rpm_pub;
rcl_publisher_t right_rpm_pub;
rcl_subscription_t cmd_vel_sub;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ROS messages
std_msgs__msg__Float32 left_rpm_msg;
std_msgs__msg__Float32 right_rpm_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// -----------------------------
// Motor PWM Setup
// -----------------------------
// Define PWM channels – use two different channels
const int LEFT_PWM_CHANNEL = 0;
const int RIGHT_PWM_CHANNEL = 1;
const int PWM_FREQ = 1000;      // 1 kHz PWM frequency
const int PWM_RESOLUTION = 8;   // 8-bit resolution

// -----------------------------
// Function Prototypes
// -----------------------------
void set_motor_speed(uint8_t in1, uint8_t in2, uint8_t pwm_pin, int pwm_channel, float speed);
void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void check_retcode(const char* msg, rcl_ret_t rc);

// -----------------------------
// Interrupt Service Routines
// -----------------------------
void IRAM_ATTR left_encoder_isr() {
  // Simple direction detection using encoder B pin
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    left_encoder_count--;
  } else {
    left_encoder_count++;
  }
}

void IRAM_ATTR right_encoder_isr() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    right_encoder_count--;
  } else {
    right_encoder_count++;
  }
}

// -----------------------------
// Motor Control Function
// -----------------------------
void set_motor_speed(uint8_t in1, uint8_t in2, uint8_t pwm_pin, int pwm_channel, float speed) {
  // speed expected in range [-1.0, 1.0]
  uint8_t duty = (uint8_t)(constrain(fabs(speed), 0.0, 1.0) * 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(pwm_channel, duty);
}

// -----------------------------
// cmd_vel Callback
// -----------------------------
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  // Assume linear.x and angular.z are normalized to [-1, 1]
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Differential drive mixing (similar to your Python code)
  float left_speed = constrain(linear - angular, -1.0, 1.0);
  float right_speed = constrain(linear + angular, -1.0, 1.0);

  // Set motor speeds accordingly
  set_motor_speed(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM, LEFT_PWM_CHANNEL, left_speed);
  set_motor_speed(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM, RIGHT_PWM_CHANNEL, right_speed);
}

// -----------------------------
// Timer Callback: RPM Calculation & Publishing
// -----------------------------
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) { return; }

  uint32_t current_time = millis();
  float dt = (current_time - last_rpm_time) / 1000.0; // in seconds
  if (dt <= 0) { dt = RPM_TIMER_PERIOD; } // fallback

  // Read and reset counts (atomic access assumed since ISR updates are simple)
  int32_t left_delta = left_encoder_count - left_last_count;
  int32_t right_delta = right_encoder_count - right_last_count;
  left_last_count = left_encoder_count;
  right_last_count = right_encoder_count;
  last_rpm_time = current_time;

  // Calculate RPM: (pulses / CPR) * (60 / dt)
  float left_rpm = (left_delta / (float)ENCODER_CPR) * (60.0 / dt);
  float right_rpm = (right_delta / (float)ENCODER_CPR) * (60.0 / dt);

  // Publish RPMs
  left_rpm_msg.data = left_rpm;
  right_rpm_msg.data = right_rpm;

  rcl_ret_t rc = rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to publish left RPM");
  }
  rc = rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);
  if (rc != RCL_RET_OK) {
    Serial.println("Failed to publish right RPM");
  }
  // For debugging: print the computed RPM values
  Serial.print("Left RPM: ");
  Serial.print(left_rpm, 2);
  Serial.print("  Right RPM: ");
  Serial.println(right_rpm, 2);
}

// -----------------------------
// Error Handling Helper
// -----------------------------
void check_retcode(const char* msg, rcl_ret_t rc) {
  Serial.print(msg);
  Serial.print(" returned: ");
  Serial.println(rc);
  if (rc != RCL_RET_OK) {
    Serial.print("Error details: ");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
}

// -----------------------------
// Setup Function
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting micro-ROS node...");

  // Configure status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // -----------------------------
  // Set up motor pins
  // -----------------------------
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // -----------------------------
  // Set up encoder pins
  // -----------------------------
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // -----------------------------
  // Set up PWM channels
  // -----------------------------
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_PWM, LEFT_PWM_CHANNEL);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_MOTOR_PWM, RIGHT_PWM_CHANNEL);

  // -----------------------------
  // Attach encoder interrupts
  // -----------------------------
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), right_encoder_isr, RISING);

  // Initialize encoder timing
  last_rpm_time = millis();
  left_last_count = left_encoder_count;
  right_last_count = right_encoder_count;

  // -----------------------------
  // Set up micro-ROS transport
  // -----------------------------
  set_microros_transports();
  Serial.println("Micro-ROS transports set.");

  // -----------------------------
  // Initialize micro-ROS structures
  // -----------------------------
  allocator = rcl_get_default_allocator();
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  check_retcode("rclc_support_init", rc);

  rc = rclc_node_init_default(&node, "motor_control_node", "", &support);
  check_retcode("rclc_node_init_default", rc);

  // Initialize publishers for RPM (using Float32)
  rc = rclc_publisher_init_default(
        &left_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_motor_rpm");
  check_retcode("left_rpm publisher init", rc);

  rc = rclc_publisher_init_default(
        &right_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_motor_rpm");
  check_retcode("right_rpm publisher init", rc);

  // Initialize subscription for cmd_vel (using Twist)
  rc = rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
  check_retcode("cmd_vel subscription init", rc);

  // Initialize timer for RPM calculation & publishing
  rc = rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS((uint64_t)(RPM_TIMER_PERIOD * 1000)),
        timer_callback);
  check_retcode("timer init", rc);

  // Initialize executor with capacity for 3 handles (subscription, timer, etc.)
  rc = rclc_executor_init(&executor, &support.context, 3, &allocator);
  check_retcode("executor init", rc);

  rc = rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA);
  check_retcode("executor add subscription", rc);
  rc = rclc_executor_add_timer(&executor, &timer);
  check_retcode("executor add timer", rc);

  // Announce setup complete
  Serial.println("Setup complete.");
}

// -----------------------------
// Loop Function
// -----------------------------
void loop() {
  // Let executor process incoming messages and timer events
  rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    Serial.print("Executor error: ");
    Serial.println(rc);
  }
  delay(10);
}
