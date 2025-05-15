#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>

// -------------------------------
// Pin definitions for ESP32-S3
// -------------------------------
#define LEFT_MOTOR_IN1     3    // TB6612FNG AIN1
#define LEFT_MOTOR_IN2     10   // TB6612FNG AIN2
#define LEFT_MOTOR_PWM     11   // TB6612FNG PWMA
#define LEFT_MOTOR_ENC_A   13   // Encoder A
#define LEFT_MOTOR_ENC_B   14   // Encoder B
#define RIGHT_MOTOR_IN1    8    // TB6612FNG BIN1
#define RIGHT_MOTOR_IN2    7    // TB6612FNG BIN2
#define RIGHT_MOTOR_PWM    6    // TB6612FNG PWMB
#define RIGHT_MOTOR_ENC_A  4    // Encoder A
#define RIGHT_MOTOR_ENC_B  5    // Encoder B
#define LED_PIN            2    // Status LED

// -------------------------------
// ROS objects
// -------------------------------
rcl_publisher_t left_encoder_pub;
rcl_publisher_t right_encoder_pub;
rcl_subscription_t left_motor_sub;
rcl_subscription_t right_motor_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// -------------------------------
// ROS messages
// -------------------------------
std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
std_msgs__msg__Int16 left_motor_msg;
std_msgs__msg__Int16 right_motor_msg;

// -------------------------------
// Motor and encoder state
// -------------------------------
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;

// -------------------------------
// Error handling
// -------------------------------
void error_loop() {
  Serial.println("Entering error loop...");
  // Blink LED forever
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Helper function to check return codes and print errors
void checkRetCode(const char *label, rcl_ret_t rc)
{
  Serial.print(label);
  Serial.print(" returned: ");
  Serial.println(rc);

  if (rc != RCL_RET_OK) {
    // Print the detailed micro-ROS error string if available
    Serial.print("Error details: ");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
    error_loop();
  }
}

// -------------------------------
// Interrupt Service Routines
// -------------------------------
void IRAM_ATTR left_encoder_isr() {
  if (digitalRead(LEFT_MOTOR_ENC_B) == HIGH) {
    left_encoder_count--;
  } else {
    left_encoder_count++;
  }
}

void IRAM_ATTR right_encoder_isr() {
  if (digitalRead(RIGHT_MOTOR_ENC_B) == HIGH) {
    right_encoder_count--;
  } else {
    right_encoder_count++;
  }
}

// -------------------------------
// Motor speed control
// -------------------------------
void set_motor_speed(int in1, int in2, int pwm_pin, int pwm_channel, int16_t speed)
{
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwm_channel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwm_channel, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwm_channel, 0);
  }
}

// -------------------------------
// Subscription callbacks
// -------------------------------
void left_motor_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  set_motor_speed(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM, 0, msg->data);
}

void right_motor_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  set_motor_speed(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM, 1, msg->data);
}

// -------------------------------
// Timer callback
// -------------------------------
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish encoder counts
    left_encoder_msg.data = left_encoder_count;
    right_encoder_msg.data = right_encoder_count;
    
    rcl_ret_t rc;
    rc = rcl_publish(&left_encoder_pub, &left_encoder_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.println("Failed to publish left_encoder_msg!");
      Serial.print("Error details: ");
      Serial.println(rcl_get_error_string().str);
      rcl_reset_error();
    }

    rc = rcl_publish(&right_encoder_pub, &right_encoder_msg, NULL);
    if (rc != RCL_RET_OK) {
      Serial.println("Failed to publish right_encoder_msg!");
      Serial.print("Error details: ");
      Serial.println(rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
}

// -------------------------------
// Arduino setup
// -------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting sketch...");

  // 1) Set up micro-ROS transport
  set_microros_transports(); 
  Serial.println("Transports set.");
  //   - Make sure this matches the micro-ROS Agent (e.g. serial, WiFi, etc.)

  // 2) Configure LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("LED initialized.");

  // 3) Configure motor pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_ENC_A, INPUT);
  pinMode(LEFT_MOTOR_ENC_B, INPUT);
  pinMode(RIGHT_MOTOR_ENC_A, INPUT);
  pinMode(RIGHT_MOTOR_ENC_B, INPUT);

  // 4) Set up PWM with the new LEDC API
  ledcAttach(LEFT_MOTOR_PWM, 1000, 8);    // auto-assign channel, freq=1000Hz, 8-bit
  ledcChangeFrequency(LEFT_MOTOR_PWM, 1000, 8);

  ledcAttach(RIGHT_MOTOR_PWM, 1000, 8);   // auto-assign channel, freq=1000Hz, 8-bit
  ledcChangeFrequency(RIGHT_MOTOR_PWM, 1000, 8);

  Serial.println("PWM initialized.");

  // 5) Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_ENC_A), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_ENC_A), right_encoder_isr, RISING);
  Serial.println("Interrupts attached.");

  delay(2000);
  Serial.println("Delay complete.");

  // -------------------------------
  // 6) micro-ROS initialization
  // -------------------------------
  allocator = rcl_get_default_allocator();

  // rclc_support_init
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  checkRetCode("rclc_support_init", rc);
  Serial.println("Support initialized.");

  // rclc_node_init_default
  rc = rclc_node_init_default(&node, "motor_control_node", "", &support);
  checkRetCode("rclc_node_init_default", rc);
  Serial.println("Node initialized.");

  // rclc_publisher_init_default
  rc = rclc_publisher_init_default(
          &left_encoder_pub,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "motor_left_encoder");
  checkRetCode("rclc_publisher_init_default (left_encoder_pub)", rc);

  rc = rclc_publisher_init_default(
          &right_encoder_pub,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "motor_right_encoder");
  checkRetCode("rclc_publisher_init_default (right_encoder_pub)", rc);

  // rclc_subscription_init_default
  rc = rclc_subscription_init_default(
          &left_motor_sub,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
          "motor_left_cmd");
  checkRetCode("rclc_subscription_init_default (left_motor_sub)", rc);

  rc = rclc_subscription_init_default(
          &right_motor_sub,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
          "motor_right_cmd");
  checkRetCode("rclc_subscription_init_default (right_motor_sub)", rc);
  Serial.println("Publishers and subscribers initialized.");

  // rclc_timer_init_default
  const unsigned int timer_timeout_ms = 50;
  rc = rclc_timer_init_default(
          &timer,
          &support,
          RCL_MS_TO_NS(timer_timeout_ms),
          timer_callback);
  checkRetCode("rclc_timer_init_default", rc);
  Serial.println("Timer initialized.");

  // rclc_executor_init
  rc = rclc_executor_init(&executor, &support.context, 3, &allocator);
  checkRetCode("rclc_executor_init", rc);

  // Add subscriptions to executor
  rc = rclc_executor_add_subscription(
          &executor,
          &left_motor_sub,
          &left_motor_msg,
          &left_motor_callback,
          ON_NEW_DATA);
  checkRetCode("rclc_executor_add_subscription (left_motor_sub)", rc);

  rc = rclc_executor_add_subscription(
          &executor,
          &right_motor_sub,
          &right_motor_msg,
          &right_motor_callback,
          ON_NEW_DATA);
  checkRetCode("rclc_executor_add_subscription (right_motor_sub)", rc);

  // Add timer to executor
  rc = rclc_executor_add_timer(&executor, &timer);
  checkRetCode("rclc_executor_add_timer", rc);
  Serial.println("Executor initialized.");

  // Initialize messages
  left_encoder_msg.data  = 0;
  right_encoder_msg.data = 0;
  left_motor_msg.data    = 0;
  right_motor_msg.data   = 0;

  Serial.println("Setup complete.");
}

// -------------------------------
// Arduino loop
// -------------------------------
void loop() {
  delay(10);

  // Spin executor, check for errors
  rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    Serial.println("Executor spin error!");
    Serial.print("Error details: ");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
  }

  Serial.println("Loop running...");
}
