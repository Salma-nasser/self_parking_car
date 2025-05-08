#include "motion_control.h"

#include <Arduino.h>

#include "motor_control.h"
#include "pid_controller.h"

// PID controllers for left and right wheels sides
static PIDController pid_left;
static PIDController pid_right;

// Current motion control state
static MotionMode current_mode = MOTION_STOP;
static float target_value = 0.0f;

// Base speed settings
#define BASE_SPEED 50      // Base speed percentage (0-100)
#define TURNING_SPEED 30   // Speed for turning (0-100)
#define MAX_CORRECTION 30  // Maximum correction percentage

void motion_control_init(void) {
  // Initialize motor control
  motor_init();

  // Initialize PID controllers with tuning values
  float kp = 2.0f;  // Proportional gain
  float ki = 0.1f;  // Integral gain
  float kd = 0.5f;  // Derivative gain

  // Initialize left wheel PID
  pid_init(&pid_left, kp, ki, kd, -MAX_CORRECTION, MAX_CORRECTION, 100.0f);

  // Initialize right wheel PID
  pid_init(&pid_right, kp, ki, kd, -MAX_CORRECTION, MAX_CORRECTION, 100.0f);

  Serial.println("[MOTION_CONTROL] Motion control initialized");
}

void motion_control_set_mode(MotionMode mode, float target) {
  current_mode = mode;
  target_value = target;

  // Reset PIDs when changing modes
  pid_reset_integral(&pid_left);
  pid_reset_integral(&pid_right);

  // Define the variables first
  MotorDirection left_dir = MOTOR_STOP;
  MotorDirection right_dir = MOTOR_STOP;
  int left_speed = 0;
  int right_speed = 0;

  // Add variables for spin angle control
  static SpinDirection spin_direction;
  static int spin_angle;

  if (mode == MOTION_STOP) {
    motor_drive(MOTOR_LEFT, MOTOR_STOP, 0);
    motor_drive(MOTOR_RIGHT, MOTOR_STOP, 0);
  }

  switch (mode) {
    case MOTION_STOP:
      left_dir = MOTOR_STOP;
      right_dir = MOTOR_STOP;
      left_speed = 0;
      right_speed = 0;
      break;

    case MOTION_FORWARD:
      left_dir = MOTOR_FORWARD;
      right_dir = MOTOR_FORWARD;
      break;

    case MOTION_REVERSE:
      left_dir = MOTOR_REVERSE;
      right_dir = MOTOR_REVERSE;
      break;

    case MOTION_SPIN_LEFT:
      spin_direction = SPIN_COUNTER_CLOCKWISE;
      spin_angle = 90;  // Default to 90 degrees if not specified
      break;

    case MOTION_SPIN_RIGHT:
      spin_direction = SPIN_CLOCKWISE;
      spin_angle = 90;  // Default to 90 degrees
      break;

    case MOTION_FOLLOW_DISTANCE:
      pid_set_setpoint(&pid_left, target);
      pid_set_setpoint(&pid_right, target);
      break;
    default:
      break;
  }
}

void motion_control_update(float front_distance, float back_distance, float left_distance, float right_distance) {
  uint32_t current_time = millis();

  int left_speed = BASE_SPEED;
  int right_speed = BASE_SPEED;
  MotorDirection left_dir = MOTOR_STOP;
  MotorDirection right_dir = MOTOR_STOP;

  // Define this variable at the top level to avoid redeclaration errors
  float error = 0.0f;
  float left_correction = 0.0f;
  float right_correction = 0.0f;
  float side_correction = 0.0f;  // Add this missing variable

  switch (current_mode) {
    case MOTION_STOP:
      left_dir = MOTOR_STOP;
      right_dir = MOTOR_STOP;
      left_speed = 0;
      right_speed = 0;
      break;

    case MOTION_FORWARD:
      // Base control on front distance
      error = front_distance - target_value;

      // Consider side distances to avoid obstacles
      side_correction = 0;
      if (left_distance < 15 && right_distance > 15) {
        // Obstacle on left, steer right
        side_correction = 10;
      } else if (right_distance < 15 && left_distance > 15) {
        // Obstacle on right, steer left
        side_correction = -10;
      }

      // Compute PID corrections
      left_correction = pid_compute(&pid_left, front_distance, current_time) + side_correction;
      right_correction = pid_compute(&pid_right, front_distance, current_time) - side_correction;

      // Apply corrections to base speed
      left_speed = BASE_SPEED + (int)left_correction;
      right_speed = BASE_SPEED - (int)right_correction;

      // Ensure speeds are within valid range
      if (left_speed > 100) left_speed = 100;
      if (left_speed < 0) left_speed = 0;
      if (right_speed > 100) right_speed = 100;
      if (right_speed < 0) right_speed = 0;

      left_dir = MOTOR_FORWARD;
      right_dir = MOTOR_FORWARD;
      break;

    case MOTION_REVERSE:
      // Use rear distance sensor for backing up
      if (back_distance < 5) {
        // Getting too close to rear obstacle
        left_speed = 0;
        right_speed = 0;
      } else {
        left_speed = BASE_SPEED;
        right_speed = BASE_SPEED;
      }

      left_dir = MOTOR_REVERSE;
      right_dir = MOTOR_REVERSE;
      break;

    case MOTION_SPIN_LEFT:
      if (car_spin_angle(SPIN_COUNTER_CLOCKWISE, TURNING_SPEED, 90)) {
        // When complete, automatically switch to STOP mode
        motion_control_set_mode(MOTION_STOP, 0);
      }
      return;

    case MOTION_SPIN_RIGHT:
      if (car_spin_angle(SPIN_CLOCKWISE, TURNING_SPEED, 90)) {
        // When complete, automatically switch to STOP mode
        motion_control_set_mode(MOTION_STOP, 0);
      }
      return;

    case MOTION_FOLLOW_DISTANCE:
      // Use PID to maintain target distance from obstacle
      error = front_distance - target_value;

      // Compute PID corrections (without redefining the variables)
      left_correction = pid_compute(&pid_left, front_distance, current_time);
      right_correction = pid_compute(&pid_right, front_distance, current_time);

      // Apply corrections to steering
      left_speed = BASE_SPEED + (int)left_correction;
      right_speed = BASE_SPEED - (int)right_correction;

      // Ensure speeds are within valid range
      if (left_speed > 100) left_speed = 100;
      if (left_speed < 0) left_speed = 0;
      if (right_speed > 100) right_speed = 100;
      if (right_speed < 0) right_speed = 0;

      left_dir = MOTOR_FORWARD;
      right_dir = MOTOR_FORWARD;
      break;
  }

  motor_drive(MOTOR_LEFT, left_dir, left_speed);
  motor_drive(MOTOR_RIGHT, right_dir, right_speed);
}

void motion_control_get_pid_output(float *left_output, float *right_output) {
  *left_output = pid_left.output;
  *right_output = pid_right.output;
}

MotionMode motion_control_get_mode(void) { return current_mode; }
