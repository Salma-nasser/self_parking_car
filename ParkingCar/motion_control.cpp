#include "motion_control.h"
#include "motor_control.h"
#include <Arduino.h>
#include "pid_controller.h"

// PID controllers for left and right wheels sides
static PIDController pid_left;
static PIDController pid_right;

// Current motion control state
static MotionMode current_mode = MOTION_STOP;
static float target_value = 0.0f;

// Base speed settings
#define BASE_SPEED 50      // Base speed percentage (0-100)
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

  if (mode == MOTION_STOP) {
    motor_drive(MOTOR_LEFT, MOTOR_STOP, 0);
    motor_drive(MOTOR_RIGHT, MOTOR_STOP, 0);
  }

  switch (mode) {
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

  switch (current_mode) {
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
      car_spin(SPIN_COUNTER_CLOCKWISE, BASE_SPEED);
      return;

    case MOTION_SPIN_RIGHT:
      car_spin(SPIN_CLOCKWISE, BASE_SPEED);
      return;

    case MOTION_FOLLOW_DISTANCE: {
      float error = front_distance;

      float left_correction = pid_compute(&pid_left, error, current_time);
      float right_correction = pid_compute(&pid_right, error, current_time);

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
  }

  motor_drive(MOTOR_LEFT, left_dir, left_speed);
  motor_drive(MOTOR_RIGHT, right_dir, right_speed);
}

void motion_control_get_pid_output(float *left_output, float *right_output) {
  *left_output = pid_left.output;
  *right_output = pid_right.output;
}
