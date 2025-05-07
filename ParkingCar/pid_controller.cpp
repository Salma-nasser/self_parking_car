#include "pid_controller.h"

void pid_init(PIDController *pid, float kp, float ki, float kd, float output_min, float output_max,
              float integral_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->setpoint = 0.0f;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output = 0.0f;
  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->integral_limit = integral_limit;
  pid->last_time = 0;
}

float pid_compute(PIDController *pid, float input, uint32_t current_time) {
  float dt;

  if (pid->last_time == 0) {
    dt = 0.01f;  // Assume 10ms for first run
  } else {
    dt = (current_time - pid->last_time) / 1000.0f;  // ms to seconds
  }

  pid->last_time = current_time;

  if (dt <= 0.0f) {
    dt = 0.01f;
  }

  float error = pid->setpoint - input;

  // Proportional
  float p_term = pid->kp * error;

  // Integral with anti-windup
  pid->integral += error * dt;
  if (pid->integral > pid->integral_limit)
    pid->integral = pid->integral_limit;
  else if (pid->integral < -pid->integral_limit)
    pid->integral = -pid->integral_limit;

  float i_term = pid->ki * pid->integral;

  // Derivative
  float derivative = (error - pid->prev_error) / dt;
  float d_term = pid->kd * derivative;
  pid->prev_error = error;

  // Output with clamping
  pid->output = p_term + i_term + d_term;
  if (pid->output > pid->output_max)
    pid->output = pid->output_max;
  else if (pid->output < pid->output_min)
    pid->output = pid->output_min;

  return pid->output;
}

void pid_set_setpoint(PIDController *pid, float setpoint) {
  pid->setpoint = setpoint;
}

void pid_reset_integral(PIDController *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}
