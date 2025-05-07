#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
  // PID constants
  float kp;  // Proportional gain
  float ki;  // Integral gain
  float kd;  // Derivative gain

  // PID variables
  float setpoint;    // Desired target value
  float integral;    // Accumulated error
  float prev_error;  // Previous error for derivative calculation
  float output;      // PID output

  // Limits
  float output_min;      // Minimum output limit
  float output_max;      // Maximum output limit
  float integral_limit;  // Anti-windup limit

  // Timing
  uint32_t last_time;  // Last update time in milliseconds
} PIDController;

// Initialize PID controller with gains and limits
void pid_init(PIDController *pid, float kp, float ki, float kd, float output_min, float output_max,
              float integral_limit);

// Update PID calculation with new input value
float pid_compute(PIDController *pid, float input, uint32_t current_time);

// Set a new target setpoint
void pid_set_setpoint(PIDController *pid, float setpoint);

// Reset the integral term
void pid_reset_integral(PIDController *pid);

#endif