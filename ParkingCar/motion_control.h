#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

#include "pid_controller.h"

// Motion control modes
typedef enum {
  MOTION_STOP,
  MOTION_FORWARD,
  MOTION_REVERSE,
  MOTION_SPIN_LEFT,
  MOTION_SPIN_RIGHT,
  MOTION_FOLLOW_DISTANCE
} MotionMode;

// Initialize motion control system
void motion_control_init(void);

// Update motion control based on sensors
void motion_control_update(float front_distance, float back_distance, float left_distance, float right_distance);

// Set desired motion mode
void motion_control_set_mode(MotionMode mode, float target_value);

// Get current PID outputs for debugging
void motion_control_get_pid_output(float *left_output, float *right_output);

#endif