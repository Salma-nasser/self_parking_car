#include <Arduino.h>
#include "servo_control.h"
#include "ultrasonic.h"
#include "slot_detection.h"

// Pin definitions
#define LEFT_TRIG_PIN    32
#define LEFT_ECHO_PIN    33
#define RIGHT_TRIG_PIN   12
#define RIGHT_ECHO_PIN   13
#define FRONT_TRIG_PIN   5
#define FRONT_ECHO_PIN   18

// Thresholds
#define SLOT_DEPTH_THRESHOLD_CM 30
#define SLOT_WIDTH_THRESHOLD_CM 35



// // Utility: read distance from HC-SR04 (in cm)
// long read_distance_cm(uint8_t trig, uint8_t echo) {
//   digitalWrite(trig, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trig, LOW);
//   long duration = pulseIn(echo, HIGH, 25000);  // Timeout 25 ms
//   return duration * 0.034 / 2;
// }

// Main slot detection logic
SlotDirection detect_parking_slot() {
  long left_dist = read_distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  long right_dist = read_distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  bool candidate_left = left_dist > SLOT_DEPTH_THRESHOLD_CM;
  bool candidate_right = right_dist > SLOT_DEPTH_THRESHOLD_CM;

  if (!candidate_left && !candidate_right)
    return NO_SLOT;

  // Initialize and rotate servo toward the gap direction
  servo_init();

  if (candidate_left) {
    servo_set_angle(SERVO_LEFT);   // Rotate to left
    delay(500);                    // Allow servo to settle
    long width = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    return (width > SLOT_WIDTH_THRESHOLD_CM) ? SLOT_LEFT : NO_SLOT;
  }

  if (candidate_right) {
    servo_set_angle(SERVO_RIGHT);  // Rotate to right
    delay(500);
    long width = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    return (width > SLOT_WIDTH_THRESHOLD_CM) ? SLOT_RIGHT : NO_SLOT;
  }

  return NO_SLOT;
}
