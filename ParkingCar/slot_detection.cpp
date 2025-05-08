#include <Arduino.h>
#include "ultrasonic.h"
#include "slot_detection.h"

// Pin definitions
#define LEFT_TRIG_PIN    32
#define LEFT_ECHO_PIN    33
#define RIGHT_TRIG_PIN   12
#define RIGHT_ECHO_PIN   13
#define FRONT_TRIG_PIN   5
#define FRONT_ECHO_PIN   18


// State variables for slot detection
static bool leftSlotDetectionActive = false;
static bool rightSlotDetectionActive = false;
static unsigned long leftSlotStartTime = 0;
static unsigned long rightSlotStartTime = 0;
static float leftSlotLength = 0;
static float rightSlotLength = 0;

// Main slot detection logic
SlotDirection detect_parking_slot() {
  float left_dist = read_distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  float right_dist = read_distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  
  // Debug output
  Serial.print("Left distance: ");
  Serial.print(left_dist);
  Serial.print(" cm, Right distance: ");
  Serial.print(right_dist);
  Serial.println(" cm");

  // Detect potential parking slots (open spaces) on left side
  if (left_dist > SLOT_DEPTH_THRESHOLD_CM) {
    if (!leftSlotDetectionActive) {
      leftSlotDetectionActive = true;
      leftSlotStartTime = millis();
      Serial.println("Left slot detection started");
    }
  } else {
    // End of slot detection if distance becomes small
    if (leftSlotDetectionActive) {
      unsigned long elapsed = millis() - leftSlotStartTime;
      leftSlotLength = (elapsed / 1000.0) * CAR_SPEED_CMS;  // Convert to seconds then multiply by speed
      Serial.print("Left slot ended. Length: ");
      Serial.print(leftSlotLength);
      Serial.println(" cm");
      leftSlotDetectionActive = false;
    }
  }
  
  // Same logic for right side
  if (right_dist > SLOT_DEPTH_THRESHOLD_CM) {
    if (!rightSlotDetectionActive) {
      rightSlotDetectionActive = true;
      rightSlotStartTime = millis();
      Serial.println("Right slot detection started");
    }
  } else {
    // End of slot detection if distance becomes small
    if (rightSlotDetectionActive) {
      unsigned long elapsed = millis() - rightSlotStartTime;
      rightSlotLength = (elapsed / 1000.0) * CAR_SPEED_CMS;  // Convert to seconds then multiply by speed
      Serial.print("Right slot ended. Length: ");
      Serial.print(rightSlotLength);
      Serial.println(" cm");
      rightSlotDetectionActive = false;
    }
  }

  // Check if we have valid slots and return the appropriate direction
  // Priority: left side if both are valid
  if (leftSlotLength > MIN_SLOT_LENGTH_CM) {
    float tempLength = leftSlotLength;
    leftSlotLength = 0;  // Reset for next detection
    return SLOT_LEFT;
  } 
  else if (rightSlotLength > MIN_SLOT_LENGTH_CM) {
    float tempLength = rightSlotLength;
    rightSlotLength = 0;  // Reset for next detection
    return SLOT_RIGHT;
  }

  return NO_SLOT;
}