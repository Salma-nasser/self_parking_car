#ifndef SLOT_DETECTION_H
#define SLOT_DETECTION_H

#include <stdbool.h>
#include <stdint.h>
#include <Arduino.h>   // for uint8_t, delay(), etc.

// Thresholds
#define MIN_SLOT_LENGTH_CM 15   // Minimum slot length to be considered valid
#define CAR_SPEED_CMS 20   // Car speed in cm/s
#define SLOT_DEPTH_THRESHOLD_CM 30

// “Which way has the slot?”  
typedef enum {
  NO_SLOT,
  SLOT_LEFT,
  SLOT_RIGHT
} SlotDirection;

// your C++ prototype, matching slot_detection.cpp:
SlotDirection detect_parking_slot();

#endif