#include "ultrasonic.h"
#include "low_level_functions.h"
#include "gpio_defs.h"

#include <Arduino.h>

#define TAG "ULTRASONIC"
#define MAX_TIMEOUT_US 30000  // 30ms timeout

// Initialize a sensor pair
void init_sensor(uint8_t trig, uint8_t echo) {
  pinConfig(trig, OUTPUT);
  pinConfig(echo, INPUT);
}

// Read distance in cm
float read_distance(uint8_t trig, uint8_t echo) {
  digitalWriteLowLevel(trig, 0);
  custDelayMicroseconds(2);

  digitalWriteLowLevel(trig, 1);
  custDelayMicroseconds(10);
  digitalWriteLowLevel(trig, 0);

  uint64_t start = micros();
  while (!digitalReadLowLevel(echo)) {
    if ((micros() - start) > MAX_TIMEOUT_US) {
      Serial.print("Timeout waiting for echo HIGH");
      return -1;
    }
  }

  uint64_t echo_start = micros();
  while (digitalReadLowLevel(echo)) {
    if ((micros() - echo_start) > MAX_TIMEOUT_US) {
      Serial.print("Timeout waiting for echo LOW");
      return -1;
    }
  }

  uint64_t echo_end = micros();
  float duration = echo_end - echo_start;
  float distance_cm = (duration * 0.0343f) / 2.0f;
  return distance_cm;
}
