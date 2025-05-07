#include "ultrasonic.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "low_level_functions.h"

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

  uint64_t start = esp_timer_get_time();
  while (!digitalReadLowLevel(echo)) {
    if ((esp_timer_get_time() - start) > MAX_TIMEOUT_US) {
      ESP_LOGW(TAG, "Timeout waiting for echo HIGH");
      return -1;
    }
  }

  uint64_t echo_start = esp_timer_get_time();
  while (digitalReadLowLevel(echo)) {
    if ((esp_timer_get_time() - echo_start) > MAX_TIMEOUT_US) {
      ESP_LOGW(TAG, "Timeout waiting for echo LOW");
      return -1;
    }
  }

  uint64_t echo_end = esp_timer_get_time();
  float duration = echo_end - echo_start;
  float distance_cm = (duration * 0.0343f) / 2.0f;
  return distance_cm;
}
