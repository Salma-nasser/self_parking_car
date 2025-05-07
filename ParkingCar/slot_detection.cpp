#include "slot_detection.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "gpio_defs.h"
#include "low_level_functions.h"
#include "motion_control.h"
#include "pin_defs.h"
#include "servo_control.h"
#include "ultrasonic.h"

#define TAG "SLOT_DETECTION"

static volatile uint64_t slot_start_time = 0;
static volatile bool slot_detection_active = false;
static volatile float last_left_distance = 0;
static volatile float last_right_distance = 0;

void init_slot_detection(void) {
  // Initialize variables
  slot_detection_active = false;
  slot_start_time = 0;
}

static bool check_side_distances(void) {
  // Read distances from side sensors
  last_left_distance = read_distance(TRIG_LEFT, ECHO_LEFT);
  last_right_distance = read_distance(TRIG_RIGHT, ECHO_RIGHT);

  // Check if we have enough space on either side (using LENGTH now, not width)
  return (last_left_distance > (CAR_LENGTH_CM + MIN_SLOT_MARGIN) ||
          last_right_distance > (CAR_LENGTH_CM + MIN_SLOT_MARGIN));
}

bool detect_parking_slot(ParkingSlot* slot) {
  static uint64_t current_time;

  if (!slot_detection_active) {
    if (check_side_distances()) {
      slot_detection_active = true;
      slot_start_time = esp_timer_get_time();
      slot->start_position = 0;
    }
    return false;
  }

  current_time = esp_timer_get_time();

  if (!check_side_distances()) {
    // Calculate width from car movement
    uint64_t time_diff = current_time - slot_start_time;
    float width = (time_diff / 1000000.0f) * CAR_SPEED_CMS;

    // Stop car for length measurement
    motion_control_set_mode(MOTION_STOP, 0);
    custDelayMicroseconds(100000);  // Wait for car to stop completely

    float length = 0.0f;
    // Take multiple measurements for accuracy
    for (int i = 0; i < 3; i++) {
      // Rotate sensor right
      servo_set_angle(SERVO_RIGHT);
      custDelayMicroseconds(500000);  // Wait for servo

      // Measure and accumulate length
      float current_length = read_distance(TRIG_FRONT, ECHO_FRONT);
      if (current_length > length) {
        length = current_length;  // Take largest valid reading
      }

      custDelayMicroseconds(50000);  // Short delay between readings
    }

    // Return sensor to forward position
    servo_set_angle(SERVO_FORWARD);
    custDelayMicroseconds(500000);

    // Store and validate measurements
    if (length > 0) {  // Ensure valid reading
      slot->width = width;
      slot->length = length;
      slot->is_valid = validate_parking_dimensions(slot);
    } else {
      slot->is_valid = false;
      ESP_LOGW(TAG, "Invalid length measurement");
    }

    slot_detection_active = false;
    return true;
  }

  return false;
}

bool validate_parking_dimensions(ParkingSlot* slot) {
  if (slot->length >= (CAR_LENGTH_CM + MIN_SLOT_MARGIN) && slot->width >= (CAR_WIDTH_CM + MIN_SLOT_MARGIN)) {
    ESP_LOGI(TAG, "Valid parking slot found: Length=%.2f, Width=%.2f", slot->length, slot->width);
    return true;
  }
  ESP_LOGW(TAG, "Invalid slot dimensions: Length=%.2f, Width=%.2f", slot->length, slot->width);
  return false;
}