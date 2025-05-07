#include "slot_detection.h"

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

void init_slot_detection() {
  // Initialize variables
  slot_detection_active = false;
  slot_start_time = 0;
}

static bool check_side_distances() {
  // Read distances from side sensors
  last_left_distance = read_distance(TRIG_LEFT, ECHO_LEFT);
  last_right_distance = read_distance(TRIG_RIGHT, ECHO_RIGHT);

  // Check if we have enough space on either side
  return (last_left_distance > (CAR_LENGTH_CM + MIN_SLOT_MARGIN) ||
          last_right_distance > (CAR_LENGTH_CM + MIN_SLOT_MARGIN));
}

bool detect_parking_slot(ParkingSlot* slot) {
  static uint64_t current_time;

  if (!slot_detection_active) {
    if (check_side_distances()) {
      slot_detection_active = true;
      slot_start_time = micros();  // Arduino-compatible timer
      slot->start_position = 0;    // Could be encoder reading if available
    }
    return false;
  }

  current_time = micros();

  if (!check_side_distances()) {
    // Calculate width based on time difference and speed
    uint64_t time_diff = current_time - slot_start_time;
    float width = (time_diff / 1000000.0f) * CAR_SPEED_CMS;

    // Stop car for length measurement
    motion_control_set_mode(MOTION_STOP, 0);
    custDelayMicroseconds(100000);  // Wait for car to stop

    float length = 0.0f;
    for (int i = 0; i < 3; i++) {
      servo_set_angle(SERVO_RIGHT);
      delay(500);  // Wait for servo to rotate

      float current_length = read_distance(TRIG_FRONT, ECHO_FRONT);
      if (current_length > length) {
        length = current_length;  // Take largest valid reading
      }

      custDelayMicroseconds(50000);  // Short delay
    }

    // Reset servo to forward
    servo_set_angle(SERVO_FORWARD);
    delay(500);

    if (length > 0) {
      slot->width = width;
      slot->length = length;
      slot->is_valid = validate_parking_dimensions(slot);
    } else {
      slot->is_valid = false;
      Serial.println("[SLOT_DETECTION] Invalid length measurement");
    }

    slot_detection_active = false;
    return true;
  }

  return false;
}

bool validate_parking_dimensions(ParkingSlot* slot) {
  if (slot->length >= (CAR_LENGTH_CM + MIN_SLOT_MARGIN) &&
      slot->width >= (CAR_WIDTH_CM + MIN_SLOT_MARGIN)) {
    Serial.print("[SLOT_DETECTION] Valid parking slot found: Length=");
    Serial.print(slot->length, 2);
    Serial.print(", Width=");
    Serial.println(slot->width, 2);
    return true;
  }

  Serial.print("[SLOT_DETECTION] Invalid slot dimensions: Length=");
  Serial.print(slot->length, 2);
  Serial.print(", Width=");
  Serial.println(slot->width, 2);
  return false;
}
