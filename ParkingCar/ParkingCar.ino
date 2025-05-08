#include <Arduino.h>
#define CONFIG_BT_ENABLED
#include <BluetoothSerial.h>

#include "low_level_functions.h"
#include "motion_control.h"
#include "motor_control.h"
#include "motor_pwm.h"
#include "pid_controller.h"
#include "sdkconfig.h"
#include "slot_detection.h"
#include "ultrasonic.h"

// Pin definitions for ultrasonic sensors
#define LEFT_TRIG_PIN 32
#define LEFT_ECHO_PIN 33
#define RIGHT_TRIG_PIN 12
#define RIGHT_ECHO_PIN 13
#define FRONT_TRIG_PIN 5
#define FRONT_ECHO_PIN 18
#define REAR_TRIG_PIN 2
#define REAR_ECHO_PIN 15

// Constants for parking
#define DRIVING_SPEED 20    // Normal driving speed (%)
#define PARKING_SPEED 30    // Speed during parking maneuvers (%)
#define TURNING_SPEED 50    // Speed for differential steering turns (%)
#define SAFETY_DISTANCE 15  // Safety distance from obstacles (cm)

// Bluetooth setup
BluetoothSerial SerialBT;

// State machine for perpendicular parking process
enum ParkingState {
  STATE_IDLE,
  STATE_SEARCH_SLOT,
  STATE_TURN_INTO_SLOT,
  STATE_BACK_INTO_SLOT,
  STATE_PARKED,
  STATE_ERROR,
  STATE_WAIT_FOR_TURN
};

ParkingState currentState = STATE_IDLE;
SlotDirection detectedSlot = NO_SLOT;
float frontDistance, rearDistance, leftDistance, rightDistance;
unsigned long stateTimer = 0;

// Add these variables to track spin state
bool spinStarted = false;
bool spinCompleted = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car");

  // Initialize motion control
  motion_control_init();

  // Initialize all sensors
  init_sensor(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  init_sensor(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  init_sensor(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  init_sensor(REAR_TRIG_PIN, REAR_ECHO_PIN);

  Serial.println("Self-Parking Car initialized");
  SerialBT.println("Self-Parking Car ready. Connect to start.");
}

// Read all sensor distances
void updateSensorReadings() {
  frontDistance = read_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  rearDistance = read_distance(REAR_TRIG_PIN, REAR_ECHO_PIN);
  leftDistance = read_distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  rightDistance = read_distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  // Debug sensor readings
  Serial.printf("Distances - Front: %.1f, Rear: %.1f, Left: %.1f, Right: %.1f\n", frontDistance, rearDistance,
                leftDistance, rightDistance);
}

// Execute perpendicular parking to the right (using differential steering)
void executeRightPerpendicularParking() {
  SerialBT.println("Starting right perpendicular parking maneuver");

  // Step 1: Move slightly forward
  // moveCarDistance(MOTION_FORWARD, PARKING_SPEED, 10);
  // custDelay(500);

  // Step 2: Turn RIGHT in place
  motion_control_set_mode(MOTION_SPIN_RIGHT, 0);
  custDelay(1200);
  motion_control_set_mode(MOTION_STOP, 0);
  custDelay(500);

  // Step 3: Reverse precisely 35cm
  moveCarDistance(MOTION_REVERSE, PARKING_SPEED, 35);

  SerialBT.println("Parking complete");
}

// Execute perpendicular parking to the left (using differential steering)
void executeLeftPerpendicularParking() {
  SerialBT.println("Starting left perpendicular parking maneuver");

  // Step 1: Move slightly forward
  // SHOULD NOT MOVE FORWARD
  // moveCarDistance(MOTION_FORWARD, PARKING_SPEED, 10);
  // custDelay(500);

  // Step 2: Turn left in place
  motion_control_set_mode(MOTION_SPIN_LEFT, 0);
  // car_spin(SPIN_COUNTER_CLOCKWISE, TURNING_SPEED);
  custDelay(1200);
  motion_control_set_mode(MOTION_STOP, 0);
  custDelay(500);

  // Step 3: Reverse precisely 35cm
  moveCarDistance(MOTION_REVERSE, PARKING_SPEED, 35);

  SerialBT.println("Parking complete");
}
// Add this function to your code
void moveCarDistance(MotionMode direction, int speed_percent, float distance_cm) {
  // Set the direction
  motion_control_set_mode(direction, 0);

  // Set speed directly
  motor_set_speed(LEFT_PWM_CHANNEL, speed_percent);
  motor_set_speed(RIGHT_PWM_CHANNEL, speed_percent);

  // Estimate time needed to travel the distance
  float speed_cm_per_sec = speed_percent * 0.2;  // Approximate conversion
  int time_needed_ms = (distance_cm / speed_cm_per_sec) * 1000;

  // Execute the movement with timeout
  unsigned long startTime = millis();
  while ((millis() - startTime < time_needed_ms)) {
    updateSensorReadings();

    // Safety check
    if ((direction == MOTION_FORWARD && frontDistance < 10) || (direction == MOTION_REVERSE && rearDistance < 5)) {
      break;  // Stop if too close to obstacle
    }

    custDelay(50);
  }

  motion_control_set_mode(MOTION_STOP, 0);
}
void loop() {
  if (SerialBT.hasClient()) {
    // Check if there's a command from Bluetooth
    if (SerialBT.available()) {
      char cmd = SerialBT.read();
      if (cmd == 'S') {
        // Start parking sequence
        currentState = STATE_SEARCH_SLOT;
        stateTimer = millis();
        SerialBT.println("Starting perpendicular parking sequence...");
      } else if (cmd == 'X') {
        // Emergency stop
        currentState = STATE_IDLE;
        motion_control_set_mode(MOTION_STOP, 0);
        SerialBT.println("Emergency stop activated");
      }
    }

    // Update sensor readings
    updateSensorReadings();

    // Main state machine for perpendicular parking
    switch (currentState) {
      case STATE_IDLE:
        // Do nothing, wait for commands
        break;

      case STATE_SEARCH_SLOT:
        SerialBT.println("Searching for parking slot...");

        // follow front‐distance sensor, hold SAFETY_DISTANCE cm
        motion_control_set_mode(MOTION_FOLLOW_DISTANCE, 20);
        // Detect parking slot
        detectedSlot = detect_parking_slot();

        if (detectedSlot != NO_SLOT) {
          // Slot detected, stop and prepare for parking
          motion_control_set_mode(MOTION_STOP, 0);

          if (detectedSlot == SLOT_LEFT) {
            SerialBT.println("Perpendicular parking slot detected on LEFT side");
          } else {
            SerialBT.println("Perpendicular parking slot detected on RIGHT side");
          }

          currentState = STATE_TURN_INTO_SLOT;
          stateTimer = millis();
          custDelay(1000);  // Brief pause
        }

        // Add timeout for search
        if (millis() - stateTimer > 30000) {  // 30-second timeout
          SerialBT.println("Timeout: No suitable parking slot found");
          motion_control_set_mode(MOTION_STOP, 0);
          currentState = STATE_IDLE;
        }
        break;

      case STATE_TURN_INTO_SLOT:
        SerialBT.println("Turning into parking slot...");

        if (!spinStarted) {
          spinStarted = true;

          // Set the appropriate spin mode based on slot direction
          if (detectedSlot == SLOT_LEFT) {
            SerialBT.println("Executing left perpendicular parking");
            motion_control_set_mode(MOTION_SPIN_LEFT, 90);  // Use angle as parameter
          } else {
            SerialBT.println("Executing right perpendicular parking");
            motion_control_set_mode(MOTION_SPIN_RIGHT, 90);  // Use angle as parameter
          }

          currentState = STATE_WAIT_FOR_TURN;
        }
        break;

      case STATE_WAIT_FOR_TURN:
        // Check if we're back in STOP mode (which happens when turn completes)
        if (motion_control_get_mode() == MOTION_STOP) {
          currentState = STATE_BACK_INTO_SLOT;
        }
        break;

      case STATE_PARKED:
        // Nothing to do, car is parked
        break;

      case STATE_ERROR:
        // Handle error state
        SerialBT.println("Error in parking sequence");
        motion_control_set_mode(MOTION_STOP, 0);
        currentState = STATE_IDLE;
        break;
    }
  }

  // Safety checks - stop if obstacles are too close
  if ((currentState != STATE_IDLE && currentState != STATE_PARKED) &&
      ((frontDistance < SAFETY_DISTANCE && (currentState == STATE_SEARCH_SLOT)) ||
       (rearDistance < 2 && (currentState == STATE_TURN_INTO_SLOT || currentState == STATE_BACK_INTO_SLOT)))) {
    motion_control_set_mode(MOTION_STOP, 0);
    SerialBT.println("Safety stop: obstacle detected too close");
    currentState = STATE_ERROR;
  }

  // System monitoring
  motion_control_update(frontDistance, rearDistance, leftDistance, rightDistance);

  custDelay(100);  // Main loop delay
}