#include <Arduino.h>
#include <BluetoothSerial.h>
#include "slot_detection.h"
#include "motion_control.h"
#include "motor_control.h"
#include "ultrasonic.h"
#include "servo_control.h"
#include "motor_pwm.h"

// Pin definitions for ultrasonic sensors
#define LEFT_TRIG_PIN    32
#define LEFT_ECHO_PIN    33
#define RIGHT_TRIG_PIN   12
#define RIGHT_ECHO_PIN   13
#define FRONT_TRIG_PIN   5
#define FRONT_ECHO_PIN   18

// PWM channel definitions
#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1

// Bluetooth setup
BluetoothSerial SerialBT;

bool moved = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car");

  motion_control_init();
  //motor_init();
  servo_init();

  // Initialize all sensors
  init_sensor(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  init_sensor(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  init_sensor(FRONT_TRIG_PIN, FRONT_ECHO_PIN);

  Serial.println("System initialized");
}

void loop() {
  if (SerialBT.hasClient() && !moved) {
    Serial.println("Bluetooth connected. Moving forward slowly...");
    
    // Start moving forward
    motor_drive(MOTOR_LEFT, MOTOR_FORWARD, 5);
    motor_drive(MOTOR_RIGHT, MOTOR_FORWARD, 5);

    // Check for parking slot
    SlotDirection slot = detect_parking_slot();

    // If slot is detected, stop the car and print results
    if (slot != NO_SLOT) {
      // Stop motors
      motor_drive(MOTOR_LEFT, MOTOR_STOP, 0);
      motor_drive(MOTOR_RIGHT, MOTOR_STOP, 0);

      if (slot == SLOT_LEFT) {
        Serial.println("Parking slot detected on LEFT side");
      } else if (slot == SLOT_RIGHT) {
        Serial.println("Parking slot detected on RIGHT side");
      }

      moved = true;
      Serial.println("Stopped.");
    }
  }
}

// Main pwm test
/* #include <Arduino.h>
#include <BluetoothSerial.h>
#include "motor_pwm.h"
#include "low_level_functions.h"

// Bluetooth setup
BluetoothSerial SerialBT;

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8  // 0–255 duty range

bool moved = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car");



  Serial.println("Waiting for Bluetooth connection...");
}

void loop() {
  if (SerialBT.hasClient() && !moved) {
    Serial.println("Bluetooth connected. Moving forward slowly...");

    // Set speed (e.g. 40% for slow movement)
    uint8_t speed = 20;
    motor_set_speed(LEFT_PWM_CHANNEL, speed);
    motor_set_speed(RIGHT_PWM_CHANNEL, speed);

    delay(3000); // Run for 3 seconds

    // Stop motors
    motor_set_speed(LEFT_PWM_CHANNEL, 0);
    motor_set_speed(RIGHT_PWM_CHANNEL, 0);

    moved = true;
    Serial.println("Stopped.");
  }
} */
