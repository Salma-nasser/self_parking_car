#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

#define SERVO_PIN 19   // GPIO pin for servo
#define SERVO_FREQ 50  // 50Hz for typical servos
#define SERVO_TIMER 0
#define SERVO_CHANNEL 4

// Servo positions in degrees
#define SERVO_FORWARD 90
#define SERVO_RIGHT 180
#define SERVO_LEFT 0

void servo_init(void);
void servo_set_angle(uint8_t angle);

#endif