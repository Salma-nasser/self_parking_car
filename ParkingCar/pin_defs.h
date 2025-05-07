#include <ESP32Servo.h>

// Ultrasonic sensor pins
#define TRIG_FRONT 5
#define ECHO_FRONT 18
#define TRIG_REAR 2
#define ECHO_REAR 15
#define TRIG_LEFT 32
#define ECHO_LEFT 33
#define TRIG_RIGHT 12
#define ECHO_RIGHT 13
#define SERVO_PIN 19

#define MOTOR1_FORWARD 26
#define MOTOR1_BACKWARD 25
#define MOTOR2_FORWARD 16
#define MOTOR2_BACKWARD 17

// Servo control (angles for rotation)
#define SERVO_RIGHT    90   // Degrees to look right
#define SERVO_FORWARD  0    // Default forward angle
