#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

typedef enum { MOTOR_LEFT, MOTOR_RIGHT } MotorSide;

typedef enum { MOTOR_FORWARD, MOTOR_REVERSE, MOTOR_STOP } MotorDirection;
typedef enum { SPIN_CLOCKWISE, SPIN_COUNTER_CLOCKWISE } SpinDirection;

void motor_init();
void motor_drive(MotorSide side, MotorDirection direction, uint8_t speed_percent);
void car_spin(SpinDirection spin_dir, uint8_t speed_percent);
#endif
