#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

void motor_pwm_init(uint8_t pwm_pin, uint8_t channel, uint32_t freq, uint8_t resolution);
void motor_set_speed(uint8_t channel, uint8_t speed_percent);  // 0–100%

#endif
