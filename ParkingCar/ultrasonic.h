#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

void init_sensor(uint8_t trig, uint8_t echo);
float read_distance(uint8_t trig, uint8_t echo);

#endif
