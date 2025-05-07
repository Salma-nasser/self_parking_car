#include "gpio_defs.h"
#include "low_level_functions.h"
#include "servo_control.h"

// PWM configurations for servo control
#define SERVO_RESOLUTION 16
#define SERVO_MIN_DUTY 1638  // 1ms pulse (0 degrees)
#define SERVO_MAX_DUTY 8192  // 2ms pulse (180 degrees)

void servo_init(void) {
  // Configure GPIO as output
  pinConfig(SERVO_PIN, OUTPUT);

  // Configure PWM timer directly using registers
  *((volatile uint32_t*)(DR_REG_GPIO_BASE + 0x554)) = ((SERVO_FREQ << 4) | (SERVO_RESOLUTION << 1));

  // Set initial position to forward
  servo_set_angle(SERVO_FORWARD);
}

void servo_set_angle(uint8_t angle) {
  // Convert angle to duty cycle
  uint32_t duty = SERVO_MIN_DUTY + ((SERVO_MAX_DUTY - SERVO_MIN_DUTY) * angle / 180);

  // Set PWM duty cycle directly using registers
  *((volatile uint32_t*)(DR_REG_GPIO_BASE + 0x558 + SERVO_CHANNEL * 4)) = duty;
}