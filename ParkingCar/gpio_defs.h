#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

#include <stdint.h>

// Base addresses for GPIO register blocks (ESP32 Technical Reference Manual)
#define DR_REG_GPIO_BASE 0x3FF44000

// GPIO Output registers
#define GPIO_OUT_REG (DR_REG_GPIO_BASE + 0x04)
#define GPIO_OUT_W1TS_REG (DR_REG_GPIO_BASE + 0x08)
#define GPIO_OUT_W1TC_REG (DR_REG_GPIO_BASE + 0x0C)

// GPIO Input register
#define GPIO_IN_REG (DR_REG_GPIO_BASE + 0x3C)

// GPIO Direction (Enable) registers
#define GPIO_ENABLE_REG (DR_REG_GPIO_BASE + 0x20)
#define GPIO_ENABLE_W1TS_REG (DR_REG_GPIO_BASE + 0x24)
#define GPIO_ENABLE_W1TC_REG (DR_REG_GPIO_BASE + 0x28)

// For GPIOs >= 32 (GPIO32-GPIO39), use GPIO1 registers
#define DR_REG_GPIO_BASE_HIGH 0x3FF44400
#define GPIO_IN1_REG (DR_REG_GPIO_BASE + 0x40)
#define GPIO_ENABLE1_REG (DR_REG_GPIO_BASE + 0x2C)
#define GPIO_ENABLE1_W1TS_REG (DR_REG_GPIO_BASE + 0x30)
#define GPIO_ENABLE1_W1TC_REG (DR_REG_GPIO_BASE + 0x34)
#define GPIO_OUT1_REG (DR_REG_GPIO_BASE + 0x10)
#define GPIO_OUT1_W1TS_REG (DR_REG_GPIO_BASE + 0x14)
#define GPIO_OUT1_W1TC_REG (DR_REG_GPIO_BASE + 0x18)

#endif  // GPIO_DEFS_H
