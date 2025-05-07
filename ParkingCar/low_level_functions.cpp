#include "low_level_functions.h"

// These includes are available in ESP32 Arduino core
#include <rom/ets_sys.h>         // for ets_delay_us
#include <freertos/FreeRTOS.h>   // for vTaskDelay and portTICK_PERIOD_MS
#include "gpio_defs.h"

void pinConfig(uint8_t pin, uint8_t mode) {
  if (mode == OUTPUT) {
    if (pin < 32) {
      *((volatile uint32_t *)GPIO_ENABLE_W1TS_REG) = (1 << pin);
    } else {
      *((volatile uint32_t *)GPIO_ENABLE1_W1TS_REG) = (1 << (pin - 32));
    }
  } else if (mode == INPUT) {
    if (pin < 32) {
      *((volatile uint32_t *)GPIO_ENABLE_W1TC_REG) = (1 << pin);
    } else {
      *((volatile uint32_t *)GPIO_ENABLE1_W1TC_REG) = (1 << (pin - 32));
    }
  }
}

void digitalWriteLowLevel(uint8_t pin, uint8_t value) {
  if (value) {
    if (pin < 32) {
      *((volatile uint32_t *)GPIO_OUT_W1TS_REG) = (1 << pin);
    } else {
      *((volatile uint32_t *)GPIO_OUT1_W1TS_REG) = (1 << (pin - 32));
    }
  } else {
    if (pin < 32) {
      *((volatile uint32_t *)GPIO_OUT_W1TC_REG) = (1 << pin);
    } else {
      *((volatile uint32_t *)GPIO_OUT1_W1TC_REG) = (1 << (pin - 32));
    }
  }
}

int digitalReadLowLevel(uint8_t pin) {
  if (pin < 32) {
    return ((*((volatile uint32_t *)GPIO_IN_REG)) >> pin) & 1;
  } else {
    return ((*((volatile uint32_t *)GPIO_IN1_REG)) >> (pin - 32)) & 1;
  }
}

void custDelay(uint32_t ms) {
  vTaskDelay(ms / portTICK_PERIOD_MS);  // Only works inside a task
}

void custDelayMicroseconds(uint32_t us) {
  ets_delay_us(us);
}
