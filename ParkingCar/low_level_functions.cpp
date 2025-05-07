#include "low_level_functions.h"
#include "gpio_defs.h"
#include "soc/io_mux_reg.h"       // For IO_MUX and related
#include "soc/gpio_reg.h"         // For GPIO register addresses
#include "driver/gpio.h"          // For GPIO mode constants
// #include "soc/soc.h"              // For REG_READ, REG_WRITE

// These are part of the ESP32 Arduino core and are safe to use
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <Arduino.h>

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

static inline uint32_t getCycleCount() {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount" : "=a"(ccount));
  return ccount;
}

void custDelayMicroseconds(uint32_t us) {
  uint32_t start = getCycleCount();
  uint32_t wait = us * 240;
  while ((getCycleCount() - start) < wait);
}

void custDelay(uint32_t ms) {
  while (ms--) {
    custDelayMicroseconds(1000);
  }
}

/* void configureGPIOInterrupt(uint8_t pin, gpio_int_type_t intr_type) {
  // Set pin interrupt type
  uint32_t reg = GPIO_PIN_REG(pin);
  uint32_t val = REG_READ(reg);
  val &= ~(GPIO_PIN_INT_TYPE_M);  // Clear interrupt type bits
  val |= (intr_type << GPIO_PIN_INT_TYPE_S);
  REG_WRITE(reg, val);

  // Enable interrupt
  REG_SET_BIT(GPIO_INT_ENA_REG, (1UL << pin));
}

void installGPIOISR(uint8_t pin, void (*isr_handler)(void *), void *arg) {
  gpio_install_isr_service(0);  // Only needs to be called once
  gpio_isr_handler_add((gpio_num_t)pin, isr_handler, arg);
}

void clearGPIOInterruptStatus(uint8_t pin) {
  REG_WRITE(GPIO_STATUS_W1TC_REG, (1UL << pin));
}

uint32_t getGPIOInterruptStatus(uint8_t pin) {
  return (REG_READ(GPIO_STATUS_REG) & (1UL << pin));
} */
