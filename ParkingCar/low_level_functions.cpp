#include "low_level_functions.h"

// These are part of the ESP32 Arduino core and are safe to use
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"  // for ets_delay_us
#include "freertos/FreeRTOS.h"  // for vTaskDelay
#include "freertos/task.h"      // for portTICK_PERIOD_MS

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

static inline uint32_t get_ccount() {
  uint32_t ccount;
  __asm__ volatile("rsr.ccount %0"
                   : "=a"(ccount));
  return ccount;
}
void custDelay(uint32_t ms) {
  uint32_t start = micros();
  while ((micros() - start) < (ms * 1000))
    ;
}

void custDelayMicroseconds(uint32_t us) {
  uint32_t start = get_ccount();
  uint32_t cycles = us * 240;
  while ((get_ccount() - start) < cycles)
    ;
}

void configureGPIOInterrupt(uint8_t pin, uint8_t intr_type) {
  uint32_t reg_idx = pin / 32;
  uint32_t bit_pos = pin % 32;

  // Set interrupt type
  auto int_type_reg = reinterpret_cast<volatile uint32_t *>(GPIO_PIN_INT_TYPE_REG + (reg_idx * 4));
  *int_type_reg |= (intr_type << (bit_pos * 2));

  // Enable interrupt
  auto int_ena_reg = reinterpret_cast<volatile uint32_t *>(GPIO_PIN_INT_ENA_REG + (reg_idx * 4));
  *int_ena_reg |= (1U << bit_pos);
}

void installGPIOISR(uint8_t pin, void (*isr_handler)(void *), void *arg) {
  ets_isr_mask(1 << ETS_GPIO_INTR_SOURCE);
  ets_isr_attach(ETS_GPIO_INTR_SOURCE, isr_handler, arg);
  ets_isr_unmask(1 << ETS_GPIO_INTR_SOURCE);
}

void clearGPIOInterruptStatus(uint8_t pin) {
  REG_WRITE(GPIO_STATUS_W1TC_REG, (1UL << pin));
}

uint32_t getGPIOInterruptStatus(uint8_t pin) {
  return (REG_READ(GPIO_STATUS_REG) & (1UL << pin));
}
