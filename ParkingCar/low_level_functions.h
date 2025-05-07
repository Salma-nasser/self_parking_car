#ifndef LOW_LEVEL_FUNCTIONS_H
#define LOW_LEVEL_FUNCTIONS_H

#include <stdint.h>

// Simplified pin modes (compatible with your style)
#define OUTPUT 1
#define INPUT 0

void pinConfig(uint8_t pin, uint8_t mode);
void digitalWriteLowLevel(uint8_t pin, uint8_t value);
int digitalReadLowLevel(uint8_t pin);
void custDelay(uint32_t ms);
void custDelayMicroseconds(uint32_t us);

/* // Add new function declarations
void configureGPIOInterrupt(uint8_t pin, uint8_t intr_type);
void installGPIOISR(uint8_t pin, void (*isr_handler)(void*), void* arg);
void clearGPIOInterruptStatus(uint8_t pin);
uint8_t getGPIOInterruptStatus(uint8_t pin);
 */
#endif
