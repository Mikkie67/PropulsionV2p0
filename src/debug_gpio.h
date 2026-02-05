#ifndef DEBUG_GPIO_H
#define DEBUG_GPIO_H

#include <Arduino.h>

// Store original digitalWrite and digitalRead
extern int __real_digitalRead(uint8_t pin);

// Wrapper for digitalRead that logs GPIO 0 access
inline int digitalRead(uint8_t pin) {
  if (pin == 0) {
    log_e("*** CAUGHT: digitalRead(0) called! Stack trace follows ***");
  }
  return __real_digitalRead(pin);
}

#endif // DEBUG_GPIO_H
