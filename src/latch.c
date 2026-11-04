#include "latch.h"

void set_latch_high(GPIO_TypeDef *gpio, uint8_t pin) { gpio->BSRR = 1U << pin; }
void set_latch_low(GPIO_TypeDef *gpio, uint8_t pin) {
  gpio->BSRR = 1U << (pin + 16U);
}