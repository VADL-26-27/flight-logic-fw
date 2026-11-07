#include "latch.h"

void set_latch_high(GPIO_TypeDef *gpio, uint8_t pin) {
  if ((gpio == GPIOA && pin != SW_3) || (gpio == GPIOB && pin == SW_3)) {
    return;
  }
  gpio->BSRR = 1U << pin;
}
void set_latch_low(GPIO_TypeDef *gpio, uint8_t pin) {
  if ((gpio == GPIOA && pin != SW_3) || (gpio == GPIOB && pin == SW_3)) {
    return;
  }
  gpio->BSRR = 1U << (pin + 16U);
}