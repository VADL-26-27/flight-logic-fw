#ifndef LATCH_H_
#define LATCH_H_

#define SW_1 4U
#define SW_2 5U
#define SW_3 8U

#include "stm32f413xx.h"
void set_latch_high(GPIO_TypeDef *gpio, uint8_t pin);
void set_latch_low(GPIO_TypeDef *gpio, uint8_t pin);
#endif // LATCH_H_