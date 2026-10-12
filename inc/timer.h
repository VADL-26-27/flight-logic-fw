#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include "stm32f411xe.h"

void timer_init(void);
uint32_t now_ms(void);
void delay_ms(uint32_t duration); 
#endif // TIMER_H_