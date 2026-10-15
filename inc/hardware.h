#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include <stdint.h>

void hardwareInit(void);
uint32_t millis(void);
void state_leds_init(void);
void state_leds_all_on(void);
void state_leds_all_off(void);
void state_leds_show(uint8_t state);
#endif // HARDWARE_H_