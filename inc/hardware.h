#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include <stdint.h>

void hardwareInit(void);
uint32_t millis(void);
#endif // HARDWARE_H_