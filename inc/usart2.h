#ifndef USART2_H_
#define USART2_H_
#include "stm32f411xe.h"
#include <stdint.h>
#include "ring_buf.h"

// Configure USART2 for 115200 baud, 
void usart2_init(void);
void usart2_receive(rx_buf* buf, uint32_t len);
void usart2_write_command(const char *command);

extern rx_buf imu_buf;
extern volatile uint8_t new_data;

#endif // USART2_H_