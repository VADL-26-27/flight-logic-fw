#ifndef USART1_H_
#define USART1_H_
#include "stm32f411xe.h"
#include <stdint.h>
#include "ring_buf.h"

// Configure USART1 for 115200 baud,
void usart1_init(void);
void usart1_receive(ring_buf *buf, uint32_t len);
void usart1_write_command(const char *command);

extern ring_buf rx_buf;
extern volatile uint8_t new_data;

#endif // USART1_H_