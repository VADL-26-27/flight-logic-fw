#ifndef USART2_H_
#define USART2_H_
#include "stm32f411xe.h"
#include <stdbool.h>
#include <stdint.h>
#include "ring_buf.h"

#define USART2_PACKET_SIZE 36
// Transmit only, 115200 baud with a 16 MHz APB1 clock.
void usart2_init(void);
bool usart2_write_packet(const uint8_t packet[USART2_PACKET_SIZE]);

extern ring_buf tx_buf;
#endif // USART2_H_
