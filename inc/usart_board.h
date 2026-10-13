#ifndef USART_BOARD_H_
#define USART_BOARD_H_

#include "usart.h"
#include "hardware.h"

extern usart_t uart1;
extern usart_t uart2;

void board_usart_init(void);
#endif // USART_BOARD_H_