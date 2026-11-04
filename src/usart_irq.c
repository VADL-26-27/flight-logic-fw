#include "usart.h"
#include "usart_board.h"

void USART1_IRQHandler(void)
{
    usart_handle_irq(&uart1);
}

void USART2_IRQHandler(void)
{
    usart_handle_irq(&uart2);
}