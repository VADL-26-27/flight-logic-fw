#include "usart2.h"

rx_buf imu_buf;
volatile uint8_t new_data = 0;

void usart2_init(void) {
    init_buf(&imu_buf);
    // Enable USART2 periphreal clock (low power and regular)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB1LPENR |= RCC_APB1LPENR_USART2LPEN;
    
    // Disable USART2 before configuration
    USART2->CR1 &= ~(USART_CR1_UE);

    /*
    * Baud Rate assuming APB1 periphreal clock = 16 MHZ
    * Baud Rate = 115200
     */
    USART2->BRR = (16000000U + 115200U / 2U) / 115200U;
    
    /*
    * CR1 Config:
    * 8 data bits
    * No Parity
    * RX Enable
    * TX Enable to send IMU Command
    * Enable RX Interrupts
    */

    // Enable TX, RX, sets OVER8 to 16 and Word length to 8
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE;

   // 1 stop bit, don't need other features in register
   USART2->CR2 = 0;

   // Nothing needed in CR3 either
   USART2->CR3 = 0;

   NVIC_EnableIRQ(USART2_IRQn);
   USART2->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE;
}

void USART2_IRQHandler(void) {
    uint32_t status = USART2->SR;

    if (status & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)USART2->DR;
        enqueue(&imu_buf, byte);
        new_data = 1;
    }
}

void usart2_write_command(const char* command) {
    while (*command != '\0') {
        while (!(USART2->SR & USART_SR_TXE)) {
            // Wait until the transmit data reg is empty
            // Done while on ground so busy wait blocking is fine
        }
        USART2->DR = (uint8_t)*command++;
    }

    // Wait until final byte has finished leaving uart
    while(!(USART2->SR & USART_SR_TC)) {}
}