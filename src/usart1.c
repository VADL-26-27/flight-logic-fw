#include "usart1.h"

ring_buf rx_buf;
volatile uint8_t new_data = 0;

void usart1_init(void) {
  init_buf(&rx_buf);
  // Enable USART1 periphreal clock (low power and regular)
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->APB2LPENR |= RCC_APB2LPENR_USART1LPEN;

  // Disable USART1 before configuration
  USART1->CR1 &= ~(USART_CR1_UE);

  /*
   * Baud Rate assuming APB2 peripheral clock = 16 MHz
   * Baud Rate = 115200
   */
  USART1->BRR = (16000000U + 115200U / 2U) / 115200U;

  /*
   * CR1 Config:
   * 8 data bits
   * No Parity
   * RX Enable
   * TX Enable to send IMU Command
   * Enable RX Interrupts
   */

  // Enable TX, RX, sets OVER8 to 16 and Word length to 8
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE;

  // 1 stop bit, don't need other features in register
  USART1->CR2 = 0;

  // Nothing needed in CR3 either
  USART1->CR3 = 0;

  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_EnableIRQ(USART1_IRQn);
  USART1->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE;
}

void USART1_IRQHandler(void) {
  uint32_t status = USART1->SR;

  if (status & (USART_SR_RXNE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
    // Read DR after SR to clear receive/error flags, even without RXNE.
    uint8_t byte = (uint8_t)USART1->DR;
    if (!(status & USART_SR_RXNE)) {
      return;
    }
    enqueue(&rx_buf, byte);
    new_data = 1;
  }
}

void usart1_write_command(const char *command) {
  while (*command != '\0') {
    while (!(USART1->SR & USART_SR_TXE)) {
      // Wait until the transmit data reg is empty
      // Done while on ground so busy wait blocking is fine
    }
    USART1->DR = (uint8_t)*command++;
  }

  // Wait until final byte has finished leaving uart
  while (!(USART1->SR & USART_SR_TC)) {
  }
}
