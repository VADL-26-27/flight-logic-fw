#include "usart2.h"

ring_buf tx_buf;

void usart2_init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB1LPENR |= RCC_APB1LPENR_USART2LPEN;

  USART2->CR1 = 0;

  init_buf(&tx_buf);
  // 115200 baud with a 16 MHz APB1 clock, 8 data bits, no parity.
  USART2->BRR = (16000000U + 115200U / 2U) / 115200U;

  USART2->CR2 = 0;
  USART2->CR3 = 0;

  NVIC_ClearPendingIRQ(USART2_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);

  // Transmit only. TXEIE is enabled only while data is queued.
  USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void USART2_IRQHandler(void) {
  uint32_t status = USART2->SR;

  if ((status & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
    if (atomic_load_explicit(&tx_buf.count, memory_order_acquire) != 0U) {
      uint8_t byte;
      read_from_buf(&tx_buf, &byte, 1U);
      USART2->DR = byte;
    }

    // Disable TX interrupts when the buffer is empty.
    if (atomic_load_explicit(&tx_buf.count, memory_order_acquire) == 0U) {
      USART2->CR1 &= ~USART_CR1_TXEIE;
    }
  }
}

static bool queue_bytes(const uint8_t *data, uint32_t length) {
  // Serialize queue updates with the handler and preserve interrupt state.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t free_bytes = BUFFER_SIZE -
      atomic_load_explicit(&tx_buf.count, memory_order_acquire);
  if (length > free_bytes) {
    __set_PRIMASK(primask);
    return false;
  }
  for (uint32_t i = 0; i < length; ++i) {
    enqueue(&tx_buf, data[i]);
  }
  if (length != 0U) {
    USART2->CR1 |= USART_CR1_TXEIE;
  }
  __set_PRIMASK(primask);
  return true;
}

bool usart2_write_packet(const uint8_t packet[USART2_PACKET_SIZE]) {
  return queue_bytes(packet, USART2_PACKET_SIZE);
}
