#include "spi.h"

void spi_1_enable_clock() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->APB2LPENR |= RCC_APB2LPENR_SPI1LPEN;
  (void)RCC->AHB1ENR;
  (void)RCC->AHB1LPENR;
}

static void spi_dma_init(spi_t *port) {
  // Enable DMA2 Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  RCC->AHB1LPENR |= RCC_AHB1LPENR_DMA2LPEN;

  (void)RCC->AHB1ENR;
  (void)RCC->AHB1LPENR;

  // Request Disable
  port->rx_dma->CR &= ~DMA_SxCR_EN;
  port->tx_dma->CR &= ~DMA_SxCR_EN;

  while (port->rx_dma->CR & DMA_SxCR_EN) {
  }
  while (port->tx_dma->CR & DMA_SxCR_EN) {
  }

  // Channel 3; byte-size transfers; normal mode
  // Periphreal increment disabled
  // Implied DIR = 00 (periphreal to memory)
  port->rx_dma->CR = (3U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                     DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  port->tx_dma->CR = (3U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
                     DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE |
                     DMA_SxCR_DIR_0;

  // Direct mode: FIFO disabled.
  port->rx_dma->FCR = 0;
  port->tx_dma->FCR = 0;

  // Both directions use SPI's data register.
  port->rx_dma->PAR = (uint32_t)&port->regs->DR;
  port->tx_dma->PAR = (uint32_t)&port->regs->DR;

  // Clear pending DMA interrupt flags for Streams 0 and 3.
  port->dma->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
                     DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0 | DMA_LIFCR_CFEIF3 |
                     DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 |
                     DMA_LIFCR_CTCIF3;

  NVIC_ClearPendingIRQ(port->rx_dma_irqn);
  NVIC_ClearPendingIRQ(port->tx_dma_irqn);

  NVIC_EnableIRQ(port->rx_dma_irqn);
  NVIC_EnableIRQ(port->tx_dma_irqn);
}

void spi_init(spi_t *port) {
  port->enable_clock();
  port->init_gpio();

  // Mode 0, MSB-first bit order, 8-bit frames.
  port->regs->CR1 = 0;
  port->regs->CR2 = 0;
  port->regs->I2SCFGR = 0; // Selects SPI rather than I2S

  // Configure Baud Rate for divisor to be /16
  port->regs->CR1 |=
      SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_BR_0;

  spi_dma_init(port);

  NVIC_ClearPendingIRQ(port->irqn);
  NVIC_EnableIRQ(port->irqn);

  // No RXNEIE or TXEIE since we are using DMA
  port->regs->CR2 |= SPI_CR2_ERRIE;

  port->regs->CR1 |= SPI_CR1_SPE;
}