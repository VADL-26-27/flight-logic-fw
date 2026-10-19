#ifndef SPI_H_
#define SPI_H_

#include "stm32f413xx.h"
#include <stdbool.h>

typedef struct spi spi_t;

/* This current driver assume that the SPI driver uses a DMA stream for both
   MISO and MOSI, as there is no other device that requires use of the DMA,
   thus, using it for flight logging is good use of periphreals
 */
struct spi {
  SPI_TypeDef *regs;

  DMA_TypeDef *dma;
  DMA_Stream_TypeDef *tx_dma;
  DMA_Stream_TypeDef *rx_dma;

  IRQn_Type tx_dma_irqn;
  IRQn_Type rx_dma_irqn;
  IRQn_Type irqn;

  uint32_t clock_hz;
  uint32_t target_hz;

  volatile bool dma_done;
  volatile bool error;
  bool busy;

  void (*enable_clock)(void);
  void (*init_gpio)(void);
};

void spi_init(spi_t *port);
bool spi_transfer_dma(spi_t *port, const uint8_t *tx, uint8_t *rx,
                      uint32_t len);

void spi_handle_irq(spi_t *port);
void spi_handle_tx_dma_irq(spi_t *port);
void spi_handle_rx_dma_irq(spi_t *port);
void spi_poll(spi_t *port);

void spi_1_enable_clock();
#endif // SPI_H_