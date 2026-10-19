#include "spi_board.h"

static void hardwareSPI1Init(void) {}

spi_t spi1 = {
    .regs = SPI1,
    .dma = DMA2,
    .tx_dma = DMA2_Stream3,
    .rx_dma = DMA2_Stream0,
    .tx_dma_irqn = DMA2_Stream3_IRQn,
    .rx_dma_irqn = DMA2_Stream0_IRQn,
    .irqn = SPI1_IRQn,
    .clock_hz = 25000000U,
    .target_hz = 6250000U,
    .enable_clock = spi_1_enable_clock,
    .init_gpio = hardwareSPI1Init,
};

void spi_board_init() { spi_init(&spi1); }