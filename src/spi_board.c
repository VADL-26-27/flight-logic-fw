#include "spi_board.h"

static void hardwareSPI1Init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  (void)RCC->AHB1ENR;

  // Pre-load CS high before making output
  GPIOA->BSRR = (1U << 4);

  // Push-Pull on PA4/5/7, PA6 is irrelevant as it is MISO
  GPIOA->OTYPER &= ~((1U << 4) | (1U << 5) | (1U << 7));

  // CS (PA4) Pull up, SCLK (PA5) pull down, none for MISO/MOSI (PA6/7)
  GPIOA->PUPDR &=
      ~((3U << 4 * 2) | (3U << 5 * 2) | (3U << 6 * 2) | (3U << 7 * 2));
  GPIOA->PUPDR |= ((1U << 4 * 2) | (2U << 5 * 2));

  // Clear PA5/6/7 alternate function, then set to SPI1 (AF5) (PA4 keep as GPIO,
  // as CS needs to be GPIO)
  GPIOA->AFR[0] &=
      ~((0xFU << 4 * 4) | (0xFU << 5 * 4) | (0xFU << 6 * 4) | (0xFU << 7 * 4));
  GPIOA->AFR[0] |= ((5U << 5 * 4) | (5U << 6 * 4) | (5U << 7 * 4));

  // Medium Speed for PA4/5/7 (None for PA6 doesn't need it since it is MISO)
  GPIOA->OSPEEDR &=
      ~((3U << 4 * 2) | (3U << 5 * 2) | (3U << 6 * 2) | (3U << 7 * 2));
  GPIOA->OSPEEDR |= ((1U << 4 * 2) | (1U << 5 * 2) | (1U << 7 * 2));

  // Clear PA4/5/6/7 and then set them to alternate function, keep PA4 (CS) as
  // GPIO
  GPIOA->MODER &=
      ~((3U << 4 * 2) | (3U << 5 * 2) | (3U << 6 * 2) | (3U << 7 * 2));
  GPIOA->MODER |=
      ((1U << 4 * 2) | (2U << 5 * 2) | (2U << 6 * 2) | (2U << 7 * 2));
}

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