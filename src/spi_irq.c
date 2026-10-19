#include "spi.h"
#include "spi_board.h"

void SPI1_IRQHandler(void) { spi_handle_irq(&spi1); }
