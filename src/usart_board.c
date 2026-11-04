#include "usart_board.h"

// Each UART gets separate RX and TX buffers.
static ring_buf uart1_rx;
static ring_buf uart1_tx;
static ring_buf uart2_rx;
static ring_buf uart2_tx;

static void hardwareUsart2Init(void) {
  // Clear PA2/PA3, Make PA2/PA3 Alternate Function Mode
  GPIOA->MODER &= ~((3U << 2 * 2) | (3U << 3 * 2));
  GPIOA->MODER |= ((2U << 2 * 2) | (2U << 3 * 2));

  // Clear PA2/PA3 Alternate Functions, Make PA2/PA3 Alternate Functions USART2
  // (AF7)
  GPIOA->AFR[0] &= ~((0xFU << 2 * 4) | (0xFU << 3 * 4));
  GPIOA->AFR[0] |= ((7U << 2 * 4) | (7U << 3 * 4));

  // High Speed for PA2/PA3
  GPIOA->OSPEEDR |= ((3U << 2 * 2) | (3U << 3 * 2));

  // Push PULL TX (PA2)
  GPIOA->OTYPER &= ~(1U << 2);

  // No pull up, no pull down
  GPIOA->PUPDR &= ~((3U << 2 * 2) | (3U << 3 * 2));
}

static void hardwareUsart1Init(void) {
  // USART1 IMU: PA9 TX and PA10 RX, alternate function 7.
  GPIOA->MODER &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
  GPIOA->MODER |= (2U << (9 * 2)) | (2U << (10 * 2));

  GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
  GPIOA->AFR[1] |= (7U << 4) | (7U << 8);

  GPIOA->OSPEEDR |= (3U << (9 * 2)) | (3U << (10 * 2));

  GPIOA->OTYPER &= ~(1U << 9);

  GPIOA->PUPDR &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
}

// The callback functions must be declared before these initializers.
usart_t uart1 = {
    .regs = USART1,
    .irqn = USART1_IRQn,
    .clock_hz = 25000000U,
    .baud_rate = 115200U,
    .rx = &uart1_rx,
    .tx = &uart1_tx,
    .enable_clock = usart_1_enable_clock,
    .init_gpio = hardwareUsart1Init,
};

usart_t uart2 = {
    .regs = USART2,
    .irqn = USART2_IRQn,
    .clock_hz = 25000000U,
    .baud_rate = 115200U,
    .rx = &uart2_rx,
    .tx = &uart2_tx,
    .enable_clock = usart_2_enable_clock,
    .init_gpio = hardwareUsart2Init,
};

void board_usart_init(void) {
  usart_init(&uart1);
  usart_init(&uart2);
}