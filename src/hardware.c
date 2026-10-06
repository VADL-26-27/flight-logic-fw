#include "hardware.h"

void hardwareUsart2Init(void) {
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

void hardwareUsart1Init(void) {
  // USART1 IMU: PA9 TX and PA10 RX, alternate function 7.
  GPIOA->MODER &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
  GPIOA->MODER |= (2U << (9 * 2)) | (2U << (10 * 2));

  GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
  GPIOA->AFR[1] |= (7U << 4) | (7U << 8);

  GPIOA->OSPEEDR |= (3U << (9 * 2)) | (3U << (10 * 2));

  GPIOA->OTYPER &= ~(1U << 9);

  GPIOA->PUPDR &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
}

void led2Init(void) {
  // configure PA5 for output push/pull
  GPIOA->MODER |= (1U << (5 * 2));

  // set push/pull
  GPIOA->OTYPER &= ~(1U << 5);

  // set PU/PD to off
  GPIOA->PUPDR &= ~(3U << (5 * 2));

  // initialize off
  GPIOA->BSRR |= (1U << (16 + 5));
  // GPIOA->BSRR |= (1U << 5);
}

void hardwareInit(void) {
  // Enable clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // -- USART2 --
  hardwareUsart2Init();

  // -- USART 1 --
  hardwareUsart1Init();

  // -- LED --
  led2Init();
}
