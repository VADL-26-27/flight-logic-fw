#include "hardware.h"

void led2Init(void) {
  // configure PA5 for output push/pull
  GPIOA->MODER |= (1U << (5 * 2));

  // set push/pull
  GPIOA->OTYPER &= ~(1U << 5);

  // set PU/PD to off
  GPIOA->PUPDR &= ~(3U << (5 * 2));

  // initialize off
  GPIOA->BSRR = (1U << (16 + 5));
  // GPIOA->BSRR |= (1U << 5);
}

void timer_init(void) {
  SystemCoreClockUpdate();

  if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
    while (1) {
    }
  }

  NVIC_SetPriority(SysTick_IRQn, 15U);
}

void clock_init_hse(void) {
  RCC->CR &= ~RCC_CR_HSEBYP;
  RCC->CR |= RCC_CR_HSEON;

  uint32_t timeout = 1000000U;

  while (!(RCC->CR & RCC_CR_HSERDY)) {
    if (--timeout == 0U) {
      return; // SYSCLK Remains HSI
    }
  }

  // Select HSE as system clock
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;

  timeout = 1000000U;

  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) {
    if (--timeout == 0U) {
      return;
    }
  }

  SystemCoreClockUpdate();
}

void latch_init(void) {
  // Set PA8 (Latch_SW3) to low
  GPIOA->BSRR = (1U << (8 + 16));

  // Set PB4 (Latch_SW1) and PB5 (Latch_SW2) to low
  GPIOB->BSRR = (1U << (4 + 16)) | (1U << (5 + 16));

  // Push-Pull Output Type
  GPIOA->OTYPER &= ~(1U << 8);
  GPIOB->OTYPER &= ~((1U << 4) | (1U << 5));

  // No Internal pulls, external gate pull downs are present
  GPIOA->PUPDR &= ~(3U << (2 * 8));
  GPIOB->PUPDR &= ~((3U << (2 * 4)) | (3U << (2 * 5)));

  // Set output speed low
  GPIOA->OSPEEDR &= ~(3U << (2 * 8));
  GPIOB->OSPEEDR &= ~((3U << (2 * 4)) | (3U << (2 * 5)));

  // GPIO Output Mode
  GPIOA->MODER &= ~(3U << (2 * 8));
  GPIOA->MODER |= (1U << (2 * 8));

  GPIOB->MODER &= ~((3U << (2 * 4)) | (3U << (2 * 5)));
  GPIOB->MODER |= ((1U << (2 * 4)) | (1U << (2 * 5)));
}

void hardwareInit(void) {
  // Enable clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1LPENR |= RCC_AHB1LPENR_GPIOALPEN | RCC_AHB1LPENR_GPIOBLPEN;

  (void)RCC->AHB1ENR;
  (void)RCC->AHB1LPENR;

  clock_init_hse();

  timer_init();

  latch_init();
  // -- LED --
  led2Init();
}
