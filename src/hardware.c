#include "hardware.h"

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

void hardwareInit(void) {
  // Enable clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  clock_init_hse();

  timer_init();

  // -- LED --
  led2Init();
}
