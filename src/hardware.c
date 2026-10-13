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

void timer_init(void)
{
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
        while (1) {
        }
    }

    NVIC_SetPriority(SysTick_IRQn, 15U);
}

void hardwareInit(void) {
  // Enable clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  timer_init();

  // -- LED --
  led2Init();
}


