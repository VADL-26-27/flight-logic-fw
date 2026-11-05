#include "hardware.h"
static bool wait_register(volatile uint32_t *reg, uint32_t mask,
                          uint32_t expected) {
  uint32_t timeout = 1000000U;

  while ((*reg & mask) != expected) {
    if (--timeout == 0U) {
      return false;
    }
  }

  return true;
}

void led0Init(void) {
  GPIOA->BSRR = (1U << 0);          // high = LED off
  GPIOA->OTYPER &= ~(1U << 0);      // push-pull
  GPIOA->PUPDR &= ~(3U << (0 * 2)); // no pull
  GPIOA->MODER &= ~(3U << (0 * 2));
  GPIOA->MODER |= (1U << (0 * 2)); // output
}

void timer_init(void) {
  SystemCoreClockUpdate();

  if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
    while (1) {
    }
  }

  NVIC_SetPriority(SysTick_IRQn, 15U);
}

void clock_init(void) {

  // Require normal HSI/reset
  if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI ||
      (RCC->CR & RCC_CR_PLLON)) {
    return;
  }

  // Enable External Crystal Oscillator
  RCC->CR &= ~RCC_CR_HSEBYP;
  RCC->CR |= RCC_CR_HSEON;

  // Wait for the Register
  if (!wait_register(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY))
    return;

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;

  // Voltage Scale 1: VOS = 11
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0 | PWR_CR_VOS_1;
  (void)PWR->CR;

  // Three wait states for 96 MHz at 3.3 V.
  FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_3WS;

  if (!wait_register(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS))
    return;

  // HCLK = SYSCLK /1.
  // PCLK1 = HCLK /2; PCLK2 = HCLK /2.
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
              RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;

  // PLLM = 25, PLLN = 200, PLLP = 2, PLLQ = 4.
  // PLLP encoding 00 means divide by 2.
  RCC->PLLCFGR =
      (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP |
                        RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ)) |
      25U | (200U << 6) | RCC_PLLCFGR_PLLSRC_HSE | (4U << 24);

  RCC->CR |= RCC_CR_PLLON;

  if (!wait_register(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY))
    return;

  if (!wait_register(&PWR->CSR, PWR_CSR_VOSRDY, PWR_CSR_VOSRDY))
    return;

  // Switch SYSCLK from HSI to PLL.
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;

  if (!wait_register(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL))
    return;

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

void enable_12V(void) {
  GPIOC->BSRR = (1U << (13 + 16));
  GPIOC->OTYPER &= ~(1U << 13);
  GPIOC->PUPDR &= ~(3U << (2 * 13));
  GPIOC->OSPEEDR &= ~(3U << (2 * 13));

  GPIOC->MODER &= ~(3U << (2 * 13));
  GPIOC->MODER |= (1U << (2 * 13));

  // Enable, assuming active-high
  GPIOC->BSRR = 1U << 13;
}

void hardwareInit(void) {
  // Enable clock
  RCC->AHB1ENR |=
      RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1LPENR |= RCC_AHB1LPENR_GPIOALPEN | RCC_AHB1LPENR_GPIOBLPEN |
                    RCC_AHB1LPENR_GPIOCLPEN;

  (void)RCC->AHB1ENR;
  (void)RCC->AHB1LPENR;

  clock_init();

  timer_init();
  enable_12V();
  latch_init();
  led0Init();
}