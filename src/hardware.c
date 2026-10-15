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

static volatile uint32_t g_ms = 0;

// Runs automatically every 1 ms. The name must match the startup file exactly.
void SysTick_Handler(void)
{
    g_ms++;
}

uint32_t millis(void)
{
    return g_ms;
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

  SystemCoreClockUpdate();                 // refresh the CMSIS clock-speed variable
  SysTick_Config(SystemCoreClock / 1000);  // interrupt every 1 ms
}

// The five state LED pins on port B: PB5, PB4, PB10, PB6, PB8
#define STATE_LED_MASK  ((1U << 5) | (1U << 4) | (1U << 10) | (1U << 6) | (1U << 8))

// Configure the five LED pins as outputs. Call once at startup.
void state_leds_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;      // turn on port B's clock

    // Each pin has 2 mode bits at position (pin * 2). Clear them, then set 01 = output.
    GPIOB->MODER &= ~((3U << (5*2)) | (3U << (4*2)) | (3U << (10*2)) | (3U << (6*2)) | (3U << (8*2)));
    GPIOB->MODER |=   (1U << (5*2)) | (1U << (4*2)) | (1U << (10*2)) | (1U << (6*2)) | (1U << (8*2));
}

// Drive all five LED pins HIGH.
void state_leds_all_on(void)
{
    GPIOB->BSRR = STATE_LED_MASK;             // lower 16 bits = set high
}

// Drive all five LED pins LOW.
void state_leds_all_off(void)
{
    GPIOB->BSRR = STATE_LED_MASK << 16;       // upper 16 bits = set low
}

// Light only the LED for the given state (0 = STANDBY ... 4 = LANDED).
void state_leds_show(uint8_t state)
{
    // Which pin belongs to each state, in the same order as the State enum
    static const uint8_t state_pin[5] = {5, 4, 10, 6, 8};

    if (state > 4) return;    // guard against a bad value indexing past the table

    GPIOB->BSRR = (STATE_LED_MASK << 16)          // all five LEDs off...
                | (1U << state_pin[state]);       // ...except this one on
}


