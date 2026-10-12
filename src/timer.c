#include "timer.h"

static volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void)
{
    ++ms_ticks;
}

uint32_t now_ms(void)
{
    return ms_ticks;
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

void delay_ms(uint32_t duration)
{
    uint32_t start = now_ms();

    while ((uint32_t)(now_ms() - start) < duration) {
        // SysTick and UART interrupts continue running.
    }
}