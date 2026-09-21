#include "stm32f4xx.h"
#include "stm32f411xe.h"

void init_led2(void) {

    // turn on AHB1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // configure PA5 for output push/pull
    GPIOA->MODER |= (1U << (5 * 2));

    // set push/pull
    GPIOA->OTYPER &= ~(1U << 5);

    // set PU/PD to off
    GPIOA->PUPDR &= ~(3U << (5 * 2));

    // initialize off
    GPIOA->BSRR |= (1U << (16 + 5));
}

int main(void)
{
    init_led2();
    GPIOA->BSRR |= (1U << 5);
    while (1)
    {

        
    }
}


