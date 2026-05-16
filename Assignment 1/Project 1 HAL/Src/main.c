/* Lab 1A - LED Blink (No HAL) on STM32F446RE Nucleo
 * PA5 = LD2 (User LED)
 * Clock source: HSI 16 MHz (default after reset)
 */

#include "stm32f446xx.h"

void delay_ms(uint32_t ms)
{
    /* Software delay: ~16 MHz HSI */
    for (uint32_t i = 0; i < ms * 4000; i++)
    {
        __NOP();
    }
}

int main(void)
{
    /* 1. Enable GPIOA clock on AHB1 bus */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __NOP();
    __NOP();   /* Brief settle delay */

    /* 2. Set PA5 as General Purpose Output (MODER[11:10] = 01) */
    GPIOA->MODER &= ~(3UL << (5 * 2));   // Clear bits
    GPIOA->MODER |=  (1UL << (5 * 2));   // Output mode

    /* 3. Output type: Push-Pull */
    GPIOA->OTYPER &= ~(1UL << 5);

    /* 4. Output speed: Low */
    GPIOA->OSPEEDR &= ~(3UL << (5 * 2));

    /* 5. No Pull-up / Pull-down */
    GPIOA->PUPDR &= ~(3UL << (5 * 2));

    while (1)
    {
        GPIOA->BSRR = (1UL << 5);          // LED ON
        delay_ms(500);

        GPIOA->BSRR = (1UL << (5 + 16));   // LED OFF
        delay_ms(500);
    }
}
