/* Lab 3A - External Interrupt on PC13 (User Button) - No HAL
 * PC13 = B1 (User Button, active LOW)
 * Toggles PA5 (LD2) on each falling edge
 */

#include "stm32f446xx.h"

/* Interrupt Service Routine for EXTI lines 15 to 10 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1UL << 13))      // Check if line 13 interrupt pending
    {
        EXTI->PR = (1UL << 13);      // Clear pending bit (write 1 to clear)
        GPIOA->ODR ^= (1UL << 5);    // Toggle PA5 LED
    }
}

int main(void)
{
    /* 1. Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   // Enable SYSCFG for EXTI

    __NOP();
    __NOP();

    /* 2. PA5 as push-pull output (LED) */
    GPIOA->MODER &= ~(3UL << 10);
    GPIOA->MODER |=  (1UL << 10);

    /* 3. PC13 as input (User Button) */
    GPIOC->MODER &= ~(3UL << 26);   // Clear to input mode

    /* 4. Route PC13 to EXTI line 13 */
    SYSCFG->EXTICR[3] &= ~(0xFUL << 4);
    SYSCFG->EXTICR[3] |=  (0x2UL << 4);   // 0x2 = Port C

    /* 5. Configure EXTI line 13 */
    EXTI->IMR  |= (1UL << 13);   // Unmask interrupt
    EXTI->RTSR &= ~(1UL << 13);  // Disable rising edge trigger
    EXTI->FTSR |= (1UL << 13);   // Enable falling edge trigger

    /* 6. Enable interrupt in NVIC */
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    while (1)
    {
        __WFI();   // Wait For Interrupt (sleep until interrupt occurs)
    }
}
