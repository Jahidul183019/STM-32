#define STM32F446xx
#include "stm32f4xx.h"
#include <stdio.h>

/* =========================================================
 * Constants & Global Variables
 * ========================================================= */
#define TIM3_ARR  999U

volatile uint32_t ms_count = 0;

static const uint8_t sine_lut[256] = {
     50, 51, 52, 54, 55, 56, 57, 59, 60, 61, 62, 64, 65, 66, 67, 68,
     70, 71, 72, 73, 74, 75, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86,
     87, 88, 89, 89, 90, 91, 92, 93, 93, 94, 95, 95, 96, 97, 97, 98,
     98, 99, 99, 99,100,100,100,100,100,100,100,100,100,100, 99, 99,
     99, 98, 98, 97, 97, 96, 95, 95, 94, 93, 93, 92, 91, 90, 89, 89,
     88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 75, 74, 73, 72,
     71, 70, 68, 67, 66, 65, 64, 62, 61, 60, 59, 57, 56, 55, 54, 52,
     51, 50, 48, 47, 46, 44, 43, 42, 41, 39, 38, 37, 36, 34, 33, 32,
     31, 30, 28, 27, 26, 25, 24, 23, 21, 20, 19, 18, 17, 16, 15, 14,
     13, 12, 11, 10,  9,  9,  8,  7,  6,  6,  5,  4,  4,  3,  3,  2,
      2,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,
      1,  2,  2,  3,  3,  4,  4,  5,  6,  6,  7,  8,  9,  9, 10, 11,
     12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 24, 25, 26, 27, 28,
     30, 31, 32, 33, 34, 36, 37, 38, 39, 41, 42, 43, 44, 46, 47, 48,
     49, 50, 51, 52, 53, 55, 56, 57, 58, 60, 61, 62, 63, 64, 66, 67,
     68, 69, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 50
};

/* =========================================================
 * System Clock — Forced to 16 MHz (HSI)
 * ========================================================= */
void SystemClock_Config_16MHz(void) {
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    RCC->CR &= ~RCC_CR_PLLON; /* Disable PLL safely */
}

/* =========================================================
 * TIM2 — 1 ms interrupt for delay_ms() (16 MHz Clock)
 * ========================================================= */
void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __NOP(); __NOP();

    TIM2->CR1  &= ~TIM_CR1_CEN;
    TIM2->PSC   = 15U;       /* 16 MHz / 16 = 1 MHz (1 µs tick) */
    TIM2->ARR   = 999U;      /* 1000 µs = 1 ms period */
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->SR    = 0U;
    TIM2->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM2_IRQn, 0U);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        ms_count++;
    }
}

void delay_ms(uint32_t ms) {
    uint32_t start = ms_count;
    while ((ms_count - start) < ms) {}
}

/* =========================================================
 * USART2 — 115200 Baud @ 16 MHz
 * ========================================================= */
void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 (TX) - Alternate Function 7 */
    GPIOA->MODER  &= ~(3U << (2*2));
    GPIOA->MODER  |=  (2U << (2*2));
    GPIOA->AFR[0] &= ~(0xFU << (4*2));
    GPIOA->AFR[0] |=  (7U   << (4*2));

    /* 16 MHz / (16 * 115200) = 8.6805 -> Mantissa=8, Fraction=11 -> BRR = 0x8B */
    USART2->BRR = (8U << 4) | 11U;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART2_SendString(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC)) {} /* Wait for transmission complete */
}

/* =========================================================
 * TIM3 PWM — Channel 1 on PA6 @ 1 kHz (16 MHz Clock)
 * ========================================================= */
void TIM3_PWM_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    __NOP(); __NOP();

    /* Configure PA6 to AF2 (TIM3_CH1) with push-pull & high speed */
    GPIOA->MODER   &= ~(3U << (6*2));
    GPIOA->MODER   |=  (2U << (6*2));
    GPIOA->OTYPER  &= ~(1U << 6);        /* Push-pull */
    GPIOA->OSPEEDR |=  (3U << (6*2));    /* High speed */
    GPIOA->PUPDR   &= ~(3U << (6*2));    /* No pull */

    GPIOA->AFR[0]  &= ~(0xFU << (4*6));
    GPIOA->AFR[0]  |=  (2U   << (4*6));

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->PSC  = 15U;       /* 16 MHz / 16 = 1 MHz tick */
    TIM3->ARR  = TIM3_ARR;  /* 1000 µs = 1 kHz frequency */

    /* PWM Mode 1 and Preload Enable */
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |=  TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM3->CCMR1 |=  TIM_CCMR1_OC1PE;

    TIM3->CCR1 = 0U;

    TIM3->CCER |=  TIM_CCER_CC1E;
    TIM3->CCER &= ~TIM_CCER_CC1P;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR  = TIM_EGR_UG;
    TIM3->SR   = 0U;

    TIM3->CR1 |= TIM_CR1_CEN;
}

void PWM_SetDuty(uint8_t pct) {
    if (pct > 100U) pct = 100U;
    TIM3->CCR1 = (uint32_t)pct * (TIM3_ARR + 1U) / 100U;
}

/* =========================================================
 * Main Execution
 * ========================================================= */
int main(void) {
    SystemClock_Config_16MHz();
    TIM2_Init();
    USART2_Init();
    TIM3_PWM_Init();

    char buf[100];

    USART2_SendString("* Lab-02 | Task 4: TIM3 PWM Generation ***\r\n");
    USART2_SendString("-------------------------------------------\r\n\r\n");

    /* --- Part 1: Duty-Cycle Sweep --- */
    USART2_SendString("[1] Duty-Cycle Sweep (0% to 100%, step 10%):\r\n");
    USART2_SendString("    Duty  |  CCR1\r\n");
    USART2_SendString("    ------|------\r\n");

    for (int d = 0; d <= 100; d += 10) {
        PWM_SetDuty((uint8_t)d);
        snprintf(buf, sizeof(buf), "    %3d%%  |  %lu\r\n", d, (unsigned long)TIM3->CCR1);
        USART2_SendString(buf);
        delay_ms(300U);
    }
    USART2_SendString("[1] Sweep complete.\r\n\r\n");

    /* --- Part 2: Sine-wave breathing effect --- */
    USART2_SendString("[2] Sine-wave breathing effect: 5 cycles (~2s each)\r\n");
    for (int cycle = 1; cycle <= 5; cycle++) {
        snprintf(buf, sizeof(buf), "    Breath cycle %d / 5 ...\r\n", cycle);
        USART2_SendString(buf);
        for (int i = 0; i < 256; i++) {
            PWM_SetDuty(sine_lut[i]);
            delay_ms(8U);
        }
    }
    USART2_SendString("[2] Breathing sequence complete.\r\n\r\n");

    /* --- Part 3: Final Hold --- */
    PWM_SetDuty(50U);
    snprintf(buf, sizeof(buf), "[3] Final hold: Duty = 50%%  |  CCR1 = %lu\r\n\r\n", (unsigned long)TIM3->CCR1);
    USART2_SendString(buf);

    USART2_SendString("* All demonstrations complete. ***\r\n");

    while (1) {}
}
