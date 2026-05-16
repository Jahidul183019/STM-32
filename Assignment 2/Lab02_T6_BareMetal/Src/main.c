/*
 * ============================================================
 * CSE 2206 — Microcontroller & Embedded System Lab-02
 * Task 6 — Option A: Passive Buzzer Tone Generation (TIM4 PWM)
 * Platform : STM32F446RE Nucleo-64
 * Clock    : SYSCLK = 180 MHz, APB1 = 45 MHz, TIM4_CLK = 90 MHz
 * Melody   : Nokia Tune
 * ============================================================
 *
 * Hardware: Passive buzzer connected to PB6 (TIM4_CH1, AF2).
 * Tone generation by varying TIM4 ARR; 50% duty square wave.
 */

#include <stm32f446xx.h>   /* CMSIS device header — all RCC/GPIO/TIM structs */
#include <stdint.h>
#include <stdio.h>

/* =========================================================
 * SECTION 0 — System Clock Configuration
 * HSI=16 MHz → PLL → SYSCLK=180 MHz
 * AHB /1=180 MHz, APB1 /4=45 MHz, APB2 /2=90 MHz
 * TIM4_CLK = TIM2_CLK = 2 × APB1 = 90 MHz (APB1 prescaler ≠ 1)
 * ========================================================= */

void SystemClock_Config(void)
{
    /* 1. Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* 2. Voltage scaling (Scale 1 mode) */
    PWR->CR |= PWR_CR_VOS;

    /* 3. Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* 4. Configure FLASH latency and enable caches */
    FLASH->ACR  = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |=  FLASH_ACR_LATENCY_5WS;

    /* 5. Configure PLL
       HSI = 16 MHz
       PLLM = 8
       PLLN = 180
       PLLP = 2
       PLLQ = 4
    */
    RCC->PLLCFGR  = 0;
    RCC->PLLCFGR |= (8U   << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (180U << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0U   << RCC_PLLCFGR_PLLP_Pos);   /* PLLP = 2 */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR |= (4U   << RCC_PLLCFGR_PLLQ_Pos);

    /* 6. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 7. Enable OverDrive mode */
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));

    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* 8. Configure Bus Prescalers
       AHB  = SYSCLK /1
       APB1 = HCLK   /4
       APB2 = HCLK   /2
    */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* 9. Select PLL as system clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* =========================================================
 * SECTION 1 — USART2
 * PA2=TX, 115200 8N1, APB1 clock = 45 MHz
 * Used for printing scale/melody tables to TeraTerm.
 * ========================================================= */

/**
 * @brief  Initialise USART2 for 115200 8N1.
 *         PA2 → TX (AF7).
 */
static void USART2_Init(void)
{
    /* 1. Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* GPIOA clock  */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   /* USART2 clock */

    /* 2. PA2: AF7 (USART2_TX) */
    GPIOA->MODER  &= ~(3U << (2*2));
    GPIOA->MODER  |=  (2U << (2*2));         /* Alternate function */
    GPIOA->AFR[0] &= ~(0xFU << (4*2));
    GPIOA->AFR[0] |=  (7U   << (4*2));       /* AF7 = USART2 */

    /* 3. Baud rate calculation
       fCK  = 45 MHz
       Baud = 115200
       USARTDIV = fCK / (16 × Baud) = 24.414
       Mantissa = 24,  Fraction ≈ 7
    */
    USART2->BRR = (24U << 4) | 7U;           /* 0x187 */

    /* 4. Enable TX and the peripheral */
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

/**
 * @brief  Transmit a null-terminated string over USART2.
 * @param  s  Pointer to string (must be null-terminated).
 */
static void USART2_SendString(const char *s)
{
    while (*s)
    {
        while (!(USART2->SR & USART_SR_TXE)) {}   /* Wait TX empty */
        USART2->DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC)) {}         /* Wait TX complete */
}

/* =========================================================
 * SECTION 2 — TIM2: 1 ms System Tick (Interrupt-driven)
 *
 * fTIM2_CLK = 90 MHz  (2 × APB1 because APB1 prescaler ≠ 1)
 * PSC = 89   → tick = 90 MHz / (89+1) = 1 MHz  → 1 µs per tick
 * ARR = 999  → update event every 1000 µs = 1 ms
 * UIE = 1    → ISR fires every 1 ms and increments ms_count
 * ========================================================= */

volatile uint32_t ms_count = 0;   /* Free-running ms counter (set by ISR) */

/**
 * @brief  Initialise TIM2 to generate a 1 ms periodic interrupt.
 *         Increments the global ms_count, used by delay_ms().
 */
static void TIM2_Init(void)
{
    /* Step 1: Enable TIM2 peripheral clock via RCC APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Step 2: Short NOP delay for clock stabilisation */
    __NOP(); __NOP();

    /* Step 3: Disable counter before configuring */
    TIM2->CR1 &= ~TIM_CR1_CEN;

    /* Step 4: Prescaler — 1 µs resolution */
    TIM2->PSC = 89U;

    /* Step 5: Auto-reload — 1 ms period */
    TIM2->ARR = 999U;

    /* Step 6: Force immediate register update (shadow registers loaded) */
    TIM2->EGR = TIM_EGR_UG;

    /* Step 7: Clear all status flags */
    TIM2->SR = 0U;

    /* Step 8: Enable update-event interrupt */
    TIM2->DIER |= TIM_DIER_UIE;

    /* Step 9: Configure NVIC — highest priority */
    NVIC_SetPriority(TIM2_IRQn, 0U);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Step 10: Start the counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief  TIM2 update-event ISR. Fires every 1 ms.
 *         Clears UIF and bumps the millisecond counter.
 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;     /* Clear update flag */
        ms_count++;
    }
}

/**
 * @brief  Millisecond blocking delay using the TIM2 ISR tick.
 * @param  ms  Delay in milliseconds.
 *
 * Note: uses unsigned subtraction so it survives ms_count wrap-around.
 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_count;
    while ((ms_count - start) < ms) {}
}

/* =========================================================
 * SECTION 3 — TIM4 PWM: Buzzer Driver
 *
 * Output  : PB6 → TIM4_CH1, AF2
 * fTIM4_CLK = 90 MHz (same as TIM2)
 * PSC = 99    → tick = 90 MHz / 100 = 900 kHz
 * ARR formula: ARR = (900000 / freq_Hz) - 1
 * Duty cycle : CCR1 = ARR/2  → 50% square wave (tone)
 * Mode       : PWM Mode 1 (OC1M = 110), preload enabled.
 * ========================================================= */

#define TIM4_PSC  99U     /* 900 kHz tick */

/**
 * @brief  Initialise TIM4 CH1 PWM on PB6 for buzzer tone output.
 *         Sets up the timer but leaves CCR1 = 0 (silent until note plays).
 */
static void TIM4_PWM_Init(void)
{
    /* Step 1: Enable GPIOB and TIM4 clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    __NOP(); __NOP();

    /* Step 2: PB6 → Alternate Function mode, AF2 (TIM4_CH1) */
    GPIOB->MODER  &= ~(3U << (6*2));
    GPIOB->MODER  |=  (2U << (6*2));          /* AF mode */
    GPIOB->OTYPER  &= ~(1U << 6);             /* Push-pull */
    GPIOB->OSPEEDR |=  (3U << (6*2));         /* Very high speed */
    GPIOB->PUPDR   &= ~(3U << (6*2));         /* No pull */

    GPIOB->AFR[0] &= ~(0xFU << (4*6));
    GPIOB->AFR[0] |=  (2U   << (4*6));        /* AF2 = TIM4 */

    /* Step 3: Disable counter before configuring */
    TIM4->CR1 &= ~TIM_CR1_CEN;

    /* Step 4: Prescaler — 900 kHz tick */
    TIM4->PSC = TIM4_PSC;

    /* Step 5: PWM Mode 1 on CH1 (OC1M = 110) + output preload */
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM4->CCMR1 |=  TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM4->CCMR1 |=  TIM_CCMR1_OC1PE;

    /* Step 6: Enable CH1 output, active-high polarity */
    TIM4->CCER |=  TIM_CCER_CC1E;
    TIM4->CCER &= ~TIM_CCER_CC1P;

    /* Step 7: Auto-reload preload (ARR is buffered) */
    TIM4->CR1  |=  TIM_CR1_ARPE;
}

/**
 * @brief  Play a single tone for a given duration.
 * @param  arr          ARR value selecting the note frequency.
 * @param  duration_ms  Time the tone is held (ms).
 */
static void TIM4_PlayNote(uint32_t arr, uint32_t duration_ms)
{
    /* Step 1: Stop counter while updating ARR/CCR1 */
    TIM4->CR1 &= ~TIM_CR1_CEN;

    /* Step 2: Load period and 50% duty */
    TIM4->ARR  = arr;
    TIM4->CCR1 = arr / 2U;

    /* Step 3: Latch new values into shadow registers */
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->SR   = 0U;

    /* Step 4: Start the tone */
    TIM4->CR1 |= TIM_CR1_CEN;

    /* Step 5: Hold for the requested duration */
    delay_ms(duration_ms);

    /* Step 6: Silence the buzzer (stop counter, drop duty to 0) */
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CCR1 = 0U;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->SR   = 0U;
}

/**
 * @brief  Insert a silent rest between notes.
 * @param  duration_ms  Length of the rest (ms).
 */
static void REST(uint32_t duration_ms)
{
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CCR1 = 0U;
    delay_ms(duration_ms);
}

/* =========================================================
 * SECTION 4 — Note Data Tables
 *
 * ARR formula reminder:  ARR = (900000 / freq_Hz) - 1   (PSC = 99)
 *
 *   Note   Freq(Hz)   ARR
 *   E5     659        1364
 *   D5     587        1532
 *   C5     523        1717
 *   B4     494        1821
 *   A4     440        2044
 *   G4     392        2295
 *   F#4    370        2431
 *   F4     349        2572
 *   E4     330        2727
 *   D4     294        3061
 *   C4     262        3436
 *   A3     220        4090
 * ========================================================= */

/* C-Major Scale (Part 1) */
typedef struct {
    const char *name;
    uint32_t    arr;
} ScaleNote;

static const ScaleNote scale[] = {
    { "C4", 3436 },
    { "D4", 3061 },
    { "E4", 2727 },
    { "F4", 2572 },
    { "G4", 2295 },
    { "A4", 2044 },
    { "B4", 1821 },
    { "C5", 1717 },
};
#define SCALE_LEN  (sizeof(scale) / sizeof(scale[0]))

/* Nokia Tune (Part 2) — 13 notes */
typedef struct {
    const char *name;
    uint32_t    arr;
    uint32_t    dur;   /* duration in ms */
} Note;

static const Note melody[] = {
    { "E5", 1364, 150 },
    { "D5", 1532, 150 },
    { "F4", 2572, 300 },   /* F#4 approximated as F4 */
    { "G4", 2295, 300 },
    { "C5", 1717, 150 },
    { "B4", 1821, 150 },
    { "D4", 3061, 300 },
    { "E4", 2727, 300 },
    { "B4", 1821, 150 },
    { "A4", 2044, 150 },
    { "C4", 3436, 300 },
    { "E4", 2727, 300 },
    { "A4", 2044, 600 },
};
#define MELODY_LEN  (sizeof(melody) / sizeof(melody[0]))

/* =========================================================
 * SECTION 5 — Main Demonstration
 *   Part 1: Walk the C-Major scale (8 notes, 400 ms each).
 *   Part 2: Play the Nokia Tune (13 notes, variable length).
 * ========================================================= */

int main(void)
{
    char buf[80];

    /* Initialise peripherals */
    SystemClock_Config();
    USART2_Init();
    TIM2_Init();
    TIM4_PWM_Init();

    USART2_SendString("\r\n==========================================\r\n");
    USART2_SendString("  Task 6A: Buzzer Tone - STM32F446RE\r\n");
    USART2_SendString("  Melody: Nokia Tune\r\n");
    USART2_SendString("  PB6, TIM4 CH1 (AF2), PSC=99\r\n");
    USART2_SendString("==========================================\r\n");

    /* --- Part 1: C-Major Scale --- */
    USART2_SendString("\r\n-- Part 1: C-Major Scale --\r\n");
    USART2_SendString("Note  |  ARR  |  Actual Freq (Hz)\r\n");
    USART2_SendString("------|-------|------------------\r\n");

    for (uint8_t i = 0; i < SCALE_LEN; i++)
    {
        uint32_t arr      = scale[i].arr;
        uint32_t f_actual = 900000UL / (arr + 1UL);
        uint32_t f_frac   = (900000UL * 10UL / (arr + 1UL)) % 10UL;

        snprintf(buf, sizeof(buf), "%-4s  | %5lu  |  %lu.%lu\r\n",
                 scale[i].name,
                 (unsigned long)arr,
                 (unsigned long)f_actual,
                 (unsigned long)f_frac);
        USART2_SendString(buf);

        TIM4_PlayNote(arr, 400U);
        delay_ms(50U);
    }

    /* --- Part 2: Nokia Tune --- */
    USART2_SendString("\r\n-- Part 2: Nokia Tune --\r\n");
    USART2_SendString("Note  |  ARR  |  Dur(ms)\r\n");
    USART2_SendString("------|-------|----------\r\n");

    for (uint8_t i = 0; i < MELODY_LEN; i++)
    {
        uint32_t arr = melody[i].arr;
        uint32_t dur = melody[i].dur;

        snprintf(buf, sizeof(buf), "%-4s  | %5lu  |  %lu\r\n",
                 melody[i].name,
                 (unsigned long)arr,
                 (unsigned long)dur);
        USART2_SendString(buf);

        TIM4_PlayNote(arr, dur);
        REST(40U);   /* short gap between notes */
    }

    USART2_SendString("\r\n====== Task 6A Complete ======\r\n");

    /* Infinite loop */
    while (1) {}
}
