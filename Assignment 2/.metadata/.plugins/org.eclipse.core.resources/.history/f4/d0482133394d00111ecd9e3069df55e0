/*
 * ============================================================
 * CSE 2206 — Microcontroller & Embedded System Lab-02
 * Task 3: Duration Measurement & Code Profiling — BARE-METAL
 * Platform : STM32F446RE Nucleo-64
 *
 * Method   : TIM2 + NVIC interrupt
 *            TIM2 overflows every 1 ms → ISR increments ms_count
 *            CNT reads 0–999 µs within the current ms
 *            time = ms_count + CNT/1000
 *
 * ── Timer Calculations ─────────────────────────────────────
 *  Source clock  : APB1 timer clock
 *                  APB1 bus  = HCLK / 4  = 180/4 = 45 MHz
 *                  APB1 ≠ HCLK, so timer clock = APB1 × 2
 *                  fTIM2_CLK = 45 × 2 = 90 MHz
 *
 *  Prescaler     : PSC = 89
 *                  tick frequency = fTIM2_CLK / (PSC + 1)
 *                                 = 90 MHz / 90
 *                                 = 1 MHz
 *                  → 1 tick = 1 µs
 *
 *  Auto-reload   : ARR = 999
 *                  counter counts 0, 1, 2, … 999 then resets
 *                  period = (ARR + 1) ticks = 1000 ticks = 1000 µs
 *                  → overflow (update event) every 1 ms exactly
 *
 *  NVIC          : TIM2 update interrupt fires every overflow
 *                  ISR increments global ms_count
 *                  → ms_count = whole milliseconds elapsed
 *                  → TIM2->CNT = µs within the current ms (0–999)
 *
 *  Reading time  :
 *                  ms_count  → whole ms part      e.g. 1
 *                  TIM2->CNT → sub-ms µs part     e.g. 674
 *                  result    = ms_count + CNT/1000 = 1.674 ms
 *
 *  Cycles estimate:
 *                  fCPU = 180 MHz = 180 cycles per µs
 *                  est_cycles = elapsed_us × 180
 * ============================================================
 */

#include <stm32f446xx.h>
#include <stdint.h>
#include <stdio.h>

/* =========================================================
 * Global millisecond counter
 * Incremented by TIM2 ISR every 1 ms.
 * volatile: tells compiler this can change outside normal flow.
 * ========================================================= */
volatile uint32_t ms_count = 0;

/* =========================================================
 * System Clock — fCPU = 180 MHz
 *
 * HSI = 16 MHz (internal oscillator)
 * PLLM = 8   → PLL input = 16/8 = 2 MHz
 * PLLN = 180 → VCO output = 2 × 180 = 360 MHz
 * PLLP = 2   → SYSCLK = 360/2 = 180 MHz
 *
 * Bus prescalers set BEFORE switching to PLL:
 *   AHB  = SYSCLK / 1 = 180 MHz  (HCLK)
 *   APB1 = HCLK   / 4 =  45 MHz  (max allowed: 45 MHz)
 *   APB2 = HCLK   / 2 =  90 MHz  (max allowed: 90 MHz)
 * ========================================================= */
void SystemClock_Config(void)
{
    /* 1. Power controller clock + voltage scaling for 180 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR      |= PWR_CR_VOS;           /* Scale 1 = up to 180 MHz */

    /* 2. Enable HSI (16 MHz internal RC) and wait until stable */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* 3. Flash latency for 180 MHz + enable instruction/data caches
     *    At 180 MHz and 3.3 V, datasheet requires 5 wait states     */
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |=  FLASH_ACR_LATENCY_5WS;

    /* 4. Configure PLL
     *    PLLM=8: VCO input = 16/8 = 2 MHz (must be 1–2 MHz)
     *    PLLN=180: VCO output = 2×180 = 360 MHz (must be 100–432)
     *    PLLP=0 (meaning /2): SYSCLK = 360/2 = 180 MHz
     *    PLLQ=2: USB clock = 360/2 = 180 MHz (not used here)        */
    RCC->PLLCFGR  = 0;
    RCC->PLLCFGR |= (8U   << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (180U << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0U   << RCC_PLLCFGR_PLLP_Pos);  /* 0b00 = /2  */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR |= (2U   << RCC_PLLCFGR_PLLQ_Pos);

    /* 5. Enable PLL and wait for lock */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 6. OverDrive mode — mandatory for 180 MHz on STM32F446
     *    Without this the core cannot run above 168 MHz            */
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* 7. Set bus prescalers BEFORE switching to PLL
     *    If we switch first, APB1 briefly runs at 180 MHz
     *    which violates its 45 MHz maximum and can hang the bus    */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;    /* AHB  = 180 MHz          */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;   /* APB1 =  45 MHz          */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;   /* APB2 =  90 MHz          */

    /* 8. Switch SYSCLK source to PLL and wait for confirmation */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* =========================================================
 * USART2 — 115200 8N1
 * PA2 = TX (AF7), PA3 = RX (AF7)
 *
 * Baud rate calculation:
 *   fCK = APB1 = 45 MHz
 *   USARTDIV = fCK / (16 × Baud)
 *            = 45,000,000 / (16 × 115200)
 *            = 24.414
 *   Mantissa = 24       → stored in BRR[15:4]
 *   Fraction = 0.414×16 = 6.6 ≈ 7  → stored in BRR[3:0]
 *   BRR = (24 << 4) | 7 = 0x187
 * ========================================================= */
static void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 → Alternate Function mode, AF7 = USART2_TX */
    GPIOA->MODER  &= ~(3U << (2*2));
    GPIOA->MODER  |=  (2U << (2*2));
    GPIOA->AFR[0] &= ~(0xFU << (4*2));
    GPIOA->AFR[0] |=  (7U   << (4*2));

    /* PA3 → Alternate Function mode, AF7 = USART2_RX */
    GPIOA->MODER  &= ~(3U << (3*2));
    GPIOA->MODER  |=  (2U << (3*2));
    GPIOA->AFR[0] &= ~(0xFU << (4*3));
    GPIOA->AFR[0] |=  (7U   << (4*3));

    USART2->BRR = (24U << 4) | 7U;   /* 0x187 = 115200 baud @ 45MHz */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void USART2_SendString(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC)) {}
}

/* =========================================================
 * TIM2 — 1 ms interrupt, CNT = µs within current ms
 *
 * ── Calculation recap ──────────────────────────────────
 *  fTIM2_CLK = 90 MHz  (APB1=45MHz, prescaler≠1 → ×2)
 *
 *  PSC = 89
 *  tick_freq = 90,000,000 / (89+1) = 1,000,000 Hz = 1 MHz
 *  tick_period = 1 µs
 *
 *  ARR = 999
 *  overflow_period = (999+1) ticks × 1 µs = 1000 µs = 1 ms
 *  → update interrupt fires every 1 ms
 *
 *  After init:
 *    ms_count   increments every 1 ms via ISR
 *    TIM2->CNT  counts 0→999 µs and resets
 *
 *  Together:
 *    time_ms = ms_count + TIM2->CNT / 1000
 *            = whole_ms + fractional_ms
 * ─────────────────────────────────────────────────────── */
static void TIM2_Init(void)
{
    /* 1. Enable TIM2 peripheral clock on APB1 bus */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __NOP(); __NOP();                    /* Short stabilisation delay */

    /* 2. Stop counter before configuring registers */
    TIM2->CR1 &= ~TIM_CR1_CEN;

    /* 3. Prescaler: divide 90 MHz down to 1 MHz (1 µs per tick)
     *    PSC value = (desired_divisor - 1) = 90 - 1 = 89            */
    TIM2->PSC = 89U;

    /* 4. Auto-reload: overflow after 1000 ticks = 1 ms
     *    ARR value = (ticks_per_period - 1) = 1000 - 1 = 999        */
    TIM2->ARR = 999U;

    /* 5. Force PSC and ARR into shadow registers immediately */
    TIM2->EGR = TIM_EGR_UG;

    /* 6. Clear the update flag set by step 5 (would trigger ISR) */
    TIM2->SR = 0U;

    /* 7. Enable update interrupt (fires every ARR overflow = 1 ms) */
    TIM2->DIER |= TIM_DIER_UIE;

    /* 8. Configure NVIC for TIM2
     *    Priority 0 = highest, ensures ms_count stays accurate
     *    even if other interrupts are added later               */
    NVIC_SetPriority(TIM2_IRQn, 0U);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* 9. Start counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* =========================================================
 * TIM2 Interrupt Service Routine
 * Fires every 1 ms when CNT overflows from 999 → 0.
 * Must clear UIF flag, otherwise ISR fires continuously.
 * ========================================================= */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)      /* Check update interrupt flag */
    {
        TIM2->SR &= ~TIM_SR_UIF;    /* Clear flag FIRST            */
        ms_count++;                 /* Count one more millisecond  */
    }
}

/* =========================================================
 * get_time_us()
 *
 * Returns a snapshot of current time in microseconds.
 *
 * Race condition protection:
 *   If an overflow happens between reading ms_count and CNT:
 *     ms_count read as N,  then overflow fires → ms_count=N+1,
 *     CNT resets to 0,  we read CNT=3
 *     → result = N*1000 + 3  (WRONG, should be (N+1)*1000 + 3)
 *
 *   Fix: read ms_count twice, retry if it changed mid-read.
 *   This guarantees ms_count and CNT belong to the same ms.
 *
 * ── Why return µs not ms? ──────────────────────────────
 *   Returning raw µs lets the PROFILE macro do simple
 *   integer subtraction for elapsed time.
 *   Profile_Print then splits into ms_count + cnt for display.
 * ========================================================= */
static uint32_t get_time_us(void)
{
    uint32_t ms, cnt;
    do {
        ms  = ms_count;        /* Read whole-ms counter from ISR  */
        cnt = TIM2->CNT;       /* Read sub-ms µs position (0–999) */
    } while (ms != ms_count);  /* Retry if overflow happened       */

    /*  ms  = whole milliseconds elapsed
     *  cnt = microseconds within the current millisecond (0–999)
     *  total µs = ms × 1000 + cnt                                  */
    return ms * 1000U + cnt;
}

/* =========================================================
 * Profile_Print
 *
 * Receives elapsed_us = t1_us - t0_us from PROFILE macro.
 *
 * Splits into display form using professor's formula:
 *   ms_count = elapsed_us / 1000   → whole ms
 *   cnt      = elapsed_us % 1000   → fractional ms (0–999 µs)
 *   display  = "ms_count.cnt ms"   e.g. "1.912 ms"
 *
 * Cycle estimate:
 *   At 180 MHz, 1 µs = 180 CPU cycles
 *   est_cycles = elapsed_us × 180
 *   Accuracy: ±1 µs → ±180 cycles (TIM2 resolution limit)
 *
 * No float used — safe with --specs=nano.specs.
 * %03lu on cnt ensures zero-padding:
 *   7 µs remainder → "007", not "7"  (so 1.007 not 1.7)
 * ========================================================= */
static void Profile_Print(const char *label, uint32_t elapsed_us)
{
    char buf[128];

    uint32_t ms_cnt    = elapsed_us / 1000U;   /* whole ms part       */
    uint32_t cnt       = elapsed_us % 1000U;   /* µs remainder 0–999  */
    uint32_t est_cycles = elapsed_us * 180U;   /* 180 cycles per µs   */

    snprintf(buf, sizeof(buf),
        "%-32s | %7lu us | %lu.%03lu ms | ~%lu cyc\r\n",
        label,
        (unsigned long)elapsed_us,
        (unsigned long)ms_cnt,
        (unsigned long)cnt,
        (unsigned long)est_cycles);

    USART2_SendString(buf);
}

/* =========================================================
 * PROFILE macro
 *
 * Captures microsecond timestamp before and after block.
 * Uses get_time_us() which combines ms_count + CNT safely.
 * Elapsed = t1 - t0 in µs, passed to Profile_Print.
 *
 * No blank lines inside — blank line breaks '\' continuation
 * and causes "initializer element is not constant" errors.
 * ========================================================= */
#define PROFILE(label, block)                  \
    do {                                       \
        uint32_t _t0 = get_time_us();          \
        { block }                              \
        uint32_t _t1 = get_time_us();          \
        Profile_Print(label, _t1 - _t0);       \
    } while (0)

/* =========================================================
 * Code Blocks Under Test
 * ========================================================= */

/* --- Block 1: Bubble Sort (worst case: reverse sorted) --- */
#define SORT_N 100U
static int sort_arr[SORT_N];

static void BubbleSort_PrepareWorstCase(void)
{
    for (uint32_t i = 0; i < SORT_N; i++)
        sort_arr[i] = (int)(SORT_N - i);   /* 100, 99, ..., 1 */
}

static void BubbleSort(void)
{
    int temp;
    for (uint32_t i = 0; i < SORT_N - 1U; i++) {
        for (uint32_t j = 0; j < SORT_N - 1U - i; j++) {
            if (sort_arr[j] > sort_arr[j + 1U]) {
                temp             = sort_arr[j];
                sort_arr[j]      = sort_arr[j + 1U];
                sort_arr[j + 1U] = temp;
            }
        }
    }
}

/* --- Block 2: Integer Square Root x1000 --- */
static uint32_t isqrt(uint32_t n)
{
    if (n == 0U) return 0U;
    uint32_t x = n;
    uint32_t y = (x + 1U) / 2U;
    while (y < x) {
        x = y;
        y = (x + n / x) / 2U;
    }
    return x;
}

static volatile uint32_t isqrt_result;   /* volatile: prevent optimiser removal */

static void IsqrtBenchmark(void)
{
    for (uint32_t i = 0; i < 1000U; i++)
        isqrt_result = isqrt(i * 7U + 1U);
}

/* --- Block 3: Byte-by-byte Memory Copy (512 bytes) --- */
static uint8_t src_buf[512U];
static uint8_t dst_buf[512U];

static void MemCopy_ByteByByte(void)
{
    for (uint32_t i = 0; i < 512U; i++)
        dst_buf[i] = src_buf[i];
}

/* =========================================================
 * Main
 * ========================================================= */
int main(void)
{
    SystemClock_Config();
    USART2_Init();
    TIM2_Init();          /* Starts ms_count incrementing via NVIC */

    /* Fill source buffer with known pattern */
    for (uint32_t i = 0; i < 512U; i++)
        src_buf[i] = (uint8_t)(i & 0xFFU);

    /* Show ms_count + CNT/1000 at startup — confirms ISR is running */
    char tbuf[64];
    uint32_t now_us  = get_time_us();
    uint32_t now_ms  = now_us / 1000U;    /* ms_count part  */
    uint32_t now_cnt = now_us % 1000U;    /* CNT/1000 part  */
    snprintf(tbuf, sizeof(tbuf),
        "Timer running. ms_count=%lu, cnt=%lu → %lu.%03lu ms\r\n",
        (unsigned long)now_ms,
        (unsigned long)now_cnt,
        (unsigned long)now_ms,
        (unsigned long)now_cnt);
    USART2_SendString(tbuf);

    /* Table header */
    USART2_SendString("\r\n===== Lab-02 Task 3: Code Profiling =====\r\n");
    USART2_SendString(
        "Block                            |   us     |  ms_cnt.cnt ms  | ~Cycles\r\n"
        "---------------------------------|----------|-----------------|--------\r\n");

    /* Block 1: Bubble sort worst case */
    BubbleSort_PrepareWorstCase();
    PROFILE("[1] Bubble sort N=100 (worst)", BubbleSort(););

    /* Block 2: USART SendString — the string itself prints mid-line,
     * that is expected since it is the code block being measured    */
    PROFILE("[2] USART2_SendString 48 B",
        USART2_SendString("\r\nPROFILING: STM32F446RE USART2 @ 115200 baud OK!\r\n"););

    /* Block 3: isqrt x1000 */
    PROFILE("[3] isqrt() x1000 inputs", IsqrtBenchmark(););

    /* Block 4: MemCopy 512 B */
    PROFILE("[4] MemCopy byte x512", MemCopy_ByteByByte(););

    USART2_SendString("\r\n===== Task 3 Complete =====\r\n");

    while (1) {}
}
