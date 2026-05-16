/*
 * ============================================================
 * CSE 2206 — Microcontroller & Embedded System Lab-02
 * Task 3: Duration Measurement & Code Profiling — BARE-METAL
 * Platform : STM32F446RE Nucleo-64
 * Clock    : fCPU = 180 MHz, fTIM2_CLK = 90 MHz (APB1×2)
 * ============================================================
 */

#include <stm32f446xx.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/**
 * @brief  Configure fCPU = 180 MHz using HSI as source
 */
void SystemClock_Config(void) {
    /* 1. Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* 2. Voltage scaling (Scale 1 mode for 180 MHz) */
    PWR->CR |= PWR_CR_VOS;

    /* 3. Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* 4. Configure FLASH latency and enable caches */
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    /* 5. Configure PLL: HSI=16MHz, M=8, N=180, P=2 -> 180MHz */
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) | (180 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI);

    /* 6. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 7. Enable OverDrive mode */
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* 8. Configure Bus Prescalers (APB1 must be <= 45MHz) */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // HCLK = 180MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 = 45MHz (Timers = 90MHz)
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 = 90MHz

    /* 9. Select PLL as system clock */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/**
 * @brief  Initialise USART2 (115200 8N1).
 */
static void USART2_Init(void) {
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR  |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER  |= (2U << (2*2)) | (2U << (3*2)); // AF Mode
    GPIOA->AFR[0] |= (7U << (4*2)) | (7U << (4*3)); // AF7

    /* BRR = 45MHz / 115200 = 24.414 -> Mantissa 24, Fraction 7 */
    USART2->BRR = (24 << 4) | 7;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void USART2_SendString(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC));
}

/**
 * @brief  TIM6 for delay_ms (Task 2 compatibility)
 */
static void TIM6_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 89U; // 90MHz/90 = 1MHz
    TIM6->ARR = 0xFFFFU;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;
}

static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        TIM6->CNT = 0;
        while (TIM6->CNT < 1000);
    }
}

/**
 * @brief  DWT Cycle Counter (Method A)
 */
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  TIM2 Free-Running 1us Stopwatch (Method B)
 */
static void TIM2_FreeRun_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 89U; // 90MHz / 90 = 1MHz
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void Profile_Print(const char *label, uint32_t cycles, uint32_t tim2_us) {
    char buf[160];
    // ns = cycles * (1000/180)
    uint32_t ns = (uint32_t)((uint64_t)cycles * 1000U / 180U);
    uint32_t us = ns / 1000U;
    uint32_t ms = us / 1000U;

    snprintf(buf, sizeof(buf),
        "%-30s | %12lu | %10lu | %8lu | %4lu | %8lu\r\n",
        label, (unsigned long)cycles, (unsigned long)ns,
        (unsigned long)us, (unsigned long)ms, (unsigned long)tim2_us);
    USART2_SendString(buf);
}

/* ---- Blocks Under Test ---- */
#define SORT_N 100
static int sort_arr[SORT_N];
static void BubbleSort(void) {
    for (int i = 0; i < SORT_N - 1; i++)
        for (int j = 0; j < SORT_N - i - 1; j++)
            if (sort_arr[j] > sort_arr[j+1]) {
                int t = sort_arr[j]; sort_arr[j] = sort_arr[j+1]; sort_arr[j+1] = t;
            }
}

static uint32_t isqrt(uint32_t n) {
    uint32_t x = n, y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return x;
}

#define PROFILE(label, block) do {                     \
    uint32_t t0_d = DWT->CYCCNT;                       \
    uint32_t t0_t = TIM2->CNT;                         \
    { block }                                          \
    uint32_t t1_d = DWT->CYCCNT;                       \
    uint32_t t1_t = TIM2->CNT;                         \
    Profile_Print(label, t1_d - t0_d, t1_t - t0_t);    \
} while(0)

int main(void) {
    SystemClock_Config();
    USART2_Init();
    TIM6_Init();
    DWT_Init();
    TIM2_FreeRun_Init();

    static uint8_t s_buf[512], d_buf[512];

    USART2_SendString("\r\n# Block Description            | Cycles       | ns         | us       | ms   | TIM2 us\r\n");
    USART2_SendString("----------------------------------------------------------------------------------------\r\n");

    // [1] Bubble Sort Worst Case
    for(int i=0; i<SORT_N; i++) sort_arr[i] = SORT_N - i;
    PROFILE("[1] Bubble sort N=100", BubbleSort(););

    // [2] delay_ms(100)
    PROFILE("[2] delay_ms(100)", delay_ms(100););

    // [3] SendString 48B
    PROFILE("[3] SendString 48B", USART2_SendString("PROFILING: STM32F446RE USART2 @ 115200 baud OK!\r\n"););

    // [4] ISQRT x1000
    PROFILE("[4] isqrt() x1000", for(int i=0; i<1000; i++) isqrt(i*7+1););

    // [5] Memcpy 512B
    PROFILE("[5] MemCopy 512B", for(int i=0; i<512; i++) d_buf[i] = s_buf[i];);

    while(1);
}
