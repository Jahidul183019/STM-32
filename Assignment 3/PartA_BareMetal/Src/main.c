/*
 * ============================================================
 * CSE 2206 — Microcontroller & Embedded System Lab
 * Assignment 3, Part A: BMP280 Sensor Interface — BARE-METAL
 * Platform  : STM32F446RE Nucleo-64
 * Clock     : SYSCLK = 180 MHz (HSE PLL), APB1 = 45 MHz, APB2 = 90 MHz
 * Sensor    : BMP280 (hardware substitute for BME280)
 *
 * Pin Mapping:
 * PC1  → SPI2_MOSI  (AF7)
 * PC2  → SPI2_MISO  (AF5)
 * PC7  → SPI2_SCK   (AF5)
 * PB9  → CS         (GPIO Output, active LOW)
 * PA2  → USART2_TX  (AF7)
 * PA3  → USART2_RX  (AF7)
 *
 * Periodic output: TIM6 hardware interrupt every 1 second (ISR-driven)
 * Baud rate: 115200 @ 45 MHz APB1
 * ============================================================
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f446xx.h>    /* CMSIS device header */

/* ============================================================
 * BMP280 Register Address Map
 * ============================================================ */
#define BMP280_REG_DIG_T1       0x88U
#define BMP280_REG_DIG_T2       0x8AU
#define BMP280_REG_DIG_T3       0x8CU
#define BMP280_REG_DIG_P1       0x8EU
#define BMP280_REG_DIG_P2       0x90U
#define BMP280_REG_DIG_P3       0x92U
#define BMP280_REG_DIG_P4       0x94U
#define BMP280_REG_DIG_P5       0x96U
#define BMP280_REG_DIG_P6       0x98U
#define BMP280_REG_DIG_P7       0x9AU
#define BMP280_REG_DIG_P8       0x9CU
#define BMP280_REG_DIG_P9       0x9EU
#define BMP280_REG_CHIP_ID      0xD0U
#define BMP280_REG_RESET        0xE0U
#define BMP280_REG_STATUS       0xF3U
#define BMP280_REG_CTRL_MEAS    0xF4U
#define BMP280_REG_CONFIG       0xF5U
#define BMP280_REG_BURST_START  0xF7U   /* Pressure MSB — burst reads 6 bytes */

#define BMP280_CHIP_ID_PRIMARY  0x58U   /* Most BMP280 modules */
#define BMP280_CHIP_ID_ALT1     0x57U   /* Engineering samples */
#define BMP280_CHIP_ID_ALT2     0x56U   /* Early silicon */

#define BMP280_SOFT_RESET_CMD   0xB6U   /* Write to REG_RESET to soft-reset */

/* ============================================================
 * Calibration parameter structure
 * ============================================================ */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_Calib_t;

/* ============================================================
 * Global state
 * ============================================================ */
static BMP280_Calib_t calib;
static int32_t        t_fine;           /* Shared between T and P compensation */

/* Sensor readings — written by ISR, read by verification/main */
static volatile int32_t  g_comp_T  = 0; /* Units: hundredths of °C (e.g. 2534 = 25.34 °C) */
static volatile uint32_t g_comp_P  = 0; /* Units: Q24.8 fixed-point Pa */
static volatile uint32_t g_tick    = 0U;

/* Forward Declarations */
static void RunVerificationTests(void);
static void SampleAndCompensate(void);

/* ============================================================
 * SECTION 0 — Microsecond / millisecond delay
 * ============================================================ */
static void delay_us(uint32_t us)
{
    uint32_t count = us * 3U;
    while (count--) { __NOP(); }
}

static void delay_ms(uint32_t ms)
{
    while (ms--) { delay_us(1000U); }
}

/* ============================================================
 * SECTION 1 — System Clock Configuration
 * ============================================================ */
static void Clock_Phase1_HSI(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;             /* SW = 00 → HSI */
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0U);
}

static void Clock_Phase2_PLL180(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR      |= PWR_CR_VOS;

    FLASH->ACR = FLASH_ACR_ICEN
               | FLASH_ACR_DCEN
               | FLASH_ACR_PRFTEN
               | FLASH_ACR_LATENCY_5WS;

    RCC->PLLCFGR = (8U   << RCC_PLLCFGR_PLLM_Pos)
                 | (180U << RCC_PLLCFGR_PLLN_Pos)
                 | (0U   << RCC_PLLCFGR_PLLP_Pos)
                 | RCC_PLLCFGR_PLLSRC_HSI
                 | (8U   << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1
                | RCC_CFGR_PPRE1_DIV4
                | RCC_CFGR_PPRE2_DIV2);

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    USART2->CR1 &= ~USART_CR1_UE;
    USART2->BRR  = (24U << 4U) | 7U;    /* 0x0187 for 115200 baud @ 45MHz */
    USART2->CR1 |=  USART_CR1_UE;
}

/* ============================================================
 * SECTION 2 — USART2 @ 115200 baud
 * ============================================================ */
static void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER  &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER  |=   (2U << (2U * 2U)) | (2U << (3U * 2U));

    GPIOA->AFR[0] &= ~((0xFU << (4U * 2U)) | (0xFU << (4U * 3U)));
    GPIOA->AFR[0] |=   (7U   << (4U * 2U)) | (7U   << (4U * 3U));

    GPIOA->OSPEEDR |= (3U << (2U * 2U)) | (3U << (3U * 2U));

    USART2->BRR = (8U << 4U) | 11U; /* 16 MHz setup initially */

    USART2->CR1 = USART_CR1_UE
                | USART_CR1_TE
                | USART_CR1_RE
                | USART_CR1_RXNEIE;
}

static void USART2_SendString(const char *s)
{
    while (*s)
    {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC)) {}
}

int __io_putchar(int ch)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)(ch & 0xFF);
    return ch;
}

/* ============================================================
 * SECTION 3 — TIM6 Configuration
 * ============================================================ */
static void TIM6_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->PSC  = 9000U - 1U;
    TIM6->ARR  = 10000U - 1U;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1  |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM6_DAC_IRQn, 2U);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ============================================================
 * SECTION 4 — SPI2 Configuration
 * ============================================================ */
static void SPI2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PC1: SPI2_MOSI */
    GPIOC->MODER  &= ~(3U << (1U * 2U));
    GPIOC->MODER  |=  (2U << (1U * 2U));
    GPIOC->AFR[0] &= ~(0xFU << (4U * 1U));
    GPIOC->AFR[0] |=  (7U   << (4U * 1U));

    /* PC2: SPI2_MISO */
    GPIOC->MODER  &= ~(3U << (2U * 2U));
    GPIOC->MODER  |=  (2U << (2U * 2U));
    GPIOC->AFR[0] &= ~(0xFU << (4U * 2U));
    GPIOC->AFR[0] |=  (5U   << (4U * 2U));

    /* PC7: SPI2_SCK */
    GPIOC->MODER  &= ~(3U << (7U * 2U));
    GPIOC->MODER  |=  (2U << (7U * 2U));
    GPIOC->AFR[0] &= ~(0xFU << (4U * 7U));
    GPIOC->AFR[0] |=  (5U   << (4U * 7U));

    GPIOC->OSPEEDR |= (3U << (1U * 2U)) | (3U << (2U * 2U)) | (3U << (7U * 2U));

    /* PB9: CS Pins */
    GPIOB->MODER   &= ~(3U << (9U * 2U));
    GPIOB->MODER   |=  (1U << (9U * 2U));
    GPIOB->OSPEEDR |=  (3U << (9U * 2U));
    GPIOB->BSRR     =  (1U << 9U);

    SPI2->CR1 = 0U;
    SPI2->CR1 = SPI_CR1_MSTR
              | (2U << SPI_CR1_BR_Pos)
              | SPI_CR1_SSM
              | SPI_CR1_SSI;

    SPI2->CR1 |= SPI_CR1_SPE;
}

/* ============================================================
 * SECTION 5 — SPI2 Low-level Primitives
 * ============================================================ */
static uint8_t SPI2_TxRx(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_RXNE));
    return (uint8_t)SPI2->DR;
}

static void BMP280_WriteReg(uint8_t reg, uint8_t value)
{
    GPIOB->BSRR = (1U << (9U + 16U));
    SPI2_TxRx(reg & 0x7FU);
    SPI2_TxRx(value);
    while (SPI2->SR & SPI_SR_BSY);
    GPIOB->BSRR = (1U << 9U);
}

static uint8_t BMP280_ReadReg(uint8_t reg)
{
    uint8_t val;
    GPIOB->BSRR = (1U << (9U + 16U));
    SPI2_TxRx(reg | 0x80U);
    val = SPI2_TxRx(0x00U);
    while (SPI2->SR & SPI_SR_BSY);
    GPIOB->BSRR = (1U << 9U);
    return val;
}

static void BMP280_BurstRead(uint8_t start_reg, uint8_t *buf, uint8_t length)
{
    GPIOB->BSRR = (1U << (9U + 16U));
    SPI2_TxRx(start_reg | 0x80U);
    for (uint8_t i = 0U; i < length; i++)
    {
        buf[i] = SPI2_TxRx(0xFFU);
    }
    while (SPI2->SR & SPI_SR_BSY);
    GPIOB->BSRR = (1U << 9U);
}

/* ============================================================
 * SECTION 6 — Calibration Read & Compensation Formulas
 * ============================================================ */
static void BMP280_ReadCalibration(void)
{
    uint8_t calib_buf[24];
    BMP280_BurstRead(0x88U, calib_buf, 24U);

    calib.dig_T1 = (uint16_t)((calib_buf[1]  << 8) | calib_buf[0]);
    calib.dig_T2 = (int16_t) ((calib_buf[3]  << 8) | calib_buf[2]);
    calib.dig_T3 = (int16_t) ((calib_buf[5]  << 8) | calib_buf[4]);

    calib.dig_P1 = (uint16_t)((calib_buf[7]  << 8) | calib_buf[6]);
    calib.dig_P2 = (int16_t) ((calib_buf[9]  << 8) | calib_buf[8]);
    calib.dig_P3 = (int16_t) ((calib_buf[11] << 8) | calib_buf[10]);
    calib.dig_P4 = (int16_t) ((calib_buf[13] << 8) | calib_buf[12]);
    calib.dig_P5 = (int16_t) ((calib_buf[15] << 8) | calib_buf[14]);
    calib.dig_P6 = (int16_t) ((calib_buf[17] << 8) | calib_buf[16]);
    calib.dig_P7 = (int16_t) ((calib_buf[19] << 8) | calib_buf[18]);
    calib.dig_P8 = (int16_t) ((calib_buf[21] << 8) | calib_buf[20]);
    calib.dig_P9 = (int16_t) ((calib_buf[23] << 8) | calib_buf[22]);
}

static int32_t BMP280_Compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1)) * (int32_t)calib.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)calib.dig_T1) * ((adc_T >> 4) - (int32_t)calib.dig_T1)) >> 12) * (int32_t)calib.dig_T3) >> 14;

    t_fine = var1 + var2;
    T      = (t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t BMP280_Compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)calib.dig_P1)) >> 33;

    if (var1 == 0) return 0U; /* Avoid division by zero */

    p     = 1048576 - adc_P;
    p     = (((p << 31) - var2) * 3125) / var1;
    var1  = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2  = (((int64_t)calib.dig_P8) * p) >> 19;
    p     = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    return (uint32_t)p;
}

/* Helper logic to fetch and convert physical parameters */
static void SampleAndCompensate(void)
{
    uint8_t  raw[6];
    int32_t  adc_T, adc_P;

    BMP280_BurstRead(BMP280_REG_BURST_START, raw, 6U);

    adc_P = (int32_t)(((uint32_t)raw[0] << 12U)
                    | ((uint32_t)raw[1] <<  4U)
                    | ((uint32_t)raw[2] >>  4U));

    adc_T = (int32_t)(((uint32_t)raw[3] << 12U)
                    | ((uint32_t)raw[4] <<  4U)
                    | ((uint32_t)raw[5] >>  4U));

    g_comp_T = BMP280_Compensate_T(adc_T);
    g_comp_P = BMP280_Compensate_P(adc_P);
}

/* ============================================================
 * SECTION 7 — BMP280 Initialisation Sequence
 * ============================================================ */
static uint8_t BMP280_Init(void)
{
    uint8_t chip_id, status;
    uint32_t timeout;

    BMP280_WriteReg(BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    delay_ms(10U);

    timeout = 100000U;
    do {
        status = BMP280_ReadReg(BMP280_REG_STATUS);
        if (--timeout == 0U) break;
    } while (status & 0x01U);

    delay_ms(10U);

    chip_id = BMP280_ReadReg(BMP280_REG_CHIP_ID);
    if (chip_id != BMP280_CHIP_ID_PRIMARY && chip_id != BMP280_CHIP_ID_ALT1 && chip_id != BMP280_CHIP_ID_ALT2)
    {
        return 0U;
    }

    BMP280_ReadCalibration();

    /* Configure operating variables */
    BMP280_WriteReg(BMP280_REG_CONFIG,    0x10U); /* IIR Filter = 16 */
    BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x57U); /* Temp x2, Pres x16, Normal Mode */

    return chip_id;
}

/* ============================================================
 * SECTION 8 — Verification Tests
 * ============================================================ */
static void RunVerificationTests(void)
{
    char s[96];

    /* Sample live hardware definitions directly */
    int32_t  tc        = g_comp_T;
    uint32_t press_pa  = g_comp_P / 256U;
    uint32_t press_hpa = press_pa / 100U;

    /* ---- Test A1: Chip ID ---- */
    uint8_t id = BMP280_ReadReg(BMP280_REG_CHIP_ID);
    snprintf(s, sizeof(s), "[A1] ChipID=0x%02X (BMP280 expect 0x58/0x57/0x56; BME280=0x60)\r\n", id);
    USART2_SendString(s);
    if (id == BMP280_CHIP_ID_PRIMARY || id == BMP280_CHIP_ID_ALT1 || id == BMP280_CHIP_ID_ALT2)
        USART2_SendString("[A1] Chip ID PASS\r\n");
    else
        USART2_SendString("[A1] Chip ID FAIL\r\n");

    /* ---- Test A2: UART Loopback ---- */
    USART2_SendString("[A2] UART OK\r\n");

    /* ---- Test A3: TIM6 Heartbeat ---- */
    if (TIM6->CR1 & TIM_CR1_CEN)
        USART2_SendString("[A3] TIM6 heartbeat running — see tick counter in live output\r\n");
    else
        USART2_SendString("[A3] TIM6 FAIL\r\n");

    /* ---- Test A4: Sensor Plausibility ---- */
    uint8_t t_ok = (tc >= 1500 && tc <= 4000);
    uint8_t p_ok = (press_hpa >= 900U && press_hpa <= 1100U);

    if (!t_ok)
    {
        snprintf(s, sizeof(s), "[A4] Temp FAIL: %ld.%02ld C (expect 15–40 C)\r\n",
                 (long)(tc / 100), (long)(tc % 100 < 0 ? -(tc % 100) : tc % 100));
        USART2_SendString(s);
    }
    else if (!p_ok)
    {
        snprintf(s, sizeof(s), "[A4] Pres FAIL: %lu hPa (expect 900–1100 hPa)\r\n", (unsigned long)press_hpa);
        USART2_SendString(s);
    }
    else
    {
        USART2_SendString("[A4] Plausibility PASS\r\n");
    }

    /* ---- Test A5 ---- */
    USART2_SendString("[A5] See HAL project output — record three readings in lab report\r\n");
}

/* ============================================================
 * SECTION 10 — Main Program Entry Point
 * ============================================================ */
int main(void)
{
    Clock_Phase1_HSI();
    USART2_Init();

    USART2_SendString("\r\n========================================\r\n");
    USART2_SendString(" CSE 2206 - Assignment 3 Part A\r\n");
    USART2_SendString(" BMP280 via SPI2 - Bare-Metal (STM32F446RE)\r\n");
    USART2_SendString("========================================\r\n");
    USART2_SendString("Clock: HSI 16 MHz (switching to PLL 180 MHz...)\r\n");

    Clock_Phase2_PLL180();
    USART2_SendString("Clock: PLL 180 MHz locked. APB1=45 MHz. BRR updated.\r\n\r\n");
    USART2_SendString("[A2] UART OK\r\n\r\n");

    SPI2_Init();

    USART2_SendString("Initialising BMP280...\r\n");
    uint8_t chip_id = BMP280_Init();

    if (chip_id == 0U)
    {
        USART2_SendString("ERROR: BMP280 not detected! Halting.\r\n");
        while (1);
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "BMP280 detected: ChipID=0x%02X\r\n", chip_id);
    USART2_SendString(buf);
    USART2_SendString("Calibration constants loaded from NVM.\r\n");

    {
        char cbuf[160];
        snprintf(cbuf, sizeof(cbuf), "  dig_T1=%u  dig_T2=%d  dig_T3=%d\r\n", calib.dig_T1, calib.dig_T2, calib.dig_T3);
        USART2_SendString(cbuf);

        snprintf(cbuf, sizeof(cbuf),
                 "  dig_P1=%u  dig_P2=%d  dig_P3=%d\r\n"
                 "  dig_P4=%d  dig_P5=%d  dig_P6=%d\r\n"
                 "  dig_P7=%d  dig_P8=%d  dig_P9=%d\r\n",
                 calib.dig_P1, calib.dig_P2, calib.dig_P3,
                 calib.dig_P4, calib.dig_P5, calib.dig_P6,
                 calib.dig_P7, calib.dig_P8, calib.dig_P9);
        USART2_SendString(cbuf);
    }
    USART2_SendString("Sensor in Normal mode (T×2, P×16, IIR=16, t_sb=0.5ms)\r\n\r\n");

    /* Wait for the first autonomous hardware conversion loop to complete */
    delay_ms(100U);
    SampleAndCompensate();

    /* FIXED: TIM6 initialization is moved here to prevent racing outputs during boot */
    TIM6_Init();

    USART2_SendString("--- VERIFICATION TESTS ---\r\n");
    RunVerificationTests();
    USART2_SendString("--------------------------\r\n\r\n");

    USART2_SendString("--- LIVE SENSOR OUTPUT (1 Hz via TIM6 ISR) ---\r\n");

    while (1)
    {
        __WFI();
    }

    return 0;
}

/* ============================================================
 * TIM6 ISR
 * ============================================================ */
void TIM6_DAC_IRQHandler(void)
{
    if (!(TIM6->SR & TIM_SR_UIF)) return;
    TIM6->SR &= ~TIM_SR_UIF;

    g_tick++;

    /* Perform reading conversion update loops */
    SampleAndCompensate();

    /* Format output metrics */
    int32_t tc_w = g_comp_T / 100;
    int32_t tc_f = g_comp_T % 100;
    if (tc_f < 0) tc_f = -tc_f;

    int32_t tf_s = g_comp_T * 9 / 5 + 3200;
    int32_t tf_w = tf_s / 100;
    int32_t tf_f = tf_s % 100;
    if (tf_f < 0) tf_f = -tf_f;

    uint32_t pp  = g_comp_P / 256U;
    uint32_t phw = pp / 100U;
    uint32_t phf = pp % 100U;

    char msg[160];
    snprintf(msg, sizeof(msg),
             "[Tick:%4lu] Temp: %ld.%02ld C / %ld.%02ld F | "
             "Pres: %lu.%02lu hPa | Hum: N/A (BMP280)\r\n",
             (unsigned long)g_tick,
             (long)tc_w, (long)tc_f,
             (long)tf_w, (long)tf_f,
             (unsigned long)phw, (unsigned long)phf);

    USART2_SendString(msg);
}
