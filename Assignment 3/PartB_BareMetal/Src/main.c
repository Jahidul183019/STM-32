/*
 * ============================================================
 * CSE 2206 — Microcontroller & Embedded System Lab
 * Assignment 3, Part B: BMP280 Sensor Interface — BARE-METAL
 * Platform  : STM32F446RE Nucleo-64
 * Clock     : SYSCLK = 180 MHz (HSE PLL), APB1 = 45 MHz
 * Sensor    : BMP280 (I2C Mode)
 *
 * Pin Mapping:
 * PB6  → I2C1_SCL   (AF4, Open-Drain, Pull-Up)
 * PB7  → I2C1_SDA   (AF4, Open-Drain, Pull-Up)
 * PA2  → USART2_TX  (AF7)
 * PA3  → USART2_RX  (AF7)
 * ============================================================
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f446xx.h>

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
#define BMP280_REG_BURST_START  0xF7U

#define BMP280_CHIP_ID_PRIMARY  0x58U
#define BMP280_CHIP_ID_ALT1     0x57U
#define BMP280_CHIP_ID_ALT2     0x56U
#define BMP280_SOFT_RESET_CMD   0xB6U

typedef struct {
    uint16_t dig_T1; int16_t  dig_T2; int16_t  dig_T3;
    uint16_t dig_P1; int16_t  dig_P2; int16_t  dig_P3; int16_t  dig_P4;
    int16_t  dig_P5; int16_t  dig_P6; int16_t  dig_P7; int16_t  dig_P8; int16_t  dig_P9;
} BMP280_Calib_t;

static BMP280_Calib_t calib;
static int32_t        t_fine;
static uint8_t        dev_addr = 0x76U;

static volatile int32_t  g_comp_T  = 0;
static volatile uint32_t g_comp_P  = 0;
static volatile uint32_t g_tick    = 0U;

static void RunVerificationTests(void);
static void SampleAndCompensate(void);

static void delay_us(uint32_t us) {
    uint32_t count = us * 3U;
    while (count--) { __NOP(); }
}
static void delay_ms(uint32_t ms) {
    while (ms--) { delay_us(1000U); }
}

static void Clock_Phase1_HSI(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0U);
}

static void Clock_Phase2_PLL180(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR      |= PWR_CR_VOS;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    RCC->PLLCFGR = (8U << RCC_PLLCFGR_PLLM_Pos) | (180U << RCC_PLLCFGR_PLLN_Pos) | (0U << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSI | (8U << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2);
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    USART2->CR1 &= ~USART_CR1_UE;
    USART2->BRR  = (24U << 4U) | 7U;
    USART2->CR1 |=  USART_CR1_UE;
}

static void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER  &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER  |=   (2U << (2U * 2U)) | (2U << (3U * 2U));
    GPIOA->AFR[0] &= ~((0xFU << (4U * 2U)) | (0xFU << (4U * 3U)));
    GPIOA->AFR[0] |=   (7U   << (4U * 2U)) | (7U   << (4U * 3U));
    GPIOA->OSPEEDR |= (3U << (2U * 2U)) | (3U << (3U * 2U));
    USART2->BRR = (8U << 4U) | 11U;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void USART2_SendString(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC)) {}
}

static void TIM6_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC  = 9000U - 1U;
    TIM6->ARR  = 10000U - 1U;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1  |= TIM_CR1_CEN;
    NVIC_SetPriority(TIM6_DAC_IRQn, 2U);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

static void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER   &= ~((3U << (6U * 2U)) | (3U << (7U * 2U)));
    GPIOB->MODER   |=  ((2U << (6U * 2U)) | (2U << (7U * 2U)));
    GPIOB->AFR[0]  &= ~((0xFU << (4U * 6U)) | (0xFU << (4U * 7U)));
    GPIOB->AFR[0]  |=  ((4U   << (4U * 6U)) | (4U   << (4U * 7U)));

    GPIOB->OTYPER  |= (1U << 6U) | (1U << 7U);
    GPIOB->OSPEEDR |= (2U << (6U * 2U)) | (2U << (7U * 2U));
    GPIOB->PUPDR   &= ~((3U << (6U * 2U)) | (3U << (7 * 2U)));
    GPIOB->PUPDR   |=  ((1U << (6U * 2U)) | (1U << (7 * 2U)));

    GPIOB->MODER   &= ~(3U << (6U * 2U));
    GPIOB->MODER   |=  (1U << (6U * 2U));
    for (uint8_t i = 0; i < 9; i++) {
        GPIOB->BSRR = (1U << 6U);
        delay_us(5U);
        GPIOB->BSRR = (1U << (6U + 16U));
        delay_us(5U);
    }
    GPIOB->MODER   &= ~(3U << (6U * 2U));
    GPIOB->MODER   |=  (2U << (6U * 2U));

    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    delay_ms(10U);
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR2   = 45U;
    I2C1->CCR   = 225U;
    I2C1->TRISE = 46U;
    I2C1->CR1  |= I2C_CR1_PE;
}

/* ============================================================
 * I2C PING (Scanner) - Checks if device physically exists
 * ============================================================ */
static uint8_t I2C_Ping(uint8_t addr)
{
    uint32_t timeout;

    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000U; /* Fast timeout */
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0U) return 0; }

    I2C1->DR = (addr << 1U) | 0U;
    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
    {
        if (I2C1->SR1 & I2C_SR1_AF)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0;
        }
        if (--timeout == 0U)
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0;
        }
    }

    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
    return 1;
}

/* ============================================================
 * I2C READ/WRITE primitives (Fast Timeouts added)
 * ============================================================ */
static void BMP280_WriteReg(uint8_t reg, uint8_t value) {
    uint32_t timeout = 10000U;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0U) return; }

    timeout = 10000U;
    I2C1->DR = (dev_addr << 1U) | 0U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0U) goto err_out; }
    (void)I2C1->SR2;

    timeout = 10000U;
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { if (--timeout == 0U) goto err_out; }

    timeout = 10000U;
    I2C1->DR = value;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) { if (--timeout == 0U) goto err_out; }

err_out:
    I2C1->CR1 |= I2C_CR1_STOP;
}

static uint8_t BMP280_ReadReg(uint8_t reg) {
    uint8_t val = 0xFFU;
    uint32_t timeout = 10000U;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0U) return 0xFF; }

    timeout = 10000U;
    I2C1->DR = (dev_addr << 1U) | 0U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return 0xFF; } }
    (void)I2C1->SR2;

    timeout = 10000U;
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return 0xFF; } }

    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return 0xFF; } }

    timeout = 10000U;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0U) return 0xFF; }

    timeout = 10000U;
    I2C1->DR = (dev_addr << 1U) | 1U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return 0xFF; } }

    I2C1->CR1 &= ~I2C_CR1_ACK;
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;

    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0U) return 0xFF; }
    val = (uint8_t)I2C1->DR;

    return val;
}

static void BMP280_BurstRead(uint8_t start_reg, uint8_t *buf, uint8_t length) {
    uint32_t timeout;
    if (length == 0U) return;

    timeout = 10000U;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0U) return; }

    timeout = 10000U;
    I2C1->DR = (dev_addr << 1U) | 0U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return; } }
    (void)I2C1->SR2;

    timeout = 10000U;
    I2C1->DR = start_reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return; } }

    timeout = 10000U;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return; } }

    timeout = 10000U;
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if (--timeout == 0U) return; }

    timeout = 10000U;
    I2C1->DR = (dev_addr << 1U) | 1U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if (--timeout == 0U) { I2C1->CR1 |= I2C_CR1_STOP; return; } }
    (void)I2C1->SR2;

    for (uint8_t i = 0U; i < length; i++) {
        if (i == (length - 1U)) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |=  I2C_CR1_STOP;
        }
        timeout = 10000U;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) { if (--timeout == 0U) return; }
        buf[i] = (uint8_t)I2C1->DR;
    }
}

static void BMP280_ReadCalibration(void) {
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

static int32_t BMP280_Compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = (((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1)) * (int32_t)calib.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)calib.dig_T1) * ((adc_T >> 4) - (int32_t)calib.dig_T1)) >> 12) * (int32_t)calib.dig_T3) >> 14;
    t_fine = var1 + var2;
    T      = (t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t BMP280_Compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)calib.dig_P1)) >> 33;
    if (var1 == 0) return 0U;
    p     = 1048576 - adc_P;
    p     = (((p << 31) - var2) * 3125) / var1;
    var1  = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2  = (((int64_t)calib.dig_P8) * p) >> 19;
    p     = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)p;
}

static void SampleAndCompensate(void) {
    uint8_t  raw[6];
    BMP280_BurstRead(BMP280_REG_BURST_START, raw, 6U);
    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12U) | ((uint32_t)raw[1] <<  4U) | ((uint32_t)raw[2] >>  4U));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12U) | ((uint32_t)raw[4] <<  4U) | ((uint32_t)raw[5] >>  4U));
    g_comp_T = BMP280_Compensate_T(adc_T);
    g_comp_P = BMP280_Compensate_P(adc_P);
}

static uint8_t BMP280_Init(void) {
    uint8_t chip_id, status;
    uint32_t timeout;

    BMP280_WriteReg(BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    delay_ms(10U);

    timeout = 100U; /* FIXED: Reduced from 100,000 to prevent long freezes */
    do {
        status = BMP280_ReadReg(BMP280_REG_STATUS);
        if (status == 0xFF) return 0; /* Hardware failure caught immediately */
        if (--timeout == 0U) break;
    } while (status & 0x01U);

    chip_id = BMP280_ReadReg(BMP280_REG_CHIP_ID);
    BMP280_ReadCalibration();
    BMP280_WriteReg(BMP280_REG_CONFIG,    0x10U);
    BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x57U);
    return chip_id;
}

static void RunVerificationTests(void) {
    char s[96];
    int32_t  tc        = g_comp_T;
    uint32_t press_pa  = g_comp_P / 256U;
    uint32_t press_hpa = press_pa / 100U;

    uint8_t id = BMP280_ReadReg(BMP280_REG_CHIP_ID);
    snprintf(s, sizeof(s), "[B1] ChipID=0x%02X (BMP280 expect 0x58/0x57/0x56; BME280=0x60)\r\n", id);
    USART2_SendString(s);
    if (id == BMP280_CHIP_ID_PRIMARY || id == BMP280_CHIP_ID_ALT1 || id == BMP280_CHIP_ID_ALT2)
        USART2_SendString("[B1] Chip ID PASS\r\n");
    else
        USART2_SendString("[B1] Chip ID FAIL\r\n");

    I2C1->CR1 |= I2C_CR1_START;
    uint32_t handshake_timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_SB) && --handshake_timeout);

    if (handshake_timeout) {
        I2C1->DR = (dev_addr << 1U) | 0U;
        handshake_timeout = 100000U;
        while (!(I2C1->SR1 & I2C_SR1_ADDR) && --handshake_timeout);
    }

    if (!handshake_timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        USART2_SendString("[B2] I2C ACK FAIL\r\n");
    } else {
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        USART2_SendString("[B2] I2C ACK OK\r\n");
    }

    if (TIM6->CR1 & TIM_CR1_CEN)
        USART2_SendString("[B3] TIM6 heartbeat running\r\n");
    else
        USART2_SendString("[B3] TIM6 FAIL\r\n");

    uint8_t t_ok = (tc >= 1500 && tc <= 4000);
    uint8_t p_ok = (press_hpa >= 900U && press_hpa <= 1100U);

    if (!t_ok) {
        snprintf(s, sizeof(s), "[B4] Temp FAIL: %ld.%02ld C (expect 15–40 C)\r\n", (long)(tc / 100), (long)(tc % 100 < 0 ? -(tc % 100) : tc % 100));
        USART2_SendString(s);
    } else if (!p_ok) {
        snprintf(s, sizeof(s), "[B4] Pres FAIL: %lu hPa (expect 900–1100 hPa)\r\n", (unsigned long)press_hpa);
        USART2_SendString(s);
    } else {
        USART2_SendString("[B4] Plausibility PASS\r\n");
    }
    USART2_SendString("[B5] See HAL project output — record three readings in lab report\r\n");
}

int main(void) {
    Clock_Phase1_HSI();
    USART2_Init();
    USART2_SendString("\r\n========================================\r\n");
    USART2_SendString(" CSE 2206 - Assignment 3 Part B\r\n");
    USART2_SendString(" BMP280 via I2C1 - Bare-Metal (STM32F446RE)\r\n");
    USART2_SendString("========================================\r\n");
    USART2_SendString("Clock: HSI 16 MHz (switching to PLL 180 MHz...)\r\n");

    Clock_Phase2_PLL180();
    USART2_SendString("Clock: PLL 180 MHz locked. APB1=45 MHz. BRR updated.\r\n\r\n");
    USART2_SendString("[B2] UART OK\r\n\r\n");

    I2C1_Init();

    /* ---- HARDWARE I2C SCANNER ADDED TO PREVENT HANGING ---- */
    USART2_SendString("Scanning I2C Bus for BMP280...\r\n");
    if (I2C_Ping(0x76U)) {
        dev_addr = 0x76U;
        USART2_SendString(" -> SUCCESS: Sensor Found at Address 0x76 (SDO tied to GND)\r\n\n");
    } else if (I2C_Ping(0x77U)) {
        dev_addr = 0x77U;
        USART2_SendString(" -> SUCCESS: Sensor Found at Address 0x77 (SDO tied to VCC)\r\n\n");
    } else {
        USART2_SendString("\r\n**************************************************\r\n");
        USART2_SendString("HARDWARE FAILURE: NO I2C DEVICES RESPONDING!\r\n");
        USART2_SendString("1. The CS Pin MUST be connected to 3.3V.\r\n");
        USART2_SendString("2. You MUST have 4.7k pull-up resistors on SDA/SCL.\r\n");
        USART2_SendString("3. Check wires: PB6 -> SCL, PB7 -> SDA.\r\n");
        USART2_SendString("System Halting.\r\n");
        USART2_SendString("**************************************************\r\n");
        while(1); /* Stop here securely rather than freezing later */
    }

    USART2_SendString("Initialising BMP280...\r\n");
    uint8_t chip_id = BMP280_Init();

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

    delay_ms(100U);
    SampleAndCompensate();
    TIM6_Init();

    USART2_SendString("--- VERIFICATION TESTS ---\r\n");
    RunVerificationTests();
    USART2_SendString("--------------------------\r\n\r\n");
    USART2_SendString("--- LIVE SENSOR OUTPUT (1 Hz via TIM6 ISR) ---\r\n");

    while (1) { __WFI(); }
    return 0;
}

void TIM6_DAC_IRQHandler(void) {
    if (!(TIM6->SR & TIM_SR_UIF)) return;
    TIM6->SR &= ~TIM_SR_UIF;
    g_tick++;
    SampleAndCompensate();

    int32_t tc_w = g_comp_T / 100;
    int32_t tc_f = g_comp_T % 100; if (tc_f < 0) tc_f = -tc_f;
    int32_t tf_s = g_comp_T * 9 / 5 + 3200;
    int32_t tf_w = tf_s / 100;
    int32_t tf_f = tf_s % 100; if (tf_f < 0) tf_f = -tf_f;

    uint32_t pp  = g_comp_P / 256U;
    uint32_t phw = pp / 100U;
    uint32_t phf = pp % 100U;

    char msg[160];
    snprintf(msg, sizeof(msg), "[Tick:%4lu] Temp: %ld.%02ld C / %ld.%02ld F | Pres: %lu.%02lu hPa | Hum: N/A (BMP280)\r\n", (unsigned long)g_tick, (long)tc_w, (long)tc_f, (long)tf_w, (long)tf_f, (unsigned long)phw, (unsigned long)phf);
    USART2_SendString(msg);
}
