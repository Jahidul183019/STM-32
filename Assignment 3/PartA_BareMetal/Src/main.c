/*
 * ============================================================
 * University of Dhaka — Department of CSE
 * CSE 2206 — Microcontroller & Embedded System Lab
 * Assignment 3: BME280 via SPI — Bare-Metal Implementation
 * Platform : STM32F446RE Nucleo-64
 * Clock    : SYSCLK = 180 MHz, APB1 = 45 MHz, APB2 = 90 MHz
 * ============================================================
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f446xx.h>

/* BME280 Register Address Map (Step A3.6) */
#define BME280_REG_DIG_T1       0x88
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_BURST_START  0xF7

/* Calibration Parameters Structure */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4;
    int16_t  dig_P5; int16_t dig_P6; int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
    uint08_t dig_H1; int16_t dig_H2; uint08_t dig_H3; int16_t dig_H4; int16_t dig_H5; int08_t dig_H6;
} BME280_Calib_t;

static BME280_Calib_t calib;
static int32_t t_fine;

/* Volatile Globals for Inter-process tracking */
static uint8_t sensor_data[8];
float temp_C, temp_F, pres_hPa, hum_RH;

void delay_ms(uint32_t ms) {
    uint32_t count = ms * 30000;
    while (count--) { __NOP(); }
}

/* =========================================================
 * A3.1 — Clock Configuration (180 MHz HSE-PLL)
 * ========================================================= */
void SystemClock_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    RCC->PLLCFGR  = 0;
    RCC->PLLCFGR |= (8   << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (180 << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0   << RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR |= (2   << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));
    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  /* AHB = 180MHz */
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; /* APB1 = 45MHz */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; /* APB2 = 90MHz */

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* =========================================================
 * A3.2 & A3.3 — GPIO & USART2 Configuration
 * ========================================================= */
void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER  &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER  |=  ((2U << (2*2)) | (2U << (3*2)));
    GPIOA->AFR[0] &= ~((0xFU << (4*2)) | (0xFU << (4*3)));
    GPIOA->AFR[0] |=  ((7U   << (4*2)) | (7U   << (4*3)));
    GPIOA->OSPEEDR|=  ((3U << (2*2)) | (3U << (3*2)));

    USART2->BRR = (24U << 4) | 7U;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART_SendString(const char *s) {
    while (*s) {
        while (!(USART2->SR & (1 << 7))); /* Wait TXE */
        USART2->DR = (uint8_t)(*s++);
    }
}

int __io_putchar(int ch) {
    while (!(USART2->SR & (1 << 7)));
    USART2->DR = (ch & 0xFF);
    return ch;
}

/* =========================================================
 * A3.5 — SPI2 Configuration (Lab Specification Prescaler)
 * ========================================================= */
void SPI2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PC1: MOSI (AF7), PC2: MISO (AF5), PC7: SCK (AF5) */
    GPIOC->MODER  &= ~((3U << (1*2)) | (3U << (2*2)) | (3U << (7*2)));
    GPIOC->MODER  |=  ((2U << (1*2)) | (2U << (2*2)) | (2U << (7*2)));
    GPIOC->AFR[0] &= ~((0xFU << (4*1)) | (0xFU << (4*2)) | (0xFU << (4*7)));
    GPIOC->AFR[0] |=  ((7U   << (4*1)) | (5U   << (4*2)) | (5U   << (4*7)));
    GPIOC->OSPEEDR |= ((3U << (1*2)) | (3U << (2*2)) | (3U << (7*2)));

    /* PB9: CS Output */
    GPIOB->MODER  &= ~(3U << (9*2));
    GPIOB->MODER  |=  (1U << (9*2));
    GPIOB->OSPEEDR|=  (3U << (9*2));
    GPIOB->ODR    |=  (1 << 9); /* High */

    /* Match manual Step A3.5 bit expressions exactly */
    SPI2->CR1 = (1 << 2)   /* MSTR = 1 */
              | (2 << 3)   /* BR = 010 (fPCLK / 8) */
              | (0 << 1)   /* CPOL = 0 */
              | (0 << 0)   /* CPHA = 0 (Mode 00) */
              | (0 << 7)   /* LSBFIRST = 0 */
              | (1 << 9)   /* SSM = 1 */
              | (1 << 8)   /* SSI = 1 */
              | (0 << 11); /* DFF = 0 (8-bit) */

    SPI2->CR1 |= (1 << 6); /* SPE = 1 */
}

/* =========================================================
 * A4 — Low-Level Functions
 * ========================================================= */
uint8_t SPI_TxRx(uint8_t data) {
    while (!(SPI2->SR & (1 << 1))); /* Wait TXE */
    SPI2->DR = data;
    while (!(SPI2->SR & (1 << 0))); /* Wait RXNE */
    return (uint8_t)SPI2->DR;
}

void BME280_SPI_WriteReg(uint8_t reg, uint8_t data) {
    GPIOB->ODR &= ~(1 << 9); /* CS LOW */
    SPI_TxRx(reg & 0x7F);    /* MSB = 0 */
    SPI_TxRx(data);
    GPIOB->ODR |= (1 << 9);  /* CS HIGH */
}

void BME280_SPI_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    GPIOB->ODR &= ~(1 << 9); /* CS LOW */
    SPI_TxRx(reg | 0x80);    /* MSB = 1 */
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI_TxRx(0xFF);
    }
    GPIOB->ODR |= (1 << 9);  /* CS HIGH */
}

/* =========================================================
 * A3.6 — Calibration Constants Mapping Parsing
 * ========================================================= */
void BME280_ReadCalibration(void) {
    uint8_t buf[24];
    BME280_SPI_ReadRegs(BME280_REG_DIG_T1, buf, 24);
    calib.dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
    calib.dig_T2 = (int16_t)((buf[3] << 8)  | buf[2]);
    calib.dig_T3 = (int16_t)((buf[5] << 8)  | buf[4]);
    calib.dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
    calib.dig_P2 = (int16_t)((buf[9] << 8)  | buf[8]);
    calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

    calib.dig_H1 = BME280_SPI_ReadRegs(0xA1, buf, 1), buf[0];
    BME280_SPI_ReadRegs(0xE1, buf, 7);
    calib.dig_H2 = (int16_t)((buf[1] << 8) | buf[0]);
    calib.dig_H3 = buf[2];
    calib.dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
    calib.dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));
    calib.dig_H6 = (int08_t)buf[6];
}

/* Data Compensation Algorithms (Step A3.7) */
int32_t BME280_Compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t BME280_Compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 31);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)p;
}

uint32_t BME280_Compensate_H(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

/* =========================================================
 * A3.4 — Hardware TIM6 (1-Second Telemetry Execution Interrupt)
 * ========================================================= */
void TIM6_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 9000 - 1;       /* 90 MHz APB1 target adjustments */
    TIM6->ARR = 10000 - 1;      /* 1 Second trigger profile matching manual values */
    TIM6->DIER |= TIM6_DIER_UIE;
    TIM6->CR1 |= TIM6_CR1_CEN;

    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM6_SR_UIF) {
        TIM6->SR &= ~TIM6_SR_UIF; /* Clear flag */

        /* Recover raw sensor arrays (Step A3.7) */
        BME280_SPI_ReadRegs(BME280_REG_BURST_START, sensor_data, 8);

        /* Apply exact casting shifts directly matching the manual formulas */
        int32_t adc_P = (int32_t)((((uint32_t)sensor_data[0]) << 12) | (((uint32_t)sensor_data[1]) << 4) | (((uint32_t)sensor_data[2]) >> 4));
        int32_t adc_T = (int32_t)((((uint32_t)sensor_data[3]) << 12) | (((uint32_t)sensor_data[4]) << 4) | (((uint32_t)sensor_data[5]) >> 4));
        int32_t adc_H = (int32_t)((((uint32_t)sensor_data[6]) << 8)  | ((uint32_t)sensor_data[7]));

        /* Execute Bosch compensation equations */
        int32_t comp_T = BME280_Compensate_T(adc_T);
        uint32_t comp_P = BME280_Compensate_P(adc_P);
        uint32_t comp_H = BME280_Compensate_H(adc_H);

        /* Precise casting down directly to matching parameters fractions */
        temp_C = (float)comp_T / 100.0f;
        temp_F = (temp_C * 9.0f / 5.0f) + 32.0f;
        pres_hPa = (float)comp_P / 256.0f / 100.0f;
        hum_RH = (float)comp_H / 1024.0f;

        /* Print formatting payload string matching Step A4.4 perfectly */
        char msg[128];
        sprintf(msg, "[SPI] Temp: %.2fC / %.2fF | Pres: %.2fhPa | Hum: %.2f%%\r\n",
                temp_C, temp_F, pres_hPa, hum_RH);

        UART_SendString(msg);
    }
}

/* =========================================================
 * Main Core Initialization Block
 * ========================================================= */
int main(void) {
    SystemClock_Config();
    USART2_Init();
    SPI2_Init();

    UART_SendString("Initializing BME280 Base Core...\r\n");

    /* Soft Reset Configuration sequence (Step A3.6) */
    BME280_SPI_WriteReg(BME280_REG_RESET, 0xB6);
    delay_ms(10);

    /* Load Calibration matrices vectors */
    BME280_ReadCalibration();

    /* Write register sequence directly matching your assignment values */
    BME280_SPI_WriteReg(BME280_REG_CTRL_HUM, 0x01);   /* ctrl_hum: x1 oversampling */
    BME280_SPI_WriteReg(BME280_REG_CTRL_MEAS, 0x57);  /* ctrl_meas */
    BME280_SPI_WriteReg(BME280_REG_CONFIG, 0x10);     /* config */

    UART_SendString("BME280 Operational Mode Engaged. Arming TIM6...\r\n");

    /* Turn on hardware interrupt trigger */
    TIM6_Init();

    while (1) {
        __WFI(); /* Sleep mode idle state saving power */
    }
}
