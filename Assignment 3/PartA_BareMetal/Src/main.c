/*
 * ============================================================
 * CSE 2206 — Microcontroller & Embedded System Lab
 * Assignment 3, Part A: BMP280 Sensor Interface — BARE-METAL
 * Platform : STM32F446RE Nucleo-64
 * Clock    : SYSCLK = 180 MHz, APB1 = 45 MHz, APB2 = 90 MHz
 * Target   : BMP280 (Modified from BME280 template)
 * SPI Mode : Mode 00 (CPOL=0, CPHA=0) as per Lab Mandate
 * ============================================================
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f446xx.h>   /* CMSIS device header */

/* BMP280 Register Address Map */
#define BMP280_REG_DIG_T1       0x88
#define BMP280_REG_DIG_T2       0x8A
#define BMP280_REG_DIG_T3       0x8C
#define BMP280_REG_DIG_P1       0x8E
#define BMP280_REG_DIG_P2       0x90
#define BMP280_REG_DIG_P3       0x92
#define BMP280_REG_DIG_P4       0x94
#define BMP280_REG_DIG_P5       0x96
#define BMP280_REG_DIG_P6       0x98
#define BMP280_REG_DIG_P7       0x9A
#define BMP280_REG_DIG_P8       0x9C
#define BMP280_REG_DIG_P9       0x9E
#define BMP280_REG_CHIP_ID      0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_BURST_START  0xF7  /* Pressure MSB start point */

/* Calibration parameters structure (BMP280 has no Humidity parameters) */
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

static BMP280_Calib_t calib;
static int32_t t_fine; /* Global variable used for pressure math translation */

/* Microsecond and millisecond loops calibrated for 180 MHz core clock */
void delay_us(uint32_t us)
{
    uint32_t count = us * 30;
    while (count--) {
        __NOP();
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

/* =========================================================
 * SECTION 0 — System Clock Configuration (180 MHz HSE-PLL)
 * ========================================================= */
void SystemClock_Config(void)
{
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
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI);
    RCC->PLLCFGR |= (2   << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));

    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; /* APB1 = 45 MHz */
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; /* APB2 = 90 MHz */

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |=  RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* =========================================================
 * SECTION 1 — USART2 Configuration (115200 Baud)
 * ========================================================= */
static void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2: TX, PA3: RX assigned to Alternative Function 7 */
    GPIOA->MODER  &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER  |=  ((2U << (2*2)) | (2U << (3*2)));

    GPIOA->AFR[0] &= ~((0xFU << (4*2)) | (0xFU << (4*3)));
    GPIOA->AFR[0] |=  ((7U   << (4*2)) | (7U   << (4*3)));

    USART2->BRR = (24U << 4) | 7U; /* 45 MHz APB1 clock target */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
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
    USART2->DR = (ch & 0xFF);
    return ch;
}

/* =========================================================
 * SECTION 3 — SPI2 Configuration (Lab Specification Mandate)
 * ========================================================= */
static void SPI2_Init(void)
{
    /* Enable clocks for Port B, Port C and SPI2 Peripheral */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PC1: SPI2_MOSI (AF7) */
    GPIOC->MODER  &= ~(3U << (1*2));
    GPIOC->MODER  |=  (2U << (1*2));
    GPIOC->AFR[0] &= ~(0xFU << (4*1));
    GPIOC->AFR[0] |=  (7U   << (4*1));

    /* PC2: SPI2_MISO (AF5) */
    GPIOC->MODER  &= ~(3U << (2*2));
    GPIOC->MODER  |=  (2U << (2*2));
    GPIOC->AFR[0] &= ~(0xFU << (4*2));
    GPIOC->AFR[0] |=  (5U   << (4*2));

    /* PC7: SPI2_SCK (AF5) */
    GPIOC->MODER  &= ~(3U << (7*2));
    GPIOC->MODER  |=  (2U << (7*2));
    GPIOC->AFR[0] &= ~(0xFU << (4*7));
    GPIOC->AFR[0] |=  (5U   << (4*7));

    /* Apply High Speed '11' to OSPEEDR for sharp transition square waves on 180MHz clock */
    GPIOC->OSPEEDR |= ((3U << (1*2)) | (3U << (2*2)) | (3U << (7*2)));

    /* PB9: GPIO Output for CS Line (Active Low) */
    GPIOB->MODER  &= ~(3U << (9*2));
    GPIOB->MODER  |=  (1U << (9*2));   /* Output Mode */
    GPIOB->OSPEEDR|=  (3U << (9*2));   /* High speed drive */
    GPIOB->BSRR   =  (1U << 9);        /* CS High (Unselected) */

    /* SPI2 Setup Configuration */
    SPI2->CR1 = 0;
    SPI2->CR1 |= SPI_CR1_MSTR;                 /* Set Master Mode */

    /* Baud rate divider: fPCLK / 32
     * APB1 clock = 45 MHz. Divider /32 = 1.4 MHz (Ensures safe transmission on breadboards) */
    SPI2->CR1 |= (4 << SPI_CR1_BR_Pos);

    /* MANDATED LAB CORRECTION: SPI Mode 00 (CPOL=0, CPHA=0)
     * Clear both bits explicitly to ensure conformity with manual assignment guidelines */
    SPI2->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;    /* Enable Software Slave Management */
    SPI2->CR1 |= SPI_CR1_SPE;                  /* Enable Peripheral hardware */
}

/* Low-level single byte SPI exchange over SPI2 */
static uint8_t SPI2_Transceive(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE));          /* Check that transmit buffer is empty */
    SPI2->DR = data;                           /* Load payload byte */
    while (!(SPI2->SR & SPI_SR_RXNE));         /* Await incoming data buffer */
    return SPI2->DR;                           /* Pop out and return read byte */
}

/* BMP280 SPI Register Write: Mask Address bit 7 to Low */
static void BMP280_SPI_WriteReg(uint8_t reg, uint8_t value)
{
    GPIOB->BSRR = (1U << (9 + 16));            /* Drive CS Low */
    SPI2_Transceive(reg & 0x7F);               /* Address write modification control bit */
    SPI2_Transceive(value);                    /* Dispatch state code */
    while (SPI2->SR & SPI_SR_BSY);             /* Block execution until flag drops */
    GPIOB->BSRR = (1U << 9);                   /* Reset CS to High */
}

/* BMP280 SPI Register Read: Assert Address bit 7 to High */
static uint8_t BMP280_SPI_ReadReg(uint8_t reg)
{
    uint8_t val;
    GPIOB->BSRR = (1U << (9 + 16));            /* Drive CS Low */
    SPI2_Transceive(reg | 0x80);               /* Read command identifier modification */
    val = SPI2_Transceive(0x00);               /* Run dummy clock cycle to clear buffer */
    while (SPI2->SR & SPI_SR_BSY);
    GPIOB->BSRR = (1U << 9);                   /* Reset CS to High */
    return val;
}

/* Sequential multi-byte packet recovery across memory register offsets */
static void BMP280_SPI_BurstRead(uint8_t start_reg, uint8_t *buffer, uint16_t length)
{
    GPIOB->BSRR = (1U << (9 + 16));            /* Drive CS Low */
    SPI2_Transceive(start_reg | 0x80);
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = SPI2_Transceive(0x00);     /* Read register while address auto-increments */
    }
    while (SPI2->SR & SPI_SR_BSY);
    GPIOB->BSRR = (1U << 9);                   /* Reset CS to High */
}

/* =========================================================
 * SECTION 4 — BMP280 Calibration Mapping & Compensation Data
 * ========================================================= */
static void BMP280_ReadCalibration(void)
{
    uint8_t buf[24];
    BMP280_SPI_BurstRead(BMP280_REG_DIG_T1, buf, 24);

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
}

static int32_t BMP280_Compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;
    if (calib.dig_T1 == 0) return 0; /* Guard against zero configurations */

    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t BMP280_Compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    if (calib.dig_P1 == 0) return 0; /* Guard against uninitialized math matrices */

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 31);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; /* Clear divide by zero fault risks */
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)p;
}

/* =========================================================
 * SECTION 5 — Main Program Flow
 * ========================================================= */
int main(void)
{
    SystemClock_Config();
    USART2_Init();
    SPI2_Init(); /* Remapped SPI bus interface initialization */

    USART2_SendString("\r\n========================================\r\n");
    USART2_SendString("STM32F446RE - Assignment 3 Part A\r\n");
    USART2_SendString("BMP280 Sensor Interface - Bare Metal SPI2\r\n");
    USART2_SendString("========================================\r\n");

    /* Read and validate the identity of a BMP280 sensor module */
    uint8_t chip_id = BMP280_SPI_ReadReg(BMP280_REG_CHIP_ID);
    printf("Reading BMP280 CHIP ID...\r\n");
    printf("CHIP ID Recieved: 0x%02X (Expected: 0x58, 0x57, or 0x56)\r\n", chip_id);

    if (chip_id != 0x58 && chip_id != 0x57 && chip_id != 0x56) {
        printf("ERROR: Unexpected Chip ID! Halting program...\r\n");
        while(1);
    }
    printf("BMP280 Communication verified successfully!\r\n");

    /* Issue Soft Reset */
    BMP280_SPI_WriteReg(BMP280_REG_RESET, 0xB6);

    /* CRITICAL FIX: Give the sensor 300ms to load its calibration variables
     * from internal factory NVM back to digital mirrors before parsing */
    delay_ms(300);

    /* Parse NVM Trimming Constants */
    BMP280_ReadCalibration();
    printf("Calibration matrices parsed from sensor NVM.\r\n");

    /* config (0xF5): Standby 0.5ms, IIR filter coefficient = 16
     * Value = (0 << 5) | (4 << 2) = 0x10 */
    BMP280_SPI_WriteReg(BMP280_REG_CONFIG, 0x10);

    /* ctrl_meas (0xF4): Pressure x16, Temp x2, Normal mode operational loop state
     * Value = (0x02 << 5) | (0x05 << 2) | 0x03 = 0x57 */
    BMP280_SPI_WriteReg(BMP280_REG_CTRL_MEAS, 0x57);
    printf("Sensor runtime metrics updated to Normal Operation.\r\n\r\n");

    uint8_t sensor_data[6];
    int32_t adc_P, adc_T;
    int32_t comp_T;
    uint32_t comp_P;

    while (1)
        {
            /* Burst read 6 continuous environmental data bytes from register 0xF7 */
            BMP280_SPI_BurstRead(BMP280_REG_BURST_START, sensor_data, 6);

            /* Reconstruct Raw Registers */
            adc_P = (int32_t)((((uint32_t)sensor_data[0]) << 12) | (((uint32_t)sensor_data[1]) << 4) | (((uint32_t)sensor_data[2]) >> 4));
            adc_T = (int32_t)((((uint32_t)sensor_data[3]) << 12) | (((uint32_t)sensor_data[4]) << 4) | (((uint32_t)sensor_data[5]) >> 4));

            /* Compute Compensated Metrics */
            comp_T = BMP280_Compensate_T(adc_T);
            comp_P = BMP280_Compensate_P(adc_P);

            // --- MOCK-FLOAT PARSING LOGIC ---
            // Temperature Math (e.g., 2532 becomes Whole: 25, Fraction: 32)
            int32_t temp_whole = comp_T / 100;
            int32_t temp_fraction = comp_T % 100;
            if (temp_fraction < 0) temp_fraction = -temp_fraction; // Handle sub-zero temperatures safely

            // Pressure Math: Convert fixed-point Q24.8 to standard hPa with 2 decimal places
            // comp_P / 256 gives Pascal integers. Divide by 100 to get hPa.
            uint32_t press_pascal = comp_P / 256;
            uint32_t press_hpa_whole = press_pascal / 100;
            uint32_t press_hpa_fraction = press_pascal % 100;

            // This statement behaves EXACTLY like a float specifier, but will NEVER freeze or crash.
            printf("Environment -> Temp: %ld.%02ld *C | Press: %lu.%02lu hPa\r\n",
                   temp_whole, temp_fraction, press_hpa_whole, press_hpa_fraction);

            delay_ms(1000);
        }

    return 0;
}
