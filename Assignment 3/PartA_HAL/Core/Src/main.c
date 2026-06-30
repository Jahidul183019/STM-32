/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - BMP280 SPI2 Sensor Interface (HAL)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * CSE 2206 — Microcontroller & Embedded System Lab
  * Assignment 3, Part A: BMP280 Sensor Interface — HAL VERSION
  * Platform  : STM32F446RE Nucleo-64
  * Clock     : SYSCLK = 180 MHz (HSE PLL), APB1 = 45 MHz, APB2 = 90 MHz
  * Sensor    : BMP280 (hardware substitute for BME280)
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * BMP280 Calibration parameter structure
 */
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/**
 * Global sensor state
 */
static BMP280_Calib_t calib;
static int32_t        t_fine;           /* Shared between T and P compensation */

/* Sensor readings — written by ISR, read by verification/main */
static volatile int32_t  g_comp_T  = 0; /* Units: hundredths of °C (e.g. 2534 = 25.34 °C) */
static volatile uint32_t g_comp_P  = 0; /* Units: Q24.8 fixed-point Pa */
static volatile uint32_t g_tick    = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* BMP280 Functions */
static void BMP280_ReadCalibration(void);
static int32_t BMP280_Compensate_T(int32_t adc_T);
static uint32_t BMP280_Compensate_P(int32_t adc_P);
static void SampleAndCompensate(void);
static uint8_t BMP280_Init(void);
static void RunVerificationTests(void);

/* Helper Functions */
static void USART2_SendString(const char *s);
static uint8_t SPI2_TxRx(uint8_t data);
static void BMP280_WriteReg(uint8_t reg, uint8_t value);
static uint8_t BMP280_ReadReg(uint8_t reg);
static void BMP280_BurstRead(uint8_t start_reg, uint8_t *buf, uint8_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Millisecond Delay (HAL-based)
 */
static void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * USART2 Send String
 */
static void USART2_SendString(const char *s)
{
    while (*s)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)s, 1U, HAL_MAX_DELAY);
        s++;
    }
}

/**
 * Printf redirect for snprintf support
 */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)(ch & 0xFF);
    HAL_UART_Transmit(&huart2, &c, 1U, HAL_MAX_DELAY);
    return ch;
}

/**
 * SPI2 Transmit/Receive Single Byte
 */
static uint8_t SPI2_TxRx(uint8_t data)
{
    uint8_t rx_data = 0U;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1U, HAL_MAX_DELAY);
    return rx_data;
}

/**
 * BMP280 Write Register
 */
static void BMP280_WriteReg(uint8_t reg, uint8_t value)
{
    /* CS Low (active) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    SPI2_TxRx(reg & 0x7FU);    /* Write address (MSB=0 for write) */
    SPI2_TxRx(value);           /* Write data */

    /* Wait for SPI to finish */
    while (hspi2.Instance->SR & SPI_SR_BSY);

    /* CS High (inactive) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

/**
 * BMP280 Read Register
 */
static uint8_t BMP280_ReadReg(uint8_t reg)
{
    uint8_t val;

    /* CS Low (active) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    SPI2_TxRx(reg | 0x80U);     /* Read address (MSB=1 for read) */
    val = SPI2_TxRx(0x00U);     /* Read data */

    /* Wait for SPI to finish */
    while (hspi2.Instance->SR & SPI_SR_BSY);

    /* CS High (inactive) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    return val;
}

/**
 * BMP280 Burst Read (Multiple Bytes)
 */
static void BMP280_BurstRead(uint8_t start_reg, uint8_t *buf, uint8_t length)
{
    /* CS Low (active) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    SPI2_TxRx(start_reg | 0x80U);  /* Read address (MSB=1 for read) */

    for (uint8_t i = 0U; i < length; i++)
    {
        buf[i] = SPI2_TxRx(0xFFU);
    }

    /* Wait for SPI to finish */
    while (hspi2.Instance->SR & SPI_SR_BSY);

    /* CS High (inactive) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

/**
 * BMP280 Read Calibration Constants from NVM
 */
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

/**
 * BMP280 Temperature Compensation
 * Returns temperature in units of 0.01 °C (e.g., 2534 = 25.34 °C)
 */
static int32_t BMP280_Compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = (((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1)) * (int32_t)calib.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)calib.dig_T1) * ((adc_T >> 4) - (int32_t)calib.dig_T1)) >> 12) * (int32_t)calib.dig_T3) >> 14;

    t_fine = var1 + var2;
    T      = (t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * BMP280 Pressure Compensation
 * Returns pressure in units of Q24.8 Pa (divide by 256 for Pa, by 25600 for hPa)
 */
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

/**
 * Sample ADC and Apply Compensation
 */
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

/**
 * BMP280 Initialization Sequence
 * Returns: Chip ID if successful, 0 if failed
 */
static uint8_t BMP280_Init(void)
{
    uint8_t chip_id, status;
    uint32_t timeout;

    /* Soft Reset */
    BMP280_WriteReg(BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    delay_ms(10U);

    /* Wait for status bit to clear */
    timeout = 100000U;
    do {
        status = BMP280_ReadReg(BMP280_REG_STATUS);
        if (--timeout == 0U) break;
    } while (status & 0x01U);

    delay_ms(10U);

    /* Read and verify Chip ID */
    chip_id = BMP280_ReadReg(BMP280_REG_CHIP_ID);
    if (chip_id != BMP280_CHIP_ID_PRIMARY &&
        chip_id != BMP280_CHIP_ID_ALT1 &&
        chip_id != BMP280_CHIP_ID_ALT2)
    {
        return 0U;
    }

    /* Load calibration constants */
    BMP280_ReadCalibration();

    /* Configure operating mode */
    BMP280_WriteReg(BMP280_REG_CONFIG,    0x10U); /* IIR Filter = 16 */
    BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x57U); /* Temp x2, Pres x16, Normal Mode */

    return chip_id;
}

/**
 * Run Verification Tests (A1-A5)
 */
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
    if (htim6.Instance->CR1 & TIM_CR1_CEN)
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /* ============================================================
   * APPLICATION START
   * ============================================================ */
  USART2_SendString("\r\n========================================\r\n");
  USART2_SendString(" CSE 2206 - Assignment 3 Part A\r\n");
  USART2_SendString(" BMP280 via SPI2 - HAL (STM32F446RE)\r\n");
  USART2_SendString("========================================\r\n");
  USART2_SendString("System Clock: 180 MHz (PLL)\r\n");
  USART2_SendString("APB1: 45 MHz | APB2: 90 MHz\r\n\r\n");
  USART2_SendString("[A2] UART OK\r\n\r\n");

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
      snprintf(cbuf, sizeof(cbuf), "  dig_T1=%u  dig_T2=%d  dig_T3=%d\r\n",
               calib.dig_T1, calib.dig_T2, calib.dig_T3);
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

  /* Start TIM6 interrupt (1 Hz periodic) */
  HAL_TIM_Base_Start_IT(&htim6);

  USART2_SendString("--- VERIFICATION TESTS ---\r\n");
  RunVerificationTests();
  USART2_SendString("--------------------------\r\n\r\n");

  USART2_SendString("--- LIVE SENSOR OUTPUT (1 Hz via TIM6 ISR) ---\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    __WFI();  /* Wait for interrupt */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  *
  * CORRECTED: PSC = 8999 (45 MHz / 9000 = 5 kHz)
  *           ARR = 4999 (5000 / 5 kHz = 1 second)
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;  /* CORRECTED: Was 0, should be PSC-1 for 1 sec period */
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;     /* CORRECTED: Was 65535, should be ARR-1 for 1 sec period */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI2_CS_Pin (PB9) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * TIM6 Period Elapsed Callback (1 Hz interrupt)
 * This callback is called automatically by HAL when TIM6 interrupt fires
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
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
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
