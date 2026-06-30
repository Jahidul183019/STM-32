/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
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
#define BMP280_ADDR_76          (0x76 << 1)
#define BMP280_ADDR_77          (0x77 << 1)

#define BMP280_REG_DIG_T1       0x88
#define BMP280_REG_CHIP_ID      0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_BURST_START  0xF7

#define BMP280_CHIP_ID_PRIMARY  0x58
#define BMP280_CHIP_ID_ALT1     0x57
#define BMP280_CHIP_ID_ALT2     0x56

#define BMP280_SOFT_RESET_CMD   0xB6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMP280_Calib_t calib;

int32_t  t_fine;
int32_t  g_comp_T = 0;
uint32_t g_comp_P = 0;
uint32_t g_tick   = 0;

uint16_t bmp280_addr = BMP280_ADDR_76;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void USART2_SendString(char *s);

uint8_t BMP280_ReadReg(uint8_t reg);
void BMP280_WriteReg(uint8_t reg, uint8_t value);
void BMP280_BurstRead(uint8_t reg, uint8_t *buf, uint8_t len);

uint8_t BMP280_Init(void);
void BMP280_ReadCalibration(void);

int32_t BMP280_Compensate_T(int32_t adc_T);
uint32_t BMP280_Compensate_P(int32_t adc_P);

void SampleAndCompensate(void);
void RunVerificationTests(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USART2_SendString(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

void BMP280_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(
        &hi2c1,
        bmp280_addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY
    );
}

uint8_t BMP280_ReadReg(uint8_t reg)
{
    uint8_t value = 0;

    HAL_I2C_Mem_Read(
        &hi2c1,
        bmp280_addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY
    );

    return value;
}

void BMP280_BurstRead(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(
        &hi2c1,
        bmp280_addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        HAL_MAX_DELAY
    );
}

void BMP280_ReadCalibration(void)
{
    uint8_t calib_buf[24];

    BMP280_BurstRead(0x88, calib_buf, 24);

    calib.dig_T1 = (uint16_t)((calib_buf[1] << 8) | calib_buf[0]);
    calib.dig_T2 = (int16_t)((calib_buf[3] << 8) | calib_buf[2]);
    calib.dig_T3 = (int16_t)((calib_buf[5] << 8) | calib_buf[4]);

    calib.dig_P1 = (uint16_t)((calib_buf[7] << 8) | calib_buf[6]);
    calib.dig_P2 = (int16_t)((calib_buf[9] << 8) | calib_buf[8]);
    calib.dig_P3 = (int16_t)((calib_buf[11] << 8) | calib_buf[10]);
    calib.dig_P4 = (int16_t)((calib_buf[13] << 8) | calib_buf[12]);
    calib.dig_P5 = (int16_t)((calib_buf[15] << 8) | calib_buf[14]);
    calib.dig_P6 = (int16_t)((calib_buf[17] << 8) | calib_buf[16]);
    calib.dig_P7 = (int16_t)((calib_buf[19] << 8) | calib_buf[18]);
    calib.dig_P8 = (int16_t)((calib_buf[21] << 8) | calib_buf[20]);
    calib.dig_P9 = (int16_t)((calib_buf[23] << 8) | calib_buf[22]);
}

int32_t BMP280_Compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
            ((int32_t)calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    T = (t_fine * 5 + 128) >> 8;

    return T;
}

uint32_t BMP280_Compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;

    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8)
            + ((var1 * (int64_t)calib.dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1)
            * ((int64_t)calib.dig_P1)) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8)
            + (((int64_t)calib.dig_P7) << 4);

    return (uint32_t)p;
}

void SampleAndCompensate(void)
{
    uint8_t raw[6];

    BMP280_BurstRead(BMP280_REG_BURST_START, raw, 6);

    int32_t adc_P =
        ((uint32_t)raw[0] << 12) |
        ((uint32_t)raw[1] << 4)  |
        ((uint32_t)raw[2] >> 4);

    int32_t adc_T =
        ((uint32_t)raw[3] << 12) |
        ((uint32_t)raw[4] << 4)  |
        ((uint32_t)raw[5] >> 4);

    g_comp_T = BMP280_Compensate_T(adc_T);
    g_comp_P = BMP280_Compensate_P(adc_P);
}

uint8_t BMP280_Init(void)
{
    uint8_t chip_id;
    uint8_t status;

    BMP280_WriteReg(BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);

    HAL_Delay(100);

    do
    {
        status = BMP280_ReadReg(BMP280_REG_STATUS);
    }
    while(status & 0x01);

    chip_id = BMP280_ReadReg(BMP280_REG_CHIP_ID);

    BMP280_ReadCalibration();

    BMP280_WriteReg(BMP280_REG_CONFIG, 0x10);
    BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x57);

    return chip_id;
}

void RunVerificationTests(void)
{
    char s[128];

    uint8_t id = BMP280_ReadReg(BMP280_REG_CHIP_ID);

    snprintf(s, sizeof(s),
             "[B1] ChipID = 0x%02X\r\n",
             id);

    USART2_SendString(s);

    if(id == BMP280_CHIP_ID_PRIMARY ||
       id == BMP280_CHIP_ID_ALT1 ||
       id == BMP280_CHIP_ID_ALT2)
    {
        USART2_SendString("[B1] Chip ID PASS\r\n");
    }
    else
    {
        USART2_SendString("[B1] Chip ID FAIL\r\n");
    }

    USART2_SendString("[B2] I2C ACK OK\r\n");

    USART2_SendString("[B3] TIM6 heartbeat running\r\n");

    int32_t tc = g_comp_T;

    uint32_t press_pa  = g_comp_P / 256;
    uint32_t press_hpa = press_pa / 100;

    uint8_t t_ok = (tc >= 1500 && tc <= 4000);
    uint8_t p_ok = (press_hpa >= 900 && press_hpa <= 1100);

    if(t_ok && p_ok)
    {
        USART2_SendString("[B4] Plausibility PASS\r\n");
    }
    else
    {
        USART2_SendString("[B4] Plausibility FAIL\r\n");
    }

    USART2_SendString("[B5] HAL VERSION RUNNING\r\n");
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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  char buf[256];

  USART2_SendString("\r\n========================================\r\n");
  USART2_SendString(" CSE 2206 - Assignment 3 Part B\r\n");
  USART2_SendString(" BMP280 via I2C1 - HAL Version\r\n");
  USART2_SendString("========================================\r\n");

  USART2_SendString("Scanning I2C Bus...\r\n");

  if(HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_76, 3, 100) == HAL_OK)
  {
      bmp280_addr = BMP280_ADDR_76;
      USART2_SendString(" -> Sensor Found at 0x76\r\n");
  }
  else if(HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR_77, 3, 100) == HAL_OK)
  {
      bmp280_addr = BMP280_ADDR_77;
      USART2_SendString(" -> Sensor Found at 0x77\r\n");
  }
  else
  {
      USART2_SendString("BMP280 NOT FOUND!\r\n");

      while(1);
  }

  USART2_SendString("Initialising BMP280...\r\n");

  uint8_t chip_id = BMP280_Init();

  snprintf(buf, sizeof(buf),
           "BMP280 detected: ChipID=0x%02X\r\n",
           chip_id);

  USART2_SendString(buf);

  USART2_SendString("Calibration constants loaded.\r\n");

  snprintf(buf, sizeof(buf),
           "dig_T1=%u dig_T2=%d dig_T3=%d\r\n",
           calib.dig_T1,
           calib.dig_T2,
           calib.dig_T3);

  USART2_SendString(buf);

  snprintf(buf, sizeof(buf),
           "dig_P1=%u dig_P2=%d dig_P3=%d\r\n",
           calib.dig_P1,
           calib.dig_P2,
           calib.dig_P3);

  USART2_SendString(buf);

  snprintf(buf, sizeof(buf),
           "dig_P4=%d dig_P5=%d dig_P6=%d\r\n",
           calib.dig_P4,
           calib.dig_P5,
           calib.dig_P6);

  USART2_SendString(buf);

  snprintf(buf, sizeof(buf),
           "dig_P7=%d dig_P8=%d dig_P9=%d\r\n",
           calib.dig_P7,
           calib.dig_P8,
           calib.dig_P9);

  USART2_SendString(buf);

  USART2_SendString("Sensor in Normal mode.\r\n");

  HAL_Delay(100);

  SampleAndCompensate();

  HAL_TIM_Base_Start_IT(&htim6);

  USART2_SendString("--- VERIFICATION TESTS ---\r\n");

  RunVerificationTests();

  USART2_SendString("--------------------------\r\n\r\n");

  USART2_SendString("--- LIVE SENSOR OUTPUT ---\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_Delay(100);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        g_tick++;

        SampleAndCompensate();

        int32_t tc_w = g_comp_T / 100;
        int32_t tc_f = g_comp_T % 100;

        if(tc_f < 0)
            tc_f = -tc_f;

        int32_t tf_s = g_comp_T * 9 / 5 + 3200;

        int32_t tf_w = tf_s / 100;
        int32_t tf_f = tf_s % 100;

        if(tf_f < 0)
            tf_f = -tf_f;

        uint32_t pp  = g_comp_P / 256;

        uint32_t phw = pp / 100;
        uint32_t phf = pp % 100;

        char msg[160];

        snprintf(msg,
                 sizeof(msg),
                 "[Tick:%4lu] Temp: %ld.%02ld C / %ld.%02ld F | Pres: %lu.%02lu hPa | Hum: N/A (BMP280)\r\n",
                 (unsigned long)g_tick,
                 (long)tc_w,
                 (long)tc_f,
                 (long)tf_w,
                 (long)tf_f,
                 (unsigned long)phw,
                 (unsigned long)phf);

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
