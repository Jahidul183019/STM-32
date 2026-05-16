/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CSE 2206 Lab-02 Task 4 — TIM3 PWM @ 1 kHz on PA6 (HAL)
  *                   180 MHz SYSCLK, TIM6 1 us tick for delay, USART2 reporting
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM3_ARR    999U      /* matches CubeMX TIM3 Period setting */
#define LUT_SIZE    256U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Sine LUT, 256 entries scaled 0..100. Identical to the bare-metal table.
 * LUT[i] = round( 50 * (1 + sin(2*pi*i/256)) )                  (Algo A4.3) */
static const uint8_t sine_lut[LUT_SIZE] = {
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN 0 */

/* UART helper -- blocking transmit using HAL */
static void USART2_SendString(const char *s)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* TIM6 microsecond / millisecond delay (1 us tick from CubeMX config) */
static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) < us) { /* wait */ }
}

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) delay_us(1000U);
}

/* Algorithm A4.2 -- runtime duty cycle, glitch-free via OC1PE preload */
static void PWM_SetDuty(uint8_t pct)
{
    if (pct > 100U) pct = 100U;
    uint32_t ccr = (uint32_t)pct * (TIM3_ARR + 1U) / 100U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /* Start TIM6 (delay base) and TIM3 PWM on Channel 1 (PA6) */
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  char buf[100];

  USART2_SendString("\r\n*** Lab-02 | Task 4: TIM3 PWM Generation (HAL) ***\r\n");
  USART2_SendString("---------------------------------------------------\r\n\r\n");

  /* --- Part 1: Duty-Cycle Sweep --- */
  USART2_SendString("[1] Duty-Cycle Sweep (0% to 100%, step 10%):\r\n");
  USART2_SendString("    Duty  |  CCR1\r\n");
  USART2_SendString("    ------|------\r\n");

  for (int d = 0; d <= 100; d += 10)
  {
      PWM_SetDuty((uint8_t)d);
      snprintf(buf, sizeof(buf), "    %3d%%  |  %lu\r\n",
               d, (unsigned long)TIM3->CCR1);
      USART2_SendString(buf);
      delay_ms(300U);
  }
  USART2_SendString("[1] Sweep complete.\r\n\r\n");

  /* --- Part 2: Sine-wave breathing effect, 5 cycles --- */
  USART2_SendString("[2] Sine-wave breathing effect: 5 cycles (~2s each)\r\n");
  for (int cycle = 1; cycle <= 5; cycle++)
  {
      snprintf(buf, sizeof(buf), "    Breath cycle %d / 5 ...\r\n", cycle);
      USART2_SendString(buf);
      for (int i = 0; i < (int)LUT_SIZE; i++)
      {
          PWM_SetDuty(sine_lut[i]);
          delay_ms(8U);
      }
  }
  USART2_SendString("[2] Breathing sequence complete.\r\n\r\n");

  /* --- Part 3: Final hold at exactly 50% --- */
  PWM_SetDuty(50U);
  snprintf(buf, sizeof(buf),
           "[3] Final hold: Duty = 50%%  |  CCR1 = %lu\r\n\r\n",
           (unsigned long)TIM3->CCR1);
  USART2_SendString(buf);

  USART2_SendString("*** All demonstrations complete. ***\r\n");
  /* USER CODE END 2 */

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration -- HSI -> PLL -> 180 MHz.
  *
  * fSYSCLK = 16 MHz (HSI) * 180 / (8 * 2) = 180 MHz
  * APB1 = HCLK / 4 = 45 MHz  -> APB1 timer clock = 90 MHz (TIM3, TIM6)
  * APB2 = HCLK / 2 = 90 MHz  -> APB2 timer clock = 90 MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)  { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)            { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  { Error_Handler(); }
}

/**
  * @brief TIM3 -- 1 kHz PWM on CH1 (PA6).
  *        90 MHz / (89+1) = 1 MHz tick; 1 MHz / (999+1) = 1 kHz.
  *        ARPE enabled (auto-reload preload).
  *        OC1PE (output compare preload) is enabled automatically by
  *        HAL_TIM_PWM_ConfigChannel() on F4 -- no struct field exists.
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;   /* ARPE = 1 */
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)              { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  { Error_Handler(); }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)               { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  { Error_Handler(); }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;             /* OC1M = 110 */
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;         /* CC1P  = 0  */
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  /* OC1PE is auto-enabled by HAL_TIM_PWM_ConfigChannel on F4 */
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM6 -- 1 us tick base for delay_us / delay_ms.
  *        90 MHz / (89+1) = 1 MHz.
  */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)              { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  { Error_Handler(); }
}

/**
  * @brief USART2 -- 115200 8N1.
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)                 { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* LD2 (PA5) -- optional reference toggle pin */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add reporting here */
}
#endif /* USE_FULL_ASSERT */
