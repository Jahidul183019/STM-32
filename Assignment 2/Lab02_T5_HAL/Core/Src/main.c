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
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    uint8_t r, g, b;
} WS_RGB;

typedef struct
{
    const char *name;
    uint8_t r, g, b;
} Palette_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_COUNT              5

#define WS2812_BITS_PER_LED    24
#define WS2812_RESET_SLOTS     50

#define WS_ARR                 225
#define WS_T1H                 150
#define WS_T0H                 75

#define PWM_BUFFER_SIZE ((LED_COUNT * WS2812_BITS_PER_LED) + WS2812_RESET_SLOTS)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

WS_RGB leds[LED_COUNT];

static const Palette_t palette[] =
{
    {"Red",255,0,0},
    {"Green",0,255,0},
    {"Blue",0,0,255},
    {"Yellow",255,255,0},
    {"Cyan",0,255,255},
    {"Magenta",255,0,255},
    {"White",255,255,255},
    {"Warm White",255,200,80},
    {"DU Blue",31,56,100},
    {"Off",0,0,0}
};

uint16_t pwmData[PWM_BUFFER_SIZE];

volatile uint8_t datasentflag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void UART_Print(char *msg);
void UART_Printf(const char *fmt, ...);

void delay_us(uint32_t us);

void WS2812_SetColor(uint8_t r,
                     uint8_t g,
                     uint8_t b);

void WS2812_SetChain(WS_RGB *leds,
                     uint8_t n);

void WS2812_Send(void);

WS_RGB HSV_ToRGB(uint16_t h);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)msg,
                      strlen(msg),
                      HAL_MAX_DELAY);
}

void UART_Printf(const char *fmt, ...)
{
    char buf[256];

    va_list args;

    va_start(args, fmt);

    vsnprintf(buf, sizeof(buf), fmt, args);

    va_end(args);

    UART_Print(buf);
}

/* ===================================================== */
/* MICROSECOND DELAY */
/* ===================================================== */

void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    while(__HAL_TIM_GET_COUNTER(&htim6) < us);
}

/* ===================================================== */
/* WS2812 SEND */
/* ===================================================== */

void WS2812_Send(void)
{
    datasentflag = 0;

    HAL_TIM_PWM_Start_DMA(&htim1,
                          TIM_CHANNEL_1,
                          (uint32_t*)pwmData,
                          PWM_BUFFER_SIZE);

    while(!datasentflag);

    delay_us(60);
}

/* ===================================================== */
/* SEND SINGLE LED */
/* ===================================================== */

void WS2812_SetColor(uint8_t r,
                     uint8_t g,
                     uint8_t b)
{
    for(uint8_t i = 0; i < LED_COUNT; i++)
    {
        leds[i].r = r;
        leds[i].g = g;
        leds[i].b = b;
    }

    WS2812_SetChain(leds, LED_COUNT);
}

/* ===================================================== */
/* SEND LED CHAIN */
/* ===================================================== */

void WS2812_SetChain(WS_RGB *leds,
                     uint8_t n)
{
    uint32_t indx = 0;

    for(uint8_t led = 0; led < n; led++)
    {
        uint32_t color =
                ((uint32_t)leds[led].g << 16) |
                ((uint32_t)leds[led].r << 8 ) |
                ((uint32_t)leds[led].b);

        for(int8_t bit = 23; bit >= 0; bit--)
        {
            if(color & (1 << bit))
            {
                pwmData[indx] = WS_T1H;
            }
            else
            {
                pwmData[indx] = WS_T0H;
            }

            indx++;
        }
    }

    while(indx < PWM_BUFFER_SIZE)
    {
        pwmData[indx] = 0;
        indx++;
    }

    WS2812_Send();
}

/* ===================================================== */
/* DMA COMPLETE CALLBACK */
/* ===================================================== */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);

        datasentflag = 1;
    }
}

/* ===================================================== */
/* HSV TO RGB */
/* ===================================================== */

WS_RGB HSV_ToRGB(uint16_t h)
{
    WS_RGB c;

    uint8_t seg  = h / 60;
    uint8_t frac = h % 60;

    uint8_t q = 255 * (60 - frac) / 60;
    uint8_t t = 255 * frac / 60;

    switch(seg)
    {
        case 0:
            c.r = 255;
            c.g = t;
            c.b = 0;
            break;

        case 1:
            c.r = q;
            c.g = 255;
            c.b = 0;
            break;

        case 2:
            c.r = 0;
            c.g = 255;
            c.b = t;
            break;

        case 3:
            c.r = 0;
            c.g = q;
            c.b = 255;
            break;

        case 4:
            c.r = t;
            c.g = 0;
            c.b = 255;
            break;

        default:
            c.r = 255;
            c.g = 0;
            c.b = q;
            break;
    }

    return c;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);

  __HAL_TIM_MOE_ENABLE(&htim1);

  UART_Print("\r\n====================================\r\n");
  UART_Print(" WS2812 HAL TASK\r\n");
  UART_Print(" TIM1 CH1 -> PA8\r\n");
  UART_Print("====================================\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      /* ========================================= */
      /* (1) PALETTE DEMO - ALL 5 LEDs */
      /* ========================================= */

      UART_Print("\r\n--- Palette Demo ---\r\n");

      for(int i = 0;
          i < (sizeof(palette) / sizeof(palette[0]));
          i++)
      {
          WS2812_SetColor(palette[i].r,
                          palette[i].g,
                          palette[i].b);

          UART_Printf(
              "Colour: %-12s R=%3d G=%3d B=%3d GRB=[%02X %02X %02X]\r\n",
              palette[i].name,
              palette[i].r,
              palette[i].g,
              palette[i].b,
              palette[i].g,
              palette[i].r,
              palette[i].b
          );

          HAL_Delay(1000);
      }

      /* ========================================= */
      /* (2) HUE SWEEP - ALL 5 LEDs */
      /* ========================================= */

      UART_Print("\r\n--- Hue Sweep ---\r\n");

      for(uint16_t h = 0; h < 360; h += 3)
      {
          WS_RGB c = HSV_ToRGB(h);

          WS2812_SetColor(c.r,
                          c.g,
                          c.b);

          UART_Printf("H=%3d R=%3d G=%3d B=%3d\r\n",
                      h,
                      c.r,
                      c.g,
                      c.b);

          HAL_Delay(25);
      }

      /* ========================================= */
      /* (3) 4 LED CHASE */
      /* ========================================= */

      UART_Print("\r\n--- 4 LED Chase ---\r\n");

      WS_RGB chain[LED_COUNT];

      memset(chain, 0, sizeof(chain));

      for(int round = 0; round < 3; round++)
      {
          /* Only first 4 LEDs used */
          for(uint8_t active = 0; active < 4; active++)
          {
              for(int i = 0; i < LED_COUNT; i++)
              {
                  if(i == active)
                  {
                      chain[i].r = 255;
                      chain[i].g = 0;
                      chain[i].b = 0;
                  }
                  else
                  {
                      chain[i].r = 0;
                      chain[i].g = 0;
                      chain[i].b = 0;
                  }
              }

              /* Keep 5th LED OFF */
              chain[4].r = 0;
              chain[4].g = 0;
              chain[4].b = 0;

              WS2812_SetChain(chain, LED_COUNT);

              UART_Printf("Round %d Active LED: %d\r\n",
                          round + 1,
                          active);

              HAL_Delay(200);
          }
      }

      /* ========================================= */
      /* TURN ALL LEDs OFF */
      /* ========================================= */

      WS2812_SetColor(0, 0, 0);

      UART_Print("\r\nTask Completed. LEDs OFF.\r\n");

      /* Stop program forever */
      while(1)
      {
      }

    /* USER CODE END WHILE */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 225;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
    if(tim_pwmHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
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

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
