/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CSE 2206 Lab-02 Task 3 — Code Profiling (HAL)
  *                   Single-shot DWT + TIM2 dual measurement
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSCLK_HZ       180000000UL   /* 180 MHz                          */
#define SORT_N          100U           /* bubble-sort array size           */
#define SQRT_ITERS      1000U          /* integer sqrt iterations          */
#define MEMCPY_BYTES    512U           /* memory-copy block size           */

/* Exactly 48 bytes including \r\n: 46 chars + \r + \n */
#define SEND_STR        "STM32F446RE USART2 @ 115200 baud OK!\r\n"
#define SEND_STR_LEN    48U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===========================================================================
 * UART helper
 * =========================================================================== */
static void USART2_SendString(const char *s)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* ===========================================================================
 * Task-2 delay functions (TIM6, 1 us tick, needed for block [2])
 * =========================================================================== */
static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) < us) {}
}

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) delay_us(1000U);
}

/* ===========================================================================
 * Algorithm A3.1 — DWT Cycle Counter Enable
 * =========================================================================== */
static void DWT_Init(void)
{
    /* Step 1: Enable trace / DWT via CoreDebug DEMCR */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Step 2: Reset cycle counter */
    DWT->CYCCNT = 0U;

    /* Step 3: Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ===========================================================================
 * Profiling result structure
 * =========================================================================== */
typedef struct {
    uint32_t dwt_cycles;
    uint32_t tim2_us;
} Profile_t;

/* ===========================================================================
 * SINGLE-SHOT profiling — both DWT and TIM2 wrap the SAME execution.
 * This is required so the two methods measure identical work, allowing the
 * post-lab "justify any differences" question to be answered correctly
 * (differences then reduce to TIM2's 1 us quantisation vs DWT's 5.56 ns).
 * =========================================================================== */
typedef void (*ProfileFn_t)(void *ctx);

static Profile_t profile_block(ProfileFn_t fn, void *ctx)
{
    Profile_t r;

    /* Reset both stopwatches as close together as possible */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    uint32_t dwt_t0 = DWT->CYCCNT;

    /* --- ONE execution, measured by both methods simultaneously --- */
    fn(ctx);

    uint32_t dwt_t1 = DWT->CYCCNT;
    r.tim2_us       = __HAL_TIM_GET_COUNTER(&htim2);
    r.dwt_cycles    = dwt_t1 - dwt_t0;   /* unsigned subtraction — overflow-safe */

    return r;
}

/* Helper: convert DWT cycles to time units and print one table row.
 * Format matches the assignment deliverable: # | Block | Cycles | ns | us | ms
 * Both DWT and TIM2 rows are printed for each block.
 */
static void print_row(const char *label,
                      uint32_t dwt_cycles,
                      uint32_t tim2_us)
{
    /* DWT time conversions */
    uint32_t dwt_ns  = (uint32_t)((uint64_t)dwt_cycles * 1000000000ULL / SYSCLK_HZ);
    uint32_t dwt_us  = dwt_ns  / 1000U;
    uint32_t dwt_ms  = dwt_us  / 1000U;

    /* TIM2 time conversions (already in us) */
    uint32_t tim_ns  = tim2_us * 1000U;
    uint32_t tim_ms  = tim2_us / 1000U;

    char buf[200];

    /* DWT row */
    snprintf(buf, sizeof(buf),
             "| %-22s | DWT  | %10lu | %10lu | %8lu | %6lu |\r\n",
             label,
             (unsigned long)dwt_cycles,
             (unsigned long)dwt_ns,
             (unsigned long)dwt_us,
             (unsigned long)dwt_ms);
    USART2_SendString(buf);

    /* TIM2 row */
    snprintf(buf, sizeof(buf),
             "| %-22s | TIM2 |         -- | %10lu | %8lu | %6lu |\r\n",
             "",
             (unsigned long)tim_ns,
             (unsigned long)tim2_us,
             (unsigned long)tim_ms);
    USART2_SendString(buf);

    /* separator */
    USART2_SendString("|------------------------|------|------------|------------|----------|--------|\r\n");
}

/* ===========================================================================
 * Block [1] — Bubble sort, N=100, worst-case (reverse-filled)
 * =========================================================================== */
static int sort_arr[SORT_N];

static void block_bubble_sort(void *ctx)
{
    (void)ctx;
    int n = (int)SORT_N;
    for (int i = 0; i < n - 1; i++)
    {
        for (int j = 0; j < n - 1 - i; j++)
        {
            if (sort_arr[j] > sort_arr[j + 1])
            {
                int tmp        = sort_arr[j];
                sort_arr[j]    = sort_arr[j + 1];
                sort_arr[j+1]  = tmp;
            }
        }
    }
}

static void fill_reverse(void)
{
    for (int i = 0; i < (int)SORT_N; i++)
        sort_arr[i] = (int)(SORT_N - 1 - i);   /* 99,98,...,1,0 */
}

/* ===========================================================================
 * Block [2] — delay_ms(100)
 * =========================================================================== */
static void block_delay_ms100(void *ctx)
{
    (void)ctx;
    delay_ms(100U);
}

/* ===========================================================================
 * Block [3] — USART2_SendString, fixed 48-byte string (incl. \r\n)
 * =========================================================================== */
static void block_send_string(void *ctx)
{
    (void)ctx;
    USART2_SendString(SEND_STR);   /* SEND_STR_LEN = 48 bytes */
}

/* ===========================================================================
 * Block [4] — Integer square root (Newton–Raphson), 1000 inputs
 * =========================================================================== */
static uint32_t isqrt_nr(uint32_t n)
{
    if (n == 0U) return 0U;
    uint32_t x = n;
    uint32_t y = (x + 1U) / 2U;
    while (y < x)
    {
        x = y;
        y = (x + n / x) / 2U;
    }
    return x;
}

static volatile uint32_t sink;   /* prevent optimisation of sqrt results */

static void block_sqrt_1000(void *ctx)
{
    (void)ctx;
    for (uint32_t i = 0; i < SQRT_ITERS; i++)
        sink = isqrt_nr(i * 37U + 1U);   /* varied inputs */
}

/* ===========================================================================
 * Block [5] — Byte-by-byte memory copy, 512 bytes
 * =========================================================================== */
static uint8_t src_buf[MEMCPY_BYTES];
static uint8_t dst_buf[MEMCPY_BYTES];

static void block_memcpy512(void *ctx)
{
    (void)ctx;
    for (uint32_t i = 0; i < MEMCPY_BYTES; i++)
        dst_buf[i] = src_buf[i];
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Start timers and DWT */
  HAL_TIM_Base_Start(&htim6);   /* TIM6 — 1 us tick for delay_ms          */
  HAL_TIM_Base_Start(&htim2);   /* TIM2 — 1 us free-running stopwatch      */
  DWT_Init();                    /* DWT  — cycle counter, 5.56 ns per tick  */

  /* Initialise source buffer for memcpy test */
  for (uint32_t i = 0; i < MEMCPY_BYTES; i++)
      src_buf[i] = (uint8_t)(i & 0xFFU);

  /* =========================================================================
   * Print table header
   * ========================================================================= */
  USART2_SendString("\r\n===== Lab-02 Task 3: Code Profiling (HAL) =====\r\n\r\n");
  USART2_SendString("|------------------------|------|------------|------------|----------|--------|\r\n");
  USART2_SendString("| Block                  | Meth |     Cycles |         ns |       us |     ms |\r\n");
  USART2_SendString("|------------------------|------|------------|------------|----------|--------|\r\n");

  Profile_t r;

  /* =========================================================================
   * [1] Bubble sort N=100 (worst-case, reverse-filled)
   *     Refill BEFORE the single-shot measurement so the array is in true
   *     worst-case state when the (one) sort runs.
   * ========================================================================= */
  fill_reverse();
  r = profile_block(block_bubble_sort, NULL);
  print_row("[1] Bubble sort N=100", r.dwt_cycles, r.tim2_us);

  /* =========================================================================
   * [2] delay_ms(100) — compare with expected 100 000 us
   * ========================================================================= */
  r = profile_block(block_delay_ms100, NULL);
  print_row("[2] delay_ms(100)", r.dwt_cycles, r.tim2_us);

  /* =========================================================================
   * [3] SendString 48 B
   *     Effective baud = (bytes * 10 bits) / time_seconds
   *     (8 data + 1 start + 1 stop = 10 bits/byte, no parity)
   * ========================================================================= */
  r = profile_block(block_send_string, NULL);
  print_row("[3] SendString 48B", r.dwt_cycles, r.tim2_us);

  /* Print calculated effective baud rate from TIM2 measurement */
  {
      char buf[120];
      /* baud = (SEND_STR_LEN * 10 bits) / (tim2_us * 1e-6 s)
       *      = (SEND_STR_LEN * 10 * 1e6) / tim2_us
       *      = 480000000 / tim2_us  for SEND_STR_LEN = 48
       */
      uint32_t eff_baud = (r.tim2_us > 0U)
                          ? (uint32_t)((uint64_t)SEND_STR_LEN * 10ULL * 1000000ULL / r.tim2_us)
                          : 0U;
      snprintf(buf, sizeof(buf),
               "  >> Effective baud (TIM2): %lu baud  (configured: 115200)\r\n\r\n",
               (unsigned long)eff_baud);
      USART2_SendString(buf);
  }

  /* =========================================================================
   * [4] Integer sqrt, 1000 inputs (Newton-Raphson)
   * ========================================================================= */
  r = profile_block(block_sqrt_1000, NULL);
  print_row("[4] isqrt x1000 (N-R)", r.dwt_cycles, r.tim2_us);

  /* =========================================================================
   * [5] Byte-by-byte memcpy, 512 bytes
   * ========================================================================= */
  r = profile_block(block_memcpy512, NULL);
  print_row("[5] memcpy 512B (byte)", r.dwt_cycles, r.tim2_us);

  USART2_SendString("\r\n===== Task 3 Complete =====\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;            /* 90MHz / (89+1) = 1 MHz -> 1 us tick */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;       /* 0xFFFFFFFF — full 32-bit range     */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim6.Init.Prescaler = 89;            /* 1 us tick */
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
