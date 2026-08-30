/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Lab 04 Flash Data Logging - HAL Version
  ******************************************************************************
  * Target MCU: STM32F446RE, HSI clock = 16 MHz
  * USART2: PA2 = TX, PA3 = RX, 115200 baud
  *
  * Sector 6 (0x08040000): Write-once student identity
  * Sector 7 (0x08060000): Erasable and replaceable test results
  *
  * Features:
  * 1. HAL Flash unlock, lock, sector erase, word write and direct read
  * 2. One-time identity provisioning with read-back verification
  * 3. Previous result display at boot
  * 4. UART-triggered Sector 7 erase/write/read demonstration
  * 5. Separate TC2/TC3 self-test using 0xDEADBEEF
  *
  * This is the FLASH-ONLY part of Lab 04. Simulated voltage values test
  * Flash logging only; they do not complete the ADC milestones.
  *
  * Linker/programmer setup: reserve 0x08040000-0x0807FFFF for Sectors 6/7
  * (application FLASH length 256K from 0x08000000), and use sector erase when
  * downloading after provisioning. A mass erase destroys stored identity.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  FLASH_APP_OK = 0,
  FLASH_APP_ERR_ARGUMENT,
  FLASH_APP_ERR_HAL,
  FLASH_APP_ERR_VERIFY,
  FLASH_APP_ERR_ALREADY_PROVISIONED
} FlashAppStatus_t;

typedef struct {
  uint32_t marker;
  char registration[16];
  char roll[12];
  char name[32];
} StudentInfo_t;

typedef union {
  StudentInfo_t value;
  uint32_t words[sizeof(StudentInfo_t) / sizeof(uint32_t)];
} StudentInfoImage_t;

typedef struct {
  uint32_t marker;
  uint32_t mv12;
  uint32_t mv10;
  uint32_t mv8;
  uint32_t mv6;
} ResultsBlock_t;

typedef union {
  ResultsBlock_t value;
  uint32_t words[sizeof(ResultsBlock_t) / sizeof(uint32_t)];
} ResultsImage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_PROVISION_IDENTITY_BUILD  0
#define FLASH_SELF_TEST_BUILD           0

#if FLASH_PROVISION_IDENTITY_BUILD && FLASH_SELF_TEST_BUILD
#error "Provisioning and self-test builds must not be enabled together"
#endif

#define IDENTITY_SECTOR       FLASH_SECTOR_6
#define IDENTITY_BASE         0x08040000U
#define RESULTS_SECTOR        FLASH_SECTOR_7
#define RESULTS_BASE          0x08060000U
#define SECTOR7_END           0x08080000U

#define IDENTITY_MARKER       0xB1010001U
#define RESULTS_MARKER        0xCAFEBABEU
#define ERASED_WORD           0xFFFFFFFFU
#define FLASH_TEST_PATTERN    0xDEADBEEFU

#define FLASH_ALL_CLEAR_FLAGS (FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  | \
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                               FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
_Static_assert(sizeof(StudentInfo_t) == 64U, "StudentInfo_t must be 64 bytes");
_Static_assert(sizeof(ResultsBlock_t) == 20U, "ResultsBlock_t must be 20 bytes");

static const StudentInfoImage_t student_image = {
  .value = {
    .marker = IDENTITY_MARKER,
    .registration = "2023015944",
    .roll = "01",
    .name = "MD.Jahidul Islam Sarker"
  }
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void UART_SendString(const char *text);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_SendString(const char *text)
{
  (void)HAL_UART_Transmit(&huart2,
                         (uint8_t *)text,
                         (uint16_t)strlen(text),
                         HAL_MAX_DELAY);
}

static const char *FlashApp_StatusText(FlashAppStatus_t status)
{
  switch (status) {
    case FLASH_APP_OK:                      return "OK";
    case FLASH_APP_ERR_ARGUMENT:            return "invalid argument/address";
    case FLASH_APP_ERR_HAL:                 return "HAL Flash operation failed";
    case FLASH_APP_ERR_VERIFY:              return "read-back verification failed";
    case FLASH_APP_ERR_ALREADY_PROVISIONED: return "identity already provisioned";
    default:                                return "unknown error";
  }
}

static void Print_FlashStatus(const char *operation, FlashAppStatus_t status)
{
  UART_SendString(operation);
  UART_SendString(": ");
  UART_SendString(FlashApp_StatusText(status));
  UART_SendString("\r\n");

  if (status == FLASH_APP_ERR_HAL) {
    char line[48];
    snprintf(line, sizeof(line), "HAL Flash error code: 0x%08lX\r\n",
             (unsigned long)HAL_FLASH_GetError());
    UART_SendString(line);
  }
}

static uint8_t Flash_IsAllowedWordAddress(uint32_t address)
{
  return ((address >= IDENTITY_BASE) &&
          (address <= (SECTOR7_END - sizeof(uint32_t))) &&
          ((address & 3U) == 0U)) ? 1U : 0U;
}

static uint32_t Flash_ReadWord_HAL(uint32_t address)
{
  return *(volatile const uint32_t *)(uintptr_t)address;
}

static FlashAppStatus_t Flash_EraseSector_HAL(uint32_t sector)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0xFFFFFFFFU;
  HAL_StatusTypeDef hal_status;

  /* 1. Permit only the two sectors assigned by Lab 04. */
  if ((sector != IDENTITY_SECTOR) && (sector != RESULTS_SECTOR)) {
    return FLASH_APP_ERR_ARGUMENT;
  }

  /* 2. Unlock the Flash controller. */
  hal_status = HAL_FLASH_Unlock();
  if (hal_status != HAL_OK) {
    return FLASH_APP_ERR_HAL;
  }

  /* 3. Clear stale completion/error flags. */
  __HAL_FLASH_CLEAR_FLAG(FLASH_ALL_CLEAR_FLAGS);

  /* 4. Configure one-sector erase at the board's 3.3 V supply. */
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = sector;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  /* 5. HAL performs SER/SNB/STRT setup and waits for completion. */
  hal_status = HAL_FLASHEx_Erase(&erase, &sector_error);

  /* 6. Always re-lock, even when erase reports an error. */
  if (HAL_FLASH_Lock() != HAL_OK) {
    hal_status = HAL_ERROR;
  }

  __DSB();
  __ISB();

  if ((hal_status != HAL_OK) || (sector_error != 0xFFFFFFFFU)) {
    return FLASH_APP_ERR_HAL;
  }
  return FLASH_APP_OK;
}

static FlashAppStatus_t Flash_WriteWord_HAL(uint32_t address, uint32_t data)
{
  HAL_StatusTypeDef hal_status;

  /* 1. Validate alignment and restrict writes to Sector 6/7. */
  if (Flash_IsAllowedWordAddress(address) == 0U) {
    return FLASH_APP_ERR_ARGUMENT;
  }

  /* 2. Unlock Flash and clear previous status flags. */
  hal_status = HAL_FLASH_Unlock();
  if (hal_status != HAL_OK) {
    return FLASH_APP_ERR_HAL;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_ALL_CLEAR_FLAGS);
  /* 3. Program exactly one 32-bit word. */
  hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);

  /* 4. Re-lock Flash regardless of programming success. */
  if (HAL_FLASH_Lock() != HAL_OK) {
    hal_status = HAL_ERROR;
  }

  __DSB();
  __ISB();

  if (hal_status != HAL_OK) {
    return FLASH_APP_ERR_HAL;
  }
  /* 5. Verify the programmed value by direct memory-mapped reading. */
  return (Flash_ReadWord_HAL(address) == data)
         ? FLASH_APP_OK : FLASH_APP_ERR_VERIFY;
}

static uint8_t Flash_BlockMatches_HAL(uint32_t base,
                                      const uint32_t *words,
                                      uint32_t word_count)
{
  for (uint32_t i = 0U; i < word_count; i++) {
    if (Flash_ReadWord_HAL(base + (i * 4U)) != words[i]) {
      return 0U;
    }
  }
  return 1U;
}

static FlashAppStatus_t Flash_WriteBlockMarkerLast_HAL(uint32_t base,
                                                        const uint32_t *words,
                                                        uint32_t word_count)
{
  FlashAppStatus_t status;

  if ((words == NULL) || (word_count < 2U)) {
    return FLASH_APP_ERR_ARGUMENT;
  }

  /* Write payload first so an interrupted write has no valid marker. */
  for (uint32_t i = 1U; i < word_count; i++) {
    status = Flash_WriteWord_HAL(base + (i * 4U), words[i]);
    if (status != FLASH_APP_OK) {
      return status;
    }
  }

  /* Marker is the commit record and is programmed last. */
  status = Flash_WriteWord_HAL(base, words[0]);
  if (status != FLASH_APP_OK) {
    return status;
  }

  return (Flash_BlockMatches_HAL(base, words, word_count) != 0U)
         ? FLASH_APP_OK : FLASH_APP_ERR_VERIFY;
}

static FlashAppStatus_t Flash_SelfTest_HAL(void)
{
  FlashAppStatus_t status;

  status = Flash_EraseSector_HAL(RESULTS_SECTOR);
  if (status != FLASH_APP_OK) {
    return status;
  }
  if (Flash_ReadWord_HAL(RESULTS_BASE) != ERASED_WORD) {
    return FLASH_APP_ERR_VERIFY;
  }

  status = Flash_WriteWord_HAL(RESULTS_BASE, FLASH_TEST_PATTERN);
  if (status != FLASH_APP_OK) {
    return status;
  }
  if (Flash_ReadWord_HAL(RESULTS_BASE) != FLASH_TEST_PATTERN) {
    return FLASH_APP_ERR_VERIFY;
  }

  status = Flash_EraseSector_HAL(RESULTS_SECTOR);
  if ((status == FLASH_APP_OK) &&
      (Flash_ReadWord_HAL(RESULTS_BASE) != ERASED_WORD)) {
    status = FLASH_APP_ERR_VERIFY;
  }
  return status;
}

static FlashAppStatus_t Provision_Identity_HAL(void)
{
  FlashAppStatus_t status;

  /* 1. Never erase or overwrite a valid identity. */
  if (Flash_ReadWord_HAL(IDENTITY_BASE) == IDENTITY_MARKER) {
    return FLASH_APP_ERR_ALREADY_PROVISIONED;
  }

  /* 2. Erase Sector 6 for a first/interrupted provisioning attempt. */
  status = Flash_EraseSector_HAL(IDENTITY_SECTOR);
  if (status != FLASH_APP_OK) {
    return status;
  }
  /* 3. Confirm the marker word is erased. */
  if (Flash_ReadWord_HAL(IDENTITY_BASE) != ERASED_WORD) {
    return FLASH_APP_ERR_VERIFY;
  }

  /* 4. Write identity fields first and validity marker last. */
  return Flash_WriteBlockMarkerLast_HAL(
      IDENTITY_BASE,
      student_image.words,
      (uint32_t)(sizeof(student_image.words) / sizeof(student_image.words[0])));
}

static void Load_Identity_HAL(StudentInfoImage_t *image)
{
  for (uint32_t i = 0U;
       i < (sizeof(image->words) / sizeof(image->words[0]));
       i++) {
    image->words[i] = Flash_ReadWord_HAL(IDENTITY_BASE + (i * 4U));
  }
}

static void Display_Identity_HAL(void)
{
  StudentInfoImage_t image;
  char line[96];

  Load_Identity_HAL(&image);
  UART_SendString("---- Student Identity (Sector 6) ----\r\n");

  if (image.value.marker == IDENTITY_MARKER) {
    snprintf(line, sizeof(line), "Registration : %.16s\r\n",
             image.value.registration);
    UART_SendString(line);
    snprintf(line, sizeof(line), "Roll         : %.12s\r\n",
             image.value.roll);
    UART_SendString(line);
    snprintf(line, sizeof(line), "Name         : %.32s\r\n",
             image.value.name);
    UART_SendString(line);
  } else if (image.value.marker == ERASED_WORD) {
    UART_SendString("NOT YET PROVISIONED.\r\n");
  } else {
    UART_SendString("INVALID/CORRUPT IDENTITY MARKER.\r\n");
  }
  UART_SendString("---------------------------------------\r\n");
}

static FlashAppStatus_t Save_Results_HAL(const ResultsImage_t *image)
{
  /* 1. Erase old Sector 7 results. */
  FlashAppStatus_t status = Flash_EraseSector_HAL(RESULTS_SECTOR);

  if (status != FLASH_APP_OK) {
    return status;
  }
  /* 2. Confirm erase before writing replacement data. */
  if (Flash_ReadWord_HAL(RESULTS_BASE) != ERASED_WORD) {
    return FLASH_APP_ERR_VERIFY;
  }

  /* 3. Write result payload first and validity marker last. */
  return Flash_WriteBlockMarkerLast_HAL(
      RESULTS_BASE,
      image->words,
      (uint32_t)(sizeof(image->words) / sizeof(image->words[0])));
}

static void Load_Results_HAL(ResultsImage_t *image)
{
  for (uint32_t i = 0U;
       i < (sizeof(image->words) / sizeof(image->words[0]));
       i++) {
    image->words[i] = Flash_ReadWord_HAL(RESULTS_BASE + (i * 4U));
  }
}

static void Print_mV(const char *label, uint32_t millivolts)
{
  char line[64];
  snprintf(line, sizeof(line), "%s: %lu.%03lu V\r\n",
           label,
           (unsigned long)(millivolts / 1000U),
           (unsigned long)(millivolts % 1000U));
  UART_SendString(line);
}

static void Display_Results_HAL(void)
{
  ResultsImage_t image;

  Load_Results_HAL(&image);
  UART_SendString("---- Previous Results (Sector 7) ------\r\n");

  if (image.value.marker == RESULTS_MARKER) {
    Print_mV("12-bit", image.value.mv12);
    Print_mV("10-bit", image.value.mv10);
    Print_mV(" 8-bit", image.value.mv8);
    Print_mV(" 6-bit", image.value.mv6);
  } else if (image.value.marker == ERASED_WORD) {
    UART_SendString("No previous test data.\r\n");
  } else {
    UART_SendString("Invalid/corrupt results marker.\r\n");
  }
  UART_SendString("---------------------------------------\r\n");
}

static FlashAppStatus_t Run_FlashOnlyResultsDemo_HAL(void)
{
  static uint32_t run_number = 0U;
  ResultsImage_t image = {0};
  uint32_t base_mv;

  run_number++;
  base_mv = 700U + ((run_number * 347U) % 1900U);

  image.value.marker = RESULTS_MARKER;
  image.value.mv12 = base_mv;
  image.value.mv10 = base_mv + 1U;
  image.value.mv8  = base_mv + 6U;
  image.value.mv6  = base_mv + 25U;

  return Save_Results_HAL(&image);
}

static void UART_DebounceFlush(void)
{
  uint8_t discarded;
  uint32_t quiet_start = HAL_GetTick();

  while ((HAL_GetTick() - quiet_start) < 50U) {
    if (HAL_UART_Receive(&huart2, &discarded, 1U, 1U) == HAL_OK) {
      quiet_start = HAL_GetTick();
    }
  }
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  FlashAppStatus_t status;
  uint8_t received_byte;
  /* USER CODE END 1 */

  /* 1. Initialize HAL, system clock, GPIO and USART2. */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
#if FLASH_PROVISION_IDENTITY_BUILD
  /* 2A. Dedicated provisioning build: write identity once, then stop. */
  status = Provision_Identity_HAL();
  Display_Identity_HAL();
  Print_FlashStatus("Provisioning", status);
  UART_SendString("Rebuild with FLASH_PROVISION_IDENTITY_BUILD=0.\r\n");
  while (1) {
    HAL_Delay(1000U);
  }
#else
  /* 2B. Normal boot order: identity first, previous results second. */
  Display_Identity_HAL();
  Display_Results_HAL();
#endif

  UART_SendString("Lab 04 Flash-only HAL build.\r\n");

#if FLASH_SELF_TEST_BUILD
  /* 3A. Dedicated TC2/TC3 build; this intentionally erases Sector 7. */
  UART_SendString("WARNING: Flash self-test erases Sector 7.\r\n");
  status = Flash_SelfTest_HAL();
  Print_FlashStatus("TC2/TC3 Flash self-test", status);
  while (1) {
    HAL_Delay(1000U);
  }
#else
  /* 3B. Normal mode: any received byte starts one complete update. */
  UART_SendString("Send any UART byte to erase/rewrite Sector 7 demo results.\r\n");
  /* USER CODE END 2 */

  while (1) {
    if (HAL_UART_Receive(&huart2, &received_byte, 1U, 10U) == HAL_OK) {
      (void)received_byte;
      /* 4. Flush extra bytes so one keypress/burst causes one run. */
      UART_DebounceFlush();

      /* 5. Erase Sector 7, store new results and verify them. */
      status = Run_FlashOnlyResultsDemo_HAL();
      Print_FlashStatus("Sector 7 update", status);
      /* 6. Display data only after a successful update. */
      if (status == FLASH_APP_OK) {
        Display_Results_HAL();
      }
      UART_SendString("Send any byte to run again.\r\n");
    }
  }
#endif
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef oscillator = {0};
  RCC_ClkInitTypeDef clocks = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscillator.HSIState = RCC_HSI_ON;
  oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscillator.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
    Error_Handler();
  }

  clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV1;
  clocks.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* Explicit setup keeps this main.c independent of Cube-generated MSP GPIO. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &gpio);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
