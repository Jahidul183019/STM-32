/* Lab 04 - Flash Data Logging - Bare Metal (No HAL)
 * Target MCU: STM32F446RE, HSI clock = 16 MHz
 * USART2: PA2 = TX, PA3 = RX, 115200 baud
 *
 * Sector 6 (0x08040000): Write-once student identity
 * Sector 7 (0x08060000): Erasable and replaceable test results
 *
 * Features:
 * 1. Flash unlock, lock, sector erase, word write and direct read
 * 2. One-time identity provisioning with read-back verification
 * 3. Previous result display at boot
 * 4. UART-triggered Sector 7 erase/write/read demonstration
 * 5. Separate TC2/TC3 self-test using 0xDEADBEEF
 *
 * This is the FLASH-ONLY part of Lab 04. It uses simulated voltage values
 * to test Flash logging; therefore it does not complete the ADC milestones.
 *
 * Linker: reserve Sectors 6-7 by limiting application FLASH to 256K.
 * Programmer: use sector erase after provisioning; mass erase deletes data.
 */

#include <stdint.h>
#include <stdio.h>
#include <stm32f446xx.h>

/* Set exactly one of these to 1 for its dedicated build. */
#define FLASH_PROVISION_IDENTITY_BUILD  0
#define FLASH_SELF_TEST_BUILD           0

#if FLASH_PROVISION_IDENTITY_BUILD && FLASH_SELF_TEST_BUILD
#error "Provisioning and self-test builds must not be enabled together"
#endif

#define IDENTITY_SECTOR       6U
#define IDENTITY_BASE         0x08040000U
#define RESULTS_SECTOR        7U
#define RESULTS_BASE          0x08060000U
#define SECTOR7_END           0x08080000U

#define IDENTITY_MARKER       0xB1010001U
#define RESULTS_MARKER        0xCAFEBABEU
#define ERASED_WORD           0xFFFFFFFFU
#define FLASH_TEST_PATTERN    0xDEADBEEFU

#define FLASH_KEY1_VALUE      0x45670123U
#define FLASH_KEY2_VALUE      0xCDEF89ABU
#define FLASH_TIMEOUT_LOOPS   160000000U

#define FLASH_ERROR_BITS (FLASH_SR_OPERR  | FLASH_SR_WRPERR | \
                          FLASH_SR_PGAERR | FLASH_SR_PGPERR | \
                          FLASH_SR_PGSERR | FLASH_SR_RDERR)
#define FLASH_CLEAR_BITS (FLASH_SR_EOP | FLASH_ERROR_BITS)

typedef enum {
    FLASH_OK = 0,
    FLASH_ERR_ARGUMENT,
    FLASH_ERR_TIMEOUT,
    FLASH_ERR_UNLOCK,
    FLASH_ERR_STATUS,
    FLASH_ERR_VERIFY,
    FLASH_ERR_ALREADY_PROVISIONED
} FlashStatus_t;

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

static void delay_us(uint32_t us)
{
    uint32_t count = us * 4U;
    while (count-- != 0U) {
        __NOP();
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_us(1000U);
    }
}

static void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));

    GPIOA->AFR[0] &= ~((0xFU << (4U * 2U)) | (0xFU << (4U * 3U)));
    GPIOA->AFR[0] |=  ((7U << (4U * 2U)) | (7U << (4U * 3U)));

    GPIOA->OSPEEDR |= (3U << (2U * 2U)) | (3U << (3U * 2U));
    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->PUPDR |= (1U << (3U * 2U));

    /* 16 MHz PCLK1, oversampling by 16, 115200 baud -> BRR = 0x008B. */
    USART2->BRR = 0x008BU;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void UART_SendString(const char *text)
{
    while (*text != '\0') {
        while ((USART2->SR & USART_SR_TXE) == 0U) {
        }
        USART2->DR = (uint8_t)*text++;
    }
    while ((USART2->SR & USART_SR_TC) == 0U) {
    }
}

static uint8_t UART_TryReceive(uint8_t *byte)
{
    if ((USART2->SR & USART_SR_RXNE) != 0U) {
        *byte = (uint8_t)USART2->DR;
        return 1U;
    }
    return 0U;
}

static void UART_DebounceFlush(void)
{
    uint8_t discarded;
    uint32_t quiet_ms = 0U;

    while (quiet_ms < 50U) {
        if (UART_TryReceive(&discarded) != 0U) {
            quiet_ms = 0U;
        } else {
            delay_ms(1U);
            quiet_ms++;
        }
    }
}

static const char *Flash_StatusText(FlashStatus_t status)
{
    switch (status) {
        case FLASH_OK:                      return "OK";
        case FLASH_ERR_ARGUMENT:            return "invalid argument/address";
        case FLASH_ERR_TIMEOUT:             return "operation timeout";
        case FLASH_ERR_UNLOCK:              return "unlock failed";
        case FLASH_ERR_STATUS:              return "controller status error";
        case FLASH_ERR_VERIFY:              return "read-back verification failed";
        case FLASH_ERR_ALREADY_PROVISIONED: return "identity already provisioned";
        default:                            return "unknown error";
    }
}

static void Print_FlashStatus(const char *operation, FlashStatus_t status)
{
    UART_SendString(operation);
    UART_SendString(": ");
    UART_SendString(Flash_StatusText(status));
    UART_SendString("\r\n");
}

static FlashStatus_t Flash_WaitReady(void)
{
    uint32_t timeout = FLASH_TIMEOUT_LOOPS;

    while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
        if (timeout-- == 0U) {
            return FLASH_ERR_TIMEOUT;
        }
    }
    return FLASH_OK;
}

static FlashStatus_t Flash_Unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR = FLASH_KEY1_VALUE;
        FLASH->KEYR = FLASH_KEY2_VALUE;
    }
    return ((FLASH->CR & FLASH_CR_LOCK) == 0U) ? FLASH_OK : FLASH_ERR_UNLOCK;
}

static void Flash_Lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void Flash_ClearStatus(void)
{
    /* EOP and error flags are cleared by writing 1 to their SR bits. */
    FLASH->SR = FLASH_CLEAR_BITS;
}

static FlashStatus_t Flash_CheckControllerStatus(void)
{
    return ((FLASH->SR & FLASH_ERROR_BITS) == 0U) ? FLASH_OK : FLASH_ERR_STATUS;
}

static uint8_t Flash_IsAllowedWordAddress(uint32_t address)
{
    return ((address >= IDENTITY_BASE) &&
            (address <= (SECTOR7_END - sizeof(uint32_t))) &&
            ((address & 3U) == 0U)) ? 1U : 0U;
}

static FlashStatus_t Flash_EraseSector(uint8_t sector_number)
{
    FlashStatus_t status;

    /* 1. Permit only the two sectors assigned by Lab 04. */
    if ((sector_number != IDENTITY_SECTOR) &&
        (sector_number != RESULTS_SECTOR)) {
        return FLASH_ERR_ARGUMENT;
    }

    /* 2. Wait for any previous operation, then unlock Flash. */
    status = Flash_WaitReady();
    if (status != FLASH_OK) {
        return status;
    }

    status = Flash_Unlock();
    if (status != FLASH_OK) {
        return status;
    }

    /* 3. Clear stale completion/error flags before starting. */
    Flash_ClearStatus();

    /* 4. Select 32-bit parallelism and the requested sector. */
    FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB |
                   FLASH_CR_PG | FLASH_CR_MER);
    FLASH->CR |= FLASH_CR_PSIZE_1; /* x32 parallelism at 2.7-3.6 V. */
    FLASH->CR |= FLASH_CR_SER |
                 ((uint32_t)sector_number << FLASH_CR_SNB_Pos);
    /* 5. Start sector erase. */
    FLASH->CR |= FLASH_CR_STRT;

    /* 6. Wait until erase ends, then disable sector-erase mode. */
    status = Flash_WaitReady();
    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);

    if (status == FLASH_OK) {
        status = Flash_CheckControllerStatus();
    }

    /* 7. Re-lock Flash before returning to application code. */
    Flash_Lock();
    __DSB();
    __ISB();
    return status;
}

static FlashStatus_t Flash_WriteWord(uint32_t address, uint32_t data)
{
    FlashStatus_t status;

    /* 1. Check that the address is aligned and inside Sector 6/7. */
    if (Flash_IsAllowedWordAddress(address) == 0U) {
        return FLASH_ERR_ARGUMENT;
    }

    /* 2. Wait until Flash is ready and unlock the controller. */
    status = Flash_WaitReady();
    if (status != FLASH_OK) {
        return status;
    }

    status = Flash_Unlock();
    if (status != FLASH_OK) {
        return status;
    }

    /* 3. Clear old flags and select 32-bit word programming. */
    Flash_ClearStatus();
    FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SER |
                   FLASH_CR_SNB | FLASH_CR_MER);
    FLASH->CR |= FLASH_CR_PSIZE_1; /* 32-bit word programming. */
    FLASH->CR |= FLASH_CR_PG;

    /* 4. Programming begins when data is written to the Flash address. */
    *(volatile uint32_t *)(uintptr_t)address = data;

    /* 5. Wait, disable programming mode and inspect error flags. */
    status = Flash_WaitReady();
    FLASH->CR &= ~FLASH_CR_PG;

    if (status == FLASH_OK) {
        status = Flash_CheckControllerStatus();
    }

    /* 6. Re-lock the controller after every word operation. */
    Flash_Lock();
    __DSB();
    __ISB();

    /* 7. Read back the word so a silent programming failure is detected. */
    if ((status == FLASH_OK) &&
        (*(volatile uint32_t *)(uintptr_t)address != data)) {
        status = FLASH_ERR_VERIFY;
    }
    return status;
}

static uint32_t Flash_ReadWord(uint32_t address)
{
    return *(volatile const uint32_t *)(uintptr_t)address;
}

static uint8_t Flash_BlockMatches(uint32_t base,
                                  const uint32_t *words,
                                  uint32_t word_count)
{
    for (uint32_t i = 0U; i < word_count; i++) {
        if (Flash_ReadWord(base + (i * 4U)) != words[i]) {
            return 0U;
        }
    }
    return 1U;
}

static FlashStatus_t Flash_WriteBlockMarkerLast(uint32_t base,
                                                 const uint32_t *words,
                                                 uint32_t word_count)
{
    FlashStatus_t status;

    if ((words == NULL) || (word_count < 2U)) {
        return FLASH_ERR_ARGUMENT;
    }

    /* Payload first: a power loss leaves the marker erased/invalid. */
    for (uint32_t i = 1U; i < word_count; i++) {
        status = Flash_WriteWord(base + (i * 4U), words[i]);
        if (status != FLASH_OK) {
            return status;
        }
    }

    /* Commit marker last, only after every payload word is present. */
    status = Flash_WriteWord(base, words[0]);
    if (status != FLASH_OK) {
        return status;
    }

    return (Flash_BlockMatches(base, words, word_count) != 0U)
           ? FLASH_OK : FLASH_ERR_VERIFY;
}

static FlashStatus_t Flash_SelfTest(void)
{
    FlashStatus_t status;

    status = Flash_EraseSector(RESULTS_SECTOR);
    if (status != FLASH_OK) {
        return status;
    }
    if (Flash_ReadWord(RESULTS_BASE) != ERASED_WORD) {
        return FLASH_ERR_VERIFY;
    }

    status = Flash_WriteWord(RESULTS_BASE, FLASH_TEST_PATTERN);
    if (status != FLASH_OK) {
        return status;
    }
    if (Flash_ReadWord(RESULTS_BASE) != FLASH_TEST_PATTERN) {
        return FLASH_ERR_VERIFY;
    }

    status = Flash_EraseSector(RESULTS_SECTOR);
    if ((status == FLASH_OK) &&
        (Flash_ReadWord(RESULTS_BASE) != ERASED_WORD)) {
        status = FLASH_ERR_VERIFY;
    }
    return status;
}

static FlashStatus_t Provision_Identity(void)
{
    FlashStatus_t status;

    /* 1. A valid identity is never erased or overwritten. */
    if (Flash_ReadWord(IDENTITY_BASE) == IDENTITY_MARKER) {
        return FLASH_ERR_ALREADY_PROVISIONED;
    }

    /* 2. Erase Sector 6 for a first or interrupted provisioning attempt. */
    status = Flash_EraseSector(IDENTITY_SECTOR);
    if (status != FLASH_OK) {
        return status;
    }

    /* 3. Confirm that erase returned the marker word to 0xFFFFFFFF. */
    if (Flash_ReadWord(IDENTITY_BASE) != ERASED_WORD) {
        return FLASH_ERR_VERIFY;
    }

    /* 4. Write details first and validity marker last, then verify. */
    return Flash_WriteBlockMarkerLast(
        IDENTITY_BASE,
        student_image.words,
        (uint32_t)(sizeof(student_image.words) / sizeof(student_image.words[0])));
}

static void Load_Identity(StudentInfoImage_t *image)
{
    for (uint32_t i = 0U;
         i < (sizeof(image->words) / sizeof(image->words[0]));
         i++) {
        image->words[i] = Flash_ReadWord(IDENTITY_BASE + (i * 4U));
    }
}

static void Display_Identity(void)
{
    StudentInfoImage_t image;
    char line[96];

    Load_Identity(&image);
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

static FlashStatus_t Save_Results(const ResultsImage_t *image)
{
    /* 1. Previous results must be erased before replacement. */
    FlashStatus_t status = Flash_EraseSector(RESULTS_SECTOR);

    if (status != FLASH_OK) {
        return status;
    }
    /* 2. Confirm that Sector 7 is blank. */
    if (Flash_ReadWord(RESULTS_BASE) != ERASED_WORD) {
        return FLASH_ERR_VERIFY;
    }

    /* 3. Write four results first and commit marker last. */
    return Flash_WriteBlockMarkerLast(
        RESULTS_BASE,
        image->words,
        (uint32_t)(sizeof(image->words) / sizeof(image->words[0])));
}

static void Load_Results(ResultsImage_t *image)
{
    for (uint32_t i = 0U;
         i < (sizeof(image->words) / sizeof(image->words[0]));
         i++) {
        image->words[i] = Flash_ReadWord(RESULTS_BASE + (i * 4U));
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

static void Display_Results(void)
{
    ResultsImage_t image;

    Load_Results(&image);
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

static FlashStatus_t Run_FlashOnlyResultsDemo(void)
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

    return Save_Results(&image);
}

int main(void)
{
    FlashStatus_t status;
    uint8_t received_byte;

    /* 1. Initialize USART2 for messages and the required UART trigger. */
    USART2_Init();

#if FLASH_PROVISION_IDENTITY_BUILD
    /* 2A. Dedicated provisioning build: write identity once, then stop. */
    status = Provision_Identity();
    Display_Identity();
    Print_FlashStatus("Provisioning", status);
    UART_SendString("Rebuild with FLASH_PROVISION_IDENTITY_BUILD=0.\r\n");
    while (1) {
        __WFI();
    }
#else
    /* 2B. Normal boot order: identity first, previous results second. */
    Display_Identity();
    Display_Results();
#endif

    UART_SendString("Lab 04 Flash-only bare-metal build.\r\n");

#if FLASH_SELF_TEST_BUILD
    /* 3A. Dedicated TC2/TC3 build; this intentionally erases Sector 7. */
    UART_SendString("WARNING: Flash self-test erases Sector 7.\r\n");
    status = Flash_SelfTest();
    Print_FlashStatus("TC2/TC3 Flash self-test", status);
    while (1) {
        __WFI();
    }
#else
    /* 3B. Normal mode: any received byte starts one complete update. */
    UART_SendString("Send any UART byte to erase/rewrite Sector 7 demo results.\r\n");

    while (1) {
        if (UART_TryReceive(&received_byte) != 0U) {
            (void)received_byte;

            /* 4. Flush extra bytes so one keypress/burst causes one run. */
            UART_DebounceFlush();

            /* 5. Erase Sector 7, store a new block and verify it. */
            status = Run_FlashOnlyResultsDemo();
            Print_FlashStatus("Sector 7 update", status);
            /* 6. Display data only after a successful update. */
            if (status == FLASH_OK) {
                Display_Results();
            }
            UART_SendString("Send any byte to run again.\r\n");
        }
    }
#endif
}
