/* Lab 4A - UART2 Receive Interrupt + Ring Buffer (No HAL)
 * Receives bytes in ISR into a ring buffer.
 * Main loop detects complete lines and echoes them.
 */
#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#define BUF_SIZE 128

volatile char ring[BUF_SIZE];
volatile uint8_t head = 0, tail = 0;
volatile uint8_t lineReady = 0;


void SystemClock_Config(void)
{
    /* 1. Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* 2. Voltage scaling (Scale 1 mode) */
    PWR->CR |= PWR_CR_VOS;

    /* 3. Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* 4. FLASH latency + caches */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    /* 5. Configure PLL
       HSI = 16 MHz
       PLLM = 8  -> 2 MHz
       PLLN = 180 -> 360 MHz
       PLLP = 2   -> 180 MHz SYSCLK
       PLLQ = 2
    */

    RCC->PLLCFGR = 0;

    RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (180 << RCC_PLLCFGR_PLLN_Pos);

    /* PLLP = 2 (00 in bits) */
    RCC->PLLCFGR &= ~(3 << RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLQ_Pos);

    /* 6. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 7. Enable OverDrive (required for 180 MHz) */
    PWR->CR |= PWR_CR_ODEN;
    while (!(PWR->CSR & PWR_CSR_ODRDY));

    PWR->CR |= PWR_CR_ODSWEN;
    while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* 8. Bus prescalers */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* 9. Switch to PLL */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void USART2_Init ( void ) {
    /* 1. Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __NOP(); __NOP();

    /* 2. PA2 , PA3 -> Alternate Function 7 ( USART2 ) */
    GPIOA->MODER &= ~((3UL << 4) | (3UL << 6));
    GPIOA->MODER |= ((2UL << 4) | (2UL << 6)); /* AF mode */

    GPIOA->AFR[0] &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOA->AFR[0] |= ((7UL << 8) | (7UL << 12)); /* AF7 */

    GPIOA->OTYPER &= ~((1UL << 2) | (1UL << 3)); /* Push - Pull */

    GPIOA->OSPEEDR |= ((3UL << 4) | (3UL << 6)); /* Very High */

    /* 3. Baud rate : CLK = 45MHz , Baud = 115200 ,  USARTDIV = 24.414 (CLK/(16*Baud)) , Mantissa = 24
     Fraction ≈ 7
     */
    USART2->BRR = (24 << 4) | 7;   // 0x187

    /* 4. Enable USART , transmitter , receiver */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;


    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

/* USART2 ISR */
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char c = (char)USART2->DR;  /* Reading DR clears RXNE */
        ring[head] = c;
        head = (head + 1) % BUF_SIZE;
        if (c == '\r') lineReady = 1;  /* Line complete */
    }

    if (USART2->SR & USART_SR_ORE) {
        (void)USART2->DR;  /* Reading DR clears ORE */
    }
}

/* Send a string via USART2 */
void USART2_SendString(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));  /* Wait until TX buffer empty */
        USART2->DR = (uint8_t)(*s++);
    }
}

int main(void) {
	SystemClock_Config();
	USART2_Init();

    /* Welcome message */
    USART2_SendString("UART Interrupt Demo - type and press Enter:\r\n");

    /* Main loop */
    while (1) {
        if (lineReady) {
            lineReady = 0;

            /* Drain ring buffer into a local line buffer */
            char line[BUF_SIZE];
            uint8_t i = 0;
            while (tail != head) {
                line[i++] = ring[tail];
                tail = (tail + 1) % BUF_SIZE;
            }
            line[i] = '\0';

            /* Prepare response string */
            char resp[BUF_SIZE + 16];
            snprintf(resp, sizeof(resp), "Echo: %s\r\n", line);
            USART2_SendString(resp);
        }
    }
}
