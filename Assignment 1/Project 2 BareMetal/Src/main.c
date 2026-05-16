/* Lab 2A - UART2 Polling TX/RX (No HAL) on STM32F446RE
 * UART2 : PA2=TX , PA3=RX | APB1 Bus | Baud 115200 , 8N1
 * APB1 clock assumed = 45 MHz ( configure PLL accordingly )
 */

#include <stdint.h>
#include "stm32f446xx.h"

#define APB1_CLK 45000000UL
#define BAUD_RATE 115200UL

void SystemClock_Config(void)
{
    /* 1. Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* 2. Voltage scaling (Scale 1 mode) */
    PWR->CR |= PWR_CR_VOS;

    /* 3. Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    /* 4. Configure FLASH latency and enable caches */
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    /* 5. Configure PLL
       HSI = 16 MHz
       PLLM = 8
       PLLN = 180
       PLLP = 2
       PLLQ = 2
    */

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (180 << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);   // PLLP = 2
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI);
    RCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLQ_Pos);

    /* 6. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    /* 7. Enable OverDrive mode */
    PWR->CR |= PWR_CR_ODEN;
    while(!(PWR->CSR & PWR_CSR_ODRDY));

    PWR->CR |= PWR_CR_ODSWEN;
    while(!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* 8. Configure Bus Prescalers
       AHB = SYSCLK /1
       APB1 = HCLK /4
       APB2 = HCLK /2
    */

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* 9. Select PLL as system clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
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
}

void USART2_SendChar ( char c ) {
    while (!( USART2->SR & USART_SR_TXE ) ) ; /* wait TX buffer empty */
    USART2->DR = ( uint8_t )c ;
}

void USART2_SendString ( const char * str ) {
    while (* str ) USART2_SendChar (* str ++) ;
}

char USART2_RecvChar ( void ) {
    while (!( USART2->SR & USART_SR_RXNE ) ) ; /* wait data ready */
    return ( char ) USART2->DR ; /* reading DR clears RXNE */
}

int main ( void ) {
	SystemClock_Config();
    USART2_Init () ;
    USART2_SendString (" STM32F446RE UART Polling Demo \r\n") ;
    USART2_SendString (" Type a character -- it will be echoed :\r\n") ;

    while (1) {
        char c = USART2_RecvChar () ; /* blocking receive */
        USART2_SendChar ( c ) ; /* echo back */
        if ( c == '\r') USART2_SendChar ('\n') ;
    }
}

