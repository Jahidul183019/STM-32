/*
 * CSE 2206: Lab-02
 * Task 5 - WS2812B RGB LED: Colour Mixing and Animation
 *
 */

#ifndef HELPER_H_
#define HELPER_H_

void SystemClock_Config(void);

void USART2_Init(void);
void USART2_SendString(const char *);

void TIM6_Init(void);
void delay_us(uint16_t);
void delay_ms(uint32_t);

#endif /* HELPER_H_ */
