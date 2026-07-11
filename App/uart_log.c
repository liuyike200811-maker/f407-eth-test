/*
 * 串口日志实现 —— USART1 (PA9=TX), 115200 8N1
 * 寄存器级初始化, 只发不收; 阻塞发送(调试用足够)。
 * PA9/PA10 走板载 CH340 → USB, 与以太网(PA1/2/7)不冲突。
 */
#include "stm32f4xx_hal.h"
#include "uart_log.h"
#include <stdarg.h>
#include <stdio.h>

void uart_log_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_USART1_CLK_ENABLE();

   /* PA9 = USART1_TX, 复用推挽 */
   g.Pin       = GPIO_PIN_9;
   g.Mode      = GPIO_MODE_AF_PP;
   g.Pull      = GPIO_NOPULL;
   g.Speed     = GPIO_SPEED_FREQ_HIGH;
   g.Alternate = GPIO_AF7_USART1;
   HAL_GPIO_Init(GPIOA, &g);

   /* 波特率: BRR = PCLK2 / 波特率 (F407 USART1 挂 APB2) */
   USART1->BRR = HAL_RCC_GetPCLK2Freq() / 115200U;
   USART1->CR1 = USART_CR1_UE | USART_CR1_TE;   /* 使能USART + 发送 */
}

static void uart_putc(char c)
{
   while (!(USART1->SR & USART_SR_TXE)) { }   /* 等发送寄存器空 */
   USART1->DR = (uint8_t)c;
}

void uart_log(const char *fmt, ...)
{
   char buf[160];
   va_list ap;
   int n, i;

   va_start(ap, fmt);
   n = vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);
   if (n < 0) return;
   if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
   for (i = 0; i < n; i++) uart_putc(buf[i]);
}
