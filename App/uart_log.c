/*
 * 串口日志实现 —— USART1 (PA9=TX, PA10=RX), 115200 8N1
 * 寄存器级初始化; 阻塞发送 + 非阻塞收(轮询)。
 * PA9/PA10 走板载 CH340 → USB, 与以太网(PA1/2/7)不冲突。
 * 同一路串口既发反馈又收命令: 电脑App/HMI/蓝牙用一根线双向通信。
 * uart_log()额外把同一份内容镜像发到USB CDC(usbd_cdc_if.c), 收命令那边
 * (ecat_motion.c的poll_cmd)也并列轮询usb_cdc_getc() —— CH340不可用时用USB口顶上。
 */
#include "stm32f4xx_hal.h"
#include "uart_log.h"
#include "usbd_cdc_if.h"
#include "gui_log.h"
#include <stdarg.h>
#include <stdio.h>

void uart_log_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_USART1_CLK_ENABLE();

   /* PA9 = USART1_TX, PA10 = USART1_RX, 复用推挽 */
   g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
   g.Mode      = GPIO_MODE_AF_PP;
   g.Pull      = GPIO_PULLUP;                    /* RX上拉, 悬空时不误触发 */
   g.Speed     = GPIO_SPEED_FREQ_HIGH;
   g.Alternate = GPIO_AF7_USART1;
   HAL_GPIO_Init(GPIOA, &g);

   /* 波特率: BRR = PCLK2 / 波特率 (F407 USART1 挂 APB2) */
   USART1->BRR = HAL_RCC_GetPCLK2Freq() / 115200U;
   USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;  /* 使能USART + 发送 + 接收 */
}

/* 非阻塞收: RXNE置位则读走DR(读DR会自动清RXNE); 否则返回-1 */
int uart_rx_getc(void)
{
   if (USART1->SR & USART_SR_RXNE)
      return (int)(USART1->DR & 0xFF);
   return -1;
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

   /* 屏幕"实时日志"页用: 纯内存拷贝进环形缓冲, 不阻塞, 不影响上面两路发送 */
   gui_log_push(buf);

   /* USB CDC同步镜像一份: 非阻塞, 忙(上一包没发完)或没插USB线就直接丢,
      USART1(CH340)始终是权威通道, 这里丢了不影响功能, 只是那次没显示。 */
   CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}
