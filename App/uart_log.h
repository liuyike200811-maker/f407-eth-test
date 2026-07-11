/*
 * 串口日志 —— USART1 (PA9=TX), 115200 8N1, 走板载 CH340 USB
 * 用法: uart_log_init() 一次, 之后 uart_log("...%d...", x) 像 printf 一样打
 */
#ifndef _uart_log_h_
#define _uart_log_h_

#ifdef __cplusplus
extern "C" {
#endif

void uart_log_init(void);
void uart_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
