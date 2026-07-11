/*
 * STM32F407 以太网裸帧收发层 —— 给 SOEM 的 oshw_mac_* 用
 */
#ifndef _stm32_eth_h_
#define _stm32_eth_h_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int oshw_mac_init(const uint8_t *mac_address);
int oshw_mac_send(const void *payload, size_t tot_len);
int oshw_mac_recv(void *buffer, size_t buffer_length);

#ifdef __cplusplus
}
#endif

#endif
