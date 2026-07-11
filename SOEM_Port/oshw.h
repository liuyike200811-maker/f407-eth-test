/*
 * SOEM oshw 头 —— STM32F407 裸机版 (参照 oshw/rtk/oshw.h)
 */
#ifndef _oshw_
#define _oshw_

#ifdef __cplusplus
extern "C" {
#endif

#include "soem/soem.h"
#include "nicdrv.h"
#include <stddef.h>
#include <stdint.h>

/* 网卡底层三函数(在 stm32_eth.c 用 HAL_ETH 实现) */
int oshw_mac_init(const uint8_t *mac_address);
int oshw_mac_send(const void *payload, size_t tot_len);
int oshw_mac_recv(void *buffer, size_t buffer_length);

uint16 oshw_htons(uint16 host);
uint16 oshw_ntohs(uint16 network);

ec_adaptert *oshw_find_adapters(void);
void oshw_free_adapters(ec_adaptert *adapter);

#ifdef __cplusplus
}
#endif

#endif
