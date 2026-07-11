/*
 * SOEM oshw 实现 —— STM32F407 裸机版 (参照 oshw/rtk/oshw.c)
 * STM32 是小端, 以太网头是大端, 所以 htons/ntohs 就是字节交换。
 */
#include "oshw.h"

uint16 oshw_htons(uint16 host)
{
   return (uint16)((host << 8) | (host >> 8));
}

uint16 oshw_ntohs(uint16 network)
{
   return (uint16)((network << 8) | (network >> 8));
}

/* 裸机固定单网口, 不做适配器枚举 */
ec_adaptert *oshw_find_adapters(void)      { return NULL; }
void oshw_free_adapters(ec_adaptert *a)    { (void)a; }
