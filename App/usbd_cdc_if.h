/*
 * USB CDC 应用层接口: 供 ecat_motion.c / uart_log.c 调用的收发函数
 */
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#include "usbd_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* 非阻塞发送: EP忙则直接丢弃返回, 不等待(避免拖慢4ms周期) */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* 非阻塞收一个字节: 有数据返回0~255, 无数据返回-1 */
int usb_cdc_getc(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */
