/*
 * Modbus RTU 从站 —— USART3 (PB10=TX, PB11=RX), 115200 8N1, 站号1
 * ================================================================
 * 通用协议引擎: 只负责 Modbus 帧收发/解析 + 维护两个寄存器数组,
 * 不含任何业务语义。业务翻译(线圈→命令、反馈→寄存器)在 ecat_motion.c 做。
 *
 * 与 HMI 的契约见 HMI/STM32_Modbus寄存器契约.txt (v2):
 *   线圈 0x0000~0x0008 : 控制区(HMI写)   -> modbus_coils[0..8]
 *   保持寄存器 4x0000~ : 设定区/反馈区    -> modbus_hreg[0..24]
 * 地址基数=0, 即 Modbus 地址号 = 数组下标。
 *
 * 帧尾用 USART3 的 IDLE 行空闲中断判定(RTU 帧间隔), 全程中断收帧;
 * 解析+应答在 modbus_poll() 里做(主上下文, 由 poll_cmd() 每周期调一次)。
 * 支持功能码: 01/02 读线圈  03 读保持寄存器  05/06 写单个  0F/10 写多个。
 */
#ifndef _modbus_slave_h_
#define _modbus_slave_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_SLAVE_ID    1
#define MODBUS_NUM_COILS   16    /* 用到 0..8, 留余量 */
#define MODBUS_NUM_HREGS   25    /* 用到 0..24 (契约 v2: 含 4x0018) */

/* 寄存器区(地址基数0, 下标=Modbus地址号)。仅主上下文读写;
   ISR 只碰内部 RX 缓冲, 不碰这两个数组, 故无需临界区。 */
extern volatile uint8_t  modbus_coils[MODBUS_NUM_COILS];
extern volatile uint16_t modbus_hreg[MODBUS_NUM_HREGS];

void modbus_init(void);         /* 初始化 USART3 + NVIC, 调一次 */
void modbus_poll(void);         /* 有整帧则解析并应答; 每通信周期调一次 */
void modbus_usart3_isr(void);   /* 供 USART3_IRQHandler() 转调 */

#ifdef __cplusplus
}
#endif

#endif
