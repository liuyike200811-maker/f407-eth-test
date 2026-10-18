/*
 * Modbus RTU 从站 —— USART3 (PB10=TX, PB11=RX), 115200 8N1, 站号1
 * ================================================================
 * 通用协议引擎: 只负责 Modbus 帧收发/解析 + 维护两个寄存器数组,
 * 不含任何业务语义。业务翻译(线圈→命令、反馈→寄存器)在 ecat_motion.c 做。
 *
 * 契约见 精简总结/01-Modbus寄存器契约(权威v4).md，2026-10-18补充：
 *   线圈0..10：控制区(HMI写)，10选择外部模式7。
 *   保持寄存器0..25：16总体状态、24模式、25简化进度(0..5)。
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
#define MODBUS_NUM_COILS   16    /* 用到0..10，留余量 */
#define MODBUS_NUM_HREGS   26    /* 用到0..25，地址基数0，不移动旧地址 */

/* 寄存器区(地址基数0, 下标=Modbus地址号)。仅主上下文读写;
   ISR 只碰内部 RX 缓冲, 不碰这两个数组, 故无需临界区。 */
extern volatile uint8_t  modbus_coils[MODBUS_NUM_COILS];
extern volatile uint16_t modbus_hreg[MODBUS_NUM_HREGS];

/* 联调诊断计数器: frames=收到的帧数(任意内容, 看 RX 有没有字节到);
   valid=通过站号+CRC的合法帧数(看是不是给本站的正确请求)。 */
extern volatile uint32_t modbus_stat_frames;
extern volatile uint32_t modbus_stat_valid;

void modbus_init(void);         /* 初始化 USART3 + NVIC, 调一次 */
void modbus_poll(void);         /* 有整帧则解析并应答; 每通信周期调一次 */
void modbus_usart3_isr(void);   /* 供 USART3_IRQHandler() 转调 */
void modbus_loopback_selftest_tx(void);  /* 回环自测: 平时空操作, 定义 MB_LOOPBACK_TEST 时每调发一串测试字节 */

#ifdef __cplusplus
}
#endif

#endif
