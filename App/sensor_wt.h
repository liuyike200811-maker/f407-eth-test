/*
 * 维特智能 WT901 姿态传感器接入 —— USART6 (PC7=RX / PC6=TX), 115200 8N1
 * ================================================================
 * 数据链路: 传感器 --WiFi/UDP--> Windows 电脑 --USB转TTL--> 板子 USART6
 *   PC → 板子 PC7(RX): 传感器 54B 姿态帧(下述)
 *   板子 PC6(TX) → PC: 每控制周期回一个"本轮收没收到有效帧"的状态字节(链路诊断用)
 *
 * 帧格式(WiFi 这条路, Win 端原样透传, 共 54 字节, 无 checksum):
 *   [12B 设备ID ASCII, 以 "WT" 开头] + [40B 载荷] + [0x0D 0x0A]
 * 载荷内我们关心:
 *   偏移 4-7  : 帧计数器 (uint32 小端, 约 20ms 递增, 传感器回传速率50Hz), 用于去重/判重发
 *   偏移 8-13 : AccX/AccY/AccZ (各 int16 小端), ×16/32768 = g   —— 加速度源解倾角
 *   偏移 22-25: Roll, Pitch (各 int16 小端), ×180/32768 = 度     —— 角度字段源
 *   偏移 38-39: 固件版本号 (uint16 小端), 应 == 13032, 防字节错位
 *
 * ★ 安全重写要点(本模块负责"一手数据"的解析与校验, 详见 sensor_wt.c 顶注):
 *   1) 数据源【锁定】: 进入前 sensor_reset() 解锁, 头几帧观测后锁死"角度字段"或
 *      "加速度计"其一, 之后绝不逐帧翻转 —— 消除标定/跟随量纲不一致导致的大跳变。
 *   2) 帧内【范围+物理合理性】校验: |角度|≤90°; 用加速度源时要求 |a| 落在静态合理
 *      区间(≈1g 附近), 排除快速甩动的线性加速度被误读成大倾角。
 *   3) 帧间【尖峰过滤】: 此私有帧无 checksum, 故用时间一致性防 bit 翻转 —— 相邻帧
 *      跳变超阈值的"孤立离群帧"直接丢弃; 只有连续两帧一致确认才接受真实快速运动。
 *
 * 用法: 主循环每周期调 sensor_get(), 返回 1 表示取到一帧【通过全部校验】的新数据。
 * 收帧全程在中断里(RXNE 攒字节 + IDLE 判帧尾), 与 uart_log/Modbus 并行不阻塞。
 */
#ifndef SENSOR_WT_H
#define SENSOR_WT_H

#include <stdint.h>

#define SENSOR_FW_VERSION  13032    /* 固件版本号, 用于帧校验(见备忘) */

void sensor_init(void);          /* 初始化 USART6 + NVIC, 上电调一次 */
void sensor_usart6_isr(void);    /* 供 USART6_IRQHandler() 转调 */

/* 复位解析器: 清跳变历史 + 解锁数据源, 重新开始观测锁源。
 * 进入"传感器标定/跟随"前调一次, 保证标定与跟随用同一个锁定源、量纲一致。 */
void sensor_reset(void);

/* 取最新【通过全部校验】的姿态角(度, 原始未减零偏)。
 * 返回 1 = 取到一帧新的有效数据(*roll,*pitch 已更新);
 *      0 = 无新有效帧(数据源尚未锁定 / 未收到 / 未过校验; 出参不动)。 */
int  sensor_get(float *roll_deg, float *pitch_deg);

/* 往 USART6 TX(PC6) 发一个字节(阻塞等 TXE, 单字节 @115200 ≈ 87us)。 */
void sensor_tx_byte(uint8_t b);

extern volatile uint32_t sensor_stat_frames;  /* 收到的帧边界数(含无效, 调试用) */
extern volatile uint32_t sensor_stat_valid;   /* 通过【全部】校验的有效帧数(调试用) */
/* 最近一次未产出有效数据的原因:
 *   '0'无帧  'L'长度  'H'帧头  'T'帧尾  'V'版本  'D'重复帧(计数器未变)
 *   'U'源未锁定(观测中)  'R'角度越界  'A'加速度幅值异常(甩动/垃圾)  'J'跳变尖峰被拒 */
extern volatile char     sensor_fail_reason;

/* 锁定后的数据源: 1=加速度计解算, 0=角度字段(或尚未锁定) */
int sensor_using_accel(void);

#endif /* SENSOR_WT_H */
