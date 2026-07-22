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
 *   偏移 8-13 : AccX/AccY/AccZ (各 int16 小端), ×16/32768 = g      —— 融合输入(重力基准)
 *   偏移 14-19: GyrX/GyrY/GyrZ (各 int16 小端), ×2000/32768 = °/s  —— 融合输入(角速度)
 *   偏移 38-39: 固件版本号 (uint16 小端), 应 == 13032, 防字节错位
 *   (注: 偏移22-25 的"欧拉角字段"在6轴模式实测为死值 -1, 已弃用, 改由融合自算)
 *
 * ★ 姿态解算(本模块负责"一手数据"→可信姿态角, 详见 sensor_wt.c 顶注):
 *   改用 x-io Fusion 6轴 AHRS(加速度+陀螺, 无磁力计): 陀螺给平滑跟手、加速度定重力
 *   基准纠漂, 内置【加速度剔除】从源头挡住"平台运动线加速度污染倾角"(抖动根治点)。
 *   前置结构校验(长度/头/尾/版本/去重) + 输入合理性(陀螺越量程/加速度荒谬则丢帧) +
 *   复位后收敛预热若干帧不产出。sensor_reset() 复位融合四元数与预热计数。
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

/* 复位解析器: 复位融合四元数 + 预热计数 + 计数器去重历史。
 * 进入"传感器标定/跟随"前调一次, 让融合从头收敛、标定与跟随一致。 */
void sensor_reset(void);

/* 取最新【通过全部校验+6轴融合】的姿态角(度, 原始未减零偏)。
 * 返回 1 = 取到一帧新的有效数据(*roll,*pitch 已更新);
 *      0 = 无新有效帧(未收到 / 未过校验 / 融合预热中; 出参不动)。 */
int  sensor_get(float *roll_deg, float *pitch_deg);

/* 往 USART6 TX(PC6) 发一个字节(阻塞等 TXE, 单字节 @115200 ≈ 87us)。 */
void sensor_tx_byte(uint8_t b);

extern volatile uint32_t sensor_stat_frames;  /* 收到的帧边界数(含无效, 调试用) */
extern volatile uint32_t sensor_stat_valid;   /* 通过【全部】校验的有效帧数(调试用) */
/* 最近一次未产出有效数据的原因:
 *   '0'无帧  'L'长度  'H'帧头  'T'帧尾  'V'版本  'D'重复帧(计数器未变)
 *   'U'融合预热中  'R'融合角越界  'A'加速度幅值荒谬(丢帧)  'J'陀螺越量程(bit翻转) */
extern volatile char     sensor_fail_reason;

/* 保留接口(现全程6轴融合, 恒返回1): 供上层日志文案沿用, 无实际分支意义。 */
int sensor_using_accel(void);

/* 调试: 拷贝最近一个帧边界的原始字节到 out(最多 max), 返回实际字节数。
 * 无论有没有过校验都更新, 供降级空转/独立测试直接 hexdump 一手数据。 */
uint16_t sensor_debug_last_frame(uint8_t *out, uint16_t max);

#endif /* SENSOR_WT_H */
